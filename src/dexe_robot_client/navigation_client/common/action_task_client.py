"""ActionTaskClient —— 通用 action 任务客户端（二次封装模板）。

一个实例管理一个 action 的完整生命周期，统一提供：
- TaskState 枚举状态机，所有转移在锁内完成；
- task_id 世代机制：旧任务的一切回调自动丢弃；
- accept watchdog：goal 发出后超时未收到 goal response → LOST（防 server
  掉线导致的静默丢失）；
- pending-cancel：goal 尚未被接受时取消 → 登记标志立即返回，接受回调里
  自动补发取消（无阻塞、无轮询、不丢取消）；
- 异常路径全部收敛为状态转移（FAILED_TO_SEND / LOST），不静默吞掉；
- feedback 缓存 + 节流心跳日志；
- **延迟诊断检查点链（受开关控制，默认关闭）**：send() 之前由调用方
  （chassis_client.py）打印链路起点"触发"，随后本类在同一 task_id 上依次
  记录 "server就绪" → "已发出" → "accepted"（**发送链路**）、"进度"（节流）
  → "result"（**反馈链路**）。`timing.diagnostics_enabled()` 打开时，每个
  中间检查点都以 `timing.format_checkpoint()` 生成的统一格式打印（阶段名 +
  挂钟时间 + 相对上一检查点增量 + 相对"触发"累计耗时）；关闭时（默认）只有
  "触发"和"result/返回"两行普通日志会打印（无 `[延迟诊断]` 标签，返回那行
  带总耗时），中间检查点仍静默记账（保证关闭状态下"返回"的总耗时依然准确），
  只是不打印。

使用方（features 层）只需构造 goal 与状态取值映射，各自约 50-80 行；
把上层传入的 `trigger_ts` 透传给 `send()` 即可接入检查点链。
"""

import threading
import time

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient

from .states import TaskState, ACTIVE_STATES
from . import timing


class ActionTaskClient:

    def __init__(self, node, action_type, action_name, *, label: str,
                 wait_server_timeout: float = 1.0,
                 accept_timeout: float = 2.0,
                 feedback_log_interval: float = 2.0,
                 feedback_formatter=None):
        self._node = node
        self._label = label
        self._client = (ActionClient(node, action_type, action_name)
                        if action_name else None)
        self._wait_server_timeout = wait_server_timeout
        self._accept_timeout = accept_timeout
        self._fb_interval = feedback_log_interval
        self._fb_formatter = feedback_formatter

        self._lock = threading.Lock()
        self._state = TaskState.IDLE
        self._task_id = 0
        self._goal_handle = None
        self._pending_cancel = False
        self._result_ok = False
        self._result_msg = ""
        self._last_feedback = None
        self._last_feedback_time = None
        self._last_fb_log = None
        self._watchdog = None

        # 延迟诊断检查点链（发送链路 + 反馈链路共用）
        self._trigger_ts = None    # 链路起点：上层调用客户端方法的挂钟时刻
        self._last_cp_ts = None    # 最近一个检查点的挂钟时刻
        self._last_cp_stage = None  # 最近一个检查点的阶段名

    # ------------------------------------------------ 查询接口
    @property
    def state(self) -> TaskState:
        with self._lock:
            return self._state

    def is_active(self) -> bool:
        with self._lock:
            return self._state in ACTIVE_STATES or self._goal_handle is not None

    def result(self):
        """返回 (success, message)。success 仅在 SUCCEEDED 且 result.success 时为 True。"""
        with self._lock:
            return self._result_ok, self._result_msg

    def last_feedback(self):
        with self._lock:
            return self._last_feedback, self._last_feedback_time

    # ------------------------------------------------ 延迟诊断检查点
    def _record_checkpoint(self, tid, stage, ts):
        """静默登记检查点（不打印），返回 (上一检查点时刻, 上一阶段名, 触发时刻)。

        任务已过期（tid 不是当前任务）时返回 (None, None, None)，调用方应
        跳过打印。feedback 检查点即使因节流不打印，也要调用本方法登记，
        这样 result 到达时能算出真实的"末次进度→result"增量。
        """
        with self._lock:
            if tid != self._task_id:
                return None, None, None
            prev_ts = self._last_cp_ts
            prev_stage = self._last_cp_stage
            trigger_ts = self._trigger_ts
            self._last_cp_ts = ts
            self._last_cp_stage = stage
        return prev_ts, prev_stage, trigger_ts

    def _checkpoint(self, tid, stage, ts=None):
        """登记一个中间检查点；仅在诊断开关打开时打印（统一格式）。

        关闭时仍会记账（更新 _last_cp_ts/_last_cp_stage），只是不打印——
        保证"result/返回"检查点在关闭状态下依然能算出正确的总耗时。
        """
        ts = ts if ts is not None else timing.now()
        prev_ts, prev_stage, trigger_ts = self._record_checkpoint(tid, stage, ts)
        if prev_ts is None and trigger_ts is None:
            return  # 任务已过期，不打印
        if not timing.diagnostics_enabled():
            return
        self._node.get_logger().info(
            timing.format_checkpoint(
                self._label, stage, ts, tid=tid,
                prev_ts=prev_ts, prev_stage=prev_stage, trigger_ts=trigger_ts))

    # ------------------------------------------------ 发送
    def send(self, goal, *, wait_server_timeout: float = None,
             trigger_ts: float = None) -> bool:
        """发送 goal（非阻塞）。

        Args:
            trigger_ts: 上层调用入口（chassis_client.py `_log_trigger`）记录
                的挂钟时刻，作为本任务延迟诊断检查点链的"触发"基准；不传则
                以进入本方法的时刻为基准。
        """
        if self._client is None:
            self._node.get_logger().warn(
                f"[{self._label}] 当前模式未配置该 action，忽略发送。")
            return False

        entry_ts = timing.now()
        with self._lock:
            self._task_id += 1
            tid = self._task_id
            self._state = TaskState.WAITING_SERVER
            self._goal_handle = None
            self._pending_cancel = False
            self._result_ok = False
            self._result_msg = ""
            self._last_feedback = None
            self._last_feedback_time = None
            self._last_fb_log = None
            self._cancel_watchdog_locked()
            self._trigger_ts = trigger_ts if trigger_ts is not None else entry_ts
            self._last_cp_ts = self._trigger_ts
            self._last_cp_stage = "触发"

        timeout = (wait_server_timeout if wait_server_timeout is not None
                   else self._wait_server_timeout)
        if not self._client.wait_for_server(timeout_sec=timeout):
            self._checkpoint(tid, "server不可用")
            self._transition(tid, TaskState.FAILED_TO_SEND, "action server 不可用")
            return False
        self._checkpoint(tid, "server就绪")

        self._transition(tid, TaskState.SENDING)
        send_ts = timing.now()
        try:
            send_future = self._client.send_goal_async(
                goal,
                feedback_callback=lambda m, _tid=tid: self._on_feedback(m, _tid))
        except Exception as e:  # noqa: BLE001
            self._checkpoint(tid, "发送异常")
            self._transition(tid, TaskState.FAILED_TO_SEND, f"发送异常：{e}")
            return False
        self._node.get_logger().info(
            f"[{self._label}] 目标已发送 task_id={tid}，等待 server 接受。")
        self._checkpoint(tid, "已发出", send_ts)
        send_future.add_done_callback(
            lambda f, _tid=tid: self._on_goal_response(f, _tid))

        watchdog = threading.Timer(
            self._accept_timeout, self._on_accept_timeout, args=(tid,))
        watchdog.daemon = True
        with self._lock:
            self._watchdog = watchdog
        watchdog.start()
        return True

    # ------------------------------------------------ 取消
    def cancel(self) -> bool:
        with self._lock:
            tid = self._task_id
            goal_handle = self._goal_handle
            state = self._state
            if goal_handle is None and state in (TaskState.WAITING_SERVER,
                                                 TaskState.SENDING):
                self._pending_cancel = True
                pending = True
            else:
                pending = False
        if pending:
            self._node.get_logger().info(
                f"[{self._label}] 目标尚未被接受，已登记挂起取消"
                "（接受后自动补发，无需等待）。")
            return True
        if goal_handle is None:
            self._node.get_logger().info(
                f"[{self._label}] 无可取消的目标（state={state.name}）。")
            return False
        return self._do_cancel(goal_handle, tid)

    def _do_cancel(self, goal_handle, tid) -> bool:
        try:
            cancel_future = goal_handle.cancel_goal_async()
        except Exception as e:  # noqa: BLE001
            self._node.get_logger().error(
                f"[{self._label}] 取消请求发送异常 task_id={tid}，error={e}")
            return False
        self._transition(tid, TaskState.CANCELING)
        cancel_future.add_done_callback(
            lambda f, _tid=tid: self._on_cancel_response(f, _tid))
        return True

    # ------------------------------------------------ 内部：状态转移与回调
    def _transition(self, tid, new_state, detail: str = ""):
        with self._lock:
            if tid != self._task_id:
                return
            old = self._state
            self._state = new_state
        if old == new_state:
            return
        text = (f"[{self._label}] 状态 {old.name} -> {new_state.name}"
                f" task_id={tid}" + (f"（{detail}）" if detail else ""))
        if new_state in (TaskState.REJECTED, TaskState.ABORTED,
                         TaskState.FAILED_TO_SEND, TaskState.LOST):
            self._node.get_logger().error(text)
        else:
            self._node.get_logger().info(text)

    def _cancel_watchdog_locked(self):
        if self._watchdog is not None:
            self._watchdog.cancel()
            self._watchdog = None

    def _on_accept_timeout(self, tid):
        with self._lock:
            if tid != self._task_id or self._state != TaskState.SENDING:
                return
        self._checkpoint(tid, "accept超时")
        self._transition(
            tid, TaskState.LOST,
            f"accept watchdog：{self._accept_timeout:.1f}s 未收到 goal response，"
            "server 可能已掉线")

    def _on_goal_response(self, future, tid):
        with self._lock:
            if tid != self._task_id:
                return
            self._cancel_watchdog_locked()
        try:
            goal_handle = future.result()
        except Exception as e:  # noqa: BLE001
            self._checkpoint(tid, "response异常")
            self._transition(tid, TaskState.LOST, f"goal response 异常：{e}")
            return
        if not goal_handle.accepted:
            self._checkpoint(tid, "rejected")
            self._transition(tid, TaskState.REJECTED)
            return

        accept_ts = timing.now()
        with self._lock:
            if tid != self._task_id:
                return
            self._goal_handle = goal_handle
            do_cancel = self._pending_cancel
            self._pending_cancel = False
        self._checkpoint(tid, "accepted", accept_ts)
        self._transition(tid, TaskState.ACCEPTED)

        try:
            result_future = goal_handle.get_result_async()
        except Exception as e:  # noqa: BLE001
            self._checkpoint(tid, "注册result回调异常")
            self._transition(tid, TaskState.LOST, f"注册 result 回调异常：{e}")
            return
        result_future.add_done_callback(
            lambda f, _tid=tid: self._on_result(f, _tid))

        if do_cancel:
            self._node.get_logger().info(
                f"[{self._label}] 补发挂起的取消请求 task_id={tid}。")
            self._do_cancel(goal_handle, tid)

    def _on_feedback(self, feedback_msg, tid):
        fb_ts = timing.now()
        with self._lock:
            if tid != self._task_id:
                return
            self._last_feedback = feedback_msg.feedback
            self._last_feedback_time = time.monotonic()
            if self._state == TaskState.ACCEPTED:
                self._state = TaskState.EXECUTING
            now_mono = time.monotonic()
            do_log = (self._last_fb_log is None or
                     now_mono - self._last_fb_log >= self._fb_interval)
            if do_log:
                self._last_fb_log = now_mono

        # 无论本次是否打印，都登记检查点——保证 result 到达时"末次进度→result"
        # 反映的是真实最后一次 feedback，而不是最后一次被打印出来的 feedback。
        prev_ts, prev_stage, trigger_ts = self._record_checkpoint(tid, "进度", fb_ts)
        if not do_log or prev_ts is None:
            return

        detail = ""
        if self._fb_formatter is not None:
            try:
                detail = self._fb_formatter(feedback_msg.feedback)
            except Exception:  # noqa: BLE001 —— 格式化失败不影响主流程
                detail = ""

        if timing.diagnostics_enabled():
            line = timing.format_checkpoint(
                self._label, "进度", fb_ts, tid=tid,
                prev_ts=prev_ts, prev_stage=prev_stage, trigger_ts=trigger_ts)
            if detail:
                line += f"  内容={detail}"
            self._node.get_logger().info(line)
        elif detail:
            # 正常模式：只输出内容型进度日志，不含诊断标签/数值
            self._node.get_logger().info(f"[{self._label}] 进度 task_id={tid}，{detail}")

    def _on_result(self, future, tid):
        result_ts = timing.now()
        try:
            response = future.result()
        except Exception as e:  # noqa: BLE001
            with self._lock:
                if tid == self._task_id:
                    self._goal_handle = None
            self._checkpoint(tid, "result异常")
            self._transition(tid, TaskState.LOST, f"result 获取异常：{e}")
            return

        result = response.result
        success_flag = bool(getattr(result, "success", True))
        message = str(getattr(result, "message", ""))
        with self._lock:
            if tid != self._task_id:
                return
            self._goal_handle = None
            self._result_ok = (response.status == GoalStatus.STATUS_SUCCEEDED
                               and success_flag)
            self._result_msg = message

        # "返回"检查点始终打印（不受诊断开关影响），格式随开关切换：
        # 打开=完整检查点行（含 Δ/累计，[延迟诊断] 标签）；
        # 关闭（默认）=普通日志，只给挂钟时间 + 总耗时。
        prev_ts, prev_stage, trigger_ts = self._record_checkpoint(tid, "result", result_ts)
        if prev_ts is not None or trigger_ts is not None:
            if timing.diagnostics_enabled():
                self._node.get_logger().info(
                    timing.format_checkpoint(
                        self._label, "result", result_ts, tid=tid,
                        prev_ts=prev_ts, prev_stage=prev_stage,
                        trigger_ts=trigger_ts))
            else:
                self._node.get_logger().info(
                    timing.format_plain_return(
                        self._label, result_ts, trigger_ts, tid=tid))

        if response.status == GoalStatus.STATUS_SUCCEEDED:
            self._transition(tid, TaskState.SUCCEEDED, message)
        elif response.status == GoalStatus.STATUS_CANCELED:
            self._transition(tid, TaskState.CANCELED, message)
        else:
            self._transition(tid, TaskState.ABORTED,
                             f"result status={response.status} {message}")

    def _on_cancel_response(self, future, tid):
        try:
            response = future.result()
            self._node.get_logger().info(
                f"[{self._label}] 取消响应 task_id={tid}，"
                f"goals_canceling={len(response.goals_canceling)}。")
        except Exception as e:  # noqa: BLE001
            self._node.get_logger().error(
                f"[{self._label}] 取消响应异常 task_id={tid}，error={e}")
