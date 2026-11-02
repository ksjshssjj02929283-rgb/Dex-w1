"""迦智调度（CLOUDIA）导航：POST /api/v1/missions 下发 + 轮询任务状态。

与 http.py 的本体 8002/8852 是两套系统：这里连的是调度服务（默认 8858，无鉴权），
目标点是点位 id（int）或站点编码（str），不是世界坐标。
"""

from __future__ import annotations

import threading
import uuid

from ...common import TaskState
from .http import CloudiaApiError, requests

# Mission.state：0 排队（保持 ACCEPTED）/ 1 执行中 / 4 取消中 / 5 正常完成 / 6 取消完成 / 7 出错完成
_MISSION_RUNNING = 1
_MISSION_CANCELING = 4
_MISSION_FINISHED = 5
_MISSION_CANCELED = 6
_MISSION_FAILED = 7

# RobotCommand.cmd：1 单台车软急停、2 清除软急停
_ROBCMD_SOFT_STOP = 1
_ROBCMD_CLEAR_SOFT_STOP = 2

# 车端动作名，与迦智底盘 /opt/jz/jzagent/config.yml 一致：
# sched.auto_charge.charge_action / sched.stop_charge_action
CHARGE_ACTION = "charge"
STOP_CHARGE_ACTION = "stopCharge"

# 对外保持旧取值：IDLE / NAVIGATING / SUCCEEDED / FAILED / CANCELLED
_LEGACY = {
    TaskState.IDLE: "IDLE",
    TaskState.SENDING: "NAVIGATING",
    TaskState.ACCEPTED: "NAVIGATING",
    TaskState.EXECUTING: "NAVIGATING",
    TaskState.CANCELING: "NAVIGATING",
    TaskState.SUCCEEDED: "SUCCEEDED",
    TaskState.CANCELED: "CANCELLED",
    TaskState.ABORTED: "FAILED",
    TaskState.REJECTED: "FAILED",
    TaskState.FAILED_TO_SEND: "FAILED",
    TaskState.LOST: "FAILED",
}


class CloudiaSchedHttpClient:
    """调度 /api/v1 传输层；GET 允许有限重试，POST 不自动重试。"""

    def __init__(self, base_url: str):
        if requests is None:
            raise CloudiaApiError("迦智调度后端需要 requests，请先安装项目依赖。")
        self._base = (base_url or "").strip().rstrip("/")
        self._timeout_sec = 3.0
        self._query_retries = 2
        self._session = requests.Session()

    def _request(self, method: str, path: str, body=None):
        if not self._base:
            raise CloudiaApiError("CLOUDIA_SCHED_IP 未配置")
        attempts = 1 if method.upper() != "GET" else self._query_retries + 1
        last_error = None
        for _ in range(attempts):
            try:
                response = self._session.request(method,
                                                 f"{self._base}{path}",
                                                 json=body,
                                                 timeout=self._timeout_sec)
            except requests.RequestException as exc:
                last_error = exc
                continue
            try:
                payload = response.json()
            except ValueError:
                payload = None
            # 创建任务成功返回 201，其余 2xx 同样视为成功。
            if not 200 <= response.status_code < 300:
                raise CloudiaApiError(f"HTTP {response.status_code}",
                                      code=response.status_code,
                                      payload=payload)
            if not isinstance(payload, dict):
                raise CloudiaApiError(
                    f"响应不是 JSON 对象：HTTP {response.status_code}")
            return payload
        raise CloudiaApiError(f"迦智调度请求失败：{last_error}")

    def create_mission(self, steps: list, *, robot_id: int, ref_uuid: str):
        return self._request(
            "POST", "/api/v1/missions", {
                "ref_uuid": ref_uuid,
                "req_robot": robot_id,
                "priority": 0,
                "steps": steps,
            })

    def get_mission(self, mission_id: int):
        return self._request("GET", f"/api/v1/missions/{mission_id}")

    def get_robot(self, robot_id: int):
        return self._request("GET", f"/api/v1/robots/{robot_id}")

    def robot_cmd(self, robot_id: int, cmd: int):
        payload = self._request("POST", "/api/v1/robcmd", {
            "robot_id": robot_id,
            "cmd": cmd,
        })
        if payload.get("code") not in (None, 0):
            raise CloudiaApiError(payload.get("msg") or f"robcmd cmd={cmd} 失败",
                                  payload=payload)
        return payload

    def cancel_mission(self, mission_id: int):
        payload = self._request("POST", "/api/v1/mscmds", {
            "mission_id": mission_id,
            "cmd": 2,
            "sync": True,
        })
        if payload.get("result") == 2:
            raise CloudiaApiError(payload.get("err_msg") or "取消任务出错",
                                  payload=payload)
        return payload


class CloudiaSchedNavigationFeature:
    """调度单点导航，形状与 Nav2NavigationFeature 一致（send/state/cancel）。

    迦智的取消是“安全取消”，车可能走完当前这段才停。所以 cancel 先软急停让车立刻停，
    再取消任务；软急停由本对象打上的，下一次 send 前先清除，否则新任务会被调度以
    SKIP_EMEG_STOP 跳过、只排队不执行。别人（调度网页、cloudia_move）打的软急停不替他清。
    """

    _ACTIVE = frozenset({
        TaskState.SENDING, TaskState.ACCEPTED, TaskState.EXECUTING,
        TaskState.CANCELING
    })
    _TERMINAL = frozenset({
        TaskState.SUCCEEDED, TaskState.CANCELED, TaskState.ABORTED,
        TaskState.REJECTED, TaskState.FAILED_TO_SEND, TaskState.LOST
    })

    def __init__(self, node, api, cfg, *, label="迦智调度导航"):
        self._node = node
        self._api = api
        self._label = label
        if not isinstance(cfg.cloudia_sched_robot,
                          int) or cfg.cloudia_sched_robot <= 0:
            raise ValueError("迦智调度导航必须指定车号 CLOUDIA_SCHED_ROBOT（正整数），"
                             f"收到：{cfg.cloudia_sched_robot!r}")
        self._robot_id = cfg.cloudia_sched_robot
        self._map_name = cfg.cloudia_sched_map
        self._poll_interval = cfg.cloudia_poll_interval_s
        self._poll_fail_limit = cfg.cloudia_poll_fail_limit
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._thread = None
        self._generation = 0
        self._mission_id = None
        self._state = TaskState.IDLE
        self._result = (False, "")
        self._poll_failures = 0
        self._soft_stopped = False

    def _resolve_map_name(self):
        if self._map_name:
            return self._map_name
        name = self._api.get_robot(self._robot_id).get("map_name")
        if not name:
            raise CloudiaApiError(f"调度未返回机器人 {self._robot_id} 的地图名")
        return name

    @staticmethod
    def _build_step(target, map_name, action=""):
        # bool 是 int 的子类，显式排除。
        if isinstance(target, int) and not isinstance(target, bool):
            # target=0 是原地动作，必须带 action。
            if target < 0 or (target == 0 and not action):
                raise ValueError(f"点位 id 应为正整数，收到：{target}")
            return {"map_name": map_name, "target": target, "action": action}
        if isinstance(target, str) and target.strip():
            return {
                "map_name": map_name,
                "target_code": target.strip(),
                "action": action
            }
        raise ValueError(f"目标应为点位 id(int) 或站点编码(str)，收到：{target!r}")

    def send(self,
             target,
             *,
             action: str = "",
             trigger_ts: float = None) -> bool:
        del trigger_ts
        with self._lock:
            if self._state in self._ACTIVE:
                self._node.get_logger().warning(
                    f"[{self._label}] 已有活跃任务，拒绝并发下发。")
                return False
            self._generation += 1
            generation = self._generation
            self._state = TaskState.SENDING
            self._result = (False, "")
            self._poll_failures = 0
            self._mission_id = None

        try:
            step = self._build_step(target, self._resolve_map_name(), action)
            self._clear_soft_stop()
            mission = self._api.create_mission([step],
                                               robot_id=self._robot_id,
                                               ref_uuid=str(uuid.uuid4()))
            mission_id = mission.get("id")
            if mission_id is None:
                raise CloudiaApiError("任务创建响应缺少 id", payload=mission)
        except (CloudiaApiError, ValueError) as exc:
            with self._lock:
                if generation == self._generation:
                    self._state = (
                        TaskState.REJECTED if isinstance(exc, ValueError) or
                        exc.code is not None else TaskState.FAILED_TO_SEND)
                    self._result = (False, str(exc))
            self._node.get_logger().error(f"[{self._label}] 下发失败：{exc}")
            return False

        with self._lock:
            if generation != self._generation:
                return False
            self._mission_id = mission_id
            self._state = TaskState.ACCEPTED
            self._result = (False, "accepted")
            self._stop_event.clear()
            if self._thread is None or not self._thread.is_alive():
                self._thread = threading.Thread(target=self._poll_loop,
                                                name="cloudia-sched-poll",
                                                daemon=True)
                self._thread.start()
        self._node.get_logger().info(
            f"[{self._label}] 任务已创建 mission_id={mission_id}，目标={target!r}，"
            f"动作={action!r}。")
        return True

    def _clear_soft_stop(self):
        """清除本对象打的软急停；失败就抛出，不下发注定被跳过的任务。"""
        with self._lock:
            if not self._soft_stopped:
                return
        self._api.robot_cmd(self._robot_id, _ROBCMD_CLEAR_SOFT_STOP)
        with self._lock:
            self._soft_stopped = False
        self._node.get_logger().info(f"[{self._label}] 已清除软急停。")

    def _poll_loop(self):
        while not self._stop_event.is_set():
            self.refresh()
            self._stop_event.wait(self._poll_interval)

    def refresh(self):
        with self._lock:
            if self._mission_id is None or self._state in self._TERMINAL:
                return
            mission_id = self._mission_id
        try:
            mission = self._api.get_mission(mission_id)
        except CloudiaApiError as exc:
            with self._lock:
                self._poll_failures += 1
                if self._poll_failures >= self._poll_fail_limit:
                    self._state = TaskState.LOST
                    self._result = (False, str(exc))
                    self._stop_event.set()
            self._node.get_logger().warning(f"[{self._label}] 状态查询失败：{exc}，"
                                            f"连续失败={self._poll_failures}")
            return
        with self._lock:
            self._poll_failures = 0
            if mission_id != self._mission_id:
                return
            self._apply_mission_locked(mission)

    def _apply_mission_locked(self, mission):
        code = mission.get("state")
        if code == _MISSION_RUNNING:
            self._state = TaskState.EXECUTING
        elif code == _MISSION_CANCELING:
            self._state = TaskState.CANCELING
        elif code == _MISSION_FINISHED:
            self._state = TaskState.SUCCEEDED
            self._result = (True, "finished")
            self._stop_event.set()
        elif code == _MISSION_CANCELED:
            self._state = TaskState.CANCELED
            self._result = (False, "canceled")
            self._stop_event.set()
        elif code == _MISSION_FAILED:
            self._state = TaskState.ABORTED
            self._result = (False, mission.get("err_msg") or "迦智调度任务失败")
            self._stop_event.set()
        # _MISSION_QUEUED 及未知码：保持当前状态，下一轮再查。

    def state(self) -> str:
        with self._lock:
            return _LEGACY[self._state]

    def result(self):
        with self._lock:
            return self._result

    def has_cancelable_goal(self) -> bool:
        with self._lock:
            return self._state in self._ACTIVE

    def cancel(self) -> bool:
        with self._lock:
            if (self._state not in {TaskState.ACCEPTED, TaskState.EXECUTING} or
                    self._mission_id is None):
                return False
            mission_id = self._mission_id
        # 先软急停立刻停车；失败也继续取消任务，车最多走完当前这段。
        # 发之前就记上：请求超时时急停可能已生效，下次 send 照样要清（清未急停的车无害）。
        with self._lock:
            self._soft_stopped = True
        try:
            self._api.robot_cmd(self._robot_id, _ROBCMD_SOFT_STOP)
            self._node.get_logger().info(f"[{self._label}] 已软急停。")
        except CloudiaApiError as exc:
            self._node.get_logger().error(
                f"[{self._label}] 软急停失败：{exc}，只取消任务，车可能走完当前这段才停。")
        try:
            self._api.cancel_mission(mission_id)
        except CloudiaApiError as exc:
            # 取消请求没发成功不代表任务停了：保持原状态，由轮询继续跟踪。
            self._node.get_logger().error(f"[{self._label}] 取消失败：{exc}")
            return False
        with self._lock:
            if self._state not in self._TERMINAL:
                self._state = TaskState.CANCELING
        return True

    def close(self):
        self._stop_event.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)
