import time
import threading
import numpy as np
import math
from abc import ABC, abstractmethod
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from scipy.spatial.transform import Rotation as R
from nav2_msgs.action import NavigateToPose
from chassis_interfaces.action import AutoCharging
try:
    from nav_interfaces.action import PreciseControl
    PRECISE_CONTROL_IMPORT_ERROR = None
except ImportError as e:
    # 允许使用旧版 nav_interfaces 的环境先启动其他功能；直控接口会保持不可用。
    PreciseControl = None
    PRECISE_CONTROL_IMPORT_ERROR = str(e)
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import ParameterType
from rclpy.parameter import Parameter
from geometry_msgs.msg import Polygon, Point32
from std_msgs.msg import Bool

from ..common import get_logger


class NavigationClient(ABC, Node):
    MOTION_PROGRESS_LOG_INTERVAL = 2.0

    NAV_CANCELABLE_STATES = {"NAVIGATING"}
    AUTO_CHARGING_CANCELABLE_STATES = {
        "WAITING_SERVER", "SENDING", "ACCEPTED", "EXECUTING"
    }
    CONTROL_CANCELABLE_STATES = {
        "WAITING_SERVER", "SENDING", "CONTROL", "CANCELLING"
    }

    AUTO_CHARGING_STATUS_NAMES = {
        GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
        GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
        GoalStatus.STATUS_EXECUTING: "EXECUTING",
        GoalStatus.STATUS_CANCELING: "CANCELING",
        GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
        GoalStatus.STATUS_CANCELED: "CANCELED",
        GoalStatus.STATUS_ABORTED: "ABORTED",
    }

    ACTION_RESULT_STATUS_NAMES = {
        GoalStatus.STATUS_UNKNOWN: "UNKNOWN（结果状态未知）",
        GoalStatus.STATUS_ACCEPTED: "ACCEPTED（已接受）",
        GoalStatus.STATUS_EXECUTING: "EXECUTING（执行中）",
        GoalStatus.STATUS_CANCELING: "CANCELING（取消中）",
        GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED（成功）",
        GoalStatus.STATUS_CANCELED: "CANCELED（已取消）",
        GoalStatus.STATUS_ABORTED: "ABORTED（已中止）",
    }

    def __init__(self,
                 name: str,
                 is_sim: bool = False,
                 timeout_sec: float = 1.0):
        self.logger = get_logger()
        if is_sim:
            self.node_name = name + "_sim"
        else:
            self.node_name = name + "_real"
        super().__init__(self.node_name)
        self.get_logger().info(f"[导航客户端] 已创建：{self.node_name}")
        self._last_msg_time = None
        self._last_chassis_processed_feedback = None
        self._cur_pos = None
        self._connection_timeout = 1.0
        self._chassis_feedback_disconnected = False
        self._set_nav_publisher = None
        self._set_nav_client = None
        self._get_pos_subscriber = None
        self._auto_charging_action_client = None
        self._auto_charging_goal_handle = None
        self._auto_charging_send_goal_future = None
        self._auto_charging_result_future = None
        self._auto_charging_cancel_future = None
        self._auto_charging_task_id = 0
        self._auto_charging_status = "IDLE"
        self._auto_charging_done = False
        self._auto_charging_success = False
        self._last_auto_charging_feedback = None
        self._last_auto_charging_result_message = ""
        self._auto_charging_started_time = None
        self._auto_charging_accepted_time = None
        self._auto_charging_cancel_requested_time = None
        self._control_action_client = None
        self._control_goal_handle = None
        self._control_send_goal_future = None
        self._control_result_future = None
        self._control_cancel_future = None
        self._control_task_id = 0
        self._control_status = "IDLE"
        self._control_done = True
        self._control_success = False
        self._control_cancel_requested = False
        self._last_control_feedback = None
        self._last_control_result_message = ""
        self._control_accepted_time = None
        self._control_cancel_requested_time = None
        self._controller_set_param_client = None
        self._local_costmap_set_param_client = None
        self._global_costmap_set_param_client = None

        self._controller_get_param_client = None
        self._local_costmap_get_param_client = None
        self._global_costmap_get_param_client = None

        self._battery_percentage = -1.0
        self._obstacle_subscriber = None
        self._has_obstacle = False

        self._nav_goal_handle = None
        self._nav_result_future = None
        self._nav_cancel_future = None
        self._nav_status = "IDLE"
        self._nav_result = None
        self._nav_task_id = 0
        self._nav_target_position = None
        self._nav_started_time = None
        self._nav_accepted_time = None
        self._nav_cancel_requested_time = None
        self._nav_last_progress_log_time = None
        self._nav_last_feedback_summary = None
        # 业务线程和 ROS 回调线程可能同时访问 Action 状态。发送入口串行化后，
        # 每次 task_id 与对应目标状态可以作为一个整体完成初始化。
        self._nav_request_lock = threading.Lock()
        self._auto_charging_request_lock = threading.Lock()
        self._control_request_lock = threading.Lock()
        self._control_target_position = None
        self._control_started_time = None
        self._control_last_progress_log_time = None
        self._last_auto_charging_feedback_signature = None
        self._last_auto_charging_feedback_log_time = None
        # 底盘出站操作审计：用于区分“后台状态/位姿回调日志”和
        # “本客户端确实发起了底盘运动或取消请求”。
        self._chassis_command_audit_lock = threading.Lock()
        self._chassis_command_sequence = 0
        self._last_chassis_command_type = None
        self._last_chassis_command_channel = None
        self._last_chassis_command_detail = None
        self._last_chassis_command_time = None

        self._init_subscriber(is_sim)
        self._init_publisher(is_sim)
        self.spin_once(timeout_sec=timeout_sec)

        self.create_timer(0.1, self._check_connection_status)

    def _check_connection_status(self):
        """定时检查消息是否超时

        注意：这里必须用 time.time()（系统时钟）而不是 self.get_clock().now()，
        因为 _chassis_feedback_callback 里 _last_msg_time 也是用 time.time() 记录的。
        若混用系统时钟与 ROS 时钟（仿真时 use_sim_time=True 两者不一致），
        会导致差值无意义，超时检测失效。
        """
        cur_time = time.time()
        if self._last_msg_time is not None:
            if (cur_time - self._last_msg_time) > self._connection_timeout:
                self._cur_pos = None
                self._last_msg_time = None
                self._chassis_feedback_disconnected = True
                self.get_logger().warn(
                    "[底盘位姿反馈中断] 超过 "
                    f"{self._connection_timeout:.1f}s 未收到底盘位姿，"
                    "上层将拒绝新的底盘运动指令。")

    def is_connected(self, timeout_sec: float = 0.1):
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            self.spin_once()
            if self._last_msg_time is not None:
                return True
            time.sleep(0.01)
        return False

    @abstractmethod
    def _init_subscriber(self, is_sim: bool):
        pass

    @abstractmethod
    def _init_publisher(self, is_sim: bool):
        pass

    def _parse_msg_header(self, msg) -> dict:
        header = None

        if hasattr(msg, "header"):
            header = {
                "stamp": {
                    "sec":
                        msg.header.stamp.sec
                        if hasattr(msg.header.stamp, "sec") else 0,
                    "nanosec":
                        msg.header.stamp.nanosec if hasattr(
                            msg.header.stamp, "nanosec") else 0,
                },
                "frame_id":
                    msg.header.frame_id
                    if hasattr(msg.header, "frame_id") else "",
            }
        else:
            header = {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""}

        return header

    def _chassis_feedback_callback(self, msg):
        self.get_logger().info(
            "[底盘位姿反馈已连接] 已收到首条底盘位姿消息。",
            once=True)
        if self._chassis_feedback_disconnected:
            self.get_logger().info(
                "[底盘位姿反馈恢复] 已重新收到底盘位姿消息。")
            self._chassis_feedback_disconnected = False
        processed = {"header": {}, "data": {}}
        processed["header"] = self._parse_msg_header(msg)
        processed["data"] = {
            "position": [
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
            ],
            "orientation": [
                msg.pose.orientation.x, msg.pose.orientation.y,
                msg.pose.orientation.z, msg.pose.orientation.w
            ]
        }

        self._last_chassis_processed_feedback = processed
        self._last_msg_time = time.time()
        # self.get_logger().info(f"feedback received, _last_msg_time={self._last_msg_time}", once=False)

    def _battery_feedback_callback(self, msg):
        '''解析电池状态消息'''
        for status in msg.status:
            if status.name == "/Devices/System/PowerBoard/collector_node: PowerBoard":
                for key_value in status.values:
                    if key_value.key == "battery_percentage":
                        self._battery_percentage = float(key_value.value)
                        break

    def _obstacle_callback(self, msg):
        """障碍物检测回调函数"""
        self._has_obstacle = msg.data

    def get_battery_percentage(self) -> tuple[bool, float]:
        """获取当前电池电量百分比。

        Returns:
            float: 电池电量百分比，范围为 0.0 到 100.0。如果未收到电池状态消息，则返回 -1.0。
        """
        if (self._battery_percentage < 0):
            return False, -1.0
        else:
            return True, self._battery_percentage

    def get_front_obstacle(self) -> bool:
        """获取前方障碍物检测状态。

        Returns:
            bool: 如果检测到障碍物则返回 True，否则返回 False。
        """
        self.spin_once()
        return self._has_obstacle

    def get_chassis_position(self) -> tuple[bool, np.ndarray]:
        """
        获取底盘当前的位置和姿态 (x, y, yaw)。
        Returns:
            一个元组 (success, position)。
            - success (bool): 如果成功获取位置则为 True。
            - position (np.ndarray): 包含 [x, y, yaw] 的NumPy数组，yaw单位为度
        """

        self.spin_once()

        if self._last_chassis_processed_feedback is None:
            self.get_logger().error(
                "[底盘位姿不可用] 尚未收到底盘位姿反馈。")
            return False, None

        pos = self._last_chassis_processed_feedback["data"]["position"]
        x, y = pos[0], pos[1]

        q = self._last_chassis_processed_feedback["data"]["orientation"]
        quat_array = np.array([q[0], q[1], q[2], q[3]])
        rotation = R.from_quat(quat_array)

        yaw = rotation.as_euler('xyz', degrees=True)[2]
        self._cur_pos = np.array([x, y, yaw])
        return True, self._cur_pos

    def _get_cached_chassis_pose_summary(self) -> str:
        """不触发额外 ROS spin，直接生成简短的底盘位姿文本。"""
        if self._last_chassis_processed_feedback is None:
            return "暂无位姿反馈"

        pos = self._last_chassis_processed_feedback["data"]["position"]
        q = self._last_chassis_processed_feedback["data"]["orientation"]
        try:
            yaw = R.from_quat(q).as_euler('xyz', degrees=True)[2]
            return f"({pos[0]:.3f}, {pos[1]:.3f}, {yaw:.1f}°)"
        except ValueError:
            return f"({pos[0]:.3f}, {pos[1]:.3f}, yaw无效)"

    @staticmethod
    def _elapsed_since(start_time) -> float:
        """计算单调时钟耗时；阶段尚未开始时返回 -1。"""
        if start_time is None:
            return -1.0
        return max(0.0, time.monotonic() - start_time)

    @classmethod
    def _action_result_status_name(cls, status: int) -> str:
        """把 ROS 2 Action 结果码转换为便于排查的状态文本。"""
        return cls.ACTION_RESULT_STATUS_NAMES.get(
            status, f"UNKNOWN_CODE（未知结果码 {status}）")

    def _nav_target_summary(self) -> str:
        """返回当前 Nav2 目标摘要，不读取新的 ROS 消息。"""
        target = self._nav_target_position
        if target is None:
            return "目标未记录"
        return f"({target[0]:.3f}, {target[1]:.3f}, {target[2]:.1f}°)"

    def _record_chassis_command(self, command_type: str, channel: str,
                                detail: str) -> int:
        """记录并打印一次真正发起的底盘出站操作。"""
        now = time.monotonic()
        with self._chassis_command_audit_lock:
            self._chassis_command_sequence += 1
            sequence = self._chassis_command_sequence
            self._last_chassis_command_type = command_type
            self._last_chassis_command_channel = channel
            self._last_chassis_command_detail = detail
            self._last_chassis_command_time = now

        self.get_logger().debug(
            f"[底盘指令] #{sequence} {command_type}，{detail}")
        return sequence

    def _get_chassis_command_audit_summary(self) -> str:
        """返回最近一次底盘出站操作摘要，不发送任何 ROS 请求。"""
        now = time.monotonic()
        with self._chassis_command_audit_lock:
            sequence = self._chassis_command_sequence
            command_type = self._last_chassis_command_type
            channel = self._last_chassis_command_channel
            detail = self._last_chassis_command_detail
            command_time = self._last_chassis_command_time

        if sequence == 0 or command_time is None:
            return "底盘指令记录：本程序启动后尚未发送任何底盘运动或取消请求"

        age = max(0.0, now - command_time)
        return (
            f"底盘指令记录：本程序启动后共发送 {sequence} 条；最近一条是"
            f"“{command_type}”，发送方式={channel}，具体内容={detail}，"
            f"发生在 {age:.3f} 秒前")

    def _normalize_motion_target(self, position, label: str):
        """校验并复制三维运动目标，避免非法输入破坏当前任务状态。"""
        try:
            target = np.asarray(position, dtype=float).reshape(-1)
        except (TypeError, ValueError) as e:
            self.get_logger().error(
                f"[{label}目标非法] 无法转换为数值数组，目标={position}，error={e}")
            return None

        if target.size != 3:
            self.get_logger().error(
                f"[{label}目标非法] 目标必须包含 [x, y, yaw] 三个数值，"
                f"当前元素数量={target.size}，目标={position}。")
            return None
        if not np.all(np.isfinite(target)):
            self.get_logger().error(
                f"[{label}目标非法] 目标包含 NaN 或无穷大，目标={target.tolist()}。")
            return None
        return target.copy()

    def set_chassis_nav2_position(self, position) -> bool:
        """串行发送 Nav2 目标；同一客户端中后调用的目标具有更高优先级。"""
        with self._nav_request_lock:
            return self._set_chassis_nav2_position(position)

    def _set_chassis_nav2_position(self, position) -> bool:
        """异步发送 Nav2 导航目标。

        Args:
            position (list/np.ndarray): 目标位置 [x, y, yaw],yaw单位为度。

        Returns:
            bool: True 仅表示 goal 请求已发出；最终结果由 Action 回调更新。
        """
        if self._set_nav_client is None:
            self.get_logger().error(
                "[导航目标发送失败] NavigateToPose Action 客户端未初始化。")
            return False

        target = self._normalize_motion_target(position, "导航")
        if target is None:
            return False

        self._nav_task_id += 1
        current_task_id = self._nav_task_id
        self._nav_goal_handle = None
        self._nav_result_future = None
        self._nav_cancel_future = None
        self._nav_result = None
        self._nav_target_position = target
        self._nav_started_time = time.monotonic()
        self._nav_accepted_time = None
        self._nav_cancel_requested_time = None
        self._nav_last_progress_log_time = None
        self._nav_last_feedback_summary = None
        self._set_nav_status(
            "NAVIGATING", current_task_id,
            "任务已创建，正在等待 /navigate_to_pose Action 服务")

        nav2_position = NavigateToPose.Goal()
        nav2_position.pose = PoseStamped()
        nav2_position.pose.header.frame_id = "map"
        nav2_position.pose.header.stamp = self.get_clock().now().to_msg()
        nav2_position.pose.pose.position.x = target[0]
        nav2_position.pose.pose.position.y = target[1]
        nav2_position.pose.pose.position.z = 0.0

        rotation = R.from_euler('xyz', [0, 0, target[2]], degrees=True)
        quat_arry = rotation.as_quat()

        nav2_position.pose.pose.orientation.x = quat_arry[0]
        nav2_position.pose.pose.orientation.y = quat_arry[1]
        nav2_position.pose.pose.orientation.z = quat_arry[2]
        nav2_position.pose.pose.orientation.w = quat_arry[3]

        wait_started_time = time.monotonic()
        if not self._set_nav_client.wait_for_server(timeout_sec=1.0):
            self._set_nav_status(
                "FAILED", current_task_id,
                "等待 /navigate_to_pose Action 服务超过 1.0s")
            self.get_logger().error(
                f"[导航目标发送失败] task_id={current_task_id}，"
                "NavigateToPose Action 服务不可用，"
                f"等待服务耗时={self._elapsed_since(wait_started_time):.3f}s，"
                f"目标={self._nav_target_summary()}。"
                "请检查 bt_navigator 是否已激活、/navigate_to_pose 是否存在。")
            return False
        wait_server_elapsed = self._elapsed_since(wait_started_time)

        try:
            send_future = self._set_nav_client.send_goal_async(
                nav2_position,
                feedback_callback=lambda feedback_msg: self._on_nav_feedback(
                    feedback_msg, current_task_id))
            send_future.add_done_callback(
                lambda future: self._on_nav_goal_response(
                    future, current_task_id))
            self._record_chassis_command(
                "导航到目标点", "导航 Action（/navigate_to_pose）",
                f"任务编号={current_task_id}，目标={self._nav_target_summary()}")
            self.get_logger().info(
                f"[导航] 已发送 task_id={current_task_id}，"
                f"目标={self._nav_target_summary()}，"
                f"服务等待={wait_server_elapsed:.2f}s。")
            return True
        except Exception as e:
            self._set_nav_status(
                "FAILED", current_task_id, "发送 Action goal 时发生异常")
            self.get_logger().error(
                f"[导航目标发送异常] task_id={current_task_id}，"
                f"目标={self._nav_target_summary()}，"
                f"总耗时={self._elapsed_since(self._nav_started_time):.3f}s，"
                f"error={e}")
            return False

    def _set_nav_status(self, status: str, task_id: int,
                        reason: str = "") -> None:
        """统一更新导航状态并记录状态变化与任务耗时。"""
        if task_id != self._nav_task_id:
            return

        old_status = self._nav_status
        self._nav_status = status
        if old_status == status:
            return

        reason_text = f"，原因={reason}" if reason else ""
        self.get_logger().info(
            f"[导航状态] id={task_id} {old_status}->{status}，"
            f"耗时={self._elapsed_since(self._nav_started_time):.1f}s"
            f"{reason_text}。")

    def _on_nav_feedback(self, feedback_msg, task_id):
        """根据 Nav2 feedback 低频记录导航进度。"""
        if task_id != self._nav_task_id:
            self.get_logger().debug(
                f"[导航旧反馈已忽略] feedback_task_id={task_id}，"
                f"current_task_id={self._nav_task_id}。")
            return

        now = time.monotonic()
        if self._nav_last_progress_log_time is not None and \
                now - self._nav_last_progress_log_time < \
                self.MOTION_PROGRESS_LOG_INTERVAL:
            return
        self._nav_last_progress_log_time = now

        feedback = feedback_msg.feedback
        pose = feedback.current_pose.pose.position
        navigation_time = (feedback.navigation_time.sec +
                           feedback.navigation_time.nanosec * 1e-9)
        self._nav_last_feedback_summary = {
            "current_x": pose.x,
            "current_y": pose.y,
            "distance_remaining": feedback.distance_remaining,
            "navigation_time": navigation_time,
            "recoveries": feedback.number_of_recoveries,
        }
        self.get_logger().info(
            f"[导航进度] id={task_id}，剩余={feedback.distance_remaining:.2f}m，"
            f"已运行={navigation_time:.1f}s，恢复={feedback.number_of_recoveries}。")

    def get_nav_state(self) -> str:
        """获取当前导航状态。

        Returns:
            str: 导航状态字符串，可能为：
                - "IDLE": 空闲
                - "NAVIGATING": 导航中
                - "CANCELLING": 正在取消，等待 Nav2 最终结果
                - "SUCCEEDED": 成功
                - "ABORTED": Nav2 服务端中止
                - "FAILED": 失败
                - "CANCELLED": 已取消
        """
        # 独立运行时主动处理一次回调；由 W1Manager Executor 管理时为空操作。
        self.spin_once()

        return self._nav_status

    def has_cancelable_nav_goal(self) -> bool:
        """返回当前是否存在可取消的 Nav2 goal。"""
        return (self._nav_status in self.NAV_CANCELABLE_STATES
                or self._nav_goal_handle is not None)

    def cancel_chassis_nav2(self) -> bool:
        """异步请求取消导航；最终停下以 Action result 为准。"""
        if self._nav_cancel_future is not None:
            self.get_logger().info(
                f"[导航取消跳过] task_id={self._nav_task_id}，"
                "取消请求已经发出，正在等待 Nav2 最终结果。")
            return True

        # goal 请求已发出但尚未收到接受回调时，短暂等待 goal_handle。
        if self._nav_goal_handle is None and self._nav_status == "NAVIGATING":
            self.get_logger().info(
                f"[导航取消等待] task_id={self._nav_task_id}，"
                "目标尚未被 Nav2 接受，最多等待 1.0s。")
            start_time = time.time()
            while time.time() - start_time < 1.0:
                self.spin_once(timeout_sec=0.05)
                if self._nav_goal_handle is not None:
                    break
                time.sleep(0.05)

        if self._nav_goal_handle is None:
            self.get_logger().info(
                f"[导航取消跳过] task_id={self._nav_task_id}，"
                f"当前状态={self._nav_status}，没有已接受的导航目标。")
            return False

        current_task_id = self._nav_task_id

        try:
            self._nav_cancel_requested_time = time.monotonic()
            self._set_nav_status(
                "CANCELLING", current_task_id,
                "上层已调用 cancel_chassis_nav2()")
            self._nav_cancel_future = (
                self._nav_goal_handle.cancel_goal_async())
            self._nav_cancel_future.add_done_callback(
                lambda future: self._on_nav_cancel_response(
                    future, current_task_id))
            self._record_chassis_command(
                "取消导航", "导航 Action 取消接口（/navigate_to_pose）",
                f"任务编号={current_task_id}")
            self.get_logger().info(
                f"[导航取消请求已发送] task_id={current_task_id}，"
                f"目标={self._nav_target_summary()}，"
                f"请求取消时任务已运行="
                f"{self._elapsed_since(self._nav_started_time):.3f}s。"
                "这里只表示取消请求已发送，最终以 Action result 为准。")
            return True
        except Exception as e:
            self._nav_cancel_future = None
            self.get_logger().error(
                f"[导航取消请求发送异常] task_id={current_task_id}，"
                f"状态={self._nav_status}，目标={self._nav_target_summary()}，"
                f"error={e}")
            return False

    def _set_auto_charging_status(self, status: str, task_id: int,
                                  reason: str = "") -> None:
        """统一更新回充状态并记录状态变化与任务耗时。"""
        if task_id != self._auto_charging_task_id:
            return

        old_status = self._auto_charging_status
        self._auto_charging_status = status
        if old_status == status:
            return

        reason_text = f"，原因={reason}" if reason else ""
        self.get_logger().info(
            f"[自动回充状态] id={task_id} {old_status}->{status}，"
            f"耗时={self._elapsed_since(self._auto_charging_started_time):.1f}s"
            f"{reason_text}。")

    def _auto_charging_feedback_callback(self, feedback_msg, task_id):
        if task_id != self._auto_charging_task_id:
            self.get_logger().debug(
                f"[自动回充旧反馈已忽略] feedback_task_id={task_id}，"
                f"current_task_id={self._auto_charging_task_id}。")
            return

        feedback = feedback_msg.feedback
        self._last_auto_charging_feedback = feedback
        if self._auto_charging_status != "CANCELING":
            self._set_auto_charging_status(
                "EXECUTING", task_id,
                f"收到服务端反馈 current_state={feedback.current_state}")

        now = time.monotonic()
        feedback_signature = (feedback.current_state, feedback.retry_count)
        state_changed = (
            feedback_signature != self._last_auto_charging_feedback_signature)
        heartbeat_due = (
            self._last_auto_charging_feedback_log_time is None or
            now - self._last_auto_charging_feedback_log_time >=
            self.MOTION_PROGRESS_LOG_INTERVAL)
        if state_changed or heartbeat_due:
            self._last_auto_charging_feedback_signature = feedback_signature
            self._last_auto_charging_feedback_log_time = now
            self.get_logger().info(
                f"[自动回充进度] id={task_id}，阶段={feedback.current_state}，"
                f"重试={feedback.retry_count}，"
                f"已运行={self._elapsed_since(self._auto_charging_started_time):.1f}s。")

    def set_chassis_auto_charging(self,
                                  force: bool = True,
                                  timeout_sec: float = 10.0) -> bool:
        """串行发送回充目标，拒绝同一客户端重复启动回充任务。"""
        with self._auto_charging_request_lock:
            return self._set_chassis_auto_charging(force, timeout_sec)

    def _set_chassis_auto_charging(self,
                                   force: bool = True,
                                   timeout_sec: float = 10.0) -> bool:
        """异步发送回充 goal；返回值不代表最终回充成功。"""
        if self._auto_charging_action_client is None:
            self.get_logger().error(
                "[自动回充发送失败] AutoCharging Action 客户端未初始化。")
            return False

        if self.has_cancelable_auto_charging_goal():
            self.get_logger().warn(
                "[自动回充重复请求已拒绝] "
                f"当前 task_id={self._auto_charging_task_id}，"
                f"状态={self._auto_charging_status}。"
                "请等待当前回充结束或先取消当前回充任务。")
            return False

        action_name = getattr(self._auto_charging_action_client, "_action_name",
                              "/auto_charging")

        self._auto_charging_task_id += 1
        current_task_id = self._auto_charging_task_id
        self._auto_charging_started_time = time.monotonic()
        self._auto_charging_accepted_time = None
        self._auto_charging_cancel_requested_time = None
        self._auto_charging_done = False
        self._auto_charging_success = False
        self._auto_charging_goal_handle = None
        self._auto_charging_result_future = None
        self._auto_charging_cancel_future = None
        self._last_auto_charging_feedback = None
        self._last_auto_charging_result_message = ""
        self._last_auto_charging_feedback_signature = None
        self._last_auto_charging_feedback_log_time = None
        self._set_auto_charging_status(
            "WAITING_SERVER", current_task_id,
            f"等待 Action 服务 {action_name}")

        wait_started_time = time.monotonic()
        if not self._auto_charging_action_client.wait_for_server(
                timeout_sec=timeout_sec):
            self._set_auto_charging_status(
                "FAILED_TO_SEND", current_task_id,
                f"等待 Action 服务超过 {timeout_sec:.1f}s")
            self.get_logger().error(
                f"[自动回充发送失败] task_id={current_task_id}，"
                f"Action 服务={action_name} 不可用，"
                f"等待服务耗时={self._elapsed_since(wait_started_time):.3f}s。"
                "请检查 auto_charging Action server 是否启动。")
            self._auto_charging_done = True
            return False
        wait_server_elapsed = self._elapsed_since(wait_started_time)

        goal_msg = AutoCharging.Goal()
        goal_msg.force = force

        self._set_auto_charging_status(
            "SENDING", current_task_id,
            f"Action 服务已发现，等待服务耗时 {wait_server_elapsed:.3f}s")

        def auto_charging_feedback_callback(feedback_msg):
            self._auto_charging_feedback_callback(feedback_msg, current_task_id)

        try:
            self._auto_charging_send_goal_future = (
                self._auto_charging_action_client.send_goal_async(
                    goal_msg,
                    feedback_callback=auto_charging_feedback_callback))
            self._auto_charging_send_goal_future.add_done_callback(
                lambda future: self._auto_charging_goal_done_callback(
                    future, current_task_id))
            self._record_chassis_command(
                "开始自动回充", f"回充 Action（{action_name}）",
                f"任务编号={current_task_id}，是否强制回充="
                f"{'是' if force else '否'}")
            self.get_logger().info(
                f"[自动回充] 已发送 id={current_task_id}，"
                f"force={str(force).lower()}，"
                f"服务等待={wait_server_elapsed:.2f}s。")
            return True
        except Exception as e:
            self._set_auto_charging_status(
                "FAILED_TO_SEND", current_task_id,
                "发送 AutoCharging goal 时发生异常")
            self.get_logger().error(
                f"[自动回充目标发送异常] task_id={current_task_id}，"
                f"任务耗时="
                f"{self._elapsed_since(self._auto_charging_started_time):.3f}s，"
                f"error={e}")
            self._auto_charging_done = True
            return False

    def get_auto_charging_status(self) -> str:
        """获取当前回充 Action 状态；终态保留到下一次回充任务。"""
        return self._auto_charging_status

    def has_cancelable_auto_charging_goal(self) -> bool:
        """返回当前是否存在可取消的自动回充 goal。"""
        return (self._auto_charging_status in self.AUTO_CHARGING_CANCELABLE_STATES
                or self._auto_charging_goal_handle is not None)

    def get_auto_charging_result(self) -> tuple[bool, str]:
        """获取最近一次回充 action 结果。"""
        return self._auto_charging_success, self._last_auto_charging_result_message

    def cancel_auto_charging(self) -> bool:
        """异步取消回充；最终是否取消完成以 Action result 为准。"""
        if self._auto_charging_cancel_future is not None:
            self.get_logger().info(
                f"[自动回充取消跳过] task_id={self._auto_charging_task_id}，"
                "取消请求已经发送，正在等待最终结果。")
            return True

        if self._auto_charging_goal_handle is None and self._auto_charging_status in {
                "WAITING_SERVER", "SENDING", "ACCEPTED", "EXECUTING"
        }:
            self.get_logger().info(
                f"[自动回充取消等待] task_id={self._auto_charging_task_id}，"
                "目标尚未被接受，最多等待 1.0s。")
            current_task_id = self._auto_charging_task_id
            start_time = time.time()
            while time.time() - start_time < 1.0:
                if current_task_id != self._auto_charging_task_id:
                    break
                self._spin_once_if_not_executed(timeout_sec=0.05)
                if self._auto_charging_goal_handle is not None:
                    break
                time.sleep(0.05)

        if self._auto_charging_goal_handle is None:
            self.get_logger().info(
                f"[自动回充取消跳过] task_id="
                f"{self._auto_charging_task_id}，"
                f"当前状态={self._auto_charging_status}，"
                "没有已接受的回充目标。")
            return False

        try:
            current_task_id = self._auto_charging_task_id
            self._auto_charging_cancel_requested_time = time.monotonic()
            self._set_auto_charging_status(
                "CANCELING", current_task_id,
                "上层已调用 cancel_auto_charging()")
            self._auto_charging_cancel_future = (
                self._auto_charging_goal_handle.cancel_goal_async())
            self._auto_charging_cancel_future.add_done_callback(
                lambda future: self._auto_charging_cancel_done_callback(
                    future, current_task_id))
            self._record_chassis_command(
                "取消自动回充", "回充 Action 取消接口（/auto_charging）",
                f"任务编号={current_task_id}")
            self.get_logger().info(
                f"[自动回充取消请求已发送] task_id={current_task_id}，"
                f"请求取消时任务已运行="
                f"{self._elapsed_since(self._auto_charging_started_time):.3f}s。"
                "这里只表示取消请求已发出，最终以 Action result 为准。")
            return True
        except Exception as e:
            self._auto_charging_cancel_future = None
            self.get_logger().error(
                f"[自动回充取消请求异常] task_id={current_task_id}，"
                f"状态={self._auto_charging_status}，"
                f"error={e}")
            return False

    def wait_for_auto_charging_done(self, timeout: float = 300.0,
                                    cancel_on_timeout: bool = True
                                    ) -> tuple[bool, str]:
        """轮询等待回充 Action 进入最终状态；风格与 wait_for_navigation_arrival 一致。

        Args:
            timeout: 总超时秒数。行为树 Dock 60s×3 + CheckCharging 20s×3，
                理论上限约 240s，默认 300s 留余量。
            cancel_on_timeout: 超时是否主动取消回充 goal。

        Returns:
            (success, message)：
                - success=True 仅当 Action 状态为 SUCCEEDED 且 result.success=True；
                - 其余终态（FAILED/CANCELED/ABORTED/REJECTED/FAILED_TO_SEND）、
                  超时或接口不可用均返回 False，message 携带原因。
        """
        start_time = time.monotonic()
        while True:
            self.spin_once()
            status = self._auto_charging_status

            if status == "SUCCEEDED":
                success = bool(self._auto_charging_success)
                message = self._last_auto_charging_result_message or "回充完成"
                self.get_logger().info(
                    f"[自动回充等待结束] 成功={success}，message={message}，"
                    f"耗时={time.monotonic() - start_time:.1f}s。")
                return success, message

            if status in {"FAILED", "FAILED_TO_SEND", "CANCELED",
                          "CANCELLED", "ABORTED", "REJECTED", "UNKNOWN"}:
                message = self._last_auto_charging_result_message or status
                self.get_logger().error(
                    f"[自动回充等待结束] 终态={status}，message={message}，"
                    f"耗时={time.monotonic() - start_time:.1f}s。")
                return False, message

            if time.monotonic() - start_time > timeout:
                self.get_logger().error(
                    f"[自动回充等待超时] 当前状态={status}，"
                    f"超时={timeout:.1f}s。")
                if cancel_on_timeout:
                    self.cancel_auto_charging()
                return False, f"回充等待超时（{timeout:.1f}s）"

            time.sleep(0.1)

    def _auto_charging_goal_done_callback(self, future, task_id):
        if task_id != self._auto_charging_task_id:
            self.get_logger().debug(
                f"[自动回充旧目标响应已忽略] response_task_id={task_id}，"
                f"current_task_id={self._auto_charging_task_id}。")
            return

        try:
            goal_handle = future.result()
        except Exception as e:
            self._set_auto_charging_status(
                "FAILED_TO_SEND", task_id, "获取 goal 响应时发生异常")
            self.get_logger().error(
                f"[自动回充目标响应异常] task_id={task_id}，"
                f"发送后等待耗时="
                f"{self._elapsed_since(self._auto_charging_started_time):.3f}s，"
                f"error={e}")
            self._auto_charging_done = True
            return

        if not goal_handle.accepted:
            self._set_auto_charging_status(
                "REJECTED", task_id, "Action server 拒绝 goal")
            self.get_logger().error(
                f"[自动回充目标被拒绝] task_id={task_id}，"
                f"总耗时="
                f"{self._elapsed_since(self._auto_charging_started_time):.3f}s。"
                "请检查是否已有回充任务、服务端前置条件和回充状态机日志。")
            self._auto_charging_done = True
            return

        self._auto_charging_goal_handle = goal_handle
        self._auto_charging_accepted_time = time.monotonic()
        self._set_auto_charging_status(
            "ACCEPTED", task_id, "Action server 已接受 goal")
        self.get_logger().info(
            f"[自动回充目标已接受] task_id={task_id}，"
            f"从发送到接受耗时="
            f"{self._elapsed_since(self._auto_charging_started_time):.3f}s，"
            "等待回充反馈和最终结果。")
        self._auto_charging_result_future = goal_handle.get_result_async()
        self._auto_charging_result_future.add_done_callback(
            lambda future: self._auto_charging_result_done_callback(
                future, task_id))

    def _auto_charging_cancel_done_callback(self, future, task_id):
        if task_id != self._auto_charging_task_id:
            self.get_logger().debug(
                f"[自动回充旧取消响应已忽略] response_task_id={task_id}，"
                f"current_task_id={self._auto_charging_task_id}。")
            return

        try:
            response = future.result()
            cancel_count = len(response.goals_canceling)
            message = (
                f"[自动回充取消响应] task_id={task_id}，"
                f"正在取消的目标数={cancel_count}，"
                f"响应耗时="
                f"{self._elapsed_since(self._auto_charging_cancel_requested_time):.3f}s。"
                "收到取消响应不等于回充已停止，继续等待 Action result。")
            if cancel_count > 0:
                self.get_logger().info(message)
            else:
                self.get_logger().warn(message)
        except Exception as e:
            self.get_logger().error(
                f"[自动回充取消响应异常] task_id={task_id}，"
                f"error={e}")

    def _auto_charging_result_done_callback(self, future, task_id):
        if task_id != self._auto_charging_task_id:
            self.get_logger().debug(
                f"[自动回充旧结果已忽略] result_task_id={task_id}，"
                f"current_task_id={self._auto_charging_task_id}。")
            return

        try:
            response = future.result()
            result = response.result
            final_status = self.AUTO_CHARGING_STATUS_NAMES.get(
                response.status, "UNKNOWN")
            self._auto_charging_success = (final_status
                                           == "SUCCEEDED" and
                                           bool(result.success))
            self._last_auto_charging_result_message = result.message
            self._auto_charging_done = True
            self._auto_charging_goal_handle = None
            self._auto_charging_cancel_future = None
            self._set_auto_charging_status(
                final_status, task_id,
                f"收到 Action result："
                f"{self._action_result_status_name(response.status)}")

            feedback_state = (self._last_auto_charging_feedback.current_state
                              if self._last_auto_charging_feedback is not None
                              else "无反馈")
            retry_count = (self._last_auto_charging_feedback.retry_count
                           if self._last_auto_charging_feedback is not None
                           else -1)
            message = (
                f"[自动回充最终结果] task_id={task_id}，"
                f"客户端状态={final_status}，"
                f"Action结果={self._action_result_status_name(response.status)}，"
                f"success={bool(result.success)}，message={result.message}，"
                f"最后服务端阶段={feedback_state}，重试次数={retry_count}，"
                f"总耗时="
                f"{self._elapsed_since(self._auto_charging_started_time):.3f}s，"
                f"接受后执行耗时="
                f"{self._elapsed_since(self._auto_charging_accepted_time):.3f}s，"
                f"当前位置={self._get_cached_chassis_pose_summary()}。")
            if final_status in {"SUCCEEDED", "CANCELED"}:
                self.get_logger().info(message)
            else:
                self.get_logger().error(message)
        except Exception as e:
            self._set_auto_charging_status(
                "UNKNOWN", task_id, "获取 Action result 时发生异常")
            self.get_logger().error(
                f"[自动回充结果异常] task_id={task_id}，"
                f"总耗时="
                f"{self._elapsed_since(self._auto_charging_started_time):.3f}s，"
                f"最后反馈={self._last_auto_charging_feedback}，error={e}")
            self._auto_charging_success = False
            self._auto_charging_done = True
            self._auto_charging_goal_handle = None
            self._auto_charging_cancel_future = None

    def get_control_status(self) -> str:
        """获取当前底盘直控 Action 状态。

        Returns:
            str: 控制状态字符串，可能的取值：
                - "IDLE": 空闲，未在控制
                - "WAITING_SERVER": 正在等待 Action 服务
                - "SENDING": 正在发送 Action goal
                - "CONTROL": 正在执行控制指令
                - "SUCCEEDED": 本次控制已完成
                - "CANCELLING": 正在取消，平滑停止中
                - "CANCELLED": 本次控制已取消
                - "FAILED": 目标被拒绝、执行失败或结果异常

        终态会保留到下一条控制目标发送，避免上层轮询漏读。
        """
        self.spin_once()
        return self._control_status

    def has_cancelable_control_goal(self) -> bool:
        """返回当前是否有值得取消的底盘直控 Action goal。"""
        return (self._control_status in self.CONTROL_CANCELABLE_STATES or
                self._control_goal_handle is not None)

    def cancel_chassis_control(self) -> bool:
        """异步取消当前底盘直控 Action goal。"""
        if not self.has_cancelable_control_goal():
            self.get_logger().info(
                f"[底盘直控取消跳过] 当前状态={self._control_status}，"
                "没有可取消的目标。")
            return False

        if self._control_cancel_future is not None:
            self.get_logger().info(
                f"[底盘直控取消跳过] task_id={self._control_task_id}，"
                "取消请求已经发送，正在等待底层平滑停车。")
            return True

        current_task_id = self._control_task_id
        if self._control_goal_handle is None:
            self._control_cancel_requested = True
            self._control_cancel_requested_time = time.monotonic()
            self._set_control_status(
                "CANCELLING", current_task_id,
                "goal 尚未接受，已记录延迟取消请求")
            self.get_logger().info(
                f"[底盘直控取消等待] task_id={current_task_id}，"
                "goal 尚未被服务端接受，接受后将立即发送取消请求。")
            return True

        try:
            self._control_cancel_requested = True
            self._control_cancel_requested_time = time.monotonic()
            self._set_control_status(
                "CANCELLING", current_task_id,
                "上层已调用 cancel_chassis_control()")
            self._control_cancel_future = (
                self._control_goal_handle.cancel_goal_async())
            self._control_cancel_future.add_done_callback(
                lambda future: self._on_control_cancel_response(
                    future, current_task_id))
            self._record_chassis_command(
                "取消底盘直控", "Action 取消接口（/control/precise_control）",
                f"任务编号={current_task_id}")
            self.get_logger().info(
                f"[底盘直控取消请求已发送] task_id={current_task_id}，"
                f"请求取消时任务已运行="
                f"{self._elapsed_since(self._control_started_time):.3f}s，"
                "等待底层平滑停车和最终 Action result。")
            return True
        except Exception as e:
            self._control_cancel_future = None
            self.get_logger().error(
                f"[底盘直控取消请求异常] task_id={current_task_id}，"
                f"状态={self._control_status}，"
                f"error={e}")
            return False

    def set_chassis_control_position(self, position) -> bool:
        """串行发送底盘直控目标，避免多个业务线程同时创建任务。"""
        with self._control_request_lock:
            return self._set_chassis_control_position(position)

    def _set_chassis_control_position(self, position) -> bool:
        """发送一个底盘控制目标点，精度较高。

        Args:
            position (list/np.ndarray): 目标位置 [x, 0, yaw]，yaw单位为度。
                注意：x和yaw不能同时非零。

        Returns:
            bool: 如果目标发送成功则为 True。
        """
        target = self._normalize_motion_target(position, "底盘直控")
        if target is None:
            return False

        if target[0] != 0.0 and target[2] != 0:
            self.get_logger().info(
                "[底盘直控目标非法] x 和 yaw 不能同时为非零值，"
                f"当前目标=({target[0]:.3f}, {target[2]:.1f}°)。")
            return False

        if self._control_action_client is None:
            self.get_logger().error(
                "[底盘直控目标发送失败] PreciseControl Action 客户端未初始化。")
            return False

        if self.has_cancelable_control_goal():
            self.get_logger().error(
                f"[底盘直控目标发送失败] 当前任务仍未结束，"
                f"状态={self._control_status}。请先取消或等待当前任务完成。")
            return False

        self._control_task_id += 1
        current_task_id = self._control_task_id
        self._control_goal_handle = None
        self._control_result_future = None
        self._control_cancel_future = None
        self._control_cancel_requested = False
        self._control_done = False
        self._control_success = False
        self._last_control_feedback = None
        self._last_control_result_message = ""
        self._control_target_position = target
        self._control_started_time = time.monotonic()
        self._control_accepted_time = None
        self._control_cancel_requested_time = None
        self._control_last_progress_log_time = None
        self._set_control_status(
            "WAITING_SERVER", current_task_id,
            "等待 /control/precise_control Action 服务")

        wait_started_time = time.monotonic()
        if not self._control_action_client.wait_for_server(timeout_sec=1.0):
            self._control_done = True
            self._set_control_status(
                "FAILED", current_task_id,
                "等待 /control/precise_control Action 服务超过 1.0s")
            self.get_logger().error(
                f"[底盘直控目标发送失败] task_id={current_task_id}，"
                "/control/precise_control Action 服务不可用，"
                f"等待服务耗时={self._elapsed_since(wait_started_time):.3f}s，"
                f"目标=相对位移{target[0]:.3f}m/"
                f"相对转角{target[2]:.1f}°。"
                "请确认 mobile_control_action_node 已启动，"
                "且旧 mobile_control_node 未同时控制 /control/cmd_vel。")
            return False
        wait_server_elapsed = self._elapsed_since(wait_started_time)

        chassis_position = PoseStamped()
        chassis_position.header.frame_id = "base_link"
        chassis_position.header.stamp = self.get_clock().now().to_msg()
        chassis_position.pose.position.x = target[0]
        chassis_position.pose.position.y = 0.0
        chassis_position.pose.position.z = 0.0

        rotation = R.from_euler('xyz', [0, 0, target[2]], degrees=True)
        quat_arry = rotation.as_quat()
        chassis_position.pose.orientation.x = quat_arry[0]
        chassis_position.pose.orientation.y = quat_arry[1]
        chassis_position.pose.orientation.z = quat_arry[2]
        chassis_position.pose.orientation.w = quat_arry[3]

        goal_msg = PreciseControl.Goal()
        goal_msg.target_pose = chassis_position

        self._record_chassis_command(
            "底盘直控", "Action（/control/precise_control）",
            f"任务编号={current_task_id}，相对前后移动={target[0]:.3f}m，"
            f"旋转角度={target[2]:.1f}°")
        self._set_control_status(
            "SENDING", current_task_id,
            f"Action 服务已发现，等待服务耗时 {wait_server_elapsed:.3f}s")

        try:
            self._control_send_goal_future = (
                self._control_action_client.send_goal_async(
                    goal_msg,
                    feedback_callback=lambda feedback_msg:
                    self._on_control_feedback(
                        feedback_msg, current_task_id)))
            self._control_send_goal_future.add_done_callback(
                lambda future: self._on_control_goal_response(
                    future, current_task_id))
            self.get_logger().info(
                f"[底盘直控] 已发送 id={current_task_id}，"
                f"相对位移={target[0]:.3f}m，相对转角={target[2]:.1f}°，"
                f"服务等待={wait_server_elapsed:.2f}s。")
            return True
        except Exception as e:
            self._control_done = True
            self._set_control_status(
                "FAILED", current_task_id, "发送 PreciseControl goal 时发生异常")
            self.get_logger().error(
                f"[底盘直控目标发送异常] task_id={current_task_id}，"
                f"任务耗时={self._elapsed_since(self._control_started_time):.3f}s，"
                f"error={e}")
            return False

    def _set_control_status(self, status: str, task_id: int,
                            reason: str = "") -> None:
        """更新当前底盘直控 Action 状态，并记录状态变化或低频进度。"""
        if task_id != self._control_task_id:
            return

        old_status = self._control_status
        self._control_status = status
        if old_status != status:
            reason_text = f"，原因={reason}" if reason else ""
            self.get_logger().info(
                f"[底盘直控状态] id={task_id} {old_status}->{status}，"
                f"耗时={self._elapsed_since(self._control_started_time):.1f}s"
                f"{reason_text}。")

        if status != "CONTROL":
            return

        now = time.monotonic()
        if old_status == status and \
                self._control_last_progress_log_time is not None and \
                now - self._control_last_progress_log_time < \
                self.MOTION_PROGRESS_LOG_INTERVAL:
            return

        self._control_last_progress_log_time = now
        elapsed = (now - self._control_started_time
                   if self._control_started_time is not None else -1.0)
        target = self._control_target_position
        target_summary = (
            f"相对位移={target[0]:.3f}m，相对转角={target[2]:.1f}°"
            if target is not None else "目标未记录")
        self.get_logger().info(
            f"[底盘直控进度] id={task_id}，{target_summary}，"
            f"已执行={elapsed:.1f}s。")

    def _on_control_feedback(self, feedback_msg, task_id: int) -> None:
        """处理 PreciseControl feedback，并忽略旧任务的迟到反馈。"""
        if task_id != self._control_task_id:
            self.get_logger().debug(
                f"[底盘直控旧反馈已忽略] feedback_task_id={task_id}，"
                f"current_task_id={self._control_task_id}。")
            return

        feedback = feedback_msg.feedback
        self._last_control_feedback = feedback
        status = feedback.current_status
        if status in {"CONTROL", "CANCELLING"}:
            self._set_control_status(status, task_id)
        elif status == "SUCCEEDED":
            # feedback 只表示服务端进入完成阶段，Action result 才是最终结果。
            self._set_control_status("CONTROL", task_id)
            self.get_logger().debug(
                f"[底盘直控完成反馈] task_id={task_id}，"
                "已收到 SUCCEEDED feedback，继续等待 Action result。")
        elif status == "IDLE":
            # goal 已被接受后，服务端可能在等待 map->base_link TF 时反馈 IDLE。
            # IDLE 不是 Action 最终结果，客户端仍保持 CONTROL，避免上层误判完成。
            self._set_control_status("CONTROL", task_id)
        else:
            self.get_logger().warn(
                f"[底盘直控未知反馈] task_id={task_id}，"
                f"current_status={status}。")

    def _on_control_goal_response(self, future, task_id: int) -> None:
        """处理 PreciseControl goal 接受或拒绝结果。"""
        if task_id != self._control_task_id:
            self.get_logger().debug(
                f"[底盘直控旧目标响应已忽略] response_task_id={task_id}，"
                f"current_task_id={self._control_task_id}。")
            return

        try:
            goal_handle = future.result()
        except Exception as e:
            self._control_done = True
            self._set_control_status(
                "FAILED", task_id, "获取 goal 响应时发生异常")
            self.get_logger().error(
                f"[底盘直控目标响应异常] task_id={task_id}，"
                f"发送后等待耗时="
                f"{self._elapsed_since(self._control_started_time):.3f}s，"
                f"error={e}")
            return

        if not goal_handle.accepted:
            self._control_done = True
            self._set_control_status(
                "FAILED", task_id, "Action server 拒绝 goal")
            self.get_logger().error(
                f"[底盘直控目标被拒绝] task_id={task_id}。"
                "可能已有其他 PreciseControl goal 正在执行，"
                "或目标 TF 转换失败。")
            return

        self._control_goal_handle = goal_handle
        self._control_accepted_time = time.monotonic()
        self._set_control_status(
            "CANCELLING" if self._control_cancel_requested else "CONTROL",
            task_id, "Action server 已接受 goal")
        self.get_logger().info(
            f"[底盘直控目标已接受] task_id={task_id}，"
            f"从发送到接受耗时="
            f"{self._elapsed_since(self._control_started_time):.3f}s，"
            "等待执行反馈和最终结果。")
        self._control_result_future = goal_handle.get_result_async()
        self._control_result_future.add_done_callback(
            lambda result_future: self._on_control_result(
                result_future, task_id))

        if self._control_cancel_requested:
            self.cancel_chassis_control()

    def _on_control_cancel_response(self, future, task_id: int) -> None:
        """记录 Action cancel 请求是否被服务端接受。"""
        if task_id != self._control_task_id:
            self.get_logger().debug(
                f"[底盘直控旧取消响应已忽略] response_task_id={task_id}，"
                f"current_task_id={self._control_task_id}。")
            return

        try:
            response = future.result()
            cancel_count = len(response.goals_canceling)
            message = (
                f"[底盘直控取消响应] task_id={task_id}，"
                f"正在取消的目标数={cancel_count}，"
                f"响应耗时="
                f"{self._elapsed_since(self._control_cancel_requested_time):.3f}s。"
                "收到取消响应不等于已经停车，继续等待 Action result。")
            if cancel_count > 0:
                self.get_logger().info(message)
            else:
                self.get_logger().warn(message)
        except Exception as e:
            self.get_logger().error(
                f"[底盘直控取消响应异常] task_id={task_id}，error={e}")

    def _on_control_result(self, future, task_id: int) -> None:
        """处理与本次 goal 绑定的 PreciseControl 最终结果。"""
        if task_id != self._control_task_id:
            self.get_logger().debug(
                f"[底盘直控旧结果已忽略] result_task_id={task_id}，"
                f"current_task_id={self._control_task_id}。")
            return

        try:
            response = future.result()
            result = response.result
            self._last_control_result_message = result.message
            self._control_success = (
                response.status == GoalStatus.STATUS_SUCCEEDED and
                bool(result.success))

            if self._control_success:
                final_status = "SUCCEEDED"
            elif response.status == GoalStatus.STATUS_CANCELED:
                final_status = "CANCELLED"
            else:
                final_status = "FAILED"

            self._control_done = True
            self._control_goal_handle = None
            self._control_cancel_requested = False
            self._control_cancel_future = None
            self._set_control_status(
                final_status, task_id,
                f"收到 Action result："
                f"{self._action_result_status_name(response.status)}")
            elapsed = (time.monotonic() - self._control_started_time
                       if self._control_started_time is not None else -1.0)
            message = (
                f"[底盘直控结果] id={task_id}，状态={final_status}，"
                f"success={bool(result.success)}，耗时={elapsed:.1f}s，"
                f"message={result.message}。")
            if final_status in {"SUCCEEDED", "CANCELLED"}:
                self.get_logger().info(message)
            else:
                self.get_logger().error(message)
        except Exception as e:
            self._control_done = True
            self._control_success = False
            self._control_goal_handle = None
            self._control_cancel_requested = False
            self._control_cancel_future = None
            self._set_control_status(
                "FAILED", task_id, "获取 Action result 时发生异常")
            self.get_logger().error(
                f"[底盘直控结果异常] task_id={task_id}，"
                f"总耗时={self._elapsed_since(self._control_started_time):.3f}s，"
                f"最后反馈={self._last_control_feedback}，error={e}")

    def set_goal_tolerance(self, xy_tolerance: float,
                           yaw_tolerance: float) -> bool:
        """
        设置导航到达精度
        yaw_tolerance 单位：弧度（Nav2 内部用 rad）
        """

        lon_ok = self._set_param(self._controller_set_param_client,
                                 self._controller_get_param_client,
                                 'goal_checker.lon_goal_tolerance',
                                 xy_tolerance)
        lat_ok = self._set_param(self._controller_set_param_client,
                                 self._controller_get_param_client,
                                 'goal_checker.lat_goal_tolerance',
                                 xy_tolerance)
        yaw_ok = self._set_param(self._controller_set_param_client,
                                 self._controller_get_param_client,
                                 'goal_checker.yaw_goal_tolerance',
                                 yaw_tolerance)

        self.get_logger().info(
            f"Set goal tolerance: xy={xy_tolerance}, yaw={yaw_tolerance}")

        return lon_ok and lat_ok and yaw_ok

    def set_footprint(self, vertices: list) -> bool:
        """
        设置机器人 footprint
        vertices: [[x1,y1], [x2,y2], [x3,y3], [x4,y4]]
        """

        if len(vertices) != 4:
            self.get_logger().error("Footprint must have 4 vertices")
            return False

        footprint_str = "[" + ",".join([f"[{v[0]},{v[1]}]" for v in vertices
                                       ]) + "]"

        # local
        local_success = self._set_param(self._local_costmap_set_param_client,
                                        self._local_costmap_get_param_client,
                                        'footprint', footprint_str)

        # global
        global_success = self._set_param(self._global_costmap_set_param_client,
                                         self._global_costmap_get_param_client,
                                         'footprint', footprint_str)

        if local_success and global_success:
            self.get_logger().info(
                f"Successfully set and verified footprint: {footprint_str}")
            return True
        else:
            self.get_logger().error(f"Failed to set footprint.")
            return False

    def set_inflation_radius(self, radius: float) -> bool:
        """
        设置膨胀层半径（local + global）
        """

        local_success = self._set_param(self._local_costmap_set_param_client,
                                        self._local_costmap_get_param_client,
                                        'inflation_layer.inflation_radius',
                                        radius)

        global_success = self._set_param(self._global_costmap_set_param_client,
                                         self._global_costmap_get_param_client,
                                         'inflation_layer.inflation_radius',
                                         radius)

        if local_success and global_success:
            self.get_logger().info(
                f"Successfully set and verified inflation radius to {radius}")
            return True
        else:
            self.get_logger().error(f"Failed to set inflation radius.")
            return False

    def set_follow_velocity_limits(self, vx_max: float, wz_max: float):
        """
        同时设置 FollowPath 线速度 & 角速度上限
        """

        vx_success = self._set_param(self._controller_set_param_client,
                                     self._controller_get_param_client,
                                     'FollowPath.vx_max', vx_max)

        wz_success = self._set_param(self._controller_set_param_client,
                                     self._controller_get_param_client,
                                     'FollowPath.wz_max', wz_max)

        if vx_success and wz_success:
            self.get_logger().info(
                f"Successfully set and verified FollowPath limits: vx_max={vx_max}, wz_max={wz_max}"
            )
            return True
        else:
            self.get_logger().error(f"Failed to set FollowPath limits.")
            return False

    def spin_once(self, timeout_sec: float = 0.1):
        r"""仅在独立运行模式下处理一次回调，避免抢占外部 Executor 管理的节点。"""
        if self._is_managed_by_executor():
            return
        rclpy.spin_once(self, timeout_sec=timeout_sec)

    def _is_managed_by_executor(self) -> bool:
        """Return whether this node is currently owned by an executor."""
        executor = self.executor
        return executor is not None and self in executor.get_nodes()

    def _wait_for_future(self, future, timeout_sec: float) -> bool:
        """等待 Future 完成，避免为当前节点启动第二个 Executor。

        当节点被 Executor 管理时，不能调用 rclpy.spin_until_future_complete
        （会为同一节点启动第二个 Executor 导致冲突），改用 busy-wait 轮询。
        但如果调用方本身就在 Executor 的回调线程中，回调响应无法被调度执行，
        future 永远不会 done，最终超时。
        """
        if not self._is_managed_by_executor():
            rclpy.spin_until_future_complete(
                self, future, timeout_sec=timeout_sec)
            if not future.done():
                self.get_logger().warn(
                    f"_wait_for_future 超时(独立模式): "
                    f"timeout={timeout_sec}s, future未完成")
            return future.done()

        # 节点被 Executor 管理，用 busy-wait 轮询等待 future 完成
        deadline = time.monotonic() + timeout_sec
        while not future.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                self.get_logger().warn(
                    f"_wait_for_future 超时(executor模式): "
                    f"timeout={timeout_sec}s。"
                    f"可能原因: 1)服务端未响应 2)当前调用线程就是executor回调线程导致响应无法被调度"
                )
                return False
            time.sleep(min(0.01, remaining))
        return True

    def _spin_once_if_not_executed(self, timeout_sec: float = 0.01):
        try:
            self.spin_once(timeout_sec=timeout_sec)
        except RuntimeError as e:
            self.get_logger().debug(f"spin_once skipped: {e}")

    def spin(self):
        r"""Continuously spin the node while rclpy is running."""
        if self._is_managed_by_executor():
            return
        while rclpy.ok():
            self.spin_once(timeout_sec=0.1)

    def _on_nav_goal_response(self, future, task_id):
        """处理 NavigateToPose goal 的接受或拒绝响应。"""
        if task_id != self._nav_task_id:
            self.get_logger().debug(
                f"[导航旧目标响应已忽略] response_task_id={task_id}，"
                f"current_task_id={self._nav_task_id}。")
            return

        try:
            goal_handle = future.result()
        except Exception as e:
            self._set_nav_status(
                "FAILED", task_id, "获取 NavigateToPose goal 响应时发生异常")
            self.get_logger().error(
                f"[导航目标响应异常] task_id={task_id}，"
                f"等待响应耗时={self._elapsed_since(self._nav_started_time):.3f}s，"
                f"目标={self._nav_target_summary()}，error={e}")
            return

        if not goal_handle.accepted:
            self._set_nav_status(
                "FAILED", task_id, "NavigateToPose Action server 拒绝 goal")
            self.get_logger().error(
                f"[导航目标被拒绝] task_id={task_id}，"
                f"等待响应耗时={self._elapsed_since(self._nav_started_time):.3f}s，"
                f"目标={self._nav_target_summary()}。"
                "请检查 Nav2 生命周期状态、目标合法性及是否存在冲突任务。")
            return

        self._nav_goal_handle = goal_handle
        self._nav_accepted_time = time.monotonic()
        self.get_logger().info(
            f"[导航目标已接受] task_id={task_id}，"
            f"从请求到接受耗时={self._elapsed_since(self._nav_started_time):.3f}s，"
            f"目标={self._nav_target_summary()}。"
            "Nav2 已开始执行，等待 feedback 和最终 result。")

        self._nav_result_future = goal_handle.get_result_async()
        self._nav_result_future.add_done_callback(
            lambda future: self._on_nav_result(future, task_id))

    def _on_nav_result(self, future, task_id):
        """处理 NavigateToPose 的最终结果。"""
        if task_id != self._nav_task_id:
            self.get_logger().debug(
                f"[导航旧结果已忽略] result_task_id={task_id}，"
                f"current_task_id={self._nav_task_id}。")
            return

        try:
            response = future.result()
            self._nav_result = response
            if response.status == GoalStatus.STATUS_SUCCEEDED:
                final_status = "SUCCEEDED"
            elif response.status == GoalStatus.STATUS_CANCELED:
                final_status = "CANCELLED"
            elif response.status == GoalStatus.STATUS_ABORTED:
                final_status = "ABORTED"
            else:
                final_status = "FAILED"

            action_status = self._action_result_status_name(response.status)
            nav_result = response.result
            error_code = getattr(nav_result, "error_code", "无此字段")
            error_msg = getattr(nav_result, "error_msg", "") or "无"
            self._set_nav_status(
                final_status, task_id, f"收到 Action result：{action_status}")
            self._nav_goal_handle = None
            self._nav_cancel_future = None

            feedback = self._nav_last_feedback_summary
            feedback_summary = (
                f"最后反馈剩余距离={feedback['distance_remaining']:.3f}m，"
                f"Nav2导航耗时={feedback['navigation_time']:.1f}s，"
                f"恢复行为次数={feedback['recoveries']}"
                if feedback is not None else "未收到导航进度反馈")
            cancel_elapsed = (
                f"，取消请求至结果耗时="
                f"{self._elapsed_since(self._nav_cancel_requested_time):.3f}s"
                if self._nav_cancel_requested_time is not None else "")
            message = (
                f"[导航最终结果] task_id={task_id}，"
                f"客户端状态={final_status}，Action结果={action_status}，"
                f"Nav2错误码={error_code}，错误信息={error_msg}，"
                f"总耗时={self._elapsed_since(self._nav_started_time):.3f}s，"
                f"接受后执行耗时={self._elapsed_since(self._nav_accepted_time):.3f}s"
                f"{cancel_elapsed}，目标={self._nav_target_summary()}，"
                f"最终位姿={self._get_cached_chassis_pose_summary()}，"
                f"{feedback_summary}。")
            if final_status in {"SUCCEEDED", "CANCELLED"}:
                self.get_logger().info(message)
            else:
                self.get_logger().error(message)
        except Exception as e:
            # 继续使用 FAILED，避免上层等待逻辑因不认识额外状态而超时。
            self._set_nav_status(
                "FAILED", task_id, "获取 NavigateToPose Action result 时发生异常")
            self._nav_goal_handle = None
            self._nav_cancel_future = None
            self.get_logger().error(
                f"[导航结果获取异常] task_id={task_id}，"
                f"总耗时={self._elapsed_since(self._nav_started_time):.3f}s，"
                f"目标={self._nav_target_summary()}，"
                f"最后反馈={self._nav_last_feedback_summary}，error={e}")

    def _on_nav_cancel_response(self, future, task_id):
        """记录导航取消请求响应；最终状态仍以 Action result 为准。"""
        if task_id != self._nav_task_id:
            self.get_logger().debug(
                f"[导航旧取消响应已忽略] response_task_id={task_id}，"
                f"current_task_id={self._nav_task_id}。")
            return

        try:
            response = future.result()
            cancel_count = len(response.goals_canceling)
            message = (
                f"[导航取消响应] task_id={task_id}，"
                f"正在取消的目标数={cancel_count}，"
                f"响应耗时="
                f"{self._elapsed_since(self._nav_cancel_requested_time):.3f}s，"
                "客户端继续保持 CANCELLING。"
                "收到取消响应不等于机器人已停止，最终以 Action result 为准。")
            if cancel_count > 0:
                self.get_logger().info(message)
            else:
                self.get_logger().warn(message)
        except Exception as e:
            self.get_logger().error(
                f"[导航取消响应异常] task_id={task_id}，"
                f"响应耗时="
                f"{self._elapsed_since(self._nav_cancel_requested_time):.3f}s，"
                f"客户端继续等待最终结果，error={e}")

    def _get_param(self, client, name: str):
        if not client.wait_for_service(timeout_sec=0.5):
            self.get_logger().error(
                f"Get parameter service '{client.srv_name}' not available")
            return None

        req = GetParameters.Request()
        req.names = [name]
        future = client.call_async(req)
        if not self._wait_for_future(future, timeout_sec=1.0):
            self.get_logger().error(
                f"Failed to get parameter '{name}': service call timed out.")
            return None

        response = future.result()
        if not response.values:
            self.get_logger().error(
                f"Failed to get parameter '{name}': parameter not found.")
            return None

        return response.values[0]

    def _set_param(self, set_client, get_client, name: str, value):
        if not set_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().error(
                f"Set parameter service '{set_client.srv_name}' not available")
            return False

        # ✅ 用 rclpy.parameter.Parameter
        param = Parameter(name=name, value=value)

        req = SetParameters.Request()
        req.parameters = [param.to_parameter_msg()]

        future = set_client.call_async(req)
        if not self._wait_for_future(future, timeout_sec=1.0):
            self.get_logger().error(f"Timeout setting parameter '{name}'")
            return False

        result = future.result().results[0]
        if not result.successful:
            self.get_logger().error(
                f"Failed to set parameter '{name}': {result.reason}")
            return False

        self.get_logger().info(
            f"Parameter '{name}' set to '{value}', verifying...")

        # ---------- 校验 ----------
        time.sleep(0.1)
        get_param_value = self._get_param(get_client, name)
        if get_param_value is None:
            self.get_logger().error(
                f"Failed to get parameter '{name}' for verification")
            return False

        retrieved_value = None

        if get_param_value.type == ParameterType.PARAMETER_DOUBLE:
            retrieved_value = get_param_value.double_value
            # 浮点数用近似比较
            if not math.isclose(retrieved_value, float(value), rel_tol=1e-6):
                self.get_logger().error(
                    f"Parameter '{name}' verification failed! "
                    f"Expected: {value}, Got: {retrieved_value}")
                return False

        elif get_param_value.type == ParameterType.PARAMETER_INTEGER:
            retrieved_value = get_param_value.integer_value
            if retrieved_value != int(value):
                self.get_logger().error(
                    f"Parameter '{name}' verification failed! "
                    f"Expected: {value}, Got: {retrieved_value}")
                return False

        elif get_param_value.type == ParameterType.PARAMETER_BOOL:
            retrieved_value = get_param_value.bool_value
            if retrieved_value != bool(value):
                self.get_logger().error(
                    f"Parameter '{name}' verification failed! "
                    f"Expected: {value}, Got: {retrieved_value}")
                return False

        elif get_param_value.type == ParameterType.PARAMETER_STRING:
            retrieved_value = get_param_value.string_value
            if retrieved_value != str(value):
                self.get_logger().error(
                    f"Parameter '{name}' verification failed! "
                    f"Expected: {value}, Got: {retrieved_value}")
                return False

        else:
            self.get_logger().error(
                f"Unsupported parameter type {get_param_value.type} for '{name}'"
            )
            return False

        self.get_logger().info(f"Parameter '{name}' verified successfully.")
        return True
