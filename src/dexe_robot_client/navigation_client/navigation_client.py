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
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import ParameterType
from rclpy.parameter import Parameter
from geometry_msgs.msg import Polygon, Point32
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import ParameterType
from rclpy.parameter import Parameter
from geometry_msgs.msg import Polygon, Point32
from std_msgs.msg import Bool

from ..common import get_logger


class NavigationClient(ABC, Node):
    NAV_CANCELABLE_STATES = {"NAVIGATING"}
    AUTO_CHARGING_CANCELABLE_STATES = {
        "WAITING_SERVER", "SENDING", "ACCEPTED", "EXECUTING"
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
        self._last_msg_time = None
        self._last_chassis_processed_feedback = None
        self._cur_pos = None
        self._connection_timeout = 1.0
        self._set_nav_publisher = None
        self._set_nav_client = None
        self._set_ctrl_publisher = None
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
        self._cancel_control_client = None
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
        self._nav_status = "IDLE"  # 或 NAVIGATING / SUCCEEDED / FAILED / CANCELLED
        self._nav_result = None  # 存储最后的导航结果
        self._nav_task_id = 0  # 导航任务ID，用于区分不同的导航任务
        self._last_control_status = None
        self._last_control_status_success = False
        # control status 防抖 + 防陈旧
        self._control_status_requesting = False
        self._last_control_status_time = None
        self._control_status_timeout = 1.0  # 秒，超过认为状态陈旧
        self._control_status_request_timeout = 2.0  # 秒，超过后清理旧请求并重试
        # 诊断：记录请求发出时刻、连续跳过次数
        self._control_status_request_sent_time = None
        self._control_status_skip_count = 0
        self._control_status_request_id = 0
        self._control_status_active_request_id = None
        self._control_status_request_future = None
        self._control_status_consecutive_timeouts = 0
        self._control_status_request_lock = threading.Lock()
        self._control_status_recovery_pending = False
        self._chassis_connection_recovery_pending = False
        self._chassis_feedback_received_during_recovery = False
        self._control_status_unknown_logged = False
        self._control_status_service_unavailable_logged = False

        self._init_subscriber(is_sim)
        self._init_publisher(is_sim)
        self.spin_once(timeout_sec=timeout_sec)

        self.create_timer(0.1, self._check_connection_status)
        self.create_timer(0.5, self.refresh_control_status)

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
        self.get_logger().info("chassis feedback received", once=True)
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
        should_log_connection_recovered = False
        with self._control_status_request_lock:
            if self._chassis_connection_recovery_pending:
                self._chassis_feedback_received_during_recovery = True
            control_status_fresh = (
                self._last_control_status_success and
                self._last_control_status_time is not None and
                time.time() - self._last_control_status_time <=
                self._control_status_timeout
            )
            if self._chassis_connection_recovery_pending and \
                    not self._control_status_recovery_pending and \
                    control_status_fresh:
                self._chassis_connection_recovery_pending = False
                self._chassis_feedback_received_during_recovery = False
                should_log_connection_recovered = True

        if should_log_connection_recovered:
            self.get_logger().info(
                "[底盘客户端连接恢复] 已重新收到 /feedback/chassis_pose，"
                "且底盘控制状态缓存有效；W1Manager 的底盘连接检查现在可以通过。"
            )
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
            self.get_logger().error("No chassis feedback received yet.")
            return False, None

        pos = self._last_chassis_processed_feedback["data"]["position"]
        x, y = pos[0], pos[1]

        q = self._last_chassis_processed_feedback["data"]["orientation"]
        quat_array = np.array([q[0], q[1], q[2], q[3]])
        rotation = R.from_quat(quat_array)

        yaw = rotation.as_euler('xyz', degrees=True)[2]
        self._cur_pos = np.array([x, y, yaw])
        return True, self._cur_pos

    def set_chassis_nav2_position(self, position) -> bool:
        """发送一个导航目标点 (非阻塞)。

        Args:
            position (list/np.ndarray): 目标位置 [x, y, yaw],yaw单位为度。

        Returns:
            bool: 如果目标发送成功则为 True。
        """
        nav2_position = NavigateToPose.Goal()
        nav2_position.pose = PoseStamped()
        nav2_position.pose.header.frame_id = "map"
        nav2_position.pose.pose.position.x = position[0]
        nav2_position.pose.pose.position.y = position[1]
        nav2_position.pose.pose.position.z = 0.0

        rotation = R.from_euler('xyz', [0, 0, position[2]], degrees=True)
        quat_arry = rotation.as_quat()

        nav2_position.pose.pose.orientation.x = quat_arry[0]
        nav2_position.pose.pose.orientation.y = quat_arry[1]
        nav2_position.pose.pose.orientation.z = quat_arry[2]
        nav2_position.pose.pose.orientation.w = quat_arry[3]

        if not self._set_nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error(
                "NavigateToPose action server not available.")
            return False

        # 清理旧的导航状态，防止旧回调干扰新任务
        self._nav_task_id += 1  # 新任务ID
        current_task_id = self._nav_task_id
        self._nav_goal_handle = None
        self._nav_status = "NAVIGATING"
        self._nav_result = None

        # 异步发送
        send_future = self._set_nav_client.send_goal_async(nav2_position)
        send_future.add_done_callback(
            lambda future: self._on_nav_goal_response(future, current_task_id))

        return True

    def get_nav_state(self) -> str:
        """获取当前导航状态。

        Returns:
            str: 导航状态字符串，可能为：
                - "IDLE": 空闲
                - "NAVIGATING": 导航中
                - "SUCCEEDED": 成功
                - "FAILED": 失败
                - "CANCELLED": 已取消
        """
        # 独立运行时主动处理一次回调；由 W1Manager Executor 管理时为空操作。
        self.spin_once()

        return self._nav_status

    def has_cancelable_nav_goal(self) -> bool:
        """Return whether there may be a Nav2 goal worth cancelling."""
        return (self._nav_status in self.NAV_CANCELABLE_STATES
                or self._nav_goal_handle is not None)

    # def is_nav_done(self) -> bool:
    #     """检查导航是否结束（无论成功、失败还是取消）。"""
    #     return self.get_nav_state() in ["SUCCEEDED", "FAILED", "CANCELLED"]

    def cancel_chassis_nav2(self) -> bool:
        """取消当前的导航任务 (非阻塞)。"""
        # 如果目标还未被接受，等待最多 1 秒让 goal_handle 被设置
        if self._nav_goal_handle is None and self._nav_status == "NAVIGATING":
            self.get_logger().info(
                "Waiting for navigation goal to be accepted before cancelling..."
            )
            start_time = time.time()
            while time.time() - start_time < 1.0:
                self.spin_once(timeout_sec=0.05)
                if self._nav_goal_handle is not None:
                    break
                time.sleep(0.05)

        if self._nav_goal_handle is None:
            self.get_logger().info("No active navigation goal to cancel.")
            return False

        # 保存当前任务ID
        current_task_id = self._nav_task_id

        try:
            cancel_future = self._nav_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda future: self._on_nav_cancel_response(
                    future, current_task_id))
            self.get_logger().info("Navigation cancel request sent.")
            return True
        except Exception as e:
            self.get_logger().error(
                f"Error cancelling navigation goal: {str(e)}")
            return False

    def _auto_charging_feedback_callback(self, feedback_msg, task_id):
        if task_id != self._auto_charging_task_id:
            self.get_logger().debug(
                f"Ignoring feedback for old AutoCharging task {task_id} "
                f"(current: {self._auto_charging_task_id})")
            return

        feedback = feedback_msg.feedback
        self._last_auto_charging_feedback = feedback
        if self._auto_charging_status != "CANCELING":
            self._auto_charging_status = "EXECUTING"
        self.get_logger().info("Auto charging feedback: "
                               f"state={feedback.current_state}, "
                               f"retry={feedback.retry_count}")

    def set_chassis_auto_charging(self,
                                  force: bool = True,
                                  timeout_sec: float = 10.0) -> bool:
        """发送回充 action，非阻塞返回是否成功发出。"""
        if self._auto_charging_action_client is None:
            self.get_logger().error(
                "Auto charging action client not initialized.")
            return False

        action_name = getattr(self._auto_charging_action_client, "_action_name",
                              "/auto_charging")
        self.get_logger().info(
            f"Waiting for AutoCharging action server: {action_name}")
        self._auto_charging_status = "WAITING_SERVER"
        if not self._auto_charging_action_client.wait_for_server(
                timeout_sec=timeout_sec):
            self.get_logger().error("Auto charging action server not available")
            self._auto_charging_status = "FAILED_TO_SEND"
            self._auto_charging_done = True
            self._auto_charging_success = False
            return False

        goal_msg = AutoCharging.Goal()
        goal_msg.force = force

        self.get_logger().info(f"Sending AutoCharging goal: force={force}")
        self._auto_charging_task_id += 1
        current_task_id = self._auto_charging_task_id
        self._auto_charging_done = False
        self._auto_charging_success = False
        self._auto_charging_status = "SENDING"
        self._auto_charging_goal_handle = None
        self._auto_charging_result_future = None
        self._auto_charging_cancel_future = None
        self._last_auto_charging_feedback = None
        self._last_auto_charging_result_message = ""

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
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to send AutoCharging goal: {e}")
            self._auto_charging_status = "FAILED_TO_SEND"
            self._auto_charging_done = True
            self._auto_charging_success = False
            return False

    def get_auto_charging_status(self) -> str:
        """获取当前回充 action 状态。"""
        return self._auto_charging_status

    def has_cancelable_auto_charging_goal(self) -> bool:
        """Return whether there may be an AutoCharging goal worth cancelling."""
        return (self._auto_charging_status in self.AUTO_CHARGING_CANCELABLE_STATES
                or self._auto_charging_goal_handle is not None)

    def get_auto_charging_result(self) -> tuple[bool, str]:
        """获取最近一次回充 action 结果。"""
        return self._auto_charging_success, self._last_auto_charging_result_message

    def cancel_auto_charging(self) -> bool:
        """取消当前回充 action。"""
        if self._auto_charging_goal_handle is None and self._auto_charging_status in {
                "WAITING_SERVER", "SENDING", "ACCEPTED", "EXECUTING"
        }:
            self.get_logger().info(
                "Waiting for AutoCharging goal to be accepted before cancelling..."
            )
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
            self.get_logger().info("No accepted AutoCharging goal to cancel.")
            return False

        try:
            current_task_id = self._auto_charging_task_id
            self._auto_charging_status = "CANCELING"
            self._auto_charging_cancel_future = (
                self._auto_charging_goal_handle.cancel_goal_async())
            self._auto_charging_cancel_future.add_done_callback(
                lambda future: self._auto_charging_cancel_done_callback(
                    future, current_task_id))
            self.get_logger().info("AutoCharging cancel request sent.")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to cancel AutoCharging goal: {e}")
            return False

    def _auto_charging_goal_done_callback(self, future, task_id):
        if task_id != self._auto_charging_task_id:
            self.get_logger().debug(
                f"Ignoring goal response for old AutoCharging task {task_id} "
                f"(current: {self._auto_charging_task_id})")
            return

        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"AutoCharging goal send failed: {e}")
            self._auto_charging_status = "FAILED_TO_SEND"
            self._auto_charging_done = True
            self._auto_charging_success = False
            return

        if not goal_handle.accepted:
            self.get_logger().error("AutoCharging goal was rejected.")
            self._auto_charging_status = "REJECTED"
            self._auto_charging_done = True
            self._auto_charging_success = False
            return

        self._auto_charging_goal_handle = goal_handle
        self._auto_charging_status = "ACCEPTED"
        self.get_logger().info("AutoCharging goal accepted.")
        self._auto_charging_result_future = goal_handle.get_result_async()
        self._auto_charging_result_future.add_done_callback(
            lambda future: self._auto_charging_result_done_callback(
                future, task_id))

    def _auto_charging_cancel_done_callback(self, future, task_id):
        if task_id != self._auto_charging_task_id:
            self.get_logger().debug(
                f"Ignoring cancel response for old AutoCharging task {task_id} "
                f"(current: {self._auto_charging_task_id})")
            return

        try:
            response = future.result()
            self.get_logger().info(
                f"AutoCharging cancel response: goals_canceling={len(response.goals_canceling)}"
            )
        except Exception as e:
            self.get_logger().error(f"AutoCharging cancel failed: {e}")

    def _auto_charging_result_done_callback(self, future, task_id):
        if task_id != self._auto_charging_task_id:
            self.get_logger().debug(
                f"Ignoring result for old AutoCharging task {task_id} "
                f"(current: {self._auto_charging_task_id})")
            return

        try:
            response = future.result()
            result = response.result
            self._auto_charging_status = self.AUTO_CHARGING_STATUS_NAMES.get(
                response.status, "UNKNOWN")
            self._auto_charging_success = (self._auto_charging_status
                                           == "SUCCEEDED" and
                                           bool(result.success))
            self._last_auto_charging_result_message = result.message
            self._auto_charging_done = True
            self._auto_charging_goal_handle = None

            if self._auto_charging_success:
                self.get_logger().info(
                    f"AutoCharging succeeded: {result.message}")
            else:
                self.get_logger().error(
                    f"AutoCharging finished with status={self._auto_charging_status}: "
                    f"{result.message}")
        except Exception as e:
            self.get_logger().error(f"Auto charging result error: {e}")
            self._auto_charging_status = "UNKNOWN"
            self._auto_charging_success = False
            self._auto_charging_done = True
            self._auto_charging_goal_handle = None

    def get_control_status(self) -> str:
        """
        获取当前缓存的底层控制状态（不触发服务调用）

        Returns:
            str: 控制状态字符串，可能的取值：
                - "IDLE": 空闲，未在控制
                - "CONTROL": 正在执行控制指令
                - "SUCCEEDED": 控制已完成（仅首次查询时返回一次，之后变为 IDLE）
                - "CANCELLING": 正在取消，平滑停止中
                - "CANCELLED": 已取消（仅首次查询时返回一次，之后变为 IDLE）
                - "UNKNOWN": 状态未知（服务未响应或缓存过期）
        """
        self.spin_once()
        if self._last_control_status_success and self._last_control_status_time:
            # 防止返回陈旧状态
            cache_age = time.time() - self._last_control_status_time
            if cache_age <= self._control_status_timeout:
                self._control_status_unknown_logged = False
                return self._last_control_status

        if not self._control_status_unknown_logged:
            cache_age = (
                time.time() - self._last_control_status_time
                if self._last_control_status_time else -1.0
            )
            request_age = (
                time.time() - self._control_status_request_sent_time
                if self._control_status_request_sent_time else -1.0
            )
            self.get_logger().warn(
                "[底盘控制状态不可用] 返回 UNKNOWN"
                f"状态缓存。cached_status={self._last_control_status}, "
                f"cache_age={cache_age:.3f}s, "
                f"request_in_flight={self._control_status_requesting}, "
                f"request_id={self._control_status_request_id}, "
                f"request_age={request_age:.3f}s。"
                "UNKNOWN 只表示状态查询结果不可用，不代表底盘已停止、空闲或控制失败。"
            )
            self._control_status_unknown_logged = True

        return "UNKNOWN"

    def refresh_control_status(self) -> bool:
        """
        主动向底层控制节点请求一次状态（异步，带 in-flight 防抖和超时恢复）
        Returns:
            bool: 请求是否成功发出
        """
        timed_out_future = None
        timed_out_request_id = None
        consecutive_timeouts = 0

        # 防止请求堆积；旧请求超时后主动清理，避免服务重启后永久卡死。
        with self._control_status_request_lock:
            if self._control_status_requesting:
                self._control_status_skip_count += 1
                sent_age = (
                    time.time() - self._control_status_request_sent_time
                    if self._control_status_request_sent_time else -1.0
                )
                if sent_age >= self._control_status_request_timeout:
                    timed_out_future = self._control_status_request_future
                    timed_out_request_id = \
                        self._control_status_active_request_id
                    self._control_status_requesting = False
                    self._control_status_active_request_id = None
                    self._control_status_request_future = None
                    self._control_status_consecutive_timeouts += 1
                    self._control_status_recovery_pending = True
                    if not self._chassis_connection_recovery_pending:
                        self._chassis_feedback_received_during_recovery = False
                    self._chassis_connection_recovery_pending = True
                    consecutive_timeouts = \
                        self._control_status_consecutive_timeouts
                else:
                    timed_out_request_id = None

        if timed_out_request_id is None and self._control_status_requesting:
            # 第一次立即提示，之后每 20 次（约 10 秒）提示一次，避免每 0.5 秒刷屏。
            if self._control_status_skip_count == 1 or \
                    self._control_status_skip_count % 20 == 0:
                self.get_logger().warn(
                    "[底盘控制状态查询未完成] "
                    f"request_id={self._control_status_request_id} 对 "
                    f"/control/get_control_status 的请求已等待 {sent_age:.3f}s，"
                    f"本次定时查询未发送（连续跳过 "
                    f"{self._control_status_skip_count} 次）。"
                    f"本次实时查询尚未完成；状态缓存超过 "
                    f"{self._control_status_timeout:.1f}s 后，上层将只能得到 "
                    "UNKNOWN。可能原因包括服务端未响应、服务端重启或 "
                    "DDS 通信异常。"
                )
            return False

        if timed_out_request_id is not None:
            if timed_out_future is not None:
                try:
                    self._get_status_client.remove_pending_request(
                        timed_out_future)
                except Exception as e:
                    self.get_logger().error(
                        "[底盘控制状态旧请求清理异常] "
                        f"request_id={timed_out_request_id}, error={e}")
            if consecutive_timeouts == 1 or consecutive_timeouts % 5 == 0:
                self.get_logger().warn(
                    "[底盘控制状态查询超时，启动自动恢复] "
                    f"request_id={timed_out_request_id} 等待超过 "
                    f"{self._control_status_request_timeout:.1f}s，已清理旧请求并"
                    f"释放防抖状态（连续超时 {consecutive_timeouts} 次）；"
                    "现在重新检查 /control/get_control_status，服务恢复后将"
                    "自动发送新请求。"
                )

        if not hasattr(self, '_get_status_client') or \
        not self._get_status_client.wait_for_service(timeout_sec=0.2):
            if not self._control_status_service_unavailable_logged:
                self.get_logger().warn(
                    "[底盘控制状态服务不可用] 找不到 "
                    "/control/get_control_status，本轮未发送查询请求；"
                )
                self._control_status_service_unavailable_logged = True
            with self._control_status_request_lock:
                self._control_status_recovery_pending = True
                if not self._chassis_connection_recovery_pending:
                    self._chassis_feedback_received_during_recovery = False
                self._chassis_connection_recovery_pending = True
            return False
        self._control_status_service_unavailable_logged = False

        request = Trigger.Request()
        request_id = None
        try:
            with self._control_status_request_lock:
                self._control_status_request_id += 1
                request_id = self._control_status_request_id
                self._control_status_active_request_id = request_id
                self._control_status_requesting = True
                self._control_status_request_sent_time = time.time()
                self._control_status_skip_count = 0

            future = self._get_status_client.call_async(request)
            with self._control_status_request_lock:
                self._control_status_request_future = future

            future.add_done_callback(
                lambda completed_future, completed_request_id=request_id:
                self._on_get_control_status_response(
                    completed_future, completed_request_id))
            return True
        except Exception as e:
            with self._control_status_request_lock:
                if request_id == self._control_status_active_request_id:
                    self._control_status_requesting = False
                    self._control_status_active_request_id = None
                    self._control_status_request_future = None
                    self._control_status_recovery_pending = True
                    if not self._chassis_connection_recovery_pending:
                        self._chassis_feedback_received_during_recovery = False
                    self._chassis_connection_recovery_pending = True
            self.get_logger().error(
                "[底盘控制状态请求发送失败] "
                f"request_id={request_id}, error={e}")
            return False

    def cancel_chassis_control(self) -> bool:
        """请求取消当前的控制任务。

        返回:
            bool: 如果服务调用成功则为 True。
        """
        if not hasattr(
                self, '_cancel_control_client'
        ) or not self._cancel_control_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Cancel control service not available.")
            return False

        # 非阻塞调用：发送请求后立即返回，结果由回调处理
        request = Trigger.Request()
        try:
            future = self._cancel_control_client.call_async(request)
            future.add_done_callback(self._on_cancel_control_response)
            self.get_logger().info("Cancel control request sent.")
            return True
        except Exception as e:
            self.get_logger().error(
                f"Failed to send cancel control request: {e}")
            return False

    def set_chassis_control_position(self, position) -> bool:
        """发送一个底盘控制目标点，精度较高。

        Args:
            position (list/np.ndarray): 目标位置 [x, 0, yaw]，yaw单位为度。
                注意：x和yaw不能同时非零。

        Returns:
            bool: 如果目标发送成功则为 True。
        """
        chassis_position = PoseStamped()
        chassis_position.header.frame_id = "base_link"
        chassis_position.pose.position.x = position[0]
        chassis_position.pose.position.y = 0.0
        chassis_position.pose.position.z = 0.0

        rotation = R.from_euler('xyz', [0, 0, position[2]], degrees=True)
        quat_arry = rotation.as_quat()

        chassis_position.pose.orientation.x = quat_arry[0]
        chassis_position.pose.orientation.y = quat_arry[1]
        chassis_position.pose.orientation.z = quat_arry[2]
        chassis_position.pose.orientation.w = quat_arry[3]
        if position[0] != 0.0 and position[2] != 0:
            self.get_logger().info(
                "illegal position, x and yaw should not be set together.")
            return False

        self._set_ctrl_publisher.publish(chassis_position)
        return True

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
        """Callback for navigation goal response.
        """
        # 检查是否是当前任务
        if task_id != self._nav_task_id:
            self.get_logger().warning(
                f"Ignoring goal response for old task {task_id} (current: {self._nav_task_id})"
            )
            return

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected.")
            self._nav_status = "FAILED"
            return

        self.get_logger().info("Goal accepted.")
        self._nav_goal_handle = goal_handle

        # 异步等待结果
        self._nav_result_future = goal_handle.get_result_async()
        self._nav_result_future.add_done_callback(
            lambda future: self._on_nav_result(future, task_id))

    def _on_nav_result(self, future, task_id):
        """Callback for navigation result.
        """
        # 检查是否是当前任务
        if task_id != self._nav_task_id:
            self.get_logger().warning(
                f"Ignoring nav result for old task {task_id} (current: {self._nav_task_id})"
            )
            return

        try:
            result = future.result()
            self._nav_result = result

            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self._nav_status = "SUCCEEDED"
                self.get_logger().info("Goal reached successfully.")
            elif result.status == GoalStatus.STATUS_CANCELED:
                self._nav_status = "CANCELLED"
                self.get_logger().info("Goal was cancelled.")
            else:
                self._nav_status = "FAILED"
                self.get_logger().error(
                    f"Goal failed with status {result.status}")
            self._nav_goal_handle = None
        except Exception as e:
            # future.result() 抛异常（如 action server 崩溃、通信中断等），
            # 导航确实失败了，归入 FAILED 而非单独的 ERROR，
            # 避免上层 wait_for_navigation_arrival 不认识 "ERROR" 而卡到超时
            self._nav_status = "FAILED"
            self._nav_goal_handle = None
            self.get_logger().error(
                f"Error getting navigation result: {str(e)}")

    def _on_nav_cancel_response(self, future, task_id):
        """Callback for navigation cancel response.
        """
        # 检查是否是当前任务
        if task_id != self._nav_task_id:
            self.get_logger().warning(
                f"Ignoring cancel response for old task {task_id} (current: {self._nav_task_id})"
            )
            return

        try:
            _ = future.result()
            self._nav_status = "CANCELLED"
            self._nav_goal_handle = None
            self.get_logger().info("Navigation goal cancelled.")
        except Exception as e:
            self.get_logger().error(f"Error in cancel response: {str(e)}")

    def _on_cancel_control_response(self, future):
        """Callback for cancel control service response (non-blocking)."""
        try:
            response = future.result()
            self.get_logger().info(
                f"Cancel control response: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Error in cancel control response: {e}")

    def _on_get_control_status_response(self, future, request_id):
        """Callback for get_control status async response (non-blocking)."""
        with self._control_status_request_lock:
            if request_id != self._control_status_active_request_id:
                self.get_logger().debug(
                    "[底盘控制状态响应已忽略] "
                    f"request_id={request_id} 已过期，当前活动请求为 "
                    f"{self._control_status_active_request_id}。")
                return

            self._control_status_requesting = False
            self._control_status_active_request_id = None
            self._control_status_request_future = None
            request_sent_time = self._control_status_request_sent_time
            previous_timeouts = self._control_status_consecutive_timeouts
            recovery_pending = self._control_status_recovery_pending
            chassis_connection_recovery_pending = \
                self._chassis_connection_recovery_pending
            feedback_received_during_recovery = \
                self._chassis_feedback_received_during_recovery

        try:
            response = future.result()
            chassis_feedback_connected = (
                feedback_received_during_recovery and
                self._last_msg_time is not None and
                time.time() - self._last_msg_time <= self._connection_timeout
            )
            with self._control_status_request_lock:
                self._control_status_consecutive_timeouts = 0
                if response.success:
                    self._control_status_recovery_pending = False
                    if chassis_feedback_connected:
                        self._chassis_connection_recovery_pending = False
                        self._chassis_feedback_received_during_recovery = False

            # 诊断：计算请求往返延迟
            rtt = (time.time() - request_sent_time
                   ) if request_sent_time else -1.0

            # 记录时间戳
            self._last_control_status_time = time.time()

            # 直接缓存（假设底层返回的就是合法状态）
            old_status = self._last_control_status
            self._last_control_status = response.message
            self._last_control_status_success = bool(response.success)
            self._control_status_unknown_logged = False

            if old_status != self._last_control_status:
                self.get_logger().info(
                    f"[底盘控制状态更新] request_id="
                    f"{request_id}, "
                    f"{old_status} -> {self._last_control_status}, "
                    f"success={response.success}, rtt={rtt:.3f}s"
                )
            elif not response.success:
                self.get_logger().warn(
                    f"[底盘控制状态查询失败] request_id="
                    f"{request_id}, "
                    f"服务返回 success=False, message={response.message}, "
                    f"rtt={rtt:.3f}s"
                )

            if response.success and recovery_pending:
                recovery_detail = (
                    f"期间请求连续超时 {previous_timeouts} 次"
                    if previous_timeouts > 0 else
                    "期间检测到状态服务不可用"
                )
                self.get_logger().info(
                    "[底盘控制状态服务恢复] "
                    f"request_id={request_id} 已重新取得有效状态 "
                    f"status={response.message}，{recovery_detail}，"
                    f"rtt={rtt:.3f}s，"
                    f"chassis_feedback_connected="
                    f"{chassis_feedback_connected}。"
                    "此日志只表示 /control/get_control_status 已恢复，"
                    "不代表底盘客户端已经可以发送运动指令。")

            if response.success and chassis_connection_recovery_pending:
                if chassis_feedback_connected:
                    self.get_logger().info(
                        "[底盘客户端连接恢复] 状态服务和 "
                        "/feedback/chassis_pose 均已恢复；W1Manager 的底盘"
                        "连接检查现在可以通过。")
                elif recovery_pending:
                    self.get_logger().warn(
                        "[底盘客户端尚未就绪] 状态服务已经恢复，但最近 "
                        f"{self._connection_timeout:.1f}s 内仍未收到 "
                        "/feedback/chassis_pose；此时运动指令会被拒绝，"
                        "请继续等待底盘位姿反馈恢复。")
        except Exception as e:
            self._last_control_status_success = False
            with self._control_status_request_lock:
                self._control_status_recovery_pending = True
                if not self._chassis_connection_recovery_pending:
                    self._chassis_feedback_received_during_recovery = False
                self._chassis_connection_recovery_pending = True
            self.get_logger().error(
                f"[底盘控制状态响应异常] request_id="
                f"{request_id}, error={e}")

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
