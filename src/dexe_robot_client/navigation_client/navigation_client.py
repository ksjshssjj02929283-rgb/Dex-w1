import time
import threading
import numpy as np
from abc import ABC, abstractmethod
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from scipy.spatial.transform import Rotation as R
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger

from ..common import get_logger


class NavigationClient(ABC, Node):

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
        self._set_chassis_charging_client = None
        self._cancel_control_client = None
        self._battery_percentage = -1.0

        self._nav_goal_handle = None
        self._nav_result_future = None
        self._nav_status = "IDLE"  # 或 NAVIGATING / SUCCEEDED / FAILED / CANCELLED
        self._nav_result = None  # 存储最后的导航结果

        self._init_subscriber(is_sim)
        self._init_publisher(is_sim)
        self.spin_once(timeout_sec=timeout_sec)

        self.create_timer(0.1, self._check_connection_status)

    def _check_connection_status(self):
        """定时检查消息是否超时"""
        if self._last_msg_time is not None:
            cur_time = self.get_clock().now().nanoseconds / 1e9
            if (cur_time - self._last_msg_time) > self._connection_timeout:
                self._cur_pos = None
                self._last_msg_time = None

    def is_connected(self, timeout_sec: float = 0.1):
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
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

    def _battery_feedback_callback(self, msg):
        '''解析电池状态消息'''
        for status in msg.status:
            if status.name == "/Devices/System/PowerBoard/collector_node: PowerBoard":
                for key_value in status.values:
                    if key_value.key == "battery_percentage":
                        self._battery_percentage = float(key_value.value)
                        break

    def get_battery_percentage(self) -> tuple[bool, float]:
        """获取当前电池电量百分比。

        Returns:
            float: 电池电量百分比，范围为 0.0 到 100.0。如果未收到电池状态消息，则返回 -1.0。
        """
        if (self._battery_percentage < 0):
            return False, -1.0
        else:
            return True, self._battery_percentage

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

        # 状态置为导航中
        self._nav_status = "NAVIGATING"
        self._nav_result = None

        # 异步发送
        send_future = self._set_nav_client.send_goal_async(nav2_position)
        send_future.add_done_callback(self._on_nav_goal_response)

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
        # 触发 ROS2 回调（非常重要）
        self.spin_once()

        return self._nav_status

    # def is_nav_done(self) -> bool:
    #     """检查导航是否结束（无论成功、失败还是取消）。"""
    #     return self.get_nav_state() in ["SUCCEEDED", "FAILED", "CANCELLED"]

    def cancel_chassis_nav2(self) -> bool:
        """取消当前的导航任务 (非阻塞)。"""
        if self._nav_goal_handle is None:
            self.get_logger().warning("No active navigation goal to cancel.")
            return False
        try:
            cancel_future = self._nav_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._on_nav_cancel_response)
            self.get_logger().info("Navigation cancel request sent.")
            return True
        except Exception as e:
            self.get_logger().error(
                f"Error cancelling navigation goal: {str(e)}")
            return False

    def set_chassis_auto_charging(self) -> bool:
        """调用回充服务"""
        if not self._set_chassis_charging_client.wait_for_service(
                timeout_sec=1.0):
            return False

        request = Trigger.Request()
        future = self._set_chassis_charging_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.done():
            try:
                response = future.result()
                self.get_logger().info(
                    f"Cancel control response: {response.message}")
                return response.success
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")
                return False
        else:
            self.get_logger().error("Service call timed out.")
            return False

    def get_control_status(self) -> str:
        """获取底层移动控制节点的状态。

        返回:
            str: 状态信息 ("IDLE", "CONTROL", "CANCEL CONTROL") 或错误信息。
        """
        if not hasattr(
                self, '_get_status_client'
        ) or not self._get_status_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Control status service not available.")
            return "SERVICE_NOT_AVAILABLE"

        request = Trigger.Request()
        future = self._get_status_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.done():
            try:
                response = future.result()
                self.get_logger().info(f"Control status: {response.message}")
                return response.message if response.success else "STATUS_QUERY_FAILED"
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")
                return "SERVICE_CALL_FAILED"
        else:
            self.get_logger().error("Service call timed out.")
            return "SERVICE_TIMEOUT"

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

        request = Trigger.Request()
        future = self._cancel_control_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.done():
            try:
                response = future.result()
                self.get_logger().info(
                    f"Cancel control response: {response.message}")
                return response.success
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")
                return False
        else:
            self.get_logger().error("Service call timed out.")
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

    def spin_once(self, timeout_sec: float = 0.1):
        r"""Spin the node once with a timeout of 0.1 sec."""
        rclpy.spin_once(self, timeout_sec=timeout_sec)

    def spin(self):
        r"""Continuously spin the node while rclpy is running."""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def _on_nav_goal_response(self, future):
        """Callback for navigation goal response.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected.")
            self._nav_status = "FAILED"
            return

        self.get_logger().info("Goal accepted.")
        self._nav_goal_handle = goal_handle

        # 异步等待结果
        self._nav_result_future = goal_handle.get_result_async()
        self._nav_result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        """Callback for navigation result.
        """
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
        except Exception as e:
            self._nav_status = "ERROR"
            self.get_logger().error(
                f"Error getting navigation result: {str(e)}")

    def _on_nav_cancel_response(self, future):
        """Callback for navigation cancel response.
        """
        try:
            _ = future.result()
            self._nav_status = "CANCELLED"
            self._nav_goal_handle = None
            self.get_logger().info("Navigation goal cancelled.")
        except Exception as e:
            self.get_logger().error(f"Error in cancel response: {str(e)}")
