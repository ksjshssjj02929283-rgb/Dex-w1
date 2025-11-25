import time
import threading
import numpy as np
from abc import ABC, abstractmethod
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R

from ..common import get_logger


class NavigationClient(ABC, Node):

    def __init__(self,
                 name: str,
                 is_sim: bool = False,
                 timeout_sec: float = 30.0):
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
        self._set_ctrl_publihser = None
        self._get_pos_subscriber = None

        self._init_subscriber(is_sim)
        self._init_publisher(is_sim)
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

        self.last_chassis_processed_feedback = processed
        self._last_msg_time = time.time()

    def get_chassis_position(self) -> tuple[bool, np.ndarray]:

        self.spin_once()

        if self.last_chassis_processed_feedback is None:
            self.get_logger().error("No chassis feedback received yet.")
            return False, None

        pos = self.last_chassis_processed_feedback["data"]["position"]
        x, y = pos[0], pos[1]

        q = self.last_chassis_processed_feedback["data"]["orientation"]
        quat_array = np.array([q[0], q[1], q[2], q[3]])
        rotation = R.from_quat(quat_array)

        yaw = rotation.as_euler('xyz', degrees=True)[2]
        self._cur_pos = np.array([x, y, yaw])
        return True, self._cur_pos

    def set_chassis_nav2_position(self, position) -> bool:
        """Send a nav2 position, which accuracy is low.
        """
        nav2_position = PoseStamped()
        nav2_position.header.frame_id = "map"
        nav2_position.pose.position.x = position[0]
        nav2_position.pose.position.y = position[1]
        nav2_position.pose.position.z = 0.0

        rotation = R.from_euler('xyz', [0, 0, position[2]], degrees=True)
        quat_arry = rotation.as_quat()

        nav2_position.pose.orientation.x = quat_arry[0]
        nav2_position.pose.orientation.y = quat_arry[1]
        nav2_position.pose.orientation.z = quat_arry[2]
        nav2_position.pose.orientation.w = quat_arry[3]
        self._set_nav_publisher.publish(nav2_position)
        return True

    def set_chassis_control_position(self, position) -> bool:
        """Send a chassis position, which accuracy is high.
        """
        chassis_position = PoseStamped()
        chassis_position.header.frame_id = "map"
        chassis_position.pose.position.x = position[0]
        chassis_position.pose.position.y = position[1]
        chassis_position.pose.position.z = 0.0

        rotation = R.from_euler('xyz', [0, 0, position[2]], degrees=True)
        quat_arry = rotation.as_quat()

        chassis_position.pose.orientation.x = quat_arry[0]
        chassis_position.pose.orientation.y = quat_arry[1]
        chassis_position.pose.orientation.z = quat_arry[2]
        chassis_position.pose.orientation.w = quat_arry[3]

        self._set_ctrl_publihser.publish(chassis_position)
        return True

    def spin_once(self, timeout_sec: float = 0.1):
        r"""Spin the node once with a timeout of 0.1 sec."""
        rclpy.spin_once(self, timeout_sec=timeout_sec)

    def spin(self):
        r"""Continuously spin the node while rclpy is running."""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
