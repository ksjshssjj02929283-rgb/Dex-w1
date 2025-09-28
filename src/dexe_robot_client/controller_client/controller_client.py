import time
import threading
import numpy as np
import json
from abc import ABC, abstractmethod
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ..common import get_logger

class ControllerClient(ABC, Node):
    def __init__(self,
                 name = "controller_client",
                 timeout_sec: float = 3.0):
        self.logger = get_logger()
        self.node_name = name
        super().__init__(self.node_name)

        self._traj_lock = threading.Lock()
        self.trajectory_msg = None
        self._traj_last_msg_time = None
        self.is_new_trajectory = False

        self._connection_timeout = 1.

        self._init_subscriber()
        self.spin_once(timeout_sec = 1.)

        self.create_timer(0.5, self._check_connection_status)
    
    def _init_subscriber(self):
        self.create_subscription(
            JointTrajectory,
            "/dexe_robot_client/trajectory",
            self._trajectory_callback,
            rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)
        )

    def _check_connection_status(self):
        """定时检查消息是否超时"""
        if self._traj_last_msg_time is not None:
            cur_time = self.get_clock().now().nanoseconds / 1e9
            if (cur_time - self._traj_last_msg_time) > self._connection_timeout:
                self.trajectory_msg = None
                self.is_new_trajectory = False
                self._traj_last_msg_time = None

    def is_connected(self, timeout_sec: float = 0.5):
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if self._traj_last_msg_time is not None:
                return True
            time.sleep(0.01)
        return False

    def _trajectory_callback(self, msg):
        self.logger.debug(f"Received new trajectory message")
        with self._traj_lock:
            self.trajectory_msg = msg
            self._traj_last_msg_time = time.time()
            self.is_new_trajectory = True

    def get_trajectory_numpy(self):
        with self._traj_lock:
            if self.trajectory_msg is None or self._traj_last_msg_time is None:
                # self.logger.critical(f"{self.node_name} not connected")
                return None

            traj_points = self.trajectory_msg.points
            traj_list = [point.positions for point in traj_points]
            self.trajectory = np.array(traj_list, dtype=np.float32)
            self.is_new_trajectory = False
            
        return self.trajectory
    
    def get_is_new_trajectory(self):
        with self._traj_lock:
            return self.is_new_trajectory
        
    def set_is_new_trajectory(self, is_new: bool):
        self.is_new_trajectory = is_new

    def spin_once(self, timeout_sec: float = 0.1):
        r"""Spin the node once with a timeout of 0.1 sec."""
        rclpy.spin_once(self, timeout_sec= timeout_sec)

    def spin(self):
        r"""Continuously spin the node while rclpy is running."""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec = 0.1)