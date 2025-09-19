# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import time
import threading
import numpy as np
from abc import ABC, abstractmethod
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ..common import get_logger

class TrajectoryClient(ABC, Node):
    def __init__(self,
                 name,
                timeout_sec: float = 3.0):
        self.logger = get_logger()
        self.node_name = name
        super().__init__(self.node_name)

        self._traj_lock = threading.Lock()
        self.trajectory_msg = None
        self._last_msg_time = None
        self.is_new_trajectory = False
        self.trajectory_publisher = None
        self._connection_timeout = 1.

        self.streaming_position = None
        self.last_streaming_position = None
        self.streaming_vel = None
        self.is_first_streaming = True
        self._streaming_cmd_lock = threading.Lock()

        self.streaming_dt = 0.01  # 100Hz

        self._init_subscriber()
        self._init_streaming_subscriber()
        self.spin_once(timeout_sec = 1.)
        self._init_publisher()

        self.create_timer(0.5, self._check_connection_status)

        self.received_cnt = 0

    # # FIXME: modify the topic name and queue size if needed
    # def _init_publisher(self):
    #     self.trajectory_publisher = self.create_publisher(
    #         JointTrajectory, '/dexe_robot_client/trajectory', 1
    #     )

    def _check_connection_status(self):
        """定时检查消息是否超时"""
        if self._last_msg_time is not None:
            cur_time = self.get_clock().now().nanoseconds / 1e9
            if (cur_time - self._last_msg_time) > self._connection_timeout:
                self._cur_qpos = None
                self._last_msg_time = None

    def is_connected(self, timeout_sec: float = 0.5):
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if self._last_msg_time is not None:
                return True
            time.sleep(0.01)
        return False

    @abstractmethod
    def _init_subscriber(self):
        pass
    
    @abstractmethod
    def _init_streaming_subscriber(self):
        pass

    @abstractmethod
    def _init_publisher(self):
        pass

    def publish_trajectory(self, trajectory: np.ndarray):
        """
        This function will publish the trajectory to the robot server and update self.trajectory

        Args:
            trajectory: np.ndarray of shape (N, 20), N is the number of waypoints
        Returns:
            bool: True if success, False otherwise
        """
        # self.trajectory = trajectory
        # self.is_new_trajectory = True

        if self.trajectory_publisher is None:
            self.logger.error("Publisher not initialized")
            return False
        
        if trajectory is None:
            self.logger.info("Publishing empty trajectory")
            traj_msg = JointTrajectory()
            traj_msg.joint_names = [
                "ANKLE", "KNEE", "BUTTOCK", "WAIST",
                "NEKC1", "NECK2",
                "LEFT_J1", "LEFT_J2", "LEFT_J3", "LEFT_J4", "LEFT_J5", "LEFT_J6", "LEFT_J7",
                "RIGHT_J1", "RIGHT_J2", "RIGHT_J3", "RIGHT_J4", "RIGHT_J5", "RIGHT_J6", "RIGHT_J7"
            ]
            self.trajectory_publisher.publish(traj_msg)
            return True
        
        if trajectory.shape[1] != 20:
            self.logger.error("Invalid trajectory shape")
            return False
        

        
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [
            "ANKLE", "KNEE", "BUTTOCK", "WAIST",
            "NEKC1", "NECK2",
            "LEFT_J1", "LEFT_J2", "LEFT_J3", "LEFT_J4", "LEFT_J5", "LEFT_J6", "LEFT_J7",
            "RIGHT_J1", "RIGHT_J2", "RIGHT_J3", "RIGHT_J4", "RIGHT_J5", "RIGHT_J6", "RIGHT_J7"
        ]
        for i in range(trajectory.shape[0]):
            point = JointTrajectoryPoint()
            point.positions = trajectory[i].tolist()
            point.time_from_start = rclpy.duration.Duration(seconds=0.01 * (i + 1)).to_msg()
            traj_msg.points.append(point)

        self.trajectory_publisher.publish(traj_msg)
        self.logger.debug(f"Publish new trajectory with {len(traj_msg.points)} points")

    def _trajectory_callback(self, msg):
        # self.logger.debug(f"Receive new trajectory with {len(msg.points)} points")
        with self._traj_lock:
            self.trajectory_msg = msg
            self._last_msg_time = time.time()
            self.is_new_trajectory = True
            
    def _streaming_cmd_callback(self, msg):
        with self._streaming_cmd_lock:
            if self.is_first_streaming:
                self.is_first_streaming = False
                self.streaming_position = msg.position
                self.last_streaming_position = msg.position
                self.streaming_vel = np.zeros(len(msg.position))
                if msg.streaming_finished:
                    self.is_first_streaming = True
            else:
                self.streaming_position = msg.position
                self.streaming_vel = (np.array(self.streaming_position) - np.array(self.last_streaming_position)) / self.streaming_dt
                self.last_streaming_position = msg.position
                if msg.streaming_finished:
                    self.is_first_streaming = True

    def get_streaming_cmd(self):
        with self._streaming_cmd_lock:
            if self.is_first_streaming:
                return None, np.zeros(20)
            return self.streaming_position, self.streaming_vel


    def get_trajectory_numpy(self):
        with self._traj_lock:
            if self.trajectory_msg is None or self._last_msg_time is None:
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