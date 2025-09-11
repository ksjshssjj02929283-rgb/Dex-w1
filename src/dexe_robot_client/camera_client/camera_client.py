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
from sensor_msgs_py import point_cloud2

from ..common import get_logger

class CameraClient(ABC, Node):
    def __init__(self,
                 name: str,
                 timeout_sec: float = 3.0):
        self.logger = get_logger()
        self.node_name = name
        super().__init__(self.node_name)

        self._pcd_lock = threading.Lock()
        self.point_cloud = None
        self._last_msg_time = None
        self._connection_timeout = 1.

        self._init_subscriber()
        self.spin_once(timeout_sec = 1.)
        self._init_publisher()

        self.create_timer(0.5, self._check_connection_status)

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
    def _init_publisher(self):
        pass

    def get_pcd_numpy(self):
        with self._pcd_lock:
            if self.point_cloud is None or self._last_msg_time is None:
                self.logger.critical(f"{self.node_name} not connected")
                return False, None
            if time.time() - self._last_msg_time > 0.1:
                self.logger.warning(f"{self.node_name}: data is outdated")
            points = np.array([
                [p[0], p[1], p[2]]
                for p in point_cloud2.read_points(self.point_cloud, field_names=("x", "y", "z"), skip_nans=True)
            ], dtype=np.float32)

        return True, points

    def _point_cloud_callback(self, msg):
        with self._pcd_lock:
            self.point_cloud = msg
            self._last_msg_time = time.time()

    def spin_once(self, timeout_sec: float = 0.1):
        r"""Spin the node once with a timeout of 0.1 sec."""
        rclpy.spin_once(self, timeout_sec= timeout_sec)

    def spin(self):
        r"""Continuously spin the node while rclpy is running."""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec = 0.1)