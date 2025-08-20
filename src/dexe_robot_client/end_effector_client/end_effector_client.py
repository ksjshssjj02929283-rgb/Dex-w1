# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import time
import threading
from copy import deepcopy
import numpy as np
from abc import ABC, abstractmethod
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from ..common import get_logger


class EndEffectorClient(ABC, Node):
    def __init__(self, 
                 name: str, 
                 dof: int, 
                 is_sim: bool = False,
                 timeout_sec: float = 3.0):
        self.logger = get_logger()
        if is_sim:
            self.node_name = name + "_sim"
        else:
            self.node_name = name + "_real"
        self.dof = dof
        self.qpos_ub = np.array([100.]*self.dof)
        self.qpos_lb = np.array([0.]*self.dof)
        super().__init__(self.node_name)

        self._qpos_lock = threading.Lock()
        self._cur_qpos = None
        self._last_msg_time = None
        self._connection_timeout = 1.

        self._set_qpos_publisher = None

        self._init_subscriber(is_sim)
        self.spin_once(timeout_sec = 1.)
        self._init_publisher(is_sim)

        # 创建定时器检查连接状态
        self.create_timer(0.1, self._check_connection_status)
        if self.is_connected(timeout_sec):
            self.logger.info(f"{self.node_name} init successed")
        else:
            self.logger.error(f"{self.node_name} init failed: connect to server timeout")
            raise RuntimeError

    def _check_dim(self, input: np.ndarray) -> bool:
        if input.shape[0] != self.dof:
            self.logger.error(f"len({input}) != {self.dof}")
            return False
        return True

    def _check_connection_status(self):
        """定时检查消息是否超时"""
        if self._last_msg_time is not None:
            cur_time = self.get_clock().now().nanoseconds / 1e9
            if (cur_time - self._last_msg_time) > self._connection_timeout:
                self._cur_qpos = None
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

    # qpos in [0-100]
    def set_qpos(self, qpos: np.ndarray) -> bool:
        qpos_ = np.array(qpos)
        if not self._check_dim(qpos_):
            return False
        qpos_clip = np.clip(qpos_, self.qpos_lb, self.qpos_ub)
        msg = Float64MultiArray()
        msg.data = qpos_clip.tolist()
        self._set_qpos_publisher.publish(msg)
        self.logger.debug(f"{self.node_name} published: {qpos_clip}")
        return True

    def get_qpos(self) -> tuple[bool, np.ndarray]:
        # self.spin_once() # FIXME: Only successful if run twice
        with self._qpos_lock:
            if self._cur_qpos is None or \
                self._last_msg_time is None:
                self.logger.critical(f"{self.node_name} not connected!")
                return False, None
            if time.time() - self._last_msg_time > 0.1:
                self.logger.warning(f"{self.node_name}: data is outdated.")
        return True, self._cur_qpos

    def _qpos_callback(self, msg):
        with self._qpos_lock:
            self._cur_qpos = np.array(msg.data)

            self._last_msg_time = time.time()
        # self.logger.debug(f"{self.node_name}: received {self._current_qpos}")
    
    def spin_once(self, timeout_sec: float = 0.1):
        r"""Spin the node once with a timeout of 0.1 sec."""
        rclpy.spin_once(self, timeout_sec= timeout_sec)

    def spin(self):
        r"""Continuously spin the node while rclpy is running."""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec = 0.1)