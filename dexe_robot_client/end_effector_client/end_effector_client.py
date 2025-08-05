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
from dexe_robot_client.common import get_logger


class EndEffectorClient(ABC, Node):
    def __init__(self, name: str, qpos_bound: np.ndarray):
        self.logger = get_logger()
        self.node_name = name
        self.qpos_lb = qpos_bound[0]
        self.qpos_ub = qpos_bound[1]
        super().__init__(self.node_name)

        self._qpos_lock = threading.Lock()
        self._cur_qpos = None
        self._last_qpos_time = None

        # _qpos_subscription
        # _set_qpos_publisher

    # qpos in qpos_bound
    @abstractmethod
    def set_qpos(self, qpos: np.ndarray) -> bool:
        pass

    def get_qpos(self) -> tuple[bool, np.ndarray]:
        # self.spin_once() # FIXME: Only successful if run twice
        with self._qpos_lock:
            if self._cur_qpos is None or \
                self._last_qpos_time is None:
                return False, None
            if time.time() - self._last_qpos_time > 0.1:
                return False, None
            temp = deepcopy(self._cur_qpos)
            self._cur_qpos = None
            self._last_qpos_time = None
        return True, temp

    def _qpos_callback(self, msg):
        with self._qpos_lock:
            self._cur_qpos = np.array(msg.data)
            self._last_qpos_time = time.time()
        # self.logger.debug(f"{self.node_name}: received {self._current_qpos}")

    def _check_input(self, input: np.ndarray) -> bool:
        if input.shape[0] != self.qpos_lb.shape[0]:
            self.logger.error(f"len({input}) != {self.qpos_lb.shape[0]}")
            return False
        return True

    # transfer qpos to state in [0,1]
    def set_state(self, state: np.ndarray) -> bool:
        if not self._check_input(state):
            return False
        qpos = self.qpos_lb + state * (self.qpos_ub - self.qpos_lb)
        return self.set_qpos(qpos)

    def get_state(self) -> tuple[bool, np.ndarray]:
        res, qpos = self.get_qpos()
        if not res:
            self.logger.error(f"{self.node_name}: get state failed")
            return False, None
        state = qpos / (self.qpos_ub - self.qpos_lb)
        return True, state
    
    def spin_once(self):
        r"""Spin the node once with a timeout of 0.1 sec."""
        rclpy.spin_once(self, timeout_sec=0.1)

    def spin(self):
        r"""Continuously spin the node while rclpy is running."""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)