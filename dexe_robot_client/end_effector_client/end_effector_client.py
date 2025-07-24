# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import numpy as np
from abc import ABC, abstractmethod
import rclpy
from rclpy.node import Node

class EndEffectorClient(ABC, Node):
    def __init__(self, name: str, qpos_bound: np.ndarray):
        super().__init__(name)
        self.qpos_lb = qpos_bound[0]
        self.qpos_ub = qpos_bound[1]

    # qpos in qpos_limits
    @abstractmethod
    def set_qpos(self, qpos: np.ndarray) -> bool:
        pass

    @abstractmethod
    def get_qpos(self) -> tuple[bool, np.ndarray]:
        pass

    def _check_input(self, input: np.ndarray) -> bool:
        if input.shape[0] != self.qpos_lb.shape[0]:
            self.get_logger().error(f"len({input}) != {self.qpos_lb.shape[0]}")
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