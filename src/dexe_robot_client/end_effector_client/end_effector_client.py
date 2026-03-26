'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

import copy
import time
import threading
import numpy as np
from abc import ABC, abstractmethod
import rclpy
from rclpy.node import Node
from end_effector_interfaces.msg import (EEJointControl, EEJointControlMode)

from ..common import get_logger


class EndEffectorClient(ABC, Node):

    def __init__(self,
                 name: str,
                 dof: int,
                 joint_name: list[str],
                 is_sim: bool = False,
                 timeout_sec: float = 1.0):
        if is_sim:
            self.node_name = name + "_sim"
        else:
            self.node_name = name + "_real"
        super().__init__(self.node_name)

        self.logger = get_logger()

        self.dof = dof
        self.joint_name = joint_name
        self.pos_ub = np.array([100.] * self.dof)
        self.pos_lb = np.array([0.] * self.dof)
        self.vel_ub = np.array([100.] * self.dof)
        self.vel_lb = np.array([-100.] * self.dof)
        self.current_ub = np.array([100.] * self.dof)
        self.current_lb = np.array([-100.] * self.dof)

        self._read_lock = threading.Lock()
        self._cur_msg = None
        self._last_msg_time = None
        self._connection_timeout = 1.

        self._control_publisher = None

        self._init_subscriber(is_sim)
        self.spin_once(timeout_sec=timeout_sec)
        self._init_publisher(is_sim)

        # 创建定时器检查连接状态
        self.create_timer(0.1, self._check_connection_status)

    @abstractmethod
    def _init_subscriber(self, is_sim: bool):
        pass

    @abstractmethod
    def _init_publisher(self, is_sim: bool):
        pass

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
                self._cur_msg = None
                self._last_msg_time = None

    def _is_outdated(self) -> bool:
        if self._last_msg_time is None:
            return True
        cur_time = self.get_clock().now().nanoseconds / 1e9
        if cur_time - self._last_msg_time > 0.1:
            self.logger.warning(f"{self.node_name}: data is outdated.")
            return True
        return False

    def _feedback_callback(self, msg):
        with self._read_lock:
            self._cur_msg = copy.deepcopy(msg)
            self._last_msg_time = self.get_clock().now().nanoseconds / 1e9

        # self.logger.debug(f"{self.node_name}:\n{self._cur_msg.name}"
        #     f"\npos: {self._cur_msg.position.tolist()}"
        #     f"\nvel: {self._cur_msg.velocity.tolist()}"
        #     f"\ncur: {self._cur_msg.current.tolist()}"
        #     f"\nforce: {self._cur_msg.force.tolist()}"
        #     f"\nstate: {self._cur_msg.state.tolist()}"
        # )

    def is_connected(self, timeout_sec: float = 0.1):
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if self._last_msg_time is not None:
                return True
            time.sleep(0.01)
        return False

    # pos in [0,100]
    def set_pos(self, pos: np.ndarray) -> bool:
        if not self._check_dim(np.array(pos)):
            return False

        msg = EEJointControl()
        msg.mode = EEJointControlMode.POSITION
        msg.name = self.joint_name
        msg.value = np.clip(pos, self.pos_lb, self.pos_ub).tolist()
        self._control_publisher.publish(msg)
        self.logger.debug(f"{self.node_name} set pos: {list(msg.value)}")
        return True

    # vel in [-100,100]
    def set_vel(self, vel: np.ndarray) -> bool:
        if not self._check_dim(np.array(vel)):
            return False

        msg = EEJointControl()
        msg.mode = EEJointControlMode.VELOCITY
        msg.name = self.joint_name
        msg.value = np.clip(vel, self.vel_lb, self.vel_ub).tolist()
        self._control_publisher.publish(msg)
        self.logger.debug(f"{self.node_name} set vel: {list(msg.value)}")
        return True

    # current in [-100,100]
    def set_current(self, current: np.ndarray) -> bool:
        if not self._check_dim(np.array(current)):
            return False

        msg = EEJointControl()
        msg.mode = EEJointControlMode.CURRENT
        msg.name = self.joint_name
        msg.value = np.clip(current, self.current_lb, self.current_ub).tolist()
        self._control_publisher.publish(msg)
        self.logger.debug(f"{self.node_name} set current: {list(msg.value)}")
        return True

    def get_pos(self) -> tuple[bool, np.ndarray]:
        if (not self.is_connected(0.01)) or self._cur_msg is None:
            self.logger.error(f"{self.node_name} not connected!")
            return False, None

        self._is_outdated()
        with self._read_lock:
            return True, np.array(self._cur_msg.position)

    def get_vel(self) -> tuple[bool, np.ndarray]:
        if (not self.is_connected(0.01)) or self._cur_msg is None:
            self.logger.error(f"{self.node_name} not connected!")
            return False, None

        self._is_outdated()
        with self._read_lock:
            return True, np.array(self._cur_msg.velocity)

    def get_current(self) -> tuple[bool, np.ndarray]:
        if (not self.is_connected(0.01)) or self._cur_msg is None:
            self.logger.error(f"{self.node_name} not connected!")
            return False, None

        self._is_outdated()
        with self._read_lock:
            return True, np.array(self._cur_msg.current)

    def get_force(self) -> tuple[bool, np.ndarray]:
        if (not self.is_connected(0.01)) or self._cur_msg is None:
            self.logger.error(f"{self.node_name} not connected!")
            return False, None

        self._is_outdated()
        with self._read_lock:
            return True, np.array(self._cur_msg.force)

    def spin_once(self, timeout_sec: float = 0.1):
        r"""Spin the node once with a timeout of 0.1 sec."""
        rclpy.spin_once(self, timeout_sec=timeout_sec)

    def spin(self):
        r"""Continuously spin the node while rclpy is running."""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
