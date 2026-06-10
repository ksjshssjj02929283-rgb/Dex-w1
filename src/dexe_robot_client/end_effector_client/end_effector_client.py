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
from dataclasses import dataclass
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from end_effector_interfaces.msg import (EEJointControl, EEJointControlMode,
                                         EEFeedback, EEStatus, EEJointState,
                                         EEJointStatus, EETactileState)
from geometry_msgs.msg import Wrench

from dexe_rp.utils.type import EndEffectorType as DexeRP_EEType

from ..common import get_logger
from .end_effector_params import EndEffectorFactory, EndEffectorParams


# Do NOT write in mapping_index.py to avoid circular import
def get_idx_mapping(current_names: Tuple[str, ...],
                    target_names: Tuple[str, ...]) -> Tuple[int, ...]:
    cur_dict = {name: idx for idx, name in enumerate(current_names)}
    cur_2_target_idx = []
    for name in target_names:
        if name not in cur_dict:
            get_logger().warning(f"Name '{name}' not found in current_names")
            continue
        cur_2_target_idx.append(cur_dict[name])
    return cur_2_target_idx


@dataclass(frozen=True)
class EndEffectorInfo:
    name: str
    status: EEStatus

    joint_names: Tuple[str, ...]
    joint_position: Tuple[float, ...]
    joint_velocity: Tuple[float, ...]
    joint_current: Tuple[float, ...]
    joint_temperature: Tuple[float, ...]
    joint_status: Tuple[EEJointStatus, ...]
    joint_error_code: Tuple[int, ...]

    # field_names: Tuple[str, ...]
    # tactile_states: Tuple[Tuple[Tuple[Wrench, ...], ...], ...]  # (field, taxel, wrench)


class EndEffectorClient(Node):

    def __init__(self,
                 ee_type: DexeRP_EEType,
                 is_left: bool,
                 is_sim: bool,
                 timeout_sec: float = 1.0):
        self.node_name = ee_type.name
        self.node_name += "_left" if is_left else "_right"
        self.node_name += "_sim" if is_sim else ""
        super().__init__(self.node_name)

        self.logger = get_logger()
        self.ee_type = ee_type
        self.is_left = is_left
        self.is_sim = is_sim

        self.ee_params: EndEffectorParams = EndEffectorFactory.create(ee_type)

        self._connection_timeout = 1.0
        self._read_lock = threading.Lock()
        self._last_msg_time: float = None
        self._cur_ee_info: EndEffectorInfo = None
        self._joint_names_mapping: List[int] = None

        self._init_subscriber()
        rclpy.spin_once(self, timeout_sec=timeout_sec)
        self._init_publisher()

        # 创建定时器检查连接状态
        self.create_timer(0.1, self._check_connection_status)

    def _check_connection_status(self):
        """定时连接是否超时"""
        cur_time = self.get_clock().now().nanoseconds / 1e9
        with self._read_lock:
            if self._last_msg_time is None:
                return
            if cur_time - self._last_msg_time > self._connection_timeout:
                self._cur_ee_info = None
                self._last_msg_time = None

    def _init_subscriber(self):
        """初始化订阅"""
        topic = f"/feedback_sim/ee" if self.is_sim else f"/feedback/ee"
        topic += f"/left" if self.is_left else f"/right"
        qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)
        self.create_subscription(EEFeedback, topic, self._feedback_callback,
                                 qos)

    def _feedback_callback(self, msg):
        """接收反馈消息，更新当前状态"""
        # if msg.ee_name != self.ee_type.name:
        #     self.logger.error(f"Received message for {msg.ee_name}, expected {self.ee_type.name}")
        #     return
        if len(msg.joint_states) != self.ee_params.dof:
            self.logger.error(
                f"Received joint_states length {len(msg.joint_states)}, expected {self.ee_params.dof}"
            )
            return

        cur_msg_time = self.get_clock().now().nanoseconds / 1e9

        if self._joint_names_mapping is None:
            server_joint_names = [js.name for js in msg.joint_states]
            self._joint_names_mapping = get_idx_mapping(
                server_joint_names, self.ee_params.joint_names)

        server_joint_position = np.array(
            [js.position for js in msg.joint_states])[self._joint_names_mapping]
        server_joint_velocity = np.array(
            [js.velocity for js in msg.joint_states])[self._joint_names_mapping]
        server_joint_current = np.array([js.current for js in msg.joint_states
                                        ])[self._joint_names_mapping]
        server_joint_temperature = np.array([
            js.temperature for js in msg.joint_states
        ])[self._joint_names_mapping]
        server_joint_status = np.array([js.status for js in msg.joint_states
                                       ])[self._joint_names_mapping]
        server_joint_error_code = np.array([
            js.error_code for js in msg.joint_states
        ])[self._joint_names_mapping]

        with self._read_lock:
            self._last_msg_time = cur_msg_time
            self._cur_ee_info = EndEffectorInfo(
                name=msg.ee_name,
                status=msg.status,
                joint_names=self.ee_params.joint_names,
                joint_position=tuple(server_joint_position),
                joint_velocity=tuple(server_joint_velocity),
                joint_current=tuple(server_joint_current),
                joint_temperature=tuple(server_joint_temperature),
                joint_status=tuple(server_joint_status),
                joint_error_code=tuple(server_joint_error_code))

    def _init_publisher(self):
        """初始化发布"""
        topic = f"/control_sim/ee" if self.is_sim else f"/control/ee"
        topic += f"/left" if self.is_left else f"/right"
        qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)
        self._control_publisher = self.create_publisher(EEJointControl, topic,
                                                        qos)

    def _init_service(self):
        """初始化服务（预留）"""
        pass

    def _init_action(self):
        """初始化动作（预留）"""
        pass

    def _is_outdated(self) -> bool:
        """检查当前状态是否过时"""
        if self._last_msg_time is None:
            return True
        cur_time = self.get_clock().now().nanoseconds / 1e9
        if cur_time - self._last_msg_time > 0.1 * 1.1:  # 与server发布频率相关，允许波动10%
            self.logger.warning(f"{self.node_name}: data is outdated.")
            return True
        return False

    def _check_dim(self, input: List[float]) -> bool:
        if len(input) != self.ee_params.dof:
            self.logger.error(f"len({input}) != {self.ee_params.dof}")
            return False
        return True

    def _send_control(self, mode: EEJointControlMode,
                      values: List[float]) -> bool:
        if not self._check_dim(values):
            return False

        if mode not in self.ee_params.supported_modes:
            self.logger.error(f"Unsupported control mode: {mode}")
            return False

        if mode == EEJointControlMode.POSITION:
            values = np.clip(values, self.ee_params.position.lb,
                             self.ee_params.position.ub).tolist()
        elif mode == EEJointControlMode.VELOCITY:
            values = np.clip(values, self.ee_params.velocity.lb,
                             self.ee_params.velocity.ub).tolist()
        elif mode == EEJointControlMode.CURRENT:
            values = np.clip(values, self.ee_params.current.lb,
                             self.ee_params.current.ub).tolist()
        elif mode == EEJointControlMode.FORCE:
            values = np.clip(values, self.ee_params.force.lb,
                             self.ee_params.force.ub).tolist()
        else:
            self.logger.error(f"Unsupported control mode: {mode.name}")
            return False

        msg = EEJointControl()
        msg.mode = mode
        msg.joint_names = self.ee_params.joint_names
        msg.values = [float(x) for x in values]
        self._control_publisher.publish(msg)
        self.logger.debug(
            f"{self.node_name} set mode {mode}: {list(msg.values)}")
        return True

    # ------------------- 对外接口 -------------------
    def is_connected(self, timeout_sec: float = 0.1) -> bool:
        """检查server是否连接成功"""
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if self._last_msg_time is not None:
                return True
            time.sleep(0.005)
        self.logger.error(f"{self.node_name} not connected!")
        return False

    def connect(self, timeout_sec: float = 0.1) -> bool:
        """连接末端执行器server"""
        return self.is_connected(timeout_sec)

    def set_pos(self, pos: List[float]) -> bool:
        """位置控制（非阻塞）"""
        return self._send_control(EEJointControlMode.POSITION, pos)

    def set_vel(self, vel: List[float]) -> bool:
        """速度控制（非阻塞）"""
        return self._send_control(EEJointControlMode.VELOCITY, vel)

    def set_current(self, current: List[float]) -> bool:
        """电流控制（非阻塞）"""
        return self._send_control(EEJointControlMode.CURRENT, current)

    def get_info(self) -> Tuple[bool, Optional[EndEffectorInfo]]:
        with self._read_lock:
            ee_info = self._cur_ee_info
        if ee_info is None:
            return False, None
        self._is_outdated()
        return True, ee_info
