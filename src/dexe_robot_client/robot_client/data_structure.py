'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

import numpy as np
from enum import Enum, auto
from dataclasses import dataclass
from typing import Optional, Union
"""----------------------------------------------------------------------------
enum for various states and request types
机器人状态类和指令类枚举
----------------------------------------------------------------------------"""


class RunMode(Enum):
    Position = auto()  # position controls
    Torque = auto()  # joint impedance control
    Drag = auto()  # zero-stiff control
    CartesianImpedance = auto()  # cartesian impedance control
    DragImpedance = auto()  # drag with impedance control


class Status(Enum):
    Init = auto()
    Idle = auto()
    Running = auto()
    Error = auto()


class PropertyType(Enum):
    TcpPrimary = auto()  # tool primary
    TcpSecondary = auto()  # tool secondary
    SpeedRatio = auto()  # speed ratio percentage: 0-100
    ClearError = auto()  # clear error
    SafeLevel = auto()
    SwitchRunMode = auto()  # run mode
    EnableDrag = auto()
    JointStiffness = auto()  # joint stiffness
    JointDamping = auto()  # joint damping
    CartStiffness = auto()  # cartesian stiffness
    CartDampingRatio = auto()  # cartesian damping ratio 0.7~1.3


class MotionType(Enum):
    MoveJoint = auto()
    MoveLine = auto()
    MoveTraj = auto()

    StartJointStreaming = auto()
    StartEEPoseStreaming = auto()
    StartEEVelocityStreaming = auto()

    StopMove = auto()


class MotionGroup(Enum):
    Primary = auto()
    Secondary = auto()
    BothArms = auto()
    WholeBody = auto()
    Torso = auto()


class ReferenceFrame(Enum):
    """Move frame for move requests
    移动请求的参考坐标系
    """

    ArmBase = auto()  # arm base frame
    WorldBase = auto()  # world frame
    EEFrame = auto()  # end-effector frame


""" ---------------------------------------------------------------------------
robot state and requests data structure class
机器人状态和请求数据结构
----------------------------------------------------------------------------"""


@dataclass
class RobotInfo:
    """robot info 机器人状态集合"""
    status: Status
    robot_version: str
    robot_type: str
    arm_type: str
    robot_statusword: int
    robot_controlword: int
    run_mode: RunMode
    safe_level: float
    global_speed_ratio: float
    joint_stiffness: list[float]
    joint_damping: list[float]
    cart_stiffness: list[float]
    cart_damping_ratio: list[float]
    cart_impedance_force: list[float]

    latest_sent_id: int
    latest_queued_id: int
    latest_finished_id: int
    latest_errordrop_id: int

    joint_position_cmd: list[float]
    joint_position: list[float]
    joint_velocity: list[float]
    joint_torque: list[float]

    motor_status: list[int]
    motor_error_code: list[int]
    motor_temperature: list[int]
    force_sensor: list[float]

    torque_limit_negative: list[float]
    torque_limit_positive: list[float]

    ee_pose: list[float]
    torso_pose_X_Z_RY: list[float]


@dataclass
class MoveRequest:
    """move request 移动指令"""

    type: MotionType
    motion_group: Optional[MotionGroup] = None
    value: Optional[Union[list[float], np.ndarray]] = None  # 目标位置

    reference_frame: Optional[ReferenceFrame] = None  # 参考坐标系
    speed: Optional[float] = None  # 速度百分比 0~1
    kwargs: dict = None  # 其他参数


def value_to_string(value) -> str:
    if value is None:
        return ""
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, (int, float)):
        return str(value)
    if isinstance(value, (list, tuple, np.ndarray)):
        return ",".join(str(float(v)) for v in value)
    if isinstance(value, Enum):
        return value.name()
    return str(value)
