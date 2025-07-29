# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import numpy as np
from enum import Enum, auto
from dataclasses import dataclass
from typing import Optional, Union
# from geometry_msgs.msg import Pose
# from sensor_msgs.msg import JointState

"""----------------------------------------------------------------------------
enum for various states and request types
机器人状态类和指令类枚举
----------------------------------------------------------------------------"""


class RunMode(Enum):
    Position = auto()  # position controls
    Impedance = auto()  # impedance control
    Drag = auto()  # zero-stiff control
    Teleoperate = auto()  # teleoperate control


class Status(Enum):
    Init = auto()
    Idle = auto()
    Running = auto()
    Error = auto()


class PropertyType(Enum):
    RunMode = auto()  # run mode

    ClearError = auto()  # clear error

    TcpPrimary = auto()  # tool primary
    TcpSecondary = auto()  # tool secondary

    SpeedRatio = auto()  # speed ratio percentage: 0-100


class MotionType(Enum):
    MoveJoint = auto()
    MoveLine = auto()

    StartJointStreaming = auto()
    StartEEPoseStreaming = auto()


class MotionGroup(Enum):
    Primary = auto()
    Secondary = auto()
    BothArms = auto()
    WholeBody = auto()


class ReferenceFrame(Enum):
    """Move frame for move requests
    移动请求的参考坐标系
    """

    ArmBase = auto()  # arm base frame
    WorldBase = auto()  # world frame


""" ---------------------------------------------------------------------------
robot state and requests data structure class
机器人状态和请求数据结构
----------------------------------------------------------------------------"""


@dataclass
class RobotInfo:
    """robot info 机器人状态集合"""

    status: Status
    joint_position: list[float]
    ee_pose: list[float]


@dataclass
class MoveRequest:
    """move request 移动指令"""

    type: MotionType
    motion_group: MotionGroup
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
