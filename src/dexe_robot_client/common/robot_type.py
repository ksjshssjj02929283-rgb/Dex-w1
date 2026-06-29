'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

from enum import Enum, auto


class W1ArmType(Enum):
    INDUSTRIAL = auto()
    ANTHROPOMORPHIC = auto()


class W1Version(Enum):
    V1 = auto()
    V2 = auto()
    V3 = auto()


class RobotType(Enum):
    W1V02_1 = auto()
    W1V02_2 = auto()
    W1V03_1 = auto()
