# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

from enum import Enum, auto


class W1ArmType(Enum):
    INDUSTRIAL = auto()
    ANTHROPOMORPHIC = auto()


class W1Version(Enum):
    V1 = auto()
    V2 = auto()
