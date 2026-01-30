# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

from enum import Enum, auto


class W1ArmType(Enum):
    INDUSTRIAL = auto()
    ANTHROPOMORPHIC = auto()

    # 构型_版本
    INDUSTRIAL_V2 = auto()
    INDUSTRIAL_V3 = auto()
    ANTHROPOMORPHIC_V2 = auto()
    ANTHROPOMORPHIC_V3 = auto()


class W1Version(Enum):
    V1 = auto()
    V2 = auto()
