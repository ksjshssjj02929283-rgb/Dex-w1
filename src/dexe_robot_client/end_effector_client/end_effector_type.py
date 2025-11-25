# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

from enum import Enum, auto


class EndEffectorType(Enum):
    PGC_GRIPPER = auto()
    BRAINCO_HAND = auto()
