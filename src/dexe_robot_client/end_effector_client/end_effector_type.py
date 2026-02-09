'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

from enum import Enum, auto


class EndEffectorType(Enum):
    PGC_GRIPPER = auto()
    BRAINCO_HAND = auto()
    LINKER_HAND = auto()

    # 品牌_型号
    DH_PGC14050 = auto()
    BRAINCO_REVO1 = auto()
    BRAINCO_REVO2 = auto()
    LINKER_L6 = auto()
