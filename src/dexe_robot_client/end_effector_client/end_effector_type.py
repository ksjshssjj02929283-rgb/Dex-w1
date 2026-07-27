'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

from enum import Enum, auto


# TODO: Should be deprecated, and use EndEffectorType defined in dexe_rp
class EndEffectorType(Enum):
    PGC_GRIPPER = auto()
    BRAINCO_HAND = auto()
    LINKER_HAND = auto()
    HITBOT_SUCTION = auto()


class DexeRP_EETypeFake(str, Enum):
    HITBOT_Z_ESC_70 = "hitbot_z_esc_70"
