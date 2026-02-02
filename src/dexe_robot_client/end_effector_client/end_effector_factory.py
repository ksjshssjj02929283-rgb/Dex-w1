'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

from .end_effector_type import EndEffectorType
from .brainco_hand_client import BrainCoHandClient
from .linker_hand_client import LinkerHandClient
from .pgc_gripper_client import PGCGripperClient


def create_end_effector_client(is_left: bool, is_sim: bool,
                               end_effector_type: EndEffectorType):
    if end_effector_type == EndEffectorType.BRAINCO_HAND:
        return BrainCoHandClient(is_left=is_left, is_sim=is_sim)
    elif end_effector_type == EndEffectorType.LINKER_HAND:
        return LinkerHandClient(is_left=is_left, is_sim=is_sim)
    elif end_effector_type == EndEffectorType.PGC_GRIPPER:
        return PGCGripperClient(is_left=is_left, is_sim=is_sim)
    else:
        raise ValueError(f"{end_effector_type.name} are not support.")
