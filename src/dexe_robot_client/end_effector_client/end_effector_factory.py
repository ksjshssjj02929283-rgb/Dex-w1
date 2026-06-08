'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

from dexe_rp.utils.type import EndEffectorType as DexeRP_EEType

from .end_effector_type import EndEffectorType
from .end_effector_client import EndEffectorClient


# TODO: Should be deprecated util use EndEffectorType defined in dexe_rp
def create_end_effector_client(end_effector_type: EndEffectorType,
                               is_left: bool, is_sim: bool):
    if end_effector_type == EndEffectorType.BRAINCO_HAND:
        return EndEffectorClient(ee_type=DexeRP_EEType.BrainCo_Revo1_R,
                                 is_left=is_left,
                                 is_sim=is_sim)
    elif end_effector_type == EndEffectorType.LINKER_HAND:
        return EndEffectorClient(ee_type=DexeRP_EEType.Linker_L6,
                                 is_left=is_left,
                                 is_sim=is_sim)
    elif end_effector_type == EndEffectorType.PGC_GRIPPER:
        return EndEffectorClient(ee_type=DexeRP_EEType.DH_PGC_140_50,
                                 is_left=is_left,
                                 is_sim=is_sim)
    else:
        raise ValueError(f"{end_effector_type.name} are not support.")
