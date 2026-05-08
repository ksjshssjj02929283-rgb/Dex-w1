# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

from dexe_rp.utils.type import EndEffectorType
from .brainco_hand_client import BrainCoHandClient
from .linker_hand_client import LinkerHandClient
from .pgc_gripper_client import PGCGripperClient


def create_end_effector_client(is_left: bool, is_sim: bool,
                               end_effector_type: EndEffectorType):
    client_map = {
        EndEffectorType.BrainCo_Revo1_R: BrainCoHandClient,
        EndEffectorType.Linker_L6: LinkerHandClient,
        EndEffectorType.Linker_L20: None,
        EndEffectorType.DH_PGC_140_50: PGCGripperClient,
        EndEffectorType.DH_AG_160_95: None,
        EndEffectorType.PaXini_Dex_H13: None,
        None: None,
    }

    client_class = client_map.get(end_effector_type)
    if client_class is None:
        raise ValueError(f"{end_effector_type.name} is not supported.")

    return client_class(is_left=is_left, is_sim=is_sim)


create_end_effector_client_real_and_sim = lambda is_left, end_effector_type: (
    create_end_effector_client(
        is_left=is_left, is_sim=True, end_effector_type=end_effector_type),
    create_end_effector_client(
        is_left=is_left, is_sim=False, end_effector_type=end_effector_type))
