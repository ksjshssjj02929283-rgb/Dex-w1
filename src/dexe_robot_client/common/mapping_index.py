'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

from enum import Enum, auto
from typing import List, Tuple

import pinocchio

from .logger import get_logger
from .robot_type import RobotType
from ..end_effector_client import EndEffectorType


class RealRobotIdx(Enum):
    ANKLE = auto()
    KNEE = auto()
    BUTTOCK = auto()
    WAIST = auto()
    NECK1 = auto()
    NECK2 = auto()
    LEFT_J1 = auto()
    LEFT_J2 = auto()
    LEFT_J3 = auto()
    LEFT_J4 = auto()
    LEFT_J5 = auto()
    LEFT_J6 = auto()
    LEFT_J7 = auto()
    RIGHT_J1 = auto()
    RIGHT_J2 = auto()
    RIGHT_J3 = auto()
    RIGHT_J4 = auto()
    RIGHT_J5 = auto()
    RIGHT_J6 = auto()
    RIGHT_J7 = auto()


class RealRobotIdx030(Enum):
    ANKLE_PITCH = auto()
    KNEE_PITCH = auto()
    BUTTOCK_PITCH = auto()
    WAIST_YAW = auto()
    HEAD_YAW_J1 = auto()
    HEAD_PITCH_J2 = auto()
    SHOULDER_PITCH_L_J1 = auto()
    SHOULDER_ROLL_L_J2 = auto()
    ELBOW_PITCH_L_J3 = auto()
    ELBOW_YAW_L_J4 = auto()
    WRIST_PITCH_L_J5 = auto()
    WRIST_YAW_L_J6 = auto()
    WRIST_ROLL_L_J7 = auto()
    SHOULDER_PITCH_R_J1 = auto()
    SHOULDER_ROLL_R_J2 = auto()
    ELBOW_PITCH_R_J3 = auto()
    ELBOW_YAW_R_J4 = auto()
    WRIST_PITCH_R_J5 = auto()
    WRIST_YAW_R_J6 = auto()
    WRIST_ROLL_R_J7 = auto()


class RealRobotIdx_Brainco(Enum):
    ANKLE = auto()
    KNEE = auto()
    BUTTOCK = auto()
    WAIST = auto()
    NECK1 = auto()
    NECK2 = auto()
    LEFT_J1 = auto()
    LEFT_J2 = auto()
    LEFT_J3 = auto()
    LEFT_J4 = auto()
    LEFT_J5 = auto()
    LEFT_J6 = auto()
    LEFT_J7 = auto()
    RIGHT_J1 = auto()
    RIGHT_J2 = auto()
    RIGHT_J3 = auto()
    RIGHT_J4 = auto()
    RIGHT_J5 = auto()
    RIGHT_J6 = auto()
    RIGHT_J7 = auto()
    LEFT_THUMBCMC = auto()
    LEFT_THUMBMCP = auto()
    LEFT_INDEXMCP = auto()
    LEFT_INDEXPIP = auto()
    LEFT_MIDDLEMCP = auto()
    LEFT_MIDDLEPIP = auto()
    LEFT_RINGMCP = auto()
    LEFT_RINGPIP = auto()
    LEFT_LITTLEMCP = auto()
    LEFT_LITTLEPIP = auto()
    RIGHT_THUMBCMC = auto()
    RIGHT_THUMBMCP = auto()
    RIGHT_INDEXMCP = auto()
    RIGHT_INDEXPIP = auto()
    RIGHT_MIDDLEMCP = auto()
    RIGHT_MIDDLEPIP = auto()
    RIGHT_RINGMCP = auto()
    RIGHT_RINGPIP = auto()
    RIGHT_LITTLEMCP = auto()
    RIGHT_LITTLEPIP = auto()


class RealRobotIdx030_Brainco(Enum):
    ANKLE_PITCH = auto()
    KNEE_PITCH = auto()
    BUTTOCK_PITCH = auto()
    WAIST_YAW = auto()
    HEAD_YAW_J1 = auto()
    HEAD_PITCH_J2 = auto()
    SHOULDER_PITCH_L_J1 = auto()
    SHOULDER_ROLL_L_J2 = auto()
    ELBOW_PITCH_L_J3 = auto()
    ELBOW_YAW_L_J4 = auto()
    WRIST_PITCH_L_J5 = auto()
    WRIST_YAW_L_J6 = auto()
    WRIST_ROLL_L_J7 = auto()
    SHOULDER_PITCH_R_J1 = auto()
    SHOULDER_ROLL_R_J2 = auto()
    ELBOW_PITCH_R_J3 = auto()
    ELBOW_YAW_R_J4 = auto()
    WRIST_PITCH_R_J5 = auto()
    WRIST_YAW_R_J6 = auto()
    WRIST_ROLL_R_J7 = auto()
    LEFT_T_CMC_YAW = auto()
    LEFT_T_MCP = auto()
    LEFT_IF_MCP_PITCH = auto()
    LEFT_IF_DIP = auto()
    LEFT_MF_MCP_PITCH = auto()
    LEFT_MF_DIP = auto()
    LEFT_RF_MCP_PITCH = auto()
    LEFT_RF_DIP = auto()
    LEFT_LF_MCP_PITCH = auto()
    LEFT_LF_DIP = auto()
    RIGHT_T_CMC_YAW = auto()
    RIGHT_T_MCP = auto()
    RIGHT_IF_MCP_PITCH = auto()
    RIGHT_IF_DIP = auto()
    RIGHT_MF_MCP_PITCH = auto()
    RIGHT_MF_DIP = auto()
    RIGHT_RF_MCP_PITCH = auto()
    RIGHT_RF_DIP = auto()
    RIGHT_LF_MCP_PITCH = auto()
    RIGHT_LF_DIP = auto()


class RealRobotIdx_PGC(Enum):
    ANKLE = auto()
    KNEE = auto()
    BUTTOCK = auto()
    WAIST = auto()
    NECK1 = auto()
    NECK2 = auto()
    LEFT_J1 = auto()
    LEFT_J2 = auto()
    LEFT_J3 = auto()
    LEFT_J4 = auto()
    LEFT_J5 = auto()
    LEFT_J6 = auto()
    LEFT_J7 = auto()
    RIGHT_J1 = auto()
    RIGHT_J2 = auto()
    RIGHT_J3 = auto()
    RIGHT_J4 = auto()
    RIGHT_J5 = auto()
    RIGHT_J6 = auto()
    RIGHT_J7 = auto()
    LEFT_FINGER1 = auto()
    LEFT_FINGER2 = auto()
    RIGHT_FINGER1 = auto()
    RIGHT_FINGER2 = auto()


class RealRobotIdx030_PGC(Enum):
    ANKLE_PITCH = auto()
    KNEE_PITCH = auto()
    BUTTOCK_PITCH = auto()
    WAIST_YAW = auto()
    HEAD_YAW_J1 = auto()
    HEAD_PITCH_J2 = auto()
    SHOULDER_PITCH_L_J1 = auto()
    SHOULDER_ROLL_L_J2 = auto()
    ELBOW_PITCH_L_J3 = auto()
    ELBOW_YAW_L_J4 = auto()
    WRIST_PITCH_L_J5 = auto()
    WRIST_YAW_L_J6 = auto()
    WRIST_ROLL_L_J7 = auto()
    SHOULDER_PITCH_R_J1 = auto()
    SHOULDER_ROLL_R_J2 = auto()
    ELBOW_PITCH_R_J3 = auto()
    ELBOW_YAW_R_J4 = auto()
    WRIST_PITCH_R_J5 = auto()
    WRIST_YAW_R_J6 = auto()
    WRIST_ROLL_R_J7 = auto()
    LEFT_FINGER_1 = auto()
    LEFT_FINGER_2 = auto()
    RIGHT_FINGER_1 = auto()
    RIGHT_FINGER_2 = auto()


class RealRobotIdx_Linker(Enum):
    ANKLE = auto()
    KNEE = auto()
    BUTTOCK = auto()
    WAIST = auto()
    NECK1 = auto()
    NECK2 = auto()
    LEFT_J1 = auto()
    LEFT_J2 = auto()
    LEFT_J3 = auto()
    LEFT_J4 = auto()
    LEFT_J5 = auto()
    LEFT_J6 = auto()
    LEFT_J7 = auto()
    RIGHT_J1 = auto()
    RIGHT_J2 = auto()
    RIGHT_J3 = auto()
    RIGHT_J4 = auto()
    RIGHT_J5 = auto()
    RIGHT_J6 = auto()
    RIGHT_J7 = auto()
    LEFT_THUMBCMC = auto()
    LEFT_THUMBMCP = auto()
    LEFT_THUMBPIP = auto()
    LEFT_INDEXMCP = auto()
    LEFT_INDEXPIP = auto()
    LEFT_MIDDLEMCP = auto()
    LEFT_MIDDLEPIP = auto()
    LEFT_RINGMCP = auto()
    LEFT_RINGPIP = auto()
    LEFT_LITTLEMCP = auto()
    LEFT_LITTLEPIP = auto()
    RIGHT_THUMBCMC = auto()
    RIGHT_THUMBMCP = auto()
    RIGHT_THUMBPIP = auto()
    RIGHT_INDEXMCP = auto()
    RIGHT_INDEXPIP = auto()
    RIGHT_MIDDLEMCP = auto()
    RIGHT_MIDDLEPIP = auto()
    RIGHT_RINGMCP = auto()
    RIGHT_RINGPIP = auto()
    RIGHT_LITTLEMCP = auto()
    RIGHT_LITTLEPIP = auto()


class RealRobotIdx030_Linker(Enum):
    ANKLE_PITCH = auto()
    KNEE_PITCH = auto()
    BUTTOCK_PITCH = auto()
    WAIST_YAW = auto()
    HEAD_YAW_J1 = auto()
    HEAD_PITCH_J2 = auto()
    SHOULDER_PITCH_L_J1 = auto()
    SHOULDER_ROLL_L_J2 = auto()
    ELBOW_PITCH_L_J3 = auto()
    ELBOW_YAW_L_J4 = auto()
    WRIST_PITCH_L_J5 = auto()
    WRIST_YAW_L_J6 = auto()
    WRIST_ROLL_L_J7 = auto()
    SHOULDER_PITCH_R_J1 = auto()
    SHOULDER_ROLL_R_J2 = auto()
    ELBOW_PITCH_R_J3 = auto()
    ELBOW_YAW_R_J4 = auto()
    WRIST_PITCH_R_J5 = auto()
    WRIST_YAW_R_J6 = auto()
    WRIST_ROLL_R_J7 = auto()
    LEFT_T_CMC_YAW = auto()
    LEFT_T_MCP = auto()
    LEFT_T_IP = auto()
    LEFT_IF_MCP_PITCH = auto()
    LEFT_IF_DIP = auto()
    LEFT_MF_MCP_PITCH = auto()
    LEFT_MF_DIP = auto()
    LEFT_RF_MCP_PITCH = auto()
    LEFT_RF_DIP = auto()
    LEFT_LF_MCP_PITCH = auto()
    LEFT_LF_DIP = auto()
    RIGHT_T_CMC_YAW = auto()
    RIGHT_T_MCP = auto()
    RIGHT_T_IP = auto()
    RIGHT_IF_MCP_PITCH = auto()
    RIGHT_IF_DIP = auto()
    RIGHT_MF_MCP_PITCH = auto()
    RIGHT_MF_DIP = auto()
    RIGHT_RF_MCP_PITCH = auto()
    RIGHT_RF_DIP = auto()
    RIGHT_LF_MCP_PITCH = auto()
    RIGHT_LF_DIP = auto()


def get_robot_sim2real_idx(articulation) -> List[int]:
    """
    This function maps the simulation joints to the real robot joint indices.
    
    Args: 
        articulation: The dexsim robot instance
    
    Returns:
        sim2real_idx: A list of indices mapping simulation joint names to real robot joint indices
    """
    sim_actived_names = articulation.get_actived_joint_names()
    sim_dict = {name: idx for idx, name in enumerate(sim_actived_names)}
    real_names = [realjoint.name for realjoint in RealRobotIdx]
    sim2real_idx = []
    for real_name in real_names:
        sim2real_idx.append(sim_dict[real_name])
    return sim2real_idx


def get_robot_real2sim_idx(articulation) -> List[int]:
    """
    This function maps the real robot joint indices to the simulation joints.

    Args:
        articulation: The dexsim robot instance
    
    Returns:
        real2sim_dix: A list of indices mapping real robot joint indices to simulation joints
    """
    sim_actived_names = articulation.get_actived_joint_names()
    real_names = [realjoint.name for realjoint in RealRobotIdx]
    real_dict = {name: idx for idx, name in enumerate(real_names)}
    real2sim_dix = []
    for sim_name in sim_actived_names:
        if sim_name in real_names:
            real2sim_dix.append(real_dict[sim_name])
    return real2sim_dix


def get_pin2real_idx(urdf_path,
                     robot_version: RobotType = RobotType.W1V02_1) -> List[int]:
    """
    This function maps the pinocchio joints to the real robot joint indices.

    Args:
        urdf_path: the urdf path used to build the pinocchio model
    
    Returns:
        pin2real_idx: A list of indices mapping pinocchio joints to real robot joints
    """
    if robot_version == RobotType.W1V02_1 or robot_version == RobotType.W1V02_2:
        pin_model = pinocchio.buildModelFromUrdf(urdf_path)
        if pin_model.njoints - 1 == 20:
            real_robot_idx = RealRobotIdx
        if pin_model.njoints - 1 == 40:
            real_robot_idx = RealRobotIdx_Brainco
        if pin_model.njoints - 1 == 24:
            real_robot_idx = RealRobotIdx_PGC
        if pin_model.njoints - 1 == 42:
            real_robot_idx = RealRobotIdx_Linker
        pin_names = list(pin_model.names)[1:]
        pin_dict = {name: idx for idx, name in enumerate(pin_names)}
        real_names = [realjoint.name for realjoint in real_robot_idx]
        pin2real_idx = []
        for real_name in real_names:
            pin2real_idx.append(pin_dict[real_name])
        return pin2real_idx
    elif robot_version == RobotType.W1V03_1:
        pin_model = pinocchio.buildModelFromUrdf(urdf_path)
        if pin_model.njoints - 1 == 20:
            real_robot_idx = RealRobotIdx030
        if pin_model.njoints - 1 == 40:
            real_robot_idx = RealRobotIdx030_Brainco
        if pin_model.njoints - 1 == 24:
            real_robot_idx = RealRobotIdx030_PGC
        if pin_model.njoints - 1 == 42:
            real_robot_idx = RealRobotIdx030_Linker
        pin_names = list(pin_model.names)[1:]
        pin_dict = {name: idx for idx, name in enumerate(pin_names)}
        real_names = [realjoint.name for realjoint in real_robot_idx]
        pin2real_idx = []
        for real_name in real_names:
            pin2real_idx.append(pin_dict[real_name])
        return pin2real_idx
    else:
        raise ValueError(f"Unsupported W1 version: {robot_version}")


def get_real2pin_idx(urdf_path,
                     robot_version: RobotType = RobotType.W1V02_1) -> List[int]:
    """
    This function maps the real robot joint indices to the pinocchio joints.

    Args: 
        urdf_path: the urdf path used to build the pinocchio model
    
    Returns:
        real2pin_idx: A list of indices mapping real robot joint indices to pinocchio joints
    """
    if robot_version == RobotType.W1V02_1 or robot_version == RobotType.W1V02_2:
        pin_model = pinocchio.buildModelFromUrdf(urdf_path)
        if pin_model.njoints - 1 == 20:
            real_robot_idx = RealRobotIdx
        if pin_model.njoints - 1 == 40:
            real_robot_idx = RealRobotIdx_Brainco
        if pin_model.njoints - 1 == 24:
            real_robot_idx = RealRobotIdx_PGC
        if pin_model.njoints - 1 == 42:
            real_robot_idx = RealRobotIdx_Linker
        pin_names = list(pin_model.names)[1:]
        real_names = [realjoint.name for realjoint in real_robot_idx]
        real_dict = {name: idx for idx, name in enumerate(real_names)}
        real2pin_idx = []
        for pin_name in pin_names:
            real2pin_idx.append(real_dict[pin_name])
        return real2pin_idx
    elif robot_version == RobotType.W1V03_1:
        pin_model = pinocchio.buildModelFromUrdf(urdf_path)
        if pin_model.njoints - 1 == 20:
            real_robot_idx = RealRobotIdx030
        if pin_model.njoints - 1 == 40:
            real_robot_idx = RealRobotIdx030_Brainco
        if pin_model.njoints - 1 == 24:
            real_robot_idx = RealRobotIdx030_PGC
        if pin_model.njoints - 1 == 42:
            real_robot_idx = RealRobotIdx030_Linker
        pin_names = list(pin_model.names)[1:]
        real_names = [realjoint.name for realjoint in real_robot_idx]
        real_dict = {name: idx for idx, name in enumerate(real_names)}
        real2pin_idx = []
        for pin_name in pin_names:
            real2pin_idx.append(real_dict[pin_name])
        return real2pin_idx
    else:
        raise ValueError(f"Unsupported W1 version: {robot_version}")


def get_real_idx(robot_version: RobotType = RobotType.W1V02_1,
                 end_effector_type: EndEffectorType = None):
    """
    This function returns the start and end indices for torso, head, left arm, and right arm in the real robot.

    Args:

    returns:
        - tuples that contain the start and end indices for torso, head, left arm, and right arm

    
    Notes:
        - This function requires the joints order in real robot can't have torso, head, arml, armr overlapped
    """
    if robot_version == RobotType.W1V02_1 or robot_version == RobotType.W1V02_2:
        if end_effector_type is None:
            real_robot_idx = RealRobotIdx
        if end_effector_type == EndEffectorType.BRAINCO_HAND:
            real_robot_idx = RealRobotIdx_Brainco
        if end_effector_type == EndEffectorType.PGC_GRIPPER:
            real_robot_idx = RealRobotIdx_PGC
        if end_effector_type == EndEffectorType.LINKER_HAND:
            real_robot_idx = RealRobotIdx_Linker
    elif robot_version == RobotType.W1V03_1:
        if end_effector_type is None:
            real_robot_idx = RealRobotIdx030
        if end_effector_type == EndEffectorType.BRAINCO_HAND:
            real_robot_idx = RealRobotIdx030_Brainco
        if end_effector_type == EndEffectorType.PGC_GRIPPER:
            real_robot_idx = RealRobotIdx030_PGC
        if end_effector_type == EndEffectorType.LINKER_HAND:
            real_robot_idx = RealRobotIdx030_Linker
    else:
        raise ValueError(f"Unsupported W1 version: {robot_version}")

    real_names = [realjoint.name for realjoint in real_robot_idx]
    real_dict = {name: idx for idx, name in enumerate(real_names)}

    if robot_version == RobotType.W1V02_1 or robot_version == RobotType.W1V02_2:
        ankle_idx, knee_idx, buttock_idx, waist_idx = real_dict[
            "ANKLE"], real_dict["KNEE"], real_dict["BUTTOCK"], real_dict[
                "WAIST"]
        neck1_idx, neck2_idx = real_dict["NECK1"], real_dict["NECK2"]
        left_j1_idx, left_j2_idx, left_j3_idx, left_j4_idx, left_j5_idx, left_j6_idx, left_j7_idx = \
            real_dict["LEFT_J1"], real_dict["LEFT_J2"], real_dict["LEFT_J3"], real_dict["LEFT_J4"], real_dict["LEFT_J5"], real_dict["LEFT_J6"], real_dict["LEFT_J7"]
        right_j1_idx, right_j2_idx, right_j3_idx, right_j4_idx, right_j5_idx, right_j6_idx, right_j7_idx = \
            real_dict["RIGHT_J1"], real_dict["RIGHT_J2"], real_dict["RIGHT_J3"], real_dict["RIGHT_J4"], real_dict["RIGHT_J5"], real_dict["RIGHT_J6"], real_dict["RIGHT_J7"]
        torso_idx = [ankle_idx, knee_idx, buttock_idx, waist_idx]
        head_idx = [neck1_idx, neck2_idx]
        left_arm_idx = [
            left_j1_idx, left_j2_idx, left_j3_idx, left_j4_idx, left_j5_idx,
            left_j6_idx, left_j7_idx
        ]
        right_arm_idx = [
            right_j1_idx, right_j2_idx, right_j3_idx, right_j4_idx,
            right_j5_idx, right_j6_idx, right_j7_idx
        ]

        if end_effector_type is None:
            return torso_idx, head_idx, left_arm_idx, right_arm_idx

        if end_effector_type == EndEffectorType.BRAINCO_HAND:
            left_thumb_prox_idx, left_thumb_dist_idx = real_dict[
                "LEFT_THUMBCMC"], real_dict["LEFT_THUMBMCP"]
            left_index_prox_idx, left_index_dist_idx = real_dict[
                "LEFT_INDEXMCP"], real_dict["LEFT_INDEXPIP"]
            left_middle_prox_idx, left_middle_dist_idx = real_dict[
                "LEFT_MIDDLEMCP"], real_dict["LEFT_MIDDLEPIP"]
            left_ring_prox_idx, left_ring_dist_idx = real_dict[
                "LEFT_RINGMCP"], real_dict["LEFT_RINGPIP"]
            left_little_prox_idx, left_little_dist_idx = real_dict[
                "LEFT_LITTLEMCP"], real_dict["LEFT_LITTLEPIP"]

            left_hand_idx = [
                left_thumb_prox_idx, left_thumb_dist_idx, left_index_prox_idx,
                left_index_dist_idx, left_middle_prox_idx, left_middle_dist_idx,
                left_ring_prox_idx, left_ring_dist_idx, left_little_prox_idx,
                left_little_dist_idx
            ]

            right_thumb_prox_idx, right_thumb_dist_idx = real_dict[
                "RIGHT_THUMBCMC"], real_dict["RIGHT_THUMBMCP"]
            right_index_prox_idx, right_index_dist_idx = real_dict[
                "RIGHT_INDEXMCP"], real_dict["RIGHT_INDEXPIP"]
            right_middle_prox_idx, right_middle_dist_idx = real_dict[
                "RIGHT_MIDDLEMCP"], real_dict["RIGHT_MIDDLEPIP"]
            right_ring_prox_idx, right_ring_dist_idx = real_dict[
                "RIGHT_RINGMCP"], real_dict["RIGHT_RINGPIP"]
            right_little_prox_idx, right_little_dist_idx = real_dict[
                "RIGHT_LITTLEMCP"], real_dict["RIGHT_LITTLEPIP"]

            right_hand_idx = [
                right_thumb_prox_idx, right_thumb_dist_idx,
                right_index_prox_idx, right_index_dist_idx,
                right_middle_prox_idx, right_middle_dist_idx,
                right_ring_prox_idx, right_ring_dist_idx, right_little_prox_idx,
                right_little_dist_idx
            ]

            return torso_idx, head_idx, left_arm_idx, right_arm_idx, left_hand_idx, right_hand_idx

        if end_effector_type == EndEffectorType.PGC_GRIPPER:
            left_finger1_idx, left_finger2_idx = real_dict[
                "LEFT_FINGER1"], real_dict["LEFT_FINGER2"]
            right_finger1_idx, right_finger2_idx = real_dict[
                "RIGHT_FINGER1"], real_dict["RIGHT_FINGER2"]
            left_hand_idx = [left_finger1_idx, left_finger2_idx]
            right_hand_idx = [right_finger1_idx, right_finger2_idx]

            return torso_idx, head_idx, left_arm_idx, right_arm_idx, left_hand_idx, right_hand_idx

        if end_effector_type == EndEffectorType.LINKER_HAND:
            left_thumb_prox_base_idx, left_thumb_prox_idx, left_thumb_dist_idx = \
                real_dict["LEFT_THUMBCMC"], real_dict["LEFT_THUMBMCP"], real_dict["LEFT_THUMBPIP"]
            left_index_prox_idx, left_index_dist_idx = \
                real_dict["LEFT_INDEXMCP"], real_dict["LEFT_INDEXPIP"]
            left_middle_prox_idx, left_middle_dist_idx = \
                real_dict["LEFT_MIDDLEMCP"], real_dict["LEFT_MIDDLEPIP"]
            left_ring_prox_idx, left_ring_dist_idx = \
                real_dict["LEFT_RINGMCP"], real_dict["LEFT_RINGPIP"]
            left_little_prox_idx, left_little_dist_idx = \
                real_dict["LEFT_LITTLEMCP"], real_dict["LEFT_LITTLEPIP"]

            left_hand_idx = [
                left_thumb_prox_base_idx, left_thumb_prox_idx,
                left_thumb_dist_idx, left_index_prox_idx, left_index_dist_idx,
                left_middle_prox_idx, left_middle_dist_idx, left_ring_prox_idx,
                left_ring_dist_idx, left_little_prox_idx, left_little_dist_idx
            ]

            right_thumb_prox_base_idx, right_thumb_prox_idx, right_thumb_dist_idx = \
                real_dict["RIGHT_THUMBCMC"], real_dict["RIGHT_THUMBMCP"], real_dict["RIGHT_THUMBPIP"]
            right_index_prox_idx, right_index_dist_idx = \
                real_dict["RIGHT_INDEXMCP"], real_dict["RIGHT_INDEXPIP"]
            right_middle_prox_idx, right_middle_dist_idx = \
                real_dict["RIGHT_MIDDLEMCP"], real_dict["RIGHT_MIDDLEPIP"]
            right_ring_prox_idx, right_ring_dist_idx = \
                real_dict["RIGHT_RINGMCP"], real_dict["RIGHT_RINGPIP"]
            right_little_prox_idx, right_little_dist_idx = \
                real_dict["RIGHT_LITTLEMCP"], real_dict["RIGHT_LITTLEPIP"]

            right_hand_idx = [
                right_thumb_prox_base_idx, right_thumb_prox_idx,
                right_thumb_dist_idx, right_index_prox_idx,
                right_index_dist_idx, right_middle_prox_idx,
                right_middle_dist_idx, right_ring_prox_idx, right_ring_dist_idx,
                right_little_prox_idx, right_little_dist_idx
            ]

            return torso_idx, head_idx, left_arm_idx, right_arm_idx, left_hand_idx, right_hand_idx
    elif robot_version == RobotType.W1V03_1:
        ankle_idx, knee_idx, buttock_idx, waist_idx = real_dict[
            "ANKLE_PITCH"], real_dict["KNEE_PITCH"], real_dict[
                "BUTTOCK_PITCH"], real_dict["WAIST_YAW"]
        neck1_idx, neck2_idx = real_dict["HEAD_YAW_J1"], real_dict[
            "HEAD_PITCH_J2"]
        left_j1_idx, left_j2_idx, left_j3_idx, left_j4_idx, left_j5_idx, left_j6_idx, left_j7_idx = \
            real_dict["SHOULDER_PITCH_L_J1"], real_dict["SHOULDER_ROLL_L_J2"], real_dict["ELBOW_PITCH_L_J3"], real_dict["ELBOW_YAW_L_J4"], real_dict["WRIST_PITCH_L_J5"], real_dict["WRIST_YAW_L_J6"], real_dict["WRIST_ROLL_L_J7"]
        right_j1_idx, right_j2_idx, right_j3_idx, right_j4_idx, right_j5_idx, right_j6_idx, right_j7_idx = \
            real_dict["SHOULDER_PITCH_R_J1"], real_dict["SHOULDER_ROLL_R_J2"], real_dict["ELBOW_PITCH_R_J3"], real_dict["ELBOW_YAW_R_J4"], real_dict["WRIST_PITCH_R_J5"], real_dict["WRIST_YAW_R_J6"], real_dict["WRIST_ROLL_R_J7"]
        torso_idx = [ankle_idx, knee_idx, buttock_idx, waist_idx]
        head_idx = [neck1_idx, neck2_idx]
        left_arm_idx = [
            left_j1_idx, left_j2_idx, left_j3_idx, left_j4_idx, left_j5_idx,
            left_j6_idx, left_j7_idx
        ]
        right_arm_idx = [
            right_j1_idx, right_j2_idx, right_j3_idx, right_j4_idx,
            right_j5_idx, right_j6_idx, right_j7_idx
        ]

        if end_effector_type is None:
            return torso_idx, head_idx, left_arm_idx, right_arm_idx

        if end_effector_type == EndEffectorType.BRAINCO_HAND:
            left_thumb_prox_idx, left_thumb_dist_idx = real_dict[
                "LEFT_T_CMC_YAW"], real_dict["LEFT_T_MCP"]
            left_index_prox_idx, left_index_dist_idx = real_dict[
                "LEFT_IF_MCP_PITCH"], real_dict["LEFT_IF_DIP"]
            left_middle_prox_idx, left_middle_dist_idx = real_dict[
                "LEFT_MF_MCP_PITCH"], real_dict["LEFT_MF_DIP"]
            left_ring_prox_idx, left_ring_dist_idx = real_dict[
                "LEFT_RF_MCP_PITCH"], real_dict["LEFT_RF_DIP"]
            left_little_prox_idx, left_little_dist_idx = real_dict[
                "LEFT_LF_MCP_PITCH"], real_dict["LEFT_LF_DIP"]

            left_hand_idx = [
                left_thumb_prox_idx, left_thumb_dist_idx, left_index_prox_idx,
                left_index_dist_idx, left_middle_prox_idx, left_middle_dist_idx,
                left_ring_prox_idx, left_ring_dist_idx, left_little_prox_idx,
                left_little_dist_idx
            ]

            right_thumb_prox_idx, right_thumb_dist_idx = real_dict[
                "RIGHT_T_CMC_YAW"], real_dict["RIGHT_T_MCP"]
            right_index_prox_idx, right_index_dist_idx = real_dict[
                "RIGHT_IF_MCP_PITCH"], real_dict["RIGHT_IF_DIP"]
            right_middle_prox_idx, right_middle_dist_idx = real_dict[
                "RIGHT_MF_MCP_PITCH"], real_dict["RIGHT_MF_DIP"]
            right_ring_prox_idx, right_ring_dist_idx = real_dict[
                "RIGHT_RF_MCP_PITCH"], real_dict["RIGHT_RF_DIP"]
            right_little_prox_idx, right_little_dist_idx = real_dict[
                "RIGHT_LF_MCP_PITCH"], real_dict["RIGHT_LF_DIP"]

            right_hand_idx = [
                right_thumb_prox_idx, right_thumb_dist_idx,
                right_index_prox_idx, right_index_dist_idx,
                right_middle_prox_idx, right_middle_dist_idx,
                right_ring_prox_idx, right_ring_dist_idx, right_little_prox_idx,
                right_little_dist_idx
            ]

            return torso_idx, head_idx, left_arm_idx, right_arm_idx, left_hand_idx, right_hand_idx

        if end_effector_type == EndEffectorType.PGC_GRIPPER:
            left_finger1_idx, left_finger2_idx = real_dict[
                "LEFT_FINGER_1"], real_dict["LEFT_FINGER_2"]
            right_finger1_idx, right_finger2_idx = real_dict[
                "RIGHT_FINGER_1"], real_dict["RIGHT_FINGER_2"]
            left_hand_idx = [left_finger1_idx, left_finger2_idx]
            right_hand_idx = [right_finger1_idx, right_finger2_idx]

            return torso_idx, head_idx, left_arm_idx, right_arm_idx, left_hand_idx, right_hand_idx

        if end_effector_type == EndEffectorType.LINKER_HAND:
            left_thumb_prox_base_idx, left_thumb_prox_idx, left_thumb_dist_idx = \
                real_dict["LEFT_T_CMC_YAW"], real_dict["LEFT_T_MCP"], real_dict["LEFT_T_IP"]
            left_index_prox_idx, left_index_dist_idx = \
                real_dict["LEFT_IF_MCP_PITCH"], real_dict["LEFT_IF_DIP"]
            left_middle_prox_idx, left_middle_dist_idx = \
                real_dict["LEFT_MF_MCP_PITCH"], real_dict["LEFT_MF_DIP"]
            left_ring_prox_idx, left_ring_dist_idx = \
                real_dict["LEFT_RF_MCP_PITCH"], real_dict["LEFT_RF_DIP"]
            left_little_prox_idx, left_little_dist_idx = \
                real_dict["LEFT_LF_MCP_PITCH"], real_dict["LEFT_LF_DIP"]

            left_hand_idx = [
                left_thumb_prox_base_idx, left_thumb_prox_idx,
                left_thumb_dist_idx, left_index_prox_idx, left_index_dist_idx,
                left_middle_prox_idx, left_middle_dist_idx, left_ring_prox_idx,
                left_ring_dist_idx, left_little_prox_idx, left_little_dist_idx
            ]

            right_thumb_prox_base_idx, right_thumb_prox_idx, right_thumb_dist_idx = \
                real_dict["RIGHT_T_CMC_YAW"], real_dict["RIGHT_T_MCP"], real_dict["RIGHT_T_IP"]
            right_index_prox_idx, right_index_dist_idx = \
                real_dict["RIGHT_IF_MCP_PITCH"], real_dict["RIGHT_IF_DIP"]
            right_middle_prox_idx, right_middle_dist_idx = \
                real_dict["RIGHT_MF_MCP_PITCH"], real_dict["RIGHT_MF_DIP"]
            right_ring_prox_idx, right_ring_dist_idx = \
                real_dict["RIGHT_RF_MCP_PITCH"], real_dict["RIGHT_RF_DIP"]
            right_little_prox_idx, right_little_dist_idx = \
                real_dict["RIGHT_LF_MCP_PITCH"], real_dict["RIGHT_LF_DIP"]

            right_hand_idx = [
                right_thumb_prox_base_idx, right_thumb_prox_idx,
                right_thumb_dist_idx, right_index_prox_idx,
                right_index_dist_idx, right_middle_prox_idx,
                right_middle_dist_idx, right_ring_prox_idx, right_ring_dist_idx,
                right_little_prox_idx, right_little_dist_idx
            ]

            return torso_idx, head_idx, left_arm_idx, right_arm_idx, left_hand_idx, right_hand_idx


def get_pin_idx(urdf_path, robot_version: RobotType = RobotType.W1V02_1):
    """
    This function returns the start and end indices for torso, head, left arm, and right arm in the pinocchio model.

    Args:
        urdf_path: the urdf path used to build the pinocchio model
    Returns:
        tuples that contain the start and end indices for torso, head, left arm, and right arm

    Notes:
        - This function requires the joints order in pinocchio can't have torso, head, arml, armr overlapped
    """
    pin_model = pinocchio.buildModelFromUrdf(urdf_path)
    pin_names = list(pin_model.names)[1:]
    pin_dict = {name: idx for idx, name in enumerate(pin_names)}

    if robot_version == RobotType.W1V02_1 or robot_version == RobotType.W1V02_2:
        ankle_idx, knee_idx, buttock_idx, waist_idx = pin_dict[
            "ANKLE"], pin_dict["KNEE"], pin_dict["BUTTOCK"], pin_dict["WAIST"]
        neck1_idx, neck2_idx = pin_dict["NECK1"], pin_dict["NECK2"]
        left_j1_idx, left_j2_idx, left_j3_idx, left_j4_idx, left_j5_idx, left_j6_idx, left_j7_idx = \
            pin_dict["LEFT_J1"], pin_dict["LEFT_J2"], pin_dict["LEFT_J3"], pin_dict["LEFT_J4"], pin_dict["LEFT_J5"], pin_dict["LEFT_J6"], pin_dict["LEFT_J7"]
        right_j1_idx, right_j2_idx, right_j3_idx, right_j4_idx, right_j5_idx, right_j6_idx, right_j7_idx = \
            pin_dict["RIGHT_J1"], pin_dict["RIGHT_J2"], pin_dict["RIGHT_J3"], pin_dict["RIGHT_J4"], pin_dict["RIGHT_J5"], pin_dict["RIGHT_J6"], pin_dict["RIGHT_J7"]
        torso_idx = [ankle_idx, knee_idx, buttock_idx, waist_idx]
        head_idx = [neck1_idx, neck2_idx]
        left_arm_idx = [
            left_j1_idx, left_j2_idx, left_j3_idx, left_j4_idx, left_j5_idx,
            left_j6_idx, left_j7_idx
        ]
        right_arm_idx = [
            right_j1_idx, right_j2_idx, right_j3_idx, right_j4_idx,
            right_j5_idx, right_j6_idx, right_j7_idx
        ]

        if pin_model.njoints - 1 == 20:
            return torso_idx, head_idx, left_arm_idx, right_arm_idx

        if pin_model.njoints - 1 == 40:
            left_thumb_prox_idx, left_thumb_dist_idx = pin_dict[
                "LEFT_THUMBCMC"], pin_dict["LEFT_THUMBMCP"]
            left_index_prox_idx, left_index_dist_idx = pin_dict[
                "LEFT_INDEXMCP"], pin_dict["LEFT_INDEXPIP"]
            left_middle_prox_idx, left_middle_dist_idx = pin_dict[
                "LEFT_MIDDLEMCP"], pin_dict["LEFT_MIDDLEPIP"]
            left_ring_prox_idx, left_ring_dist_idx = pin_dict[
                "LEFT_RINGMCP"], pin_dict["LEFT_RINGPIP"]
            left_little_prox_idx, left_little_dist_idx = pin_dict[
                "LEFT_LITTLEMCP"], pin_dict["LEFT_LITTLEPIP"]

            left_hand_idx = [
                left_thumb_prox_idx, left_thumb_dist_idx, left_index_prox_idx,
                left_index_dist_idx, left_middle_prox_idx, left_middle_dist_idx,
                left_ring_prox_idx, left_ring_dist_idx, left_little_prox_idx,
                left_little_dist_idx
            ]

            right_thumb_prox_idx, right_thumb_dist_idx = pin_dict[
                "RIGHT_THUMBCMC"], pin_dict["RIGHT_THUMBMCP"]
            right_index_prox_idx, right_index_dist_idx = pin_dict[
                "RIGHT_INDEXMCP"], pin_dict["RIGHT_INDEXPIP"]
            right_middle_prox_idx, right_middle_dist_idx = pin_dict[
                "RIGHT_MIDDLEMCP"], pin_dict["RIGHT_MIDDLEPIP"]
            right_ring_prox_idx, right_ring_dist_idx = pin_dict[
                "RIGHT_RINGMCP"], pin_dict["RIGHT_RINGPIP"]
            right_little_prox_idx, right_little_dist_idx = pin_dict[
                "RIGHT_LITTLEMCP"], pin_dict["RIGHT_LITTLEPIP"]

            right_hand_idx = [
                right_thumb_prox_idx, right_thumb_dist_idx,
                right_index_prox_idx, right_index_dist_idx,
                right_middle_prox_idx, right_middle_dist_idx,
                right_ring_prox_idx, right_ring_dist_idx, right_little_prox_idx,
                right_little_dist_idx
            ]

            return torso_idx, head_idx, left_arm_idx, right_arm_idx, left_hand_idx, right_hand_idx
        if pin_model.njoints - 1 == 24:
            left_finger1_idx, left_finger2_idx = pin_dict[
                "LEFT_FINGER1"], pin_dict["LEFT_FINGER2"]
            right_finger1_idx, right_finger2_idx = pin_dict[
                "RIGHT_FINGER1"], pin_dict["RIGHT_FINGER2"]
            left_hand_idx = [left_finger1_idx, left_finger2_idx]
            right_hand_idx = [right_finger1_idx, right_finger2_idx]

            return torso_idx, head_idx, left_arm_idx, right_arm_idx, left_hand_idx, right_hand_idx

        if pin_model.njoints - 1 == 42:
            left_thumb_prox_base_idx, left_thumb_prox_idx, left_thumb_dist_idx = \
                pin_dict["LEFT_THUMBCMC"], pin_dict["LEFT_THUMBMCP"], pin_dict["LEFT_THUMBPIP"]
            left_index_prox_idx, left_index_dist_idx = \
                pin_dict["LEFT_INDEXMCP"], pin_dict["LEFT_INDEXPIP"]
            left_middle_prox_idx, left_middle_dist_idx = \
                pin_dict["LEFT_MIDDLEMCP"], pin_dict["LEFT_MIDDLEPIP"]
            left_ring_prox_idx, left_ring_dist_idx = \
                pin_dict["LEFT_RINGMCP"], pin_dict["LEFT_RINGPIP"]
            left_little_prox_idx, left_little_dist_idx = \
                pin_dict["LEFT_LITTLEMCP"], pin_dict["LEFT_LITTLEPIP"]

            left_hand_idx = [
                left_thumb_prox_base_idx, left_thumb_prox_idx,
                left_thumb_dist_idx, left_index_prox_idx, left_index_dist_idx,
                left_middle_prox_idx, left_middle_dist_idx, left_ring_prox_idx,
                left_ring_dist_idx, left_little_prox_idx, left_little_dist_idx
            ]

            right_thumb_prox_base_idx, right_thumb_prox_idx, right_thumb_dist_idx = \
                pin_dict["RIGHT_THUMBCMC"], pin_dict["RIGHT_THUMBMCP"], pin_dict["RIGHT_THUMBPIP"]
            right_index_prox_idx, right_index_dist_idx = \
                pin_dict["RIGHT_INDEXMCP"], pin_dict["RIGHT_INDEXPIP"]
            right_middle_prox_idx, right_middle_dist_idx = \
                pin_dict["RIGHT_MIDDLEMCP"], pin_dict["RIGHT_MIDDLEPIP"]
            right_ring_prox_idx, right_ring_dist_idx = \
                pin_dict["RIGHT_RINGMCP"], pin_dict["RIGHT_RINGPIP"]
            right_little_prox_idx, right_little_dist_idx = \
                pin_dict["RIGHT_LITTLEMCP"], pin_dict["RIGHT_LITTLEPIP"]

            right_hand_idx = [
                right_thumb_prox_base_idx, right_thumb_prox_idx,
                right_thumb_dist_idx, right_index_prox_idx,
                right_index_dist_idx, right_middle_prox_idx,
                right_middle_dist_idx, right_ring_prox_idx, right_ring_dist_idx,
                right_little_prox_idx, right_little_dist_idx
            ]

            return torso_idx, head_idx, left_arm_idx, right_arm_idx, left_hand_idx, right_hand_idx
    elif robot_version == RobotType.W1V03_1:
        ankle_idx, knee_idx, buttock_idx, waist_idx = pin_dict[
            "ANKLE_PITCH"], pin_dict["KNEE_PITCH"], pin_dict[
                "BUTTOCK_PITCH"], pin_dict["WAIST_YAW"]
        neck1_idx, neck2_idx = pin_dict["HEAD_YAW_J1"], pin_dict[
            "HEAD_PITCH_J2"]
        left_j1_idx, left_j2_idx, left_j3_idx, left_j4_idx, left_j5_idx, left_j6_idx, left_j7_idx = \
            pin_dict["SHOULDER_PITCH_L_J1"], pin_dict["SHOULDER_ROLL_L_J2"], pin_dict["ELBOW_PITCH_L_J3"], pin_dict["ELBOW_YAW_L_J4"], pin_dict["WRIST_PITCH_L_J5"], pin_dict["WRIST_YAW_L_J6"], pin_dict["WRIST_ROLL_L_J7"]
        right_j1_idx, right_j2_idx, right_j3_idx, right_j4_idx, right_j5_idx, right_j6_idx, right_j7_idx = \
            pin_dict["SHOULDER_PITCH_R_J1"], pin_dict["SHOULDER_ROLL_R_J2"], pin_dict["ELBOW_PITCH_R_J3"], pin_dict["ELBOW_YAW_R_J4"], pin_dict["WRIST_PITCH_R_J5"], pin_dict["WRIST_YAW_R_J6"], pin_dict["WRIST_ROLL_R_J7"]
        torso_idx = [ankle_idx, knee_idx, buttock_idx, waist_idx]
        head_idx = [neck1_idx, neck2_idx]
        left_arm_idx = [
            left_j1_idx, left_j2_idx, left_j3_idx, left_j4_idx, left_j5_idx,
            left_j6_idx, left_j7_idx
        ]
        right_arm_idx = [
            right_j1_idx, right_j2_idx, right_j3_idx, right_j4_idx,
            right_j5_idx, right_j6_idx, right_j7_idx
        ]

        if pin_model.njoints - 1 == 20:
            return torso_idx, head_idx, left_arm_idx, right_arm_idx

        if pin_model.njoints - 1 == 40:
            left_thumb_prox_idx, left_thumb_dist_idx = pin_dict[
                "LEFT_T_CMC_YAW"], pin_dict["LEFT_T_MCP"]
            left_index_prox_idx, left_index_dist_idx = pin_dict[
                "LEFT_IF_MCP_PITCH"], pin_dict["LEFT_IF_DIP"]
            left_middle_prox_idx, left_middle_dist_idx = pin_dict[
                "LEFT_MF_MCP_PITCH"], pin_dict["LEFT_MF_DIP"]
            left_ring_prox_idx, left_ring_dist_idx = pin_dict[
                "LEFT_RF_MCP_PITCH"], pin_dict["LEFT_RF_DIP"]
            left_little_prox_idx, left_little_dist_idx = pin_dict[
                "LEFT_LF_MCP_PITCH"], pin_dict["LEFT_LF_DIP"]

            left_hand_idx = [
                left_thumb_prox_idx, left_thumb_dist_idx, left_index_prox_idx,
                left_index_dist_idx, left_middle_prox_idx, left_middle_dist_idx,
                left_ring_prox_idx, left_ring_dist_idx, left_little_prox_idx,
                left_little_dist_idx
            ]

            right_thumb_prox_idx, right_thumb_dist_idx = pin_dict[
                "RIGHT_T_CMC_YAW"], pin_dict["RIGHT_T_MCP"]
            right_index_prox_idx, right_index_dist_idx = pin_dict[
                "RIGHT_IF_MCP_PITCH"], pin_dict["RIGHT_IF_DIP"]
            right_middle_prox_idx, right_middle_dist_idx = pin_dict[
                "RIGHT_MF_MCP_PITCH"], pin_dict["RIGHT_MF_DIP"]
            right_ring_prox_idx, right_ring_dist_idx = pin_dict[
                "RIGHT_RF_MCP_PITCH"], pin_dict["RIGHT_RF_DIP"]
            right_little_prox_idx, right_little_dist_idx = pin_dict[
                "RIGHT_LF_MCP_PITCH"], pin_dict["RIGHT_LF_DIP"]

            right_hand_idx = [
                right_thumb_prox_idx, right_thumb_dist_idx,
                right_index_prox_idx, right_index_dist_idx,
                right_middle_prox_idx, right_middle_dist_idx,
                right_ring_prox_idx, right_ring_dist_idx, right_little_prox_idx,
                right_little_dist_idx
            ]

            return torso_idx, head_idx, left_arm_idx, right_arm_idx, left_hand_idx, right_hand_idx
        if pin_model.njoints - 1 == 24:
            left_finger1_idx, left_finger2_idx = pin_dict[
                "LEFT_FINGER_1"], pin_dict["LEFT_FINGER_2"]
            right_finger1_idx, right_finger2_idx = pin_dict[
                "RIGHT_FINGER_1"], pin_dict["RIGHT_FINGER_2"]
            left_hand_idx = [left_finger1_idx, left_finger2_idx]
            right_hand_idx = [right_finger1_idx, right_finger2_idx]

            return torso_idx, head_idx, left_arm_idx, right_arm_idx, left_hand_idx, right_hand_idx

        if pin_model.njoints - 1 == 42:
            left_thumb_prox_base_idx, left_thumb_prox_idx, left_thumb_dist_idx = \
                pin_dict["LEFT_T_CMC_YAW"], pin_dict["LEFT_T_MCP"], pin_dict["LEFT_T_IP"]
            left_index_prox_idx, left_index_dist_idx = \
                pin_dict["LEFT_IF_MCP_PITCH"], pin_dict["LEFT_IF_DIP"]
            left_middle_prox_idx, left_middle_dist_idx = \
                pin_dict["LEFT_MF_MCP_PITCH"], pin_dict["LEFT_MF_DIP"]
            left_ring_prox_idx, left_ring_dist_idx = \
                pin_dict["LEFT_RF_MCP_PITCH"], pin_dict["LEFT_RF_DIP"]
            left_little_prox_idx, left_little_dist_idx = \
                pin_dict["LEFT_LF_MCP_PITCH"], pin_dict["LEFT_LF_DIP"]

            left_hand_idx = [
                left_thumb_prox_base_idx, left_thumb_prox_idx,
                left_thumb_dist_idx, left_index_prox_idx, left_index_dist_idx,
                left_middle_prox_idx, left_middle_dist_idx, left_ring_prox_idx,
                left_ring_dist_idx, left_little_prox_idx, left_little_dist_idx
            ]

            right_thumb_prox_base_idx, right_thumb_prox_idx, right_thumb_dist_idx = \
                pin_dict["RIGHT_T_CMC_YAW"], pin_dict["RIGHT_T_MCP"], pin_dict["RIGHT_T_IP"]
            right_index_prox_idx, right_index_dist_idx = \
                pin_dict["RIGHT_IF_MCP_PITCH"], pin_dict["RIGHT_IF_DIP"]
            right_middle_prox_idx, right_middle_dist_idx = \
                pin_dict["RIGHT_MF_MCP_PITCH"], pin_dict["RIGHT_MF_DIP"]
            right_ring_prox_idx, right_ring_dist_idx = \
                pin_dict["RIGHT_RF_MCP_PITCH"], pin_dict["RIGHT_RF_DIP"]
            right_little_prox_idx, right_little_dist_idx = \
                pin_dict["RIGHT_LF_MCP_PITCH"], pin_dict["RIGHT_LF_DIP"]

            right_hand_idx = [
                right_thumb_prox_base_idx, right_thumb_prox_idx,
                right_thumb_dist_idx, right_index_prox_idx,
                right_index_dist_idx, right_middle_prox_idx,
                right_middle_dist_idx, right_ring_prox_idx, right_ring_dist_idx,
                right_little_prox_idx, right_little_dist_idx
            ]

            return torso_idx, head_idx, left_arm_idx, right_arm_idx, left_hand_idx, right_hand_idx
    else:
        raise ValueError(f"Unsupported W1 version: {robot_version}")


class LeftPGCGripperIdx(Enum):
    LEFT_FINGER1_JOINT = auto()


class RightPGCGripperIdx(Enum):
    RIGHT_FINGER1_JOINT = auto()


class LeftBrainCoHandIdx(Enum):
    LEFT_THUMBMCP = auto()
    LEFT_THUMBCMC = auto()
    LEFT_INDEXMCP = auto()
    LEFT_MIDDLEMCP = auto()
    LEFT_RINGMCP = auto()
    LEFT_LITTLEMCP = auto()


class RightBrainCoHandIdx(Enum):
    RIGHT_THUMBMCP = auto()
    RIGHT_THUMBCMC = auto()
    RIGHT_INDEXMCP = auto()
    RIGHT_MIDDLEMCP = auto()
    RIGHT_RINGMCP = auto()
    RIGHT_LITTLEMCP = auto()


def get_ee_sim2real_idx(articulation, is_left: bool,
                        end_effector_type: EndEffectorType) -> List[int]:
    sim_actived_names = articulation.get_actived_joint_names()
    sim_dict = {name: idx for idx, name in enumerate(sim_actived_names)}

    idx_class_map = {
        (EndEffectorType.BRAINCO_HAND, True): LeftBrainCoHandIdx,
        (EndEffectorType.BRAINCO_HAND, False): RightBrainCoHandIdx,
        (EndEffectorType.PGC_GRIPPER, True): LeftPGCGripperIdx,
        (EndEffectorType.PGC_GRIPPER, False): RightPGCGripperIdx,
    }

    idx_class = None
    try:
        idx_class = idx_class_map[(end_effector_type, is_left)]
    except KeyError:
        raise ValueError(f"{end_effector_type.name} are not support.")

    real_names = [realjoint.name for realjoint in idx_class]
    sim2real_idx = []
    for real_name in real_names:
        sim2real_idx.append(sim_dict[real_name])
    return sim2real_idx
