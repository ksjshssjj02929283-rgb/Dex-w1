from enum import Enum, auto
import os
import pathlib
import numpy as np
from ....common.config import ASSETS_DIR, DATA_DIR, MODEL_DIR, ENVIRONMENT_DIR
from ....end_effector_client import EndEffectorType

SINGLE_ARM_DIM = 7
DUAL_ARM_DIM = 14

# yapf: disable
LARM_BOUND = np.array([
    [-2.9670597283903604, -2.0943951023931953, -2.9670597283903604, -2.356194490192345, -2.9670597283903604, -1.5707963267948966, -2.9670597283903604],
    [2.9670597283903604, 1.5707963267948966, 2.9670597283903604, 1.5707963267948966, 2.96705972839036041, 1.5707963267948966, 2.9670597283903604]
    ])
RARM_BOUND = np.array([
    [-2.9670597283903604, -1.5707963267948966, -2.9670597283903604, -1.5707963267948966, -2.9670597283903604, -1.5707963267948966, -2.9670597283903604],
    [2.9670597283903604, 2.0943951023931953, 2.9670597283903604, 2.356194490192345, 2.9670597283903604, 1.5707963267948966, 2.9670597283903604]
    ])

"""
BRAINCO HAND JOINTS ORDER:
    - Thumb CMC, Thumb MCP, Index MCP, Index PIP, Middle MCP, Middle PIP, Ring MCP, Ring PIP, Little MCP, Little PIP
"""
BRAINCO_LEFT_LIMITS = np.array([
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [1.5708, 0.8727, 1.309, 2.0944, 1.309, 2.0944, 1.309, 2.0944, 1.309, 2.0944]
    ])
BRAINCO_RIGHT_LIMITS = np.array([
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [1.5708, 0.8727, 1.309, 2.0944, 1.309, 2.0944, 1.309, 2.0944, 1.309, 2.0944]
    ])

"""
LINKER HAND JOINTS ORDER:
    - Thumb CMC, Thumb MCP, Thumb PIP, Index MCP, Index PIP, Middle MCP, Middle PIP, Ring MCP, Ring PIP, Little MCP, Little PIP
"""
LINKER_LEFT_LIMITS = np.array([
    [0.0, 0.2359, 0.6126, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [1.0472, 0.99, 1.22, 1.1676, 1.14, 1.1379, 1.14, 1.1606, 1.14, 1.1693, 1.14]
    ])
LINKER_RIGHT_LIMITS = np.array([
    [0.0, 0.2359, 0.6126, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [1.0472, 0.99, 1.22, 1.1676, 1.14, 1.1379, 1.14, 1.1606, 1.14, 1.1693, 1.14]
    ])

# yapf: enable
PGC_LEFT_LIMITS = np.array([[0.0, 0.0], [0.025, 0.025]])
PGC_RIGHT_LIMITS = np.array([[0.0, 0.0], [0.025, 0.025]])

ANTH_MODEL_DIR = MODEL_DIR / "W1_V2_A"
INDU_MODEL_DIR = MODEL_DIR / "W1_V2_I"
ANTH_COLLISION_DIR = ANTH_MODEL_DIR / "Collision_simp"
INDU_COLLISION_DIR = INDU_MODEL_DIR / "Collision_simp"

ANTH_BRAINCO_MODEL_DIR = MODEL_DIR / "W1_V2_A_BRAINCO"
INDU_BRAINCO_MODEL_DIR = MODEL_DIR / "W1_V2_I_BRAINCO"
ANTH_BRAINCO_COLLISION_DIR = ANTH_BRAINCO_MODEL_DIR / "Collision"
INDU_BRAINCO_COLLISION_DIR = INDU_BRAINCO_MODEL_DIR / "Collision"

ANTH_PGC_MODEL_DIR = MODEL_DIR / "W1_V2_A_PGC"
INDU_PGC_MODEL_DIR = MODEL_DIR / "W1_V2_I_PGC"
ANTH_PGC_COLLISION_DIR = ANTH_PGC_MODEL_DIR / "Collision"
INDU_PGC_COLLISION_DIR = INDU_PGC_MODEL_DIR / "Collision"

ANTH_LINKER_MODEL_DIR = MODEL_DIR / "W1_V2_A_LINKER"
INDU_LINKER_MODEL_DIR = MODEL_DIR / "W1_V2_I_LINKER"
ANTH_LINKER_COLLISION_DIR = ANTH_LINKER_MODEL_DIR / "Collision"
INDU_LINKER_COLLISION_DIR = INDU_LINKER_MODEL_DIR / "Collision"

ARML_LINK_NAMES = [
    "arml1.obj", "arml2.obj", "arml3.obj", "arml4.obj", "arml5.obj",
    "arml6.obj", "arml7.obj"
]
ARMR_LINK_NAMES = [
    "armr1.obj", "armr2.obj", "armr3.obj", "armr4.obj", "armr5.obj",
    "armr6.obj", "armr7.obj"
]
HEAD_LINK_NAMES = ["head1.obj", "head2.obj"]
TORSO_LINK_NAMES = ["torso1.obj", "torso2.obj", "torso3.obj", "torso4.obj"]

BRAINCO_LEFT_LINK_NAMES = [
    "L_base_link.obj", "L_thumb_prox.obj", "L_thumb_dist.obj",
    "L_index_prox.obj", "L_index_dist.obj", "L_middle_prox.obj",
    "L_middle_dist.obj", "L_ring_prox.obj", "L_ring_dist.obj",
    "L_little_prox.obj", "L_little_dist.obj"
]
BRAINCO_RIGHT_LINK_NAMES = [
    "R_base_link.obj", "R_thumb_prox.obj", "R_thumb_dist.obj",
    "R_index_prox.obj", "R_index_dist.obj", "R_middle_prox.obj",
    "R_middle_dist.obj", "R_ring_prox.obj", "R_ring_dist.obj",
    "R_little_prox.obj", "R_little_dist.obj"
]

PGC_LEFT_LINK_NAMES = [
    "base_link.obj", "finger1_Link_acd.obj", "finger2_Link_acd.obj"
]
PGC_RIGHT_LINK_NAMES = [
    "base_link.obj", "finger1_Link_acd.obj", "finger2_Link_acd.obj"
]

LINKER_LEFT_LINK_NAMES = [
    "L_base_link.obj", "L_thumb_prox_base.obj", "L_thumb_prox.obj",
    "L_thumb_dist.obj", "L_index_prox.obj", "L_index_dist.obj",
    "L_middle_prox.obj", "L_middle_dist.obj", "L_ring_prox.obj",
    "L_ring_dist.obj", "L_little_prox.obj", "L_little_dist.obj"
]

LINKER_RIGHT_LINK_NAMES = [
    "R_base_link.obj", "R_thumb_prox_base.obj", "R_thumb_prox.obj",
    "R_thumb_dist.obj", "R_index_prox.obj", "R_index_dist.obj",
    "R_middle_prox.obj", "R_middle_dist.obj", "R_ring_prox.obj",
    "R_ring_dist.obj", "R_little_prox.obj", "R_little_dist.obj"
]

FLANGE_JOINT_NAMES = ["LEFT_EE", "RIGHT_EE"]
BRAINCO_BASE_JOINT_NAMES = [
    "LEFT_ARM_TO_LEFT_HAND_CONNECTOR", "RIGHT_ARM_TO_RIGHT_HAND_CONNECTOR"
]
PGC_BASE_JOINT_NAMES = [
    "LEFT_ARM_TO_LEFT_HAND_CONNECTOR", "RIGHT_ARM_TO_RIGHT_HAND_CONNECTOR"
]
LINKER_BASE_JOINT_NAMES = [
    "LEFT_ARM_TO_LEFT_HAND_CONNECTOR", "RIGHT_ARM_TO_RIGHT_HAND_CONNECTOR"
]


class RobotType(Enum):
    W1V02_1 = auto()
    W1V02_2 = auto()


class RobotURDF(Enum):
    W1V02_1 = os.path.join(ANTH_MODEL_DIR, "W1_V2_A.urdf")
    W1V02_2 = os.path.join(INDU_MODEL_DIR, "W1_V2_I.urdf")
    W1V02_1_BRAINCO = os.path.join(ANTH_BRAINCO_MODEL_DIR,
                                   "W1_V2_A_BRAINCO.urdf")
    W1V02_2_BRAINCO = os.path.join(INDU_BRAINCO_MODEL_DIR,
                                   "W1_V2_I_BRAINCO.urdf")
    W1V02_1_PGC = os.path.join(ANTH_PGC_MODEL_DIR, "W1_V2_A_PGC.urdf")
    W1V02_2_PGC = os.path.join(INDU_PGC_MODEL_DIR, "W1_V2_I_PGC.urdf")
    W1V02_1_LINKER = os.path.join(ANTH_LINKER_MODEL_DIR, "W1_V2_A_LINKER.urdf")
    W1V02_2_LINKER = os.path.join(INDU_LINKER_MODEL_DIR, "W1_V2_I_LINKER.urdf")


class RobotCollision(Enum):
    W1V02_ANTHROPOMORPHIC_HEAD = os.path.join(ANTH_COLLISION_DIR, "head")
    W1V02_ANTHROPOMORPHIC_TORSO = os.path.join(ANTH_COLLISION_DIR, "torso")
    W1V02_ANTHROPOMORPHIC_LEFTARM = os.path.join(ANTH_COLLISION_DIR, "left_arm")
    W1V02_ANTHROPOMORPHIC_RIGHTARM = os.path.join(ANTH_COLLISION_DIR,
                                                  "right_arm")

    W1V02_INDUSTRIAL_HEAD = os.path.join(INDU_COLLISION_DIR, "head")
    W1V02_INDUSTRIAL_TORSO = os.path.join(INDU_COLLISION_DIR, "torso")
    W1V02_INDUSTRIAL_LEFTARM = os.path.join(INDU_COLLISION_DIR, "left_arm")
    W1V02_INDUSTRIAL_RIGHTARM = os.path.join(INDU_COLLISION_DIR, "right_arm")


class RobotEE(Enum):
    """
    Please put the end effector mesh folder under the /assets
    """

    BRAINCO_LEFT_HAND = os.path.join(MODEL_DIR, "simplified_brainco",
                                     "left_hand")
    BRAINCO_RIGHT_HAND = os.path.join(MODEL_DIR, "simplified_brainco",
                                      "right_hand")

    PGC_LEFT_HAND = os.path.join(MODEL_DIR, "simplified_pgc", "left_hand")
    PGC_RIGHT_HAND = os.path.join(MODEL_DIR, "simplified_pgc", "right_hand")

    LINKER_LEFT_HAND = os.path.join(MODEL_DIR, "simplified_linker", "left_hand")
    LINKER_RIGHT_HAND = os.path.join(MODEL_DIR, "simplified_linker",
                                     "right_hand")


def get_robot_urdf(robot_type: RobotType,
                   ee_type: EndEffectorType = None) -> str:
    if ee_type is None:
        if robot_type == RobotType.W1V02_1:
            return RobotURDF.W1V02_1.value
        elif robot_type == RobotType.W1V02_2:
            return RobotURDF.W1V02_2.value
        else:
            raise ValueError(
                "Unsupported robot type. Please use W1V02_1 or W1V02_2.")
    else:
        if isinstance(ee_type, EndEffectorType):
            if ee_type == EndEffectorType.BRAINCO_HAND:
                if robot_type == RobotType.W1V02_1:
                    return RobotURDF.W1V02_1_BRAINCO.value
                elif robot_type == RobotType.W1V02_2:
                    return RobotURDF.W1V02_2_BRAINCO.value
                else:
                    raise ValueError(
                        "Unsupported robot type. Please use W1V02_1 or W1V02_2."
                    )
            elif ee_type == EndEffectorType.PGC_GRIPPER:
                if robot_type == RobotType.W1V02_1:
                    return RobotURDF.W1V02_1_PGC.value
                elif robot_type == RobotType.W1V02_2:
                    return RobotURDF.W1V02_2_PGC.value
                else:
                    raise ValueError(
                        "Unsupported robot type. Please use W1V02_1 or W1V02_2."
                    )
            elif ee_type == EndEffectorType.LINKER_HAND:
                if robot_type == RobotType.W1V02_1:
                    return RobotURDF.W1V02_1_LINKER.value
                elif robot_type == RobotType.W1V02_2:
                    return RobotURDF.W1V02_2_LINKER.value
                else:
                    raise ValueError(
                        "Unsupported robot type. Please use W1V02_1 or W1V02_2."
                    )
        else:
            raise ValueError(
                "Unsupported end effector type. Please use BRAINCO_HAND or PGC_GRIPPER or LINKER_HAND."
            )


def get_robot_mesh_path(robot_type: RobotType) -> str:
    if robot_type == RobotType.W1V02_1:
        return RobotCollision.W1V02_ANTHROPOMORPHIC_HEAD.value, RobotCollision.W1V02_ANTHROPOMORPHIC_TORSO.value, \
                RobotCollision.W1V02_ANTHROPOMORPHIC_LEFTARM.value, RobotCollision.W1V02_ANTHROPOMORPHIC_RIGHTARM.value
    elif robot_type == RobotType.W1V02_2:
        return RobotCollision.W1V02_INDUSTRIAL_HEAD.value, RobotCollision.W1V02_INDUSTRIAL_TORSO.value, \
                RobotCollision.W1V02_INDUSTRIAL_LEFTARM.value, RobotCollision.W1V02_INDUSTRIAL_RIGHTARM.value
    else:
        raise ValueError(
            "Unsupported robot type. Please use W1V02_1 or W1V02_2.")


def get_arml_full_path(robot_type: RobotType = None):
    if robot_type is None:
        raise ValueError(
            "Robot type must be specified for getting arm left full path.")
    if robot_type == RobotType.W1V02_1:
        arml_full_path = [
            RobotCollision.W1V02_ANTHROPOMORPHIC_LEFTARM.value + "/" + name
            for name in ARML_LINK_NAMES
        ]
    elif robot_type == RobotType.W1V02_2:
        arml_full_path = [
            RobotCollision.W1V02_INDUSTRIAL_LEFTARM.value + "/" + name
            for name in ARML_LINK_NAMES
        ]
    return arml_full_path


def get_armr_full_path(robot_type: RobotType = None):
    if robot_type is None:
        raise ValueError(
            "Robot type must be specified for getting arm right full path.")
    if robot_type == RobotType.W1V02_1:
        armr_full_path = [
            RobotCollision.W1V02_ANTHROPOMORPHIC_RIGHTARM.value + "/" + name
            for name in ARMR_LINK_NAMES
        ]
    elif robot_type == RobotType.W1V02_2:
        armr_full_path = [
            RobotCollision.W1V02_INDUSTRIAL_RIGHTARM.value + "/" + name
            for name in ARMR_LINK_NAMES
        ]
    return armr_full_path


def get_head_full_path(robot_type: RobotType = None):
    if robot_type is None:
        raise ValueError(
            "Robot type must be specified for getting head full path.")
    if robot_type == RobotType.W1V02_1:
        head_full_path = [
            RobotCollision.W1V02_ANTHROPOMORPHIC_HEAD.value + "/" + name
            for name in HEAD_LINK_NAMES
        ]
    elif robot_type == RobotType.W1V02_2:
        head_full_path = [
            RobotCollision.W1V02_INDUSTRIAL_HEAD.value + "/" + name
            for name in HEAD_LINK_NAMES
        ]
    return head_full_path


def get_torso_full_path(robot_type: RobotType = None):
    if robot_type is None:
        raise ValueError(
            "Robot type must be specified for getting torso full path.")
    if robot_type == RobotType.W1V02_1:
        torso_full_path = [
            RobotCollision.W1V02_ANTHROPOMORPHIC_TORSO.value + "/" + name
            for name in TORSO_LINK_NAMES
        ]
    elif robot_type == RobotType.W1V02_2:
        torso_full_path = [
            RobotCollision.W1V02_INDUSTRIAL_TORSO.value + "/" + name
            for name in TORSO_LINK_NAMES
        ]
    return torso_full_path


def get_ee_full_path(robot_type: RobotType, ee_type: EndEffectorType):
    if ee_type is None:
        return [], []
    if ee_type == EndEffectorType.BRAINCO_HAND:
        if robot_type == RobotType.W1V02_1:
            return [RobotEE.BRAINCO_LEFT_HAND.value + "/" + name for name in BRAINCO_LEFT_LINK_NAMES], \
                     [RobotEE.BRAINCO_RIGHT_HAND.value + "/" + name for name in BRAINCO_RIGHT_LINK_NAMES]
        if robot_type == RobotType.W1V02_2:
            return [RobotEE.BRAINCO_LEFT_HAND.value + "/" + name for name in BRAINCO_LEFT_LINK_NAMES], \
                     [RobotEE.BRAINCO_RIGHT_HAND.value + "/" + name for name in BRAINCO_RIGHT_LINK_NAMES]
        if robot_type is None or not isinstance(robot_type, RobotType):
            raise ValueError(
                f"Unsupported robot type: {robot_type}. Please use W1V02_1 or W1V02_2."
            )
    elif ee_type == EndEffectorType.PGC_GRIPPER:
        if robot_type == RobotType.W1V02_1:
            return [RobotEE.PGC_LEFT_HAND.value + "/" + name for name in PGC_LEFT_LINK_NAMES], \
                     [RobotEE.PGC_RIGHT_HAND.value + "/" + name for name in PGC_RIGHT_LINK_NAMES]
        if robot_type == RobotType.W1V02_2:
            return [RobotEE.PGC_LEFT_HAND.value + "/" + name for name in PGC_LEFT_LINK_NAMES], \
                     [RobotEE.PGC_RIGHT_HAND.value + "/" + name for name in PGC_RIGHT_LINK_NAMES]
        if robot_type is None or not isinstance(robot_type, RobotType):
            raise ValueError(
                f"Unsupported robot type: {robot_type}. Please use W1V02_1 or W1V02_2."
            )
    elif ee_type == EndEffectorType.LINKER_HAND:
        if robot_type == RobotType.W1V02_1:
            return [RobotEE.LINKER_LEFT_HAND.value + "/" + name for name in LINKER_LEFT_LINK_NAMES], \
                        [RobotEE.LINKER_RIGHT_HAND.value + "/" + name for name in LINKER_RIGHT_LINK_NAMES]
        if robot_type == RobotType.W1V02_2:
            return [RobotEE.LINKER_LEFT_HAND.value + "/" + name for name in LINKER_LEFT_LINK_NAMES], \
                        [RobotEE.LINKER_RIGHT_HAND.value + "/" + name for name in LINKER_RIGHT_LINK_NAMES]
        if robot_type is None or not isinstance(robot_type, RobotType):
            raise ValueError(
                f"Unsupported robot type: {robot_type}. Please use W1V02_1 or W1V02_2."
            )
    else:
        raise ValueError(
            f"Unsupported end effector type: {ee_type}. Please use PGC_GRIPPER or BRAINCO_HAND or LINKER_HAND."
        )


def get_env_json_path():
    """
    Returns the path to the environment JSON file.
    """
    # FIXME: Update this to the actual environment JSON file path when available
    return os.path.join(DATA_DIR, "environment", "environment.json")


def get_ee_joint_names(ee_type: EndEffectorType = None):
    """
    Get the names of the end effector joints which will be used in Pinocchio Forward Kinematics
    """
    if ee_type is None:
        return FLANGE_JOINT_NAMES[0], FLANGE_JOINT_NAMES[1]
    if ee_type == EndEffectorType.BRAINCO_HAND:
        return BRAINCO_BASE_JOINT_NAMES[0], BRAINCO_BASE_JOINT_NAMES[1]
    elif ee_type == EndEffectorType.PGC_GRIPPER:
        return PGC_BASE_JOINT_NAMES[0], PGC_BASE_JOINT_NAMES[1]
    elif ee_type == EndEffectorType.LINKER_HAND:
        return LINKER_BASE_JOINT_NAMES[0], LINKER_BASE_JOINT_NAMES[1]
    else:
        raise ValueError(
            "Unsupported end effector type. Please use BRAINCO_HAND or PGC_GRIPPER or LINKER_HAND."
        )
