from enum import Enum, auto
import os
import pathlib
import numpy as np
from ....common.config import ASSETS_DIR
from ....end_effector_client import EndEffectorType 

SINGLE_ARM_DIM = 7
DUAL_ARM_DIM = 14

LARM_BOUND = np.array([
        [-2.9670597283903604, -2.0943951023931953, -2.9670597283903604, -2.356194490192345, -2.9670597283903604, -1.5707963267948966, -2.9670597283903604],
        [2.9670597283903604, 1.5707963267948966, 2.9670597283903604, 1.5707963267948966, 2.96705972839036041, 1.5707963267948966, 2.9670597283903604]
    ])
RARM_BOUND = np.array([
        [-2.9670597283903604, -1.5707963267948966, -2.9670597283903604, -1.5707963267948966, -2.9670597283903604, -1.5707963267948966, -2.9670597283903604],
        [2.9670597283903604, 2.0943951023931953, 2.9670597283903604, 2.356194490192345, 2.9670597283903604, 1.5707963267948966, 2.9670597283903604]
    ])

ANTH_MODEL_DIR = ASSETS_DIR / "W1_V2_A"
INDU_MODEL_DIR = ASSETS_DIR / "W1_V2_I"
ANTH_COLLISION_DIR = ANTH_MODEL_DIR / "Collision"
INDU_COLLISION_DIR = INDU_MODEL_DIR / "Collision"

# The following link names are used for old folder structure, kept for backward compatibility
# ARML_COMMON_LINK_NAMES = ["arml1.obj", "arml2.obj", "arml3.obj", "arml4.obj", "arml5_short.obj"]
# ARMR_COMMON_LINK_NAMES = ["armr1.obj", "armr2.obj", "armr3.obj", "armr4.obj", "armr5_short.obj"]
# ARML_SPECIAL_LINK_NAMES = ["arml6.obj", "arml7.obj"]
# ARMR_SPECIAL_LINK_NAMES = ["armr6.obj", "armr7.obj"]

# The following link names are used for new folder structure
ARML_LINK_NAMES = ["arml1.obj", "arml2.obj", "arml3.obj", "arml4.obj", "arml5.obj", "arml6.obj", "arml7.obj"]
ARMR_LINK_NAMES = ["armr1.obj", "armr2.obj", "armr3.obj", "armr4.obj", "armr5.obj", "armr6.obj", "armr7.obj"]
HEAD_LINK_NAMES = ["head1.obj", "head2.obj"]
TORSO_LINK_NAMES = ["torso1.obj", "torso2.obj", "torso3.obj", "torso4.obj"]

PGC_ANTHROPOMORPHIC_LINK_NAMES = ["pgc_left_anth.obj", "pgc_right_anth.obj"]
PGC_INDUSTRIAL_LINK_NAMES = ["pgc_left_anth.obj", "pgc_right_anth.obj"]

BRAINCO_ANTHROPOMORPHIC_LINK_NAMES = ["convex_left_anth.obj", "convex_right_anth.obj"]
BRAINCO_INDUSTRIAL_LINK_NAMES = ["convex_left_industrial.obj", "convex_right_industrial.obj"]

END_EFFECTOR_JOINT_NAMES = ["LEFT_EE", "RIGHT_EE"]


class RobotType(Enum):
    W1V02_1 = auto()
    W1V02_2 = auto()

class RobotURDF(Enum):
    W1V02_1 = os.path.join(ANTH_MODEL_DIR, "W1_V2_A.urdf")
    W1V02_2 = os.path.join(INDU_MODEL_DIR, "W1_V2_I.urdf")
    
class RobotCollision(Enum):
    # W1V02_COMMON_HEAD = os.path.join(COLLISION_DIR, "Common", "Head")
    # W1V02_COMMON_TORSO = os.path.join(COLLISION_DIR, "Common", "Torso")
    # W1V02_COMMON_LEFTARM = os.path.join(COLLISION_DIR, "Common", "LeftArm")
    # W1V02_COMMON_RIGHTARM = os.path.join(COLLISION_DIR, "Common", "RightArm")

    # W1V02_1_ANTHROPOMORPHIC_LEFTARM = os.path.join(COLLISION_DIR, "Anthropomorphic", "LeftArm")
    # W1V02_1_ANTHROPOMORPHIC_RIGHTARM = os.path.join(COLLISION_DIR, "Anthropomorphic", "RightArm")

    # W1V02_2_INDUSTRIAL_LEFTARM = os.path.join(COLLISION_DIR, "Industrial", "LeftArm")
    # W1V02_2_INDUSTRIAL_RIGHTARM = os.path.join(COLLISION_DIR, "Industrial", "RightArm")
    W1V02_ANTHROPOMORPHIC_HEAD = os.path.join(ANTH_COLLISION_DIR, "head")
    W1V02_ANTHROPOMORPHIC_TORSO = os.path.join(ANTH_COLLISION_DIR, "torso")
    W1V02_ANTHROPOMORPHIC_LEFTARM = os.path.join(ANTH_COLLISION_DIR, "left_arm")
    W1V02_ANTHROPOMORPHIC_RIGHTARM = os.path.join(ANTH_COLLISION_DIR, "right_arm")

    W1V02_INDUSTRIAL_HEAD = os.path.join(INDU_COLLISION_DIR, "head")
    W1V02_INDUSTRIAL_TORSO = os.path.join(INDU_COLLISION_DIR, "torso")
    W1V02_INDUSTRIAL_LEFTARM = os.path.join(INDU_COLLISION_DIR, "left_arm")
    W1V02_INDUSTRIAL_RIGHTARM = os.path.join(INDU_COLLISION_DIR, "right_arm")



class RobotEE(Enum):
    """
    Please put the end effector mesh folder under the /assets
    """
    PGC_GRIPPER = os.path.join(ASSETS_DIR, "PGC")
    BRAINCO_HAND = os.path.join(ASSETS_DIR, "Brainco")


def get_robot_urdf(robot_type: RobotType) -> str:
    if robot_type == RobotType.W1V02_1:
        return RobotURDF.W1V02_1.value
    elif robot_type == RobotType.W1V02_2:
        return RobotURDF.W1V02_2.value
    else:
        raise ValueError("Unsupported robot type. Please use W1V02_1 or W1V02_2.")
    
def get_robot_mesh_path(robot_type: RobotType) -> str:
    if robot_type == RobotType.W1V02_1:
        return RobotCollision.W1V02_ANTHROPOMORPHIC_HEAD.value, RobotCollision.W1V02_ANTHROPOMORPHIC_TORSO.value, \
                RobotCollision.W1V02_ANTHROPOMORPHIC_LEFTARM.value, RobotCollision.W1V02_ANTHROPOMORPHIC_RIGHTARM.value
    elif robot_type == RobotType.W1V02_2:
        return RobotCollision.W1V02_INDUSTRIAL_HEAD.value, RobotCollision.W1V02_INDUSTRIAL_TORSO.value, \
                RobotCollision.W1V02_INDUSTRIAL_LEFTARM.value, RobotCollision.W1V02_INDUSTRIAL_RIGHTARM.value
    else:
        raise ValueError("Unsupported robot type. Please use W1V02_1 or W1V02_2.")
    
def get_arml_full_path(robot_type: RobotType = None):
    if robot_type is None:
        raise ValueError("Robot type must be specified for getting arm left full path.")
    # arml_common_path = [RobotCollision.W1V02_COMMON_LEFTARM.value + "/" + name for name in ARML_COMMON_LINK_NAMES]
    # if robot_type == RobotType.W1V02_1:
    #     arml_special_path = [RobotCollision.W1V02_1_ANTHROPOMORPHIC_LEFTARM.value + "/" + name for name in ARML_SPECIAL_LINK_NAMES]
    # elif robot_type == RobotType.W1V02_2:
    #     arml_special_path = [RobotCollision.W1V02_2_INDUSTRIAL_LEFTARM.value + "/" + name for name in ARML_SPECIAL_LINK_NAMES]

    # arml_full_path = arml_common_path + arml_special_path
    if robot_type == RobotType.W1V02_1:
        arml_full_path = [RobotCollision.W1V02_ANTHROPOMORPHIC_LEFTARM.value + "/" + name for name in ARML_LINK_NAMES]
    elif robot_type == RobotType.W1V02_2:
        arml_full_path = [RobotCollision.W1V02_INDUSTRIAL_LEFTARM.value + "/" + name for name in ARML_LINK_NAMES]
    return arml_full_path

def get_armr_full_path(robot_type: RobotType = None):
    if robot_type is None:
        raise ValueError("Robot type must be specified for getting arm right full path.")
    # armr_common_path = [RobotCollision.W1V02_COMMON_RIGHTARM.value + "/" + name for name in ARMR_COMMON_LINK_NAMES]
    # if robot_type == RobotType.W1V02_1:
    #     armr_special_path = [RobotCollision.W1V02_1_ANTHROPOMORPHIC_RIGHTARM.value + "/" + name for name in ARMR_SPECIAL_LINK_NAMES]
    # elif robot_type == RobotType.W1V02_2:
    #     armr_special_path = [RobotCollision.W1V02_2_INDUSTRIAL_RIGHTARM.value + "/" + name for name in ARMR_SPECIAL_LINK_NAMES]

    # armr_full_path = armr_common_path + armr_special_path
    if robot_type == RobotType.W1V02_1:
        armr_full_path = [RobotCollision.W1V02_ANTHROPOMORPHIC_RIGHTARM.value + "/" + name for name in ARMR_LINK_NAMES]
    elif robot_type == RobotType.W1V02_2:
        armr_full_path = [RobotCollision.W1V02_INDUSTRIAL_RIGHTARM.value + "/" + name for name in ARMR_LINK_NAMES]
    return armr_full_path

def get_head_full_path():
    head_full_path = [RobotCollision.W1V02_ANTHROPOMORPHIC_HEAD.value + "/" + name for name in HEAD_LINK_NAMES]
    return head_full_path    

def get_torso_full_path():
    torso_full_path = [RobotCollision.W1V02_ANTHROPOMORPHIC_TORSO.value + "/" + name for name in TORSO_LINK_NAMES]
    return torso_full_path

def get_ee_full_path(robot_type: RobotType, ee_type: EndEffectorType):
    if ee_type == EndEffectorType.PGC_GRIPPER:
        if robot_type == RobotType.W1V02_1:
            return [RobotEE.PGC_GRIPPER.value + "/" + name for name in PGC_ANTHROPOMORPHIC_LINK_NAMES]
        if robot_type == RobotType.W1V02_2:
            return [RobotEE.PGC_GRIPPER.value + "/" + name for name in PGC_INDUSTRIAL_LINK_NAMES]
        if robot_type is None or robot_type not in list(RobotType):
            raise ValueError(f"Unsupported robot type: {robot_type}. Please use W1V02_1 or W1V02_2.")
    elif ee_type == EndEffectorType.BRAINCO_HAND:
        if robot_type == RobotType.W1V02_1:
            return [RobotEE.BRAINCO_HAND.value + "/" + name for name in BRAINCO_ANTHROPOMORPHIC_LINK_NAMES]
        if robot_type == RobotType.W1V02_2:
            return [RobotEE.BRAINCO_HAND.value + "/" + name for name in BRAINCO_INDUSTRIAL_LINK_NAMES]
        if robot_type is None or robot_type not in list(RobotType):
            raise ValueError(f"Unsupported robot type: {robot_type}. Please use W1V02_1 or W1V02_2.")
    else:
        raise ValueError(f"Unsupported end effector type: {ee_type}. Please use PGC_GRIPPER or BRAINCO_HAND.")

def get_env_json_path():
    """
    Returns the path to the environment JSON file.
    """
    # FIXME: Update this to the actual environment JSON file path when available
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "model", "coffee_env.json") 


def get_ee_joint_names():
    """
    Get the names of the end effector joints which will be used in Pinocchio Forward Kinematics
    """
    return END_EFFECTOR_JOINT_NAMES[0], END_EFFECTOR_JOINT_NAMES[1]