from enum import Enum, auto
from typing import List

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


def get_sim2real_idx(articulation) -> List[int]: 
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

def get_real2sim_idx(articulation) -> List[int]:
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
        real2sim_dix.append(real_dict[sim_name])
    return real2sim_dix

def get_pin2real_idx(pin_model) -> List[int]:
    """
    This function maps the pinocchio joints to the real robot joint indices.

    Args:
        pin_model: The pinocchio model of the robot
    
    Returns:
        pin2real_idx: A list of indices mapping pinocchio joints to real robot joints
    """
    pin_names = list(pin_model.names)[1:]
    pin_dict = {name: idx for idx, name in enumerate(pin_names)}
    real_names = [realjoint.name for realjoint in RealRobotIdx]
    pin2real_idx = []
    for real_name in real_names:
        pin2real_idx.append(pin_dict[real_name])
    return pin2real_idx

def get_real2pin_idx(pin_model) -> List[int]:
    """
    This function maps the real robot joint indices to the pinocchio joints.

    Args: 
        pin_model: The pinocchio model of the robot
    
    Returns:
        real2pin_idx: A list of indices mapping real robot joint indices to pinocchio joints
    """
    pin_names = list(pin_model.names)[1:]
    real_names = [realjoint.name for realjoint in RealRobotIdx]
    real_dict = {name: idx for idx, name in enumerate(real_names)}
    real2pin_idx = []
    for pin_name in pin_names:
        real2pin_idx.append(real_dict[pin_name])
    return real2pin_idx

def get_real_idx():
    """
    This function returns the start and end indices for torso, head, left arm, and right arm in the real robot.

    returns:
        tuples that contain the start and end indices for torso, head, left arm, and right arm
    
    Notes:
        - This function requires the joints order in real robot can't have torso, head, arml, armr overlapped
    """
    real_names = [realjoint.name for realjoint in RealRobotIdx]
    real_dict = {name: idx for idx, name in enumerate(real_names)}
    torso_start_idx = real_dict["ANKLE"]
    torso_end_idx = real_dict["WAIST"]
    head_start_idx = real_dict["NECK1"]
    head_end_idx = real_dict["NECK2"]
    left_arm_start_idx = real_dict["LEFT_J1"]
    left_arm_end_idx = real_dict["LEFT_J7"]
    right_arm_start_idx = real_dict["RIGHT_J1"]
    right_arm_end_idx = real_dict["RIGHT_J7"]
    return (torso_start_idx, torso_end_idx), (head_start_idx, head_end_idx), \
           (left_arm_start_idx, left_arm_end_idx), (right_arm_start_idx, right_arm_end_idx)



def get_pin_idx(pin_model):
    """
    This function returns the start and end indices for torso, head, left arm, and right arm in the pinocchio model.

    Args:
        pin_model: The pinocchio model of the robot

    Returns:
        tuples that contain the start and end indices for torso, head, left arm, and right arm

    Notes:
        - This function requires the joints order in pinocchio can't have torso, head, arml, armr overlapped
    """
    pin_names = list(pin_model.names)[1:]
    pin_dict = {name: idx for idx, name in enumerate(pin_names)}
    torso_start_idx = pin_dict["ANKLE"]
    torso_end_idx = pin_dict["WAIST"]
    head_start_idx = pin_dict["NECK1"]
    head_end_idx = pin_dict["NECK2"]
    left_arm_start_idx = pin_dict["LEFT_J1"]
    left_arm_end_idx = pin_dict["LEFT_J7"]
    right_arm_start_idx = pin_dict["RIGHT_J1"]
    right_arm_end_idx = pin_dict["RIGHT_J7"]
    return (torso_start_idx, torso_end_idx), (head_start_idx, head_end_idx), \
           (left_arm_start_idx, left_arm_end_idx), (right_arm_start_idx, right_arm_end_idx)