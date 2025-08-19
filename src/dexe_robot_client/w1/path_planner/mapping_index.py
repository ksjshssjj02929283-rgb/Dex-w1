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
