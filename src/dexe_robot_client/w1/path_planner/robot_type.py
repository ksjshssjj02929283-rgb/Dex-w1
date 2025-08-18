from enum import Enum, auto
import os

current_path = os.path.join(os.path.dirname(os.path.abspath(__file__)))
model_path = os.path.join(current_path, "model")


class RobotType(Enum):
    W1V02_1 = auto()
    W1V02_2 = auto()

class RobotURDF(Enum):
    W1V02_1 = os.path.join(model_path, "DexforceW1_v02_1.urdf")
    W1V02_2 = os.path.join(model_path, "DexforceW1_v02_2.urdf")

class RobotCollision(Enum):
    W1V02_1 = os.path.join(model_path, "W1V02_1_collision")
    W1V02_2 = os.path.join(model_path, "W1V02_2_collision")

def get_robot_urdf(robot_type: RobotType) -> str:
    if robot_type == RobotType.W1V02_1:
        return RobotURDF.W1V02_1.value
    elif robot_type == RobotType.W1V02_2:
        return RobotURDF.W1V02_2.value
    else:
        raise ValueError("Unsupported robot type. Please use W1V02_1 or W1V02_2.")
    
def get_robot_mesh_path(robot_type: RobotType) -> str:
    if robot_type == RobotType.W1V02_1:
        return RobotCollision.W1V02_1.value
    elif robot_type == RobotType.W1V02_2:
        return RobotCollision.W1V02_2.value
    else:
        raise ValueError("Unsupported robot type. Please use W1V02_1 or W1V02_2.")