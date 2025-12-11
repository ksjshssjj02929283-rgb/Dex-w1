import numpy as np
from dexe_robot_client.w1.w1_controller import W1Controller
from dexe_robot_client.end_effector_client import EndEffectorType 
from dexe_robot_client.common import W1Version, W1ArmType
from dexe_robot_client.w1.path_planner.utils.robot_type import *
from dexe_robot_client.w1.path_planner.utils.mapping_index import *
from IPython import embed
import time

def test_w1_controller():
    w1_controller = W1Controller(
        is_sim=False,
        arm_type=W1ArmType.ANTHROPOMORPHIC,
        w1_version=W1Version.V2,
        end_effector_type=EndEffectorType.BRAINCO_HAND
    )

    time.sleep(1000000)
    while True:
        continue


if __name__ == "__main__":
    test_w1_controller()