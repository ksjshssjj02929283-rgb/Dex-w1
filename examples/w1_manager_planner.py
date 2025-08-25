import os
import numpy as np
from dexe_robot_client.end_effector_client import EndEffectorType 
from dexe_robot_client.common import W1Version, W1ArmType
from dexe_robot_client.w1 import W1Manager
from dexe_robot_client.w1.path_planner.utils.robot_type import *
from dexe_robot_client.w1.path_planner.utils.mapping_index import *

def test_single_task():
    torso_waypoint = [[0.7854, -1.5708,  0.7853, -0.]]
    head_waypoint = [[0.5, 0.3]]
    left_arm_waypoint = np.deg2rad([[5., -45., -5., -90., 0., -15., 0.]]).tolist()
    left_arm_ee_state = [[0., [0., 50., 50., 50., 50., 50.], 0.5]]
    right_arm_waypoint = np.deg2rad([[-5., 45., 5., 90., 0., 15., 0.]]).tolist()
    right_arm_ee_state = [[0., [100., 100., 0., 0., 100., 100.], 0.5]]

    w1_manager.move_dual_arm_single_task(task_dict={
                                                    "torso": torso_waypoint,
                                                    "torso_speed_ratio": [1.],
                                                    "head": head_waypoint,
                                                    "head_speed_ratio": [1.],
                                                    "left_arm": left_arm_waypoint,
                                                    "left_arm_speed_ratio": [1.],
                                                    "left_arm_ee": left_arm_ee_state,
                                                    "right_arm": right_arm_waypoint,
                                                    "right_arm_speed_ratio": [1.],
                                                    "right_arm_ee": right_arm_ee_state},
                                        speed_ratio=1)
    



if __name__ == "__main__":
    robot_type = RobotType.W1V02_1
    robot_ee_type = EndEffectorType.BRAINCO_HAND
    w1_manager = W1Manager(is_sim=True,
                           arm_type=W1ArmType.ANTHROPOMORPHIC,
                           w1_version=W1Version.V2,
                           end_effector_type=robot_ee_type)
    
    print("load w1 manager done")
    w1_manager.set_home_qpos(np.deg2rad([45, -90, 45, 0, 0, 0,
                                      5, -45, -5, -90, 0, -15, 0,
                                      -5, 45, 5, 90, 0, 15, 0]),
                            save_to_file=False)
    
    w1_manager.set_ee_qpos("left_arm_ee", np.array([0.]*6))
    w1_manager.set_ee_qpos("right_arm_ee", np.array([0.]*6))
    w1_manager.go_home(0.5)  


