import os
import numpy as np
from dexe_robot_client.end_effector_client import EndEffectorType 
from dexe_robot_client.common import W1Version, W1ArmType
from dexe_robot_client.w1 import W1Manager
from dexe_robot_client.w1.path_planner.utils.robot_type import *
from dexe_robot_client.w1.path_planner.utils.mapping_index import *
from IPython import embed

def test_single_task():
    torso_waypoint = [[0.7853981633974483, -1.5707963267948966, 0.7853981633974483, 0.03490658503988659]]
    head_waypoint = [[0.4188790204786391, -0.6108652381980153]]
    left_arm_waypoint = [[-176.73, 190.87, 72.72, -174.17, 16.13, -98.1]]
    left_arm_ee_state = [[0., [0., 50., 50., 50., 50., 50.], 0.5]]
    # left_arm_ee_state = []
    right_arm_waypoint = [[217.868, 215.12, 216.22, -169.87, 45.22, -64.85]]
    right_arm_ee_state = [[0., [100., 100., 0., 0., 100., 100.], 0.5]]
    # right_arm_ee_state = []

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
    # embed()
    

def test_single_arm():
    torso_waypoint = [[0.7854, -1.5708,  0.7853, -0.]]
    head_waypoint = [[0.5, 0.3]]
    left_arm_waypoint = [[-176.73, 190.87, 72.72, -174.17, 16.13, -98.1],
                         [-174.093, 337.351, 396.462, 72.72, 75.11, -170.8]]
    left_arm_ee_state = [[0.5, [0., 50., 50., 50., 50., 50.], 0.5],
                         [0.5, [0., 50., 50., 50., 50., 50.], 0.5]]
    # right_arm_waypoint = [[217.868, 215.12, 216.22, -169.87, 45.22, -64.85],
    #                       [187.136, 244.01, 228.763, -170.73, 44.71, -61.5]]
    # right_arm_ee_state = [[0.5, [100., 100., 0., 0., 100., 100.], 0.5],
    #                       [0.5, [0., 50., 50., 50., 50., 50.], 0.5]]

    right_arm_waypoint = []
    right_arm_ee_state = []
    
    w1_manager.move_single_arm(task_dict={
                                        "torso": torso_waypoint,
                                        "torso_speed_ratio": [1.],
                                        "head": head_waypoint,
                                        "head_speed_ratio": [1.],
                                        "left_arm": left_arm_waypoint,
                                        "left_arm_speed_ratio": [1., 1.],
                                        "left_arm_ee": left_arm_ee_state,
                                        "right_arm": right_arm_waypoint,
                                        "right_arm_speed_ratio": [1., 1.],
                                        "right_arm_ee": right_arm_ee_state},
                                        speed_ratio=1)
    



if __name__ == "__main__":
    robot_type = RobotType.W1V02_1
    robot_ee_type = EndEffectorType.BRAINCO_HAND
    # robot_ee_type = None
    w1_manager = W1Manager(is_sim=True,
                           arm_type=W1ArmType.ANTHROPOMORPHIC,
                           w1_version=W1Version.V2,
                           end_effector_type=robot_ee_type,
                           use_rlia=False)
    if not w1_manager.connect():
        raise RuntimeError("Failed to connect to W1 robot.")
    
    w1_manager.set_tcp("left_arm", np.eye(4))
    w1_manager.set_tcp("right_arm", np.array([[-1,0,0,0],
                                              [0,-1,0,0],
                                              [0,0,1,0],
                                              [0,0,0,1]]))


    print("load w1 manager done")
    w1_manager.set_home_qpos(np.array([0, 0, 0, 0, 0.0, 0.0, 
                                      -np.pi / 2, -np.pi / 4, np.pi / 2, -np.pi / 4, -np.pi / 2, np.pi / 5, 0.0, 
                                      np.pi / 2, np.pi / 4, -np.pi / 2, np.pi / 4, np.pi / 2, -np.pi / 5, 0.0]),
                            save_to_file=False)
    
    w1_manager.set_ee_qpos("left_arm_ee", np.array([0.]*6))
    w1_manager.set_ee_qpos("right_arm_ee", np.array([0.]*6))
    w1_manager.go_home(1) 

    test_single_task()
    input("Press Enter to continue to next test...")
    test_single_arm()


