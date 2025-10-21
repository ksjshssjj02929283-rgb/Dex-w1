import os
import numpy as np
from dexe_robot_client.end_effector_client import EndEffectorType 
from dexe_robot_client.common import W1Version, W1ArmType
from dexe_robot_client.w1 import W1Manager
from dexe_robot_client.w1.path_planner.utils.robot_type import *
from dexe_robot_client.common.mapping_index import *
from IPython import embed

def test_single_task(task_dict=None):

    w1_manager.move_dual_arm_single_task(task_dict=task_dict,
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
                                      0.218, -0.718, 0.563, -2.083, 1.125, -0.501, 0.393, 
                                      -0.51, 0.851, -0.291, 1.821, 1.187, -0.104, 0.946]),
                            save_to_file=False)
    
    w1_manager.set_ee_qpos("left_arm_ee", np.array([0.]*6))
    w1_manager.set_ee_qpos("right_arm_ee", np.array([0.]*6))
    w1_manager.go_home(1) 

    # input("Press Enter to continue to next test...")
    test_single_arm()


    task_dict1 = {
                "torso": [np.deg2rad([31, -70, 45, 2]).tolist()],
                "torso_speed_ratio": [1.],
                "head": [np.deg2rad([-5, -25]).tolist()],
                "head_speed_ratio": [1.],
                "left_arm": [[-252.768, 320.528, 260.741, 54.061, 84.122, 172.988]],
                "left_arm_speed_ratio": [1., 1.],
                "left_arm_ee": [[0.5, [39, 98, 60, 57, 98, 99], 0.5]],
                "right_arm": [[330.246, 97.433, 256.061, -166.055, 57.545, -57.878]],
                "right_arm_speed_ratio": [1., 1.],
                "right_arm_ee": [[0.5, [0, 0, 0, 0, 0, 0], 0.5]]
                }
    
    task_dict2 = {
                "torso": [np.deg2rad([31, -70, 45, 2]).tolist()],
                "torso_speed_ratio": [1.],
                "head": [np.deg2rad([-5, -25]).tolist()],
                "head_speed_ratio": [1.],
                "left_arm": [[-115.077, 325.576, 103.792, -159.673, 42.378, -29.296]],
                "left_arm_speed_ratio": [1., 1.],
                "left_arm_ee": [[0.5, [54, 98, 75, 75, 98, 99], 0.5]],
                "right_arm": [[330.246, 97.433, 256.061, -166.055, 57.545, -57.878]],
                "right_arm_speed_ratio": [1., 1.],
                "right_arm_ee": [[0.5, [0, 0, 0, 0, 0, 0], 0.5]]
                }
    
    task_dict3 = {
                "torso": [np.deg2rad([31, -70, 45, 2]).tolist()],
                "torso_speed_ratio": [1.],
                "head": [np.deg2rad([0, -15]).tolist()],
                "head_speed_ratio": [1.],
                "left_arm": [[92.089, 321.906, 56.05, -129.055, 54.999, -47.585]],
                "left_arm_speed_ratio": [1., 1.],
                "left_arm_ee": [[0.5, [54, 98, 75, 75, 98, 99], 0.5]],
                "right_arm": [[110.044, 219.781, 247.63, -116.803, -0.39, 25.13]],
                "right_arm_speed_ratio": [1., 1.],
                "right_arm_ee": [[0.5, [0, 0, 0, 0, 0, 0], 0.5]]
                }
    
    task_dict4 = {
                "torso": [np.deg2rad([38, -80, 50, -10]).tolist()],
                "torso_speed_ratio": [1.],
                "head": [np.deg2rad([10, -20]).tolist()],
                "head_speed_ratio": [1.],
                "left_arm": [[-263.866, 210.769, 300.563, -62.048, 3.9, 4.359]],
                "left_arm_speed_ratio": [1., 1.],
                "left_arm_ee": [[0.5, [0, 99, 54, 54, 45, 37], 0.5]],
                "right_arm": [[-96.409, 321.062, 217.857, -127.961, -36.509, 69.083]],
                "right_arm_speed_ratio": [1., 1.],
                "right_arm_ee": [[0.5, [0, 0, 0, 0, 0, 0], 0.5]]
                }
    
    task_dict5 = {
                "torso": [np.deg2rad([36, -80, 50, -45]).tolist()],
                "torso_speed_ratio": [1.],
                "head": [np.deg2rad([0, -15]).tolist()],
                "head_speed_ratio": [1.],
                "left_arm": [[-302.766, 243.229, 190.295, -85.607, 4.906, 7.884]],
                "left_arm_speed_ratio": [1., 1.],
                "left_arm_ee": [[0.5, [0, 99, 52, 54, 51, 48], 0.5]],
                "right_arm": [[84.635, 349.314, 0.621, -167.908, -44.333, 92.44]],
                "right_arm_speed_ratio": [1., 1.],
                "right_arm_ee": [[0.5, [0, 0, 0, 0, 0, 0], 0.5]]
                }
    
    task_dict6 = {
                "torso": [np.deg2rad([36, -80, 50, -45]).tolist()],
                "torso_speed_ratio": [1.],
                "head": [np.deg2rad([0, -15]).tolist()],
                "head_speed_ratio": [1.],
                "left_arm": [[-277.643, 260.611, 120.996, -72.456, -0.784, -8.71]],
                "left_arm_speed_ratio": [1., 1.],
                "left_arm_ee": [[0.5, [0, 99, 0, 0, 0, 0], 0.5]],
                "right_arm": [[180.632, 222.18, 257.174, -134.501, -30.037, -3.081]],
                "right_arm_speed_ratio": [1., 1.],
                "right_arm_ee": [[0.5, [0, 0, 0, 0, 0, 0], 0.5]]
                }
    
    test_single_task(task_dict=task_dict1)
    input("Press Enter to continue to next test...")
    test_single_task(task_dict=task_dict2)
    input("Press Enter to continue to next test...")
    test_single_task(task_dict=task_dict3)
    input("Press Enter to continue to next test...")    
    test_single_task(task_dict=task_dict4)
    input("Press Enter to continue to next test...")
    test_single_task(task_dict=task_dict5)
    input("Press Enter to continue to next test...")
    test_single_task(task_dict=task_dict6)
    input("Press Enter to exit...")
    embed()

    


