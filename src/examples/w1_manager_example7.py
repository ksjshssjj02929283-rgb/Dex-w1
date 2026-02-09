'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

import os
import numpy as np
from dexe_robot_client.end_effector_client import EndEffectorType
from dexe_robot_client.common import W1Version, W1ArmType
from dexe_robot_client.w1 import W1Manager
from dexe_robot_client.w1.path_planner.utils.robot_type import *
from dexe_robot_client.common.mapping_index import *
from scipy.spatial.transform import Rotation as R
from IPython import embed
from dexe_robot_client.common.config import DATA_DIR

# 回原点
task_dict1 = {
    "torso": [[0.785, -1.57, 0.785, 0.03490658503988659]],
    "torso_speed_ratio": [1.],
    "head": [[0.0, -0.6632251157578453]],
    "head_speed_ratio": [1.],
    "left_arm": [
        [-50.5, 179.04, 402.42, 41.54, 71.94, 128.49],
        [-148.43, 259.55, 275.99, 176.35, 76.06, -52.83],
        [-200.71, 270.69, 285.77, 157.1, 66.74, -85.37],
    ],
    "left_arm_speed_ratio": [
        1.,
        1.,
        1.,
    ],
    "left_arm_ee": [
        [0.5, [0, 0, 0, 0, 0, 0], 0.5],
        [0.2, [20, 50, 30, 30, 30, 30], 0.3],
        [0.2, [20, 100, 50, 50, 50, 50], 0.3],
    ],
    "left_use_collision_check": [
        True,
        True,
        False,
    ],
    "right_arm": [
        [71.25, 217.1, 373.48, 96.79, -81.78, -177.35],
        [260.1314, 294.7001, 280.4634, -86.3459, 6.955, 0.6132],
        [260.5308, 361.9915, 289.7171, -86.3462, 6.9514, 0.6142],
        [123.3089, 389.5347, 318.1869, -105.64, 4.12, 4.89],
    ],
    "right_arm_speed_ratio": [1., 1., 1., 1.],
    "right_arm_ee": [
        [0.5, [0, 0, 0, 0, 0, 0], 0.5],
        [0.2, [0, 100, 40, 40, 40, 40], 0.3],
        [0.0, [0, 100, 40, 40, 40, 40], 0.0],
        [0.2, [0, 99, 60, 60, 60, 60], 0.3],
    ],
    "right_use_collision_check": [True, True, False, False]
}

task_dict2 = {
    "torso": [[0.785, -1.57, 0.785, 0.03490658503988659]],
    "torso_speed_ratio": [1.],
    "head": [[0.0, -0.6632251157578453]],
    "head_speed_ratio": [1.],
    "left_arm": [
        [-170.37, 288.35, 284.77, 160.01, 66.08, -85.99],
        [9.76, 367.19, 9.32, -153.44, 17.64, -91.9],
        [-39.84, 390.27, 36.36, -141.57, 11.12, -40.43],
    ],
    "left_arm_speed_ratio": [1., 1., 1.],
    "left_arm_ee": [
        [0.0, [20, 100, 50, 50, 50, 50], 0.0],
        [0.0, [20, 99, 49, 50, 50, 50], 0.0],
        [0.2, [10, 100, 30, 30, 30, 30], 0.3],
    ],
    "left_use_collision_check": [False, True, False],
    "right_arm": [
        [270.1149, 380.2195, 313.8241, -105.6391, 4.12, 4.8892],
        [235.15, 275.23, 53.12, -152.11, 1.04, -4.34],
        [249.5474, 286.3648, 4.897, -136.9669, 3.4318, 1.7548],
    ],
    "right_arm_speed_ratio": [1., 1., 1.],
    "right_arm_ee": [
        [0.0, [0, 99, 60, 60, 60, 60], 0.0],
        [0.0, [0, 99, 60, 60, 60, 60], 0.0],
        [0.2, [0, 100, 20, 20, 20, 20], 0.3],
    ],
    "right_use_collision_check": [False, True, False]
}

task_dict3 = {
    "torso": [[0.785, -1.57, 0.785, 0.03490658503988659]],
    "torso_speed_ratio": [1.],
    "head": [[0.0, -0.6632251157578453]],
    "head_speed_ratio": [1.],
    "left_arm": [[-12.99, 329.39, -0.8, -150.06, -6.2, -42.88],
                 [52.59, 513.68, 71.56, -135.01, -15.82, -17.95]],
    "left_arm_speed_ratio": [1., 1.],
    "left_arm_ee": [[0.0, [10, 100, 30, 30, 30, 30], 0.0],
                    [0.2, [0, 0, 0, 0, 0, 0], 0.3]],
    "left_use_collision_check": [False, True],
    "right_arm": [[252.0, 273.86, 173.45, -140.2, -10.99, 1.43],
                  [-41.94, 508.91, 76.45, -113.91, 6.08, 19.82]],
    "right_arm_speed_ratio": [1., 1.],
    "right_arm_ee": [[0.0, [20, 19, 19, 20, 19, 19], 0.0],
                     [0.2, [0, 0, 0, 0, 0, 0], 0.3]],
    "right_use_collision_check": [False, True]
}

task_dict4 = {
    "torso": [[0.785, -1.57, 0.785, 0.03490658503988659]],
    "torso_speed_ratio": [1.],
    "head": [[0.0, -0.6632251157578453]],
    "head_speed_ratio": [1.],
    "left_arm": [[-36.79, 327.46, 141.09, 172.6, 4.56, -69.86],],
    "left_arm_speed_ratio": [1.],
    "left_arm_ee": [[0.0, [0, 0, 0, 0, 0, 0], 0.0],],
    "left_use_collision_check": [False],
    "right_arm": [[74.99, 254.48, 151.2, -162.2, -17.08, 55.45],],
    "right_arm_speed_ratio": [1.],
    "right_arm_ee": [[0.0, [0, 0, 0, 0, 0, 0], 0.0],],
    "right_use_collision_check": [False]
}
task_dict5 = {
    "torso": [[0.785, -1.57, 0.785, 0.03490658503988659]],
    "torso_speed_ratio": [1.],
    "head": [[0.0, -0.6632251157578453]],
    "head_speed_ratio": [1.],
    "left_arm": [[-5.6091, 256.3782, 171.7105, 172.6101, 4.5616, -69.8504]],
    "left_arm_speed_ratio": [1.],
    "left_arm_ee": [[0.0, [0, 0, 0, 0, 0, 0], 0.0]],
    "left_use_collision_check": [False],
    "right_arm": [[103.0304, 235.4964, 261.6747, -162.1885, -17.0788, 55.4408]],
    "right_arm_speed_ratio": [1.],
    "right_arm_ee": [[0.0, [0, 0, 0, 0, 0, 0], 0.0]],
    "right_use_collision_check": [False]
}

task_dict6 = {
    "torso": [[0.785, -1.57, 0.785, 0.03490658503988659]],
    "torso_speed_ratio": [1.],
    "head": [[0.0, -0.6632251157578453]],
    "head_speed_ratio": [1.],
    "left_arm": [[-154.94, 394.32, 297.24, -121.17, 12.15, -26.94]],
    "left_arm_speed_ratio": [1.],
    "left_arm_ee": [[0.2, [0, 0, 0, 0, 0, 0], 0.5]],
    "left_use_collision_check": [True],
    "right_arm": [[255.94, 240.6, 178.67, -144.42, 2.49, -5.67]],
    "right_arm_speed_ratio": [1.],
    "right_arm_ee": [[0.0, [0, 0, 0, 0, 0, 0], 0.0]],
    "right_use_collision_check": [True]
}


def test_single_task(task_dict=None,
                     load_env=True,
                     fcl_env_path=None,
                     planner_env_path=None):
    if load_env:
        w1_manager.load_fcl_environment(fcl_env_path)
        w1_manager.load_planner_environment(planner_env_path)
    w1_manager.move_dual_arm_multiple_task(task_dict=task_dict, speed_ratio=1)


def move_linear_full_body(task_dict=None):
    w1_manager.move_linear_full_body(waypoints_dict=task_dict, speed_ratio=1)


if __name__ == "__main__":
    robot_type = RobotType.W1V02_1
    robot_ee_type = EndEffectorType.BRAINCO_HAND
    # robot_ee_type = None
    w1_manager = W1Manager(is_sim=True,
                           arm_type=W1ArmType.ANTHROPOMORPHIC,
                           w1_version=W1Version.V2,
                           end_effector_type=robot_ee_type)
    if not w1_manager.connect() and not w1_manager.is_enabled():
        raise RuntimeError("Failed to connect to W1 robot.")

    # yapf: disable
    w1_manager.set_home_qpos(np.deg2rad([0, 0, 0, 0, 0, 0,
                                        5, -20, -5, -30, 0, -15, 0,
                                        -5, 20, 5, 30, 0, 15, 0]),
                            save_to_file=False)
    # yapf: enable
    w1_manager.go_home(1.)

    w1_manager.set_tcp("left_arm", np.eye(4))
    right_tcp_xpos = np.eye(4)
    right_tcp_xpos[:3, :3] = R.from_rotvec([0.0, 0.0, 180.],
                                           degrees=True).as_matrix()
    w1_manager.set_tcp("right_arm", xpos=right_tcp_xpos)

    # w1_manager.enable_viser()

    fcl_downward_env_path = f"{DATA_DIR}/environment/coffee_demo_downward.json"
    fcl_upward_env_path = f"{DATA_DIR}/environment/coffee_demo_upward.json"
    planner_downward_env_path = f"{DATA_DIR}/environment/expanded_coffee_demo_downward.json"
    planner_upward_env_path = f"{DATA_DIR}/environment/expanded_coffee_demo_upward.json"

    test_single_task(task_dict=task_dict1,
                     load_env=True,
                     fcl_env_path=fcl_upward_env_path,
                     planner_env_path=planner_upward_env_path)
    # w1_manager.add_control_part()
    # w1_manager.attach_cup_in_viser()
    w1_manager.attach_object(obj_name="cup", which_arm="right")
    test_single_task(task_dict=task_dict2, load_env=False)
    # w1_manager.detach_cup_in_viser()
    w1_manager.detach_object(obj_name="cup", which_arm="right")
    test_single_task(task_dict=task_dict3, load_env=False)
    test_single_task(task_dict=task_dict4, load_env=False)
    # w1_manager.reload_coffee_machine_in_viser()
    test_single_task(task_dict=task_dict5, load_env=False)
    test_single_task(task_dict=task_dict6,
                     load_env=True,
                     fcl_env_path=fcl_downward_env_path,
                     planner_env_path=planner_downward_env_path)
