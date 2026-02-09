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

# 回原点 + 左手抓胶囊，右手拿杯子 + 左手放胶囊，右手放杯子 + 准备下压把手
task_dict1 = {
    "torso": [[
        0.5410520681182421, -1.2217304763960306, 0.7853981633974483,
        0.03490658503988659
    ]],
    "torso_speed_ratio": [1.],
    "head": [[0.0, -0.6632251157578453]],
    "head_speed_ratio": [1.],
    "left_arm": [[-77.08, 260.39, 328.83, 91.77, 86.44, 170.82],
                 [-244.76, 300.23, 293.88, -175.98, 62.95, -61.24],
                 [-17.02, 374.27, -4.97, -153.92, 2.8, -95.08],
                 [-17.02, 374.27, -4.97, -153.92, 2.8, -95.08],
                 [17.76, 458.95, 53.88, -131.85, 11.79, -81.07]],
    "left_arm_speed_ratio": [1., 1., 1., 1., 1.],
    "left_arm_ee": [[0, [0, 0, 0, 0, 0, 0], 2],
                    [0.5, [45, 49, 50, 50, 50, 50], 0.5],
                    [0.5, [45, 49, 50, 50, 50, 50], 0.5],
                    [0.5, [0, 0, 0, 0, 0, 0], 0.5],
                    [0.5, [0, 0, 0, 0, 0, 0], 0.5]],
    "left_use_collision_check": [True, False, True, True, True],
    "right_arm": [[101.45, 242.18, 331.16, 64.6, -89.06, -149.32],
                  [276.97, 299.58, 310.68, -92.38, -17.4, 5.32],
                  [297.78, 305.9, 25.68, -150.84, 11.48, -6.06],
                  [290.03, 222.24, 56.36, -147.74, 7.29, 7.37],
                  [3.76, 413.07, 82.77, -116.84, 11.57, 38.77]],
    "right_arm_speed_ratio": [1., 1., 1., 1., 1.],
    "right_arm_ee": [[0, [0, 0, 0, 0, 0, 0], 2],
                     [0.5, [100, 50, 40, 40, 40, 40], 0.5],
                     [0.5, [100, 50, 40, 40, 40, 40], 0.5],
                     [0.5, [0, 0, 0, 0, 0, 0], 0.5],
                     [0.5, [0, 0, 0, 0, 0, 0], 0.5]],
    "right_use_collision_check": [True, False, True, True, True]
}

# 调用move_joint
# 下压把手
task_dict2 = {
    "torso": [[
        0.5410520681182421, -1.2217304763960306, 0.7853981633974483,
        0.03490658503988659
    ]],
    "torso_speed_ratio": [1.],
    "head": [[0.0, -0.6632251157578453]],
    "head_speed_ratio": [1.],
    "left_arm": [[-80.93, 280.19, 145.98, 176.0, 25.61, -109.19]],
    "left_arm_speed_ratio": [1.],
    "left_arm_ee": [[0.5, [0, 0, 0, 0, 0, 0], 0.5]],
    "right_arm": [[127.57, 265.49, 134.9, -150.2, -9.96, 64.87]],
    "right_arm_speed_ratio": [1.],
    "right_arm_ee": [[0.5, [0, 0, 0, 0, 0, 0], 0.5]]
}

# 重载把手下压环境
# 准备拿咖啡
task_dict3 = {
    "torso": [[
        0.5410520681182421, -1.2217304763960306, 0.7853981633974483,
        0.03490658503988659
    ]],
    "torso_speed_ratio": [1.],
    "head": [[0.0, -0.6632251157578453]],
    "head_speed_ratio": [1.],
    "left_arm": [[-207.26, 303.41, 227.94, -158.3, 33.16, -53.16]],
    "left_arm_speed_ratio": [1.],
    "left_arm_ee": [[0.5, [0, 0, 0, 0, 0, 0], 0.5]],
    "right_arm": [[302.75, 211.49, 115.47, -136.26, 3.46, 3.21]],
    "right_arm_speed_ratio": [1.],
    "right_arm_ee": [[0.5, [0, 0, 0, 0, 0, 0], 0.5]]
}


def test_multiple_task(task_dict=None,
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

    w1_manager.set_tcp("left_arm", np.eye(4))
    right_tcp_xpos = np.eye(4)
    right_tcp_xpos[:3, :3] = R.from_rotvec([0.0, 0.0, 180.],
                                           degrees=True).as_matrix()
    w1_manager.set_tcp("right_arm", xpos=right_tcp_xpos)

    # w1_manager.set_home_qpos(np.array([0, 0, 0, 0, 0.0, 0.0,
    #                                   0.218, -0.718, 0.563, -2.083, 1.125, -0.501, 0.393,
    #                                   -0.51, 0.851, -0.291, 1.821, 1.187, -0.104, 0.946]),
    #                         save_to_file=False)

    # w1_manager.set_ee_qpos("left_arm_ee", np.array([0.]*6))
    # w1_manager.set_ee_qpos("right_arm_ee", np.array([0.]*6))
    # w1_manager.go_home(1)

    fcl_downward_env_path = f"{DATA_DIR}/environment/coffee_demo_downward.json"
    fcl_upward_env_path = f"{DATA_DIR}/environment/coffee_demo_upward.json"
    planner_downward_env_path = f"{DATA_DIR}/environment/expanded_coffee_demo_downward.json"
    planner_upward_env_path = f"{DATA_DIR}/environment/expanded_coffee_demo_upward.json"

    # 回原点
    print("回原点 + 左手抓胶囊，右手拿杯子 + 左手放胶囊，右手放杯子 + 准备下压把手， 加载把手上提的环境")
    test_multiple_task(task_dict=task_dict1,
                       load_env=True,
                       fcl_env_path=fcl_upward_env_path,
                       planner_env_path=planner_upward_env_path)
    # 下压把手
    print("下压把手")
    move_linear_full_body(task_dict=task_dict2)
    # 重载把手下压环境
    # 准备拿咖啡
    print("重载把手下压环境， 准备拿咖啡")
    test_multiple_task(task_dict=task_dict3,
                       load_env=True,
                       fcl_env_path=fcl_downward_env_path,
                       planner_env_path=planner_downward_env_path)

    embed()
