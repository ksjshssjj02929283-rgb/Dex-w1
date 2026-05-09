'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

import os
import numpy as np
from dexe_robot_client.end_effector_client import EndEffectorType
from dexe_robot_client.common import W1Version, W1ArmType, StatusCode
from dexe_robot_client.w1 import W1Manager
from dexe_robot_client.w1.path_planner.utils.robot_type import *
from dexe_robot_client.common.mapping_index import *
from scipy.spatial.transform import Rotation as R
from IPython import embed
from dexe_robot_client.common.config import DATA_DIR
from dexe_robot_client.w1.kinematics.pin_ik_whole_body import PinIKWholeBody
from dexe_robot_client.w1.utils.unit_conversion import *

from dexe_rp import W1Vis, pos_wxyz_to_homogeneous, Box, Sphere, Mesh, Material, rotation_to_wxyz
import json
import time
from loop_rate_limiters import RateLimiter
import pinocchio as pin
"""
This example is used to test move_line solving by whole body IK with various input task for chess task
"""

chess_board_size = [0.6, 0.6, 0.03]  # 50cm x 50 cm x 3 cm
chess_board_center = [0.5, 0, 0.8]
# yapf: disable
chess_point = [
    [chess_board_center[0]-0.3, -0.2, 0.86], [chess_board_center[0]-0.3, -0.1, 0.86], [chess_board_center[0]-0.3, 0.0, 0.86], [chess_board_center[0]-0.3, 0.1, 0.86], [chess_board_center[0]-0.3, 0.2, 0.86],
    [chess_board_center[0]-0.2, -0.2, 0.86], [chess_board_center[0]-0.2, -0.1, 0.86], [chess_board_center[0]-0.2, 0.0, 0.86], [chess_board_center[0]-0.2, 0.1, 0.86], [chess_board_center[0]-0.2, 0.2, 0.86],
    [chess_board_center[0]-0.1, -0.2, 0.86], [chess_board_center[0]-0.1, -0.1, 0.86], [chess_board_center[0]-0.1, 0.0, 0.86], [chess_board_center[0]-0.1, 0.1, 0.86], [chess_board_center[0]-0.1, 0.2, 0.86],
    [chess_board_center[0], -0.2, 0.86], [chess_board_center[0], -0.1, 0.86], [chess_board_center[0], 0.0, 0.86], [chess_board_center[0], 0.1, 0.86], [chess_board_center[0], 0.2, 0.86],
    [chess_board_center[0]+0.1, -0.2, 0.86], [chess_board_center[0]+0.1, -0.1, 0.86], [chess_board_center[0]+0.1, 0.0, 0.86], [chess_board_center[0]+0.1, 0.1, 0.86], [chess_board_center[0]+0.1, 0.2, 0.86],
    [chess_board_center[0]+0.2, -0.2, 0.86], [chess_board_center[0]+0.2, -0.1, 0.86], [chess_board_center[0]+0.2, 0.0, 0.86], [chess_board_center[0]+0.2, 0.1, 0.86], [chess_board_center[0]+0.2, 0.2, 0.86],
    [chess_board_center[0]+0.3, -0.2, 0.86], [chess_board_center[0]+0.3, -0.1, 0.86], [chess_board_center[0]+0.3, 0.0, 0.86], [chess_board_center[0]+0.3, 0.1, 0.86], [chess_board_center[0]+0.3, 0.2, 0.86]
]
# yapf: enable

point_size = 0.01

task_dict1 = {
    "left_arm": [
        [
            679.9198531530203, 211.4320180584595, 865.1478794238351,
            -96.60238969929269, -0.026788773568079554, -89.9509593855713
        ],
        [
            446.53378545410254, 452.7028675398074, 960.8044457900112,
            -96.60238969929269, -0.026788773568079554, -89.9509593855713
        ],
        [
            576.4796879945455, 275.0487732686348, 895.2109706535736,
            -96.14503296968499, -2.4193587940941716, -111.12878664548232
        ],
        [
            466.53378545410254, 452.7028675398074, 960.8044457900112,
            -96.60238969929269, -0.026788773568079554, -89.9509593855713
        ],
    ],
    "left_arm_speed_ratio": [1., 1., 1., 1.],
    "left_arm_ee": [
        [0, [0, 0, 0, 0, 0, 0], 0],
        [0, [0, 0, 0, 0, 0, 0], 0],
        [0, [0, 0, 0, 0, 0, 0], 0],
        [0, [0, 0, 0, 0, 0, 0], 0],
    ],
    "right_arm": [[
        327.98307847234963, -278.47036078530897, 874.8493253064357,
        -94.09570210016686, 5.182980338841701, -38.68261345931179
    ],],
    "right_arm_speed_ratio": [1.,],
    "right_arm_ee": [[0, [0, 0, 0, 0, 0, 0], 0],],
}

task_dict2 = {
    "left_arm": [
        [
            679.9198531530203, 211.4320180584595, 865.1478794238351,
            -96.60238969929269, -0.026788773568079554, -89.9509593855713
        ],
        [
            446.53378545410254, 452.7028675398074, 960.8044457900112,
            -96.60238969929269, -0.026788773568079554, -89.9509593855713
        ],
        [
            576.4796879945455, 275.0487732686348, 895.2109706535736,
            -96.14503296968499, -2.4193587940941716, -111.12878664548232
        ],
        [
            466.53378545410254, 452.7028675398074, 960.8044457900112,
            -96.60238969929269, -0.026788773568079554, -89.9509593855713
        ],
    ],
    "left_arm_speed_ratio": [1., 1., 1., 1.],
    "left_arm_ee": [
        [0, [0, 0, 0, 0, 0, 0], 0],
        [0, [0, 0, 0, 0, 0, 0], 0],
        [0, [0, 0, 0, 0, 0, 0], 0],
        [0, [0, 0, 0, 0, 0, 0], 0],
    ],
    "right_arm": [],
    "right_arm_speed_ratio": [],
    "right_arm_ee": [],
}

task_dict3 = {
    "left_arm": [],
    "left_arm_speed_ratio": [],
    "left_arm_ee": [],
    "right_arm": [[
        327.98307847234963, -278.47036078530897, 874.8493253064357,
        -94.09570210016686, 5.182980338841701, -38.68261345931179
    ],],
    "right_arm_speed_ratio": [1.,],
    "right_arm_ee": [[0, [0, 0, 0, 0, 0, 0], 0],],
}


def _load_viser_environment(vis):
    scene_handle_list = []
    workobjects_handle_list = []

    chess_board_pose = np.eye(4)
    chess_board_pose[:3, 3] = chess_board_center
    chess_board = Box(chess_board_size[0], chess_board_size[1],
                      chess_board_size[2], chess_board_pose)
    h_chess_board = vis.add_body("/chess_board", chess_board)
    scene_handle_list.append(h_chess_board)

    for i in range(len(chess_point)):
        point_pose = np.eye(4)
        point_pose[:3, 3] = chess_point[i]
        point_sphere = Sphere(point_size, point_pose, Material.RED)
        h_point = vis.add_body(f"/chess_point_{i}", point_sphere)
        scene_handle_list.append(h_point)

    return scene_handle_list, workobjects_handle_list


def load_viser(file_path: str = None):
    vis = W1Vis()
    vis.set_qpos("robot", home_qpos)
    vis.set_ee_pos("left_ee", left_ee_home_qpos)
    vis.set_ee_pos("right_ee", right_ee_home_qpos)

    scene_handle_list, workobjects_handle_list = _load_viser_environment(vis)

    return vis, scene_handle_list, workobjects_handle_list


def viser_execute_traj(vis,
                       trajectory,
                       armlee_trigger=None,
                       armree_trigger=None):
    rate = RateLimiter(frequency=100, warn=True)
    print(f"trajectory length: {len(trajectory)}")
    idx = 0
    lee_idx = 0
    ree_idx = 0
    while True:
        if idx < len(trajectory):
            full_qpos = trajectory[idx]
            vis.set_qpos("robot", full_qpos)
            if armlee_trigger is not None and lee_idx < len(
                    armlee_trigger) and idx == armlee_trigger[lee_idx][0]:
                vis.set_ee_pos("left_ee",
                               np.asarray(armlee_trigger[lee_idx][1]))
                lee_idx += 1
            if armree_trigger is not None and ree_idx < len(
                    armree_trigger) and idx == armree_trigger[ree_idx][0]:
                vis.set_ee_pos("right_ee",
                               np.asarray(armree_trigger[ree_idx][1]))
                ree_idx += 1
            idx += 1
        else:
            break
        rate.sleep()


def test_plan_linear_whole_body_ik(start_qpos,
                                   start_left_arm_ee_qpos,
                                   start_right_arm_ee_qpos,
                                   task_dict,
                                   movement_mode="eager"):
    res, trajectory, armlee_trigger, armree_trigger = \
        w1_manager.plan_linear_whole_body_ik(start_qpos,
                                             start_left_arm_ee_qpos,
                                             start_right_arm_ee_qpos,
                                             task_dict,
                                             use_collision_validator=False,
                                             movement_mode=movement_mode)
    return res, trajectory, armlee_trigger, armree_trigger


if __name__ == "__main__":
    w1_manager = W1Manager(is_sim=True,
                           arm_type=W1ArmType.ANTHROPOMORPHIC,
                           w1_version=W1Version.V2,
                           end_effector_type=EndEffectorType.BRAINCO_HAND)
    if not w1_manager.connect() and not w1_manager.is_enabled():
        raise RuntimeError("Failed to connect to W1 robot.")

    w1_manager.set_tcp("left_arm", np.eye(4))
    right_tcp_xpos = np.eye(4)
    right_tcp_xpos[:3, :3] = R.from_rotvec([0.0, 0.0, 180.],
                                           degrees=True).as_matrix()
    w1_manager.set_tcp("right_arm", xpos=right_tcp_xpos)

    # yapf: disable
    home_qpos = np.array([
        0.785, -1.57, 0.9, 0.0,
        0.0, -0.4,
        -1.57, -0.785, 1.57, -0.785, 0.0, 0.0, 0.0,
        1.57, 0.785, -1.57, 0.785, 0.0, 0.0, 0.0
    ])
    # yapf: enable

    left_ee_home_qpos = np.array([0.0] * 6)
    right_ee_home_qpos = np.array([0.0] * 6)

    w1_manager.set_home_qpos(home_qpos, save_to_file=False)

    w1_manager.set_ee_qpos("left_arm_ee", left_ee_home_qpos)
    w1_manager.set_ee_qpos("right_arm_ee", right_ee_home_qpos)
    w1_manager.go_home(1)

    vis, scene_handle_list, workobjects_handle_list = load_viser()

    input("Press Enter to start linear whole body IK test...")
    time.sleep(1)

    # res, trajectory, armlee_trigger, armree_trigger = test_plan_linear_whole_body_ik(
    #     start_qpos=home_qpos,
    #     start_left_arm_ee_qpos=left_ee_home_qpos,
    #     start_right_arm_ee_qpos=right_ee_home_qpos,
    #     task_dict=task_dict1,
    #     movement_mode="eager")
    # if res == StatusCode.SUCCESS:
    #     viser_execute_traj(vis, trajectory, armlee_trigger, armree_trigger)

    #     left_arm_user_pose = xpos_to_user_pose(
    #         w1_manager.get_xpos("left_arm", trajectory[-1], False)[1])
    #     right_arm_user_pose = xpos_to_user_pose(
    #         w1_manager.get_xpos("right_arm", trajectory[-1], False)[1])
    #     left_arm_target_pose = task_dict1["left_arm"][-1]
    #     right_arm_target_pose = task_dict1["right_arm"][-1]
    #     print(f"left_arm_user_pose: {left_arm_user_pose}")
    #     print(f"left_arm_target_pose: {left_arm_target_pose}")
    #     print(f"right_arm_user_pose: {right_arm_user_pose}")
    #     print(f"right_arm_target_pose: {right_arm_target_pose}")
    # else:
    #     print("Plan failed for task_dict1.")

    res, trajectory, armlee_trigger, armree_trigger = test_plan_linear_whole_body_ik(
        start_qpos=home_qpos,
        start_left_arm_ee_qpos=left_ee_home_qpos,
        start_right_arm_ee_qpos=right_ee_home_qpos,
        task_dict=task_dict2,
        movement_mode="eager")
    if res == StatusCode.SUCCESS:
        viser_execute_traj(vis, trajectory, armlee_trigger, armree_trigger)
    else:
        print("Plan failed for task_dict2.")

    print("===============================")

    res, trajectory, armlee_trigger, armree_trigger = test_plan_linear_whole_body_ik(
        start_qpos=home_qpos,
        start_left_arm_ee_qpos=left_ee_home_qpos,
        start_right_arm_ee_qpos=right_ee_home_qpos,
        task_dict=task_dict3,
        movement_mode="eager")
    if res == StatusCode.SUCCESS:
        viser_execute_traj(vis, trajectory, armlee_trigger, armree_trigger)
    else:
        print("Plan failed for task_dict3.")

    embed()
