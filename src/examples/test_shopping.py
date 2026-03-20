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
from dexe_robot_client.w1.kinematics.pin_ik_whole_body import PinIKWholeBody
from dexe_robot_client.w1.utils.unit_conversion import *

from dexe_rp import W1Vis, pos_wxyz_to_homogeneous, Box, Sphere, Mesh, Material, rotation_to_wxyz
import json
from loop_rate_limiters import RateLimiter
import pinocchio as pin
"""
This example is used to test move_line solving whole_body IK susccess rate for shopping task
"""

task_dict1 = {
    "torso": [[
        0.5410520681182421, -1.2217304763960306, 0.7853981633974483,
        0.03490658503988659
    ]],
    "torso_speed_ratio": [1.],
    "head": [[0.0, -0.6632251157578453]],
    "head_speed_ratio": [1.],
    "left_arm": [[
        134.52625191632674, -485.6220971264064, 1441.7347271938338,
        -78.28236256541705, -85.89972337753717, 168.6538795628157
    ],],
    "left_arm_speed_ratio": [1.0,],
    "left_arm_ee": [[0.5, [100, 100, 100, 100, 100, 100], 0.5],],
    "right_arm": [[
        -231.33684895211266, -486.7413152669804, 1447.1942555385267,
        -138.9875076274057, 87.07150540691981, 153.0591514943005
    ],],
    "right_arm_speed_ratio": [1.0,],
    "right_arm_ee": [[0.5, [100, 100, 100, 100, 100, 100], 0.5],],
}

task_dict2 = {
    "torso": [[
        0.5410520681182421, -1.2217304763960306, 0.7853981633974483,
        0.03490658503988659
    ]],
    "torso_speed_ratio": [1.],
    "head": [[0.0, -0.6632251157578453]],
    "head_speed_ratio": [1.],
    "left_arm": [[
        509.41361106962336, 123.49248553246156, 986.25745576644,
        -15.349111810471891, -85.89972337753721, 168.65387956281572
    ],],
    "left_arm_speed_ratio": [1.0,],
    "left_arm_ee": [[0.5, [0, 0, 0, 0, 0, 0], 0.5],],
    "right_arm": [[
        459.01926804605176, -76.80076488526362, 968.8949295208046,
        147.54807576754598, 87.07150540691978, 153.05915149430027
    ],],
    "right_arm_speed_ratio": [1.0,],
    "right_arm_ee": [[0.5, [0, 0, 0, 0, 0, 0], 0.5],],
}

task_dict3 = {
    "torso": [[
        0.5410520681182421, -1.2217304763960306, 0.7853981633974483,
        0.03490658503988659
    ]],
    "torso_speed_ratio": [1.],
    "head": [[0.0, -0.6632251157578453]],
    "head_speed_ratio": [1.],
    "left_arm": [[
        -128.2095839224092, 467.8688804915105, 661.9443791014563,
        81.75897312216229, -85.89972337753721, 168.65387956281472
    ],],
    "left_arm_speed_ratio": [1.0,],
    "left_arm_ee": [[0.5, [100, 100, 100, 100, 100, 100], 0.5],],
    "right_arm": [[
        153.59061621132926, 487.6028934023632, 682.3681445107694,
        61.38069533825851, 87.07150540691984, 153.05915149430143
    ],],
    "right_arm_speed_ratio": [1.0,],
    "right_arm_ee": [[0.5, [100, 100, 100, 100, 100, 100], 0.5],],
}


def _load_viser_environment(vis):
    scene_handle_list = []
    workobjects_handle_list = []

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


def test_plan_linear_whole_body_ik(start_qpos, start_left_arm_ee_qpos,
                                   start_right_arm_ee_qpos, task_dict):
    res, trajectory, armlee_trigger, armree_trigger = \
        w1_manager.plan_linear_whole_body_ik(start_qpos,
                                             start_left_arm_ee_qpos,
                                             start_right_arm_ee_qpos,
                                             task_dict,
                                             cartesian_step=0.02,
                                             use_collision_validator=False,
                                             movement_mode="sync_point_by_point")
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
        -1.57, -0.785, 1.57, -0.785, -1.57, 0.0, 0.0,
        1.57, 0.785, -1.57, 0.785, 1.57, 0.0, 0.0
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

    res, trajectory, armlee_trigger, armree_trigger = test_plan_linear_whole_body_ik(
        start_qpos=home_qpos,
        start_left_arm_ee_qpos=left_ee_home_qpos,
        start_right_arm_ee_qpos=right_ee_home_qpos,
        task_dict=task_dict1)
    viser_execute_traj(vis, trajectory, armlee_trigger, armree_trigger)

    left_arm_user_pose = xpos_to_user_pose(
        w1_manager.get_xpos("left_arm", trajectory[-1], False)[1])
    right_arm_user_pose = xpos_to_user_pose(
        w1_manager.get_xpos("right_arm", trajectory[-1], False)[1])
    left_arm_target_pose = task_dict1["left_arm"][-1]
    right_arm_target_pose = task_dict1["right_arm"][-1]
    print(f"left_arm_user_pose: {left_arm_user_pose}")
    print(f"left_arm_target_pose: {left_arm_target_pose}")
    print(f"right_arm_user_pose: {right_arm_user_pose}")
    print(f"right_arm_target_pose: {right_arm_target_pose}")

    res, trajectory, armlee_trigger, armree_trigger = test_plan_linear_whole_body_ik(
        start_qpos=trajectory[-1],
        start_left_arm_ee_qpos=left_ee_home_qpos,
        start_right_arm_ee_qpos=right_ee_home_qpos,
        task_dict=task_dict2)
    viser_execute_traj(vis, trajectory, armlee_trigger, armree_trigger)

    left_arm_user_pose = xpos_to_user_pose(
        w1_manager.get_xpos("left_arm", trajectory[-1], False)[1])
    right_arm_user_pose = xpos_to_user_pose(
        w1_manager.get_xpos("right_arm", trajectory[-1], False)[1])
    left_arm_target_pose = task_dict2["left_arm"][-1]
    right_arm_target_pose = task_dict2["right_arm"][-1]
    print(f"left_arm_user_pose: {left_arm_user_pose}")
    print(f"left_arm_target_pose: {left_arm_target_pose}")
    print(f"right_arm_user_pose: {right_arm_user_pose}")
    print(f"right_arm_target_pose: {right_arm_target_pose}")

    res, trajectory, armlee_trigger, armree_trigger = test_plan_linear_whole_body_ik(
        start_qpos=trajectory[-1],
        start_left_arm_ee_qpos=left_ee_home_qpos,
        start_right_arm_ee_qpos=right_ee_home_qpos,
        task_dict=task_dict3)
    viser_execute_traj(vis, trajectory, armlee_trigger, armree_trigger)

    left_arm_user_pose = xpos_to_user_pose(
        w1_manager.get_xpos("left_arm", trajectory[-1], False)[1])
    right_arm_user_pose = xpos_to_user_pose(
        w1_manager.get_xpos("right_arm", trajectory[-1], False)[1])
    left_arm_target_pose = task_dict3["left_arm"][-1]
    right_arm_target_pose = task_dict3["right_arm"][-1]
    print(f"left_arm_user_pose: {left_arm_user_pose}")
    print(f"left_arm_target_pose: {left_arm_target_pose}")
    print(f"right_arm_user_pose: {right_arm_user_pose}")
    print(f"right_arm_target_pose: {right_arm_target_pose}")

    embed()
