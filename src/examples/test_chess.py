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
"""
This example is used to test move_line solving SRS IK susccess rate for chess task
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

    urdf_path = "/home/young/dexe_robot_client_data/assets/Model/W1_V2_A/W1_V2_A.urdf"
    ik_solver = PinIKWholeBody(urdf_path,
                               track_type="standard",
                               cost_type="standard")

    cur_qpos = vis.get_qpos()
    cur_arml_xpos = vis.w1_kine.get_fk("left_arm", cur_qpos)[1]
    cur_armr_xpos = vis.w1_kine.get_fk("right_arm", cur_qpos)[1]

    h_target1 = vis.server.scene.add_transform_controls(
        name="/target1",
        scale=0.2,
        position=cur_arml_xpos[:3, 3],
        wxyz=rotation_to_wxyz(cur_arml_xpos[:3, :3]))
    h_target2 = vis.server.scene.add_transform_controls(
        name="/target2",
        scale=0.2,
        position=cur_armr_xpos[:3, 3],
        wxyz=rotation_to_wxyz(cur_armr_xpos[:3, :3]))

    T_chassis_larm = vis.w1_kine.get_fk("left_arm", cur_qpos)[1]
    T_chassis_rarm = vis.w1_kine.get_fk("right_arm", cur_qpos)[1]

    T_world_target1 = pos_wxyz_to_homogeneous(
        cur_arml_xpos[:3, 3], rotation_to_wxyz(cur_arml_xpos[:3, :3]))
    T_target_larm = np.linalg.inv(T_world_target1) @ T_chassis_larm
    T_world_target2 = pos_wxyz_to_homogeneous(
        cur_armr_xpos[:3, 3], rotation_to_wxyz(cur_armr_xpos[:3, :3]))
    T_target_rarm = np.linalg.inv(T_world_target2) @ T_chassis_rarm

    armlee_T_armltcp = np.identity(4)
    armree_T_armrtcp = np.identity(4)
    armree_T_armrtcp[:3, :3] = R.from_rotvec([0.0, 0.0, 180.],
                                             degrees=True).as_matrix()
    
    focus_center = np.array([0.6, 0.0, 0.8])

    rate = RateLimiter(frequency=100, warn=True)
    while True:
        T_world_target1 = pos_wxyz_to_homogeneous(h_target1.position,
                                                  h_target1.wxyz)
        T_world_target2 = pos_wxyz_to_homogeneous(h_target2.position,
                                                  h_target2.wxyz)

        cur_qpos = vis.get_qpos()
        T_world_larm = T_world_target1 @ T_target_larm
        T_world_rarm = T_world_target2 @ T_target_rarm

        res, result_qpos = ik_solver.solve_ik(left_ee_target=T_world_larm,
                                              right_ee_target=T_world_rarm,
                                              cur_qpos=cur_qpos,
                                              focus_point=focus_center)

        world_T_armltcp = T_world_larm @ armlee_T_armltcp
        world_T_armrtcp = T_world_rarm @ armree_T_armrtcp
        arml_tcp_user_pose = xpos_to_user_pose(world_T_armltcp)
        armr_tcp_user_pose = xpos_to_user_pose(world_T_armrtcp)

        if res:
            vis.set_qpos("robot", result_qpos)
            print("qpos: ", np.round(result_qpos, 3).tolist())
            print("left arm tcp pose:", arml_tcp_user_pose)
            print("right arm tcp pose:", armr_tcp_user_pose)
            print("-----")

            rate.sleep()
        else:
            continue

    embed()
