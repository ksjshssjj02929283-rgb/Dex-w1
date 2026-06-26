'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

import os
import time
import numpy as np
from IPython import embed

from dexe_robot_client.w1.kinematics.whole_body_ik import *
from dexe_rp import W1Vis, pos_wxyz_to_homogeneous, Box, Sphere, Mesh, Material, rotation_to_wxyz
from loop_rate_limiters import RateLimiter

LOCKED_LEGS = False


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


if __name__ == "__main__":
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

    vis, scene_handle_list, workobjects_handle_list = load_viser()

    urdf_path = "/home/young/dexe_data_cache/v023/w1/robot.urdf"
    tracking_frame_config = TrackingFrameConfig.init_from_urdf(
        robot="W1V02X", urdf_path=urdf_path)
    joint_name_config = JointNameConfig.init_from_robot(robot="W1V02X")
    if LOCKED_LEGS:
        freezed_joint_dict = {"ANKLE": 0.785, "KNEE": -1.57, "BUTTOCK": 0.9}
    else:
        freezed_joint_dict = None
    ik_solver = WholeBodyIK(urdf_path=urdf_path,
                            tracking_frame_config=tracking_frame_config,
                            joint_name_config=joint_name_config,
                            solver_options=IpoptOptions(),
                            weight_config=WeightConfig(),
                            posture_config=PostureConfig(),
                            freezed_joint_dict=freezed_joint_dict)

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

    eye_focus_point = np.array([0.6, 0.0, 0.8])

    rate = RateLimiter(frequency=100, warn=True)

    input("Press Enter to start the IK control loop...")
    while True:
        T_world_target1 = pos_wxyz_to_homogeneous(h_target1.position,
                                                  h_target1.wxyz)
        T_world_target2 = pos_wxyz_to_homogeneous(h_target2.position,
                                                  h_target2.wxyz)

        cur_qpos = vis.get_qpos()
        T_world_larm = T_world_target1 @ T_target_larm
        T_world_rarm = T_world_target2 @ T_target_rarm

        if LOCKED_LEGS:
            ref_qpos = cur_qpos[3:]
        else:
            ref_qpos = cur_qpos

        start_time = time.time()
        res, result_qpos = ik_solver.solve_ik(left_arm_target=T_world_larm,
                                              right_arm_target=T_world_rarm,
                                              ref_qpos=ref_qpos,
                                              upper_body_leaning_threshold=0.0,
                                              eye_focus_point=eye_focus_point,
                                              is_strict_solution=False)
        print(f"IK solve time: {time.time() - start_time:.4f} seconds")

        # world_T_armltcp = T_world_larm @ armlee_T_armltcp
        # world_T_armrtcp = T_world_rarm @ armree_T_armrtcp
        # arml_tcp_user_pose = xpos_to_user_pose(world_T_armltcp)
        # armr_tcp_user_pose = xpos_to_user_pose(world_T_armrtcp)

        if LOCKED_LEGS:
            freezed_joint_angle = cur_qpos[:3]
            result_qpos = np.concatenate([freezed_joint_angle, result_qpos])

        if res:
            vis.set_qpos("robot", result_qpos)
            # print("qpos: ", np.round(result_qpos, 3).tolist())
            # print("left arm tcp pose:", arml_tcp_user_pose)
            # print("right arm tcp pose:", armr_tcp_user_pose)
            # print("-----")

            rate.sleep()
        else:
            continue
