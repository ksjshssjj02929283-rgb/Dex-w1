'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

import numpy as np
from IPython import embed
from scipy.spatial.transform import Rotation as R

from dexe_robot_client.w1 import W1Manager
from dexe_robot_client.common import StatusCode
from dexe_robot_client.w1.utils.unit_conversion import xpos_to_user_pose, user_pose_to_xpos

from dexe_rp import RobotVersion, EndEffectorType
from dexe_rp import W1Vis, pos_wxyz_to_homogeneous, Box, Sphere, Mesh, Material, rotation_to_wxyz
from loop_rate_limiters import RateLimiter

def _load_viser_environment(vis):
    scene_handle_list = []
    workobjects_handle_list = []

    # Add viser handlers here

    return scene_handle_list, workobjects_handle_list


def load_viser(home_qpos: np.ndarray, left_ee_home_qpos: np.ndarray,
               right_ee_home_qpos: np.ndarray):
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
    w1_manager = W1Manager(
        is_sim=True,
        robot_version=RobotVersion.W1V030,
        left_ee_type=EndEffectorType.BrainCo_Revo1_R,
        right_ee_type=EndEffectorType.BrainCo_Revo1_R
    )

    w1_manager.set_tcp("left_arm", np.eye(4))
    right_tcp_xpos = np.eye(4)
    right_tcp_xpos[:3, :3] = R.from_rotvec([0.0, 0.0, 180.],
                                           degrees=True).as_matrix()

    home_qpos = np.array([
        0.785, -1.57, 0.785, 0.0, 0.0, 0.0, -1.57, -0.785, 1.57, -1.57, -1.57, 0.0, 0.785, -1.57, 0.785, 1.57, 1.57, -1.57, 0.0, -0.785
    ])
    torso_home_qpos = home_qpos[:4]
    head_home_qpos = home_qpos[4:6]
    left_arm_home_qpos = home_qpos[6:13]
    right_arm_home_qpos = home_qpos[13:]

    left_ee_home_qpos = np.array([0.0] * 6)
    right_ee_home_qpos = np.array([0.0] * 6)

    vis, scene_handle_list, workobjects_handle_list = load_viser(home_qpos, left_ee_home_qpos, right_ee_home_qpos)

    left_arm_local_home_xpos = w1_manager.get_xpos("left_arm", left_arm_home_qpos, True)[1]
    right_arm_local_home_xpos = w1_manager.get_xpos("right_arm", right_arm_home_qpos, True)[1]
    left_arm_local_user_pose = xpos_to_user_pose(left_arm_local_home_xpos)
    right_arm_local_user_pose = xpos_to_user_pose(right_arm_local_home_xpos)

    left_arm_global_home_xpos = w1_manager.get_xpos("left_arm", home_qpos, False)[1]
    right_arm_global_home_xpos = w1_manager.get_xpos("right_arm", home_qpos, False)[1]
    
    dual_arm_center_global_home_posi = (left_arm_global_home_xpos[:3, 3] + right_arm_global_home_xpos[:3, 3]) / 2

    template_box_center_global = dual_arm_center_global_home_posi.copy()
    template_box_center_global[0] += 0.2

    template_box_frame_global = np.eye(4)
    template_box_frame_global[:3, 3] = template_box_center_global

    template_box_center_marker = Sphere(
        0.015, template_box_frame_global, Material.RED)
    h_template_box_center = vis.add_body(
        "/template_box_center", template_box_center_marker)
    scene_handle_list.append(h_template_box_center)

    dual_arm_tcp_distance = np.linalg.norm(
        left_arm_global_home_xpos[:3, 3] -
        right_arm_global_home_xpos[:3, 3])
    template_box_size = np.array([
        dual_arm_tcp_distance * 0.6,
        dual_arm_tcp_distance,
        dual_arm_tcp_distance * 0.6,
    ])
    template_box_material = Material(
        "TemplateBox", (0.2, 0.6, 1.0, 0.3))
    template_box = Box(
        template_box_size[0],
        template_box_size[1],
        template_box_size[2],
        template_box_frame_global,
        template_box_material,
    )
    h_template_box = vis.add_body("/template_box", template_box)
    scene_handle_list.append(h_template_box)

    template_box_T_current_box = np.array([
        [ 0.98480775, -0.17364818, 0.0, -0.05],
        [ 0.17364818,  0.98480775, 0.0,  0.00],
        [ 0.0,         0.0,        1.0,  0.00],
        [ 0.0,         0.0,        0.0,  1.00],
    ])

    current_box_frame_global = template_box_frame_global @ template_box_T_current_box
    

    template_box_T_left_tcp = np.linalg.inv(template_box_frame_global) @ left_arm_global_home_xpos
    template_box_T_right_tcp = np.linalg.inv(template_box_frame_global) @ right_arm_global_home_xpos

    res, left_tcp_adjusted_user_pose, right_tcp_adjusted_user_pose = w1_manager.adjust_dual_arm_taught_tcp_poses(
        template_box_T_current_box,
        left_arm_local_user_pose,
        right_arm_local_user_pose,
        template_box_T_left_tcp,
        template_box_T_right_tcp
    )

    task_dict = {
        "left_arm": [left_tcp_adjusted_user_pose],
        "left_arm_speed_ratio": [1.],
        "left_arm_ee": [],
        # "right_arm": [right_tcp_adjusted_user_pose]
    }
    res, trajectory = w1_manager.plan_linear_dual_arm_sync(
        task_dict=task_dict,
        start_qpos=home_qpos,
        start_left_arm_ee_qpos=left_ee_home_qpos,
        start_right_arm_ee_qpos=right_ee_home_qpos,
        is_local=True,
        use_collision_validator=False
    )

    input("Press Enter to visualize the trajectory...")

    current_box_center_marker = Sphere(
        0.015, current_box_frame_global, Material.GREEN)
    h_current_box_center = vis.add_body(
        "/current_box_center", current_box_center_marker)
    scene_handle_list.append(h_current_box_center)

    current_box = Box(
        template_box_size[0],
        template_box_size[1],
        template_box_size[2],
        current_box_frame_global,
        Material("CurrentBox", (0.2, 1.0, 0.6, 0.3)),
    )
    h_current_box = vis.add_body("/current_box", current_box)
    scene_handle_list.append(h_current_box)

    viser_execute_traj(vis, trajectory)










    
