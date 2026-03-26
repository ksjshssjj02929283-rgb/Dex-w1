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

from dexe_rp import W1Vis, pos_wxyz_to_homogeneous, Box, Sphere, Mesh, Material
import json
"""
This example shows advanced useage of plan_dual_arm_multiple_task
    including attaching/detaching objects, disable/enable obstacles, combining move line and RRT planning
"""

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
    "left_arm_ee": [[0.3, [10, 100, 30, 30, 30, 30], 0.3],
                    [0.2, [0, 0, 0, 0, 0, 0], 0.3]],
    "left_use_collision_check": [False, True],
    "right_arm": [[252.0, 273.86, 173.45, -140.2, -10.99, 1.43],
                  [-41.94, 508.91, 76.45, -113.91, 6.08, 19.82]],
    "right_arm_speed_ratio": [1., 1.],
    "right_arm_ee": [[0.3, [20, 19, 19, 20, 19, 19], 0.3],
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


def test_multiple_task(start_qpos,
                       start_left_arm_ee_qpos,
                       start_right_arm_ee_qpos,
                       task_dict=None,
                       load_env=True,
                       fcl_env_path=None,
                       planner_env_path=None):
    if load_env:
        w1_manager.load_fcl_environment(fcl_env_path)
        w1_manager.load_planner_environment(planner_env_path)
    res, trajectory, armlee_trigger, armree_trigger = \
        w1_manager.plan_dual_arm_multiple_task(start_qpos,
                                               start_left_arm_ee_qpos,
                                               start_right_arm_ee_qpos,
                                               task_dict=task_dict,
                                               speed_ratio=1)
    return res, trajectory, armlee_trigger, armree_trigger


def move_linear_full_body(start_qpos,
                          start_left_arm_ee_qpos,
                          start_right_arm_ee_qpos,
                          task_dict=None):
    res, trajectory, armlee_trigger, armree_trigger = \
        w1_manager.plan_linear_full_body(start_qpos,
                                         start_left_arm_ee_qpos,
                                         start_right_arm_ee_qpos,
                                         waypoints_dict=task_dict,
                                         speed_ratio=1,
                                         use_collision_validator=False)
    return res, trajectory, armlee_trigger, armree_trigger


def _load_viser_environment(vis, file_path: str = None):
    if file_path is not None:
        with open(file_path, 'r') as f:
            env_data = json.load(f)
        obstacle_cfg = env_data.get("scene_objects", None)
        workobjects_cfg = env_data.get("user_workobjects", None)
        scene_handle_list = []
        workobjects_handle_list = []
        if obstacle_cfg is not None:
            for obj in obstacle_cfg:
                if obj["shape"]["shape_type"] == "box":
                    box = Box(
                        obj["shape"]["raw_data"][0],
                        obj["shape"]["raw_data"][1],
                        obj["shape"]["raw_data"][2],
                        pos_wxyz_to_homogeneous(obj["pose"][:3],
                                                obj["pose"][3:]))
                    h_box = vis.add_body(f"/{obj['name']}", box)
                    scene_handle_list.append(h_box)
                if obj["shape"]["shape_type"] == "sphere":
                    sphere = Sphere(
                        obj["shape"]["raw_data"][0],
                        pos_wxyz_to_homogeneous(obj["pose"][:3],
                                                obj["pose"][3:]))
                    h_sphere = vis.add_body(f"/{obj['name']}", sphere)
                    scene_handle_list.append(h_sphere)
                if obj["shape"]["shape_type"] == "mesh":
                    mesh = Mesh.init_from_file(
                        obj["shape"]["raw_data"],
                        pos_wxyz_to_homogeneous(obj["pose"][:3],
                                                obj["pose"][3:]))
                    h_mesh = vis.add_body(f"/{obj['name']}", mesh)
                    scene_handle_list.append(h_mesh)
        if workobjects_cfg is not None:
            for obj in workobjects_cfg:
                if obj["shape"]["shape_type"] == "box":
                    box = Box(
                        obj["shape"]["raw_data"][0],
                        obj["shape"]["raw_data"][1],
                        obj["shape"]["raw_data"][2],
                        pos_wxyz_to_homogeneous(obj["pose"][:3],
                                                obj["pose"][3:]), Material.RED)
                    h_box = vis.add_body(f"/{obj['name_key']}", box)
                    workobjects_handle_list.append(h_box)
                if obj["shape"]["shape_type"] == "sphere":
                    sphere = Sphere(
                        obj["shape"]["raw_data"][0],
                        pos_wxyz_to_homogeneous(obj["pose"][:3],
                                                obj["pose"][3:]), Material.RED)
                    h_sphere = vis.add_body(f"/{obj['name_key']}", sphere)
                    workobjects_handle_list.append(h_sphere)
                if obj["shape"]["shape_type"] == "mesh":
                    mesh = Mesh.init_from_file(
                        obj["shape"]["raw_data"],
                        pos_wxyz_to_homogeneous(obj["pose"][:3],
                                                obj["pose"][3:]), Material.RED)
                    h_mesh = vis.add_body(f"/{obj['name_key']}", mesh)
                    workobjects_handle_list.append(h_mesh)

        return scene_handle_list, workobjects_handle_list


def load_viser(file_path: str = None):
    vis = W1Vis()
    vis.set_qpos("robot", home_qpos)
    vis.set_ee_pos("left_ee", left_ee_home_qpos)
    vis.set_ee_pos("right_ee", right_ee_home_qpos)

    scene_handle_list, workobjects_handle_list = _load_viser_environment(
        vis, file_path=file_path)

    return vis, scene_handle_list, workobjects_handle_list


def viser_execute_traj(vis,
                       trajectory,
                       armlee_trigger=None,
                       armree_trigger=None):
    from loop_rate_limiters import RateLimiter
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

    home_qpos = np.array([
        0.785, -1.57, 0.785, 0.03490658503988659, 0.0, -0.6632251157578453,
        2.1155, -0.2175, -1.0686, -1.8855, 0.0081, 0.1119, 0.6381, -2.1155,
        0.2175, 1.0686, 1.8855, -0.0081, -0.1119, -0.6381
    ])
    left_ee_home_qpos = np.array([0.0] * 6)
    right_ee_home_qpos = np.array([0.0] * 6)

    w1_manager.set_home_qpos(home_qpos, save_to_file=False)

    w1_manager.set_ee_qpos("left_arm_ee", left_ee_home_qpos)
    w1_manager.set_ee_qpos("right_arm_ee", right_ee_home_qpos)
    w1_manager.go_home(1)

    fcl_downward_env_path = f"{DATA_DIR}/environment/coffee_demo_downward.json"
    fcl_upward_env_path = f"{DATA_DIR}/environment/coffee_demo_upward.json"
    planner_downward_env_path = f"{DATA_DIR}/environment/expanded_coffee_demo_downward.json"
    planner_upward_env_path = f"{DATA_DIR}/environment/expanded_coffee_demo_upward.json"

    vis, scene_handle_list, workobjects_handle_list = load_viser(
        file_path=fcl_upward_env_path)

    trajectory, armlee_trigger, armree_trigger = None, None, None

    res, trajectory, armlee_trigger, armree_trigger = \
        test_multiple_task(home_qpos,
                           left_ee_home_qpos,
                           right_ee_home_qpos,
                           task_dict=task_dict1,
                           load_env=True,
                           fcl_env_path=fcl_upward_env_path,
                           planner_env_path=planner_upward_env_path)
    viser_execute_traj(vis, trajectory, armlee_trigger, armree_trigger)
    input("Press Enter to continue to next test...")
    # compute transformation matrix between grasp object and arm7 link
    res, cup_rotation, cup_translation = w1_manager.fcl_checker.get_grasp_object_pose(
        "cup")
    cup_pose = np.eye(4)
    cup_pose[:3, :3] = cup_rotation
    cup_pose[:3, 3] = cup_translation
    cur_qpos = trajectory[-1]
    torso_tf, head_tf, arml_tf, armr_tf = w1_manager._pin_kine.get_Kinematics(
        cur_qpos)
    arm7_T_object = np.linalg.inv(armr_tf[-1]) @ cup_pose

    w1_manager.attach_object(obj_name="cup",
                             which_arm="right",
                             arm7_T_object=arm7_T_object)

    vis.grab("right_arm", workobjects_handle_list[0])

    res, trajectory, armlee_trigger, armree_trigger = \
        test_multiple_task(np.asarray(trajectory[-1]),
                           np.asarray(armlee_trigger[-1][1]),
                           np.asarray(armree_trigger[-1][1]),
                           task_dict=task_dict2,
                           load_env=False,
                           )
    viser_execute_traj(vis, trajectory, armlee_trigger, armree_trigger)
    input("Press Enter to continue to next test...")

    torso_tf, head_tf, arml_tf, armr_tf = w1_manager._pin_kine.get_Kinematics(
        cur_qpos)

    w1_manager.detach_object(obj_name="cup",
                             which_arm="right",
                             arm7_transform=armr_tf[-1])

    vis.release("right_arm")

    res, trajectory, armlee_trigger, armree_trigger = \
        test_multiple_task(np.asarray(trajectory[-1]),
                           np.asarray(armlee_trigger[-1][1]),
                           np.asarray(armree_trigger[-1][1]),
                           task_dict=task_dict3,
                           load_env=False,
                           )
    viser_execute_traj(vis, trajectory, armlee_trigger, armree_trigger)
    input("Press Enter to continue to next test...")

    res, trajectory, armlee_trigger, armree_trigger = \
        test_multiple_task(np.asarray(trajectory[-1]),
                           np.asarray(armlee_trigger[-1][1]),
                           np.asarray(armree_trigger[-1][1]),
                           task_dict=task_dict4,
                           load_env=False,
                           )

    viser_execute_traj(vis, trajectory, armlee_trigger, armree_trigger)
    input("Press Enter to continue to next test...")

    res, trajectory, armlee_trigger, armree_trigger = \
        test_multiple_task(np.asarray(trajectory[-1]),
                           np.asarray(armlee_trigger[-1][1]),
                           np.asarray(armree_trigger[-1][1]),
                           task_dict=task_dict5,
                           load_env=False,
                           )
    viser_execute_traj(vis, trajectory, armlee_trigger, armree_trigger)
    input("Press Enter to continue to next test...")

    res, trajectory, armlee_trigger, armree_trigger = \
        test_multiple_task(np.asarray(trajectory[-1]),
                     np.asarray(armlee_trigger[-1][1]),
                     np.asarray(armree_trigger[-1][1]),
                     task_dict=task_dict6,
                     load_env=False)
    for handle in scene_handle_list:
        handle.remove()
    for handle in workobjects_handle_list:
        handle.remove()
    _load_viser_environment(vis, fcl_downward_env_path)
    viser_execute_traj(vis, trajectory, armlee_trigger, armree_trigger)
    input("Press Enter to continue to next test...")

    embed()
