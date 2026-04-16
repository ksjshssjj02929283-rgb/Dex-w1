'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

from dexe_rp import CoalCollision, CoalCollisionValidator, PinocchioKinematics, PinocchioStateToPose, OMPLPlanner
from dexe_rp import pos_wxyz_to_homogeneous, Box, Sphere, Mesh, Material, W1Vis
import numpy as np
import json
import time

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

# interpolate path
def linear_interpolate_full_body_path(full_body_path: np.ndarray, interp_num: int) -> np.ndarray:
    """
    对 full_body_path 做线性插值。

    Args:
        full_body_path: shape = (N, dof)，例如你的 (N, 20)
        interp_num: 每两个相邻点之间插入的点数

    Returns:
        interpolated_path: shape = ((N - 1) * (interp_num + 1) + 1, dof)
    """
    full_body_path = np.asarray(full_body_path)

    if full_body_path.ndim != 2:
        raise ValueError("full_body_path must be a 2D array with shape (N, dof)")

    if interp_num < 0:
        raise ValueError("interp_num must be >= 0")

    if len(full_body_path) <= 1 or interp_num == 0:
        return full_body_path.copy()

    interpolated_path = []

    for i in range(len(full_body_path) - 1):
        start = full_body_path[i]
        end = full_body_path[i + 1]

        # endpoint=False 避免重复加入 end 点
        for alpha in np.linspace(0.0, 1.0, interp_num + 1, endpoint=False):
            q = (1.0 - alpha) * start + alpha * end
            interpolated_path.append(q)

    interpolated_path.append(full_body_path[-1])

    return np.asarray(interpolated_path)

def check_traj_collision(trajectory: np.ndarray):
    for qpos in trajectory:
        res, poses = pin_kine.compute_forward_kinematics(qpos)
        if not res:
            print("Failed to compute forward kinematics.")
            continue
        coal_collision.update_robot_component_pose("torso", poses[0:4])
        coal_collision.update_robot_component_pose("head", poses[4:6])
        coal_collision.update_robot_component_pose("left_arm", poses[6:13])
        coal_collision.update_robot_component_pose("right_arm", poses[13:20])
        if coal_collision.check_collision():
            print("Collision detected at qpos:", qpos)
            

if __name__ == "__main__":
    home_qpos = np.zeros(20)
    left_ee_home_qpos = np.zeros(6)
    right_ee_home_qpos = np.zeros(6)

    vis, scene_handle_list, workobjects_handle_list = load_viser(
        file_path="/home/young/dexe_robot_client_data/environment/template_ompl.json")
    
    pin_kine = PinocchioKinematics("/home/young/dexe_robot_client_data/assets/Model/W1_V2_A/W1_V2_A.urdf")

    general_path = "/home/young/dexe_robot_client_data/assets/Model/W1_V2_A/Collision_simp"

    torso_component = [
        ("torso", "ankle", f"{general_path}/torso/torso1.obj"),
        ("torso", "knee", f"{general_path}/torso/torso2.obj"),
        ("torso", "buttock", f"{general_path}/torso/torso3.obj"),
        ("torso", "waist", f"{general_path}/torso/torso4.obj")
    ]
    head_component = [
        ("head", "neck1", f"{general_path}/head/head1.obj"),
        ("head", "neck2", f"{general_path}/head/head2.obj")
    ]
    left_arm_component = [
        ("left_arm", "left_j1", f"{general_path}/left_arm/arml1.obj"),
        ("left_arm", "left_j2", f"{general_path}/left_arm/arml2.obj"),
        ("left_arm", "left_j3", f"{general_path}/left_arm/arml3.obj"),
        ("left_arm", "left_j4", f"{general_path}/left_arm/arml4.obj"),
        ("left_arm", "left_j5", f"{general_path}/left_arm/arml5.obj"),
        ("left_arm", "left_j6", f"{general_path}/left_arm/arml6.obj"),
        ("left_arm", "left_j7", f"{general_path}/left_arm/arml7.obj")
    ]
    right_arm_component = [
        ("right_arm", "right_j1", f"{general_path}/right_arm/armr1.obj"),
        ("right_arm", "right_j2", f"{general_path}/right_arm/armr2.obj"),
        ("right_arm", "right_j3", f"{general_path}/right_arm/armr3.obj"),
        ("right_arm", "right_j4", f"{general_path}/right_arm/armr4.obj"),
        ("right_arm", "right_j5", f"{general_path}/right_arm/armr5.obj"),
        ("right_arm", "right_j6", f"{general_path}/right_arm/armr6.obj"),
        ("right_arm", "right_j7", f"{general_path}/right_arm/armr7.obj")
    ]

    W1_V2_A = torso_component + head_component + left_arm_component + right_arm_component

    coal_collision = CoalCollision(W1_V2_A, False)
    coal_collision.load_environment_from_json("/home/young/dexe_robot_client_data/environment/template_ompl.json")
    coal_collision.load_ignored_collision_pairs_from_json("/home/young/dexe_robot_client_data/assets/Model/disable_collision_pair_with_urdfLinkName.json")
    pinocchio_state_to_pose = PinocchioStateToPose("/home/young/dexe_robot_client_data/assets/Model/W1_V2_A/W1_V2_A.urdf")
    coal_collision_validator = CoalCollisionValidator(coal_collision, pinocchio_state_to_pose)
    
    state_joint_names = ["LEFT_J1", "LEFT_J2", "LEFT_J3", "LEFT_J4", "LEFT_J5", "LEFT_J6", "LEFT_J7"]
    res = pinocchio_state_to_pose.set_state_joint_names(state_joint_names)

    fixed_joint_name_position_map = {
        "RIGHT_J1": 0.0, "RIGHT_J2": 0.0, "RIGHT_J3": 0.0, "RIGHT_J4": 0.0, "RIGHT_J5": 0.0, "RIGHT_J6": 0.0, "RIGHT_J7": 0.0,
        "NECK1": 0.0, "NECK2": 0.0, "ANKLE": 0.0, "KNEE": 0.0, "BUTTOCK": 0.0, "WAIST": 0.0
    }
    pinocchio_state_to_pose.set_fixed_joint_positions(fixed_joint_name_position_map)

    left_arm_lb = np.array([-2.9670597283903604, -2.0943951023931953, -2.9670597283903604, -2.356194490192345, -2.9670597283903604, -1.5707963267948966, -2.9670597283903604])
    left_arm_ub = np.array([2.9670597283903604, 1.5707963267948966, 2.9670597283903604, 1.5707963267948966, 2.96705972839036041, 1.5707963267948966, 2.9670597283903604])

    ompl_planner = OMPLPlanner(7, left_arm_lb, left_arm_ub, coal_collision_validator)

    start_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    goal_state = np.array([-1.37, -1.51, 1.54, -0.85, 0.0, 0.0, 0.0])

    res, path = ompl_planner.plan(start_state, goal_state)

    if res:
        print("Found a path!")
        print("Path:")
        for state in path:
            print(state)
    else:
        print("No path found.")

    full_body_path = []
    for state in path:
        full_qpos = np.zeros(20)
        full_qpos[6:13] = state
        full_body_path.append(full_qpos)

    full_body_path = np.array(full_body_path)
    full_body_traj = linear_interpolate_full_body_path(full_body_path, 200)

    viser_execute_traj(vis, full_body_traj)

    check_traj_collision(full_body_traj)


    
