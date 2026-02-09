'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

from dexe_robot_client.w1.path_planner.cartesian_planner import CartesianPlanner
from dexe_robot_client.w1.path_planner.cartesian_to_joint_planner import CartesianToJointPlanner
from dexe_robot_client.common import W1ArmType, W1Version
from dexe_robot_client.end_effector_client import EndEffectorType
from dexe_robot_client.w1.utils.unit_conversion import *

from loop_rate_limiters import RateLimiter
from dexe_rp import W1Vis, pos_wxyz_to_homogeneous, Box, Material

from scipy.spatial.transform import Rotation as R
import numpy as np
import time


def test_cartesian_to_joint_planner():
    cartesian_joint_planner = CartesianToJointPlanner(
        arm_type=W1ArmType.ANTHROPOMORPHIC,
        w1_version=W1Version.V2,
        end_effector_type=EndEffectorType.BRAINCO_HAND,
    )

    cartesian_planner = CartesianPlanner(
        arm_type=W1ArmType.ANTHROPOMORPHIC,
        w1_version=W1Version.V2,
        end_effector_type=EndEffectorType.BRAINCO_HAND)

    cartesian_joint_planner.set_arm_tcp("left_arm", np.eye(4))
    cartesian_planner.set_arm_tcp("left_arm", np.eye(4))
    right_tcp_pose = np.eye(4)
    right_tcp_pose[:3, :3] = R.from_rotvec([0.0, 0.0, 180.],
                                           degrees=True).as_matrix()
    cartesian_joint_planner.set_arm_tcp("right_arm", right_tcp_pose)
    cartesian_planner.set_arm_tcp("right_arm", right_tcp_pose)

    # solve left right ee traj
    left_tcp_pose = [-259.08, 474.23, 98.73, -100.44, -3.54, 8.29]
    left_tcp_pose = user_pose_to_xpos(left_tcp_pose[:3], left_tcp_pose[3:])
    right_tcp_pose = [278.67, 464.81, 92.11, -98.66, -1.28, 1.63]
    right_tcp_pose = user_pose_to_xpos(right_tcp_pose[:3], right_tcp_pose[3:])
    cartesian_planner.set_left_right_relationship(left_tcp_pose=left_tcp_pose,
                                                  right_tcp_pose=right_tcp_pose)

    waypoint1 = [-259.08, 474.23, 98.73, -100.44, -3.54, 8.29]
    waypoint1 = user_pose_to_xpos(waypoint1[:3], waypoint1[3:])
    waypoint2 = [-94.16, 531.83, 105.7, -96.94, -10.82, -12.09]
    waypoint2 = user_pose_to_xpos(waypoint2[:3], waypoint2[3:])
    waypoint_list = [waypoint1, waypoint2]

    res, left_pose_array, right_pose_array = cartesian_planner.generate_line_traj_from_tcp_poses(
        name="left_arm", waypoint_list=waypoint_list, cartesian_step=0.005)

    # solve left right joint traj
    left_start_qpos = np.asarray(
        [-1.106, -1.351, 1.472, -0.894, -1.233, -0.368, -0.201])
    right_start_qpos = np.asarray(
        [1.106, 1.351, -1.472, 0.894, 1.233, 0.368, 0.201])
    left_res, left_joint_traj = cartesian_joint_planner.plan(
        name="left_arm",
        start_qpos=left_start_qpos,
        end_effector_traj=left_pose_array)
    right_res, right_joint_traj = cartesian_joint_planner.plan(
        name="right_arm",
        start_qpos=right_start_qpos,
        end_effector_traj=right_pose_array)
    if left_res:
        for i in range(len(left_joint_traj)):
            print(f"Left Joint Qpos {i}: ", left_joint_traj[i])

    print("===================================")
    if right_res:
        for i in range(len(right_joint_traj)):
            print(f"Right Joint Qpos {i}: ", right_joint_traj[i])

    res, left_joint_traj, right_joint_traj = cartesian_joint_planner.generate_dense_dual_arm_joint_traj(
        left_joint_traj=left_joint_traj, right_joint_traj=right_joint_traj)

    return left_joint_traj, right_joint_traj


if __name__ == "__main__":
    vis = W1Vis()

    h_torso_text = vis.server.gui.add_text(label="Torso (rad)",
                                           initial_value="[]")
    h_head_text = vis.server.gui.add_text(label="Head (rad)",
                                          initial_value="[]")
    h_left_arm_text = vis.server.gui.add_text(label="Left arm (rad)",
                                              initial_value="[]")
    h_right_arm_text = vis.server.gui.add_text(label="Right arm (rad)",
                                               initial_value="[]")

    init_qpos = np.array([
        0.785, -1.57, 0.785, 0., 0., 0., -1.106, -1.351, 1.472, -0.894, -1.233,
        -0.368, -0.201, 1.106, 1.351, -1.472, 0.894, 1.233, 0.368, 0.201
    ])
    vis.set_qpos("robot", init_qpos)

    left_joint_traj, right_joint_traj = test_cartesian_to_joint_planner()

    idx = 0
    rate = RateLimiter(frequency=100, warn=True)
    while True:
        if idx < len(left_joint_traj):
            dual_arm_qpos = np.append(left_joint_traj[idx],
                                      right_joint_traj[idx])
            vis.set_qpos("dual_arm", dual_arm_qpos)
            idx += 1
            input("Press Enter to continue...")
        rate.sleep()
