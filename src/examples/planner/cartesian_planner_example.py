'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

from dexe_robot_client.w1.path_planner.cartesian_planner import CartesianPlanner
from dexe_robot_client.w1.utils.unit_conversion import *

from dexe_rp import EndEffectorType, RobotVersion

from scipy.spatial.transform import Rotation as R
import numpy as np


def test_cartesian_planner():
    cartesian_planner = CartesianPlanner(
        robot_version=RobotVersion.W1V024,
        end_effector_type=EndEffectorType.BrainCo_Revo1_R)

    cartesian_planner.set_arm_tcp("left_arm", np.eye(4))
    right_tcp_pose = np.eye(4)
    right_tcp_pose[:3, :3] = R.from_rotvec([0.0, 0.0, 180.],
                                           degrees=True).as_matrix()
    cartesian_planner.set_arm_tcp("right_arm", right_tcp_pose)

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
    # if res:
    #     for left_pose, right_pose in zip(left_pose_array, right_pose_array):
    #         print("Left TCP Position:", left_pose[:3,3])
    #         print("Right TCP Position:", right_pose[:3,3])


if __name__ == "__main__":
    test_cartesian_planner()
