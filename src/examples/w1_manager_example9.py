'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

import numpy as np
from scipy.spatial.transform import Rotation as R
from IPython import embed
from loop_rate_limiters import RateLimiter

from dexe_robot_client.w1 import W1Manager
from dexe_robot_client.w1.path_planner.utils.robot_type import *
from dexe_robot_client.common.mapping_index import *
from dexe_robot_client.common.config import DATA_DIR

from dexe_rp import EndEffectorType, RobotVersion
from dexe_rp import W1Vis, pos_wxyz_to_homogeneous, Box, Material
"""
This example task shows demo of dual arm(fix-connection) move line using whole body ik(whole body motion)

Note:
    - The comment part is viser visualization code for debugging
    - is_local:
        -- False means the waypoints should be in robot base_link frame and
              the planner will use whole body ik solver to plan whole body motion trajectory
    - The teaching point should only have one arm's waypoint,
        the planner will automatically compute the other arm's waypoint by the fixed connection between the two arms.
    - The function set_whole_body_ik_cost_type can be used to set legs bend forward or bend backward
        but please must call move_joint to change leg's bending direction before you change the leg bend mode.
    - plan_linear_dual_arm_sync and move_linear_dual_arm_sync have the same parameters, which have the same effect.
"""

task_dict1 = {
    "torso": [[0.78539816, -1.57079633, 0.78539816, 0.03490659]],
    "torso_speed_ratio": [1.],
    "head": [[0., -0.76794487]],
    "head_speed_ratio": [1.],
    "left_arm": [[
        476.81552887392564, 151.8864369061307, 477.51278916762175,
        153.7526570682316, -85.14610235747323, 24.605372677060252
    ],
                 [
                     476.81552887392564, 151.8864369061307, 663.5256017685385,
                     153.7526570682316, -85.14610235747323, 24.605372677060252
                 ],
                 [
                     476.81552887392564, 151.8864369061307, 955.8654830562698,
                     153.7526570682316, -85.14610235747323, 24.605372677060252
                 ],
                 [
                     476.81552887392564, 151.8864369061307, 1233.6224827317324,
                     153.7526570682316, -85.14610235747323, 24.605372677060252
                 ]],
    "left_arm_speed_ratio": [0.5, 0.5, 0.5, 0.5],
    "left_arm_ee": [],
    "right_arm": [],
    "right_arm_speed_ratio": [],
    "right_arm_ee": [],
}

task_dict3 = {
    "torso": [[-0.78539816, 1.57079633, -0.78539816, 0.03490659]],
    "torso_speed_ratio": [1.],
    "head": [[0., -0.76794487]],
    "head_speed_ratio": [1.],
    "left_arm": [
        [
            476.81552887392564, 151.8864369061307, 1233.6224827317324,
            153.7526570682316, -85.14610235747323, 24.605372677060252
        ],
        [
            476.81552887392564, 151.8864369061307, 955.8654830562698,
            153.7526570682316, -85.14610235747323, 24.605372677060252
        ],
        [
            476.81552887392564, 151.8864369061307, 663.5256017685385,
            153.7526570682316, -85.14610235747323, 24.605372677060252
        ],
        [
            476.81552887392564, 151.8864369061307, 477.51278916762175,
            153.7526570682316, -85.14610235747323, 24.605372677060252
        ],
    ],
    "left_arm_speed_ratio": [1.0, 1.0, 1.0, 1.0],
    "left_arm_ee": [],
    "right_arm": [],
    "right_arm_speed_ratio": [],
    "right_arm_ee": [],
}


def test_move_linear_dual_arm_sync(start_qpos, start_left_arm_ee_qpos,
                                   start_right_arm_ee_qpos, task_dict):
    res, full_body_traj = w1_manager.plan_linear_dual_arm_sync(
        task_dict=task_dict,
        start_qpos=start_qpos,
        start_left_arm_ee_qpos=start_left_arm_ee_qpos,
        start_right_arm_ee_qpos=start_right_arm_ee_qpos,
        is_local=False,
        use_collision_validator=True)
    return res, full_body_traj


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
    w1_manager = W1Manager(is_sim=True,
                           robot_version=RobotVersion.W1V024,
                           end_effector_type=EndEffectorType.BrainCo_Revo1_R)
    if not w1_manager.connect() and not w1_manager.is_enabled():
        raise RuntimeError("Failed to connect to W1 robot.")

    w1_manager.set_tcp("left_arm", np.eye(4))
    right_tcp_xpos = np.eye(4)
    right_tcp_xpos[:3, :3] = R.from_rotvec([0.0, 0.0, 180.],
                                           degrees=True).as_matrix()
    w1_manager.set_tcp("right_arm", xpos=right_tcp_xpos)

    home_qpos = np.array([
        1.353, -2.705, 1.395, 0.009, 0.0, -0.681, -0.891, -1.655, 1.55, -0.023,
        -1.514, -0.633, 0.068, 0.894, 1.7, -1.529, 0.021, 1.324, 0.679, -0.14
    ])
    left_ee_home_qpos = np.array([0.0] * 6)
    right_ee_home_qpos = np.array([0.0] * 6)

    w1_manager.set_home_qpos(home_qpos, save_to_file=False)
    w1_manager.go_home(1.)

    vis = W1Vis()

    vis.set_qpos("robot", home_qpos)
    vis.set_ee_pos("left_ee", left_ee_home_qpos)
    vis.set_ee_pos("right_ee", right_ee_home_qpos)

    res, full_body_traj1 = test_move_linear_dual_arm_sync(home_qpos,
                                                          left_ee_home_qpos,
                                                          right_ee_home_qpos,
                                                          task_dict=task_dict1)

    viser_execute_traj(vis, full_body_traj1)

    cur_qpos = full_body_traj1[-1]
    bend_backward_qpos = cur_qpos.copy()
    bend_backward_qpos[0:3] = -bend_backward_qpos[0:3]
    task_dict2 = {
        "torso": [bend_backward_qpos[0:4].tolist()],
        "torso_speed_ratio": [1.],
        "head": [bend_backward_qpos[4:6].tolist()],
        "head_speed_ratio": [1.],
        "left_arm": [bend_backward_qpos[6:13].tolist()],
        "left_arm_speed_ratio": [1.],
        "left_arm_ee": [],
        "right_arm": [bend_backward_qpos[13:20].tolist()],
        "right_arm_speed_ratio": [1.],
        "right_arm_ee": [],
    }

    res, full_body_traj2, armlee_trigger, armree_trigger = w1_manager.plan_joint_full_body(
        start_qpos=cur_qpos,
        start_left_arm_ee_qpos=left_ee_home_qpos,
        start_right_arm_ee_qpos=right_ee_home_qpos,
        waypoints_dict=task_dict2,
        speed_ratio=1.0,
        use_collision_validator=True)

    viser_execute_traj(vis,
                       full_body_traj2,
                       armlee_trigger=armlee_trigger,
                       armree_trigger=armree_trigger)

    w1_manager.set_whole_body_ik_cost_type("standard_bend_backward")

    cur_qpos = full_body_traj2[-1]
    res, full_body_traj3 = test_move_linear_dual_arm_sync(cur_qpos,
                                                          left_ee_home_qpos,
                                                          right_ee_home_qpos,
                                                          task_dict=task_dict3)

    viser_execute_traj(vis, full_body_traj3)
