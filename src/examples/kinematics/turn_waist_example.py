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
from loop_rate_limiters import RateLimiter
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
        476.81552887392564, 151.8864369061307, 874.0060771620389,
        153.7526570682316, -85.14610235747323, 24.605372677060252
    ],
                 [
                     359.49157646785625, 316.42618056249717, 874.0060771620388,
                     153.75265706823154, -85.14610235747323, 62.174321421983464
                 ],
                 [
                     154.99840845940943, 432.0726882387296, 874.0060771620388,
                     153.75265706823149, -85.14610235747321, 99.11888114766928
                 ]],
    "left_arm_speed_ratio": [1., 1., 1., 1.],
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
        use_collision_validator=False)
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

    # home_qpos = np.array([
    #     1.353, -2.705, 1.395, 0.009, 0.0, -0.681, -0.891, -1.655, 1.55, -0.023,
    #     -1.514, -0.633, 0.068, 0.894, 1.7, -1.529, 0.021, 1.324, 0.679, -0.14
    # ])

    home_qpos = np.array([
        0.885, -1.77, 0.885, 0.000, 0.0, -0.681, -0.891, -1.655, 1.55, -0.023,
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

    input("Press Enter to start the test...")

    res, full_body_traj1 = test_move_linear_dual_arm_sync(home_qpos,
                                                          left_ee_home_qpos,
                                                          right_ee_home_qpos,
                                                          task_dict=task_dict1)

    viser_execute_traj(vis, full_body_traj1)
