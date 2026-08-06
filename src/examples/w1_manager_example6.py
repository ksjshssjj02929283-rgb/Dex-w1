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

from viser_utils import load_viser, viser_execute_traj
from dexe_robot_client.w1 import W1Manager

from dexe_rp import EndEffectorType, RobotVersion
"""
This example task shows demo of dual arm move line with point-to-point synchronization.
The task semantic description is pull down the coffee handle synchronously.

Note:
    - The comment part is viser visualization code for debugging
    - is_sync: 
        -- True means the two arms will reach each waypoint at the same time
        -- False means the two arms will reach each waypoint independently
    - use_collision_validator: 
        -- True means the planned trajectory will be checked by collision validator
        -- False means no collision check will be performed for the planned trajectory

RobotVersion: W1V024
Left_EndEffectorType: BrainCo_Revo1_R
Right_EndEffectorTYpe: BrainCo_Revo1_R
"""

ROBOT_VERSION = RobotVersion.W1V024
END_EFFECTOR_TYPE = EndEffectorType.BrainCo_Revo1_R

task_dict1 = {
    "torso": [[0.78539816, -1.57079633, 0.78539816, 0.03490659]],
    "torso_speed_ratio": [1.],
    "head": [[0., -0.6]],
    "head_speed_ratio": [1.],
    "left_arm": [[11.72, 388.28, 58.78, -107.86, -1.11, -48.09],
                 [-43.1, 392.65, 35.63, -120.16, 0.56, -52.58],
                 [-105.8, 345.08, 73.12, -134.64, 4.36, -58.38],
                 [-138.18, 270.82, 97.75, -142.12, 24.12, -67.98],
                 [-183.44, 234.04, 85.02, -150.16, 33.15, -73.2],
                 [-185.83, 225.17, 101.11, -155.94, 35.89, -77.01],
                 [-107.39, 248.17, 182.36, -170.78, 41.05, -106.71]],
    "left_arm_speed_ratio": [2.4, 2, 2.1, 2.1, 2, 2, 2],
    "left_arm_ee": [[0, [0, 0, 0, 0, 0, 0], 0], [0, [0, 0, 0, 0, 0, 0], 0],
                    [0, [0, 0, 0, 0, 0, 0], 0], [0, [0, 0, 0, 0, 0, 0], 0],
                    [0, [0, 0, 0, 0, 0, 0], 0], [0, [0, 0, 0, 0, 0, 0], 0],
                    [0, [0, 0, 0, 0, 0, 0], 0]],
    "right_arm": [[-19.91, 397.03, 46.71, -105.37, -10.92, 50.19],
                  [52.35, 380.95, 47.27, -117.38, -1.83, 53.1],
                  [100.79, 345.5, 80.76, -131.69, -0.88, 51.74],
                  [143.34, 273.37, 106.44, -139.76, -20.47, 67.06],
                  [183.8, 250.78, 110.89, -152.02, -25.8, 70.13],
                  [194.9, 222.05, 99.06, -153.02, -38.63, 79.18],
                  [105.49, 246.65, 140.12, -163.21, -40.71, 97.35]],
    "right_arm_speed_ratio": [3, 2.6, 2.5, 2.3, 2, 1.5, 1.5],
    "right_arm_ee": [[0, [0, 0, 0, 0, 0, 0], 0], [0, [0, 0, 0, 0, 0, 0], 0],
                     [0, [0, 0, 0, 0, 0, 0], 0], [0, [0, 0, 0, 0, 0, 0], 0],
                     [0, [0, 0, 0, 0, 0, 0], 0], [0, [0, 0, 0, 0, 0, 0], 0],
                     [0, [0, 0, 0, 0, 0, 0], 0]],
}

if __name__ == "__main__":
    w1_manager = W1Manager(
        is_sim=True,
        robot_version=ROBOT_VERSION,
        end_effector_type=END_EFFECTOR_TYPE,
    )

    # # judge w1_manager is connected and enabled
    # if not w1_manager.connect() and not w1_manager.is_enabled():
    #     raise RuntimeError("Failed to connect to W1 robot.")

    w1_manager.set_tcp("left_arm", np.eye(4))
    right_tcp_xpos = np.eye(4)
    right_tcp_xpos[:3, :3] = R.from_rotvec([0.0, 0.0, 180.],
                                           degrees=True).as_matrix()
    w1_manager.set_tcp("right_arm", xpos=right_tcp_xpos)

    home_qpos = np.array([
        0.7854, -1.5708, 0.7854, 0.0349, 0, -0.6, 0.962, -0.771, -0.949, -2.072,
        0.435, -0.004, 0.012, -1.0031, 0.7219, 0.8681, 2.024, -0.4591, 0.1159,
        -0.1521
    ])
    left_ee_home_qpos = np.array([0.0] * 6)
    right_ee_home_qpos = np.array([0.0] * 6)

    w1_manager.set_home_qpos(home_qpos, save_to_file=False)
    w1_manager.set_ee_qpos("left_arm_ee", left_ee_home_qpos)
    w1_manager.set_ee_qpos("right_arm_ee", right_ee_home_qpos)
    w1_manager.go_home(1)

    vis, scene_handle_list, workobjects_handle_list = load_viser(
        home_qpos, left_ee_home_qpos, right_ee_home_qpos)

    input("Press Enter to start test...")

    res, full_body_traj, armlee_trigger, armree_trigger = \
        w1_manager.plan_linear_full_body(
            start_qpos=home_qpos,
            start_left_arm_ee_qpos=left_ee_home_qpos,
            start_right_arm_ee_qpos=right_ee_home_qpos,
            waypoints_dict=task_dict1,
            is_local=True,
            is_sync=True,
            use_collision_validator=True
        )

    viser_execute_traj(vis, full_body_traj, None, None)
