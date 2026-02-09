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

task_dict1 = {
    "torso": [[0.78539816, -1.57079633, 0.78539816, 0.03490659]],
    "torso_speed_ratio": [1.],
    "head": [[0., -0.76794487]],
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


def test_move_linear_full_body(task_dict):
    w1_manager.move_linear_full_body(waypoints_dict=task_dict, is_sync=True)


if __name__ == "__main__":

    w1_manager = W1Manager(is_sim=True,
                           arm_type=W1ArmType.ANTHROPOMORPHIC,
                           w1_version=W1Version.V2,
                           end_effector_type=EndEffectorType.LINKER_HAND)
    if not w1_manager.connect() and not w1_manager.is_enabled():
        raise RuntimeError("Failed to connect to W1 robot.")

    w1_manager.set_tcp("left_arm", np.eye(4))
    right_tcp_xpos = np.eye(4)
    right_tcp_xpos[:3, :3] = R.from_rotvec([0.0, 0.0, 180.],
                                           degrees=True).as_matrix()
    w1_manager.set_tcp("right_arm", xpos=right_tcp_xpos)

    w1_manager.set_home_qpos(np.array([
        0.78539816, -1.57079633, 0.78539816, 0.03490659, 0., -0.76794487, -0.98,
        -1.451, 1.622, -0.479, -1.453, -0.513, 0.33, 0.969, 1.416, -1.594,
        0.559, 1.263, 0.53, -0.436
    ]),
                             save_to_file=False)
    w1_manager.go_home(1.)

    # vis = W1Vis()
    # h_torso_text = vis.server.gui.add_text(label="Torso (rad)",
    #                                        initial_value="[]")
    # h_head_text = vis.server.gui.add_text(label="Head (rad)",
    #                                       initial_value="[]")
    # h_left_arm_text = vis.server.gui.add_text(label="Left arm (rad)",
    #                                           initial_value="[]")
    # h_right_arm_text = vis.server.gui.add_text(label="Right arm (rad)",
    #                                            initial_value="[]")

    # init_qpos = np.array([
    #     0.78539816, -1.57079633, 0.78539816, 0.03490659, 0., -0.76794487, -0.98,
    #     -1.451, 1.622, -0.479, -1.453, -0.513, 0.33, 0.969, 1.416, -1.594,
    #     0.559, 1.263, 0.53, -0.436
    # ])
    # vis.set_qpos("robot", init_qpos)

    test_move_linear_full_body(task_dict=task_dict1)

    # total_qpos_array = w1_manager.total_qpos_array
    # input("Press Enter to start visualization...")
    # idx = 0
    # rate = RateLimiter(frequency=100, warn=True)
    # while True:
    #     if idx < len(total_qpos_array):
    #         full_qpos = total_qpos_array[idx]
    #         vis.set_qpos("robot", full_qpos)
    #         idx += 1
    #         # input("Press Enter to continue...")
    #     rate.sleep()
