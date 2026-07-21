'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

import numpy as np
from dexe_robot_client.w1 import W1Manager
from scipy.spatial.transform import Rotation as R
from IPython import embed
from dexe_robot_client.w1.utils.unit_conversion import *

from dexe_rp import EndEffectorType, RobotVersion
from dexe_rp import W1Vis, pos_wxyz_to_homogeneous, Box, Sphere, Mesh, Material, rotation_to_wxyz
from loop_rate_limiters import RateLimiter
"""
This example is used to test move_line solving SRS IK susccess rate for chess task
"""

chess_board_size = [0.6, 0.6, 0.03]  # 50cm x 50 cm x 3 cm
chess_board_center = [0.5, 0, 0.8]
# yapf: disable
chess_point = [
    [chess_board_center[0]-0.3, -0.2, 0.86], [chess_board_center[0]-0.3, -0.1, 0.86], [chess_board_center[0]-0.3, 0.0, 0.86], [chess_board_center[0]-0.3, 0.1, 0.86], [chess_board_center[0]-0.3, 0.2, 0.86],
    [chess_board_center[0]-0.2, -0.2, 0.86], [chess_board_center[0]-0.2, -0.1, 0.86], [chess_board_center[0]-0.2, 0.0, 0.86], [chess_board_center[0]-0.2, 0.1, 0.86], [chess_board_center[0]-0.2, 0.2, 0.86],
    [chess_board_center[0]-0.1, -0.2, 0.86], [chess_board_center[0]-0.1, -0.1, 0.86], [chess_board_center[0]-0.1, 0.0, 0.86], [chess_board_center[0]-0.1, 0.1, 0.86], [chess_board_center[0]-0.1, 0.2, 0.86],
    [chess_board_center[0], -0.2, 0.86], [chess_board_center[0], -0.1, 0.86], [chess_board_center[0], 0.0, 0.86], [chess_board_center[0], 0.1, 0.86], [chess_board_center[0], 0.2, 0.86],
    [chess_board_center[0]+0.1, -0.2, 0.86], [chess_board_center[0]+0.1, -0.1, 0.86], [chess_board_center[0]+0.1, 0.0, 0.86], [chess_board_center[0]+0.1, 0.1, 0.86], [chess_board_center[0]+0.1, 0.2, 0.86],
    [chess_board_center[0]+0.2, -0.2, 0.86], [chess_board_center[0]+0.2, -0.1, 0.86], [chess_board_center[0]+0.2, 0.0, 0.86], [chess_board_center[0]+0.2, 0.1, 0.86], [chess_board_center[0]+0.2, 0.2, 0.86],
    [chess_board_center[0]+0.3, -0.2, 0.86], [chess_board_center[0]+0.3, -0.1, 0.86], [chess_board_center[0]+0.3, 0.0, 0.86], [chess_board_center[0]+0.3, 0.1, 0.86], [chess_board_center[0]+0.3, 0.2, 0.86]
]
# yapf: enable

point_size = 0.01

task_dict1 = {
    "torso": [[
        0.5410520681182421, -1.2217304763960306, 0.7853981633974483,
        0.03490658503988659
    ]],
    "torso_speed_ratio": [1.],
    "head": [[0.0, -0.6632251157578453]],
    "head_speed_ratio": [1.],
    "left_arm": [[
        206.70403667604066, 169.93753269649366, 889.7818674246682,
        -91.9824338419313, -6.29905294151704, -162.21006649445212
    ],
                 [
                     715.6950275878132, 133.28224233794336, 883.4093617459521,
                     -94.32220439456964, -4.995818100957114, -138.7645200018187
                 ],
                 [
                     487.48497787384065, 500.46221570903106, 909.6156608297301,
                     -96.5604103366916, -0.7454603298521114, -96.17394215873206
                 ],
                 [
                     206.70403667604066, 169.93753269649366, 889.7818674246682,
                     -91.9824338419313, -6.29905294151704, -162.21006649445212
                 ]],
    "left_arm_speed_ratio": [0.5, 0.5, 0.5, 0.5],
    "left_arm_ee": [[0, [0, 0, 0, 0, 0, 0], 0], [0, [0, 0, 0, 0, 0, 0], 0],
                    [0, [0, 0, 0, 0, 0, 0], 0], [0, [0, 0, 0, 0, 0, 0], 0]],
    "right_arm": [[
        327.98307847234963, -278.47036078530897, 874.8493253064357,
        -94.09570210016686, 5.182980338841701, -38.68261345931179
    ],
                  [
                      498.6051148101593, -321.03429568795815, 855.0764262210761,
                      -95.63987483352426, 3.438356186233133, -59.00949674770808
                  ],
                  [
                      466.96901433825195, -451.79690616955, 899.3004155640351,
                      -96.46981249914427, 1.3195444485301902, -78.80160116834921
                  ],
                  [
                      327.98307847234963, -278.47036078530897,
                      874.8493253064357, -94.09570210016686, 5.182980338841701,
                      -38.68261345931179
                  ]],
    "right_arm_speed_ratio": [0.5, 0.5, 0.5, 0.5],
    "right_arm_ee": [[0, [0, 0, 0, 0, 0, 0], 0], [0, [0, 0, 0, 0, 0, 0], 0],
                     [0, [0, 0, 0, 0, 0, 0], 0], [0, [0, 0, 0, 0, 0, 0], 0]],
}

task_dict2 = {
    "left_arm":
        [[
            679.9198531530203, 211.4320180584595, 865.1478794238351,
            -96.60238969929269, -0.026788773568079554, -89.9509593855713
        ],
         [
             446.53378545410254, 452.7028675398074, 960.8044457900112,
             -96.60238969929269, -0.026788773568079554, -89.9509593855713
         ],
         [
             576.4796879945455, 275.0487732686348, 895.2109706535736,
             -96.14503296968499, -2.4193587940941716, -111.12878664548232
         ],
         [
             466.53378545410254, 452.7028675398074, 960.8044457900112,
             -96.60238969929269, -0.026788773568079554, -89.9509593855713
         ],
         [
             528.6839498408967, 283.32040573425064, 900.7384389722291,
             -94.86669204656081, -4.467169904877989, -132.1708824271232
         ],
         [
             446.53378545410254, 452.7028675398074, 960.8044457900112,
             -96.60238969929269, -0.026788773568079554, -89.9509593855713
         ],
         [
             487.02980590084104, 296.10198127808815, 883.8398131728836,
             -91.18324601664946, -7.828371842771592, -161.54780028749377
         ],
         [
             466.53378545410254, 452.7028675398074, 960.8044457900112,
             -96.60238969929269, -0.026788773568079554, -89.9509593855713
         ],
         [
             386.8217386507626, 304.15615962858277, 879.5645283374527,
             -91.18324601664946, -7.828371842771592, -161.54780028749377
         ],
         [
             446.53378545410254, 452.7028675398074, 960.8044457900112,
             -96.60238969929269, -0.026788773568079554, -89.9509593855713
         ],
         [
             284.7015038920969, 328.6887699082787, 892.9744295250562,
             -91.18324601664946, -7.828371842771592, -161.54780028749377
         ],
         [
             466.53378545410254, 452.7028675398074, 960.8044457900112,
             -96.60238969929269, -0.026788773568079554, -89.9509593855713
         ],
         [
             170.77726680639202, 328.35742477217923, 864.4225297033651,
             -91.18324601664946, -7.828371842771592, -161.54780028749377
         ]],
    "left_arm_speed_ratio": [
        1.,
        1.,
        1.,
        1.,
        1.,
        1.,
        1.,
        1.,
        1.,
        1.,
        1.,
        1.,
        1.,
    ],
    "left_arm_ee": [[0, [0, 0, 0, 0, 0, 0], 0], [0, [0, 0, 0, 0, 0, 0], 0],
                    [0, [0, 0, 0, 0, 0, 0], 0], [0, [0, 0, 0, 0, 0, 0], 0],
                    [0, [0, 0, 0, 0, 0, 0], 0], [0, [0, 0, 0, 0, 0, 0], 0],
                    [0, [0, 0, 0, 0, 0, 0], 0], [0, [0, 0, 0, 0, 0, 0], 0],
                    [0, [0, 0, 0, 0, 0, 0], 0], [0, [0, 0, 0, 0, 0, 0], 0],
                    [0, [0, 0, 0, 0, 0, 0], 0], [0, [0, 0, 0, 0, 0, 0], 0],
                    [0, [0, 0, 0, 0, 0, 0], 0]],
    "right_arm": [
        [
            327.98307847234963, -278.47036078530897, 874.8493253064357,
            -94.09570210016686, 5.182980338841701, -38.68261345931179
        ],
        [
            454.1027841478037, -404.1620811481561, 977.8732357741285,
            -96.60238969929264, 0.026788773567977778, -90.04904061442875
        ],
        [
            498.6051148101593, -321.03429568795815, 855.0764262210761,
            -95.63987483352426, 3.438356186233133, -59.00949674770808
        ],
        [
            474.1027841478037, -404.1620811481561, 977.8732357741285,
            -96.60238969929264, 0.026788773567977778, -90.04904061442875
        ],
        [
            466.96901433825195, -451.79690616955, 899.3004155640351,
            -96.46981249914427, 1.3195444485301902, -78.80160116834921
        ],
        [
            454.1027841478037, -404.1620811481561, 977.8732357741285,
            -96.60238969929264, 0.026788773567977778, -90.04904061442875
        ],
        [
            327.98307847234963, -278.47036078530897, 874.8493253064357,
            -94.09570210016686, 5.182980338841701, -38.68261345931179
        ],
        [
            474.1027841478037, -404.1620811481561, 977.8732357741285,
            -96.60238969929264, 0.026788773567977778, -90.04904061442875
        ],
        [
            498.6051148101593, -321.03429568795815, 855.0764262210761,
            -95.63987483352426, 3.438356186233133, -59.00949674770808
        ],
        [
            454.1027841478037, -404.1620811481561, 977.8732357741285,
            -96.60238969929264, 0.026788773567977778, -90.04904061442875
        ],
        [
            466.96901433825195, -451.79690616955, 899.3004155640351,
            -96.46981249914427, 1.3195444485301902, -78.80160116834921
        ],
        [
            474.1027841478037, -404.1620811481561, 977.8732357741285,
            -96.60238969929264, 0.026788773567977778, -90.04904061442875
        ],
        [
            327.98307847234963, -278.47036078530897, 874.8493253064357,
            -94.09570210016686, 5.182980338841701, -38.68261345931179
        ]
    ],
    "right_arm_speed_ratio": [
        1.,
        1.,
        1.,
        1.,
        1.,
        1.,
        1.,
        1.,
        1.,
        1.,
        1.,
        1.,
        1.,
    ],
    "right_arm_ee": [[0, [0, 0, 0, 0, 0, 0], 0], [0, [0, 0, 0, 0, 0, 0], 0],
                     [0, [0, 0, 0, 0, 0, 0], 0], [0, [0, 0, 0, 0, 0, 0], 0],
                     [0, [0, 0, 0, 0, 0, 0], 0], [0, [0, 0, 0, 0, 0, 0], 0],
                     [0, [0, 0, 0, 0, 0, 0], 0], [0, [0, 0, 0, 0, 0, 0], 0],
                     [0, [0, 0, 0, 0, 0, 0], 0], [0, [0, 0, 0, 0, 0, 0], 0],
                     [0, [0, 0, 0, 0, 0, 0], 0], [0, [0, 0, 0, 0, 0, 0], 0],
                     [0, [0, 0, 0, 0, 0, 0], 0]],
}

task_dict3 = {
    "left_arm": [[
        679.9198531530203, 211.4320180584595, 865.1478794238351,
        -96.60238969929269, -0.026788773568079554, -89.9509593855713
    ],],
    "left_arm_speed_ratio": [1.],
    "left_arm_ee": [[0, [0, 0, 0, 0, 0, 0], 0],],
    "right_arm": [[
        547.5442599086869, -234.4607957777345, 869.1414603526498,
        -96.60238969929264, 0.026788773567977778, -90.04904061442875
    ],],
    "right_arm_speed_ratio": [1.,],
    "right_arm_ee": [[0, [0, 0, 0, 0, 0, 0], 0],],
}
task_dict4 = {
    "left_arm": [
        [
            446.53378545410254, 452.7028675398074, 960.8044457900112,
            -96.60238969929269, -0.026788773568079554, -89.9509593855713
        ],
        [
            576.4796879945455, 275.0487732686348, 895.2109706535736,
            -96.14503296968499, -2.4193587940941716, -111.12878664548232
        ],
    ],
    "left_arm_speed_ratio": [1., 1.],
    "left_arm_ee": [
        [0, [0, 0, 0, 0, 0, 0], 0],
        [0, [0, 0, 0, 0, 0, 0], 0],
    ],
    "right_arm": [
        [
            454.1027841478037, -404.1620811481561, 977.8732357741285,
            -96.60238969929264, 0.026788773567977778, -90.04904061442875
        ],
        [
            498.6051148101593, -321.03429568795815, 855.0764262210761,
            -95.63987483352426, 3.438356186233133, -59.00949674770808
        ],
    ],
    "right_arm_speed_ratio": [1., 1.],
    "right_arm_ee": [
        [0, [0, 0, 0, 0, 0, 0], 0],
        [0, [0, 0, 0, 0, 0, 0], 0],
    ],
}
task_dict5 = {
    "left_arm": [
        [
            466.53378545410254, 452.7028675398074, 960.8044457900112,
            -96.60238969929269, -0.026788773568079554, -89.9509593855713
        ],
        [
            528.6839498408967, 283.32040573425064, 900.7384389722291,
            -94.86669204656081, -4.467169904877989, -132.1708824271232
        ],
    ],
    "left_arm_speed_ratio": [1., 1.],
    "left_arm_ee": [
        [0, [0, 0, 0, 0, 0, 0], 0],
        [0, [0, 0, 0, 0, 0, 0], 0],
    ],
    "right_arm": [
        [
            474.1027841478037, -404.1620811481561, 977.8732357741285,
            -96.60238969929264, 0.026788773567977778, -90.04904061442875
        ],
        [
            466.96901433825195, -451.79690616955, 899.3004155640351,
            -96.46981249914427, 1.3195444485301902, -78.80160116834921
        ],
    ],
    "right_arm_speed_ratio": [1., 1.],
    "right_arm_ee": [
        [0, [0, 0, 0, 0, 0, 0], 0],
        [0, [0, 0, 0, 0, 0, 0], 0],
    ],
}
task_dict6 = {
    "left_arm": [
        [
            446.53378545410254, 452.7028675398074, 960.8044457900112,
            -96.60238969929269, -0.026788773568079554, -89.9509593855713
        ],
        [
            487.02980590084104, 296.10198127808815, 883.8398131728836,
            -91.18324601664946, -7.828371842771592, -161.54780028749377
        ],
    ],
    "left_arm_speed_ratio": [1., 1.],
    "left_arm_ee": [
        [0, [0, 0, 0, 0, 0, 0], 0],
        [0, [0, 0, 0, 0, 0, 0], 0],
    ],
    "right_arm": [
        [
            454.1027841478037, -404.1620811481561, 977.8732357741285,
            -96.60238969929264, 0.026788773567977778, -90.04904061442875
        ],
        [
            327.98307847234963, -278.47036078530897, 874.8493253064357,
            -94.09570210016686, 5.182980338841701, -38.68261345931179
        ],
    ],
    "right_arm_speed_ratio": [1., 1.],
    "right_arm_ee": [
        [0, [0, 0, 0, 0, 0, 0], 0],
        [0, [0, 0, 0, 0, 0, 0], 0],
    ],
}
task_dict7 = {
    "left_arm": [
        [
            466.53378545410254, 452.7028675398074, 960.8044457900112,
            -96.60238969929269, -0.026788773568079554, -89.9509593855713
        ],
        [
            386.8217386507626, 304.15615962858277, 879.5645283374527,
            -91.18324601664946, -7.828371842771592, -161.54780028749377
        ],
    ],
    "left_arm_speed_ratio": [1., 1.],
    "left_arm_ee": [
        [0, [0, 0, 0, 0, 0, 0], 0],
        [0, [0, 0, 0, 0, 0, 0], 0],
    ],
    "right_arm": [
        [
            474.1027841478037, -404.1620811481561, 977.8732357741285,
            -96.60238969929264, 0.026788773567977778, -90.04904061442875
        ],
        [
            498.6051148101593, -321.03429568795815, 855.0764262210761,
            -95.63987483352426, 3.438356186233133, -59.00949674770808
        ],
    ],
    "right_arm_speed_ratio": [1., 1.],
    "right_arm_ee": [
        [0, [0, 0, 0, 0, 0, 0], 0],
        [0, [0, 0, 0, 0, 0, 0], 0],
    ],
}
task_dict8 = {
    "left_arm": [
        [
            446.53378545410254, 452.7028675398074, 960.8044457900112,
            -96.60238969929269, -0.026788773568079554, -89.9509593855713
        ],
        [
            284.7015038920969, 328.6887699082787, 892.9744295250562,
            -91.18324601664946, -7.828371842771592, -161.54780028749377
        ],
    ],
    "left_arm_speed_ratio": [1., 1.],
    "left_arm_ee": [
        [0, [0, 0, 0, 0, 0, 0], 0],
        [0, [0, 0, 0, 0, 0, 0], 0],
    ],
    "right_arm": [
        [
            454.1027841478037, -404.1620811481561, 977.8732357741285,
            -96.60238969929264, 0.026788773567977778, -90.04904061442875
        ],
        [
            466.96901433825195, -451.79690616955, 899.3004155640351,
            -96.46981249914427, 1.3195444485301902, -78.80160116834921
        ],
    ],
    "right_arm_speed_ratio": [1., 1.],
    "right_arm_ee": [
        [0, [0, 0, 0, 0, 0, 0], 0],
        [0, [0, 0, 0, 0, 0, 0], 0],
    ],
}
task_dict9 = {
    "left_arm": [[
        466.53378545410254, 452.7028675398074, 960.8044457900112,
        -96.60238969929269, -0.026788773568079554, -89.9509593855713
    ],
                 [
                     170.77726680639202, 328.35742477217923, 864.4225297033651,
                     -91.18324601664946, -7.828371842771592, -161.54780028749377
                 ]],
    "left_arm_speed_ratio": [1., 1.],
    "left_arm_ee": [
        [0, [0, 0, 0, 0, 0, 0], 0],
        [0, [0, 0, 0, 0, 0, 0], 0],
    ],
    "right_arm": [[
        474.1027841478037, -404.1620811481561, 977.8732357741285,
        -96.60238969929264, 0.026788773567977778, -90.04904061442875
    ],
                  [
                      327.98307847234963, -278.47036078530897,
                      874.8493253064357, -94.09570210016686, 5.182980338841701,
                      -38.68261345931179
                  ]],
    "right_arm_speed_ratio": [1., 1.],
    "right_arm_ee": [
        [0, [0, 0, 0, 0, 0, 0], 0],
        [0, [0, 0, 0, 0, 0, 0], 0],
    ],
}


def _load_viser_environment(vis):
    scene_handle_list = []
    workobjects_handle_list = []

    chess_board_pose = np.eye(4)
    chess_board_pose[:3, 3] = chess_board_center
    chess_board = Box(chess_board_size[0], chess_board_size[1],
                      chess_board_size[2], chess_board_pose)
    h_chess_board = vis.add_body("/chess_board", chess_board)
    scene_handle_list.append(h_chess_board)

    for i in range(len(chess_point)):
        point_pose = np.eye(4)
        point_pose[:3, 3] = chess_point[i]
        point_sphere = Sphere(point_size, point_pose, Material.RED)
        h_point = vis.add_body(f"/chess_point_{i}", point_sphere)
        scene_handle_list.append(h_point)

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


def test_plan_linear_whole_body_ik(start_qpos, start_left_arm_ee_qpos,
                                   start_right_arm_ee_qpos, task_dict):
    res, trajectory, armlee_trigger, armree_trigger = \
        w1_manager.plan_linear_whole_body_ik(start_qpos,
                                             start_left_arm_ee_qpos,
                                             start_right_arm_ee_qpos,
                                             task_dict,
                                             use_collision_validator=False,
                                             movement_mode="eager",
                                             eye_focus_point=np.array([0.6, 0.0, 0.8]),
                                             )
    return res, trajectory, armlee_trigger, armree_trigger


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

    w1_manager.set_home_qpos(home_qpos, save_to_file=False)

    w1_manager.set_ee_qpos("left_arm_ee", left_ee_home_qpos)
    w1_manager.set_ee_qpos("right_arm_ee", right_ee_home_qpos)
    w1_manager.go_home(1)

    vis, scene_handle_list, workobjects_handle_list = load_viser()

    input("Press Enter to start linear whole body IK test...")

    res, trajectory, armlee_trigger, armree_trigger = test_plan_linear_whole_body_ik(
        start_qpos=home_qpos,
        start_left_arm_ee_qpos=left_ee_home_qpos,
        start_right_arm_ee_qpos=right_ee_home_qpos,
        task_dict=task_dict3)
    viser_execute_traj(vis, trajectory, armlee_trigger, armree_trigger)

    left_arm_user_pose = xpos_to_user_pose(
        w1_manager.get_xpos("left_arm", trajectory[-1], False)[1])
    right_arm_user_pose = xpos_to_user_pose(
        w1_manager.get_xpos("right_arm", trajectory[-1], False)[1])
    left_arm_target_pose = task_dict3["left_arm"][-1]
    right_arm_target_pose = task_dict3["right_arm"][-1]
    print(f"left_arm_user_pose: {left_arm_user_pose}")
    print(f"left_arm_target_pose: {left_arm_target_pose}")
    print(f"right_arm_user_pose: {right_arm_user_pose}")
    print(f"right_arm_target_pose: {right_arm_target_pose}")
    # left_target1_pose = user_pose_to_xpos(task_dict3["left_arm"][-1][:3], task_dict3["left_arm"][-1][3:])
    # left_target1_rotation = left_target1_pose[:3, :3]
    # print(pin.log3(left_target1_rotation.T @ w1_manager.get_xpos("left_arm", trajectory[-1], False)[1][:3, :3]))
    # input("Press Enter to continue...")

    res, trajectory, armlee_trigger, armree_trigger = test_plan_linear_whole_body_ik(
        start_qpos=trajectory[-1],
        start_left_arm_ee_qpos=left_ee_home_qpos,
        start_right_arm_ee_qpos=right_ee_home_qpos,
        task_dict=task_dict4)
    viser_execute_traj(vis, trajectory, armlee_trigger, armree_trigger)

    left_arm_user_pose = xpos_to_user_pose(
        w1_manager.get_xpos("left_arm", trajectory[-1], False)[1])
    right_arm_user_pose = xpos_to_user_pose(
        w1_manager.get_xpos("right_arm", trajectory[-1], False)[1])
    left_arm_target_pose = task_dict4["left_arm"][-1]
    right_arm_target_pose = task_dict4["right_arm"][-1]
    print(f"left_arm_user_pose: {left_arm_user_pose}")
    print(f"left_arm_target_pose: {left_arm_target_pose}")
    print(f"right_arm_user_pose: {right_arm_user_pose}")
    print(f"right_arm_target_pose: {right_arm_target_pose}")

    res, trajectory, armlee_trigger, armree_trigger = test_plan_linear_whole_body_ik(
        start_qpos=trajectory[-1],
        start_left_arm_ee_qpos=left_ee_home_qpos,
        start_right_arm_ee_qpos=right_ee_home_qpos,
        task_dict=task_dict5)
    viser_execute_traj(vis, trajectory, armlee_trigger, armree_trigger)

    left_arm_user_pose = xpos_to_user_pose(
        w1_manager.get_xpos("left_arm", trajectory[-1], False)[1])
    right_arm_user_pose = xpos_to_user_pose(
        w1_manager.get_xpos("right_arm", trajectory[-1], False)[1])
    left_arm_target_pose = task_dict5["left_arm"][-1]
    right_arm_target_pose = task_dict5["right_arm"][-1]
    print(f"left_arm_user_pose: {left_arm_user_pose}")
    print(f"left_arm_target_pose: {left_arm_target_pose}")
    print(f"right_arm_user_pose: {right_arm_user_pose}")
    print(f"right_arm_target_pose: {right_arm_target_pose}")

    res, trajectory, armlee_trigger, armree_trigger = test_plan_linear_whole_body_ik(
        start_qpos=trajectory[-1],
        start_left_arm_ee_qpos=left_ee_home_qpos,
        start_right_arm_ee_qpos=right_ee_home_qpos,
        task_dict=task_dict6)
    viser_execute_traj(vis, trajectory, armlee_trigger, armree_trigger)

    left_arm_user_pose = xpos_to_user_pose(
        w1_manager.get_xpos("left_arm", trajectory[-1], False)[1])
    right_arm_user_pose = xpos_to_user_pose(
        w1_manager.get_xpos("right_arm", trajectory[-1], False)[1])
    left_arm_target_pose = task_dict6["left_arm"][-1]
    right_arm_target_pose = task_dict6["right_arm"][-1]
    print(f"left_arm_user_pose: {left_arm_user_pose}")
    print(f"left_arm_target_pose: {left_arm_target_pose}")
    print(f"right_arm_user_pose: {right_arm_user_pose}")
    print(f"right_arm_target_pose: {right_arm_target_pose}")

    res, trajectory, armlee_trigger, armree_trigger = test_plan_linear_whole_body_ik(
        start_qpos=trajectory[-1],
        start_left_arm_ee_qpos=left_ee_home_qpos,
        start_right_arm_ee_qpos=right_ee_home_qpos,
        task_dict=task_dict7)
    viser_execute_traj(vis, trajectory, armlee_trigger, armree_trigger)

    left_arm_user_pose = xpos_to_user_pose(
        w1_manager.get_xpos("left_arm", trajectory[-1], False)[1])
    right_arm_user_pose = xpos_to_user_pose(
        w1_manager.get_xpos("right_arm", trajectory[-1], False)[1])
    left_arm_target_pose = task_dict7["left_arm"][-1]
    right_arm_target_pose = task_dict7["right_arm"][-1]
    print(f"left_arm_user_pose: {left_arm_user_pose}")
    print(f"left_arm_target_pose: {left_arm_target_pose}")
    print(f"right_arm_user_pose: {right_arm_user_pose}")
    print(f"right_arm_target_pose: {right_arm_target_pose}")

    res, trajectory, armlee_trigger, armree_trigger = test_plan_linear_whole_body_ik(
        start_qpos=trajectory[-1],
        start_left_arm_ee_qpos=left_ee_home_qpos,
        start_right_arm_ee_qpos=right_ee_home_qpos,
        task_dict=task_dict8)
    viser_execute_traj(vis, trajectory, armlee_trigger, armree_trigger)

    left_arm_user_pose = xpos_to_user_pose(
        w1_manager.get_xpos("left_arm", trajectory[-1], False)[1])
    right_arm_user_pose = xpos_to_user_pose(
        w1_manager.get_xpos("right_arm", trajectory[-1], False)[1])
    left_arm_target_pose = task_dict8["left_arm"][-1]
    right_arm_target_pose = task_dict8["right_arm"][-1]
    print(f"left_arm_user_pose: {left_arm_user_pose}")
    print(f"left_arm_target_pose: {left_arm_target_pose}")
    print(f"right_arm_user_pose: {right_arm_user_pose}")
    print(f"right_arm_target_pose: {right_arm_target_pose}")

    res, trajectory, armlee_trigger, armree_trigger = test_plan_linear_whole_body_ik(
        start_qpos=trajectory[-1],
        start_left_arm_ee_qpos=left_ee_home_qpos,
        start_right_arm_ee_qpos=right_ee_home_qpos,
        task_dict=task_dict9)
    viser_execute_traj(vis, trajectory, armlee_trigger, armree_trigger)

    left_arm_user_pose = xpos_to_user_pose(
        w1_manager.get_xpos("left_arm", trajectory[-1], False)[1])
    right_arm_user_pose = xpos_to_user_pose(
        w1_manager.get_xpos("right_arm", trajectory[-1], False)[1])
    left_arm_target_pose = task_dict9["left_arm"][-1]
    right_arm_target_pose = task_dict9["right_arm"][-1]
    print(f"left_arm_user_pose: {left_arm_user_pose}")
    print(f"left_arm_target_pose: {left_arm_target_pose}")
    print(f"right_arm_user_pose: {right_arm_user_pose}")
    print(f"right_arm_target_pose: {right_arm_target_pose}")

    embed()
