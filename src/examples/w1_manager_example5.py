'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

import numpy as np
from IPython import embed
from scipy.spatial.transform import Rotation as R

from viser_utils import load_viser, viser_execute_traj
from dexe_robot_client.w1 import W1Manager
from dexe_robot_client.common import StatusCode

from dexe_rp import RobotVersion, EndEffectorType
"""
This example shows usages of dual arm fixed-connected move-line planning, including only dual-arm planning and whole body planning

RobotVersion: W1V024
Left_EndEffectorType: BrainCo_Revo1_R
Right_EndEffectorTYpe: BrainCo_Revo1_R

"""

IS_LOCAL = False
ROBOT_VERSION = RobotVersion.W1V024
END_EFFECTOR_TYPE = EndEffectorType.BrainCo_Revo1_R

# arm-base teached point
task_dict1 = {
    "torso": [[0.78539816, -1.57079633, 0.78539816, 0.03490659]],
    "torso_speed_ratio": [1.],
    "head": [[0., -0.6]],
    "head_speed_ratio": [1.],
    "left_arm": [
        [-156.14, 545.92, 38.57, -91.46, -5.05, -1.15],
        [14.2, 571.47, 42.12, -92.38, -0.12, -3.59],
    ],
    "left_arm_speed_ratio": [
        0.5,
        0.5,
        0.5,
    ],
    "left_arm_ee": [],
    "right_arm": [],
    "right_arm_speed_ratio": [],
    "right_arm_ee": [],
}

# chassis-base teached point
task_dict2 = {
    "torso": [[0.78539816, -1.57079633, 0.78539816, 0.03490659]],
    "torso_speed_ratio": [1.],
    "head": [[0., -0.6]],
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
    "left_arm_speed_ratio": [1.0, 1.0, 1.0, 1.0],
    "left_arm_ee": [],
    "right_arm": [],
    "right_arm_speed_ratio": [],
    "right_arm_ee": [],
}
task_dict3 = {
    "torso": [[-0.78539816, 1.57079633, -0.78539816, 0.03490659]],
    "torso_speed_ratio": [1.],
    "head": [[0., -0.6]],
    "head_speed_ratio": [1.],
    "left_arm": [
        [
            436.81552887392564, 151.8864369061307, 1233.6224827317324,
            153.7526570682316, -85.14610235747323, 24.605372677060252
        ],
        [
            436.81552887392564, 151.8864369061307, 955.8654830562698,
            153.7526570682316, -85.14610235747323, 24.605372677060252
        ],
        [
            436.81552887392564, 151.8864369061307, 663.5256017685385,
            153.7526570682316, -85.14610235747323, 24.605372677060252
        ],
        [
            436.81552887392564, 151.8864369061307, 477.51278916762175,
            153.7526570682316, -85.14610235747323, 24.605372677060252
        ],
    ],
    "left_arm_speed_ratio": [1.0, 1.0, 1.0, 1.0],
    "left_arm_ee": [],
    "right_arm": [],
    "right_arm_speed_ratio": [],
    "right_arm_ee": [],
}

if __name__ == "__main__":

    w1_manager = W1Manager(is_sim=True,
                           robot_version=ROBOT_VERSION,
                           end_effector_type=END_EFFECTOR_TYPE,
                           collision_logger_verbose=False)

    # # judge w1_manager is connected and enabled
    # if not w1_manager.connect() and not w1_manager.is_enabled():
    #     raise RuntimeError("Failed to connect to W1 robot.")

    w1_manager.set_tcp("left_arm", np.eye(4))
    right_tcp_xpos = np.eye(4)
    right_tcp_xpos[:3, :3] = R.from_rotvec([0.0, 0.0, 180.],
                                           degrees=True).as_matrix()

    # yapf: disable
    if IS_LOCAL:
        home_qpos = np.array([
            0.78539816, -1.57079633, 0.78539816, 0.03490659,
            0., -0.6,
            -0.98, -1.451, 1.622, -0.479, -1.453, -0.513, 0.33,
            0.969, 1.416, -1.594, 0.559, 1.263, 0.53, -0.436
        ])
    else:
        home_qpos = np.array([
            1.353, -2.705, 1.395, 0.009,
            0.0, -0.6,
            -0.891, -1.655, 1.55, -0.023, -1.514, -0.633, 0.068,
            0.894, 1.7, -1.529, 0.021, 1.324, 0.679, -0.14
        ])

    left_ee_home_qpos = np.array([0.0]*6)
    right_ee_home_qpos = np.array([0.0]*6)

    w1_manager.set_home_qpos(home_qpos, save_to_file=False)
    w1_manager.set_ee_qpos("left_arm_ee", left_ee_home_qpos)
    w1_manager.set_ee_qpos("right_arm_ee", right_ee_home_qpos)
    w1_manager.go_home(1)

    vis, scene_handle_list, workobjects_handle_list = load_viser(home_qpos, left_ee_home_qpos, right_ee_home_qpos)

    input("Press Enter to start test...")

    if IS_LOCAL:
        res, full_body_traj = w1_manager.plan_linear_dual_arm_sync(
            task_dict=task_dict1,
            start_qpos=home_qpos,
            start_left_arm_ee_qpos=left_ee_home_qpos,
            start_right_arm_ee_qpos=right_ee_home_qpos,
            speed_ratio=1.0,
            is_local=True,
            use_collision_validator=True
        )
        viser_execute_traj(vis, full_body_traj, None, None)
    else:
        res, full_body_traj = w1_manager.plan_linear_dual_arm_sync(
            task_dict=task_dict2,
            start_qpos=home_qpos,
            start_left_arm_ee_qpos=left_ee_home_qpos,
            start_right_arm_ee_qpos=right_ee_home_qpos,
            speed_ratio=1.0,
            is_local=False,
            use_collision_validator=True
        )
        viser_execute_traj(vis, full_body_traj, None, None)

        # change to leg leaning backward
        ref_qpos = full_body_traj[-1]
        leg_backward_qpos = ref_qpos.copy()
        leg_backward_qpos[0:3] = -leg_backward_qpos[0:3]
        task_dict_plan_joint = {
            "torso": [leg_backward_qpos[0:4].tolist()],
            "torso_speed_ratio": [1.],
            "head": [leg_backward_qpos[4:6].tolist()],
            "head_speed_ratio": [1.],
            "left_arm": [leg_backward_qpos[6:13].tolist()],
            "left_arm_speed_ratio": [1.],
            "left_arm_ee": [],
            "right_arm": [leg_backward_qpos[13:20].tolist()],
            "right_arm_speed_ratio": [1.],
            "right_arm_ee": [],
        }
        res, full_body_traj, armlee_trigger, armree_trigger = w1_manager.plan_joint_full_body(
            start_qpos=ref_qpos,
            start_left_arm_ee_qpos=left_ee_home_qpos,
            start_right_arm_ee_qpos=right_ee_home_qpos,
            waypoints_dict=task_dict_plan_joint,
            speed_ratio=1.0,
            use_collision_validator=True)

        viser_execute_traj(vis, full_body_traj, None, None)

        # change ref qpos for whole body ik
        res, full_body_traj = w1_manager.plan_linear_dual_arm_sync(
            task_dict=task_dict3,
            start_qpos=full_body_traj[-1],
            start_left_arm_ee_qpos=left_ee_home_qpos,
            start_right_arm_ee_qpos=right_ee_home_qpos,
            speed_ratio=1.0,
            is_local=False,
            use_collision_validator=True,
            is_leg_leaning_forward=False
        )
        viser_execute_traj(vis, full_body_traj, None, None)
