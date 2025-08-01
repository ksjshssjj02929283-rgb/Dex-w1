# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import numpy as np
from copy import deepcopy

from rlia.kit.drive_controllers.w1_drive_controller import W1Version, W1ArmType
from dexe_robot_client.end_effector_client import EndEffectorType 
from dexe_robot_client.w1 import W1Manager


def move_full_body_example1(move_with_ee: bool, is_sim: bool = True):
    if move_with_ee:
        end_effector_type = EndEffectorType.BRAINCO_HAND
    else:
        end_effector_type = None

    w1_manager = W1Manager(is_sim = is_sim,
                            arm_type=W1ArmType.ANTHROPOMORPHIC, 
                            w1_version=W1Version.V2, 
                            end_effector_type = end_effector_type)
    
    w1_manager.set_home_qpos(np.deg2rad([45, -90, 45, 0, 0, 0,
                                      5, -45, -5, -90, 0, -15, 0,
                                      -5, 45, 5, 90, 0, 15, 0]),
                            save_to_file=False)

    w1_manager.set_ee_state("left_arm_ee", np.array([0.]*6))
    w1_manager.set_ee_state("right_arm_ee", np.array([0.]*6))
    w1_manager.go_home(0.5)    

    torso_waypoints = [[0.7854, -1.5708,  0.7853, -0.],
                       [0.8854, -1.7708,  0.8853, -0.1],
                       [0.8854, -1.7708,  0.8853, 0.1],
                       [0.7854, -1.5708,  0.7853, -0.]]

    head_waypoints = [[0.5, 0.3],
                      [-0.5, -0.3],
                      [0., -0.3]]

    left_arm_waypoints = np.deg2rad([[5., -45., -5., -90., 0., -15., 0.],
                                     [15., -55., -15., -100., 10., -10., 10.],
                                     [-5., -35., 5., -80., -10., 10., -10.],
                                     [5., -45., -5., -90., 0., -15., 0.]])
    
    left_arm_ee_states = [[0., [0., 0.5, 0.5, 0.5, 0.5, 0.5], 0.5],
                          [0., [0., 0., 0., 0., 0., 0.], 0.5],
                          [0., [1., 1., 0., 0., 1., 1.], 0.5],
                          [0., [0., 0., 0., 0., 0., 0.], 0.]]

    right_arm_waypoints = -left_arm_waypoints

    right_arm_ee_states = [[0., [1., 1., 0., 0., 1., 1.], 0.5],
                          [0., [0., 0., 0., 0., 0., 0.], 0.5],
                          [0., [0., 0.5, 0.5, 0.5, 0.5, 0.5], 0.5],
                          [0., [0., 0., 0., 0., 0., 0.], 0.]]

    if move_with_ee:
        w1_manager.move_joint_full_body(waypoints_dict = {
                                                        "torso": torso_waypoints,
                                                        "torso_speed_ratio": [0.3, 0.3, 0.3, 0.3],
                                                        "head": head_waypoints,
                                                        "head_speed_ratio": [1., 1., 1.],
                                                        "left_arm": left_arm_waypoints.tolist(),
                                                        "left_arm_ee": left_arm_ee_states,
                                                        "right_arm": right_arm_waypoints.tolist(),
                                                        "right_arm_ee": right_arm_ee_states
                                                        }, speed_ratio=0.5)
    else:
        w1_manager.move_joint_full_body(waypoints_dict = {
                                                        "torso": torso_waypoints,
                                                        "torso_speed_ratio": [0.3, 0.3, 0.3, 0.3],
                                                        "head": head_waypoints,
                                                        "head_speed_ratio": [1., 1., 1.],
                                                        "left_arm": left_arm_waypoints.tolist(),
                                                        "right_arm": right_arm_waypoints.tolist(),
                                                        }, speed_ratio=0.5)


def move_full_body_example2(move_with_ee: bool, is_sim: bool = True):
    if move_with_ee:
        end_effector_type = EndEffectorType.BRAINCO_HAND
    else:
        end_effector_type = None
    
    w1_manager = W1Manager(is_sim = is_sim,
                            arm_type=W1ArmType.ANTHROPOMORPHIC, 
                            w1_version=W1Version.V2, 
                            end_effector_type = end_effector_type)
    
    w1_manager.set_tcp("left_arm", np.eye(4))
    w1_manager.set_tcp("right_arm", np.eye(4))

    w1_manager.set_home_qpos(np.deg2rad([45, -90, 45, 0, 0, 0,
                                      5, -45, -5, -90, 0, -15, 0,
                                      -5, 45, 5, 90, 0, 15, 0]),
                            save_to_file=False)

    w1_manager.set_ee_state("left_arm_ee", np.array([0.]*6))
    w1_manager.set_ee_state("right_arm_ee", np.array([0.]*6))
    w1_manager.go_home(0.5)  

    cur_l_pose = w1_manager.get_current_pose("left_arm")[1]
    l_pose1 = deepcopy(cur_l_pose)
    l_pose1[0] += -100
    l_pose1[2] += -100
    l_pose2 = deepcopy(l_pose1)
    l_pose2[2] += 100
    l_pose3 = deepcopy(l_pose2)
    l_pose3[0] += 100
    l_pose3[2] += -100
    l_pose4 = deepcopy(l_pose3)
    l_pose4[2] += 100
    left_arm_waypoints = [l_pose1, l_pose2, l_pose3, l_pose4]
    left_arm_ee_states = [[0., [0., 0.5, 0.5, 0.5, 0.5, 0.5], 0.5],
                          [0., [0., 0., 0., 0., 0., 0.], 0.5],
                          [0., [1., 1., 0., 0., 1., 1.], 0.5],
                          [0., [0., 0., 0., 0., 0., 0.], 0.]]

    cur_r_pose = w1_manager.get_current_pose("right_arm")[1]
    r_pose1 = deepcopy(cur_r_pose)
    r_pose1[0] += 100
    r_pose1[2] += -100
    r_pose2 = deepcopy(r_pose1)
    r_pose2[2] += 100
    r_pose3 = deepcopy(r_pose2)
    r_pose3[0] += -100
    r_pose3[2] += -100
    r_pose4 = deepcopy(r_pose3)
    r_pose4[2] += 100
    right_arm_waypoints = [r_pose1, r_pose2, r_pose3, r_pose4]
    right_arm_ee_states = [[0., [1., 1., 0., 0., 1., 1.], 0.5],
                          [0., [0., 0., 0., 0., 0., 0.], 0.5],
                          [0., [0., 0.5, 0.5, 0.5, 0.5, 0.5], 0.5],
                          [0., [0., 0., 0., 0., 0., 0.], 0.]]

    if move_with_ee:
        w1_manager.move_linear_full_body(waypoints_dict = {"left_arm": left_arm_waypoints,
                                                           "left_arm_ee": left_arm_ee_states,
                                                            "right_arm": right_arm_waypoints,
                                                            "right_arm_ee": right_arm_ee_states}, 
                                                            speed_ratio=0.5)
    else:
        w1_manager.move_linear_full_body(waypoints_dict = {"left_arm": left_arm_waypoints,
                                                            "right_arm": right_arm_waypoints}, 
                                                            speed_ratio=0.5)


if __name__ == "__main__":
    move_full_body_example1(move_with_ee = False, is_sim = True)
    # move_full_body_example2(move_with_ee = False, is_sim = True)