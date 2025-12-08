# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import numpy as np
from dexe_robot_client.common import W1Version, W1ArmType, get_logger
from dexe_robot_client.end_effector_client import EndEffectorType
from dexe_robot_client.w1 import W1Manager
from dexe_rp import SRSConfig


def arm_angle_motion():
    """利用SRSKinematics实现手臂冗余运动的示例"""
    w1_manager = W1Manager(is_sim=True,
                           arm_type=W1ArmType.ANTHROPOMORPHIC,
                           w1_version=W1Version.V2,
                           end_effector_type=EndEffectorType.BRAINCO_HAND)
    if not w1_manager.connect() and not w1_manager.is_enabled():
        return
    # yapf: disable
    w1_manager.set_home_qpos(np.deg2rad([45, -90, 45, 0, 0, 0,
                                        5, -45, -5, -90, 0, -15, 0,
                                        -5, 45, 5, 90, 0, 15, 0]),
                            save_to_file=False)
    # yapf: enable
    w1_manager.go_home(1.)

    # arm angle trajectory
    T = 10.
    t_list = np.arange(0, T + w1_manager.TIME_STEP,
                       w1_manager.TIME_STEP).round(2)
    delta_psi_list = np.deg2rad(30) * np.sin(2 * np.pi / T * t_list)

    # left arm
    cur_left_qpos = w1_manager.get_current_qpos("left_arm")[1]
    left_ref_psi = w1_manager.kine.left_arm_kine.calc_arm_angle(
        cur_left_qpos)[1]
    cur_left_xpos = w1_manager.kine.left_arm_kine.get_fk(cur_left_qpos)[1]

    left_psi_list = left_ref_psi + delta_psi_list
    left_qpos_seed = cur_left_qpos
    left_qpos_list = []
    for psi in left_psi_list:
        srs_cfg = SRSConfig(left_qpos_seed, psi)
        res, qpos = w1_manager.kine.left_arm_kine.get_ik(cur_left_xpos, srs_cfg)
        if not res:
            get_logger().error("ik failed")
            return
        if not np.allclose(qpos, left_qpos_seed, atol=np.deg2rad(90)):
            get_logger().error("continues failed")
            return
        left_qpos_list.append(qpos)
        left_qpos_seed = qpos

    # right arm
    cur_right_qpos = w1_manager.get_current_qpos("right_arm")[1]
    right_ref_psi = w1_manager.kine.right_arm_kine.calc_arm_angle(
        cur_right_qpos)[1]
    cur_right_xpos = w1_manager.kine.right_arm_kine.get_fk(cur_right_qpos)[1]

    right_psi_list = right_ref_psi - delta_psi_list
    right_qpos_seed = cur_right_qpos
    right_qpos_list = []
    for psi in right_psi_list:
        srs_cfg = SRSConfig(right_qpos_seed, psi)
        res, qpos = w1_manager.kine.right_arm_kine.get_ik(
            cur_right_xpos, srs_cfg)
        if not res:
            get_logger().error("ik failed")
            return
        if not np.allclose(qpos, right_qpos_seed, atol=np.deg2rad(90)):
            get_logger().error("continues failed")
            return
        right_qpos_list.append(qpos)
        right_qpos_seed = qpos

    # align
    cur_qpos = w1_manager.get_current_qpos()[1]
    qpos_array = np.repeat(np.atleast_2d(cur_qpos), t_list.shape[0], axis=0)
    qpos_array[:, 6:13] = np.array(left_qpos_list)
    qpos_array[:, 13:20] = np.array(right_qpos_list)

    # execute
    w1_manager.execute_joint_traj(qpos_array)


if __name__ == "__main__":
    arm_angle_motion()
