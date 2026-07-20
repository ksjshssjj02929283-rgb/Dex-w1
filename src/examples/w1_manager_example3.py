'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

import numpy as np
from dexe_robot_client.common import get_logger
from dexe_robot_client.w1 import W1Manager
from dexe_rp import StatusType


def arm_angle_motion():
    """利用SRSKinematics实现手臂冗余运动的示例"""
    w1_manager = W1Manager.create(is_sim=True)
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
    delta_rp_list = np.deg2rad(30) * np.sin(2 * np.pi / T * t_list)

    # left arm
    cur_left_qpos = w1_manager.get_current_qpos("left_arm")[1]
    left_ref_rp = w1_manager.kine.left_arm_kine.get_config(
        cur_left_qpos).redundancy_parameter
    cur_left_xpos = w1_manager.kine.left_arm_kine.get_fk(cur_left_qpos)

    left_rp_list = left_ref_rp + delta_rp_list
    left_qpos_seed = cur_left_qpos
    left_qpos_list = []
    for redundancy_parameter in left_rp_list:
        cfg = w1_manager.kine.left_arm_kine.get_config(left_qpos_seed,
                                                       redundancy_parameter)
        res_status, qpos = w1_manager.kine.left_arm_kine.get_ik(
            cur_left_xpos, cfg, left_qpos_seed)
        if res_status != StatusType.Success:
            get_logger().error("ik failed")
            return
        if not np.allclose(qpos, left_qpos_seed, atol=np.deg2rad(90)):
            get_logger().error("continues failed")
            return
        left_qpos_list.append(qpos)
        left_qpos_seed = qpos

    # right arm
    cur_right_qpos = w1_manager.get_current_qpos("right_arm")[1]
    right_ref_rp = w1_manager.kine.right_arm_kine.get_config(
        cur_right_qpos).redundancy_parameter
    cur_right_xpos = w1_manager.kine.right_arm_kine.get_fk(cur_right_qpos)

    right_rp_list = right_ref_rp - delta_rp_list
    right_qpos_seed = cur_right_qpos
    right_qpos_list = []
    for redundancy_parameter in right_rp_list:
        cfg = w1_manager.kine.right_arm_kine.get_config(right_qpos_seed,
                                                        redundancy_parameter)
        res_status, qpos = w1_manager.kine.right_arm_kine.get_ik(
            cur_right_xpos, cfg, right_qpos_seed)
        if res_status != StatusType.Success:
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
