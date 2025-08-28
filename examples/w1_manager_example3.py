# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import numpy as np
from dexe_robot_client.common import W1Version, W1ArmType, get_logger
from dexe_robot_client.end_effector_client import EndEffectorType 
from dexe_robot_client.w1 import W1Manager


def arm_angle_motion():
    w1_manager = W1Manager(is_sim=True,
                        arm_type=W1ArmType.ANTHROPOMORPHIC, 
                        w1_version=W1Version.V2, 
                        end_effector_type=EndEffectorType.BRAINCO_HAND,
                        use_rlia=False)
    if not w1_manager.connect():
        return
     
    w1_manager.go_home(1.)

    # arm angle trajectory
    T = 10.
    t_list = np.arange(0, T + w1_manager.TIME_STEP, w1_manager.TIME_STEP).round(2)
    delta_psi_list = np.deg2rad(45) * np.sin(2*np.pi / T * t_list)

    # left arm
    cur_left_qpos = w1_manager.get_current_qpos("left_arm")[1]
    left_ref_psi = w1_manager.left_arm_kine._calc_arm_angle(cur_left_qpos)[1]
    cur_left_xpos = w1_manager.left_arm_kine.get_fk(cur_left_qpos)[1]
    
    left_psi_list = left_ref_psi + delta_psi_list
    left_qpos_seed = cur_left_qpos
    left_qpos_list = []
    for psi in left_psi_list:
        res, qpos = w1_manager.left_arm_kine.get_ik(cur_left_xpos, psi, 
                                                    np.sign(left_qpos_seed[[1,3,5]]))
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
    right_ref_psi = w1_manager.right_arm_kine._calc_arm_angle(cur_right_qpos)[1]
    cur_right_xpos = w1_manager.right_arm_kine.get_fk(cur_right_qpos)[1]

    right_psi_list = right_ref_psi - delta_psi_list
    right_qpos_seed = cur_right_qpos
    right_qpos_list = []
    for psi in right_psi_list:
        res, qpos = w1_manager.right_arm_kine.get_ik(cur_right_xpos, psi, 
                                                    np.sign(right_qpos_seed[[1,3,5]]))
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
    qpos_array[:,6:13] = np.array(left_qpos_list)
    qpos_array[:,13:20] = np.array(right_qpos_list)

    # check collision
    if w1_manager._check_traj_collision(qpos_array):
        get_logger().error("Trajectory is collision, do not executed!")
        return

    # execute
    w1_manager.execute_joint_traj(qpos_array)

if __name__ == "__main__":
    arm_angle_motion()