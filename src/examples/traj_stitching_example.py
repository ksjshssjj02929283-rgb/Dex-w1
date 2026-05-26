'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

import numpy as np
from dexe_robot_client.end_effector_client import EndEffectorType
from dexe_robot_client.common import W1Version, W1ArmType, time_consuming, StatusCode
from dexe_robot_client.w1 import W1Manager
from dexe_robot_client.robot_client import MotionGroup
from IPython import embed

def main(is_sim=True):
    w1_manager = W1Manager(is_sim=is_sim,
                           arm_type=W1ArmType.ANTHROPOMORPHIC,
                           w1_version=W1Version.V2,
                           end_effector_type=None)
    if not w1_manager.connect() and not w1_manager.is_enabled():
        return
    w1_manager.set_home_qpos(np.deg2rad([45, -90, 45, 0, 0, 0,
                                        5, -45, -5, -90, 0, -15, 0,
                                        -5, 45, 5, 90, 0, 15, 0]),
                            save_to_file=False)
    w1_manager.go_home(0.2)  

    res1, start_qpos = w1_manager.get_current_qpos()
    if res1 != StatusCode.SUCCESS:
        return
    res2, pose_left_arm0 = w1_manager.get_current_pose("left_arm", is_local=False)
    if res2 != StatusCode.SUCCESS:
        return
    
    pose_left_arm1 = (np.array(pose_left_arm0) + np.array([25,0,0,5,0,0])).tolist()
    pose_left_arm2 = (np.array(pose_left_arm1) + np.array([25,0,0,5,0,0])).tolist()
    pose_left_arm3 = (np.array(pose_left_arm2) + np.array([25,25,0,0,0,0])).tolist()
    pose_left_arm4 = (np.array(pose_left_arm3) + np.array([0,25,0,0,0,0])).tolist()
    pose_left_arm5 = (np.array(pose_left_arm4) + np.array([-25,25,0,0,0,0])).tolist()
    pose_left_arm6 = (np.array(pose_left_arm5) + np.array([-50,0,0,-10,0,0])).tolist()

    pose_list = [pose_left_arm1, pose_left_arm2, pose_left_arm3, pose_left_arm4, pose_left_arm5, pose_left_arm6]
    qpos_traj = []
    start_qpos_ = start_qpos
    start_speed_ratio_ = 0.0
    end_speed_ratio_ = 0.5
    for i in range(6):
        res, qpos_traj_ = w1_manager.plan_linear_single_arm(is_left=True, 
                                                            start_qpos=start_qpos_, 
                                                            waypoints_list=[pose_list[i]],
                                                            speed_ratio_list=[1.0],
                                                            start_speed_ratio=start_speed_ratio_,
                                                            end_speed_ratio=end_speed_ratio_,
                                                            path_blend_tolerance=0.01,
                                                            is_local=False)
        if res != StatusCode.SUCCESS:
            return
        qpos_traj += qpos_traj_[1:].tolist()

        start_qpos_[6:13] = qpos_traj_[-1]
        start_speed_ratio_ = end_speed_ratio_
        if i == 5:
            end_speed_ratio_ = 0.0
        else:
            end_speed_ratio_ = 0.5

    qpos_traj = np.array(qpos_traj)
    w1_manager.robot_client.start_joint_streaming_request(MotionGroup.Primary, use_velocity=False)
    for qpos in qpos_traj:
        w1_manager.robot_client.send_streaming_command(qpos.tolist(), streaming_finished=False)
        w1_manager.robot_client.wait_second(w1_manager.TIME_STEP)
    w1_manager.robot_client.send_streaming_command(qpos_traj[-1].tolist(), streaming_finished=True)

    # from matplotlib import pyplot as plt
    # traj = qpos_traj
    # vel_traj = np.diff(traj, axis=0)/w1_manager.TIME_STEP

    # fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    # for i in range(7):
    #     axes[0].plot(traj[:,i], label=f"joint {i+1}")
    # axes[0].legend()
    # axes[0].grid()
    # for i in range(7):
    #     axes[1].plot(vel_traj[:,i], label=f"joint {i+1}")
    # axes[1].legend()
    # axes[1].grid()
    # plt.show()

if __name__ == "__main__":
    main(is_sim=True)