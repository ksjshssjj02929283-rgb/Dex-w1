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

def main(is_sim=True):
    w1_manager = W1Manager(is_sim=is_sim,
                           arm_type=W1ArmType.ANTHROPOMORPHIC,
                           w1_version=W1Version.V2,
                           end_effector_type=EndEffectorType.BRAINCO_HAND)
    if not w1_manager.connect() and not w1_manager.is_enabled():
        return
    w1_manager.set_home_qpos(np.deg2rad([45, -90, 45, 0, 0, 0,
                                        5, -45, -5, -90, 0, -15, 0,
                                        -5, 45, 5, 90, 0, 15, 0]),
                            save_to_file=False)
    w1_manager.go_home(0.2)  
    w1_manager.set_ee_qpos("left_arm_ee", np.array([0] * 6))
    w1_manager.set_ee_qpos("right_arm_ee", np.array([0] * 6))

    res1, start_qpos = w1_manager.get_current_qpos()
    if res1 != StatusCode.SUCCESS:
        return
    res2, start_left_ee_qpos = w1_manager.get_current_qpos("left_arm_ee")
    if res2 != StatusCode.SUCCESS:
        start_left_ee_qpos = None
    res3, start_right_ee_qpos = w1_manager.get_current_qpos("right_arm_ee")
    if res3 != StatusCode.SUCCESS:
        start_right_ee_qpos = None
    res4, pose_left_arm0 = w1_manager.get_current_pose("left_arm", is_local=False)
    if res4 != StatusCode.SUCCESS:
        return
    res5, pose_right_arm0 = w1_manager.get_current_pose("right_arm", is_local=False)
    if res5 != StatusCode.SUCCESS:
        return

    pose_left_arm1 = (np.array(pose_left_arm0) + np.array([50,0,0,5,0,0])).tolist()
    pose_left_arm2 = (np.array(pose_left_arm1) + np.array([0,25,0,0,0,0])).tolist()
    # break here
    pose_left_arm3 = (np.array(pose_left_arm2) + np.array([0,25,0,0,0,0])).tolist()
    pose_left_arm4 = (np.array(pose_left_arm3) + np.array([-50,0,0,-5,0,0])).tolist()

    pose_right_arm1 = (np.array(pose_right_arm0) + np.array([50,0,0,-5,0,0])).tolist()
    pose_right_arm2 = (np.array(pose_right_arm1) + np.array([0,-25,0,0,0,0])).tolist()
    # break here
    pose_right_arm3 = (np.array(pose_right_arm2) + np.array([0,-25,0,0,0,0])).tolist()
    pose_right_arm4 = (np.array(pose_right_arm3) + np.array([-50,0,0,5,0,0])).tolist()

    res6, qpos_traj1 = w1_manager.plan_linear_dual_arm(start_qpos, start_left_ee_qpos, start_right_ee_qpos,
                                                    task_dict={"left_arm": [pose_left_arm1, pose_left_arm2],
                                                            "left_arm_speed_ratio": [1, 1],
                                                            "left_arm_start_speed_ratio": 0,
                                                            "left_arm_end_speed_ratio": 1.,
                                                            "right_arm": [pose_right_arm1, pose_right_arm2],
                                                            "right_arm_speed_ratio": [1, 1],
                                                            "right_arm_start_speed_ratio": 0.,
                                                            "right_arm_end_speed_ratio": 1.},
                                                    path_blend_tolerance=0.01,
                                                    is_local=False,
                                                    use_collision_validator=True)
    if res6 != StatusCode.SUCCESS:
        from IPython import embed; embed(banner1="70")
        return
    res7, qpos_traj2 = w1_manager.plan_linear_dual_arm(qpos_traj1[-1], start_left_ee_qpos, start_right_ee_qpos,
                                                    task_dict={"left_arm": [pose_left_arm3, pose_left_arm4],
                                                            "left_arm_speed_ratio": [1, 1],
                                                            "left_arm_start_speed_ratio": 1.,
                                                            "left_arm_end_speed_ratio": 0,
                                                            "right_arm": [pose_right_arm3, pose_right_arm4],
                                                            "right_arm_speed_ratio": [1, 1],
                                                            "right_arm_start_speed_ratio": 1.,
                                                            "right_arm_end_speed_ratio": 0.},
                                                    path_blend_tolerance=0.01,
                                                    is_local=False,
                                                    use_collision_validator=True)
    if res7 != StatusCode.SUCCESS:
        from IPython import embed; embed(banner1="70")
        return

    w1_manager.execute_joint_traj(qpos_traj1)
    w1_manager.execute_joint_traj(qpos_traj2)

    from matplotlib import pyplot as plt
    traj = np.vstack((qpos_traj1, qpos_traj2[1:,:]))
    vel_traj = np.diff(traj, axis=0)/w1_manager.TIME_STEP

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    for i in range(7):
        axes[0].plot(traj[:,6+i], label=f"joint {i+1}")
    axes[0].legend()
    axes[0].grid()
    for i in range(7):
        axes[1].plot(vel_traj[:,6+i], label=f"joint {i+1}")
    axes[1].legend()
    axes[1].grid()
    plt.show()

if __name__ == "__main__":
    main(is_sim=True)