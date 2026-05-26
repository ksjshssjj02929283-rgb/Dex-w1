'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

import threading
import queue
import time
import numpy as np
from dexe_robot_client.common import W1Version, W1ArmType, StatusCode
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

    streaming_buffer = queue.Queue()
    finish_token = object()
    streaming_period = w1_manager.TIME_STEP

    def control_thread():
        w1_manager.robot_client.start_joint_streaming_request(MotionGroup.Primary, use_velocity=False)
        last_qpos = np.array(start_qpos[6:13], dtype=float).tolist()
        while True:
            start_time = time.monotonic()
            try:
                qpos = streaming_buffer.get_nowait()
            except queue.Empty:
                qpos = last_qpos

            if qpos is finish_token:
                w1_manager.robot_client.send_streaming_command(last_qpos, streaming_finished=True)
                break

            last_qpos = qpos
            w1_manager.robot_client.send_streaming_command(qpos, streaming_finished=False)
            elapsed = time.monotonic() - start_time
            if elapsed < streaming_period:
                w1_manager.robot_client.wait_second(streaming_period - elapsed)

    def planning_thread():
        start_qpos_ = np.array(start_qpos, dtype=float, copy=True)
        start_speed_ratio_ = 0.0
        end_speed_ratio_ = 1.0

        try:
            for i, pose in enumerate(pose_list):
                res, qpos_traj_ = w1_manager.plan_linear_single_arm(is_left=True,
                                                                     start_qpos=start_qpos_,
                                                                     waypoints_list=[pose],
                                                                     speed_ratio_list=[1.0],
                                                                     start_speed_ratio=start_speed_ratio_,
                                                                     end_speed_ratio=end_speed_ratio_,
                                                                     path_blend_tolerance=0.01,
                                                                     is_local=False)
                if res != StatusCode.SUCCESS:
                    print(f"plan_linear_single_arm failed at segment {i}: {res}")
                    break

                for qpos in qpos_traj_[1:]:
                    streaming_buffer.put(qpos.tolist())

                start_qpos_[6:13] = qpos_traj_[-1]
                start_speed_ratio_ = end_speed_ratio_
                end_speed_ratio_ = 0.0 if i == len(pose_list) - 1 else 0.5
        finally:
            streaming_buffer.put(finish_token)

    control_worker = threading.Thread(target=control_thread)
    planning_worker = threading.Thread(target=planning_thread)

    control_worker.start()
    planning_worker.start()
    planning_worker.join()
    control_worker.join()

if __name__ == "__main__":
    main(is_sim=True)
