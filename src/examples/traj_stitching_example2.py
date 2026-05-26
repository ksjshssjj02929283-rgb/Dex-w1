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
    res3, pose_right_arm0 = w1_manager.get_current_pose("right_arm", is_local=False)
    if res3 != StatusCode.SUCCESS:
        return
    
    pose_left_arm1 = (np.array(pose_left_arm0) + np.array([25,0,0,5,0,0])).tolist()
    pose_left_arm2 = (np.array(pose_left_arm1) + np.array([25,0,0,5,0,0])).tolist()
    pose_left_arm3 = (np.array(pose_left_arm2) + np.array([25,25,0,0,0,0])).tolist()
    pose_left_arm4 = (np.array(pose_left_arm3) + np.array([0,25,0,0,0,0])).tolist()
    pose_left_arm5 = (np.array(pose_left_arm4) + np.array([-25,25,0,0,0,0])).tolist()
    pose_left_arm6 = (np.array(pose_left_arm5) + np.array([-50,0,0,-10,0,0])).tolist()

    pose_right_arm1 = (np.array(pose_right_arm0) + np.array([25,100,0,-5,0,0])).tolist()
    pose_right_arm2 = (np.array(pose_right_arm1) + np.array([50,0,0,-5,0,0])).tolist()
    pose_right_arm3 = (np.array(pose_right_arm2) + np.array([25,-25,0,0,0,0])).tolist()
    pose_right_arm4 = (np.array(pose_right_arm3) + np.array([0,-25,0,0,0,0])).tolist()
    pose_right_arm5 = (np.array(pose_right_arm4) + np.array([-25,-25,0,0,0,0])).tolist()
    pose_right_arm6 = (np.array(pose_right_arm5) + np.array([-50,0,0,10,0,0])).tolist()
    pose_right_arm7 = (np.array(pose_right_arm6) + np.array([-50,0,0,0,0,0])).tolist()
    pose_right_arm8 = (np.array(pose_right_arm7) + np.array([0,0,50,0,0,0])).tolist()

    task_list = [
        {"left_arm": pose_left_arm1, "right_arm": pose_right_arm1},
        {"left_arm": pose_left_arm2, "right_arm": pose_right_arm2},
        {"left_arm": pose_left_arm3, "right_arm": pose_right_arm3},
        {"left_arm": pose_left_arm4, "right_arm": pose_right_arm4},
        {"left_arm": pose_left_arm5, "right_arm": pose_right_arm5},
        {"left_arm": pose_left_arm6, "right_arm": pose_right_arm6},
        {"left_arm": None, "right_arm": pose_right_arm7},
        {"left_arm": None, "right_arm": pose_right_arm8},
    ]

    left_streaming_buffer = queue.Queue()
    right_streaming_buffer = queue.Queue()
    finish_token = object()
    streaming_period = w1_manager.TIME_STEP

    def control_thread():
        w1_manager.robot_client.start_joint_streaming_request(MotionGroup.BothArms, use_velocity=False)
        last_left_qpos = np.array(start_qpos[6:13], dtype=float).tolist()
        last_right_qpos = np.array(start_qpos[13:20], dtype=float).tolist()
        left_finished = False
        right_finished = False
        while True:
            start_time = time.monotonic()
            if not left_finished:
                try:
                    left_qpos = left_streaming_buffer.get_nowait()
                    if left_qpos is finish_token:
                        left_finished = True
                        left_qpos = last_left_qpos
                except queue.Empty:
                    left_qpos = last_left_qpos
            else:
                left_qpos = last_left_qpos

            if not right_finished:
                try:
                    right_qpos = right_streaming_buffer.get_nowait()
                    if right_qpos is finish_token:
                        right_finished = True
                        right_qpos = last_right_qpos
                except queue.Empty:
                    right_qpos = last_right_qpos
            else:
                right_qpos = last_right_qpos

            if left_finished and right_finished:
                w1_manager.robot_client.send_streaming_command(last_left_qpos + last_right_qpos,
                                                               streaming_finished=True)
                break

            last_left_qpos = left_qpos
            last_right_qpos = right_qpos
            w1_manager.robot_client.send_streaming_command(left_qpos + right_qpos, streaming_finished=False)
            elapsed = time.monotonic() - start_time
            if elapsed < streaming_period:
                w1_manager.robot_client.wait_second(streaming_period - elapsed)

    def planning_thread():
        start_qpos_ = np.array(start_qpos, dtype=float, copy=True)

        left_start_speed_ratio = 0.0
        left_end_speed_ratio = 1.0
        right_start_speed_ratio = 0.0
        right_end_speed_ratio = 1.0

        def plan_arm(is_left, pose, start_speed_ratio, end_speed_ratio):
            return w1_manager.plan_linear_single_arm(is_left=is_left,
                                                     start_qpos=start_qpos_,
                                                     waypoints_list=[pose],
                                                     speed_ratio_list=[1.0],
                                                     start_speed_ratio=start_speed_ratio,
                                                     end_speed_ratio=end_speed_ratio,
                                                     path_blend_tolerance=0.01,
                                                     is_local=False)

        try:
            for i, task in enumerate(task_list):
                is_last_task = i == len(task_list) - 1
                left_pose = task.get("left_arm")
                right_pose = task.get("right_arm")
                left_traj = None
                right_traj = None

                if left_pose is not None:
                    left_end_speed_ratio = 0.0 if is_last_task else 0.5
                    res, left_traj = plan_arm(True,
                                              left_pose,
                                              left_start_speed_ratio,
                                              left_end_speed_ratio)
                    if res != StatusCode.SUCCESS:
                        print(f"plan left arm failed at segment {i}: {res}")
                        break

                if right_pose is not None:
                    right_end_speed_ratio = 0.0 if is_last_task else 0.5
                    res, right_traj = plan_arm(False,
                                               right_pose,
                                               right_start_speed_ratio,
                                               right_end_speed_ratio)
                    if res != StatusCode.SUCCESS:
                        print(f"plan right arm failed at segment {i}: {res}")
                        break

                if left_traj is not None:
                    for qpos in left_traj[1:]:
                        left_streaming_buffer.put(qpos.tolist())
                    start_qpos_[6:13] = left_traj[-1]
                    left_start_speed_ratio = left_end_speed_ratio

                if right_traj is not None:
                    for qpos in right_traj[1:]:
                        right_streaming_buffer.put(qpos.tolist())
                    start_qpos_[13:20] = right_traj[-1]
                    right_start_speed_ratio = right_end_speed_ratio
        finally:
            left_streaming_buffer.put(finish_token)
            right_streaming_buffer.put(finish_token)

    control_worker = threading.Thread(target=control_thread)
    planning_worker = threading.Thread(target=planning_thread)

    control_worker.start()
    planning_worker.start()
    planning_worker.join()
    control_worker.join()

if __name__ == "__main__":
    main(is_sim=True)
