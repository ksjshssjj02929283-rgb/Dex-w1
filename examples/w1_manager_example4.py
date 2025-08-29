# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import numpy as np
from copy import deepcopy
import threading
from dexe_robot_client.common import W1Version, W1ArmType, time_consuming 
from dexe_robot_client.w1 import W1Manager


def main(use_parallel = False):
    """利用plan和execute异步示例"""
    w1_manager = W1Manager(is_sim=True,
                        arm_type=W1ArmType.ANTHROPOMORPHIC, 
                        w1_version=W1Version.V2, 
                        end_effector_type=None,
                        use_rlia=False)
    if not w1_manager.connect():
        return
    w1_manager.set_home_qpos(np.array([0.7854, -1.5708, 0.7854, 0.0, 0.0, 0.0,  
                                       0.0068, -0.9103, 0.0235, -1.9158, 0.1378, 0.1401, 0.0072, 
                                       -0.0068, 0.9103,  0.0235,  1.9158, 0.1378, 0.1401, 0.0072]),
                            save_to_file=False)
    w1_manager.go_home(1.)

    cur_l_pose = w1_manager.get_current_pose("left_arm")[1]
    l_pose1 = deepcopy(cur_l_pose)
    l_pose1[0] += -200
    l_pose1[2] += -150
    l_pose2 = deepcopy(l_pose1)
    l_pose2[2] += 150
    l_pose3 = deepcopy(l_pose2)
    l_pose3[0] += 200
    l_pose3[2] += -150
    l_pose4 = deepcopy(l_pose3)
    l_pose4[2] += 150
    left_arm_waypoints = [l_pose1, l_pose2, l_pose3, l_pose4]

    @time_consuming
    def plan_then_move():
        for waypoint in left_arm_waypoints:
            res = w1_manager.move_linear_full_body(waypoints_dict = {"left_arm": [waypoint]}, 
                                                   cartesian_step = 0.0005)
            if not res:
                return

    @time_consuming
    def plan_and_move():
        plan_res = {}
        plan_finished_event = threading.Event()
        
        def plan_step(start_qpos, waypoints_dict, plan_res):
            res = w1_manager.plan_linear_full_body(start_qpos, 
                                                    None, 
                                                    None, 
                                                    waypoints_dict, 
                                                    cartesian_step=0.0005)
            plan_res["res"] = res
            plan_finished_event.set()

        res, cur_qpos = w1_manager.get_current_qpos()
        if not res:
            return
        start_qpos = cur_qpos
        # 规划
        threading.Thread(target=plan_step, 
                            args=(start_qpos, {"left_arm": [left_arm_waypoints[0]]}, plan_res)).start()

        for waypoint in left_arm_waypoints[1:]:
            # 等待规划完成
            plan_finished_event.wait()
            plan_finished_event.clear()
            res, total_qpos_array, _, _ = plan_res["res"]
            if not res:
                return
            start_qpos = total_qpos_array[-1]

            # 下一步 规划
            threading.Thread(target=plan_step, 
                             args=(start_qpos, {"left_arm": [waypoint]}, plan_res)).start()

            # 当前步 执行
            w1_manager.execute_joint_traj(total_qpos_array)

        # 最后一步 执行
        plan_finished_event.wait()
        res, total_qpos_array, _, _ = plan_res["res"]
        if res:
            w1_manager.execute_joint_traj(total_qpos_array)

    if use_parallel:
        plan_and_move()
    else:
        plan_then_move()


if __name__ == "__main__":
    """由于python GIL锁
        execute_joint_traj()里只有time.sleep的时候CPU才会空闲
        让出给子线程进行并行计算
        然而execute_joint_traj()在频发收发streaming指令，python中并行提升不大
        
        当execute_joint_traj()不采用streaming模式，而是move_traj模式，python中并行提升大
        """
    main(use_parallel=False)