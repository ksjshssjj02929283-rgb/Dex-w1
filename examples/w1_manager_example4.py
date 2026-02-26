# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import numpy as np
from copy import deepcopy
import threading
from dexe_robot_client.end_effector_client import EndEffectorType
from dexe_robot_client.common import W1Version, W1ArmType, time_consuming
from dexe_robot_client.w1 import W1Manager


def main(use_parallel=False):
    """利用plan和execute异步示例"""
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
    w1_manager.set_ee_qpos("left_arm_ee", np.array([0] * 6))
    w1_manager.set_ee_qpos("right_arm_ee", np.array([0] * 6))

    cur_l_pose = w1_manager.get_current_pose("left_arm")[1]
    l_pose1 = deepcopy(cur_l_pose)
    l_pose1[0] += -150
    l_pose1[2] += -100
    l_pose2 = deepcopy(l_pose1)
    l_pose2[2] += 100
    left_arm_waypoints = [l_pose1, l_pose2]
    left_arm_ee_list = [[0., [0., 50., 50., 50., 50., 50.], 0.5],
                        [0., [100., 100., 0., 0., 100., 100.], 0.5]]

    @time_consuming
    def plan_then_move():
        for i in range(3):
            res = w1_manager.move_linear_full_body(waypoints_dict={
                "left_arm": left_arm_waypoints,
                "left_arm_ee": left_arm_ee_list
            },
                                                   cartesian_step=0.0005)

    @time_consuming
    def plan_and_move():
        plan_res = {}
        plan_finished_event = threading.Event()

        def plan_step(start_qpos, waypoints_dict, start_l_ee_qpos,
                      start_r_ee_qpos, plan_res):
            res = w1_manager.plan_linear_full_body(start_qpos,
                                                   start_l_ee_qpos,
                                                   start_r_ee_qpos,
                                                   waypoints_dict,
                                                   cartesian_step=0.0005)
            plan_res["res"] = res
            plan_finished_event.set()

        start_qpos = w1_manager.get_current_qpos()[1]
        start_l_ee_qpos = w1_manager.get_current_qpos("left_arm_ee")[1]
        start_r_ee_qpos = w1_manager.get_current_qpos("right_arm_ee")[1]

        # 规划
        threading.Thread(target=plan_step,
                         args=(start_qpos, {
                             "left_arm": left_arm_waypoints,
                             "left_arm_ee": left_arm_ee_list
                         }, start_l_ee_qpos, start_r_ee_qpos, plan_res),
                         daemon=True).start()

        for i in range(2):
            # 等待规划完成
            plan_finished_event.wait()
            plan_finished_event.clear()
            res, total_qpos_array, left_arm_ee_trigger_list, right_arm_ee_trigger_list = plan_res[
                "res"]
            if not res:
                return
            start_qpos = total_qpos_array[-1]
            start_l_ee_qpos = np.array(left_arm_ee_trigger_list[-1][1])
            if right_arm_ee_trigger_list is not None:
                start_r_ee_qpos = np.array(right_arm_ee_trigger_list[-1][1])
            # 下一步 规划
            threading.Thread(target=plan_step,
                             args=(start_qpos, {
                                 "left_arm": left_arm_waypoints,
                                 "left_arm_ee": left_arm_ee_list
                             }, start_l_ee_qpos, start_r_ee_qpos, plan_res),
                             daemon=True).start()

            # 当前步 执行
            w1_manager.execute_joint_traj(total_qpos_array,
                                          left_arm_ee_trigger_list,
                                          right_arm_ee_trigger_list)

        # 最后一步 执行
        plan_finished_event.wait()
        res, total_qpos_array, _, _ = plan_res["res"]
        if res:
            w1_manager.execute_joint_traj(total_qpos_array,
                                          left_arm_ee_trigger_list,
                                          right_arm_ee_trigger_list)

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

    import time

    try:
        while True:
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("Exiting the script. Goodbye!")