'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

from dexe_robot_client.w1 import RobotType, Planner_Pin, FCLChecker
from dexe_robot_client.end_effector_client import EndEffectorType
import numpy as np
import time

if __name__ == "__main__":
    # Pinocchio Instance
    planner_pin = Planner_Pin(robot_type=RobotType.W1V02_1,
                              end_effector_type=EndEffectorType.BRAINCO_HAND)

    # FCL Instance
    fcl_checker = FCLChecker(robot_type=RobotType.W1V02_1,
                             end_effector_type=EndEffectorType.BRAINCO_HAND)
    fcl_checker.enable_visualize()

    # fcl_checker.load_env_json(
    #     "/home/young/dexe_robot_client_data/assets/environment/coffee_demo_upward.json"
    # )

    # Real robot joint order [torso, head, arml, armr]
    real_qpos = np.zeros(20)
    # real_qpos[11] = 0.79
    # real_qpos[12] = 1.05
    real_qpos[6:8] = np.array([-1.41, -1.90])
    real_qpos[13:15] = np.array([1.41, 2.06])
    # real_qpos = np.array([
    #     0.7855,     -1.5707,      0.7855,     -1.3079,   0.0062277,     -0.3491,    -0.38282,    -0.86297,     0.31062,     -1.7837,    0.030647,   -0.068155,     0.46853,      -0.273,       0.799,      -0.197,      2.0096,      0.7831,      0.0674,      0.1484
    # ])
    armlee_qpos, armree_qpos = np.zeros(6), np.zeros(6)
    torso_tf, head_tf, arml_tf, armr_tf, armlee_tf, armree_tf = planner_pin.get_Kinematics(
        real_qpos=real_qpos,
        left_ee_qpos=armlee_qpos,
        right_ee_qpos=armree_qpos,
        compute_ee=True)
    # torso_tf, head_tf, arml_tf, armr_tf = planner_pin.get_Kinematics(
    #     real_qpos=real_qpos,
    #     left_ee_qpos=armlee_qpos,
    #     right_ee_qpos=armree_qpos,
    #     compute_ee=True)
    # armlee_tf = None
    # armree_tf = None
    fcl_checker.set_head_body_transform(head_transforms=head_tf,
                                        body_transforms=torso_tf)
    fcl_checker.update_vis_transform(transforms=torso_tf + head_tf + arml_tf +
                                     armr_tf,
                                     armlee_transforms=armlee_tf,
                                     armree_transforms=armree_tf)

    input("Press Enter to check collision...")
    arm_tf = arml_tf + armr_tf
    fcl_checker.update_arm_transform(arm_tf, armlee_tf, armree_tf)
    start_time = time.time()
    is_collision = fcl_checker.naive_check_collision(
    )  # True if collision occurs
    print(f"collision checking time: {time.time() - start_time:.6f} sec")
    print("Is collision: ", is_collision)
