from dexe_robot_client.w1 import RobotType, Planner_Pin, FCLChecker
from dexe_robot_client.end_effector_client import EndEffectorType
import numpy as np

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
    # real_qpos = np.zeros(20)

    real_qpos = np.array([
        0.54105, -1.2217, 0.7854, 0.034907, 0, -0.66323, 0.86894, -0.65736,
        -0.81757, -2.0253, 0.44582, 0.070689, -0.042897, -0.099449, 0.86331,
        -0.2637, 1.6072, 0.65926, 0.38971, -0.27398
    ])

    # armlee_qpos, armree_qpos = np.zeros(6), np.zeros(6)
    armlee_qpos = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    armree_qpos = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    torso_tf, head_tf, arml_tf, armr_tf, armlee_tf, armree_tf = planner_pin.get_Kinematics(
        real_qpos=real_qpos,
        left_ee_qpos=armlee_qpos,
        right_ee_qpos=armree_qpos,
        compute_ee=True)
    fcl_checker.set_head_body_transform(head_transforms=head_tf,
                                        body_transforms=torso_tf)
    fcl_checker.update_vis_transform(transforms=torso_tf + head_tf + arml_tf +
                                     armr_tf,
                                     armlee_transforms=armlee_tf,
                                     armree_transforms=armree_tf)

    input("Press Enter to check collision...")
    arm_tf = arml_tf + armr_tf
    fcl_checker.update_arm_transform(arm_tf, armlee_tf, armree_tf)

    is_collision = fcl_checker.check_collision()  # True if collision occurs

    print("Is collision: ", is_collision)
