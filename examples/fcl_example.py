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

    fcl_checker.load_env_json(
        "/home/young/dexe_robot_client_data/assets/environment/coffee_demo_upward.json"
    )

    # Real robot joint order [torso, head, arml, armr]
    real_qpos = np.zeros(20)
    armlee_qpos, armree_qpos = np.zeros(6), np.zeros(6)
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
