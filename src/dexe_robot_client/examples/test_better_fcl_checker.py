from dexe_robot_client.w1 import RobotType, Planner_Pin
from dexe_robot_client.w1.path_planner.better_fcl_checker import FCLChecker
from dexe_robot_client.end_effector_client import EndEffectorType
import numpy as np
import time
from IPython import embed

if __name__ == "__main__":
    planner_pin = Planner_Pin(robot_type=RobotType.W1V02_1,
                              end_effector_type=EndEffectorType.BRAINCO_HAND)

    fcl_checker = FCLChecker.init_from_W1Manager(
        robot_type=RobotType.W1V02_1,
        end_effector_type=EndEffectorType.BRAINCO_HAND)

    # fcl_checker.add_obstacle(
    #     "box", "table", np.array([0.8, 0, 0.8]), np.array([1, 0, 0, 0]), size=np.array([1, 2, 0.06])
    # )
    # fcl_checker.add_obstacle(
    #     "sphere", "ball", np.array([0.0, 0.85, 1.4]), np.array([1, 0, 0, 0]), size=np.array([0.05])
    # )
    fcl_checker.load_env_json(
        json_path="/home/young/dexe_robot_client_data/environment/template.json"
    )

    real_qpos = np.zeros(20)
    armlee_qpos, armree_qpos = np.zeros(6), np.zeros(6)
    torso_tf, head_tf, arml_tf, armr_tf, armlee_tf, armree_tf = planner_pin.get_Kinematics(
        real_qpos=real_qpos,
        left_ee_qpos=armlee_qpos,
        right_ee_qpos=armree_qpos,
        compute_ee=True)

    fcl_checker.update_non_arm_pose(non_arm_transforms=torso_tf + head_tf)
    fcl_checker.update_arm_pose(arml_tf, armr_tf, armlee_tf, armree_tf)

    fcl_checker.visualize_scene()
    print(fcl_checker.check_collision_Naive())

    fcl_checker.disable_obstacle("ball1")

    ball_tf = np.array([[1, 0, 0, 0.0], [0, 1, 0, -0.85], [0, 0, 1, 1.4],
                        [0, 0, 0, 1]])
    flange_T_object = np.linalg.inv(armr_tf[-1]) @ ball_tf
    fcl_checker.attach_object("right_arm", "ball2", flange_T_object)

    real_qpos = np.zeros(20)
    real_qpos[6:8] = np.array([-1.58, -1.58])
    real_qpos[13:15] = np.array([1.58, 1.58])
    armlee_qpos, armree_qpos = np.zeros(6), np.zeros(6)
    torso_tf, head_tf, arml_tf, armr_tf, armlee_tf, armree_tf = planner_pin.get_Kinematics(
        real_qpos=real_qpos,
        left_ee_qpos=armlee_qpos,
        right_ee_qpos=armree_qpos,
        compute_ee=True)

    fcl_checker.update_non_arm_pose(non_arm_transforms=torso_tf + head_tf)
    fcl_checker.update_arm_pose(arml_tf, armr_tf, armlee_tf, armree_tf)

    fcl_checker.visualize_scene()
    print(fcl_checker.check_collision_Naive())

    fcl_checker.enable_obstacle("ball1")

    flange_transform = armr_tf[-1]
    fcl_checker.detach_object("right_arm", flange_transform)

    fcl_checker.load_env_json(
        json_path="/home/young/dexe_robot_client_data/environment/template2.json"
    )

    real_qpos = np.zeros(20)
    armlee_qpos, armree_qpos = np.zeros(6), np.zeros(6)
    torso_tf, head_tf, arml_tf, armr_tf, armlee_tf, armree_tf = planner_pin.get_Kinematics(
        real_qpos=real_qpos,
        left_ee_qpos=armlee_qpos,
        right_ee_qpos=armree_qpos,
        compute_ee=True)

    fcl_checker.update_non_arm_pose(non_arm_transforms=torso_tf + head_tf)
    fcl_checker.update_arm_pose(arml_tf, armr_tf, armlee_tf, armree_tf)

    fcl_checker.visualize_scene()
    print(fcl_checker.check_collision_Naive())

    embed()
