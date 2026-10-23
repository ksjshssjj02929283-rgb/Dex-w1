from dexe_robot_client.w1.collision.collision_checker_wrapper import CollisionCheckerWrapper
from dexe_rp import RobotVersion, EndEffectorType
import numpy as np

if __name__ == "__main__":
    robot_version = RobotVersion.W1V024
    left_ee_type = EndEffectorType.BrainCo_Revo1_R
    right_ee_type = EndEffectorType.BrainCo_Revo1_R
    collision_checker = CollisionCheckerWrapper(robot_version,
                                                left_ee_type,
                                                right_ee_type,
                                                verbose=True)
    robot_real_q = np.array([
        0.785, -1.57, 0.9, 0.0, 0.0, -0.4, -1.55, -0.87, 1.57, -0.785, -1.51,
        0.0, -0.86, 1.57, 1.75, -1.37, 0.57, 0.0, 0.0, 0.0
    ])
    left_ee_pulse = np.array([0.0] * 6)
    right_ee_pulse = np.array([100.0] * 6)

    res = collision_checker.check_collision(robot_real_q,
                                            left_ee_pulse,
                                            right_ee_pulse,
                                            save_scene=False)
    collision_checker.coal_collision.export_scene_to_ply(
        f"/home/young/temp_ply/{robot_version.robot_family}_{robot_version.version_value}_{left_ee_type}_{right_ee_type}.ply"
    )

    input("Press Enter to continue")

    collision_checker.set_verbose(False)
    res = collision_checker.check_collision(robot_real_q,
                                            left_ee_pulse,
                                            right_ee_pulse,
                                            save_scene=False)

    print(res)
