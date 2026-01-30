import rclpy
from dexe_robot_client.w1.w1_manager import W1Manager
from dexe_robot_client.common import W1ArmType, W1Version
from dexe_robot_client.end_effector_client import EndEffectorType

if __name__ == "__main__":
    rclpy.init()
    w1_manager = W1Manager(is_sim=True,
                           arm_type=W1ArmType.ANTHROPOMORPHIC,
                           w1_version=W1Version.V2,
                           end_effector_type=EndEffectorType.BRAINCO_HAND,
                           use_nav=False)

    rclpy.shutdown()
    w1_manager._destroy_clients()
