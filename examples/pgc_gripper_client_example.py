# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

from IPython import embed
import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor

from dexe_robot_client.end_effector_client import LeftPGCGripperClient, RightPGCGripperClient


def main(is_sim = True):
    rclpy.init()

    left_gripper_node = LeftPGCGripperClient(is_sim)
    right_gripper_node = RightPGCGripperClient(is_sim)

    executor = MultiThreadedExecutor()
    executor.add_node(left_gripper_node)
    executor.add_node(right_gripper_node)

    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    try:
        embed()
    finally:
        executor.shutdown()
        left_gripper_node.destroy_node()
        right_gripper_node.destroy_node()
        ros_thread.join()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main(is_sim = True)