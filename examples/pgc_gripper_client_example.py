# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import time
import numpy as np
from IPython import embed
import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor

from dexe_robot_client.end_effector_client import LeftPGCGripperClient, RightPGCGripperClient


def main():
    rclpy.init()

    left_gripper_node = LeftPGCGripperClient()
    right_gripper_node = RightPGCGripperClient()

    executor = MultiThreadedExecutor()
    executor.add_node(left_gripper_node)
    executor.add_node(right_gripper_node)

    # 在后台启动 executor
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    try:
        embed()  # 在这里交互式调试
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        ros_thread.join()
        executor.shutdown()
        left_gripper_node.destroy_node()
        right_gripper_node.destroy_node()


if __name__ == "__main__":
    main()