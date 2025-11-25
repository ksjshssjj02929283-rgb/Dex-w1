# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import numpy as np
from IPython import embed
import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor

from dexe_robot_client.end_effector_client import create_end_effector_client, EndEffectorType
from dexe_robot_client.common import get_logger


def main(is_sim=True, end_effector_type=EndEffectorType.BRAINCO_HAND):
    rclpy.init()

    left_node = create_end_effector_client(is_left=True,
                                           is_sim=is_sim,
                                           end_effector_type=end_effector_type)
    right_node = create_end_effector_client(is_left=False,
                                            is_sim=is_sim,
                                            end_effector_type=end_effector_type)

    executor = MultiThreadedExecutor()
    executor.add_node(left_node)
    executor.add_node(right_node)

    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    try:
        get_logger().info(f"left qpos: {left_node.get_pos()[1]}")
        get_logger().info(f"right qpos: {right_node.get_pos()[1]}")
        embed(banner1="Interactive debugging here")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        ros_thread.join()
        executor.shutdown()
        left_node.destroy_node()
        right_node.destroy_node()


if __name__ == "__main__":
    main(is_sim=True, end_effector_type=EndEffectorType.BRAINCO_HAND)
    # main(is_sim = True,
    #      end_effector_type = EndEffectorType.PGC_GRIPPER)
