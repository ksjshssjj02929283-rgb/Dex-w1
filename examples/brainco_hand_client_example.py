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

from dexe_robot_client.end_effector_client import LeftBrainCoHadnClient, RightBrainCoHadnClient


def main():
    rclpy.init()

    left_hand_node = LeftBrainCoHadnClient()
    right_hand_node = RightBrainCoHadnClient()

    executor = MultiThreadedExecutor()
    executor.add_node(left_hand_node)
    executor.add_node(right_hand_node)

    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    try:
        left_hand_node.set_state(np.zeros(6))
        time.sleep(1)
        for i in range(6):
            state = np.zeros(6)
            state[i] = 0.3
            left_hand_node.set_state(state)
            time.sleep(0.05)
        left_hand_node.set_state(np.zeros(6))
        embed()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        ros_thread.join()
        executor.shutdown()
        left_hand_node.destroy_node()
        right_hand_node.destroy_node()


if __name__ == "__main__":
    main()