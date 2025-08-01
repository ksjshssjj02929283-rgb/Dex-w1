# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

from dexe_robot_client.end_effector_client import LeftBrainCoHadnClient, RightBrainCoHadnClient

import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor
from IPython import embed

def main():
    rclpy.init()

    executor = MultiThreadedExecutor()
    left_hand_node = LeftBrainCoHadnClient()
    right_hand_node = RightBrainCoHadnClient()

    executor.add_node(left_hand_node)
    executor.add_node(right_hand_node)

    # 在后台启动 executor
    t = threading.Thread(target=executor.spin, daemon=True)
    t.start()

    try:
        embed()  # 在这里交互式调试
    finally:
        # 安全退出：先关闭 executor，再释放资源
        executor.shutdown()
        left_hand_node.destroy_node()
        right_hand_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()