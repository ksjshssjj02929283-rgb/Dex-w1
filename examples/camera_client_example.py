# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

from dexe_robot_client.camera_client import OrbbecClient
import numpy as np
import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor
import time

if __name__ == "__main__":
    rclpy.init()

    orbbec_node = OrbbecClient()

    executor = MultiThreadedExecutor()
    executor.add_node(orbbec_node)

    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    if not orbbec_node.is_connected():
        time.sleep(1)

    for i in range(50):
        res_pcd, point_cloud = orbbec_node.get_pcd_numpy()
        print(point_cloud.shape)
        time.sleep(0.1)

    if rclpy.ok():
        rclpy.shutdown()
    ros_thread.join()
    executor.shutdown()
    orbbec_node.destroy_node()

    