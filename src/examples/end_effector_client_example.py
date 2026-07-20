'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

from IPython import embed
import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor

from dexe_robot_client.end_effector_client import EndEffectorClient
from dexe_rp import EndEffectorType


def main(is_sim: bool = True,
         left_ee_type: EndEffectorType = EndEffectorType.BrainCo_Revo1_R,
         right_ee_type: EndEffectorType = EndEffectorType.BrainCo_Revo1_R):
    rclpy.init()

    executor = MultiThreadedExecutor()

    left_ee_node = None
    if left_ee_type is not None:
        left_ee_node = EndEffectorClient(ee_type=left_ee_type,
                                         is_left=True,
                                         is_sim=is_sim)
        executor.add_node(left_ee_node)

    right_ee_node = None
    if right_ee_type is not None:
        right_ee_node = EndEffectorClient(ee_type=right_ee_type,
                                          is_left=False,
                                          is_sim=is_sim)
        executor.add_node(right_ee_node)

    bg_thread = threading.Thread(target=executor.spin, daemon=True)
    bg_thread.start()
    try:
        if left_ee_node:
            if left_ee_node.connect(timeout_sec=0.5):
                print(f"{left_ee_node.node_name} connected.")
                print(f"left ee info: {left_ee_node.get_info()[1]}")
            else:
                raise RuntimeError

        if right_ee_node:
            if right_ee_node.connect(timeout_sec=0.5):
                print(f"{right_ee_node.node_name} connected.")
                print(f"right ee info: {right_ee_node.get_info()[1]}")
            else:
                raise RuntimeError
        embed(banner1="Interactive debugging here")
    finally:
        executor.shutdown()

        bg_thread.join()

        if left_ee_node is not None:
            executor.remove_node(left_ee_node)
        if right_ee_node is not None:
            executor.remove_node(right_ee_node)

        if left_ee_node is not None:
            left_ee_node.destroy_node()
        if right_ee_node is not None:
            right_ee_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main(is_sim=True,
         left_ee_type=EndEffectorType.BrainCo_Revo1_R,
         right_ee_type=EndEffectorType.BrainCo_Revo1_R)
    # main(is_sim=True, left_ee_type=EndEffectorType.Linker_L6, right_ee_type=EndEffectorType.Linker_L6)
    # main(is_sim=True, left_ee_type=EndEffectorType.DH_PGC_140_50, right_ee_type=EndEffectorType.DH_PGC_140_50)
