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

from dexe_robot_client.end_effector_client import EndEffectorType, create_end_effector_client


def main(is_sim: bool = True,
         left_ee_type: EndEffectorType = EndEffectorType.BRAINCO_HAND,
         right_ee_type: EndEffectorType = EndEffectorType.BRAINCO_HAND):
    rclpy.init()

    executor = MultiThreadedExecutor()

    left_ee_node = None
    if left_ee_type is not None:
        left_ee_node = create_end_effector_client(
            end_effector_type=left_ee_type, is_left=True, is_sim=is_sim)
        executor.add_node(left_ee_node)

    right_ee_node = None
    if right_ee_type is not None:
        right_ee_node = create_end_effector_client(
            end_effector_type=right_ee_type, is_left=False, is_sim=is_sim)
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
         left_ee_type=EndEffectorType.HITBOT_SUCTION,
         right_ee_type=EndEffectorType.HITBOT_SUCTION)
    # main(is_sim=True, left_ee_type=EndEffectorType.LINKER_HAND, right_ee_type=EndEffectorType.LINKER_HAND)
    # main(is_sim=True, left_ee_type=EndEffectorType.PGC_GRIPPER, right_ee_type=EndEffectorType.PGC_GRIPPER)
