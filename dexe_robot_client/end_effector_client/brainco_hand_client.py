# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import numpy as np
from copy import deepcopy
from std_msgs.msg import Float64MultiArray
from .end_effector_client import EndEffectorClient


class BrainCoHandClient(EndEffectorClient):
    def __init__(self, is_left: bool):
        if is_left:
            name = "left"
        else:
            name = "right"
        self.node_name = f"brainco_{name}_hand_client"
        super().__init__(self.node_name, qpos_bound=np.array([[0.]*6, [1000.]*6]))
        
        # Create subscriber and publisher
        self._qpos_subscription = self.create_subscription(
            Float64MultiArray, f"/brainco_{name}_hand_qpos", 
            self._qpos_callback, 10
        )
        self._set_qpos_publisher = self.create_publisher(
            Float64MultiArray, f"/set_brainco_{name}_hand_qpos", 10
        )

        self._current_qpos = None

    def _qpos_callback(self, msg):
        self._current_qpos = np.array(msg.data)
        self.get_logger().debug(f"{self.node_name}: received {self._current_qpos}")

    def set_qpos(self, qpos: np.ndarray) -> bool:
        if not self._check_input(qpos):
            return False
        qpos_ = np.clip(qpos, self.qpos_lb, self.qpos_ub)
        
        msg = Float64MultiArray()
        msg.data = qpos_.tolist()
        self._set_qpos_publisher.publish(msg)
        self.get_logger().debug(f"{self.node_name}: published {qpos_}")
        return True

    def get_qpos(self) -> tuple[bool, np.ndarray]:
        self.spin_once()
        if self._current_qpos is None:
            self.get_logger().error(f"{self.node_name}: get qpos failed")
            return False, None
        temp = deepcopy(self._current_qpos)
        self._current_qpos = None
        return True, temp


class LeftBrainCoHadnClient(BrainCoHandClient):
    def __init__(self):
        super().__init__(is_left=True)

class RightBrainCoHadnClient(BrainCoHandClient):
    def __init__(self):
        super().__init__(is_left=False)



if __name__ == "__main__":
    import rclpy
    from IPython import embed

    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()
    left_hand_node = LeftBrainCoHadnClient()
    right_hand_node = RightBrainCoHadnClient()

    embed()

    left_hand_node.destroy_node()
    right_hand_node.destroy_node()
    rclpy.shutdown()