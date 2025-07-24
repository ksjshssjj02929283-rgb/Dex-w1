# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import numpy as np
from copy import deepcopy
from std_msgs.msg import Float64, Float64MultiArray
from .end_effector_client import EndEffectorClient


class PGCGripperClient(EndEffectorClient):
    def __init__(self, is_left: bool):
        if is_left:
            name = "left"
        else:
            name = "right"
        self.node_name = f"pgc_{name}_gripper_client"
        super().__init__(self.node_name, qpos_bound=np.array([[0.], [100.]]))

        # Create subscriber and publisher
        self._qpos_subscription = self.create_subscription(
            Float64, f"/{name}_pgc_node/current_position", 
            self._qpos_callback, 10
        )
        # f"/{name}_pgc_node/grip_state" Int32

        self._set_qpos_publisher = self.create_publisher(
            Float64MultiArray, f"/set_{name}_pgc_position", 10
        )
        # f"/set_{name}_pgc_force" Float64
        # f"/set_{name}_pgc_speed" Float64

        self._current_qpos = None

    def _qpos_callback(self, msg):
        self._current_qpos = np.array([msg.data])
        self.get_logger().debug(f"{self.node_name}: received {self._current_qpos}")

    def set_qpos(self, qpos: np.ndarray | float) -> bool:
        if isinstance(qpos, (int, float)):
            qpos = np.array([float(qpos)])
        else:
            qpos = qpos
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

    def set_state(self, state: np.ndarray | float) -> bool:
        if isinstance(state, (int, float)):
            state = np.array([float(state)])
        else:
            state = state
        return super().set_state(state)


class LeftPGCGripperClient(PGCGripperClient):
    def __init__(self):
        super().__init__(is_left=True)

class RightPGCGripperClient(PGCGripperClient):
    def __init__(self):
        super().__init__(is_left=False)



if __name__ == "__main__":
    import rclpy
    from IPython import embed

    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()
    left_gripper_node = LeftPGCGripperClient()
    right_gripper_node = RightPGCGripperClient()

    embed()

    left_gripper_node.destroy_node()
    right_gripper_node.destroy_node()
    rclpy.shutdown()