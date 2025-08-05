# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import time
import threading
import numpy as np
from copy import deepcopy
from std_msgs.msg import Float64, Int32, Float64MultiArray
from .end_effector_client import EndEffectorClient
from dexe_robot_client.common import get_logger


class PGCGripperClient(EndEffectorClient):
    def __init__(self, is_left: bool):
        self.logger = get_logger()
        if is_left:
            name = "left"
        else:
            name = "right"
        node_name = f"pgc_{name}_gripper_client"
        super().__init__(node_name, qpos_bound=np.array([[0.], [1000.]]))

        # Create subscriber and publisher
        self.create_subscription(
            Float64, f"/{name}_pgc_node/current_position", 
            self._qpos_callback, 10
        )
        self.create_subscription(
            Int32, f"/{name}_pgc_node/grip_state",
            self._grip_state_callback, 10
        )

        self._set_qpos_publisher = self.create_publisher(
            Float64MultiArray, f"/set_{name}_pgc_position", 10
        )
        # f"/set_{name}_pgc_force" Float64
        # f"/set_{name}_pgc_speed" Float64

        self._grip_state_lock = threading.Lock()
        self._cur_grip_state = None
        self._last_grip_state_time = None

    def _grip_state_callback(self, msg):
        with self._grip_state_lock:
            self._cur_grip_state = msg.data
            self._last_grip_state_time = time.time()
        # self.logger.debug(f"{self.node_name}: grip state received {self._cur_qpos}")

    def get_grip_state(self) -> tuple[bool, int]:
        """
        0 -> Moving
        1 -> Reached, no grip object
        2 -> Stop moving, detect grip object
        3 -> Detect grip object but drops
        """
        # self.spin_once() # FIXME: Only successful if run twice
        with self._grip_state_lock:
            if self._cur_grip_state is None or \
                self._last_grip_state_time is None:
                return False, None
            if time.time() - self._last_grip_state_time > 0.1:
                return False, None
            temp = deepcopy(self._cur_grip_state)
            self._cur_grip_state = None
            self._qpos_time = None
        return True, temp

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
        self.logger.debug(f"{self.node_name}: qpos published {qpos_}")
        return True

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
