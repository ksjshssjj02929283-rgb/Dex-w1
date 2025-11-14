# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import time
import threading
import numpy as np
from copy import deepcopy
from std_msgs.msg import Float64, Int32
from .end_effector_client import EndEffectorClient


class PGCGripperClient(EndEffectorClient):
    def __init__(self, 
                 is_left: bool,
                 is_sim: bool = False,
                 timeout_sec: float = 3.0):
        if is_left:
            self._name = "left"
        else:
            self._name = "right"
        node_name = f"pgc_{self._name}_gripper_client"

        # self._grip_state_lock = threading.Lock()
        # self._cur_grip_state = None
        # self._last_grip_state_time = None

        super().__init__(node_name, 
                         1,
                         is_sim, 
                         timeout_sec)

    def _init_subscriber(self, is_sim: bool):
        if is_sim:
            self.create_subscription(
                Float64, f"/feedback_sim/gripper/{self._name}", 
                self._feedback_callback, 10
            )
            # self.create_subscription(
            #     Int32, f"/feedback_sim/gripper/{self._name}/grip_state",
            #     self._grip_state_callback, 10
            # )
        else:
            self.create_subscription(
                Float64, f"/feedback/gripper/{self._name}", 
                self._feedback_callback, 10
            )
            # self.create_subscription(
            #     Int32, f"/feedback/gripper/{self._name}/grip_state",
            #     self._grip_state_callback, 10
            # )

    def _init_publisher(self, is_sim: bool):
        if is_sim:
            self._set_qpos_publisher = self.create_publisher(
                Float64, f"/control_sim/gripper/{self._name}", 10
            )
        else:
            self._set_qpos_publisher = self.create_publisher(
                Float64, f"/control/gripper/{self._name}", 10
            )
        # f"/control/gripper/left/speed" Float64
        # f"/control/gripper/left/force" Float64

    # def _grip_state_callback(self, msg):
    #     with self._grip_state_lock:
    #         self._cur_grip_state = msg.data
    #         self._last_grip_state_time = time.time()
    #     # self.logger.debug(f"{self.node_name}: grip state received {self._cur_qpos}")

    # def get_grip_state(self) -> tuple[bool, int]:
    #     """
    #     0 -> Moving
    #     1 -> Reached, no grip object
    #     2 -> Stop moving, detect grip object
    #     3 -> Detect grip object but drops
    #     """
    #     # self.spin_once() # FIXME: Only successful if run twice
    #     with self._grip_state_lock:
    #         if self._cur_grip_state is None or \
    #             self._last_grip_state_time is None:
    #             return False, None
    #         if time.time() - self._last_grip_state_time > 0.1:
    #             return False, None
    #         temp = deepcopy(self._cur_grip_state)
    #         self._cur_grip_state = None
    #         self._qpos_time = None
    #     return True, temp

    def set_pos(self, qpos: np.ndarray | float) -> bool:
        if isinstance(qpos, (int, float)):
            qpos_ = np.array([float(qpos)])
        else:
            qpos_ = np.array(qpos)

        if not self._check_dim(qpos_):
            return False
        qpos_clip = np.clip(qpos_, self.pos_lb, self.pos_ub)
        
        msg = Float64()
        msg.data = qpos_clip[0]
        self._set_qpos_publisher.publish(msg)
        self.logger.debug(f"{self.node_name} published:  {qpos_clip}")
        return True