# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import numpy as np
from end_effector_interfaces.msg import EEJointControl, EEJointFeedback
from .end_effector_client import EndEffectorClient


class PGCGripperClient(EndEffectorClient):

    def __init__(self,
                 is_left: bool,
                 is_sim: bool = False,
                 timeout_sec: float = 1.0):
        if is_left:
            self._name = "left"
        else:
            self._name = "right"
        node_name = f"pgc_{self._name}_gripper_client"
        super().__init__(node_name, 1, ["FINGER"], is_sim, timeout_sec)

    def _init_subscriber(self, is_sim: bool):
        if is_sim:
            self.create_subscription(EEJointFeedback,
                                     f"/feedback_sim/gripper/{self._name}",
                                     self._feedback_callback, 10)
        else:
            self.create_subscription(EEJointFeedback,
                                     f"/feedback/gripper/{self._name}",
                                     self._feedback_callback, 10)

    def _init_publisher(self, is_sim: bool):
        if is_sim:
            self._control_publisher = self.create_publisher(
                EEJointControl, f"/control_sim/gripper/{self._name}", 10)
        else:
            self._control_publisher = self.create_publisher(
                EEJointControl, f"/control/gripper/{self._name}", 10)

    def set_pos(self, pos) -> bool:
        if isinstance(pos, (int, float)):
            pos_ = np.array([float(pos)])
        else:
            pos_ = np.array(pos)
        return super().set_pos(pos_)
