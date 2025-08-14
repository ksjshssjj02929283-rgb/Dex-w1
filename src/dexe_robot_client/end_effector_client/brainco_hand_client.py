# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------


from std_msgs.msg import Float64MultiArray
from .end_effector_client import EndEffectorClient
from ..common import get_logger


class BrainCoHandClient(EndEffectorClient):
    def __init__(self, 
                 is_left: bool, 
                 is_sim: bool = False,
                 timeout_sec: float = 3.0):
        self.logger = get_logger()
        if is_left:
            self._name = "left"
        else:
            self._name = "right"
        node_name = f"brainco_{self._name}_hand_client"
        super().__init__(node_name, 
                         6,
                         is_sim, 
                         timeout_sec)
        
    def _init_subscriber(self, is_sim: bool):
        if is_sim:
            self.create_subscription(
                Float64MultiArray, f"/feedback_sim/hand/{self._name}", 
                self._qpos_callback, 10
            )
        else:
            self.create_subscription(
                Float64MultiArray, f"/feedback/hand/{self._name}", 
                self._qpos_callback, 10
            )

    def _init_publisher(self, is_sim: bool):
        if is_sim:
            self._set_qpos_publisher = self.create_publisher(
                Float64MultiArray, f"/control_sim/hand/{self._name}", 10
            )
        else:
            self._set_qpos_publisher = self.create_publisher(
                Float64MultiArray, f"/control/hand/{self._name}", 10
            )


class LeftBrainCoHandClient(BrainCoHandClient):
    def __init__(self, is_sim: bool = False):
        super().__init__(True, is_sim)


class RightBrainCoHandClient(BrainCoHandClient):
    def __init__(self, is_sim: bool = False):
        super().__init__(False, is_sim)
