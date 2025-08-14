# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from ..common import get_logger


class BraincoHandsSimServer(Node):
    def __init__(self):
        super().__init__('brainco_hands_sim_server')
        self.logger = get_logger()

        self.create_subscription(Float64MultiArray, 
                                 '/control_sim/hand/left', 
                                 self._left_qpos_callback, 10)
        self.create_subscription(Float64MultiArray, 
                                 '/control_sim/hand/right', 
                                 self._right_qpos_callback, 10)
        
        self.left_qpos_pub = self.create_publisher(Float64MultiArray, 
                                                   '/feedback_sim/hand/left', 
                                                   10)
        self.right_qpos_pub = self.create_publisher(Float64MultiArray, 
                                                    '/feedback_sim/hand/right', 
                                                    10)
        
        self._cur_left_qpos = np.array([0.]*6)
        self._cur_right_qpos = np.array([0.]*6)

        self.create_timer(0.1, self._publish_states) # 定时器(10Hz)
        self.logger.info(f"Brainco hands sim server is running")
    
    def _left_qpos_callback(self, msg):
        if len(msg.data) == 6:
            left_data = [int(x) for x in msg.data]
            self._cur_left_qpos = np.array(left_data)
            self.logger.info(f"Left hand qpos is set: {left_data}")
    
    def _right_qpos_callback(self, msg):
        if len(msg.data) == 6:
            right_data = [int(x) for x in msg.data]
            self._cur_right_qpos = np.array(right_data)
            self.logger.info(f"Right hand qpos is set: {right_data}")

    def _publish_states(self):
        left_pos = Float64MultiArray()
        left_pos.data = [float(x) for x in self._cur_left_qpos]
        self.left_qpos_pub.publish(left_pos)
        # self.logger.debug(f"Left hand qpos: {self._cur_left_qpos}")

        right_pos = Float64MultiArray()
        right_pos.data = [float(x) for x in self._cur_right_qpos]
        self.right_qpos_pub.publish(right_pos)
        # self.logger.debug(f"Right hand qpos: {self._cur_right_qpos}")