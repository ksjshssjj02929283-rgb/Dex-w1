# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

from rclpy.node import Node
from std_msgs.msg import Float64, Int32

from dexe_robot_client.common import get_logger


class PGCGrippersSimServer(Node):
    def __init__(self):
        super().__init__('pgc_grippers_sim_server')
        self.logger = get_logger()

        self.create_subscription(Float64, 
                                 '/control_sim/gripper/left', 
                                 self._left_qpos_callback, 10)
        self.create_subscription(Float64, 
                                 '/control_sim/gripper/right', 
                                 self._right_qpos_callback, 10)
        
        self.left_qpos_pub = self.create_publisher(Float64, 
                                                   '/feedback_sim/gripper/left', 
                                                   10)
        self.right_qpos_pub = self.create_publisher(Float64, 
                                                    '/feedback_sim/gripper/right', 
                                                    10)

        self._cur_left_qpos = 0.
        self._cur_right_qpos = 0.

        self.create_timer(0.1, self._publish_feedback) # 定时器(10Hz)
        self.logger.info(f"PGC grippers sim server is running")

    def _left_qpos_callback(self, msg):
        self._cur_left_qpos = msg.data
        self.logger.info(f"Left gripper qpos is set: {self._cur_left_qpos}")

    def _right_qpos_callback(self, msg):
        self._cur_right_qpos = msg.data
        self.logger.info(f"Right gripper qpos is set: {self._cur_right_qpos}")

    def _publish_feedback(self):
        self.left_qpos_pub.publish(Float64(data=float(self._cur_left_qpos)))
        # self.logger.debug(f"Left gripper qpos: {self._cur_left_qpos}")

        self.right_qpos_pub.publish(Float64(data=float(self._cur_right_qpos)))
        # self.logger.debug(f"Right gripper qpos: {self._cur_right_qpos}")