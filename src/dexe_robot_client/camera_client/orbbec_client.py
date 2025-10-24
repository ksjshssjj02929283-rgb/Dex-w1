# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

from sensor_msgs.msg import PointCloud2
from .camera_client import CameraClient
from ..common import get_logger
import rclpy

class OrbbecClient(CameraClient):
    def __init__(self,
                 timeout_sec: float = 3.0):
        self.logger = get_logger()
        node_name = "orbbec_camera_client"
        super().__init__(node_name,
                         timeout_sec = timeout_sec)
        
    def _init_subscriber(self):
        # Orbbec Camera Dabai DCW2 subscriber

        # self.create_subscription(
        #     PointCloud2, 
        #     "/camera/depth/points",
        #     self._point_cloud_callback,
        #     rclpy.qos.QoSProfile(reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT, depth=1, durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)
        # )

        # Realsense D455 subscriber
        self.create_subscription(
            PointCloud2,
            "/camera/camera/depth/color/points",
            self._point_cloud_callback,
            rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)
        )

    def _init_publisher(self):
        self.logger.info("Currently no publisher for the orbbec camera")