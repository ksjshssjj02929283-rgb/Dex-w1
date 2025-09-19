from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from .trajectory_client import TrajectoryClient
from ..common import get_logger
import rclpy

class TrajectorySubscriber(TrajectoryClient):
    def __init__(self, timeout_sec: float = 3.0):
        self.logger = get_logger()
        node_name = "trajectory_subscriber_node"
        super().__init__(node_name,
                         timeout_sec = timeout_sec)
        
    def _init_publisher(self):
        self.logger.info("No need of publisher for trajectory subscriber")

    def _init_subscriber(self):
        self.create_subscription(
            JointTrajectory,
            "/dexe_robot_client/trajectory",
            self._trajectory_callback,
            rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)
        )

    def _init_streaming_subscriber(self):
        self.logger.info("No need of streaming subscriber for trajectory subscriber")