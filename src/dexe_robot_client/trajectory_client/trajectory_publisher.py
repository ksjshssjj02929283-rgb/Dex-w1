from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from .trajectory_client import TrajectoryClient
from ..common import get_logger
import rclpy
from robot_server_interfaces.msg import StreamingCommand

class TrajectoryPublisher(TrajectoryClient):
    def __init__(self, timeout_sec: float = 3.0):
        self.logger = get_logger()
        node_name = "trajectory_publisher_node"
        super().__init__(node_name,
                         timeout_sec = timeout_sec)
        
    def _init_publisher(self):
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory, '/dexe_robot_client/trajectory', 1
        )

    def _init_subscriber(self):
        self.logger.info("No need of subscriber for trajectory publisher")

    def _init_streaming_subscriber(self):
        self.create_subscription(
            StreamingCommand,
            "/control/streaming_command_sim",
            self._streaming_cmd_callback,
            rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)

        )

        self.create_subscription(
            StreamingCommand,
            "/control/streaming_command",
            self._streaming_cmd_callback,
            rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)
        )

