from .navigation_client import NavigationClient
from ..common import get_logger
from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger


class ChassisClient(NavigationClient):

    def __init__(self, is_sim: bool = False, timeout_sec: float = 1.0):
        self.logger = get_logger()
        node_name = "chassis_navigation"
        super().__init__(node_name, is_sim, timeout_sec)

    def _init_publisher(self, is_sim: bool):
        if is_sim:
            self._set_nav_publisher = self.create_publisher(
                PoseStamped, f"/control_sim/nav2_pose", 10)
            self._set_ctrl_publisher = self.create_publisher(
                PoseStamped, f"/control_sim/ctrl_pose", 10)
        else:
            self._set_nav_publisher = self.create_publisher(
                PoseStamped, f"/control/nav2_pose", 10)
            self._set_ctrl_publisher = self.create_publisher(
                PoseStamped, f"/control/ctrl_pose", 10)

    def _init_subscriber(self, is_sim: bool):
        if is_sim:
            self._get_pos_subscriber = self.create_subscription(
                PoseStamped, f"/feedback_sim/chassis_pose",
                self._chassis_feedback_callback, 10)
        else:
            self._get_pos_subscriber = self.create_subscription(
                PoseStamped, f"/feedback/chassis_pose",
                self._chassis_feedback_callback, 10)
            self._get_battery_subscriber = self.create_subscription(
                DiagnosticArray, f"/diagnostics_agg",
                self._battery_feedback_callback, 10)

            self._set_nav_client = ActionClient(self, NavigateToPose,
                                                'navigate_to_pose')
            self._get_status_client = self.create_client(
                Trigger, 'control/get_control_status')
            self._cancel_control_client = self.create_client(
                Trigger, 'control/cancel_chassis_control')
            self._set_chassis_charging_client = self.create_client(
                Trigger, 'auto_charging')
