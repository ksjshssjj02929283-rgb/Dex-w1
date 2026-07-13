from .navigation_client import NavigationClient
from ..common import get_logger
from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav_interfaces.action import PreciseControl, NavigateThroughPoints
from nav_interfaces.srv import GetAllNavPoints
from chassis_interfaces.action import AutoCharging
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.parameter import Parameter
from std_msgs.msg import Bool


class ChassisClient(NavigationClient):

    def __init__(self, is_sim: bool = False, timeout_sec: float = 1.0):
        self.logger = get_logger()
        node_name = "chassis_navigation"
        super().__init__(node_name, is_sim, timeout_sec)

    def _init_publisher(self, is_sim: bool):
        if is_sim:
            self._set_nav_publisher = self.create_publisher(
                PoseStamped, f"/control_sim/nav2_pose", 10)
        else:
            self._set_nav_publisher = self.create_publisher(
                PoseStamped, f"/control/nav2_pose", 10)

    def _init_subscriber(self, is_sim: bool):
        if is_sim:
            self._get_pos_subscriber = self.create_subscription(
                PoseStamped, f"/feedback_sim/chassis_pose",
                self._chassis_feedback_callback, 10)
            self._nav_points_client = self.create_client(
                GetAllNavPoints, '/map_asset_manager/get_all_nav_points')
            self._set_ctrl_client = ActionClient(self, PreciseControl,
                                                 'control/precise_control')
        else:
            self._get_pos_subscriber = self.create_subscription(
                PoseStamped, f"/feedback/chassis_pose",
                self._chassis_feedback_callback, 10)
            self._get_battery_subscriber = self.create_subscription(
                DiagnosticArray, f"/diagnostics_agg",
                self._battery_feedback_callback, 10)

            self._set_nav_client = ActionClient(self, NavigateToPose,
                                                'navigate_to_pose')
            self._road_nav_action_client = ActionClient(
                self, NavigateThroughPoses, '/navigate_through_poses')
            self._path_nav_action_client = ActionClient(
                self, NavigateThroughPoints, '/navigate_through_points')
            self._nav_points_client = self.create_client(
                GetAllNavPoints, '/map_asset_manager/get_all_nav_points')
            self._set_ctrl_client = ActionClient(self, PreciseControl,
                                                 'control/precise_control')
            self._auto_charging_action_client = ActionClient(
                self, AutoCharging, '/auto_charging')

            self._controller_set_param_client = self.create_client(
                SetParameters, '/controller_server/set_parameters')

            self._controller_get_param_client = self.create_client(
                GetParameters, '/controller_server/get_parameters')

            self._local_costmap_set_param_client = self.create_client(
                SetParameters, '/local_costmap/local_costmap/set_parameters')

            self._local_costmap_get_param_client = self.create_client(
                GetParameters, '/local_costmap/local_costmap/get_parameters')

            self._global_costmap_set_param_client = self.create_client(
                SetParameters, '/global_costmap/global_costmap/set_parameters')

            self._global_costmap_get_param_client = self.create_client(
                GetParameters, '/global_costmap/global_costmap/get_parameters')

            self._controller_set_param_client = self.create_client(
                SetParameters, '/controller_server/set_parameters')

            self._controller_get_param_client = self.create_client(
                GetParameters, '/controller_server/get_parameters')

            self._local_costmap_set_param_client = self.create_client(
                SetParameters, '/local_costmap/local_costmap/set_parameters')

            self._local_costmap_get_param_client = self.create_client(
                GetParameters, '/local_costmap/local_costmap/get_parameters')

            self._global_costmap_set_param_client = self.create_client(
                SetParameters, '/global_costmap/global_costmap/set_parameters')

            self._global_costmap_get_param_client = self.create_client(
                GetParameters, '/global_costmap/global_costmap/get_parameters')

            # 订阅障碍物检测话题（仿真和真实模式都订阅）
            self._obstacle_subscriber = self.create_subscription(
                Bool, "obstacle_detector/has_obstacle", self._obstacle_callback,
                10)
