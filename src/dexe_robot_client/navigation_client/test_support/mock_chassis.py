"""In-process ROS 2 peers used by the navigation_client automatic self-test.

This module is intentionally private.  The only public test entry is
``python3 -m dexe_robot_client.navigation_client.interface_test``.  A package
installation may additionally expose the optional ``dexe-navigation-client-test``
console alias.
"""

import math
import time

import rclpy
from chassis_interfaces.action import AutoCharging
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_interfaces.action import PreciseControl
from nav_interfaces.msg import NavPoint
from nav_interfaces.srv import GetAllNavPoints
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Bool


class MockChassisServer(Node):
    """Provide every real-mode peer plus the simulated pose topic."""

    def __init__(self):
        super().__init__("navigation_client_self_test_peer")
        callback_group = ReentrantCallbackGroup()
        self.publish_pose_enabled = True
        self.nav_goal_count = 0

        self._pose_pub = self.create_publisher(PoseStamped,
                                               "/feedback/chassis_pose", 10)
        self._sim_pose_pub = self.create_publisher(
            PoseStamped, "/feedback_sim/chassis_pose", 10)
        self._diag_pub = self.create_publisher(DiagnosticArray,
                                               "/diagnostics_agg", 10)
        self._obstacle_pub = self.create_publisher(
            Bool, "obstacle_detector/has_obstacle", 10)
        self.create_timer(0.1,
                          self._publish_state,
                          callback_group=callback_group)

        self._nav_server = ActionServer(
            self,
            NavigateToPose,
            "navigate_to_pose",
            execute_callback=self._execute_nav,
            cancel_callback=self._accept_cancel,
            callback_group=callback_group,
        )
        self._control_server = ActionServer(
            self,
            PreciseControl,
            "control/precise_control",
            execute_callback=self._execute_control,
            cancel_callback=self._accept_cancel,
            callback_group=callback_group,
        )
        self._charge_server = ActionServer(
            self,
            AutoCharging,
            "/auto_charging",
            execute_callback=self._execute_charging,
            cancel_callback=self._accept_cancel,
            callback_group=callback_group,
        )
        self.create_service(
            GetAllNavPoints,
            "/map_asset_manager/get_all_nav_points",
            self._get_nav_points,
            callback_group=callback_group,
        )

    @staticmethod
    def _accept_cancel(_request):
        return CancelResponse.ACCEPT

    def _publish_state(self):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = 1.0
        pose.pose.position.y = 2.0
        pose.pose.orientation.z = math.sin(math.radians(45.0) / 2.0)
        pose.pose.orientation.w = math.cos(math.radians(45.0) / 2.0)
        if self.publish_pose_enabled:
            self._pose_pub.publish(pose)
            self._sim_pose_pub.publish(pose)

        diagnostics = DiagnosticArray()
        status = DiagnosticStatus()
        status.name = "/Devices/System/PowerBoard/collector_node: PowerBoard"
        status.values.append(KeyValue(key="battery_percentage", value="88.0"))
        diagnostics.status.append(status)
        self._diag_pub.publish(diagnostics)
        self._obstacle_pub.publish(Bool(data=False))

    @staticmethod
    def _run_steps(goal_handle, count, feedback_factory):
        for index in range(count):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return False
            goal_handle.publish_feedback(feedback_factory(index))
            time.sleep(0.05)
        goal_handle.succeed()
        return True

    def _execute_nav(self, goal_handle):
        self.nav_goal_count += 1

        def feedback(_index):
            msg = NavigateToPose.Feedback()
            msg.distance_remaining = 1.0
            return msg

        self._run_steps(goal_handle, 12, feedback)
        return NavigateToPose.Result()

    def _execute_control(self, goal_handle):

        def feedback(_index):
            msg = PreciseControl.Feedback()
            msg.current_status = "CONTROL"
            return msg

        success = self._run_steps(goal_handle, 10, feedback)
        result = PreciseControl.Result()
        result.success = success
        result.message = "SUCCEEDED" if success else "CANCELLED"
        return result

    def _execute_charging(self, goal_handle):

        def feedback(index):
            msg = AutoCharging.Feedback()
            msg.current_state = "Docking" if index < 6 else "Charging"
            msg.retry_count = 0
            return msg

        success = self._run_steps(goal_handle, 12, feedback)
        result = AutoCharging.Result()
        result.success = success
        result.message = "charging ok" if success else "canceled by client"
        return result

    @staticmethod
    def _get_nav_points(_request, response):
        response.success = True
        response.points = [
            NavPoint(
                id=1,
                name="point_a",
                type="STATION",
                x=1.0,
                y=0.5,
                theta=0.0,
                tolerance=0.0,
            ),
            NavPoint(
                id=2,
                name="point_b",
                type="CHARGING",
                x=2.0,
                y=0.5,
                theta=1.57,
                tolerance=0.0,
            ),
        ]
        return response


def make_parameter_nodes():
    """Create peers for the three Nav2 parameter namespaces."""

    def make(name, namespace, parameters):
        node = rclpy.create_node(name, namespace=namespace)
        for parameter_name, value in parameters.items():
            node.declare_parameter(parameter_name, value)
        return node

    return (
        make(
            "controller_server",
            "",
            {
                "current_goal_checker": "high_precision_goal_checker",
                "simple_goal_checker.xy_goal_tolerance": 0.25,
                "simple_goal_checker.yaw_goal_tolerance": 0.25,
                "position_goal_checker.xy_goal_tolerance": 0.25,
                "rotation_goal_checker.yaw_goal_tolerance": 0.25,
                "longitudinal_goal_checker.lon_goal_tolerance": 0.25,
                "longitudinal_goal_checker.lat_goal_tolerance": 0.25,
                "longitudinal_goal_checker.yaw_goal_tolerance": 0.25,
                "high_precision_goal_checker.lon_goal_tolerance": 0.25,
                "high_precision_goal_checker.lat_goal_tolerance": 0.25,
                "high_precision_goal_checker.yaw_goal_tolerance": 0.25,
                "FollowPath.vx_max": 0.6,
                "FollowPath.wz_max": 0.9,
            },
        ),
        make(
            "local_costmap",
            "local_costmap",
            {
                "footprint": "[]",
                "inflation_layer.inflation_radius": 0.55,
            },
        ),
        make(
            "global_costmap",
            "global_costmap",
            {
                "footprint": "[]",
                "inflation_layer.inflation_radius": 0.55,
            },
        ),
    )
