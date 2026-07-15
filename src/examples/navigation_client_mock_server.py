#!/usr/bin/env python3
"""Mock 底盘服务端 —— 不依赖真实机器人，独立验证 navigation_client。

模拟 dev_w1 基线真实模式（is_sim=False）的全部对端接口：
  pub    /feedback/chassis_pose            10Hz 位姿
  pub    /diagnostics_agg                  1Hz 电量 88.0
  pub    obstacle_detector/has_obstacle    1Hz False
  action navigate_to_pose                  约 3s 完成，支持取消
  action /navigate_through_poses           路网导航，约 3s 完成，支持取消
  action /auto_charging                    约 3s 完成，支持取消
  action control/precise_control           精控（PreciseControl），约 2s 完成，支持取消
  srv    /map_asset_manager/get_all_nav_points  返回 2 个示例点位
  参数节点 controller_server / local_costmap/local_costmap / global_costmap/global_costmap
         （预声明 goal_checker.* / FollowPath.* / footprint / inflation_layer.inflation_radius）

用法（终端 1）：
  source /opt/ros/humble/setup.bash && source ~/dexforce/8_interfaces/install/setup.bash
  export ROS_DOMAIN_ID=88   # 与测试驱动同域，隔离本机残留的 nav2 进程
  python3 navigation_client_mock_server.py
然后在终端 2 运行 navigation_client_standalone_test.py。
"""

import math
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from nav_interfaces.action import PreciseControl
from nav_interfaces.msg import NavPoint
from nav_interfaces.srv import GetAllNavPoints
from chassis_interfaces.action import AutoCharging


class MockChassisServer(Node):

    def __init__(self):
        super().__init__("mock_chassis_server")
        cb = ReentrantCallbackGroup()

        # ---- topic：位姿 / 电量 / 障碍物 ----
        self._pose_pub = self.create_publisher(
            PoseStamped, "/feedback/chassis_pose", 10)
        self._diag_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics_agg", 10)
        self._obstacle_pub = self.create_publisher(
            Bool, "obstacle_detector/has_obstacle", 10)
        self.publish_pose_enabled = True
        self.create_timer(0.1, self._publish_pose, callback_group=cb)
        self.create_timer(1.0, self._publish_diag, callback_group=cb)

        # ---- action：Nav2 导航 / 路网导航 / 回充 / 精控 ----
        self._nav_server = ActionServer(
            self, NavigateToPose, "navigate_to_pose",
            execute_callback=self._execute_nav,
            cancel_callback=lambda req: CancelResponse.ACCEPT,
            callback_group=cb)
        self._road_nav_server = ActionServer(
            self, NavigateThroughPoses, "/navigate_through_poses",
            execute_callback=self._execute_road_nav,
            cancel_callback=lambda req: CancelResponse.ACCEPT,
            callback_group=cb)
        self._charge_server = ActionServer(
            self, AutoCharging, "/auto_charging",
            execute_callback=self._execute_charging,
            cancel_callback=lambda req: CancelResponse.ACCEPT,
            callback_group=cb)
        self._control_server = ActionServer(
            self, PreciseControl, "control/precise_control",
            execute_callback=self._execute_control,
            cancel_callback=lambda req: CancelResponse.ACCEPT,
            callback_group=cb)

        # ---- 导航点查询 ----
        self.create_service(
            GetAllNavPoints, "/map_asset_manager/get_all_nav_points",
            self._on_get_all_nav_points, callback_group=cb)

        self.get_logger().info(
            "mock 底盘服务端就绪（导航/路网导航/回充/精控/导航点/话题/参数）")

    # ---------- topic ----------
    def _publish_pose(self):
        if not self.publish_pose_enabled:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = 1.0
        msg.pose.position.y = 2.0
        msg.pose.orientation.z = math.sin(math.radians(45.0) / 2.0)
        msg.pose.orientation.w = math.cos(math.radians(45.0) / 2.0)
        self._pose_pub.publish(msg)

    def _publish_diag(self):
        arr = DiagnosticArray()
        st = DiagnosticStatus()
        st.name = "/Devices/System/PowerBoard/collector_node: PowerBoard"
        st.values.append(KeyValue(key="battery_percentage", value="88.0"))
        arr.status.append(st)
        self._diag_pub.publish(arr)
        self._obstacle_pub.publish(Bool(data=False))

    # ---------- Nav2 导航 action ----------
    def _execute_nav(self, goal_handle):
        self.get_logger().info("[mock] 收到导航目标，3s 后完成")
        for _ in range(30):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("[mock] 导航已取消")
                return NavigateToPose.Result()
            fb = NavigateToPose.Feedback()
            fb.distance_remaining = 1.0
            goal_handle.publish_feedback(fb)
            time.sleep(0.1)
        goal_handle.succeed()
        self.get_logger().info("[mock] 导航完成")
        return NavigateToPose.Result()

    # ---------- 路网导航 action ----------
    def _execute_road_nav(self, goal_handle):
        n = len(goal_handle.request.poses)
        self.get_logger().info(f"[mock] 收到路网导航目标（{n} 路点），3s 后完成")
        for _ in range(30):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("[mock] 路网导航已取消")
                return NavigateThroughPoses.Result()
            fb = NavigateThroughPoses.Feedback()
            fb.number_of_poses_remaining = n
            fb.distance_remaining = 1.0
            goal_handle.publish_feedback(fb)
            time.sleep(0.1)
        goal_handle.succeed()
        self.get_logger().info("[mock] 路网导航完成")
        return NavigateThroughPoses.Result()

    # ---------- 回充 action ----------
    def _execute_charging(self, goal_handle):
        self.get_logger().info("[mock] 收到回充目标，3s 后完成")
        for i in range(30):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = AutoCharging.Result()
                result.success = False
                result.message = "canceled by client"
                self.get_logger().info("[mock] 回充已取消")
                return result
            fb = AutoCharging.Feedback()
            fb.current_state = "Docking" if i < 15 else "Charging"
            fb.retry_count = 0
            goal_handle.publish_feedback(fb)
            time.sleep(0.1)
        goal_handle.succeed()
        result = AutoCharging.Result()
        result.success = True
        result.message = "charging ok"
        self.get_logger().info("[mock] 回充完成")
        return result

    # ---------- 精控 action（PreciseControl）----------
    def _execute_control(self, goal_handle):
        self.get_logger().info("[mock] 收到精控目标，2s 后完成")
        for i in range(20):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = PreciseControl.Result()
                result.success = False
                result.message = "CANCELLED"
                self.get_logger().info("[mock] 精控已取消")
                return result
            fb = PreciseControl.Feedback()
            fb.current_status = "CONTROL"
            goal_handle.publish_feedback(fb)
            time.sleep(0.1)
        goal_handle.succeed()
        result = PreciseControl.Result()
        result.success = True
        result.message = "SUCCEEDED"
        self.get_logger().info("[mock] 精控完成")
        return result

    # ---------- 导航点查询 ----------
    def _on_get_all_nav_points(self, request, response):
        response.success = True
        response.points = [
            NavPoint(id=1, name="point_a", type="STATION",
                    x=1.0, y=0.5, theta=0.0, tolerance=0.0),
            NavPoint(id=2, name="point_b", type="CHARGING",
                    x=2.0, y=0.5, theta=1.57, tolerance=0.0),
        ]
        return response


def _make_param_node(name: str, namespace: str, params: dict) -> Node:
    node = rclpy.create_node(name, namespace=namespace)
    for pname, pvalue in params.items():
        node.declare_parameter(pname, pvalue)
    return node


def main():
    rclpy.init()
    server = MockChassisServer()
    controller = _make_param_node(
        "controller_server", "", {
            "goal_checker.lon_goal_tolerance": 0.25,
            "goal_checker.lat_goal_tolerance": 0.25,
            "goal_checker.yaw_goal_tolerance": 0.25,
            "FollowPath.vx_max": 0.6,
            "FollowPath.wz_max": 0.9,
        })
    local_costmap = _make_param_node(
        "local_costmap", "local_costmap", {
            "footprint": "[]",
            "inflation_layer.inflation_radius": 0.55,
        })
    global_costmap = _make_param_node(
        "global_costmap", "global_costmap", {
            "footprint": "[]",
            "inflation_layer.inflation_radius": 0.55,
        })

    executor = MultiThreadedExecutor(num_threads=6)
    for node in (server, controller, local_costmap, global_costmap):
        executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
