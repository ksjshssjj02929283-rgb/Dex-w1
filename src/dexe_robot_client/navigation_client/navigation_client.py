import time
import threading
import numpy as np
import math
from abc import ABC, abstractmethod
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose2D
from action_msgs.msg import GoalStatus
from scipy.spatial.transform import Rotation as R
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav_interfaces.action import PreciseControl, NavigateThroughPoints
from nav_interfaces.msg import NavPoint
from nav_interfaces.srv import GetAllNavPoints
from chassis_interfaces.action import AutoCharging
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import ParameterType
from rclpy.parameter import Parameter
from geometry_msgs.msg import Polygon, Point32
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import ParameterType
from rclpy.parameter import Parameter
from geometry_msgs.msg import Polygon, Point32
from std_msgs.msg import Bool

from ..common import get_logger


class NavigationClient(ABC, Node):
    AUTO_CHARGING_STATUS_NAMES = {
        GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
        GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
        GoalStatus.STATUS_EXECUTING: "EXECUTING",
        GoalStatus.STATUS_CANCELING: "CANCELING",
        GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
        GoalStatus.STATUS_CANCELED: "CANCELED",
        GoalStatus.STATUS_ABORTED: "ABORTED",
    }
    ROAD_NAV_STATUS_NAMES = AUTO_CHARGING_STATUS_NAMES
    PATH_NAV_STATUS_NAMES = AUTO_CHARGING_STATUS_NAMES

    def __init__(self,
                 name: str,
                 is_sim: bool = False,
                 timeout_sec: float = 1.0):
        self.logger = get_logger()
        if is_sim:
            self.node_name = name + "_sim"
        else:
            self.node_name = name + "_real"
        super().__init__(self.node_name)
        self._last_msg_time = None
        self._last_chassis_processed_feedback = None
        self._cur_pos = None
        self._connection_timeout = 1.0
        self._set_nav_publisher = None
        self._set_nav_client = None
        self._set_ctrl_client = None
        self._get_pos_subscriber = None
        self._auto_charging_action_client = None
        self._auto_charging_goal_handle = None
        self._auto_charging_send_goal_future = None
        self._auto_charging_result_future = None
        self._auto_charging_cancel_future = None
        self._auto_charging_task_id = 0
        self._auto_charging_status = "IDLE"
        self._auto_charging_done = False
        self._auto_charging_success = False
        self._last_auto_charging_feedback = None
        self._last_auto_charging_result_message = ""
        self._controller_set_param_client = None
        self._local_costmap_set_param_client = None
        self._global_costmap_set_param_client = None

        self._controller_get_param_client = None
        self._local_costmap_get_param_client = None
        self._global_costmap_get_param_client = None

        self._battery_percentage = -1.0
        self._obstacle_subscriber = None
        self._has_obstacle = False

        self._nav_goal_handle = None
        self._nav_result_future = None
        self._nav_status = "IDLE"  # 或 NAVIGATING / SUCCEEDED / FAILED / CANCELLED
        self._nav_result = None  # 存储最后的导航结果
        self._nav_task_id = 0  # 导航任务ID，用于区分不同的导航任务
        self._road_nav_action_client = None
        self._road_nav_goal_handle = None
        self._road_nav_send_goal_future = None
        self._road_nav_result_future = None
        self._road_nav_cancel_future = None
        self._road_nav_status = "IDLE"
        self._road_nav_done = False
        self._road_nav_success = False
        self._last_road_nav_feedback = None
        self._last_road_nav_result_message = ""
        self._nav_points_client = None

        self._path_nav_action_client = None
        self._path_nav_goal_handle = None
        self._path_nav_send_goal_future = None
        self._path_nav_result_future = None
        self._path_nav_cancel_future = None
        self._path_nav_status = "IDLE"
        self._path_nav_done = False
        self._path_nav_success = False
        self._last_path_nav_feedback = None
        self._last_path_nav_result_message = ""
        self._path_nav_task_id = 0
        self._nav_points_timeout_sec = 10.0
        self._control_lock = threading.Lock()
        self._control_goal_handle = None
        self._control_result_future = None
        self._control_status = "IDLE"
        self._control_result = None
        self._control_task_id = 0
        self._last_control_status = "IDLE"
        self._last_control_status_success = True
        self._last_control_status_time = None
        self._last_logged_control_status = None
        self._control_terminal_states = {"SUCCEEDED", "FAILED", "CANCELLED"}

        self._init_subscriber(is_sim)
        self._init_publisher(is_sim)
        self.spin_once(timeout_sec=timeout_sec)

        self.create_timer(0.1, self._check_connection_status)

    def _check_connection_status(self):
        """定时检查消息是否超时"""
        if self._last_msg_time is not None:
            cur_time = self.get_clock().now().nanoseconds / 1e9
            if (cur_time - self._last_msg_time) > self._connection_timeout:
                self._cur_pos = None
                self._last_msg_time = None

    def is_connected(self, timeout_sec: float = 0.1):
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            self.spin_once()
            if self._last_msg_time is not None:
                return True
            time.sleep(0.01)
        return False

    @abstractmethod
    def _init_subscriber(self, is_sim: bool):
        pass

    @abstractmethod
    def _init_publisher(self, is_sim: bool):
        pass

    def _parse_msg_header(self, msg) -> dict:
        header = None

        if hasattr(msg, "header"):
            header = {
                "stamp": {
                    "sec":
                        msg.header.stamp.sec
                        if hasattr(msg.header.stamp, "sec") else 0,
                    "nanosec":
                        msg.header.stamp.nanosec if hasattr(
                            msg.header.stamp, "nanosec") else 0,
                },
                "frame_id":
                    msg.header.frame_id
                    if hasattr(msg.header, "frame_id") else "",
            }
        else:
            header = {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""}

        return header

    def _chassis_feedback_callback(self, msg):
        processed = {"header": {}, "data": {}}
        processed["header"] = self._parse_msg_header(msg)
        processed["data"] = {
            "position": [
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
            ],
            "orientation": [
                msg.pose.orientation.x, msg.pose.orientation.y,
                msg.pose.orientation.z, msg.pose.orientation.w
            ]
        }

        self._last_chassis_processed_feedback = processed
        self._last_msg_time = time.time()

    def _battery_feedback_callback(self, msg):
        '''解析电池状态消息'''
        for status in msg.status:
            if status.name == "/Devices/System/PowerBoard/collector_node: PowerBoard":
                for key_value in status.values:
                    if key_value.key == "battery_percentage":
                        self._battery_percentage = float(key_value.value)
                        break

    def _obstacle_callback(self, msg):
        """障碍物检测回调函数"""
        self._has_obstacle = msg.data

    def get_battery_percentage(self) -> tuple[bool, float]:
        """获取当前电池电量百分比。

        Returns:
            float: 电池电量百分比，范围为 0.0 到 100.0。如果未收到电池状态消息，则返回 -1.0。
        """
        if (self._battery_percentage < 0):
            return False, -1.0
        else:
            return True, self._battery_percentage

    def get_front_obstacle(self) -> bool:
        """获取前方障碍物检测状态。

        Returns:
            bool: 如果检测到障碍物则返回 True，否则返回 False。
        """
        # self.spin_once()
        return self._has_obstacle

    def get_chassis_position(self) -> tuple[bool, np.ndarray]:
        """
        获取底盘当前的位置和姿态 (x, y, yaw)。
        Returns:
            一个元组 (success, position)。
            - success (bool): 如果成功获取位置则为 True。
            - position (np.ndarray): 包含 [x, y, yaw] 的NumPy数组，yaw单位为度
        """

        self.spin_once()

        if self._last_chassis_processed_feedback is None:
            self.get_logger().error("No chassis feedback received yet.")
            return False, None

        pos = self._last_chassis_processed_feedback["data"]["position"]
        x, y = pos[0], pos[1]

        q = self._last_chassis_processed_feedback["data"]["orientation"]
        quat_array = np.array([q[0], q[1], q[2], q[3]])
        rotation = R.from_quat(quat_array)

        yaw = rotation.as_euler('xyz', degrees=True)[2]
        self._cur_pos = np.array([x, y, yaw])
        return True, self._cur_pos

    def set_chassis_nav2_position(self, position) -> bool:
        """发送一个导航目标点 (非阻塞)。

        Args:
            position (list/np.ndarray): 目标位置 [x, y, yaw],yaw单位为度。

        Returns:
            bool: 如果目标发送成功则为 True。
        """
        nav2_position = NavigateToPose.Goal()
        nav2_position.pose = PoseStamped()
        nav2_position.pose.header.frame_id = "map"
        nav2_position.pose.pose.position.x = position[0]
        nav2_position.pose.pose.position.y = position[1]
        nav2_position.pose.pose.position.z = 0.0

        rotation = R.from_euler('xyz', [0, 0, position[2]], degrees=True)
        quat_arry = rotation.as_quat()

        nav2_position.pose.pose.orientation.x = quat_arry[0]
        nav2_position.pose.pose.orientation.y = quat_arry[1]
        nav2_position.pose.pose.orientation.z = quat_arry[2]
        nav2_position.pose.pose.orientation.w = quat_arry[3]

        if not self._set_nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error(
                "NavigateToPose action server not available.")
            return False

        # 清理旧的导航状态，防止旧回调干扰新任务
        self._nav_task_id += 1  # 新任务ID
        current_task_id = self._nav_task_id
        self._nav_goal_handle = None
        self._nav_status = "NAVIGATING"
        self._nav_result = None

        # 异步发送
        send_future = self._set_nav_client.send_goal_async(nav2_position)
        send_future.add_done_callback(
            lambda future: self._on_nav_goal_response(future, current_task_id))

        return True

    def get_nav_state(self) -> str:
        """获取当前导航状态。

        Returns:
            str: 导航状态字符串，可能为：
                - "IDLE": 空闲
                - "NAVIGATING": 导航中
                - "SUCCEEDED": 成功
                - "FAILED": 失败
                - "CANCELLED": 已取消
        """
        # 触发 ROS2 回调（非常重要）
        # self.spin_once()

        return self._nav_status

    # def is_nav_done(self) -> bool:
    #     """检查导航是否结束（无论成功、失败还是取消）。"""
    #     return self.get_nav_state() in ["SUCCEEDED", "FAILED", "CANCELLED"]

    def cancel_chassis_nav2(self) -> bool:
        """取消当前的导航任务 (非阻塞)。"""
        # 如果目标还未被接受，等待最多 1 秒让 goal_handle 被设置
        if self._nav_goal_handle is None and self._nav_status == "NAVIGATING":
            self.get_logger().info(
                "Waiting for navigation goal to be accepted before cancelling..."
            )
            start_time = time.time()
            while time.time() - start_time < 1.0:
                self.spin_once(timeout_sec=0.05)
                if self._nav_goal_handle is not None:
                    break
                time.sleep(0.05)

        if self._nav_goal_handle is None:
            self.get_logger().warning("No active navigation goal to cancel.")
            return False

        # 保存当前任务ID
        current_task_id = self._nav_task_id

        try:
            cancel_future = self._nav_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda future: self._on_nav_cancel_response(
                    future, current_task_id))
            self.get_logger().info("Navigation cancel request sent.")
            return True
        except Exception as e:
            self.get_logger().error(
                f"Error cancelling navigation goal: {str(e)}")
            return False

    def _create_road_nav_pose(self, x: float, y: float, yaw_rad: float,
                              frame_id: str) -> PoseStamped:
        """创建 NavigateThroughPoses 路点，yaw 使用弧度。"""
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        half_yaw = float(yaw_rad) * 0.5
        pose.pose.orientation.z = math.sin(half_yaw)
        pose.pose.orientation.w = math.cos(half_yaw)
        return pose

    def set_chassis_road_navigation(self,
                                    waypoints: list,
                                    frame_id: str = "map",
                                    timeout_sec: float = 10.0,
                                    behavior_tree: str = "") -> bool:
        """发送路网/多路点导航任务，非阻塞返回是否成功发出。

        waypoints 中每个路点为 [x, y, yaw_rad] 或 (x, y, yaw_rad)。
        yaw_rad 使用弧度，和 Nav2 NavigateThroughPoses 原生接口一致。
        """
        if self._road_nav_action_client is None:
            self.get_logger().error(
                "NavigateThroughPoses action client not initialized.")
            return False
        if not waypoints:
            self.get_logger().error("Road navigation waypoints is empty.")
            return False

        action_name = getattr(self._road_nav_action_client, "_action_name",
                              "/navigate_through_poses")
        self.get_logger().info(
            f"Waiting for NavigateThroughPoses action server: {action_name}")
        self._road_nav_status = "WAITING_SERVER"
        if not self._road_nav_action_client.wait_for_server(
                timeout_sec=timeout_sec):
            self.get_logger().error(
                "NavigateThroughPoses action server is not available.")
            self._road_nav_status = "FAILED_TO_SEND"
            self._road_nav_done = True
            self._road_nav_success = False
            return False

        try:
            goal = NavigateThroughPoses.Goal()
            goal.poses = [
                self._create_road_nav_pose(x, y, yaw, frame_id)
                for x, y, yaw in waypoints
            ]
            goal.behavior_tree = behavior_tree
        except (TypeError, ValueError) as e:
            self.get_logger().error(f"Invalid road navigation waypoints: {e}")
            self._road_nav_status = "FAILED_TO_SEND"
            self._road_nav_done = True
            self._road_nav_success = False
            return False

        self.get_logger().info(f"Sending {len(goal.poses)} road nav poses.")
        for index, (x, y, yaw) in enumerate(waypoints, start=1):
            self.get_logger().info(
                f"  {index}: x={float(x):.3f}, y={float(y):.3f}, "
                f"yaw={float(yaw):.3f} rad")

        self._road_nav_done = False
        self._road_nav_success = False
        self._road_nav_status = "SENDING"
        self._road_nav_goal_handle = None
        self._road_nav_result_future = None
        self._road_nav_cancel_future = None
        self._last_road_nav_feedback = None
        self._last_road_nav_result_message = ""

        try:
            self._road_nav_send_goal_future = (
                self._road_nav_action_client.send_goal_async(
                    goal, feedback_callback=self._road_nav_feedback_callback))
            self._road_nav_send_goal_future.add_done_callback(
                self._road_nav_goal_done_callback)
            return True
        except Exception as e:
            self.get_logger().error(
                f"Failed to send NavigateThroughPoses goal: {e}")
            self._road_nav_status = "FAILED_TO_SEND"
            self._road_nav_done = True
            self._road_nav_success = False
            return False

    def get_road_navigation_status(self) -> str:
        """获取当前路网/多路点导航状态。"""
        return self._road_nav_status

    def get_road_navigation_result(self) -> tuple[bool, str]:
        """获取最近一次路网/多路点导航结果。"""
        return self._road_nav_success, self._last_road_nav_result_message

    def cancel_road_navigation(self) -> bool:
        """取消当前路网/多路点导航任务。"""
        if self._road_nav_goal_handle is None and self._road_nav_status in {
                "SENDING", "ACCEPTED", "WAITING_SERVER"
        }:
            self.get_logger().info(
                "Waiting for road navigation goal to be accepted before cancelling..."
            )
            start_time = time.time()
            while time.time() - start_time < 1.0:
                self.spin_once(timeout_sec=0.05)
                if self._road_nav_goal_handle is not None:
                    break
                time.sleep(0.05)

        if self._road_nav_goal_handle is None:
            self.get_logger().warning(
                "No accepted NavigateThroughPoses goal to cancel.")
            return False

        try:
            self._road_nav_status = "CANCELING"
            self._road_nav_cancel_future = (
                self._road_nav_goal_handle.cancel_goal_async())
            self._road_nav_cancel_future.add_done_callback(
                self._road_nav_cancel_done_callback)
            self.get_logger().info("NavigateThroughPoses cancel request sent.")
            return True
        except Exception as e:
            self.get_logger().error(
                f"Failed to cancel NavigateThroughPoses goal: {e}")
            return False

    def set_chassis_path_navigation(self,
                                    nav_point_names: list,
                                    nav_point_ids: list = None,
                                    pose: Pose2D = None,
                                    target_yaw: float = None,
                                    timeout_sec: float = 10.0) -> bool:
        """发送路径导航任务（NavigateThroughPoints action），非阻塞返回是否成功发出。

        通过 `/navigate_through_points`（nav_interfaces/action/NavigateThroughPoints）
        按点位名称串接路径，由 route_executor 解析路点并逐段执行。

        Args:
            nav_point_names (list[str]): 路点名称序列，按顺序串接成路径。
            nav_point_ids (list[str] | None): 可选路点 ID 序列，透传给服务端。
            pose (Pose2D | None): 可选附加位姿，透传给服务端。
            target_yaw (float | None): 可选目标朝向（弧度），透传给服务端。
            timeout_sec (float): 等待 action server 的超时时间，秒。

        Returns:
            bool: 目标成功发出返回 True，否则返回 False。
        """
        if self._path_nav_action_client is None:
            self.get_logger().error(
                "NavigateThroughPoints action client not initialized.")
            return False

        action_name = getattr(self._path_nav_action_client, "_action_name",
                              "/navigate_through_points")
        self.get_logger().info(
            f"Waiting for NavigateThroughPoints action server: {action_name}")
        self._path_nav_status = "WAITING_SERVER"
        if not self._path_nav_action_client.wait_for_server(
                timeout_sec=timeout_sec):
            self.get_logger().error(
                "NavigateThroughPoints action server is not available.")
            self._path_nav_status = "FAILED_TO_SEND"
            self._path_nav_done = True
            self._path_nav_success = False
            return False

        try:
            goal = NavigateThroughPoints.Goal()
            goal.nav_point_names = [str(name) for name in nav_point_names]
            if nav_point_ids is not None:
                goal.nav_point_ids = [str(pid) for pid in nav_point_ids]
            if pose is not None:
                goal.pose = pose
            if target_yaw is not None:
                goal.target_yaw = float(target_yaw)
        except (TypeError, ValueError) as e:
            self.get_logger().error(
                f"Invalid path navigation nav_point_names: {e}")
            self._path_nav_status = "FAILED_TO_SEND"
            self._path_nav_done = True
            self._path_nav_success = False
            return False

        self.get_logger().info(
            f"Sending NavigateThroughPoints goal with "
            f"{len(goal.nav_point_names)} nav_point_names.")
        for index, name in enumerate(goal.nav_point_names, start=1):
            self.get_logger().info(f"  {index}: {name}")

        self._path_nav_task_id += 1
        current_task_id = self._path_nav_task_id
        self._path_nav_done = False
        self._path_nav_success = False
        self._path_nav_status = "SENDING"
        self._path_nav_goal_handle = None
        self._path_nav_send_goal_future = None
        self._path_nav_result_future = None
        self._path_nav_cancel_future = None
        self._last_path_nav_feedback = None
        self._last_path_nav_result_message = ""

        def path_nav_feedback_callback(feedback_msg):
            self._path_nav_feedback_callback(feedback_msg, current_task_id)

        try:
            self._path_nav_send_goal_future = (
                self._path_nav_action_client.send_goal_async(
                    goal, feedback_callback=path_nav_feedback_callback))
            self._path_nav_send_goal_future.add_done_callback(
                lambda future: self._path_nav_goal_done_callback(
                    future, current_task_id))
            return True
        except Exception as e:
            self.get_logger().error(
                f"Failed to send NavigateThroughPoints goal: {e}")
            self._path_nav_status = "FAILED_TO_SEND"
            self._path_nav_done = True
            self._path_nav_success = False
            return False

    def get_path_navigation_status(self) -> str:
        """获取当前路径导航（NavigateThroughPoints）状态。"""
        return self._path_nav_status

    def get_path_navigation_result(self) -> tuple[bool, str]:
        """获取最近一次路径导航（NavigateThroughPoints）结果。"""
        return self._path_nav_success, self._last_path_nav_result_message

    def cancel_path_navigation(self) -> bool:
        """取消当前路径导航（NavigateThroughPoints）任务。"""
        if self._path_nav_goal_handle is None and self._path_nav_status in {
                "SENDING", "ACCEPTED", "WAITING_SERVER"
        }:
            self.get_logger().info(
                "Waiting for path navigation goal to be accepted before cancelling..."
            )
            current_task_id = self._path_nav_task_id
            start_time = time.time()
            while time.time() - start_time < 1.0:
                if current_task_id != self._path_nav_task_id:
                    break
                self.spin_once(timeout_sec=0.05)
                if self._path_nav_goal_handle is not None:
                    break
                time.sleep(0.05)

        if self._path_nav_goal_handle is None:
            self.get_logger().warning(
                "No accepted NavigateThroughPoints goal to cancel.")
            return False

        try:
            current_task_id = self._path_nav_task_id
            self._path_nav_status = "CANCELING"
            self._path_nav_cancel_future = (
                self._path_nav_goal_handle.cancel_goal_async())
            self._path_nav_cancel_future.add_done_callback(
                lambda future: self._path_nav_cancel_done_callback(
                    future, current_task_id))
            self.get_logger().info(
                "NavigateThroughPoints cancel request sent.")
            return True
        except Exception as e:
            self.get_logger().error(
                f"Failed to cancel NavigateThroughPoints goal: {e}")
            return False

    def _parse_nav_point(
            self,
            point: NavPoint) -> tuple[str, tuple[float, float, float]] | None:
        """从 NavPoint 消息解析点位名称和 x/y/theta_rad。"""
        if not isinstance(point, NavPoint):
            self.get_logger().warning(
                f"Skip unexpected nav point type: {type(point).__name__}")
            return None

        if not point.name:
            self.get_logger().warning("Skip nav point with empty name.")
            return None

        return (
            str(point.name),
            (
                float(point.x),
                float(point.y),
                float(point.theta),
            ),
        )

    def get_all_nav_points(self) -> dict[str, tuple[float, float, float]]:
        """获取所有路网导航点，返回点位名称到 x/y/theta_rad 的映射。"""
        if self._nav_points_client is None:
            self.get_logger().error(
                "GetAllNavPoints service client not initialized.")
            return {}

        service_name = self._nav_points_client.srv_name
        self.get_logger().info(
            f"Waiting for GetAllNavPoints service: {service_name}")
        if not self._nav_points_client.wait_for_service(
                timeout_sec=self._nav_points_timeout_sec):
            self.get_logger().error("GetAllNavPoints service is not available.")
            return {}

        future = self._nav_points_client.call_async(GetAllNavPoints.Request())
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=self._nav_points_timeout_sec)
        if not future.done():
            self.get_logger().error("GetAllNavPoints service call timed out.")
            return {}

        response = future.result()
        if response is None:
            exception_msg = (str(future.exception())
                             if future.exception() else "unknown error")
            self.get_logger().error(
                f"GetAllNavPoints service call failed: {exception_msg}")
            return {}
        if not response.success:
            self.get_logger().error(
                "GetAllNavPoints service returned success=False.")
            return {}

        points_by_name: dict[str, tuple[float, float, float]] = {}
        for point in response.points:
            parsed_point = self._parse_nav_point(point)
            if parsed_point is None:
                continue
            name, pose = parsed_point
            points_by_name[name] = pose
        return points_by_name

    def _auto_charging_feedback_callback(self, feedback_msg, task_id):
        if task_id != self._auto_charging_task_id:
            self.get_logger().debug(
                f"Ignoring feedback for old AutoCharging task {task_id} "
                f"(current: {self._auto_charging_task_id})")
            return

        feedback = feedback_msg.feedback
        self._last_auto_charging_feedback = feedback
        if self._auto_charging_status != "CANCELING":
            self._auto_charging_status = "EXECUTING"
        self.get_logger().info("Auto charging feedback: "
                               f"state={feedback.current_state}, "
                               f"retry={feedback.retry_count}")

    def set_chassis_auto_charging(self,
                                  force: bool = True,
                                  timeout_sec: float = 10.0) -> bool:
        """发送回充 action，非阻塞返回是否成功发出。"""
        if self._auto_charging_action_client is None:
            self.get_logger().error(
                "Auto charging action client not initialized.")
            return False

        action_name = getattr(self._auto_charging_action_client, "_action_name",
                              "/auto_charging")
        self.get_logger().info(
            f"Waiting for AutoCharging action server: {action_name}")
        self._auto_charging_status = "WAITING_SERVER"
        if not self._auto_charging_action_client.wait_for_server(
                timeout_sec=timeout_sec):
            self.get_logger().error("Auto charging action server not available")
            self._auto_charging_status = "FAILED_TO_SEND"
            self._auto_charging_done = True
            self._auto_charging_success = False
            return False

        goal_msg = AutoCharging.Goal()
        goal_msg.force = force

        self.get_logger().info(f"Sending AutoCharging goal: force={force}")
        self._auto_charging_task_id += 1
        current_task_id = self._auto_charging_task_id
        self._auto_charging_done = False
        self._auto_charging_success = False
        self._auto_charging_status = "SENDING"
        self._auto_charging_goal_handle = None
        self._auto_charging_result_future = None
        self._auto_charging_cancel_future = None
        self._last_auto_charging_feedback = None
        self._last_auto_charging_result_message = ""

        def auto_charging_feedback_callback(feedback_msg):
            self._auto_charging_feedback_callback(feedback_msg, current_task_id)

        try:
            self._auto_charging_send_goal_future = (
                self._auto_charging_action_client.send_goal_async(
                    goal_msg,
                    feedback_callback=auto_charging_feedback_callback))
            self._auto_charging_send_goal_future.add_done_callback(
                lambda future: self._auto_charging_goal_done_callback(
                    future, current_task_id))
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to send AutoCharging goal: {e}")
            self._auto_charging_status = "FAILED_TO_SEND"
            self._auto_charging_done = True
            self._auto_charging_success = False
            return False

    def get_auto_charging_status(self) -> str:
        """获取当前回充 action 状态。"""
        return self._auto_charging_status

    def get_auto_charging_result(self) -> tuple[bool, str]:
        """获取最近一次回充 action 结果。"""
        return self._auto_charging_success, self._last_auto_charging_result_message

    def cancel_auto_charging(self) -> bool:
        """取消当前回充 action。"""
        if self._auto_charging_goal_handle is None and self._auto_charging_status in {
                "WAITING_SERVER", "SENDING", "ACCEPTED", "EXECUTING"
        }:
            self.get_logger().info(
                "Waiting for AutoCharging goal to be accepted before cancelling..."
            )
            current_task_id = self._auto_charging_task_id
            start_time = time.time()
            while time.time() - start_time < 1.0:
                if current_task_id != self._auto_charging_task_id:
                    break
                self._spin_once_if_not_executed(timeout_sec=0.05)
                if self._auto_charging_goal_handle is not None:
                    break
                time.sleep(0.05)

        if self._auto_charging_goal_handle is None:
            self.get_logger().warning(
                "No accepted AutoCharging goal to cancel.")
            return False

        try:
            current_task_id = self._auto_charging_task_id
            self._auto_charging_status = "CANCELING"
            self._auto_charging_cancel_future = (
                self._auto_charging_goal_handle.cancel_goal_async())
            self._auto_charging_cancel_future.add_done_callback(
                lambda future: self._auto_charging_cancel_done_callback(
                    future, current_task_id))
            self.get_logger().info("AutoCharging cancel request sent.")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to cancel AutoCharging goal: {e}")
            return False

    def _auto_charging_goal_done_callback(self, future, task_id):
        if task_id != self._auto_charging_task_id:
            self.get_logger().debug(
                f"Ignoring goal response for old AutoCharging task {task_id} "
                f"(current: {self._auto_charging_task_id})")
            return

        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"AutoCharging goal send failed: {e}")
            self._auto_charging_status = "FAILED_TO_SEND"
            self._auto_charging_done = True
            self._auto_charging_success = False
            return

        if not goal_handle.accepted:
            self.get_logger().error("AutoCharging goal was rejected.")
            self._auto_charging_status = "REJECTED"
            self._auto_charging_done = True
            self._auto_charging_success = False
            return

        self._auto_charging_goal_handle = goal_handle
        self._auto_charging_status = "ACCEPTED"
        self.get_logger().info("AutoCharging goal accepted.")
        self._auto_charging_result_future = goal_handle.get_result_async()
        self._auto_charging_result_future.add_done_callback(
            lambda future: self._auto_charging_result_done_callback(
                future, task_id))

    def _auto_charging_cancel_done_callback(self, future, task_id):
        if task_id != self._auto_charging_task_id:
            self.get_logger().debug(
                f"Ignoring cancel response for old AutoCharging task {task_id} "
                f"(current: {self._auto_charging_task_id})")
            return

        try:
            response = future.result()
            self.get_logger().info(
                f"AutoCharging cancel response: goals_canceling={len(response.goals_canceling)}"
            )
        except Exception as e:
            self.get_logger().error(f"AutoCharging cancel failed: {e}")

    def _auto_charging_result_done_callback(self, future, task_id):
        if task_id != self._auto_charging_task_id:
            self.get_logger().debug(
                f"Ignoring result for old AutoCharging task {task_id} "
                f"(current: {self._auto_charging_task_id})")
            return

        try:
            response = future.result()
            result = response.result
            self._auto_charging_status = self.AUTO_CHARGING_STATUS_NAMES.get(
                response.status, "UNKNOWN")
            self._auto_charging_success = (self._auto_charging_status
                                           == "SUCCEEDED" and
                                           bool(result.success))
            self._last_auto_charging_result_message = result.message
            self._auto_charging_done = True
            self._auto_charging_goal_handle = None

            if self._auto_charging_success:
                self.get_logger().info(
                    f"AutoCharging succeeded: {result.message}")
            else:
                self.get_logger().error(
                    f"AutoCharging finished with status={self._auto_charging_status}: "
                    f"{result.message}")
        except Exception as e:
            self.get_logger().error(f"Auto charging result error: {e}")
            self._auto_charging_status = "UNKNOWN"
            self._auto_charging_success = False
            self._auto_charging_done = True
            self._auto_charging_goal_handle = None

    def _road_nav_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self._last_road_nav_feedback = feedback
        if self._road_nav_status != "CANCELING":
            self._road_nav_status = "EXECUTING"
        self.get_logger().info(
            "NavigateThroughPoses feedback: "
            f"poses_remaining={feedback.number_of_poses_remaining}, "
            f"distance_remaining={feedback.distance_remaining:.3f} m")

    def _road_nav_goal_done_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(
                f"NavigateThroughPoses goal send failed: {e}")
            self._road_nav_status = "FAILED_TO_SEND"
            self._road_nav_done = True
            self._road_nav_success = False
            return

        if not goal_handle.accepted:
            self.get_logger().error("NavigateThroughPoses goal was rejected.")
            self._road_nav_status = "REJECTED"
            self._road_nav_done = True
            self._road_nav_success = False
            return

        self._road_nav_goal_handle = goal_handle
        self._road_nav_status = "ACCEPTED"
        self.get_logger().info("NavigateThroughPoses goal accepted.")
        self._road_nav_result_future = goal_handle.get_result_async()
        self._road_nav_result_future.add_done_callback(
            self._road_nav_result_done_callback)

    def _road_nav_cancel_done_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                f"NavigateThroughPoses cancel response: goals_canceling={len(response.goals_canceling)}"
            )
        except Exception as e:
            self.get_logger().error(f"NavigateThroughPoses cancel failed: {e}")

    def _road_nav_result_done_callback(self, future):
        try:
            response = future.result()
            self._road_nav_status = self.ROAD_NAV_STATUS_NAMES.get(
                response.status, "UNKNOWN")
            self._road_nav_success = self._road_nav_status == "SUCCEEDED"
            self._last_road_nav_result_message = self._road_nav_status
            self._road_nav_done = True
            if self._road_nav_success:
                self.get_logger().info("NavigateThroughPoses succeeded.")
            else:
                self.get_logger().error(
                    f"NavigateThroughPoses finished with status={self._road_nav_status}({response.status})"
                )
        except Exception as e:
            self.get_logger().error(f"NavigateThroughPoses result error: {e}")
            self._road_nav_status = "UNKNOWN"
            self._road_nav_success = False
            self._road_nav_done = True

    def _path_nav_feedback_callback(self, feedback_msg, task_id):
        if task_id != self._path_nav_task_id:
            self.get_logger().debug(
                f"Ignoring feedback for old path navigation task {task_id} "
                f"(current: {self._path_nav_task_id})")
            return

        feedback = feedback_msg.feedback
        self._last_path_nav_feedback = feedback
        if self._path_nav_status != "CANCELING":
            self._path_nav_status = "EXECUTING"
        self.get_logger().info(
            "NavigateThroughPoints feedback: "
            f"status={feedback.current_status}, "
            f"point={feedback.current_point_index}/{feedback.total_points}")

    def _path_nav_goal_done_callback(self, future, task_id):
        if task_id != self._path_nav_task_id:
            self.get_logger().debug(
                f"Ignoring goal response for old path navigation task {task_id} "
                f"(current: {self._path_nav_task_id})")
            return

        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(
                f"NavigateThroughPoints goal send failed: {e}")
            self._path_nav_status = "FAILED_TO_SEND"
            self._path_nav_done = True
            self._path_nav_success = False
            return

        if not goal_handle.accepted:
            self.get_logger().error("NavigateThroughPoints goal was rejected.")
            self._path_nav_status = "REJECTED"
            self._path_nav_done = True
            self._path_nav_success = False
            return

        self._path_nav_goal_handle = goal_handle
        self._path_nav_status = "ACCEPTED"
        self.get_logger().info("NavigateThroughPoints goal accepted.")
        self._path_nav_result_future = goal_handle.get_result_async()
        self._path_nav_result_future.add_done_callback(
            lambda future: self._path_nav_result_done_callback(
                future, task_id))

    def _path_nav_cancel_done_callback(self, future, task_id):
        if task_id != self._path_nav_task_id:
            self.get_logger().debug(
                f"Ignoring cancel response for old path navigation task {task_id} "
                f"(current: {self._path_nav_task_id})")
            return

        try:
            response = future.result()
            self.get_logger().info(
                f"NavigateThroughPoints cancel response: "
                f"goals_canceling={len(response.goals_canceling)}")
        except Exception as e:
            self.get_logger().error(
                f"NavigateThroughPoints cancel failed: {e}")

    def _path_nav_result_done_callback(self, future, task_id):
        if task_id != self._path_nav_task_id:
            self.get_logger().debug(
                f"Ignoring result for old path navigation task {task_id} "
                f"(current: {self._path_nav_task_id})")
            return

        try:
            response = future.result()
            result = response.result
            self._path_nav_status = self.PATH_NAV_STATUS_NAMES.get(
                response.status, "UNKNOWN")
            self._path_nav_success = (self._path_nav_status == "SUCCEEDED"
                                      and bool(result.success))
            self._last_path_nav_result_message = result.message
            self._path_nav_done = True
            self._path_nav_goal_handle = None

            if self._path_nav_success:
                self.get_logger().info(
                    f"NavigateThroughPoints succeeded: {result.message}")
            else:
                self.get_logger().error(
                    f"NavigateThroughPoints finished with "
                    f"status={self._path_nav_status}({response.status}): "
                    f"{result.message}")
        except Exception as e:
            self.get_logger().error(
                f"NavigateThroughPoints result error: {e}")
            self._path_nav_status = "UNKNOWN"
            self._path_nav_success = False
            self._path_nav_done = True
            self._path_nav_goal_handle = None

    def get_control_status(self, consume: bool = True) -> str:
        """
        获取当前底盘控制 action 状态，语义与 get_nav_state() 保持一致。

        Args:
            consume (bool): 兼容旧接口参数，当前不消费终态。

        Returns:
            str:
                - "IDLE"
                - "CONTROL"
                - "SUCCEEDED"
                - "FAILED"
                - "CANCELLED"
                - action feedback 中的 current_status
        """
        with self._control_lock:
            status = self._control_status

        self._log_control_status_once(status)
        return status

    def refresh_control_status(self) -> bool:
        """
        兼容旧接口：返回是否存在有效本地状态。

        Returns:
            bool: 当前是否有可读状态。
        """
        with self._control_lock:
            return self._control_status != "UNKNOWN"

    def cancel_chassis_control(self) -> bool:
        """取消当前底盘控制 action（非阻塞）。

        返回:
            bool: 如果取消请求成功发出则为 True。
        """
        with self._control_lock:
            goal_handle = self._control_goal_handle
            current_task_id = self._control_task_id

        if goal_handle is None:
            self.get_logger().warning("No active control goal to cancel.")
            return False

        try:
            cancel_future = goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda future: self._on_control_cancel_response(
                    future, current_task_id))
            self.get_logger().info("Cancel control request sent.")
            return True
        except Exception as e:
            self.get_logger().error(
                f"Failed to send cancel control request: {e}")
            return False

    def set_chassis_control_position(self, position) -> bool:
        """发送一个底盘控制目标点，精度较高。

        Args:
            position (list/np.ndarray): 目标位置 [x, 0, yaw]，yaw单位为度。
                注意：x和yaw不能同时非零。

        Returns:
            bool: 如果目标发送成功则为 True。
        """
        if position[0] != 0.0 and position[2] != 0:
            self.get_logger().info(
                "illegal position, x and yaw should not be set together.")
            return False

        if self._set_ctrl_client is None:
            self.get_logger().error(
                "PreciseControl action client not initialized.")
            return False

        if not self._set_ctrl_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error(
                "PreciseControl action server not available.")
            return False

        control_goal = PreciseControl.Goal()
        control_goal.target_pose = PoseStamped()
        control_goal.target_pose.header.frame_id = "base_link"
        control_goal.target_pose.pose.position.x = float(position[0])
        control_goal.target_pose.pose.position.y = 0.0
        control_goal.target_pose.pose.position.z = 0.0

        rotation = R.from_euler('xyz', [0, 0, position[2]], degrees=True)
        quat_arry = rotation.as_quat()

        control_goal.target_pose.pose.orientation.x = quat_arry[0]
        control_goal.target_pose.pose.orientation.y = quat_arry[1]
        control_goal.target_pose.pose.orientation.z = quat_arry[2]
        control_goal.target_pose.pose.orientation.w = quat_arry[3]

        with self._control_lock:
            self._control_task_id += 1
            current_task_id = self._control_task_id
            self._control_goal_handle = None
            self._control_result = None
            self._control_status = "CONTROL"
            self._last_control_status = "CONTROL"
            self._last_control_status_success = True
            self._last_control_status_time = time.time()

        try:
            send_future = self._set_ctrl_client.send_goal_async(
                control_goal,
                feedback_callback=lambda feedback_msg: self.
                _on_control_feedback(feedback_msg, current_task_id))
            send_future.add_done_callback(
                lambda future: self._on_control_goal_response(
                    future, current_task_id))
        except Exception as e:
            self._set_control_status("FAILED", current_task_id)
            self._clear_control_goal(current_task_id)
            self.get_logger().error(f"Failed to send precise control goal: {e}")
            return False
        self.get_logger().info(
            f"Precise control goal sent: x={position[0]}, yaw={position[2]}deg, frame=base_link"
        )
        return True

    def set_goal_tolerance(self, xy_tolerance: float,
                           yaw_tolerance: float) -> bool:
        """
        设置导航到达精度
        yaw_tolerance 单位：弧度（Nav2 内部用 rad）
        """

        lon_ok = self._set_param(self._controller_set_param_client,
                                 self._controller_get_param_client,
                                 'goal_checker.lon_goal_tolerance',
                                 xy_tolerance)
        lat_ok = self._set_param(self._controller_set_param_client,
                                 self._controller_get_param_client,
                                 'goal_checker.lat_goal_tolerance',
                                 xy_tolerance)
        yaw_ok = self._set_param(self._controller_set_param_client,
                                 self._controller_get_param_client,
                                 'goal_checker.yaw_goal_tolerance',
                                 yaw_tolerance)

        self.get_logger().info(
            f"Set goal tolerance: xy={xy_tolerance}, yaw={yaw_tolerance}")

        return lon_ok and lat_ok and yaw_ok

    def set_footprint(self, vertices: list) -> bool:
        """
        设置机器人 footprint
        vertices: [[x1,y1], [x2,y2], [x3,y3], [x4,y4]]
        """

        if len(vertices) != 4:
            self.get_logger().error("Footprint must have 4 vertices")
            return False

        footprint_str = "[" + ",".join([f"[{v[0]},{v[1]}]" for v in vertices
                                       ]) + "]"

        # local
        local_success = self._set_param(self._local_costmap_set_param_client,
                                        self._local_costmap_get_param_client,
                                        'footprint', footprint_str)

        # global
        global_success = self._set_param(self._global_costmap_set_param_client,
                                         self._global_costmap_get_param_client,
                                         'footprint', footprint_str)

        if local_success and global_success:
            self.get_logger().info(
                f"Successfully set and verified footprint: {footprint_str}")
            return True
        else:
            self.get_logger().error(f"Failed to set footprint.")
            return False

    def set_inflation_radius(self, radius: float) -> bool:
        """
        设置膨胀层半径（local + global）
        """

        local_success = self._set_param(self._local_costmap_set_param_client,
                                        self._local_costmap_get_param_client,
                                        'inflation_layer.inflation_radius',
                                        radius)

        global_success = self._set_param(self._global_costmap_set_param_client,
                                         self._global_costmap_get_param_client,
                                         'inflation_layer.inflation_radius',
                                         radius)

        if local_success and global_success:
            self.get_logger().info(
                f"Successfully set and verified inflation radius to {radius}")
            return True
        else:
            self.get_logger().error(f"Failed to set inflation radius.")
            return False

    def set_follow_velocity_limits(self, vx_max: float, wz_max: float):
        """
        同时设置 FollowPath 线速度 & 角速度上限
        """

        vx_success = self._set_param(self._controller_set_param_client,
                                     self._controller_get_param_client,
                                     'FollowPath.vx_max', vx_max)

        wz_success = self._set_param(self._controller_set_param_client,
                                     self._controller_get_param_client,
                                     'FollowPath.wz_max', wz_max)

        if vx_success and wz_success:
            self.get_logger().info(
                f"Successfully set and verified FollowPath limits: vx_max={vx_max}, wz_max={wz_max}"
            )
            return True
        else:
            self.get_logger().error(f"Failed to set FollowPath limits.")
            return False

    def spin_once(self, timeout_sec: float = 0.1):
        r"""Spin the node once with a timeout of 0.1 sec."""
        rclpy.spin_once(self, timeout_sec=timeout_sec)

    def _spin_once_if_not_executed(self, timeout_sec: float = 0.01):
        try:
            self.spin_once(timeout_sec=timeout_sec)
        except RuntimeError as e:
            self.get_logger().debug(f"spin_once skipped: {e}")

    def spin(self):
        r"""Continuously spin the node while rclpy is running."""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def _on_nav_goal_response(self, future, task_id):
        """Callback for navigation goal response.
        """
        # 检查是否是当前任务
        if task_id != self._nav_task_id:
            self.get_logger().warning(
                f"Ignoring goal response for old task {task_id} (current: {self._nav_task_id})"
            )
            return

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected.")
            self._nav_status = "FAILED"
            return

        self.get_logger().info("Goal accepted.")
        self._nav_goal_handle = goal_handle

        # 异步等待结果
        self._nav_result_future = goal_handle.get_result_async()
        self._nav_result_future.add_done_callback(
            lambda future: self._on_nav_result(future, task_id))

    def _on_nav_result(self, future, task_id):
        """Callback for navigation result.
        """
        # 检查是否是当前任务
        if task_id != self._nav_task_id:
            self.get_logger().warning(
                f"Ignoring nav result for old task {task_id} (current: {self._nav_task_id})"
            )
            return

        try:
            result = future.result()
            self._nav_result = result

            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self._nav_status = "SUCCEEDED"
                self.get_logger().info("Goal reached successfully.")
            elif result.status == GoalStatus.STATUS_CANCELED:
                self._nav_status = "CANCELLED"
                self.get_logger().info("Goal was cancelled.")
            else:
                self._nav_status = "FAILED"
                self.get_logger().error(
                    f"Goal failed with status {result.status}")
        except Exception as e:
            self._nav_status = "ERROR"
            self.get_logger().error(
                f"Error getting navigation result: {str(e)}")

    def _on_nav_cancel_response(self, future, task_id):
        """Callback for navigation cancel response.
        """
        # 检查是否是当前任务
        if task_id != self._nav_task_id:
            self.get_logger().warning(
                f"Ignoring cancel response for old task {task_id} (current: {self._nav_task_id})"
            )
            return

        try:
            _ = future.result()
            self._nav_status = "CANCELLED"
            self._nav_goal_handle = None
            self.get_logger().info("Navigation goal cancelled.")
        except Exception as e:
            self.get_logger().error(f"Error in cancel response: {str(e)}")

    def _on_control_goal_response(self, future, task_id):
        """Callback for precise control goal response."""
        with self._control_lock:
            if task_id != self._control_task_id:
                return

        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self._set_control_status("FAILED", task_id)
                self._clear_control_goal(task_id)
                self.get_logger().error("Precise control goal was rejected.")
                return

            with self._control_lock:
                if task_id != self._control_task_id:
                    return
                self._control_goal_handle = goal_handle

            self.get_logger().info("Precise control goal accepted.")
            result_future = goal_handle.get_result_async()
            with self._control_lock:
                if task_id == self._control_task_id:
                    self._control_result_future = result_future
            result_future.add_done_callback(
                lambda result_future: self._on_control_result(
                    result_future, task_id))
        except Exception as e:
            self._set_control_status("FAILED", task_id)
            self._clear_control_goal(task_id)
            self.get_logger().error(
                f"Error in precise control goal response: {e}")

    def _on_control_feedback(self, feedback_msg, task_id):
        """Callback for precise control feedback."""
        with self._control_lock:
            if task_id != self._control_task_id:
                return
            status = feedback_msg.feedback.current_status

        if status:
            self._set_control_status(status, task_id)
            if status in self._control_terminal_states:
                self._clear_control_goal(task_id)

    def _on_control_result(self, future, task_id):
        """Callback for precise control result."""
        with self._control_lock:
            if task_id != self._control_task_id:
                return

        try:
            result = future.result()
            with self._control_lock:
                if task_id != self._control_task_id:
                    return
                self._control_result = result

            if result.status == GoalStatus.STATUS_CANCELED:
                self._set_control_status("CANCELLED", task_id)
                self._clear_control_goal(task_id)
                self.get_logger().info("Precise control goal was cancelled.")
            elif result.status == GoalStatus.STATUS_SUCCEEDED and result.result.success:
                self._set_control_status("SUCCEEDED", task_id)
                self._clear_control_goal(task_id)
                self.get_logger().info(
                    f"Precise control succeeded: {result.result.message}")
            else:
                self._set_control_status("FAILED", task_id)
                self._clear_control_goal(task_id)
                self.get_logger().error(
                    f"Precise control failed with status {result.status}: "
                    f"{result.result.message}")
        except Exception as e:
            self._set_control_status("FAILED", task_id)
            self._clear_control_goal(task_id)
            self.get_logger().error(
                f"Error getting precise control result: {e}")

    def _on_control_cancel_response(self, future, task_id):
        """Callback for precise control cancel response."""
        with self._control_lock:
            if task_id != self._control_task_id:
                return

        try:
            future.result()
        except Exception as e:
            self.get_logger().error(
                f"Error in precise control cancel response: {e}")
            return

        self._set_control_status("CANCELLING", task_id)
        self.get_logger().info("Precise control goal cancel request accepted.")

    def _set_control_status(self, status: str, task_id: int = None):
        with self._control_lock:
            if task_id is not None and task_id != self._control_task_id:
                return
            self._control_status = status
            self._last_control_status = status
            self._last_control_status_success = status != "UNKNOWN"
            self._last_control_status_time = time.time()
            should_log = status != self._last_logged_control_status
            if should_log:
                self._last_logged_control_status = status

        if should_log:
            self.get_logger().info(f"Precise control status: {status}")

    def _log_control_status_once(self, status: str):
        with self._control_lock:
            if status == self._last_logged_control_status:
                return
            self._last_logged_control_status = status
        self.get_logger().info(f"Precise control status: {status}")

    def _clear_control_goal(self, task_id: int = None):
        with self._control_lock:
            if task_id is not None and task_id != self._control_task_id:
                return
            self._control_goal_handle = None
            self._control_result_future = None

    def _get_param(self, client, name: str):
        if not client.wait_for_service(timeout_sec=0.5):
            self.get_logger().error(
                f"Get parameter service '{client.srv_name}' not available")
            return None

        req = GetParameters.Request()
        req.names = [name]
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

        if not future.done():
            self.get_logger().error(
                f"Failed to get parameter '{name}': service call timed out.")
            return None

        response = future.result()
        if not response.values:
            self.get_logger().error(
                f"Failed to get parameter '{name}': parameter not found.")
            return None

        return response.values[0]

    def _set_param(self, set_client, get_client, name: str, value):
        if not set_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().error(
                f"Set parameter service '{set_client.srv_name}' not available")
            return False

        # ✅ 用 rclpy.parameter.Parameter
        param = Parameter(name=name, value=value)

        req = SetParameters.Request()
        req.parameters = [param.to_parameter_msg()]

        future = set_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

        if not future.done():
            self.get_logger().error(f"Timeout setting parameter '{name}'")
            return False

        result = future.result().results[0]
        if not result.successful:
            self.get_logger().error(
                f"Failed to set parameter '{name}': {result.reason}")
            return False

        self.get_logger().info(
            f"Parameter '{name}' set to '{value}', verifying...")

        # ---------- 校验 ----------
        time.sleep(0.1)
        get_param_value = self._get_param(get_client, name)
        if get_param_value is None:
            self.get_logger().error(
                f"Failed to get parameter '{name}' for verification")
            return False

        retrieved_value = None

        if get_param_value.type == ParameterType.PARAMETER_DOUBLE:
            retrieved_value = get_param_value.double_value
            # 浮点数用近似比较
            if not math.isclose(retrieved_value, float(value), rel_tol=1e-6):
                self.get_logger().error(
                    f"Parameter '{name}' verification failed! "
                    f"Expected: {value}, Got: {retrieved_value}")
                return False

        elif get_param_value.type == ParameterType.PARAMETER_INTEGER:
            retrieved_value = get_param_value.integer_value
            if retrieved_value != int(value):
                self.get_logger().error(
                    f"Parameter '{name}' verification failed! "
                    f"Expected: {value}, Got: {retrieved_value}")
                return False

        elif get_param_value.type == ParameterType.PARAMETER_BOOL:
            retrieved_value = get_param_value.bool_value
            if retrieved_value != bool(value):
                self.get_logger().error(
                    f"Parameter '{name}' verification failed! "
                    f"Expected: {value}, Got: {retrieved_value}")
                return False

        elif get_param_value.type == ParameterType.PARAMETER_STRING:
            retrieved_value = get_param_value.string_value
            if retrieved_value != str(value):
                self.get_logger().error(
                    f"Parameter '{name}' verification failed! "
                    f"Expected: {value}, Got: {retrieved_value}")
                return False

        else:
            self.get_logger().error(
                f"Unsupported parameter type {get_param_value.type} for '{name}'"
            )
            return False

        self.get_logger().info(f"Parameter '{name}' verified successfully.")
        return True
