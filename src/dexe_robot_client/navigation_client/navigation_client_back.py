import time
import numpy as np
import math
from abc import ABC, abstractmethod
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from scipy.spatial.transform import Rotation as R
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger
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
        self._set_ctrl_publisher = None
        self._get_pos_subscriber = None
        self._set_chassis_charging_client = None
        self._cancel_control_client = None
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
        self._last_control_status = None
        self._last_control_status_success = False
        self._control_status_requesting = False
        self._last_control_status_time = None
        self._control_status_timeout = 1.0

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
        self.spin_once()
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
        self.spin_once()

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

    def set_chassis_auto_charging(self) -> bool:
        """调用回充服务"""
        if not self._set_chassis_charging_client.wait_for_service(
                timeout_sec=1.0):
            return False

        request = Trigger.Request()
        future = self._set_chassis_charging_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.done():
            try:
                response = future.result()
                self.get_logger().info(
                    f"Cancel control response: {response.message}")
                return response.success
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")
                return False
        else:
            self.get_logger().error("Service call timed out.")
            return False

    def get_control_status(self, consume: bool = True) -> str:
        """
        获取当前底盘控制状态（供上层业务轮询调用）。

        每次调用会先处理已到达的异步响应和缓存，再触发一次异步状态刷新。
        终态可以按需在返回后从本地缓存消费掉，避免上层重复读取同一终态。

        Args:
            consume (bool): 如果为 True，当返回 SUCCEED/SUCCEEDED/FAILED 等终态后，
                会重置本地缓存为 IDLE。
        Returns:
            str:
                - "IDLE"
                - "CONTROL"
                - "SUCCEED"
                - "SUCCEEDED"
                - "FAILED"
                - "CANCEL CONTROL"
                - "UNKNOWN"
        """
        self._spin_once_if_not_executed(timeout_sec=0.01)

        if self._last_control_status_success and self._last_control_status_time:
            if (time.time() - self._last_control_status_time
               ) <= self._control_status_timeout:
                status = self._last_control_status
                if consume and status in [
                        "SUCCEED", "SUCCEEDED", "FAILED", "CANCEL CONTROL",
                        "CANCELLED"
                ]:
                    self._last_control_status = "IDLE"
                self.refresh_control_status()
                return status

        self.refresh_control_status()
        return "UNKNOWN"

    def refresh_control_status(self) -> bool:
        """
        主动向底层控制节点请求一次状态（异步，带 in-flight 防抖）。

        兼容旧接口；不会被定时器自动调用。

        Returns:
            bool: 请求是否成功发出
        """
        if self._control_status_requesting:
            return False

        if (not hasattr(self, '_get_status_client')
                or not self._get_status_client.wait_for_service(
                    timeout_sec=0.2)):
            return False

        request = Trigger.Request()
        try:
            self._control_status_requesting = True
            future = self._get_status_client.call_async(request)
            future.add_done_callback(self._on_get_control_status_response)
            return True
        except Exception as e:
            self._control_status_requesting = False
            self.get_logger().error(f"Failed to refresh control status: {e}")
            return False

    def cancel_chassis_control(self) -> bool:
        """请求取消当前的控制任务。

        返回:
            bool: 如果服务调用成功则为 True。
        """
        if not hasattr(
                self, '_cancel_control_client'
        ) or not self._cancel_control_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Cancel control service not available.")
            return False

        # 非阻塞调用：发送请求后立即返回，结果由回调处理
        request = Trigger.Request()
        try:
            future = self._cancel_control_client.call_async(request)
            future.add_done_callback(self._on_cancel_control_response)
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
        chassis_position = PoseStamped()
        chassis_position.header.frame_id = "base_link"
        chassis_position.pose.position.x = position[0]
        chassis_position.pose.position.y = 0.0
        chassis_position.pose.position.z = 0.0

        rotation = R.from_euler('xyz', [0, 0, position[2]], degrees=True)
        quat_arry = rotation.as_quat()

        chassis_position.pose.orientation.x = quat_arry[0]
        chassis_position.pose.orientation.y = quat_arry[1]
        chassis_position.pose.orientation.z = quat_arry[2]
        chassis_position.pose.orientation.w = quat_arry[3]
        if position[0] != 0.0 and position[2] != 0:
            self.get_logger().info(
                "illegal position, x and yaw should not be set together.")
            return False

        self._set_ctrl_publisher.publish(chassis_position)
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

    def spin(self):
        r"""Continuously spin the node while rclpy is running."""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def _spin_once_if_not_executed(self, timeout_sec: float = 0.01):
        """独立使用节点时处理一次回调；已有 executor 时由后台线程处理。"""
        if getattr(self, "executor", None) is not None:
            return
        try:
            self.spin_once(timeout_sec=timeout_sec)
        except Exception as e:
            self.get_logger().debug(f"spin_once skipped: {e}")

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

    def _on_cancel_control_response(self, future):
        """Callback for cancel control service response (non-blocking)."""
        try:
            response = future.result()
            self.get_logger().info(
                f"Cancel control response: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Error in cancel control response: {e}")

    def _on_get_control_status_response(self, future):
        """Callback for get_control_status async response."""
        self._control_status_requesting = False

        try:
            response = future.result()
            self._last_control_status_time = time.time()
            self._last_control_status = response.message
            self._last_control_status_success = bool(response.success)
        except Exception as e:
            self._last_control_status_success = False
            self.get_logger().error(
                f"Error in get control status response: {e}")

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
