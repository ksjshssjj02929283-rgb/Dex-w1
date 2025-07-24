# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import numpy as np
import json
import rclpy
import time
from typing import Any, Optional
from rclpy.node import Node
from rclpy.action import ActionClient
from .data_structure import *
from .robot_client import RobotClient
from robot_server_interfaces.srv import GetProperty, SetProperty
from robot_server_interfaces.action import MoveRobot
from robot_server_interfaces.msg import StreamingCommand
from std_msgs.msg import String


class RobotClientRos2(Node, RobotClient):
    """ros2 client universal interfaces"""

    def __init__(self, is_sim: bool = False, timeout_sec: float = 3.0):
        rclpy.init()
        Node.__init__(self, "robot_client")
        RobotClient.__init__(self, is_sim=is_sim)
        self._init_subscriber()
        self._init_publisher()
        self._init_service_client()
        self._init_action_client()
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            rclpy.spin_once(self, timeout_sec = 0.1)
            if self.status != Status.Init:
                self.get_logger().info(f"robot client init successed")
                break
        else:
            self.get_logger().error("robot client init failed: connect to server timeout")

    """ ---------------------------------------------------------------------------
    implement interfaces of base class RobotClient
    实现RobotClient基类的abstract接口
    ----------------------------------------------------------------------------"""
    def refresh(self):
        rclpy.spin_once(self, timeout_sec = 0.001)
        rclpy.spin_once(self, timeout_sec = 0.001)

    def wait_second(self, wait_second: float):
        """Wait for a specified time in seconds."""
        start_time = time.time()
        while time.time() - start_time < wait_second:
            rclpy.spin_once(self, timeout_sec=0.001)
        # self.get_logger().info(f"Waited for {wait_second} seconds")

    def wait_request_finish(
        self, request_id: Optional[int] = None, timeout_sec: int = 100
    ) -> bool:
        if not request_id:
            request_id = self.latest_sent_id
        # 如果未完成到指定request id则阻塞等待, 每两秒打印一次信息, 如果超时则返回
        self.get_logger().info(
            "Wait_request_finish_sync(): Executing received request, waiting for finishing"
        )

        start_time = time.time()
        last_log_time = start_time
        while self.latest_finished_id < request_id:
            rclpy.spin_once(self)
            now = time.time()
            if now - last_log_time >= 2.0:
                self.get_logger().info(
                    f"----Waiting for request id: {request_id} finish, "
                    f"latest finished id: {self.latest_finished_id}"
                )
                last_log_time = now
            if now - start_time > timeout_sec:
                self.get_logger().warn(
                    f"Wait_request_finish_sync(): Timeout, timeout_sec: {timeout_sec}"
                )
                return False
        self.get_logger().info(
            f"----Wait_move_finish: wait for id: {request_id}, "
            f"latest finished id: {self.latest_finished_id}, move finished"
        )
        return True

    def send_set_property_request(self, type: PropertyType, value: Any) -> int:
        if not self._srv_set_property.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("Ros2 set_property service not available")
            return False

        req = SetProperty.Request()
        req.property_type = type.name
        req.value = value_to_string(value)
        future = self._srv_set_property.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.latest_sent_id = future.result().request_id
        self.get_logger().info(
            f"Property request id: {future.result().request_id} accepted,  type: {type}"
        )
        return future.result().request_id

    def send_move_robot_request(self, request: MoveRequest) -> int:
        # 检查动作服务器是否可用
        if not self._action_move_robot.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Ros2 action server not available!")
            # return False  todo

        # 配置目标
        goal_msg = MoveRobot.Goal()
        goal_msg.motion_group = request.motion_group.name
        goal_msg.type = request.type.name

        if request.value is not None:
            goal_msg.target = [float(x) for x in request.value]

        config_dict = {}
        if request.reference_frame is not None:
            config_dict["reference_frame"] = request.reference_frame.name
        if request.speed is not None:
            config_dict["speed"] = request.speed
        if request.kwargs is not None:
            config_dict.update(request.kwargs)
        if config_dict:
            goal_msg.config = json.dumps(config_dict)
        else:
            goal_msg.config = ""

        future = self._action_move_robot.send_goal_async(
            goal_msg, feedback_callback=self._feedback_move_robot_callback
        )
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if goal_handle is None:
            self.get_logger().error(f"Movement request failed to get goal handle!")
            return
        if not goal_handle.accepted:
            self.get_logger().error(f"Movement request rejected!")
            return False

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._get_result_move_robot_callback)

        # 等待 feedback request id
        start_time = time.time()
        while not self._move_request_id_returned:
            rclpy.spin_once(self, timeout_sec=0.01)
            if time.time() - start_time > 1.0:
                self.get_logger().error("Timeout waiting for request_id feedback!")
                return -1

        target_str = (
            str(np.round(request.value, 4)) if request.value is not None else "None"
        )
        self.get_logger().info(
            f"Movement request_id: {self.latest_sent_id} accepted, type: {request.type}, motion_group: {request.motion_group}, target: {target_str}"
        )
        self._move_request_id_returned = False
        return self.latest_sent_id

    def send_streaming_command(
        self,
        position: list,
        streaming_finished: bool = False,
    ) -> bool:
        position_msg = StreamingCommand()
        position_msg.request_id = self.latest_sent_id
        position_msg.streaming_finished = streaming_finished
        position_msg.position = [float(x) for x in position]
        # self.get_logger().info(
        #     f"Publishing streaming command: {position_msg.position}, streaming finished: {streaming_finished}"
        # )
        self._publisher_streaming_command.publish(position_msg)
        return True

    def close_client(self):
        rclpy.shutdown()
        self.get_logger().info("robot client is closed")

    """ ---------------------------------------------------------------------------
    init action
    ----------------------------------------------------------------------------"""

    def _init_action_client(self):
        if self.is_sim:
            self._action_move_robot = ActionClient(
                self, MoveRobot, "/control/move_robot_sim"
            )
        else:
            self._action_move_robot = ActionClient(
                self, MoveRobot, "/control/move_robot"
            )

    def _feedback_move_robot_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.latest_sent_id = feedback.request_id
        self.get_logger().info(f"movement request id feedback: {feedback.request_id}")
        # 通知主线程
        self._move_request_id_returned = True

    def _get_result_move_robot_callback(self, future):
        result = future.result().result
        if result is None:
            self.get_logger().error("MoveRobot action failed or was cancelled")
            return
        # 处理结果, 如果成功则更新last_finished_id
        if result.success:
            self.get_logger().info(
                f"Movement request_id: {result.request_id} executed, result: {result.success}"
            )
        else:
            self.get_logger().error(f"Movement request_id: {result.request_id} failed")

    """ ---------------------------------------------------------------------------
    init subscriber
    ----------------------------------------------------------------------------"""

    def _init_subscriber(self):
        if self.is_sim:
            self.create_subscription(
                String,
                "/feedback/robot_server_state_sim",
                self._subscriber_robot_server_state_callback,
                10,
            )
        else:
            self.create_subscription(
                String,
                "/feedback/robot_server_state",
                self._subscriber_robot_server_state_callback,
                10,
            )

    """ ---------------------------------------------------------------------------
    init publisher
    ----------------------------------------------------------------------------"""

    def _init_publisher(self):
        if self.is_sim:
            self._publisher_streaming_command = self.create_publisher(
                StreamingCommand, "/control/streaming_command_sim", 10
            )
        else:
            self._publisher_streaming_command = self.create_publisher(
                StreamingCommand, "/control/streaming_command", 10
            )

    """ ---------------------------------------------------------------------------
    init service interfaces
    ----------------------------------------------------------------------------"""

    def _init_service_client(self):
        if self.is_sim:
            self._srv_set_property = self.create_client(
                SetProperty, "/control/set_property_sim"
            )
        else:
            self._srv_set_property = self.create_client(
                SetProperty, "/control/set_property"
            )

    """ ---------------------------------------------------------------------------
    callbacks for subscribers
    ----------------------------------------------------------------------------"""

    def _subscriber_robot_server_state_callback(self, msg):
        state = json.loads(msg.data)

        if "status" in state:
            self.status = Status[state["status"]]
        if "latest_queued_id" in state:
            self.latest_queued_id = state["latest_queued_id"]
        if "latest_finished_id" in state:
            self.latest_finished_id = state["latest_finished_id"]
        if "latest_errordrop_id" in state:
            self.latest_errordrop_id = state["latest_errordrop_id"]
        if "joint_position" in state:
            self.joint_position = state["joint_position"]
        if "ee_pose" in state:
            self.ee_pose = state["ee_pose"]
