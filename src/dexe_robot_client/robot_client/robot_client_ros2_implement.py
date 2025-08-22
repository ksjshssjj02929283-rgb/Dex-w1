# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import numpy as np
import json
import time
from typing import Any, Optional
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from robot_server_interfaces.srv import GetProperty, SetProperty
from robot_server_interfaces.action import MoveRobot
from robot_server_interfaces.msg import StreamingCommand

from .data_structure import *
from .robot_client import RobotClient
from ..common import get_logger


class RobotClientRos2(Node, RobotClient):
    """ros2 client universal interfaces"""

    def __init__(self, 
                 is_sim: bool = False, 
                 timeout_sec: float = 3.0):
        self.logger = get_logger()
        if is_sim:
            self.node_name = "robot_client_sim"
        else:
            self.node_name = "robot_client_real"
        Node.__init__(self, self.node_name)
        RobotClient.__init__(self, is_sim=is_sim)
        self._last_msg_time = None
        self._connection_timeout = 1.0

        self._init_subscriber()
        rclpy.spin_once(self, timeout_sec = 1.)
        self._init_publisher()
        self._init_service_client()
        self._init_action_client()

        # 创建定时器检查连接状态
        self.create_timer(0.1, self._check_connection_status)

    def _check_connection_status(self):
        """定时检查消息是否超时"""
        if self._last_msg_time is not None:
            cur_time = self.get_clock().now().nanoseconds / 1e9
            if (cur_time - self._last_msg_time) > self._connection_timeout:
                self.status = None
                self.joint_position = None
                self.ee_pose = None
                self.actual_torque = None
                self.command_torque = None
                self._last_msg_time = None
                self.latest_queued_id = None
                self.latest_finished_id = None
                self.latest_errordrop_id = None

    def is_connected(self, timeout_sec: float = 0.5):
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if self._last_msg_time is not None:
                return True
            self.wait_second(0.01)
        return False
    
    """ ---------------------------------------------------------------------------
    implement interfaces of base class RobotClient
    实现RobotClient基类的abstract接口
    ----------------------------------------------------------------------------"""

    def connect_robot(self, timeout_sec: float = 0.5, ip: str = None) -> bool:
        return self.is_connected(timeout_sec)
    
    def wait_second(self, wait_second: float):
        """Wait for a specified time in seconds."""
        start_time = time.time()
        while time.time() - start_time < wait_second:
            time.sleep(0.001)

    def wait_request_finish(
        self, request_id: Optional[int] = None, timeout_sec: int = 30
    ) -> bool:
        if not request_id:
            request_id = self.latest_sent_id
        if not self.connect_robot():
            self.logger.error(f"No connection to robot")
            return False
        if request_id < 0:
            self.logger.error(f"Waiting for request id: {request_id}, no valid request")
            return False
        # 如果未完成到指定request id则阻塞等待, 如果超时则返回
        self.logger.info(
            f"Waiting for request id: {request_id}, latest finished id: {self.latest_finished_id}"
        )
        start_time = last_log_time = time.time()
        while self.latest_finished_id < request_id:
            if self.status == Status.Error:
                self.logger.error(f"Waiting for request id failed: {request_id}, robot status is Error, please check and clear error!")
                return False
            self.wait_second(0.001)
            if not self.connect_robot():
                self.logger.error(f"No connection to robot")
                return False
            now = time.time()
            if now - last_log_time >= 3.0:
                self.logger.info(
                    f"Waiting for request id: {request_id}, latest finished id: {self.latest_finished_id}"
                )
                last_log_time = now
            if now - start_time > timeout_sec:
                self.logger.warning(
                    f"Wait_request_finish Timeout, timeout_sec: {timeout_sec}"
                )
                return False
        self.logger.info(
            f"request id: {request_id}, wait move finished"
        )
        return True

    def send_set_property_request(self, type: PropertyType, value: Any) -> int:
        if not self._srv_set_property.wait_for_service(timeout_sec=self._connection_timeout):
            self.logger.error("Ros2 set_property service server not available")
            self.latest_sent_id = -1
            return self.latest_sent_id
        req = SetProperty.Request()
        req.property_type = type.name
        req.value = value_to_string(value)
        future = self._srv_set_property.call_async(req)
        start_time = time.time()
        while not future.done():
            self.wait_second(0.01)
            if time.time() - start_time > self._connection_timeout:
                self.logger.error("Service return timeout, please check ros2 spin!")
                self.latest_sent_id = -1
                return self.latest_sent_id
        if future.result() is not None:
            self.logger.info(
                f"Property request id: {future.result().request_id} accepted, {type}"
            )
            self.latest_sent_id = future.result().request_id
            return self.latest_sent_id

    def send_move_robot_request(self, request: MoveRequest) -> int:
        # 检查动作服务器是否可用
        if not self._action_move_robot.wait_for_server(timeout_sec=self._connection_timeout):
            self.logger.error("Ros2 action server not available!")
            return False
        # 配置目标
        goal_msg = MoveRobot.Goal()
        goal_msg.motion_group = request.motion_group.name
        goal_msg.type = request.type.name

        if self.status == Status.Error:
            self.logger.error(f"Send {request.type.name} requset failed: Robot status is Error, please check and clear error!")
            self.latest_sent_id = -1
            return self.latest_sent_id

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
        start_time = time.time()
        while not future.done():
            self.wait_second(0.01)
            if time.time() - start_time > self._connection_timeout:
                self.logger.error("Action call timeout!")
                self.latest_sent_id = -1
                return self.latest_sent_id
        if future.result() is not None:
            goal_handle = future.result()
            if goal_handle is None:
                self.logger.error(f"Movement request failed to get goal handle!")
                self.latest_sent_id = -1
                return self.latest_sent_id
            if not goal_handle.accepted:
                self.logger.error(f"Movement request rejected!")
                self.latest_sent_id = -1
                return self.latest_sent_id

            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(self._get_result_move_robot_callback)

            # 等待 feedback request id
            start_time = time.time()
            while not self._move_request_id_returned:
                self.wait_second(0.01)
                if time.time() - start_time > 1.0:
                    self.logger.error("Timeout waiting for request_id feedback!")
                    self.latest_sent_id = -1
                    return self.latest_sent_id

            target_str = (
                str(np.round(request.value, 4)) if request.value is not None else "None"
            )
            self.logger.info(
                f"Movement request_id: {self.latest_sent_id} {request.motion_group.name} {request.type.name} accepted, target: {target_str}"
            )
            self._move_request_id_returned = False
            return self.latest_sent_id
        else:
            self.latest_sent_id = -1
            return self.latest_sent_id

    def send_streaming_command(
        self,
        position: list,
        streaming_finished: bool = False,
    ) -> bool:
        position_msg = StreamingCommand()
        if self.latest_sent_id < 0:
            return False
        position_msg.request_id = self.latest_sent_id
        position_msg.streaming_finished = streaming_finished
        position_msg.position = [float(x) for x in position]
        # self.logger.debug(
        #     f"Publishing streaming command: {position_msg.position}, streaming finished: {streaming_finished}"
        # )
        self._publisher_streaming_command.publish(position_msg)
        return True

    def close_client(self):
        pass

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
        # 通知主线程
        self._move_request_id_returned = True

    def _get_result_move_robot_callback(self, future):
        result = future.result().result
        if result is None:
            self.logger.error("MoveRobot action failed or was cancelled")
            return
        # 处理结果, 如果成功则更新last_finished_id
        if result.success:
            self.logger.info(
                f"Movement request_id: {result.request_id} executed, result: {result.success}"
            )
        else:
            self.logger.error(f"Movement request_id: {result.request_id} failed, please check and clear error")

    """ ---------------------------------------------------------------------------
    init subscriber
    ----------------------------------------------------------------------------"""

    def _init_subscriber(self):
        if self.is_sim:
            self.create_subscription(
                String,
                "/feedback/robot_server_state_sim",
                self._subscriber_robot_server_state_callback,
                rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)
            )
        else:
            self.create_subscription(
                String,
                "/feedback/robot_server_state",
                self._subscriber_robot_server_state_callback,
                rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)
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
        self._last_msg_time = self.get_clock().now().nanoseconds / 1e9
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
        if "joint_velocity" in state:
            self.joint_velocity = state["joint_velocity"]
        if "ee_pose" in state:
            self.ee_pose = state["ee_pose"]
        if "actual_arm_torque" in state:
            self.actual_arm_torque = state["actual_arm_torque"]
        if "command_arm_torque" in state:
            self.command_arm_torque = state["command_arm_torque"]