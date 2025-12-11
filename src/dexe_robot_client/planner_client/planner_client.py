import time
import threading
import numpy as np
import json
from abc import ABC, abstractmethod
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from std_msgs.msg import Float64MultiArray
from ..robot_client.data_structure import *
from ..end_effector_client import EndEffectorType
from robot_server_interfaces.msg import StreamingCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from collections import deque
from end_effector_interfaces.msg import EEJointControl, EEJointFeedback

from ..common import get_logger

class PlannerClient(ABC, Node):
    def __init__(self,
                 is_sim: bool = True,
                 end_effector_type: EndEffectorType = None,
                 name = "planner_client",
                 timeout_sec: float = 3.0):
        self.logger = get_logger()
        self.is_sim = is_sim
        if not isinstance(end_effector_type, EndEffectorType) and end_effector_type is not None:
            raise ValueError(f"Unsupported end effector type for {end_effector_type}")
        self.ee_type = end_effector_type
        self.node_name = name
        super().__init__(self.node_name)

        # subscribe the state feedback variables
        self._state_lock = threading.Lock()
        self.status = None
        self.joint_position = None

        self._armlee_state_lock = threading.Lock()
        self.armlee_state = None
        self._armree_state_lock = threading.Lock()
        self.armree_state = None

        self._state_last_msg_time = None
        self._armlee_last_msg_time = None
        self._armree_last_msg_time = None

        # subscribe the streaming point for dynamic planning
        self._streaming_cmd_lock = threading.Lock()
        self._streaming_last_msg_time = None
        self.streaming_position = None
        self.last_streaming_position = None
        self.streaming_vel = None
        self.is_first_streaming = True
        self.streaming_dt = 0.01  # 100Hz
        
        self.streaming_history = deque(maxlen=100) # buffer to store the last 20 streaming points

        self._connection_timeout = 1.0

        self._init_subscriber()
        self.spin_once(timeout_sec = 1.)
        self._init_publisher()

        self.create_timer(0.5, self._check_state_connection_status)
        self.create_timer(0.5, self._check_armlee_connection_status)
        self.create_timer(0.5, self._check_armree_connection_status)
        self.create_timer(0.5, self._check_streaming_connection_status)

    def _check_state_connection_status(self):
        """定时检查消息是否超时"""
        if self._state_last_msg_time is not None:
            cur_time = self.get_clock().now().nanoseconds / 1e9
            if (cur_time - self._state_last_msg_time) > self._connection_timeout:
                self.status = None
                self.joint_position = None
                self._state_last_msg_time = None

    def _check_armlee_connection_status(self):
        """定时检查消息是否超时"""
        if self._armlee_last_msg_time is not None:
            cur_time = self.get_clock().now().nanoseconds / 1e9
            if (cur_time - self._armlee_last_msg_time) > self._connection_timeout:
                self.armlee_state = None
                self._armlee_last_msg_time = None

    def _check_armree_connection_status(self):
        """定时检查消息是否超时"""
        if self._armree_last_msg_time is not None:
            cur_time = self.get_clock().now().nanoseconds / 1e9
            if (cur_time - self._armree_last_msg_time) > self._connection_timeout:
                self.armree_state = None
                self._armree_last_msg_time = None

    def _check_streaming_connection_status(self):
        """定时检查消息是否超时"""
        if self._streaming_last_msg_time is not None:
            cur_time = self.get_clock().now().nanoseconds / 1e9
            if (cur_time - self._streaming_last_msg_time) > 0.2:
                self.streaming_position = None
                self.last_streaming_position = None
                self.streaming_vel = None
                self.is_first_streaming = True
                self._streaming_last_msg_time = None
                self.streaming_history.clear()

    def is_connected(self, timeout_sec: float = 0.5):
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if self._state_last_msg_time is not None and \
                self._armlee_last_msg_time is not None and \
                 self._armree_last_msg_time is not None:
                return True
            time.sleep(0.01)
        return False

    def _init_publisher(self):
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory, '/dexe_robot_client/trajectory', 1
        )
        self.stop_dynamic_moving_publisher = self.create_publisher(
            String, '/dexe_robot_client/stop_dynamic_moving', 1
        )

    def _init_subscriber(self):
        if self.is_sim: 
            self.create_subscription(
                String,
                "/feedback/robot_server_state_sim",
                self._robot_server_state_callback,
                rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)
            )
            self.create_subscription(
                StreamingCommand,
                "/control/streaming_command_sim",
                self._streaming_cmd_callback,
                rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)
            )
            if self.ee_type == EndEffectorType.BRAINCO_HAND:
                self.create_subscription(
                    EEJointFeedback,
                    "/feedback_sim/hand/left",
                    self._left_end_effector_state_callback,
                    rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)
                )
                self.create_subscription(
                    EEJointFeedback,
                    "/feedback_sim/hand/right",
                    self._right_end_effector_state_callback,
                    rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)
                )
            elif self.ee_type == EndEffectorType.PGC_GRIPPER:
                self.create_subscription(
                    EEJointFeedback,
                    "/feedback_sim/gripper/left",
                    self._left_end_effector_state_callback,
                    rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)
                ) 
                self.create_subscription(
                    EEJointFeedback,
                    "/feedback_sim/gripper/right",
                    self._right_end_effector_state_callback,
                    rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)
                )    
        else:
            self.create_subscription(
                String,
                "/feedback/robot_server_state",
                self._robot_server_state_callback,
                rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)
            )
            self.create_subscription(
                StreamingCommand,
                "/control/streaming_command",
                self._streaming_cmd_callback,
                rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)
            )
            if self.ee_type == EndEffectorType.BRAINCO_HAND:
                self.create_subscription(
                    EEJointFeedback,
                    "/feedback/hand/left",
                    self._left_end_effector_state_callback,
                    rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)
                )
                self.create_subscription(
                    EEJointFeedback,
                    "/feedback/hand/right",
                    self._right_end_effector_state_callback,
                    rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)
                )
            elif self.ee_type == EndEffectorType.PGC_GRIPPER:
                self.create_subscription(
                    EEJointFeedback,
                    "/feedback/gripper/left",
                    self._left_end_effector_state_callback,
                    rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)
                ) 
                self.create_subscription(
                    EEJointFeedback,
                    "/feedback/gripper/right",
                    self._right_end_effector_state_callback,
                    rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)
                )    

    def _robot_server_state_callback(self, msg: String):
        """robot server state callback"""
        with self._state_lock:
            state = json.loads(msg.data)
            self._state_last_msg_time = self.get_clock().now().nanoseconds / 1e9
            if "status" in state:
                self.status = Status[state["status"]]
            if "joint_position" in state:
                self.joint_position = state["joint_position"]

    def _left_end_effector_state_callback(self, msg):
        with self._armlee_state_lock:
            self.armlee_state = np.array(msg.position)
            self._armlee_last_msg_time = self.get_clock().now().nanoseconds / 1e9

    def _right_end_effector_state_callback(self, msg):
        with self._armree_state_lock:
            self.armree_state = np.array(msg.position)
            self._armree_last_msg_time = self.get_clock().now().nanoseconds / 1e9

    def _streaming_cmd_callback(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        # self.logger.debug(f"receiving time: {now}")
        # self.logger.debug(f"streaming command: {msg.position[6:13]}")
        with self._streaming_cmd_lock:
            if self.is_first_streaming:
                self.is_first_streaming = False
                self.streaming_position = msg.position
                self.last_streaming_position = msg.position
                self.streaming_vel = np.zeros(len(msg.position))
                self._streaming_last_msg_time = now

                self.streaming_history.append({
                    "time": now,
                    "position": msg.position,
                    "velocity": np.zeros(len(msg.position))
                })

                if msg.streaming_finished:
                    self.is_first_streaming = True
            else:
                self.streaming_position = msg.position
                self.streaming_vel = (np.array(self.streaming_position) - np.array(self.last_streaming_position)) / self.streaming_dt
                self._streaming_last_msg_time = now

                self.streaming_history.append({
                    "time": now,
                    "position": msg.position,
                    "velocity": (np.array(msg.position) - np.array(self.last_streaming_position)) / self.streaming_dt
                })
                self.last_streaming_position = msg.position

                if msg.streaming_finished:
                    self.is_first_streaming = True

    def get_streaming_cmd(self):
        with self._streaming_cmd_lock:
            if self.is_first_streaming:
                return None, np.zeros(20), None, None
            return self.streaming_position, self.streaming_vel, self._streaming_last_msg_time, self.streaming_history


    def get_state(self):
        """
        Return the current status and joint positions of the robot.

        Returns:
            status (Status or None): The current status of the robot, or None if not available.
            joint_position (list or None): The current joint positions of the robot, or None if not available.
        """
        with self._state_lock:
            if self.status is None or \
                self.joint_position is None or \
                 self._state_last_msg_time is None:
                self.logger.critical(f"{self.node_name} robot not connected!")
                return False, None, None
            if time.time() - self._state_last_msg_time > 0.1:
                self.logger.warning(f"{self.node_name} Node's robot data is outdated.")

            return True, self.status, self.joint_position
        
    def get_left_ee_state(self):
        with self._armlee_state_lock:
            if self.armlee_state is None or \
                self._armlee_last_msg_time is None:
                self.logger.critical(f"{self.node_name} left end effector not connected!")
                return False, None
            if time.time() - self._armlee_last_msg_time > 0.1:
                self.logger.warning(f"{self.node_name} Node's left end effector data is outdated.")
            
            return True, self.armlee_state
        
    def get_right_ee_state(self):
        with self._armree_state_lock:
            if self.armree_state is None or \
                self._armree_last_msg_time is None:
                self.logger.critical(f"{self.node_name} right end effector not connected!")
                return False, None
            if time.time() - self._armree_last_msg_time > 0.1:
                self.logger.warning(f"{self.node_name} Node's right end effector data is outdated.")
            return True, self.armree_state
        
    def publish_trajectory(self, trajectory: np.ndarray):
        """
        This function will publish the trajectory to the robot server and update self.trajectory

        Args:
            trajectory: np.ndarray of shape (N, 20), N is the number of waypoints
        Returns:
            bool: True if success, False otherwise
        """
        # self.trajectory = trajectory
        # self.is_new_trajectory = True

        if self.trajectory_publisher is None:
            self.logger.error("Publisher not initialized")
            return False
        
        if trajectory is None:
            self.logger.info("Publishing empty trajectory")
            traj_msg = JointTrajectory()
            traj_msg.joint_names = [
                "ANKLE", "KNEE", "BUTTOCK", "WAIST",
                "NEKC1", "NECK2",
                "LEFT_J1", "LEFT_J2", "LEFT_J3", "LEFT_J4", "LEFT_J5", "LEFT_J6", "LEFT_J7",
                "RIGHT_J1", "RIGHT_J2", "RIGHT_J3", "RIGHT_J4", "RIGHT_J5", "RIGHT_J6", "RIGHT_J7"
            ]
            self.trajectory_publisher.publish(traj_msg)
            return True
        
        if trajectory.shape[1] != 20:
            self.logger.error("Invalid trajectory shape")
            return False
        

        
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [
            "ANKLE", "KNEE", "BUTTOCK", "WAIST",
            "NEKC1", "NECK2",
            "LEFT_J1", "LEFT_J2", "LEFT_J3", "LEFT_J4", "LEFT_J5", "LEFT_J6", "LEFT_J7",
            "RIGHT_J1", "RIGHT_J2", "RIGHT_J3", "RIGHT_J4", "RIGHT_J5", "RIGHT_J6", "RIGHT_J7"
        ]
        for i in range(trajectory.shape[0]):
            point = JointTrajectoryPoint()
            point.positions = trajectory[i].tolist()
            point.time_from_start = rclpy.duration.Duration(seconds=0.01 * (i + 1)).to_msg()
            traj_msg.points.append(point)

        self.trajectory_publisher.publish(traj_msg)
        self.logger.info(f"Publish new trajectory with {len(traj_msg.points)} points")

    def spin_once(self, timeout_sec: float = 0.1):
        r"""Spin the node once with a timeout of 0.1 sec."""
        rclpy.spin_once(self, timeout_sec= timeout_sec)

    def spin(self):
        r"""Continuously spin the node while rclpy is running."""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec = 0.1)