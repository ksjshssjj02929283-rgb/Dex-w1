# ----------------------------------------------------------------------------
# Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import time
import json
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from end_effector_interfaces.msg import EEJointFeedback

from ..common import (get_logger, generate_w1_urdf, download_w1_urdf,
                      download_environment, W1Version, W1ArmType,
                      get_robot_sim2real_idx, get_robot_real2sim_idx,
                      get_ee_sim2real_idx, get_ee_real2sim_idx)
from ..end_effector_client import EndEffectorType


# python3 -m dexe_robot_client.sim_server.viser_server
class ViserServer:

    DEFAULT_CONFIG = {
        "robot_topic": "/feedback/robot_server_state_sim",
        "ee_topics": {
            EndEffectorType.BRAINCO_HAND: {
                "left": "/feedback_sim/hand/left",
                "right": "/feedback_sim/hand/right",
            },
            EndEffectorType.PGC_GRIPPER: {
                "left": "/feedback_sim/gripper/left",
                "right": "/feedback_sim/gripper/right",
            }
        }
    }

    def __init__(self,
                 arm_type: W1ArmType,
                 w1_version: W1Version = W1Version.V2,
                 end_effector_type: EndEffectorType = None,
                 config: dict = None):
        self.logger = get_logger()
        self._end_effector_type = end_effector_type

        # init config
        self.config = {**self.DEFAULT_CONFIG, **(config or {})}

        res, self._urdf_path = download_w1_urdf(arm_type, w1_version,
                                                end_effector_type)
        if not res:
            raise ValueError(f"Found URDF failed.")

        self._lock = threading.Lock()
        self._cur_qpos = None
        self._init_visualization()

        self._init_ros2_node()

        self.logger.debug("Viser server initialized.")

    def _init_ros2_node(self):
        self._ros2_nodes = []

        # initialize robot node
        self._robot_node = self._create_node_with_subscription(
            "robot_node", self.config["robot_topic"],
            self._subscriber_robot_callback, String)
        self._ros2_nodes.append(self._robot_node)

        # initialize end-effector nodes
        if self._end_effector_type is not None:
            ee_topics = self.config["ee_topics"]
            for side in ["left", "right"]:
                node_name = f"{side}_ee_node"
                topic = ee_topics[self._end_effector_type][side]
                callback = getattr(self, f"_subscriber_{side}_ee_callback")
                ee_node = self._create_node_with_subscription(
                    node_name, topic, callback, EEJointFeedback)
                self._ros2_nodes.append(ee_node)

        # start ROS thread
        self._start_ros_thread()

    def _create_node_with_subscription(self, node_name, topic, callback,
                                       msg_type):
        node = Node(node_name)
        node.create_subscription(msg_type, topic, callback, qos_profile=100)
        return node

    def _start_ros_thread(self):
        self._executor = rclpy.executors.MultiThreadedExecutor()
        for node in self._ros2_nodes:
            self._executor.add_node(node)

        self._ros_thread = threading.Thread(target=self._executor.spin,
                                            daemon=True)
        self._ros_thread.start()

    def _init_visualization(self):
        try:
            import yourdfpy
            import viser
            from viser.extras import ViserUrdf

            viser_server = viser.ViserServer()

            w1_urdf = yourdfpy.URDF.load(self._urdf_path)
            self.w1_viser_handle = ViserUrdf(viser_server, w1_urdf)
            dof = len(self.w1_viser_handle.get_actuated_joint_names())
            self._cur_qpos = np.zeros(dof)
            self.w1_viser_handle.update_cfg(self._cur_qpos)
        except Exception as e:
            self.logger.error(f"Failed to initialize viser: {e}")
            raise e

        joint_names = self.w1_viser_handle.get_actuated_joint_names()
        joint_limits_map = self.w1_viser_handle.get_actuated_joint_limits()
        joint_limits = [joint_limits_map[name] for name in joint_names]

        if self._end_effector_type is not None:
            self.left_ee_sim2real_index = get_ee_sim2real_idx(
                joint_names, True, self._end_effector_type)
            self.left_ee_joint_limits = np.array(
                [joint_limits[i] for i in self.left_ee_sim2real_index])

            self.right_ee_sim2real_index = get_ee_sim2real_idx(
                joint_names, False, self._end_effector_type)
            self.right_ee_joint_limits = np.array(
                [joint_limits[i] for i in self.right_ee_sim2real_index])

        self.robot_sim2real_index = get_robot_sim2real_idx(joint_names)

    def _subscriber_robot_callback(self, msg):
        try:
            state = json.loads(msg.data)
            if "joint_position" not in state:
                self.logger.warning(
                    "Received robot state without 'joint_position'.")
                return
            with self._lock:
                self._cur_qpos[self.robot_sim2real_index] = np.array(
                    state["joint_position"])
            self.w1_viser_handle.update_cfg(self._cur_qpos)
        except Exception as e:
            self.logger.error(f"Error in robot callback: {e}")

    def _update_ee_joint_positions(self, raw_qpos, joint_limits,
                                   sim2real_index):
        try:
            qpos = joint_limits[:, 0] + raw_qpos / 100.0 * (joint_limits[:, 1] -
                                                            joint_limits[:, 0])
            with self._lock:
                self._cur_qpos[sim2real_index] = qpos
            self.w1_viser_handle.update_cfg(self._cur_qpos)
        except Exception as e:
            self.logger.error(f"Failed to update joint positions: {e}")

    def _subscriber_left_ee_callback(self, msg):
        raw_qpos = np.array(msg.position)
        self._update_ee_joint_positions(raw_qpos, self.left_ee_joint_limits,
                                        self.left_ee_sim2real_index)

    def _subscriber_right_ee_callback(self, msg):
        raw_qpos = np.array(msg.position)
        self._update_ee_joint_positions(raw_qpos, self.right_ee_joint_limits,
                                        self.right_ee_sim2real_index)

    def _shutdown(self):
        try:
            self._executor.shutdown()
            if self._ros_thread and self._ros_thread.is_alive():
                self._ros_thread.join()
            for node in self._ros2_nodes:
                if node:
                    node.destroy_node()
            self.logger.debug("Viser server closed.")
        except Exception as e:
            self.logger.error(f"Error during shutdown: {e}")

    def run(self):
        try:
            self.logger.debug("Viser server running.")
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        except Exception as e:
            self.logger.error(f"Unexpected error: {e}")
        finally:
            self._shutdown()


if __name__ == "__main__":
    rclpy.init()

    try:
        server = ViserServer(arm_type=W1ArmType.ANTHROPOMORPHIC,
                             w1_version=W1Version.V2,
                             end_effector_type=EndEffectorType.BRAINCO_HAND)
        server.run()
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        rclpy.shutdown()
