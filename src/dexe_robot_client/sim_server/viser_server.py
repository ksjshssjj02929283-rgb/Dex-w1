# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
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

    def __init__(self,
                 arm_type: W1ArmType,
                 w1_version: W1Version = W1Version.V2,
                 end_effector_type: EndEffectorType = None):
        self.logger = get_logger()
        self._end_effector_type = end_effector_type

        # res, file_path = generate_w1_urdf(arm_type, w1_version, end_effector_type)
        res, file_path = download_w1_urdf(arm_type, w1_version,
                                          end_effector_type)
        if not res:
            raise ValueError(f"Found URDF failed.")
        self._urdf_path = file_path

        self._lock = threading.Lock()
        self._cur_qpos = None
        self._init_visualization()

        self._init_ros2_node()
        self.logger.debug("Dexsim server initialized.")

    def _init_ros2_node(self):
        self._ros2_nodes = []

        self._robot_node = Node("robot_node")
        self._sub_robot = self._robot_node.create_subscription(
            String, "/feedback/robot_server_state_sim",
            self._subscriber_robot_callback, 10)
        self._ros2_nodes.append(self._robot_node)

        if self._end_effector_type is not None:
            ee_topics = {
                EndEffectorType.BRAINCO_HAND: {
                    "left": "/feedback_sim/hand/left",
                    "right": "/feedback_sim/hand/right",
                },
                EndEffectorType.PGC_GRIPPER: {
                    "left": "/feedback_sim/gripper/left",
                    "right": "/feedback_sim/gripper/right",
                }
            }

            self._left_ee_node = Node("left_ee_node")
            self._right_ee_node = Node("right_ee_node")
            if self._end_effector_type == EndEffectorType.BRAINCO_HAND:
                self._sub_left_ee = self._left_ee_node.create_subscription(
                    EEJointFeedback, ee_topics[self._end_effector_type]["left"],
                    self._subscriber_left_ee_callback, 10)

                self._sub_right_ee = self._right_ee_node.create_subscription(
                    EEJointFeedback,
                    ee_topics[self._end_effector_type]["right"],
                    self._subscriber_right_ee_callback, 10)

            elif self._end_effector_type == EndEffectorType.PGC_GRIPPER:
                self._sub_left_ee = self._left_ee_node.create_subscription(
                    EEJointFeedback, ee_topics[self._end_effector_type]["left"],
                    self._subscriber_left_ee_callback, 10)

                self._sub_right_ee = self._right_ee_node.create_subscription(
                    EEJointFeedback,
                    ee_topics[self._end_effector_type]["right"],
                    self._subscriber_right_ee_callback, 10)

            self._ros2_nodes.append(self._left_ee_node)
            self._ros2_nodes.append(self._right_ee_node)

        # 在单独线程运行ros node
        self._executor = rclpy.executors.MultiThreadedExecutor()
        for node in self._ros2_nodes:
            self._executor.add_node(node)

        self._ros_thread = threading.Thread(target=self._executor.spin,
                                            daemon=True)
        self._ros_thread.start()

    def _init_visualization(self):
        import yourdfpy
        import viser
        from viser.extras import ViserUrdf

        viser_server = viser.ViserServer()

        w1_urdf = yourdfpy.URDF.load(self._urdf_path)
        self.w1_viser_handle = ViserUrdf(viser_server, w1_urdf)
        dof = len(self.w1_viser_handle.get_actuated_joint_names())
        self._cur_qpos = np.zeros(dof)
        self.w1_viser_handle.update_cfg(self._cur_qpos)

        joint_names = self.w1_viser_handle.get_actuated_joint_names()
        joint_limits_map = self.w1_viser_handle.get_actuated_joint_limits()
        joint_limits = [joint_limits_map[name] for name in joint_names]

        if self._end_effector_type is not None:
            self.left_ee_sim2real_index = get_ee_sim2real_idx(
                joint_names, True, self._end_effector_type)
            self.left_ee_joint_limits = np.array([joint_limits[i] for i in self.left_ee_sim2real_index])

            self.right_ee_sim2real_index = get_ee_sim2real_idx(
                joint_names, False, self._end_effector_type)
            self.right_ee_joint_limits = np.array([joint_limits[i] for i in self.right_ee_sim2real_index])

        self.robot_sim2real_index = get_robot_sim2real_idx(joint_names)


    def _subscriber_robot_callback(self, msg):
        state = json.loads(msg.data)
        if "joint_position" not in state:
            return
        # with self._lock:
        #     self._articulation.set_current_qpos(np.array(state["joint_position"]),
        #                                         np.array(self.robot_sim2real_index))
        with self._lock:
            self._cur_qpos[self.robot_sim2real_index] = np.array(
                state["joint_position"])
        self.w1_viser_handle.update_cfg(self._cur_qpos)

    def _subscriber_left_ee_callback(self, msg):
        raw_qpos = np.array(msg.position)
        qpos = self.left_ee_joint_limits[:,0] + raw_qpos/100. * \
            (self.left_ee_joint_limits[:,1] - self.left_ee_joint_limits[:,0])
        # with self._lock:
        #     self._articulation.set_current_qpos(qpos,
        #                                         np.array(self.left_ee_sim2real_index))
        with self._lock:
            self._cur_qpos[self.left_ee_sim2real_index] = qpos

    def _subscriber_right_ee_callback(self, msg):
        raw_qpos = np.array(msg.position)
        qpos = self.right_ee_joint_limits[:,0] + raw_qpos/100. * \
            (self.right_ee_joint_limits[:,1] - self.right_ee_joint_limits[:,0])
        # with self._lock:
        #     self._articulation.set_current_qpos(qpos,
        #                                         np.array(self.right_ee_sim2real_index))
        with self._lock:
            self._cur_qpos[self.right_ee_sim2real_index] = qpos

    def _shutdown(self):
        try:
            self._executor.shutdown()
            self._ros_thread.join()
            for node in self._ros2_nodes:
                node.destroy_node()
            self.logger.debug("Dexsim server closed.")
        except Exception as e:
            self.logger.error(f"Error during shutdown: {e}")

    def run(self):
        try:
            self.logger.debug("Dexsim server running.")
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self._shutdown()

if __name__ == "__main__":
    rclpy.init()

    server = ViserServer(
        arm_type=W1ArmType.ANTHROPOMORPHIC,
        w1_version=W1Version.V2,
        end_effector_type=EndEffectorType.BRAINCO_HAND)
    server.run()
