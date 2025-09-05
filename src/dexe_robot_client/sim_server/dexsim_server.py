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
from std_msgs.msg import String, Float64MultiArray, Float64

from ..common import (get_logger, 
                      generate_w1_urdf,
                      W1Version, W1ArmType,
                      get_robot_sim2real_idx, get_robot_real2sim_idx,
                      get_ee_sim2real_idx, get_ee_real2sim_idx)
from ..end_effector_client import EndEffectorType

# TODO: set_current_qpos太慢，频率约10Hz
class DexsimServer:
    def __init__(self, 
                arm_type: W1ArmType, 
                w1_version: W1Version = W1Version.V2,
                end_effector_type: EndEffectorType = None):
        self.logger = get_logger()
        self._end_effector_type = end_effector_type

        res, file_path = generate_w1_urdf(arm_type, w1_version, end_effector_type)
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
                                            String, 
                                            "/feedback/robot_server_state_sim", 
                                            self._subscriber_robot_callback,
                                            10)
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
                        }}
            
            self._left_ee_node = Node("left_ee_node")
            self._right_ee_node = Node("right_ee_node")
            if self._end_effector_type == EndEffectorType.BRAINCO_HAND:
                self._sub_left_ee = self._left_ee_node.create_subscription(
                                            Float64MultiArray, 
                                            ee_topics[self._end_effector_type]["left"], 
                                            self._subscriber_left_ee_callback,
                                            10)
                
                self._sub_right_ee = self._right_ee_node.create_subscription(
                                            Float64MultiArray, 
                                            ee_topics[self._end_effector_type]["right"], 
                                            self._subscriber_right_ee_callback,
                                            10)
                
            elif self._end_effector_type == EndEffectorType.PGC_GRIPPER:
                self._sub_left_ee = self._left_ee_node.create_subscription(
                                            Float64, 
                                            ee_topics[self._end_effector_type]["left"], 
                                            self._subscriber_left_ee_callback,
                                            10)
                
                self._sub_right_ee = self._right_ee_node.create_subscription(
                                            Float64, 
                                            ee_topics[self._end_effector_type]["right"], 
                                            self._subscriber_right_ee_callback,
                                            10)         
                
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
        import dexsim
        # 仿真世界配置
        config = dexsim.WorldConfig()
        config.open_windows = True
        # 初始化仿真世界
        self._world = dexsim.World(config)
        # 显示世界坐标系
        self._world.show_coordinate_axis(True) # 可设置成False
        # 启用物理仿真
        self._world.enable_physics(True)

        # 获取环境管理句柄
        self._env = self._world.get_env()

        # 下载并设置天空顶IBL
        ibl_data = dexsim.data.IBL()
        iblight_fpath = ibl_data.extract_dir + "/IBL/lightroom_14b"
        self._env.set_IBL(iblight_fpath)

        # 设置地板
        default_length = 1000
        repeat_uv_size = int(default_length / 2)
        plane_handle = self._env.create_plane(
            0, default_length, repeat_uv_size, repeat_uv_size
        )
        plane_handle.set_name("default_plane")
        plane_collision_handle = self._env.create_cube(default_length, 
                                                 default_length, 
                                                 default_length / 10)
        plane_collision_xpos = np.eye(4, dtype=float)
        plane_collision_xpos[2, 3] = - default_length / 20 - 0.001
        plane_collision_handle.set_local_pose(plane_collision_xpos)
        plane_collision_handle.add_rigidbody(dexsim.types.ActorType.STATIC, 
                                             dexsim.types.RigidBodyShape.CONVEX)
        # 设置地板材质
        material_data = dexsim.data.Materials()
        material_fpath = material_data.extract_dir + "/materials"
        material_handle = self._env.load_material(material_fpath + "/Concrete033", "Concrete033")
        plane_handle.set_material(material_handle.get_inst())

        # 设置机器人
        self._articulation = self._env.load_urdf(str(self._urdf_path))
        # 设置世界坐标系
        robot_pose = np.eye(4, dtype=float)
        self._articulation.set_world_pose(robot_pose)
        # 取消自碰撞
        self._articulation.set_articulation_flag(dexsim.types.ArticulationFlag.DISABLE_SELF_COLLISION, True)
        
        if self._end_effector_type is not None:
            self.left_ee_sim2real_index = get_ee_sim2real_idx(self._articulation, True, self._end_effector_type)
            self.left_ee_joint_limits = self._articulation.get_joint_limits()[self.left_ee_sim2real_index]

            self.right_ee_sim2real_index = get_ee_sim2real_idx(self._articulation, False, self._end_effector_type)
            self.right_ee_joint_limits = self._articulation.get_joint_limits()[self.right_ee_sim2real_index] 

            self._articulation.set_drive(stiffness = 1e2, damping = 1e1, max_force = 1e3, 
                                        drive_type = dexsim.types.DriveType.FORCE)

        self.robot_sim2real_index = get_robot_sim2real_idx(self._articulation)
        self._articulation.set_drive(stiffness = 1e4, damping = 1e3, max_force = 1e5, 
                                    drive_type = dexsim.types.DriveType.FORCE,
                                    joint_ids = self.robot_sim2real_index)

        torso_ids = self.robot_sim2real_index[:4]
        self._articulation.set_drive(stiffness = 1e5, damping = 1e4, max_force = 1e6, 
                                    drive_type = dexsim.types.DriveType.FORCE,
                                    joint_ids = torso_ids)

        self._cur_qpos = self._articulation.get_current_qpos()

    def _subscriber_robot_callback(self, msg):
        state = json.loads(msg.data)
        if "joint_position" not in state:
            return 
        # with self._lock:
        #     self._articulation.set_current_qpos(np.array(state["joint_position"]),
        #                                         np.array(self.robot_sim2real_index))
        with self._lock:
            self._cur_qpos[self.robot_sim2real_index] = np.array(state["joint_position"])
        self._articulation.set_current_qpos(self._cur_qpos)

    def _subscriber_left_ee_callback(self, msg):
        raw_qpos = np.array(msg.data)
        qpos = self.left_ee_joint_limits[:,0] + raw_qpos/100. * \
            (self.left_ee_joint_limits[:,1] - self.left_ee_joint_limits[:,0])
        # with self._lock:
        #     self._articulation.set_current_qpos(qpos, 
        #                                         np.array(self.left_ee_sim2real_index))
        with self._lock:
            self._cur_qpos[self.left_ee_sim2real_index] = qpos

    def _subscriber_right_ee_callback(self, msg):
        raw_qpos = np.array(msg.data)
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