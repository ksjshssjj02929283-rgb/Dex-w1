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

from ..common import (get_logger, 
                      ASSETS_DIR, URL_PREFIX, 
                      download_and_unzip_file, 
                      W1Version, W1ArmType)
from ..end_effector_client import EndEffectorType
from .utils.mapping_idx import get_sim2real_idx, get_real2sim_idx

"""TODO
add end-effector
"""
class DexsimServer:
    def __init__(self, 
                arm_type: W1ArmType, 
                w1_version: W1Version = W1Version.V2,
                end_effector_type: EndEffectorType = None):
        self.logger = get_logger()

        if w1_version != W1Version.V2:
            raise ValueError(f"{w1_version.name} are not support.")
        if not download_and_unzip_file(URL_PREFIX + "/DexforceW1V02_v1.zip"):
            raise FileNotFoundError

        if arm_type == W1ArmType.INDUSTRIAL:
            self._urdf_path = ASSETS_DIR / "DexforceW1V02_v1/DexforceW1_v02_2.urdf"
        elif arm_type == W1ArmType.ANTHROPOMORPHIC:
            self._urdf_path = ASSETS_DIR / "DexforceW1V02_v1/DexforceW1_v02_1.urdf"
        else:
            raise ValueError(f"{arm_type.name} are not support.")

        self._init_visualization()
        self.real2sim_index = get_real2sim_idx(self._articulation)
        self.sim2real_index = get_sim2real_idx(self._articulation)
        self._init_ros2_node()
        self.logger.debug("Dexsim server initialized.")

    def _init_ros2_node(self):
        self._ros_node = Node("dexsim_server")
        self._ros_node.create_subscription(String, 
                                        "/feedback/robot_server_state_sim", 
                                        self._subscriber_robot_server_state_sim_callback,
                                        10)

        # 在单独线程运行ros node
        self._executor = rclpy.executors.MultiThreadedExecutor()
        self._executor.add_node(self._ros_node)
        self._ros_thread = threading.Thread(target=self._executor.spin)
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

        # 下载天空顶IBL
        ibl_data = dexsim.data.IBL()
        iblight_fpath = ibl_data.extract_dir + "/IBL/lightroom_14b"
        # 设置仿真世界天空顶
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
        robot_pose = np.eye(4, dtype=float)
        # 设置世界坐标系
        self._articulation.set_world_pose(robot_pose)
        # 取消自碰撞
        self._articulation.set_articulation_flag(dexsim.types.ArticulationFlag.DISABLE_SELF_COLLISION, True)
        # 设置运动学仿真dexsim.types.DriveType.FORCE 或 动力学仿真dexsim.types.DriveType.TARGET
        self._articulation.set_drive(drive_type=dexsim.types.DriveType.FORCE)

    def _subscriber_robot_server_state_sim_callback(self, msg):
        state = json.loads(msg.data)
        if "joint_position" in state:
            arr = np.array(state["joint_position"])
            qpos = arr[self.real2sim_index]
            self._articulation.set_current_qpos(qpos)

    def _shutdown(self):
        try:
            self._executor.shutdown()
            self._ros_thread.join()
            self._ros_node.destroy_node()
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