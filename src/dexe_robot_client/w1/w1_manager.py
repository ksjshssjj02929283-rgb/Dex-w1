# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import numpy as np

np.set_printoptions(precision=4, suppress=True)
import time
import threading
import pathlib
import json
import atexit
from queue import Queue, Empty
from copy import deepcopy
import concurrent.futures
from typing import Tuple, List

import rclpy
from rclpy.executors import MultiThreadedExecutor

from ..common import time_consuming, get_logger, W1ArmType, W1Version, download_w1_urdf, StatusCode
from ..common.mapping_index import *
from ..end_effector_client import EndEffectorType, create_end_effector_client
from ..navigation_client import ChassisClient
from ..robot_client import RobotClientRos2 as RobotClient
from ..robot_client import MotionGroup, Status
from .trajectory_planner.toppra_trajectory_planner import ToppraTrajectoryPlanner
from .path_planner.blend_path import PathType, PathRnBezier, PathR3xSO3Bezier, weighted_norm
from .path_planner.cartesian_planner import CartesianPlanner
from .path_planner.cartesian_to_joint_planner import CartesianToJointPlanner
from .path_planner.fcl_checker import FCLChecker
from .path_planner.planner_pinocchio import Planner_Pin
from .path_planner.utils.robot_type import *
from .path_planner.hybrid_planner import HybridPlanner
from .utils.w1_params import W1Params
from .utils.unit_conversion import user_pose_to_xpos, xpos_to_user_pose

from dexe_rp import DexforceW1Kinematics, SRSConfig

# from clientsupervisor.move_sync_option import MoveSyncOption
from clientsupervisor.client_supervisior import RobotClientSupervisor
from .mgr_util.supervisor_view import SupervisorViewW1
from .mgr_util.robot_client_warpper import RobotClientWrapper, GripperClientWrapper
from .movement.move_line_dual_arm import DualArmMoveLineW1
from .movement.move_joint_dual_arm import DualArmMoveJointW1


class W1Manager:
    _instance = None
    _lock = threading.Lock()

    def __new__(cls, *args, **kwargs):
        with cls._lock:
            if not cls._instance:
                cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self,
                 is_sim: bool,
                 arm_type: W1ArmType,
                 w1_version: W1Version = W1Version.V2,
                 end_effector_type: EndEffectorType = None,
                 use_nav: bool = False):
        self.logger = get_logger()

        self.TIME_STEP = 0.01  # 100Hz

        self.robot_client = None
        self._robot_client_sim = None
        self._robot_client_real = None

        self.left_ee_client = None
        self._left_ee_client_sim = None
        self._left_ee_client_real = None

        self.right_ee_client = None
        self._right_ee_client_sim = None
        self._right_ee_client_real = None

        self._use_nav = use_nav
        self.chassis_client = None
        self._chassis_client_sim = None
        self._chassis_client_real = None

        self._clients = []

        self._executor = None
        self._node_thread = None

        self._is_sim = None
        self._arm_type = arm_type
        self._w1_version = w1_version
        self._end_effector_type = end_effector_type

        self.w1_params = W1Params(self._arm_type, self._w1_version,
                                  self._end_effector_type)

        res, file_path = download_w1_urdf(arm_type, w1_version,
                                          end_effector_type)
        res, file_path = download_w1_urdf(arm_type, w1_version, None)

        # 在主线程中正常退出时自动调用
        atexit.register(self._destroy_clients)

        try:
            self._init_clients()
        except Exception:
            raise RuntimeError("Ros2 node create failed")

        self._init_kine_solver()

        if not self.set_sim_mode(is_sim):
            if is_sim:
                raise RuntimeError("Set sim mode failed")
            raise RuntimeError("Set real mode failed")

        # initialize fcl checker and pinocchio model and planner
        self._init_fcl_checker(arm_type=arm_type,
                               w1_version=w1_version,
                               end_effector_type=end_effector_type)
        self._init_pinocchio(arm_type=arm_type,
                             w1_version=w1_version,
                             end_effector_type=end_effector_type)
        self.real_torso_idx, self.real_head_idx, self.real_arml_idx, self.real_armr_idx = get_real_idx(
        )
        self._init_planner(arm_type=arm_type,
                           w1_version=w1_version,
                           end_effector_type=end_effector_type)

        self._init_cartesian_planner(arm_type=arm_type,
                                     w1_version=w1_version,
                                     end_effector_type=end_effector_type)

        self._stop_move_event = threading.Event()
        self._stop_move_event.clear()

        self.collision_cnt = 0

        # initialize client supervisor
        self._init_client_supervisor()

    def _init_fcl_checker(self,
                          arm_type: W1ArmType,
                          w1_version: W1Version = W1Version.V2,
                          end_effector_type: EndEffectorType = None):
        if w1_version == W1Version.V2:
            if arm_type == W1ArmType.ANTHROPOMORPHIC:
                fcl_robot_type = RobotType.W1V02_1
            elif arm_type == W1ArmType.INDUSTRIAL:
                fcl_robot_type = RobotType.W1V02_2
            else:
                raise ValueError(
                    f"Not supported arm type: {arm_type} for fcl checker")
                return
            self.fcl_checker = FCLChecker(fcl_robot_type, end_effector_type)
            self.fcl_checker.enable_visualize()
        else:
            raise ValueError(
                f"Not supported w1 version: {w1_version} for fcl checker")

    def _init_pinocchio(self,
                        arm_type: W1ArmType = W1ArmType.ANTHROPOMORPHIC,
                        w1_version: W1Version = W1Version.V2,
                        end_effector_type: EndEffectorType = None):
        """
        Initialize the pinocchio model and data for the robot.
        """
        if w1_version == W1Version.V2:
            if arm_type == W1ArmType.ANTHROPOMORPHIC:
                pin_robot_type = RobotType.W1V02_1
            elif arm_type == W1ArmType.INDUSTRIAL:
                pin_robot_type = RobotType.W1V02_2
            else:
                raise ValueError(
                    f"Not supported arm type: {arm_type} for pinocchio model")
                return
            if end_effector_type is None or isinstance(end_effector_type,
                                                       EndEffectorType):
                pin_ee_type = end_effector_type
            else:
                raise ValueError(
                    f"Not supported end_effector_type: {end_effector_type} for pinocchio model"
                )
                return

            self._pin_kine = Planner_Pin(robot_type=pin_robot_type,
                                         end_effector_type=pin_ee_type)

        else:
            raise ValueError(
                f"Not supported w1 version: {w1_version} for pinocchio model")

    def _init_planner(self,
                      arm_type: W1ArmType,
                      w1_version: W1Version.V2,
                      end_effector_type: EndEffectorType = None):
        if w1_version == W1Version.V2:
            if arm_type == W1ArmType.ANTHROPOMORPHIC:
                planner_robot_type = RobotType.W1V02_1
            elif arm_type == W1ArmType.INDUSTRIAL:
                planner_robot_type = RobotType.W1V02_2
            else:
                raise ValueError(
                    f"Not supported arm type: {arm_type} for planner")
                return
            HybridPlanner.set_arms_cfg(SINGLE_ARM_DIM, SINGLE_ARM_DIM,
                                       LARM_BOUND, RARM_BOUND)
            HybridPlanner.set_robot_cfg(robot_type=planner_robot_type,
                                        ee_type=end_effector_type)
            # HybridPlanner.set_env_json(get_env_json_path())
            self.hybrid_planner = HybridPlanner()

    def _init_cartesian_planner(self,
                                arm_type: W1ArmType,
                                w1_version: W1Version,
                                end_effector_type: EndEffectorType = None):
        self.cartesian_joint_planner = CartesianToJointPlanner(
            arm_type, w1_version, end_effector_type)
        self.cartesian_planner = CartesianPlanner(arm_type, w1_version,
                                                  end_effector_type)

    def _init_clients(self):
        if not rclpy.ok():
            rclpy.init()

        self._robot_client_sim = RobotClient(is_sim=True)
        self._robot_client_real = RobotClient(is_sim=False)
        self._chassis_client_real = ChassisClient(is_sim=False)
        self.robot_client = self._robot_client_sim

        if self._end_effector_type is None:
            self.logger.info(f"No movable end effector.")
            if self._use_nav:
                self._clients = [
                    self._robot_client_sim, self._robot_client_real,
                    self._chassis_client_real
                ]
            else:
                self._clients = [
                    self._robot_client_sim, self._robot_client_real
                ]
        else:
            self._left_ee_client_sim = create_end_effector_client(
                is_left=True,
                is_sim=True,
                end_effector_type=self._end_effector_type)
            self._right_ee_client_sim = create_end_effector_client(
                is_left=False,
                is_sim=True,
                end_effector_type=self._end_effector_type)
            self._left_ee_client_real = create_end_effector_client(
                is_left=True,
                is_sim=False,
                end_effector_type=self._end_effector_type)
            self._right_ee_client_real = create_end_effector_client(
                is_left=False,
                is_sim=False,
                end_effector_type=self._end_effector_type)
            self.left_ee_client = self._left_ee_client_sim
            self.right_ee_client = self._right_ee_client_sim

            if self._use_nav:
                self._clients = [
                    self._robot_client_sim, self._robot_client_real,
                    self._left_ee_client_sim, self._left_ee_client_real,
                    self._right_ee_client_sim, self._right_ee_client_real,
                    self._chassis_client_real
                ]
            else:
                self._clients = [
                    self._robot_client_sim, self._robot_client_real,
                    self._left_ee_client_sim, self._left_ee_client_real,
                    self._right_ee_client_sim, self._right_ee_client_real
                ]

        self._is_sim = True

        self._executor = MultiThreadedExecutor()
        for node in self._clients:
            self._executor.add_node(node)
        self._node_thread = threading.Thread(target=self._executor.spin,
                                             daemon=True)
        self._node_thread.start()
        self.logger.info("Ros2 client initialized.")

    def _init_client_supervisor(self):

        robot_client_wrapper_sim = RobotClientWrapper(self._robot_client_sim)
        robot_client_wrapper_real = RobotClientWrapper(self._robot_client_real)

        if self._end_effector_type is not None:
            left_ee_client_wrapper_sim = GripperClientWrapper(
                self._left_ee_client_sim)
            left_ee_client_wrapper_real = GripperClientWrapper(
                self._left_ee_client_real)

            right_ee_client_wrapper_sim = GripperClientWrapper(
                self._right_ee_client_sim)
            right_ee_client_wrapper_real = GripperClientWrapper(
                self._right_ee_client_real)

        sim_executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        real_executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)

        self._client_supervisor_sim = RobotClientSupervisor(
            robot_client=robot_client_wrapper_sim,
            client_map={
                "left_arm_ee": left_ee_client_wrapper_sim,
                "right_arm_ee": right_ee_client_wrapper_sim
            },
            executor_owned=sim_executor,
        )

        if self._end_effector_type is not None:
            if self._use_nav:
                self._client_supervisor_real = RobotClientSupervisor(
                    robot_client=robot_client_wrapper_real,
                    client_map={
                        "left_arm_ee": left_ee_client_wrapper_real,
                        "right_arm_ee": right_ee_client_wrapper_real,
                        # TODO: add chassis client wrapper
                        "chassis": self._chassis_client_real
                    },
                    executor_owned=real_executor,
                )
            else:
                self._client_supervisor_real = RobotClientSupervisor(
                    robot_client=robot_client_wrapper_real,
                    client_map={
                        "left_arm_ee": left_ee_client_wrapper_real,
                        "right_arm_ee": right_ee_client_wrapper_real
                    },
                    executor_owned=real_executor,
                )
        else:
            if self._use_nav:
                self._client_supervisor_real = RobotClientSupervisor(
                    robot_client=robot_client_wrapper_real,
                    client_map={
                        # TODO: add chassis client wrapper
                        "chassis": self._chassis_client_real
                    },
                    executor_owned=real_executor,
                )
            else:
                self._client_supervisor_real = RobotClientSupervisor(
                    robot_client=robot_client_wrapper_real,
                    client_map={},
                    executor_owned=real_executor,
                )

        self._client_supervisor_sim.initialize()
        self._client_supervisor_real.initialize()

        self.supervisor_view_sim = SupervisorViewW1(
            supervisor=self._client_supervisor_sim,
            w1_params=self.w1_params,
            kine_solver=self.kine,
            pin_kine_solver=self._pin_kine,
            collision_checker=self.fcl_checker,
            hybrid_planner=self.hybrid_planner,
            cartesian_planner=self.cartesian_planner,
            cartesian_joint_planner=self.cartesian_joint_planner,
            stop_move_event=self._stop_move_event,
            time_step=self.TIME_STEP)

        self.supervisor_view_real = SupervisorViewW1(
            supervisor=self._client_supervisor_real,
            w1_params=self.w1_params,
            kine_solver=self.kine,
            pin_kine_solver=self._pin_kine,
            collision_checker=self.fcl_checker,
            hybrid_planner=self.hybrid_planner,
            cartesian_planner=self.cartesian_planner,
            cartesian_joint_planner=self.cartesian_joint_planner,
            stop_move_event=self._stop_move_event,
            time_step=self.TIME_STEP)

        self.target_supervisor = self._client_supervisor_sim if self._is_sim else self._client_supervisor_real
        self.target_supervisor_view = self.supervisor_view_sim if self._is_sim else self.supervisor_view_real

        self.logger.info("Client supervisor initialized.")

    def _destroy_clients(self):
        try:
            try:
                if self._client_supervisor_real is not None:
                    self._client_supervisor_real.stop()
                    self._client_supervisor_real = None
            except Exception as e:
                self.logger.error(f"Failed to stop real client supervisor: {e}")

            try:
                if self._client_supervisor_sim is not None:
                    self._client_supervisor_sim.stop()
                    self._client_supervisor_sim = None
            except Exception as e:
                self.logger.error(f"Failed to stop sim client supervisor: {e}")

            if rclpy.ok():
                rclpy.shutdown()  # 停止ros触发spin

            if self._node_thread is not None:
                self._node_thread.join()

            if self._executor is not None:
                self._executor.shutdown()

            for node in self._clients:
                node.destroy_node()

            self.logger.debug(f"Ros2 node destroyed.")
        except Exception as e:
            self.logger.error(f"Ros2 node destroy failed: {e}.")

    def _init_kine_solver(self):
        ik_nearest_weight = np.array([2., 2., 2., 1., 1., 1., 1.])
        self.kine = DexforceW1Kinematics(
            self._arm_type == W1ArmType.ANTHROPOMORPHIC)
        self.kine.left_arm_kine.set_ik_nearest_weight(ik_nearest_weight)
        self.kine.right_arm_kine.set_ik_nearest_weight(ik_nearest_weight)
        self.kine.set_user_qpos_limits(self.w1_params.qpos_lb,
                                       self.w1_params.qpos_ub)

        self.logger.info("Initialize kinematic solver.")

    def _has_chassis_client(self) -> bool:
        if self._use_nav is False:
            self.logger.error("Navigation function is disabled.")
            return False
        if self.chassis_client is None:
            self.logger.error("Chassis client has not initialized")
            return False
        return True

    def _get_chassis_client(self):
        if not self._has_chassis_client():
            return None
        return self.chassis_client

    def load_fcl_environment(self, file_path: str = None) -> None:
        """
        file_path: the path of fcl environment json file path
        """
        if file_path is None:
            env_json_path = get_env_json_path()
        else:
            env_json_path = file_path
        self.fcl_checker.load_env_json(env_json_path)

    def load_planner_environment(self, file_path: str = None) -> None:
        """
        file_path: the path of planner environment json file path
        """
        if file_path is None:
            env_json_path = get_env_json_path()
        else:
            env_json_path = file_path
        self.hybrid_planner.set_env_json(env_json_path)
        self.hybrid_planner.update_fcl_env(env_json_path)

    ######## 基础API #############
    def connect(self, timeout_sec: float = 0.5) -> StatusCode:
        """连接client(sim/real)

        Args:
            timeout_sec (float, optional): 等待连接超时时间. Defaults to 0.5.

        Returns:
            StatusCode: 
        """
        if self.is_connected(timeout_sec):
            return StatusCode.SUCCESS
        return StatusCode.FAIL

    def is_connected(self, timeout_sec: float = 0.1) -> StatusCode:
        """检查所有client是否处于连接状态

        Args:
            timeout_sec (float, optional): 等待连接超时时间. Defaults to 0.5.

        Returns:
            StatusCode: 
        """
        robot_client = self.target_supervisor.robot_client._client
        left_ee_client = self.target_supervisor.client_map.get(
            "left_arm_ee", None)
        right_ee_client = self.target_supervisor.client_map.get(
            "right_arm_ee", None)
        # chassis_client = self.target_supervisor.client_map.get("chassis", None)

        if not robot_client.is_connected(timeout_sec):
            self.logger.info(f"{robot_client.node_name} is not connected!")
            return StatusCode.ROBOT_NOT_CONNECT
        if self.target_supervisor_view._has_arm_ee_client("left_arm_ee", False):
            if not left_ee_client._client.is_connected(timeout_sec):
                self.logger.info(
                    f"{left_ee_client._client.node_name} is not connected!")
                return StatusCode.EE_NOT_CONNECT
        if self.target_supervisor_view._has_arm_ee_client(
                "right_arm_ee", False):
            if not right_ee_client._client.is_connected(timeout_sec):
                self.logger.info(
                    f"{right_ee_client._client.node_name} is not connected!")
                return StatusCode.EE_NOT_CONNECT
        if self._use_nav:
            if not self.chassis_client.is_connected():
                self.logger.error(
                    f"{self.chassis_client.node_name} is not connected!")
            return StatusCode.CHASSIS_NOT_CONNECT

        return StatusCode.SUCCESS

    def enable(self, timeout_sec: float = 0.5) -> StatusCode:
        self.logger.warning(
            "Not implemented (Currently it will automatically enable).")
        return StatusCode.SUCCESS

    def is_enabled(self) -> StatusCode:
        """检查robot是否处于使能状态

        Returns:
            StatusCode: 
        """
        # robot_client = self.target_supervisor.robot_client._client
        robot_client = self.robot_client

        if not robot_client.is_connected(0.5):
            self.logger.error(f"{robot_client.node_name} is not connected!")
            return StatusCode.ROBOT_NOT_CONNECT
        if robot_client.status == Status.Init:
            self.logger.error(
                f"{robot_client.node_name} is not enabled, robot status: {robot_client.robot_statusword}."
            )
            return StatusCode.ROBOT_NOT_ENABLE
        return StatusCode.SUCCESS

    def clear_robot_error(self) -> StatusCode:
        """清除机器人错误

        Returns:
            StatusCode: 
        """
        # robot_client = self.target_supervisor.robot_client._client
        robot_client = self.robot_client

        if self._is_sim:
            self.logger.error("Only support real mode.")
            return StatusCode.FAIL

        res = robot_client.clear_error()
        if res == -1:
            self.logger.error("Clear robot error failed!")
            return StatusCode.FAIL

        return StatusCode.SUCCESS

    def set_drag_mode(self, enable_drag: bool) -> StatusCode:
        """开关机器人拖动示教模式

        Args:
            enable_drag (bool): 是否开启拖动示教模式

        Returns:
            StatusCode: 
        """
        if self._is_sim:
            self.logger.error("Only support real mode.")
            return StatusCode.FAIL

        res = self.robot_client.set_drag_mode(enable_drag)
        if res == -1:
            self.logger.error("Set drag mode failed!")
            return StatusCode.FAIL

        return StatusCode.SUCCESS

    def set_safe_level(self, safe_level: float) -> StatusCode:
        """设置机器人安全等级

        Args:
            safe_level (float): 安全级别, 取值范围 [0.1, 1], 默认为0.5  
                                0.1时机械臂柔性最大,1.0时机械臂刚性最大   
                                值越小机械臂柔性越大越安全, 刚性越大运动跟踪性能越好但碰撞时容易造成机械臂损坏

        Returns:
            StatusCode: 
        """
        if self._is_sim:
            self.logger.error("Only support real mode.")
            return StatusCode.FAIL

        res = self.robot_client.set_safe_level(safe_level)
        if res == -1:
            self.logger.error("Set save level failed!")
            return StatusCode.FAIL

        return StatusCode.SUCCESS

    def set_sim_mode(self, enable_sim: bool) -> StatusCode:
        """设置仿真/真实模式"""
        # TODO: 优化代码结构，避免重复

        if self._is_sim == enable_sim:
            mode = "sim" if self._is_sim else "real"
            self.logger.info(f"W1 manager already in {mode} mode.")
            return StatusCode.SUCCESS

        if enable_sim:
            target_name = "sim"
            target_robot_client = self._robot_client_sim
            target_left_ee_client = self._left_ee_client_sim
            target_right_ee_client = self._right_ee_client_sim
            target_chassis_client = None
            if self._use_nav:
                self.logger.info(
                    "Navigation client only support real mode. use_nav set to False."
                )
                self._use_nav = False
        else:
            target_name = "real"
            target_robot_client = self._robot_client_real
            target_left_ee_client = self._left_ee_client_real
            target_right_ee_client = self._right_ee_client_real
            if self._use_nav:
                target_chassis_client = self._chassis_client_real
            else:
                target_chassis_client = None

        # 检查target client
        if not target_robot_client.is_connected(timeout_sec=0.1):
            self.logger.error(
                f"{target_robot_client.node_name} is not connected!")
            return StatusCode.ROBOT_NOT_CONNECT

        if self._end_effector_type is not None:
            if not target_left_ee_client.is_connected():
                self.logger.error(
                    f"{target_left_ee_client.node_name} is not connected!")
                return StatusCode.EE_NOT_CONNECT
            if not target_right_ee_client.is_connected():
                self.logger.error(
                    f"{target_right_ee_client.node_name} is not connected!")
                return StatusCode.EE_NOT_CONNECT

        if self._use_nav and target_chassis_client is not None and not target_chassis_client.is_connected(
        ):
            self.logger.error(
                f"{target_chassis_client.node_name} is not connected!")
            return StatusCode.CHASSIS_NOT_CONNECT

        # 检查current client
        if not self.robot_client.is_connected():
            self.logger.error(
                f"{self.robot_client.node_name} is not connected!")
            return StatusCode.ROBOT_NOT_CONNECT

        if self._end_effector_type is not None:
            if not self.left_ee_client.is_connected():
                self.logger.error(
                    f"{self.left_ee_client.node_name} is not connected!")
                return StatusCode.EE_NOT_CONNECT
            if not self.right_ee_client.is_connected():
                self.logger.error(
                    f"{self.right_ee_client.node_name} is not connected!")
                return StatusCode.EE_NOT_CONNECT

        self.robot_client = target_robot_client
        if self._end_effector_type is not None:
            self.left_ee_client = target_left_ee_client
            self.right_ee_client = target_right_ee_client
        self._is_sim = enable_sim
        if self._use_nav and target_chassis_client is not None:
            self.chassis_client = target_chassis_client
        self.logger.info(f"W1 manager set to {target_name} mode.")

        self.target_supervisor = self._client_supervisor_sim if self._is_sim else self._client_supervisor_real
        self.target_supervisor_view = self.supervisor_view_sim if self._is_sim else self.supervisor_view_real

        return StatusCode.SUCCESS

    def move_chassis_nav2(self, base_position: np.ndarray) -> StatusCode:
        """移动底盘至目标位置，使用导航栈"""
        if not self._use_nav:
            self.logger.error("Navigation function is disabled.")
            return StatusCode.FAIL
        if not self._has_chassis_client():
            self.logger.error("Chassis client has not initialized")
            return StatusCode.FAIL
        res = self._get_chassis_client().set_chassis_nav2_position(
            base_position)
        return (StatusCode.SUCCESS if res else StatusCode.FAIL)

    def move_chassis_ctrl(self, base_position: np.ndarray) -> StatusCode:
        """移动底盘至目标位置，使用底盘控制接口"""
        if not self._use_nav:
            self.logger.error("Navigation function is disabled.")
            return StatusCode.FAIL
        if not self._has_chassis_client():
            self.logger.error("Chassis client has not initialized")
            return StatusCode.FAIL
        res = self._get_chassis_client().set_chassis_control_position(
            base_position)
        return (StatusCode.SUCCESS if res else StatusCode.FAIL)

    def get_current_chassis_pose(self) -> tuple[StatusCode, np.ndarray]:
        """获取底盘当前位姿[x(m), y(m), yaw(angle)]"""
        if not self._use_nav:
            self.logger.error("Navigation function is disabled.")
            return StatusCode.FAIL, None
        if not self._has_chassis_client():
            self.logger.error("Chassis client has not initialized")
            return StatusCode.FAIL, None
        res, chassis_pos = self._get_chassis_client().get_chassis_position()
        return (StatusCode.SUCCESS if res else StatusCode.FAIL), chassis_pos

    def get_current_battery_percentage(self) -> tuple[StatusCode, float]:
        """获取当前电池电量百分比"""
        if not self._use_nav:
            self.logger.error("Navigation function is disabled.")
            return StatusCode.FAIL, None
        if not self._has_chassis_client():
            self.logger.error("Chassis client has not initialized")
            return StatusCode.FAIL, None
        res, value = self._get_chassis_client().get_battery_percentage()
        return (StatusCode.SUCCESS if res else StatusCode.FAIL), value

    def get_nav_status(self) -> str:
        """获取导航状态"""
        if not self._use_nav:
            self.logger.error("Navigation function is disabled.")
            return "DISABLED"
        if not self._has_chassis_client():
            self.logger.error("Chassis client has not initialized")
            return "NOT_INITIALIZED"
        return self._get_chassis_client().get_nav_state()

    def get_control_status(self) -> str:
        """获取底盘控制状态"""
        if not self._use_nav:
            self.logger.error("Navigation function is disabled.")
            return "DISABLED"
        if not self._has_chassis_client():
            self.logger.error("Chassis client has not initialized")
            return "NOT_INITIALIZED"
        return self._get_chassis_client().get_control_status()

    def set_auto_charging(self) -> StatusCode:
        """调用回充服务"""
        if not self._use_nav:
            self.logger.error("Navigation function is disabled.")
            return StatusCode.FAIL
        if not self._has_chassis_client():
            self.logger.error("Chassis client has not initialized")
            return StatusCode.FAIL
        res = self._get_chassis_client().set_chassis_auto_charging()
        return (StatusCode.SUCCESS if res else StatusCode.FAIL)

    def cancel_chassis_motion(self) -> StatusCode:
        """统一取消底盘运动，同时调用取消导航和取消控制。"""
        if not self._use_nav:
            self.logger.error("Navigation function is disabled.")
            return StatusCode.FAIL
        if not self._has_chassis_client():
            self.logger.error("Chassis client has not initialized")
            return StatusCode.FAIL

        self.logger.info("Sending request to cancel navigation and control...")
        nav_cancel_ok = self._get_chassis_client().cancel_chassis_nav2()
        control_cancel_ok = self._get_chassis_client().cancel_chassis_control()

        # 只要有一个成功，就认为取消指令已尽力发送
        if nav_cancel_ok or control_cancel_ok:
            return StatusCode.SUCCESS
        return StatusCode.FAIL

    def set_ee_qpos(self, name: str, qpos: np.ndarray) -> StatusCode:
        """控制end effector运动

        Args:
            name (str): ["left_arm_ee", "right_arm_ee"]
            qpos (np.ndarray): 期望位置 0~100

        Returns:
            StatusCode
        """
        qpos_action = {}
        qpos = np.array(qpos)
        qpos_action["action"] = qpos
        if not self.target_supervisor_view._has_arm_ee_client(name, False):
            return StatusCode.FAIL
        if not self.target_supervisor_view._check_qpos_dimension(name, qpos):
            return StatusCode.DIM_MISMATCH

        res = self.target_supervisor_view.get_arm_ee_client(name).invoke_async(
            request_id=-1, content=qpos_action)

        return (StatusCode.SUCCESS if res else StatusCode.FAIL)

    def set_home_qpos(self,
                      home_qpos: np.ndarray,
                      save_to_file: bool = False) -> StatusCode:
        """设置home qpos

        Args:
            home_qpos (np.ndarray): 
            save_to_file (bool, optional): 是否保存至默认路径data/config.json. Defaults to False.

        Returns:
            StatusCode: 
        """
        if not self.target_supervisor_view._check_qpos_limits(
                "full_body", home_qpos):
            return StatusCode.EXCEED_LIMIT
        self.w1_params.home_qpos = home_qpos
        if save_to_file:
            res = StaticUtilityW1.save_home_qpos(self.target_supervisor_view)
            if not res:
                return StatusCode.FAIL
        return StatusCode.SUCCESS

    def get_home_qpos(self) -> np.ndarray:
        return self.w1_params.home_qpos

    def get_qpos_limits(
            self,
            name: str = "full_body"
    ) -> tuple[StatusCode, np.ndarray, np.ndarray]:
        """获取特定部件(支持ee)的关节极限

        Args:
            name (str, optional): ["full_body", "torso", "head", "left_arm", "right_arm", 
                                    "left_arm_ee", "right_arm_ee"]. Defaults to "full_body".

        Returns:
            tuple[StatusCode, np.ndarray, np.ndarray]: 状态码；下极限；上极限
        """
        return self.target_supervisor_view.get_qpos_limits(name)

    def get_current_qpos(self,
                         name: str = "full_body"
                        ) -> tuple[StatusCode, np.ndarray]:
        """获取特定部件(支持ee)的当前关节位置

        Args:
            name (str, optional): ["full_body", "torso", "head", "left_arm", "right_arm", 
                                    "left_arm_ee", "right_arm_ee"]. Defaults to "full_body".

        Returns:
            tuple[StatusCode, np.ndarray]
        """
        return self.target_supervisor_view.get_current_qpos(name)

    def get_current_xpos(
            self,
            name: str,
            is_local: bool = True) -> tuple[StatusCode, np.ndarray]:
        """is_local=True，获取left_arm/right_arm相对于arm base坐标系的笛卡尔位姿
        
            is_local=False，获取["head", "head_flange" 
                                 "left_arm_base", "left_arm", "left_arm_flange", 
                                "right_arm_base", "right_arm", "right_arm_flange"]
                                相对于chassis坐标系的笛卡尔位姿

        Args:
            name (str): ["left_arm", "right_arm"]
            is_local (bool, optional): w.r.t. arm base frame or chassis frame. Defaults to True.

        Returns:
            tuple[StatusCode, np.ndarray]: 
        """
        return self.target_supervisor_view.get_current_xpos(name, is_local)

    def get_xpos(self,
                 name: str,
                 qpos: np.ndarray,
                 is_local: bool = True) -> tuple[StatusCode, np.ndarray]:
        """is_local=True，获取left_arm/right_arm相对于arm base坐标系的笛卡尔位姿，size(qpos)=7
        
            is_local=False，获取["head", "head_flange" 
                                 "left_arm_base", "left_arm", "left_arm_flange", 
                                "right_arm_base", "right_arm", "right_arm_flange"]
                                相对于chassis坐标系的笛卡尔位姿，size(qpos)=20

        Args:
            name (str): ["head", "head_flange"
                        "left_arm_base", "left_arm", "left_arm_flange", 
                        "right_arm_base", "right_arm", "right_arm_flange"]
            qpos (np.ndarray): 
            is_local (bool, optional): w.r.t. arm base frame or chassis frame. Defaults to True.

        Returns:
            tuple[StatusCode, np.ndarray]: 
        """
        return self.target_supervisor_view.get_xpos(name, qpos, is_local)

    def set_tcp(self, name: str, xpos: np.ndarray) -> StatusCode:
        """设置left_arm/right_arm的tcp

        Args:
            name (str): ["left_arm", "right_arm"]
            xpos (np.ndarray): 目标tcp

        Returns:
            StatusCode: 
        """
        # TODO: set tcp in robot client too

        self.cartesian_planner.set_arm_tcp(name, xpos)
        self.cartesian_joint_planner.set_arm_tcp(name, xpos)

        res = self.kine.set_tcp(name, xpos)
        return (StatusCode.SUCCESS if res else StatusCode.FAIL)

    def get_tcp(self, name: str) -> tuple[StatusCode, np.ndarray]:
        """获取left_arm/right_arm的tcp

        Args:
            name (str): ["left_arm", "right_arm"]
        """
        res, value = self.kine.get_tcp(name)
        return (StatusCode.SUCCESS if res else StatusCode.FAIL), value

    def go_home(self, speed_ratio: float = 1.0) -> StatusCode:
        """从当前点MoveJ移动到home qpos

        Args:
            speed_ratio (float, optional): 0.1~1.0. Defaults to 1.0.

        Returns:
            StatusCode: 
        """
        request_id = self.robot_client.move_joint_sync(self.w1_params.home_qpos,
                                                       MotionGroup.WholeBody,
                                                       speed=speed_ratio)
        if request_id == -1:
            self.logger.error("Go home failed!")
            return StatusCode.FAIL
        return StatusCode.SUCCESS

    def stop_move(self, timeout_sec: float = 3.0) -> StatusCode:
        """停止运动

        Args:
            timeout_sec (float, optional): 等待停止超时时间. Defaults to 3.0.

        Args:
            timeout_sec (float, optional): 等待停止超时时间. Defaults to 3.0.

        Returns:
            StatusCode: 
        """
        return self.target_supervisor_view.stop_move(timeout_sec)

    def is_collision(self,
                     qpos: np.ndarray,
                     armlee_qpos: np.ndarray = None,
                     armree_qpos: np.ndarray = None,
                     robot_name: str = "whole_body") -> StatusCode:
        """检查qpos状态是否碰撞"""
        return self.target_supervisor_view.is_collision(qpos, armlee_qpos,
                                                        armree_qpos, robot_name)

    def plan_joint_traj(
            self,
            name: str,
            qpos_array: np.ndarray,
            is_sparse: bool,
            qvel_limits_array: np.ndarray = None
    ) -> tuple[StatusCode, np.ndarray]:
        """对各部件进行关节轨迹规划，可指定路径点的各关节的速度约束;速度表示从上一点到当前点的最大关节速度

        Args:
            name (str): ["full_body", "torso", "head", "left_arm", "right_arm"]
            qpos_array (np.ndarray): 关节路径点  

            qvel_limits_array (np.ndarray, optional): 路径点关节速度约束，省略则按默认关节速度

        Returns:
            tuple[StatusCode, np.ndarray]
        """
        movement = DualArmMovementBaseW1(
            w1_params=self.w1_params,
            supervisor=self.target_supervisor,
            kine_solver=self.kine,
            pin_kine_solver=self._pin_kine,
            collision_checker=self.fcl_checker,
            hybrid_planner=self.hybrid_planner,
            cartesian_planner=self.cartesian_planner,
            cartesian_joint_planner=self.cartesian_joint_planner,
            stop_move_event=self._stop_move_event,
            time_step=self.TIME_STEP)
        return movement.plan_joint_traj(name, qpos_array, is_sparse,
                                        qvel_limits_array)

    def plan_cartesian_traj(
            self,
            name: str,
            xpos_list: list[np.ndarray],
            qpos_seed: np.ndarray,
            qvel_limits_array: np.ndarray = None,
            cartesian_step: float = 0.005) -> tuple[StatusCode, np.ndarray]:
        """对机械臂进行笛卡尔轨迹规划，可指定路径点的各关节的速度约束；速度表示从上一点到当前点的最大关节速度

        Args:
            name (str): ["left_arm", "right_arm"]
            xpos_list (list[np.ndarray]): 笛卡尔路径点
            qpos_seed (np.ndarray): 起始参考关节seed
            qvel_limits_array (np.ndarray, optional): 路径点关节速度约束，省略则按默认关节速度
            cartesian_step (float, optional): 笛卡尔轨迹采样步长 默认0.005m

        Returns:
            tuple[StatusCode, np.ndarray]
        """
        movement = DualArmMovementBaseW1(
            w1_params=self.w1_params,
            supervisor=self.target_supervisor,
            kine_solver=self.kine,
            pin_kine_solver=self._pin_kine,
            collision_checker=self.fcl_checker,
            hybrid_planner=self.hybrid_planner,
            cartesian_planner=self.cartesian_planner,
            cartesian_joint_planner=self.cartesian_joint_planner,
            stop_move_event=self._stop_move_event,
            time_step=self.TIME_STEP)
        return movement.plan_cartesian_traj(name, xpos_list, qpos_seed,
                                            qvel_limits_array, cartesian_step)

    def plan_arm_ee_trigger(self,
                            arm_ee_state_list: list) -> tuple[StatusCode, list]:
        movement = DualArmMovementBaseW1(
            w1_params=self.w1_params,
            supervisor=self.target_supervisor,
            kine_solver=self.kine,
            pin_kine_solver=self._pin_kine,
            collision_checker=self.fcl_checker,
            hybrid_planner=self.hybrid_planner,
            cartesian_planner=self.cartesian_planner,
            cartesian_joint_planner=self.cartesian_joint_planner,
            stop_move_event=self._stop_move_event,
            time_step=self.TIME_STEP)
        return movement.plan_arm_ee_trigger(arm_ee_state_list)

    def execute_joint_traj(
        self,
        qpos_array: np.ndarray,
        left_arm_ee_trigger_list: list[Tuple[int, List[float]]] = None,
        right_arm_ee_trigger_list: list[Tuple[int, List[float]]] = None
    ) -> StatusCode:
        """按10ms间隔执行关节轨迹

        Args:
            qpos_array (np.ndarray): 关节轨迹
            left_arm_ee_trigger_list (list[Tuple[int, List[float]]], optional): 左末端执行器触发序列. Defaults to None.
            right_arm_ee_trigger_list (list[Tuple[int, List[float]]], optional): 右末端执行器触发序列. Defaults to None.

        Returns:
            StatusCode: 
        """
        return self.target_supervisor_view.execute_joint_traj(
            qpos_array, left_arm_ee_trigger_list, right_arm_ee_trigger_list)

    ########## demo定制API ###########
    def get_current_pose(self,
                         name: str,
                         is_local: bool = True) -> tuple[StatusCode, list]:
        """is_local=True，获取left_arm/right_arm当前相对于arm base坐标系的笛卡尔位姿[xyz_mm, rpy_deg]

            is_local=False，获取left_arm/right_arm当前相对于chassis坐标系的笛卡尔位姿[xyz_mm, rpy_deg]

        Args:
            name (str): ["left_arm", "right_arm"]
            is_local (bool, optional): w.r.t. arm base frame or chassis frame. Defaults to True.

        Returns:
            tuple[StatusCode, list]: 
        """
        return self.supervisor_view_sim.get_current_pose(name, is_local)

    def get_pose(self,
                 name: str,
                 qpos: np.ndarray,
                 is_local: bool = True) -> tuple[StatusCode, list]:
        """is_local=True，获取left_arm/right_arm相对于arm base坐标系的笛卡尔位姿[xyz_mm, rpy_deg]，size(qpos)=7

            is_local=False，获取left_arm/right_arm相对于chassis坐标系的笛卡尔位姿[xyz_mm, rpy_deg]，size(qpos)=20

        Args:
            name (str): ["left_arm", "right_arm"]
            qpos (np.ndarray): 
            is_local (bool, optional): w.r.t. arm base frame or chassis frame. Defaults to True.

        Returns:
            tuple[StatusCode, list]: 
        """
        return self.supervisor_view_sim.get_pose(name, qpos, is_local)

    def transfer_grasp_pose(
            self, name: str, pre_grasp_pose: list, pre_torso_qpos: np.ndarray,
            target_torso_qpos: np.ndarray) -> tuple[StatusCode, list]:
        """保持抓取点在全身坐标系的位置，获取torso运动后的相对于机械臂基座坐标系的抓取pose

        Args:
            name (str): left_arm / right_arm
            pre_grasp_pose (list): 抓取前相对于机械臂基座的pose
            pre_torso_qpos (np.ndarray): torso运动前的qpos
            target_torso_qpos (np.ndarray): torso运动后的qpos

        Returns:
            tuple[StatusCode, list]
        """
        return self.supervisor_view_sim.transfer_grasp_pose(
            name, pre_grasp_pose, pre_torso_qpos, target_torso_qpos)

    def plan_dual_arm_cartesian_traj(
        self,
        left_xpos_list: list[np.ndarray],
        right_xpos_list: list[np.ndarray],
        left_qpos_seed: np.ndarray,
        right_qpos_seed: np.ndarray,
        left_qvel_limits_array: np.ndarray = None,
        right_qvel_limits_array: np.ndarray = None,
        cartesian_step: float = 0.005
    ) -> tuple[StatusCode, np.ndarray, np.ndarray]:
        movement = DualArmMoveLineW1(
            w1_params=self.w1_params,
            supervisor=self.target_supervisor,
            kine_solver=self.kine,
            pin_kine_solver=self._pin_kine,
            collision_checker=self.fcl_checker,
            hybrid_planner=self.hybrid_planner,
            cartesian_planner=self.cartesian_planner,
            cartesian_joint_planner=self.cartesian_joint_planner,
            stop_move_event=self._stop_move_event,
            time_step=self.TIME_STEP)
        return movement.plan_dual_arm_cartesian_traj(
            left_xpos_list, right_xpos_list, left_qpos_seed, right_qpos_seed,
            left_qvel_limits_array, right_qvel_limits_array, cartesian_step)

    @time_consuming
    def plan_joint_full_body(
            self,
            start_qpos: np.ndarray,
            start_left_arm_ee_qpos: np.ndarray,
            start_right_arm_ee_qpos: np.ndarray,
            waypoints_dict: dict,
            speed_ratio: float = 1.0,
            is_sparse: bool = True
    ) -> tuple[StatusCode, np.ndarray, list, list]:
        """规划各部件独立关节空间运动
        - 每个key接收多个qpos路径点，key省略视为不控制
        - 同一部件之间的key需要对齐：torso、head、left_xx和right_xx
        - 未指定速度的路径点(None)按speed_ratio补全
        - left_arm_ee/right_arm_ee的每个路径点输入格式为[pre_delay_time, ee_qpos, post_delay_time]
        - key对应的value为 None / [] 视为不控制，其余特殊输入视为不支持

        Args:
            start_qpos (np.ndarray): 规划起始的全身qpos

            start_left_arm_ee_qpos (np.ndarray): 规划起始的left_arm_ee_qpos

            start_right_arm_ee_qpos (np.ndarray): 规划起始的right_arm_ee_qpos   

            waypoints_dict (dict): {"torso": [torso_qpos1,torso_qpos2,], "torso_speed_ratio": [torso_speed1, torso_speed2,],   
                                    "head": [head_qpos1,], "head_speed_ratio": [head_speed1,],   
                                    "left_arm": [left_arm_qpos1,], "left_arm_speed_ratio": [left_arm_speed1,], 
                                    "left_arm_ee": [[0.,left_arm_ee_qpos1,0.5],],  
                                    "right_arm": [right_arm_qpos1,], "right_arm_speed_ratio": [right_arm_speed1], 
                                    "right_arm_ee": [[0.,right_arm_ee_qpos1,0.5],]}

            speed_ratio (float, optional): 相对于默认关节速度的比率. Defaults to 1.0,
                        is_sparse (bool, optional): 输入路径点是否稀疏（点间距是否小于路径blend过渡阈值）.

        Returns:
            tuple[bool, np.ndarray, list, list]: 规划成功与否，全身关节轨迹，左末端执行器轨迹，右末端执行器轨迹
        """
        movement = DualArmMoveJointW1(
            w1_params=self.w1_params,
            supervisor=self.target_supervisor,
            kine_solver=self.kine,
            pin_kine_solver=self._pin_kine,
            collision_checker=self.fcl_checker,
            hybrid_planner=self.hybrid_planner,
            cartesian_planner=self.cartesian_planner,
            cartesian_joint_planner=self.cartesian_joint_planner,
            stop_move_event=self._stop_move_event,
            time_step=self.TIME_STEP)
        return movement.plan_joint_full_body(start_qpos, start_left_arm_ee_qpos,
                                             start_right_arm_ee_qpos,
                                             waypoints_dict, speed_ratio,
                                             is_sparse)

    def move_joint_full_body(self,
                             waypoints_dict: dict,
                             speed_ratio: float = 1.0,
                             is_sparse: bool = True) -> StatusCode:
        """从当前位置规划并执行各部件独立关节空间运动
        - 每个key接收多个qpos路径点，key省略视为不控制
        - 同一部件之间的key需要对齐：torso、head、left_xx和right_xx
        - 未指定速度的路径点(None)按speed_ratio补全
        - left_arm_ee/right_arm_ee的每个路径点输入格式为[pre_delay_time, ee_qpos, post_delay_time]
        - key对应的value为 None / [] 视为不控制，其余特殊输入视为不支持

        Args:
            waypoints_dict (dict): {"torso": [torso_qpos1,torso_qpos2,], "torso_speed_ratio": [torso_speed1, torso_speed2,],   
                                    "head": [head_qpos1,], "head_speed_ratio": [head_speed1,],   
                                    "left_arm": [left_arm_qpos1,], "left_arm_speed_ratio": [left_arm_speed1,], 
                                    "left_arm_ee": [[0.,left_arm_ee_qpos1,0.5],],  
                                    "right_arm": [right_arm_qpos1,], "right_arm_speed_ratio": [right_arm_speed1], 
                                    "right_arm_ee": [[0.,right_arm_ee_qpos1,0.5],]}

            speed_ratio (float, optional): 相对于默认关节速度的比率. Defaults to 1.0,
            is_sparse (bool, optional): 输入路径点是否稀疏（点间距是否小于路径blend过渡阈值）.

        Returns:
            bool
        """
        movement = DualArmMoveJointW1(
            w1_params=self.w1_params,
            supervisor=self.target_supervisor,
            kine_solver=self.kine,
            pin_kine_solver=self._pin_kine,
            collision_checker=self.fcl_checker,
            hybrid_planner=self.hybrid_planner,
            cartesian_planner=self.cartesian_planner,
            cartesian_joint_planner=self.cartesian_joint_planner,
            stop_move_event=self._stop_move_event,
            time_step=self.TIME_STEP)
        return movement.move_joint_full_body(waypoints_dict, speed_ratio,
                                             is_sparse)

    @time_consuming
    def plan_linear_full_body(
            self,
            start_qpos: np.ndarray,
            start_left_arm_ee_qpos: np.ndarray,
            start_right_arm_ee_qpos: np.ndarray,
            waypoints_dict: dict,
            left_qpos_seed: np.ndarray = None,
            right_qpos_seed: np.ndarray = None,
            cartesian_step: float = 0.005,
            speed_ratio: float = 1.0,
            is_local: bool = True,
            is_sync: bool = False) -> tuple[StatusCode, np.ndarray, list, list]:
        """规划各部件独立关节空间运动，arm执行笛卡尔空间运动
        - 每个key接收多个qpos路径点，key省略视为不控制
        - 同一部件之间的key需要对齐：torso、head、left_xx和right_xx
        - 未指定速度的路径点(None)按speed_ratio补全
        - left_arm_ee/right_arm_ee的每个路径点输入格式为[pre_delay_time, ee_qpos, post_delay_time]
        - key对应的value为 None / [] 视为不控制，其余特殊输入视为不支持

        Args:
            start_qpos (np.ndarray): 规划起始的全身qpos

            start_left_arm_ee_qpos (np.ndarray): 规划起始的left_arm_ee_qpos

            start_right_arm_ee_qpos (np.ndarray): 规划起始的right_arm_ee_qpos   

            waypoints_dict (dict): {"torso": [torso_qpos1,torso_qpos2,], "torso_speed_ratio": [torso_speed1, torso_speed2,],   
                                    "head": [head_qpos1,], "head_speed_ratio": [head_speed1,],   
                                    "left_arm": [left_arm_qpos1,], "left_arm_speed_ratio": [left_arm_speed1,], 
                                    "left_arm_ee": [[0.,left_arm_ee_qpos1,0.5],],  
                                    "right_arm": [right_arm_qpos1,], "right_arm_speed_ratio": [right_arm_speed1], 
                                    "right_arm_ee": [[0.,right_arm_ee_qpos1,0.5],]}

            left_qpos_seed (np.ndarray, optional): ik reference seed. Defaults to None.

            right_qpos_seed (np.ndarray, optional): ik reference seed. Defaults to None.
            
            cartesian_step (float, optional): cartesian sample step. Defaults to 0.005.

            speed_ratio (float, optional): 相对于默认关节速度的比率. Defaults to 1.0.

            is_local (bool, optional): w.r.t. arm base frame or chassis frame. Defaults to True.

        Returns:
            tuple[StatusCode, np.ndarray, list, list]: 规划成功与否，全身关节轨迹，左末端执行器轨迹，右末端执行器轨迹
        """
        movement = DualArmMoveLineW1(
            w1_params=self.w1_params,
            supervisor=self.target_supervisor,
            kine_solver=self.kine,
            pin_kine_solver=self._pin_kine,
            collision_checker=self.fcl_checker,
            hybrid_planner=self.hybrid_planner,
            cartesian_planner=self.cartesian_planner,
            cartesian_joint_planner=self.cartesian_joint_planner,
            stop_move_event=self._stop_move_event,
            time_step=self.TIME_STEP)
        return movement.plan_linear_full_body(start_qpos,
                                              start_left_arm_ee_qpos,
                                              start_right_arm_ee_qpos,
                                              waypoints_dict, left_qpos_seed,
                                              right_qpos_seed, cartesian_step,
                                              speed_ratio, is_local, is_sync)

    def move_linear_full_body(self,
                              waypoints_dict: dict,
                              left_qpos_seed: np.ndarray = None,
                              right_qpos_seed: np.ndarray = None,
                              cartesian_step: float = 0.005,
                              speed_ratio: float = 1.0,
                              is_local: bool = True,
                              is_sync: bool = False) -> StatusCode:
        """从当前位置规划各部件独立关节空间运动，arm执行笛卡尔空间运动
        - 每个key接收多个qpos路径点，key省略视为不控制
        - 同一部件之间的key需要对齐：torso、head、left_xx和right_xx
        - 未指定速度的路径点(None)按speed_ratio补全
        - left_arm_ee/right_arm_ee的每个路径点输入格式为[pre_delay_time, ee_qpos, post_delay_time]
        - key对应的value为 None / [] 视为不控制，其余特殊输入视为不支持

        Args:
            waypoints_dict (dict): {"torso": [torso_qpos1,torso_qpos2,], "torso_speed_ratio": [torso_speed1, torso_speed2,],   
                                    "head": [head_qpos1,], "head_speed_ratio": [head_speed1,],   
                                    "left_arm": [left_arm_pose1,], "left_arm_speed_ratio": [left_arm_speed1,], 
                                    "left_arm_ee": [[0.,left_arm_ee_qpos1,0.5],],  
                                    "right_arm": [right_arm_pose1,], "right_arm_speed_ratio": [right_arm_speed1], 
                                    "right_arm_ee": [[0.,right_arm_ee_qpos1,0.5],]} 

            left_qpos_seed (np.ndarray, optional): ik reference seed. Defaults to None.

            right_qpos_seed (np.ndarray, optional): ik reference seed. Defaults to None.

            cartesian_step (float, optional): cartesian sample step. Defaults to 0.005.

            speed_ratio (float, optional): 相对于默认关节速度的比率. Defaults to 1.0.

            is_local (bool, optional): w.r.t. arm base frame or chassis frame. Defaults to True.

        Returns:
            StatusCode
        """
        movement = DualArmMoveLineW1(
            w1_params=self.w1_params,
            supervisor=self.target_supervisor,
            kine_solver=self.kine,
            pin_kine_solver=self._pin_kine,
            collision_checker=self.fcl_checker,
            hybrid_planner=self.hybrid_planner,
            cartesian_planner=self.cartesian_planner,
            cartesian_joint_planner=self.cartesian_joint_planner,
            stop_move_event=self._stop_move_event,
            time_step=self.TIME_STEP)
        return movement.move_linear_full_body(waypoints_dict, left_qpos_seed,
                                              right_qpos_seed, cartesian_step,
                                              speed_ratio, is_local, is_sync)

    def move_torso_linear(self,
                          delta_x_mm: float,
                          delta_z_mm: float,
                          cartesian_step=0.002,
                          speed_ratio: float = 1.) -> StatusCode:
        """腿部从当前位置直线移动相对距离

        Args:
            delta_x_mm (float): x方向(前后)直线相对运动距离
            delta_z_mm (float): z方向(上下)直线相对运动距离
            cartesian_step (float, optional): 直线离散步长. Defaults to 0.002.
            speed_ratio (float, optional): 运动速率. Defaults to 1..

        Returns:
            StatusCode: 
        """
        movement = DualArmMoveLineW1(
            w1_params=self.w1_params,
            supervisor=self.target_supervisor,
            kine_solver=self.kine,
            pin_kine_solver=self._pin_kine,
            collision_checker=self.fcl_checker,
            hybrid_planner=self.hybrid_planner,
            cartesian_planner=self.cartesian_planner,
            cartesian_joint_planner=self.cartesian_joint_planner,
            stop_move_event=self._stop_move_event,
            time_step=self.TIME_STEP)
        return movement.move_torso_linear(delta_x_mm,
                                          delta_z_mm,
                                          cartesian_step=cartesian_step,
                                          speed_ratio=speed_ratio)

    def move_arm_linear(self,
                        name: str,
                        delta_trans_mm: list,
                        cartesian_step=0.005,
                        speed_ratio: float = 1.,
                        along_base_frame: bool = True) -> StatusCode:
        """机械臂从当前位置直线移动相对距离

        Args:
            name (str): ["left_arm", "right_arm"]
            delta_trans_mm (list): 直线相对运动距离
            cartesian_step (float, optional): 直线离散步长. Defaults to 0.005.
            speed_ratio (float, optional): 运动速率. Defaults to 1..
            along_base_frame (bool, optional): 参考chassis_base坐标系/tcp坐标系. Defaults to True.

        Returns:
            StatusCode: 
        """
        movement = DualArmMoveLineW1(
            w1_params=self.w1_params,
            supervisor=self.target_supervisor,
            kine_solver=self.kine,
            pin_kine_solver=self._pin_kine,
            collision_checker=self.fcl_checker,
            hybrid_planner=self.hybrid_planner,
            cartesian_planner=self.cartesian_planner,
            cartesian_joint_planner=self.cartesian_joint_planner,
            stop_move_event=self._stop_move_event,
            time_step=self.TIME_STEP)
        return movement.move_arm_linear(name,
                                        delta_trans_mm,
                                        cartesian_step=cartesian_step,
                                        speed_ratio=speed_ratio,
                                        along_base_frame=along_base_frame)

    def rotate_arm_linear(self,
                          name: str,
                          delta_rot_deg: list,
                          speed_ratio: float = 1.,
                          along_base_frame: bool = True) -> StatusCode:
        """机械臂从当前位置旋转相对角度

        Args:
            name (str): ["left_arm", "right_arm"]
            delta_rot_deg (list): 旋转相对运动距离
            speed_ratio (float, optional): 运动速率. Defaults to 1..
            along_base_frame (bool, optional): 参考chassis_base坐标系/tcp坐标系. Defaults to True.

        Returns:
            StatusCode: 
        """
        movement = DualArmMoveLineW1(
            w1_params=self.w1_params,
            supervisor=self.target_supervisor,
            kine_solver=self.kine,
            pin_kine_solver=self._pin_kine,
            collision_checker=self.fcl_checker,
            hybrid_planner=self.hybrid_planner,
            cartesian_planner=self.cartesian_planner,
            cartesian_joint_planner=self.cartesian_joint_planner,
            stop_move_event=self._stop_move_event,
            time_step=self.TIME_STEP)
        return movement.rotate_arm_linear(name,
                                          delta_rot_deg,
                                          speed_ratio=speed_ratio,
                                          along_base_frame=along_base_frame)

    def attach_object(self,
                      obj_name: str,
                      which_arm: str,
                      arm7_T_object: np.ndarray = None) -> StatusCode:
        """
        Attach grasp object to the arm flange in collision checker and planner

        Args:
            obj_name (str): object name
            which_arm (str): "left" or "right"
            arm7_T_object (np.ndarray, optional, shape=(4, 4)): transformation from object to arm link 7 frame. Defaults to None.

        Returns:
            StatusCode: 
        """

        if arm7_T_object is None:
            res, rotation, translation = self.hybrid_planner.fcl_checker.get_grasp_object_pose(
                name=obj_name)
            if not res:
                self.logger.error(
                    f"Attach object failed, cannot get obstacle pose: {obj_name}"
                )
                return StatusCode.FAIL
            else:
                object_tf = np.eye(4)
                object_tf[:3, :3] = rotation
                object_tf[:3, 3] = translation
            res, current_qpos = self.get_current_qpos()
            if res.value < 0:
                self.logger.error(
                    "Attach object failed, cannot get current qpos")
                return res
            torso_tf, head_tf, arml_tf, armr_tf = self._pin_kine.get_Kinematics(
                real_qpos=current_qpos)
            if which_arm == "left":
                arm7_T_object = np.linalg.inv(arml_tf[-1]) @ object_tf
                res_fcl = self.fcl_checker.attach_object_arml(
                    obj_name=obj_name, arml7_T_object=arm7_T_object)
                res_planner = self.hybrid_planner.attach_object(
                    obj_name=obj_name,
                    which_arm=which_arm,
                    arm7_T_object=arm7_T_object)
                return (StatusCode.SUCCESS if
                        (res_fcl and res_planner) else StatusCode.FAIL)
            elif which_arm == "right":
                arm7_T_object = np.linalg.inv(armr_tf[-1]) @ object_tf
                res_fcl = self.fcl_checker.attach_object_armr(
                    obj_name=obj_name, armr7_T_object=arm7_T_object)
                res_planner = self.hybrid_planner.attach_object(
                    obj_name=obj_name,
                    which_arm=which_arm,
                    arm7_T_object=arm7_T_object)
                return (StatusCode.SUCCESS if
                        (res_fcl and res_planner) else StatusCode.FAIL)
            else:
                self.logger.error(
                    f"Attach object failed, invalid which_arm: {which_arm}")
                return StatusCode.NAME_MISMATCH
        else:
            if arm7_T_object.shape != (4, 4):
                self.logger.error(
                    f"Attach object failed, invalid arm7_T_object shape: {arm7_T_object.shape}"
                )
                return StatusCode.FAIL
            if which_arm == "left":
                res_fcl = self.fcl_checker.attach_object_arml(
                    obj_name=obj_name, arml7_T_object=arm7_T_object)
                res_planner = self.hybrid_planner.attach_object(
                    obj_name=obj_name,
                    which_arm=which_arm,
                    arm7_T_object=arm7_T_object)
                return (StatusCode.SUCCESS if
                        (res_fcl and res_planner) else StatusCode.FAIL)
            elif which_arm == "right":
                res_fcl = self.fcl_checker.attach_object_armr(
                    obj_name=obj_name, armr7_T_object=arm7_T_object)
                res_planner = self.hybrid_planner.attach_object(
                    obj_name=obj_name,
                    which_arm=which_arm,
                    arm7_T_object=arm7_T_object)
                return (StatusCode.SUCCESS if
                        (res_fcl and res_planner) else StatusCode.FAIL)
            else:
                self.logger.error(
                    f"Attach object failed, invalid which_arm: {which_arm}")
                return StatusCode.NAME_MISMATCH

    def detach_object(self,
                      obj_name: str,
                      which_arm: str,
                      arm7_transform: np.ndarray = None) -> StatusCode:
        """
        Detach grasp object from the arm flange in collision checker and planner

        Args:
            obj_name (str): object name
            which_arm (str): "left" or "right"
            arm7_transform (np.ndarray, optional, shape=(4, 4)): transformation of arm 7 link based on the robot base link. Defaults to None.

        Returns:
            StatusCode: 
        """
        if arm7_transform is None:
            res, current_qpos = self.get_current_qpos()
            if res.value < 0:
                self.logger.error(
                    "Detach object failed, cannot get current qpos")
                return res
            torso_tf, head_tf, arml_tf, armr_tf = self._pin_kine.get_Kinematics(
                real_qpos=current_qpos)
            if which_arm == "left":
                arm7_transform = arml_tf[-1]
                res_fcl = self.fcl_checker.detach_object_arml(
                    obj_name=obj_name, arml7_transform=arm7_transform)
                res_planner = self.hybrid_planner.detach_object(
                    obj_name=obj_name,
                    which_arm=which_arm,
                    arm7_transform=arm7_transform)
                return (StatusCode.SUCCESS if
                        (res_fcl and res_planner) else StatusCode.FAIL)
            elif which_arm == "right":
                arm7_transform = armr_tf[-1]
                res_fcl = self.fcl_checker.detach_object_armr(
                    obj_name=obj_name, armr7_transform=arm7_transform)
                res_planner = self.hybrid_planner.detach_object(
                    obj_name=obj_name,
                    which_arm=which_arm,
                    arm7_transform=arm7_transform)
                return (StatusCode.SUCCESS if
                        (res_fcl and res_planner) else StatusCode.FAIL)
            else:
                self.logger.error(
                    f"Detach object failed, invalid which_arm: {which_arm}")
                return StatusCode.NAME_MISMATCH
        else:
            if arm7_transform.shape != (4, 4):
                self.logger.error(
                    f"Detach object failed, invalid arm7_transform shape: {arm7_transform.shape}"
                )
                return StatusCode.DIM_MISMATCH
            if which_arm == "left":
                res_fcl = self.fcl_checker.detach_object_arml(
                    obj_name=obj_name, arml7_transform=arm7_transform)
                res_planner = self.hybrid_planner.detach_object(
                    obj_name=obj_name,
                    which_arm=which_arm,
                    arm7_transform=arm7_transform)
                return (StatusCode.SUCCESS if
                        (res_fcl and res_planner) else StatusCode.FAIL)
            elif which_arm == "right":
                res_fcl = self.fcl_checker.detach_object_armr(
                    obj_name=obj_name, armr7_transform=arm7_transform)
                res_planner = self.hybrid_planner.detach_object(
                    obj_name=obj_name,
                    which_arm=which_arm,
                    arm7_transform=arm7_transform)
                return (StatusCode.SUCCESS if
                        (res_fcl and res_planner) else StatusCode.FAIL)
            else:
                self.logger.error(
                    f"Detach object failed, invalid which_arm: {which_arm}")
                return StatusCode.NAME_MISMATCH

    def transform_obstacle(self, name: str, new_center: np.ndarray,
                           new_quaternion: np.ndarray):
        """Transform obstacle in collision checker

        Args:
            name (str): obstacle name
            new_center (np.ndarray): new center position
            new_quaternion (np.ndarray): new orientation quaternion
        """
        res_fcl = self.fcl_checker.transform_obstacle(name, new_center,
                                                      new_quaternion)
        res_planner = self.hybrid_planner.transform_obstacle(
            name, new_center, new_quaternion)

        return (StatusCode.SUCCESS if res_fcl else StatusCode.FAIL), (
            StatusCode.SUCCESS if res_planner else StatusCode.FAIL)

    def disable_specific_obstacle(self, name: str):
        """Disable specific obstacle in collision checker

        Args:
            name (str): obstacle name
        """
        res_fcl = self.fcl_checker.disable_obstacle(name)
        res_planner = self.hybrid_planner.disable_obstacle(name)

        return (StatusCode.SUCCESS if res_fcl else StatusCode.FAIL), (
            StatusCode.SUCCESS if res_planner else StatusCode.FAIL)

    def enable_specific_obstacle(self, name: str):
        """Enable specific obstacle in collision checker

        Args:
            name (str): obstacle name
        """
        res_fcl = self.fcl_checker.enable_obstacle(name)
        res_planner = self.hybrid_planner.enable_obstacle(name)

        return (StatusCode.SUCCESS if res_fcl else StatusCode.FAIL), (
            StatusCode.SUCCESS if res_planner else StatusCode.FAIL)

    def move_dual_arm_single_task(self,
                                  task_dict: dict,
                                  speed_ratio: float = 1.0) -> StatusCode:
        """
        This function is used to execute a single task, which only suits for 1 torso target, 1 head target, 1 arml target, 1 armr target

        Args:
            task_dict (dict): {
                              "torso": [torso_target_qpos], "troso_speed_ratio": [torso_speed],
                              "head": [head_target_qpos], "head_speed_ratio": [head_speed],
                              "left_arm": [left_arm_target_qpos], "left_arm_speed_ratio": [left_arm_speed],
                              "left_arm_ee": [[float., left_arm_ee_qpos, float]], 
                              "right_arm": [right_arm_target_qpos], "right_arm_speed_ratio": [right_arm_speed],
                              "right_arm_ee": [[float., right_arm_ee_qpos, float]]
                              }
            speed_ratio (float, optional): a ratio to scale the default joint velocity. Defaults to 1.0.
            
        Notes:
            - It will execute the moving of torso and head first, then execute the moving of left and right arms including the end effectors.
        """
        movement = DualArmMovementBaseW1(
            w1_params=self.w1_params,
            supervisor=self.target_supervisor,
            kine_solver=self.kine,
            pin_kine_solver=self._pin_kine,
            collision_checker=self.fcl_checker,
            hybrid_planner=self.hybrid_planner,
            cartesian_planner=self.cartesian_planner,
            cartesian_joint_planner=self.cartesian_joint_planner,
            stop_move_event=self._stop_move_event,
            time_step=self.TIME_STEP)
        return movement.move_dual_arm_single_task(task_dict, speed_ratio)

    def move_single_arm(self, task_dict: dict, speed_ratio: float = 1.0):
        """
        This function is used to execute a single arm task, when the other arm is static

        Args:
            task_dict (dict): {
                              "torso": [torso_target_qpos1, torso_target_qpos2,], "troso_speed_ratio": [torso_speed1, torso_speed2,],
                              "head": [head_target_qpos1, head,target_qpos2,], "head_speed_ratio": [head_speed1, head_speed2,],
                              "left_arm": [left_arm_target_qpos1, left_arm_target_qpos2,], "left_arm_speed_ratio": [left_arm_speed1, left_arm_speed2,],
                              "left_arm_ee": [[float, left_arm_ee_qpos1, float], [float, left_arm_ee_qpos2, float]], 
                              "right_arm": [right_arm_target_qpos1, right_arm_target_qpos2,], "right_arm_speed_ratio": [right_arm_speed1, right_arm_speed2,],
                              "right_arm_ee": [[float, right_arm_ee_qpos1, float], [float, right_arm_ee_qpos2, float]]
                              }
            speed_ratio (float, optional): a ratio to scale the default joint velocity. Defaults to 1.0.

        Notes:
            - The parameter task_dict should only have one arm's target, the other arm's target should be None or []
        """
        movement = DualArmMovementBaseW1(
            w1_params=self.w1_params,
            supervisor=self.target_supervisor,
            kine_solver=self.kine,
            pin_kine_solver=self._pin_kine,
            collision_checker=self.fcl_checker,
            hybrid_planner=self.hybrid_planner,
            cartesian_planner=self.cartesian_planner,
            cartesian_joint_planner=self.cartesian_joint_planner,
            stop_move_event=self._stop_move_event,
            time_step=self.TIME_STEP)
        return movement.move_single_arm(task_dict, speed_ratio)

    def plan_dual_arm_multiple_task(self,
                                    task_dict: list,
                                    speed_ratio: float = 1.0):
        """
        This function is used to execute multiple dual arm tasks sequentially

        Args:
            task_dict (dict): {
                              "torso": [torso_target_qpos], "troso_speed_ratio": [torso_speed],
                              "head": [head_target_qpos], "head_speed_ratio": [head_speed],
                              "left_arm": [left_arm_target_qpos1, left_arm_target_qpos2,], 
                              "left_arm_speed_ratio": [left_arm_speed1, left_arm_speed2,],
                              "left_arm_ee": [[float., left_arm_ee_qpos1, float], [float., left_arm_ee_qpos2, float],],
                              "left_use_collision_check": [bool, bool,]
                              "right_arm": [right_arm_target_qpos1, right_arm_target_qpos2,], 
                              "right_arm_speed_ratio": [right_arm_speed1, right_arm_speed2,],
                              "right_arm_ee": [[float., right_arm_ee_qpos1, float], [float., right_arm_ee_qpos2, float],],
                              "right_use_collision_check": [bool, bool,]
                              }
            speed_ratio (float, optional): a ratio to scale the default joint velocity. Defaults to 1.0.

        Note:
            - "left_use_collision_check" and "right_use_collision_check" are parameters to specify whether use a planning method with collision
                or use move line for the specifc task. Default is True.
        """
        movement = DualArmMovementBaseW1(
            w1_params=self.w1_params,
            supervisor=self.target_supervisor,
            kine_solver=self.kine,
            pin_kine_solver=self._pin_kine,
            collision_checker=self.fcl_checker,
            hybrid_planner=self.hybrid_planner,
            cartesian_planner=self.cartesian_planner,
            cartesian_joint_planner=self.cartesian_joint_planner,
            stop_move_event=self._stop_move_event,
            time_step=self.TIME_STEP)
        return movement.plan_dual_arm_multiple_task(task_dict, speed_ratio)

    def move_dual_arm_multiple_task(self,
                                    task_dict: list,
                                    speed_ratio: float = 1.0):
        movement = DualArmMovementBaseW1(
            w1_params=self.w1_params,
            supervisor=self.target_supervisor,
            kine_solver=self.kine,
            pin_kine_solver=self._pin_kine,
            collision_checker=self.fcl_checker,
            hybrid_planner=self.hybrid_planner,
            cartesian_planner=self.cartesian_planner,
            cartesian_joint_planner=self.cartesian_joint_planner,
            stop_move_event=self._stop_move_event,
            time_step=self.TIME_STEP)
        return movement.move_dual_arm_multiple_task(task_dict, speed_ratio)

    def plan_linear_dual_arm_sync(self,
                                  task_dict: dict,
                                  speed_ratio: float = 1.0,
                                  is_local: bool = True):
        movement = DualArmMoveLineW1(w1_params=self.w1_params,
                                     supervisor=self.target_supervisor,
                                     kine_solver=self.kine,
                                     pin_kine_solver=self._pin_kine,
                                     collision_checker=self.fcl_checker)
        return movement.plan_linear_dual_arm_sync(task_dict=task_dict,
                                                  speed_ratio=speed_ratio,
                                                  is_local=is_local)

    def move_linear_dual_arm_sync(self,
                                  task_dict: dict,
                                  speed_ratio: float = 1.0,
                                  is_local: bool = True):
        movement = DualArmMoveLineW1(w1_params=self.w1_params,
                                     supervisor=self.target_supervisor,
                                     kine_solver=self.kine,
                                     pin_kine_solver=self._pin_kine,
                                     collision_checker=self.fcl_checker)
        return movement.move_linear_dual_arm_sync(task_dict=task_dict,
                                                  speed_ratio=speed_ratio,
                                                  is_local=is_local)
