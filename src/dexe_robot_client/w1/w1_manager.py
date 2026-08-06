'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

import numpy as np

np.set_printoptions(precision=4, suppress=True)
import time
import threading
import pathlib
import json
import atexit
from queue import Queue, Empty
import rclpy
from rclpy.executors import MultiThreadedExecutor
from scipy.spatial.transform import Rotation, Slerp

from ..common import time_consuming, get_logger, DATA_DIR, StatusCode
from ..navigation_client import ChassisClient
from ..robot_client import RobotClientRos2 as RobotClient
from ..robot_client import MotionGroup, Status
from ..end_effector_client import EndEffectorClient
from .trajectory_planner.toppra_trajectory_planner import ToppraTrajectoryPlanner
from .path_planner.blend_path import PathType, PathRnBezier, PathR3xSO3Bezier, weighted_norm, check_SE3_sparse_waypoints
from .path_planner.cartesian_planner import CartesianPlanner
from .path_planner.cartesian_to_joint_planner import CartesianToJointPlanner
from .collision.collision_checker_wrapper import CollisionCheckerWrapper
from .utils.w1_params import W1Params
from .utils.unit_conversion import user_pose_to_xpos, xpos_to_user_pose

from dexe_rp import StatusType, RobotVersion, EndEffectorType, ChassisType, read_config
from dexe_rp import (R2, R2Tangent, R4, R4Tangent, R7, R7Tangent, R20,
                     R20Tangent, SE3, R3xSO3, R3xSO3Tangent,
                     TrajectoryConstraint, BlendType,
                     create_double_s_traj_along_blend_path, create_blend_path)
from dexe_rp import DexforceW1Kinematics

USE_TOPPRA = False


class W1Manager:
    _instance = None
    _lock = threading.Lock()

    def __new__(cls, *args, **kwargs):
        with cls._lock:
            if not cls._instance:
                cls._instance = super().__new__(cls)
        return cls._instance

    @classmethod
    def create(cls, is_sim: bool, use_nav: bool = False):
        """TODO: Add ChassisType rather than use_nav.

                Support left and right different ee type."""
        config = read_config()
        robot_version = RobotVersion.from_robot_and_version_safe(
            config["robot_type"], config["version"])
        left_ee_type = EndEffectorType.from_string_safe(
            config["left_hand_type"])
        right_ee_type = EndEffectorType.from_string_safe(
            config["right_hand_type"])
        chassis_type = ChassisType.from_string_safe(config.get("chassis_type"))
        if chassis_type is None or chassis_type == ChassisType.CORE:
            if use_nav:
                raise ValueError(f"底盘类型错配，请检查！！！")
        return cls(is_sim, robot_version, left_ee_type, right_ee_type, use_nav)

    def __init__(self,
                 is_sim: bool,
                 robot_version: RobotVersion,
                 left_ee_type: EndEffectorType = None,
                 right_ee_type: EndEffectorType = None,
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

        self.chassis_client = None
        self._chassis_client_sim = None
        self._chassis_client_real = None

        self._clients = []

        self._executor = None
        self._node_thread = None

        self._is_sim = None
        self._robot_version = robot_version
        self._left_ee_type = left_ee_type
        self._right_ee_type = right_ee_type
        self._use_nav = use_nav

        self.logger.info(
            f"Initialize W1 manager: " \
            f"\nrobot version: {robot_version}" \
            f"\nleft ee type: {left_ee_type}" \
            f"\nright ee type: {right_ee_type}" \
            f"\nuse_nav: {use_nav}"
        )

        self.w1_params = W1Params(self._robot_version, self._left_ee_type,
                                  self._right_ee_type)

        # 在主线程中正常退出时自动调用
        atexit.register(self._destroy_clients)

        self._init_clients()

        self._init_kine_solver()

        if self.set_sim_mode(is_sim) != StatusCode.SUCCESS:
            if is_sim:
                self.logger.error(f"Set sim mode failed")
                raise RuntimeError("Set sim mode failed")
            self.logger.error(f"Set real mode failed")
            raise RuntimeError("Set real mode failed")

        self.real_torso_idx, self.real_head_idx, self.real_arml_idx, self.real_armr_idx = \
            self.w1_params.torso_index, self.w1_params.head_index, self.w1_params.left_arm_index, self.w1_params.right_arm_index

        self.collision_checker = CollisionCheckerWrapper(
            robot_version=self._robot_version,
            left_end_effector_type=None,
            right_end_effector_type=None,
            verbose=False)
        self._init_cartesian_planner(self._robot_version)

        self._stop_move_event = threading.Event()
        self._stop_move_event.clear()

    def _init_cartesian_planner(self,
                                robot_version: RobotVersion,
                                left_ee_type: EndEffectorType = None,
                                right_ee_type: EndEffectorType = None):
        self.cartesian_joint_planner = CartesianToJointPlanner(
            robot_version, left_ee_type, right_ee_type)
        self.cartesian_planner = CartesianPlanner(robot_version, left_ee_type,
                                                  right_ee_type)

    def _init_clients(self):
        if self._executor is not None:
            self.logger.error(
                f"[{StatusCode.FAIL.value}]:Ros2 already initialized")
            raise RuntimeError("Ros2 already initialized")

        if not rclpy.ok():
            rclpy.init()

        self._clients = []
        self._executor = None
        self._node_thread = None

        try:
            self._robot_client_sim = RobotClient(is_sim=True)
            self._robot_client_real = RobotClient(is_sim=False)
            self.robot_client = self._robot_client_sim
            self._clients.append(self._robot_client_sim)
            self._clients.append(self._robot_client_real)

            if self._use_nav:
                self._chassis_client_real = ChassisClient(is_sim=False)
                self._clients.append(self._chassis_client_real)

            if self._left_ee_type is not None:
                self._left_ee_client_sim = EndEffectorClient(
                    ee_type=self._left_ee_type, is_left=True, is_sim=True)
                self._left_ee_client_real = EndEffectorClient(
                    ee_type=self._left_ee_type, is_left=True, is_sim=False)
                self.left_ee_client = self._left_ee_client_sim
                self._clients.append(self._left_ee_client_sim)
                self._clients.append(self._left_ee_client_real)

            if self._right_ee_type is not None:
                self._right_ee_client_sim = EndEffectorClient(
                    ee_type=self._right_ee_type, is_left=False, is_sim=True)
                self._right_ee_client_real = EndEffectorClient(
                    ee_type=self._right_ee_type, is_left=False, is_sim=False)
                self.right_ee_client = self._right_ee_client_sim
                self._clients.append(self._right_ee_client_sim)
                self._clients.append(self._right_ee_client_real)

        except Exception as e:
            self._destroy_clients()
            self.logger.error(
                f"[{StatusCode.FAIL.value}]:Ros2 node create failed: {e}")
            raise RuntimeError(f"Ros2 node create failed: {e}")

        self._is_sim = True

        try:
            self._executor = MultiThreadedExecutor()
            for node in self._clients:
                self._executor.add_node(node)
            self._node_thread = threading.Thread(target=self._executor.spin,
                                                 daemon=True)
            self._node_thread.start()
        except Exception as e:
            self._destroy_clients()
            self.logger.error(
                f"[{StatusCode.FAIL.value}]:Ros2 node spin failed: {e}")
            raise RuntimeError(f"Ros2 node spin failed: {e}")

        self.logger.info("Ros2 client initialized.")

    def _destroy_clients(self):
        try:
            if self._executor is not None:
                self._executor.shutdown()

            if self._node_thread is not None and self._node_thread.is_alive():
                self._node_thread.join(timeout=5.0)

            if self._executor:
                for node in self._clients:
                    try:
                        self._executor.remove_node(node)
                    except Exception:
                        pass

            for node in self._clients:
                if node.context.ok():
                    node.destroy_node()

            self._clients.clear()
            self._executor = None
            self._node_thread = None

            if rclpy.ok():
                rclpy.shutdown()

        except Exception as e:
            self.logger.error(
                f"[{StatusCode.FAIL.value}]:Ros2 node destroy failed: {e}.")

        self.logger.info(f"Ros2 node destroyed.")

    def _init_kine_solver(self):
        self.kine = DexforceW1Kinematics()
        self.kine.set_user_qpos_limits(self.w1_params.qpos_lb,
                                       self.w1_params.qpos_ub)

        self.logger.info("Initialize kinematic solver.")

    def _has_arm_ee_client(self, name: str, verbose: bool = True) -> bool:
        if name in self.w1_params.arm_ee_names:
            if name == "left_arm_ee" and self.left_ee_client is not None:
                return True
            if name == "right_arm_ee" and self.right_ee_client is not None:
                return True
            if verbose:
                status = StatusCode.LEFT_EE_NOT_CONNECT
                if name == "right_arm_ee":
                    status = StatusCode.RIGHT_EE_NOT_CONNECT
                self.logger.error(
                    f"[{status.value}]:{name} has not initialized")
            return False

        self.logger.error(
            f"[{StatusCode.INPUT_INVALID.value}]:{self.w1_params.arm_ee_names} is supported, except {name}."
        )
        return False

    def _has_chassis_client(self) -> bool:
        if self._use_nav is False:
            self.logger.error(
                f"[{StatusCode.CHASSIS_NOT_USE.value}]:Navigation function is disabled."
            )
            return False
        if self.chassis_client is None:
            self.logger.error(
                f"[{StatusCode.CHASSIS_NOT_USE.value}]:Chassis client has not initialized"
            )
            return False
        # 仅在真正使用到底盘相关接口时检查连接状态
        if not self.chassis_client.is_connected():
            self.logger.error(
                f"[{StatusCode.CHASSIS_NOT_CONNECT.value}]:{self.chassis_client.node_name} is not connected!"
            )
            return False
        return True

    def _get_chassis_client(self):
        if not self._has_chassis_client():
            return None
        return self.chassis_client

    def _get_arm_kine(self, name: str):
        if not self._check_arm_name(name):
            return None
        if name == "left_arm":
            return self.kine.left_arm_kine
        elif name == "right_arm":
            return self.kine.right_arm_kine

    def _get_arm_ee_client(self, name: str):
        if not self._has_arm_ee_client(name):
            return None
        if name == "left_arm_ee":
            return self.left_ee_client
        elif name == "right_arm_ee":
            return self.right_ee_client

    def _check_name(self, name: str) -> bool:
        """仅支持component检查，不包含ee"""
        if name not in self.w1_params.names:
            self.logger.error(
                f"[{StatusCode.INPUT_INVALID.value}]:{self.w1_params.names} is supported, except {name}."
            )
            return False
        return True

    def _check_arm_name(self, name: str) -> bool:
        support_names = self.w1_params.arm_names
        if name not in support_names:
            self.logger.error(
                f"[{StatusCode.INPUT_INVALID.value}]:{support_names} is supported, except {name}."
            )
            return False
        return True

    def _check_qpos_dimension(self, name: str, qpos) -> bool:
        """支持component和ee的检查"""
        if name in self.w1_params.arm_ee_names:
            if not self._has_arm_ee_client(name):
                return False
            dofs = None
            if name == "left_arm_ee":
                dofs = self.w1_params.left_arm_ee_dofs
            if name == "right_arm_ee":
                dofs = self.w1_params.right_arm_ee_dofs
        elif self._check_name(name):
            dofs = self.w1_params.index_infos[name]["dofs"]
        else:
            return False

        if isinstance(qpos, np.ndarray):
            qpos = np.atleast_2d(qpos)
            if qpos.shape[1] == dofs:
                return True
        else:  # list
            if all(len(temp) == dofs for temp in qpos):
                return True

        self.logger.error(
            f"[{StatusCode.DIM_MISMATCH.value}]:{name} qpos dimension mismatch")
        return False

    def _check_qpos_limits(self, name: str, qpos: np.ndarray) -> bool:
        """支持多个qpos检查；支持component和ee的检查"""
        if not self._check_qpos_dimension(name, qpos):
            return False
        res, qpos_lb, qpos_ub = self.get_qpos_limits(name)
        if not res:
            return False
        # qpos = np.atleast_2d(qpos)
        if np.any(qpos < qpos_lb) or np.any(qpos > qpos_ub):
            invalid_idx = np.where((qpos < qpos_lb) | (qpos > qpos_ub))[0][0]
            self.logger.error(f"[{StatusCode.EXCEED_LIMIT.value}]:{name} {invalid_idx+1}th qpos({qpos.tolist()}) " + \
                              f"exceeded limits([{qpos_lb[invalid_idx]}, {qpos_ub[invalid_idx]}]).")
            return False
        return True

    def _check_arm_ee_states(self, ee_name: str,
                             arm_ee_state_list: list) -> bool:
        if len(arm_ee_state_list) == 0:
            self.logger.error(
                f"[{StatusCode.INPUT_INVALID.value}]:Unexpected error, FIXME!")
            return False

        valid_ee_state = [
            ee_state for ee_state in arm_ee_state_list if ee_state is not None
        ]
        if not all(
                isinstance(ee_state, (float, list, float))
                for ee_state in valid_ee_state):
            self.logger.error(
                f"[{StatusCode.INPUT_INVALID.value}]:arm_ee_state should be [pre_delay, ee_qpos, post_delay]."
            )
            return False
        if not self._check_qpos_dimension(
                ee_name, [ee_state[1] for ee_state in valid_ee_state]):
            return False
        return True

    def _save_home_qpos(self, file_path=DATA_DIR / "config.json") -> bool:
        file_path_ = pathlib.Path(file_path)
        if not file_path_.exists() or file_path_.stat().st_size == 0:
            self.logger.error(
                f"[{StatusCode.INPUT_INVALID.value}]:{str(file_path)} invalid.")
            return False

        with open(file_path, "r") as f:
            json_data = json.load(f)

        json_data["home_qpos"] = self.get_home_qpos().tolist()

        with open(file_path, "w") as f:
            json.dump(json_data, f, indent=4, ensure_ascii=False)
        self.w1_params.update_params(file_path)
        return True

    def _qpos_align(
            self, torso_qpos_array: np.ndarray, head_qpos_array: np.ndarray,
            left_arm_qpos_array: np.ndarray,
            right_arm_qpos_array: np.ndarray) -> tuple[bool, np.ndarray]:
        """轨迹对齐，按先运动后等待的模式进行补齐"""
        qpos_array_list = [
            np.atleast_2d(torso_qpos_array),
            np.atleast_2d(head_qpos_array),
            np.atleast_2d(left_arm_qpos_array),
            np.atleast_2d(right_arm_qpos_array)
        ]
        if all(q.size == 0 for q in qpos_array_list):
            self.logger.error(
                f"[{StatusCode.FAIL.value}]:No qpos to be aligned")
            return False, None

        max_count = max(q.shape[0] for q in qpos_array_list)

        temp_arrays = []
        for qpos_array in qpos_array_list:
            pad_count = max_count - qpos_array.shape[0]
            if pad_count > 0:
                padded = np.vstack(
                    [qpos_array,
                     np.tile(qpos_array[-1], (pad_count, 1))])
            else:
                padded = qpos_array
            temp_arrays.append(padded)

        return True, np.hstack(temp_arrays)

    def load_fcl_environment(self, file_path: str = None) -> None:
        # TODO: rename the function, here to keep function name same as before
        """
        file_path: the path of fcl environment json file path
        """
        if file_path is None:
            raise ValueError(f"file path should be provided")
        else:
            env_json_path = file_path
        self.collision_checker.collision_handler.load_environment_from_json(
            env_json_path)

    ######## 基础API #############
    def connect(self, timeout_sec: float = 0.5) -> StatusCode:
        """连接client(sim/real)

        Args:
            timeout_sec (float, optional): 等待连接超时时间. Defaults to 0.5.

        Returns:
            StatusCode: 
        """
        return self.is_connected(timeout_sec)

    def is_connected(self, timeout_sec: float = 0.1) -> StatusCode:
        """检查所有client是否处于连接状态

        Args:
            timeout_sec (float, optional): 等待连接超时时间. Defaults to 0.5.

        Returns:
            StatusCode: 
        """
        if not self.robot_client.is_connected(timeout_sec):
            status = StatusCode.ROBOT_NOT_CONNECT
            self.logger.error(
                f"[{status.value}]:{self.robot_client.node_name} is not connected!"
            )
            return status
        if self._has_arm_ee_client("left_arm_ee", False):
            if not self.left_ee_client.is_connected(timeout_sec):
                status = StatusCode.LEFT_EE_NOT_CONNECT
                self.logger.error(
                    f"[{status.value}]:{self.left_ee_client.node_name} is not connected!"
                )
                return status
        if self._has_arm_ee_client("right_arm_ee", False):
            if not self.right_ee_client.is_connected(timeout_sec):
                status = StatusCode.RIGHT_EE_NOT_CONNECT
                self.logger.error(
                    f"[{status.value}]:{self.right_ee_client.node_name} is not connected!"
                )
                return status
        if self._use_nav and not self._is_sim:
            if not self.chassis_client.is_connected():
                status = StatusCode.CHASSIS_NOT_CONNECT
                self.logger.error(
                    f"[{status.value}]:{self.chassis_client.node_name} is not connected!"
                )
                return status
        return StatusCode.SUCCESS

    def enable(self, timeout_sec: float = 0.5) -> StatusCode:
        return self.is_enabled()

    def is_enabled(self) -> StatusCode:
        """检查robot是否处于使能状态

        Returns:
            StatusCode: 
        """
        if not self.robot_client.is_connected(0.5):
            status = StatusCode.ROBOT_NOT_CONNECT
            self.logger.error(
                f"[{status.value}]:{self.robot_client.node_name} is not connected!"
            )
            return status
        if self.robot_client.status == Status.Init:
            status = StatusCode.ROBOT_NOT_ENABLE
            self.logger.error(
                f"[{status.value}]:{self.robot_client.node_name} is not enabled, robot status: {self.robot_client.robot_statusword}."
            )
            return status
        return StatusCode.SUCCESS

    def clear_robot_error(self) -> StatusCode:
        """清除机器人错误

        Returns:
            StatusCode: 
        """
        if self._is_sim:
            status = StatusCode.SIM_MODE_NOT_SUPPORT
            self.logger.error(
                f"[{status.value}]:Clear error only support real mode.")
            return status
        res = self.robot_client.clear_error()
        if res == -1:
            status = StatusCode.ROBOT_CLEAR_ERROR_FAIL
            self.logger.error(f"[{status.value}]:Clear robot error failed!")
            return status
        return StatusCode.SUCCESS

    def set_drag_mode(self, enable_drag: bool) -> StatusCode:
        """开关机器人拖动示教模式

        Args:
            enable_drag (bool): 是否开启拖动示教模式

        Returns:
            StatusCode: 
        """
        if self._is_sim:
            status = StatusCode.SIM_MODE_NOT_SUPPORT
            self.logger.error(
                f"[{status.value}]:Drag mode only support real mode.")
            return status
        res = self.robot_client.set_drag_mode(enable_drag)
        if res == -1:
            status = StatusCode.ROBOT_SET_DRAG_FAIL
            self.logger.error(f"[{status.value}]:Set drag mode failed!")
            return status
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
            status = StatusCode.SIM_MODE_NOT_SUPPORT
            self.logger.error(
                f"[{status.value}]:Safe level only support real mode.")
            return status
        res = self.robot_client.set_safe_level(safe_level)
        if res == -1:
            status = StatusCode.ROBOT_SET_SAFE_LEVEL_FAIL
            self.logger.error(f"[{status.value}]:Set safe level failed!")
            return status
        return StatusCode.SUCCESS

    def set_sim_mode(self, enable_sim: bool) -> StatusCode:
        """设置仿真/真实模式"""
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
            status = StatusCode.ROBOT_NOT_CONNECT
            self.logger.error(
                f"[{status.value}]:{target_robot_client.node_name} is not connected!"
            )
            return status

        if self._left_ee_type is not None:
            if not target_left_ee_client.is_connected():
                status = StatusCode.LEFT_EE_NOT_CONNECT
                self.logger.error(
                    f"[{status.value}]:{target_left_ee_client.node_name} is not connected!"
                )
                return status

        if self._right_ee_type is not None:
            if not target_right_ee_client.is_connected():
                status = StatusCode.RIGHT_EE_NOT_CONNECT
                self.logger.error(
                    f"[{status.value}]:{target_right_ee_client.node_name} is not connected!"
                )
                return status

        # 检查current client
        if not self.robot_client.is_connected():
            self.logger.error(
                f"[{status.value}]:{self.robot_client.node_name} is not connected!"
            )
            status = StatusCode.ROBOT_NOT_CONNECT
            return status

        if self._left_ee_type is not None:
            if not self.left_ee_client.is_connected():
                status = StatusCode.LEFT_EE_NOT_CONNECT
                self.logger.error(
                    f"[{status.value}]:{self.left_ee_client.node_name} is not connected!"
                )
                return status

        if self._right_ee_type is not None:
            if not self.right_ee_client.is_connected():
                status = StatusCode.RIGHT_EE_NOT_CONNECT
                self.logger.error(
                    f"[{status.value}]:{self.right_ee_client.node_name} is not connected!"
                )
                return status

        self.robot_client = target_robot_client
        if self._left_ee_type is not None:
            self.left_ee_client = target_left_ee_client
        if self._right_ee_type is not None:
            self.right_ee_client = target_right_ee_client
        if self._use_nav and target_chassis_client is not None:
            self.chassis_client = target_chassis_client
        self._is_sim = enable_sim
        if self._use_nav and target_chassis_client is not None:
            self.chassis_client = target_chassis_client
        self.logger.info(f"W1 manager set to {target_name} mode.")
        return StatusCode.SUCCESS

    def _check_chassis(self) -> StatusCode:
        if not self._use_nav:
            status = StatusCode.CHASSIS_NOT_USE
            self.logger.error(
                f"[{status.value}]:Navigation function is disabled.")
            return status
        if not self._has_chassis_client():
            return StatusCode.CHASSIS_NOT_CONNECT
        return StatusCode.SUCCESS

    def move_chassis_nav2(self, base_position: np.ndarray) -> StatusCode:
        """移动底盘至目标位置，使用导航栈"""
        status = self._check_chassis()
        if status != StatusCode.SUCCESS:
            return status
        res = self._get_chassis_client().set_chassis_nav2_position(
            base_position)
        if not res:
            status = StatusCode.CHASSIS_MOVE_NAV2_FAIL
            self.logger.error(f"[{status.value}]:Chassis nav2 move failed")
            return status
        return StatusCode.SUCCESS

    def move_chassis_road_navigation(self,
                                     waypoints: list,
                                     frame_id: str = "map",
                                     timeout_sec: float = 10.0,
                                     behavior_tree: str = "") -> StatusCode:
        """路网/多路点导航，waypoints 为 [x, y, yaw_rad] 列表。"""
        status = self._check_chassis()
        if status != StatusCode.SUCCESS:
            return status
        res = self._get_chassis_client().set_chassis_road_navigation(
            waypoints, frame_id, timeout_sec, behavior_tree)
        if not res:
            status = StatusCode.CHASSIS_MOVE_ROAD_NAV_FAIL
            self.logger.error(f"[{status.value}]:Chassis road nav move failed")
            return status
        return StatusCode.SUCCESS

    def move_chassis_ctrl(self, base_position: np.ndarray) -> StatusCode:
        """移动底盘至目标位置，使用底盘控制接口"""
        status = self._check_chassis()
        if status != StatusCode.SUCCESS:
            return status
        res = self._get_chassis_client().set_chassis_control_position(
            base_position)
        if not res:
            status = StatusCode.CHASSIS_MOVE_CTRL_FAIL
            self.logger.error(f"[{status.value}]:Chassis control move failed")
            return status
        return StatusCode.SUCCESS

    def get_current_chassis_pose(self) -> tuple[StatusCode, np.ndarray]:
        """获取底盘当前位姿[x(m), y(m), yaw(angle)]"""
        status = self._check_chassis()
        if status != StatusCode.SUCCESS:
            return status, None
        res, chassis_pos = self._get_chassis_client().get_chassis_position()
        if not res:
            status = StatusCode.CHASSIS_GET_POSE_FAIL
            self.logger.error(f"[{status.value}]:Get chassis pose failed")
            return status, None
        return StatusCode.SUCCESS, chassis_pos

    def get_current_battery_percentage(self) -> tuple[StatusCode, float]:
        """获取当前电池电量百分比"""
        status = self._check_chassis()
        if status != StatusCode.SUCCESS:
            return status, None
        res, value = self._get_chassis_client().get_battery_percentage()
        if not res:
            status = StatusCode.CHASSIS_GET_BATTERY_FAIL
            self.logger.error(f"[{status.value}]:Get battery percentage failed")
            return status, None
        return StatusCode.SUCCESS, value

    def get_front_obstacle(self) -> tuple[StatusCode, bool]:
        """获取前方障碍物检测状态
        
        Returns:
            tuple[StatusCode, bool]: 状态码和障碍物检测结果
                - 如果检测到障碍物则返回 (StatusCode.SUCCESS, True)
                - 如果没有障碍物则返回 (StatusCode.SUCCESS, False)
                - 如果导航功能未启用或客户端未初始化则返回 (StatusCode.FAIL, False)
        """
        status = self._check_chassis()
        if status != StatusCode.SUCCESS:
            return status, False
        has_obstacle = self._get_chassis_client().get_front_obstacle()
        return StatusCode.SUCCESS, has_obstacle

    def get_nav_status(self) -> tuple[StatusCode, str]:
        """获取导航状态"""
        if not self._use_nav:
            status = StatusCode.CHASSIS_NOT_USE
            self.logger.error(
                f"[{status.value}]:Navigation function is disabled.")
            return status, "DISABLED"
        if not self._has_chassis_client():
            return StatusCode.CHASSIS_NOT_CONNECT, "NOT_INITIALIZED"
        return StatusCode.SUCCESS, self._get_chassis_client().get_nav_state()

    def get_road_navigation_status(self) -> tuple[StatusCode, str]:
        """获取当前路网/多路点导航状态。"""
        if not self._use_nav:
            status = StatusCode.CHASSIS_NOT_USE
            self.logger.error(
                f"[{status.value}]:Navigation function is disabled.")
            return status, "DISABLED"
        if not self._has_chassis_client():
            return StatusCode.CHASSIS_NOT_CONNECT, "NOT_INITIALIZED"
        return StatusCode.SUCCESS, self._get_chassis_client(
        ).get_road_navigation_status()

    def get_road_navigation_result(self) -> tuple[StatusCode, bool, str]:
        """获取最近一次路网/多路点导航结果。"""
        if not self._use_nav:
            status = StatusCode.CHASSIS_NOT_USE
            self.logger.error(
                f"[{status.value}]:Navigation function is disabled.")
            return status, False, "DISABLED"
        if not self._has_chassis_client():
            return StatusCode.CHASSIS_NOT_CONNECT, False, "NOT_INITIALIZED"
        success, message = self._get_chassis_client(
        ).get_road_navigation_result()
        return StatusCode.SUCCESS, success, message

    def get_all_nav_points(
            self) -> tuple[StatusCode, dict[str, tuple[float, float, float]]]:
        """获取所有路网导航点，返回点位名称到 x/y/theta_rad 的映射。"""
        status = self._check_chassis()
        if status != StatusCode.SUCCESS:
            return status, {}
        nav_points = self._get_chassis_client().get_all_nav_points()
        if not nav_points:
            status = StatusCode.CHASSIS_GET_NAV_POINTS_FAIL
            self.logger.error(f"[{status.value}]:Get all nav points failed")
            return status, nav_points
        return StatusCode.SUCCESS, nav_points

    def get_control_status(self) -> tuple[StatusCode, str]:
        """获取底盘控制状态"""
        if not self._use_nav:
            status = StatusCode.CHASSIS_NOT_USE
            self.logger.error(
                f"[{status.value}]:Navigation function is disabled.")
            return status, "DISABLED"
        if not self._has_chassis_client():
            return StatusCode.CHASSIS_NOT_CONNECT, "NOT_INITIALIZED"
        return StatusCode.SUCCESS, self._get_chassis_client(
        ).get_control_status()

    def set_auto_charging(self) -> StatusCode:
        """发送回充 action"""
        status = self._check_chassis()
        if status != StatusCode.SUCCESS:
            return status
        res = self._get_chassis_client().set_chassis_auto_charging()
        if not res:
            status = StatusCode.CHASSIS_SET_AUTO_CHARGING_FAIL
            self.logger.error(f"[{status.value}]:Set auto charging failed")
            return status
        return StatusCode.SUCCESS

    def cancel_auto_charging(self) -> StatusCode:
        """取消当前回充 action"""
        status = self._check_chassis()
        if status != StatusCode.SUCCESS:
            return status
        res = self._get_chassis_client().cancel_auto_charging()
        if not res:
            status = StatusCode.CHASSIS_CANCEL_AUTO_CHARGING_FAIL
            self.logger.error(f"[{status.value}]:Cancel auto charging failed")
            return status
        return StatusCode.SUCCESS

    def cancel_chassis_motion(self) -> StatusCode:
        """统一取消底盘运动，同时调用取消导航和取消控制。"""
        status = self._check_chassis()
        if status != StatusCode.SUCCESS:
            return status

        self.logger.info(
            "Sending request to cancel navigation, control and auto charging..."
        )
        nav_cancel_ok = self._get_chassis_client().cancel_chassis_nav2()
        road_nav_cancel_ok = self._get_chassis_client().cancel_road_navigation()
        graph_points_nav_cancel_ok = (
            self._get_chassis_client().cancel_graph_points_navigation())
        control_cancel_ok = self._get_chassis_client().cancel_chassis_control()
        auto_charging_cancel_ok = (
            self._get_chassis_client().cancel_auto_charging())

        # 只要有一个成功，就认为取消指令已尽力发送
        if (nav_cancel_ok or road_nav_cancel_ok or graph_points_nav_cancel_ok or
                control_cancel_ok or auto_charging_cancel_ok):
            return StatusCode.SUCCESS
        status = StatusCode.CHASSIS_CANCEL_MOTION_FAIL
        self.logger.error(f"[{status.value}]:Cancel chassis motion failed")
        return status

    def set_chassis_footprint(self, vertices: list) -> StatusCode:
        """设置底盘足迹，以底盘中心为原点，其他四个点为四边形四角点
        vertices: [[x1,y1], [x2,y2], [x3,y3], [x4,y4]]
        """
        status = self._check_chassis()
        if status != StatusCode.SUCCESS:
            return status
        if len(vertices) != 4:
            status = StatusCode.INPUT_INVALID
            self.logger.error(
                f"[{status.value}]:Footprint must have 4 vertices")
            return status
        self.logger.info("Sending request to set chassis footprint...")
        set_footprint_ok = self._get_chassis_client().set_footprint(vertices)
        if set_footprint_ok:
            return StatusCode.SUCCESS
        status = StatusCode.CHASSIS_SET_FOOTPRINT_FAIL
        self.logger.error(f"[{status.value}]:Set chassis footprint failed")
        return status

    def set_chassis_nav_tolerance(self, xy_tol: float,
                                  yaw_tol: float) -> StatusCode:
        """设置底盘导航容差"""
        status = self._check_chassis()
        if status != StatusCode.SUCCESS:
            return status
        set_nav_tolerance = self._get_chassis_client().set_goal_tolerance(
            xy_tol, yaw_tol)
        if set_nav_tolerance:
            return StatusCode.SUCCESS
        status = StatusCode.CHASSIS_SET_NAV_TOLERANCE_FAIL
        self.logger.error(f"[{status.value}]:Set chassis nav tolerance failed")
        return status

    def set_chassis_inflation_radius(self, radius: float) -> StatusCode:
        """设置底盘膨胀半径"""
        status = self._check_chassis()
        if status != StatusCode.SUCCESS:
            return status
        set_chassis_inflation = self._get_chassis_client().set_inflation_radius(
            radius)
        if set_chassis_inflation:
            return StatusCode.SUCCESS
        status = StatusCode.CHASSIS_SET_INFLATION_RADIUS_FAIL
        self.logger.error(
            f"[{status.value}]:Set chassis inflation radius failed")
        return status

    def set_follow_velocity_limits(self, vx_max: float,
                                   wz_max: float) -> StatusCode:
        """
        同时设置 FollowPath 线速度 & 角速度上限
        """
        status = self._check_chassis()
        if status != StatusCode.SUCCESS:
            return status
        set_follow_velocity_limits = self._get_chassis_client(
        ).set_follow_velocity_limits(vx_max, wz_max)
        if set_follow_velocity_limits:
            return StatusCode.SUCCESS
        status = StatusCode.CHASSIS_SET_FOLLOW_VELOCITY_LIMITS_FAIL
        self.logger.error(f"[{status.value}]:Set follow velocity limits failed")
        return status

    # TODO: 修改为set_ee_pos更符合语义；提供pos <-> qpos之间的转换后，可增加set_ee_qpos接口
    def set_ee_qpos(self, name: str, qpos: np.ndarray) -> StatusCode:
        """控制end effector运动

        Args:
            name (str): ["left_arm_ee", "right_arm_ee"]
            qpos (np.ndarray): 期望位置 0~100

        Returns:
            StatusCode
        """
        qpos_ = np.array(qpos)
        if not self._has_arm_ee_client(name):
            return StatusCode.FAIL
        if not self._check_qpos_dimension(name, qpos_):
            return StatusCode.DIM_MISMATCH
        res = self._get_arm_ee_client(name).set_pos(qpos_.tolist())
        if not res:
            status = StatusCode.EE_MOVE_FAIL
            self.logger.error(f"[{status.value}]:Set {name} qpos failed")
            return status
        return StatusCode.SUCCESS

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
        if not self._check_qpos_limits("full_body", home_qpos):
            return StatusCode.EXCEED_LIMIT
        self.w1_params.home_qpos = home_qpos
        if save_to_file:
            res = self._save_home_qpos()
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
        if name in self.w1_params.arm_ee_names:
            if not self._has_arm_ee_client(name):
                return StatusCode.FAIL, None, None
            qpos_lb = None
            qpos_ub = None
            if name == "left_arm_ee":
                qpos_lb = np.array(self.w1_params._left_ee_params.position.lb)
                qpos_ub = np.array(self.w1_params._left_ee_params.position.ub)
            if name == "right_arm_ee":
                qpos_lb = np.array(self.w1_params._right_ee_params.position.lb)
                qpos_ub = np.array(self.w1_params._right_ee_params.position.ub)

        elif not self._check_name(name):
            return StatusCode.NAME_MISMATCH, None, None
        else:
            qpos_lb = self.w1_params.qpos_lb[self.w1_params.index_infos[name]
                                             ["index"]]
            qpos_ub = self.w1_params.qpos_ub[self.w1_params.index_infos[name]
                                             ["index"]]
        return StatusCode.SUCCESS, qpos_lb, qpos_ub

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
        support_names = list(self.w1_params.names) + self.w1_params.arm_ee_names
        if name not in support_names:
            status = StatusCode.NAME_MISMATCH
            self.logger.error(
                f"[{status.value}]:{support_names} is supported, except {name}."
            )
            return status, None

        if name in self.w1_params.arm_ee_names:
            if not self._has_arm_ee_client(name):
                return StatusCode.FAIL, None
            res, info = self._get_arm_ee_client(name).get_info()
            if not res:
                status = StatusCode.EE_GET_INFO_FAIL
                self.logger.error(f"[{status.value}]:Get {name} info failed.")
                return status, None
            return StatusCode.SUCCESS, np.array(info.joint_position)

        robot_info = self.robot_client.get_info()
        if robot_info.joint_position_cmd is None and robot_info.joint_position is None:
            status = StatusCode.ROBOT_STATUS_INVALID
            self.logger.error(
                f"[{status.value}]:Get current robot info failed, status: {robot_info.status}."
            )
            return status, None
        if robot_info.joint_position_cmd is None or len(
                robot_info.joint_position_cmd) == 0:
            cur_qpos = np.array(robot_info.joint_position)
        else:
            cur_qpos = np.array(robot_info.joint_position_cmd)

        if name == "full_body":
            return StatusCode.SUCCESS, cur_qpos
        return StatusCode.SUCCESS, cur_qpos[self.w1_params.index_infos[name]
                                            ["index"]]

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
        if is_local:
            res, cur_qpos = self.get_current_qpos(name)
            if res.value < 0:
                return res, None
        else:
            res, cur_qpos = self.get_current_qpos("full_body")
            if res.value < 0:
                return res, None
        return self.get_xpos(name, cur_qpos, is_local)

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
        if is_local:
            if not self._check_qpos_dimension(name, qpos):
                return StatusCode.DIM_MISMATCH, None

            arm_kine = self._get_arm_kine(name)
            if arm_kine is None:
                return StatusCode.NAME_MISMATCH, None

            value = arm_kine.get_fk(qpos)
            return StatusCode.SUCCESS, value
        else:
            if not self._check_qpos_dimension("full_body", qpos):
                return StatusCode.DIM_MISMATCH, None

            res, value = self.kine.get_fk(name, qpos)
            if not res:
                status = StatusCode.FK_FAIL
                self.logger.error(f"[{status.value}]:Get {name} fk failed.")
                return status, None
            return StatusCode.SUCCESS, value

    def set_tcp(self, name: str, xpos: np.ndarray) -> StatusCode:
        """设置left_arm/right_arm的tcp

        Args:
            name (str): ["left_arm", "right_arm"]
            xpos (np.ndarray): 目标tcp

        Returns:
            StatusCode: 
        """
        # TODO: set tcp in robot client too
        self.logger.info(f"Set {name} tcp: {xpos.tolist()}")
        self.cartesian_planner.set_arm_tcp(name, xpos)
        self.cartesian_joint_planner.set_arm_tcp(name, xpos)
        res = self.kine.set_tcp(name, xpos)
        if not res:
            status = StatusCode.SET_TCP_FAIL
            self.logger.error(f"[{status.value}]:Set {name} tcp failed.")
            return status
        return StatusCode.SUCCESS

    def get_tcp(self, name: str) -> tuple[StatusCode, np.ndarray]:
        """获取left_arm/right_arm的tcp

        Args:
            name (str): ["left_arm", "right_arm"]
        """
        res, value = self.kine.get_tcp(name)
        if not res:
            status = StatusCode.GET_TCP_FAIL
            self.logger.error(f"[{status.value}]:Get {name} tcp failed.")
            return status, None
        return StatusCode.SUCCESS, value

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
            status = StatusCode.ROBOT_MOVE_FAIL
            self.logger.error(f"[{status.value}]:Go home failed!")
            return status
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
        self.logger.info("Stop move by user!")
        self._stop_move_event.set()
        if self.robot_client.stop_move() == -1:
            status = StatusCode.STOP_MOVE_FAIL
            self.logger.error(f"[{status.value}]:Stop move by user failed!")
            return status
        t0 = time.time()
        while self.robot_client.status != Status.Idle:
            if time.time() - t0 > timeout_sec:
                status = StatusCode.TIME_OUT
                self.logger.error(f"[{status.value}]:Stop move wait timeout!")
                return status
            time.sleep(2 * self.TIME_STEP)
        return StatusCode.SUCCESS

    def set_collision_checker_verbose(self, verbose: bool):
        self.collision_checker.set_verbose(verbose=verbose)

    def is_collision(self,
                     qpos: np.ndarray,
                     armlee_qpos: np.ndarray = None,
                     armree_qpos: np.ndarray = None,
                     robot_name: str = "whole_body",
                     save_scene: bool = True) -> StatusCode:
        """检查qpos状态是否碰撞"""
        if not self._check_qpos_dimension("full_body", qpos):
            return StatusCode.DIM_MISMATCH
        if armlee_qpos is not None and not self._check_qpos_dimension(
                "left_arm_ee", armlee_qpos):
            return StatusCode.DIM_MISMATCH
        if armree_qpos is not None and not self._check_qpos_dimension(
                "right_arm_ee", armree_qpos):
            return StatusCode.DIM_MISMATCH
        res = self.collision_checker.check_collision(qpos=qpos,
                                                     left_ee_pulse=armlee_qpos,
                                                     right_ee_pulse=armree_qpos,
                                                     robot_name=robot_name,
                                                     save_scene=save_scene)
        # StatusCode.SUCCESS when no collision, StatusCode.COLLISION_FAIL when collision happened
        return res

    def _is_traj_collision(self,
                           qpos_array: np.ndarray,
                           left_arm_start_ee_qpos: np.ndarray | list[float],
                           right_arm_start_ee_qpos: np.ndarray | list[float],
                           left_arm_ee_trigger_list: list[tuple[int,
                                                                list]] = None,
                           right_arm_ee_trigger_list: list[tuple[int,
                                                                 list]] = None,
                           step: int = 5,
                           save_scene: bool = True) -> StatusCode:
        """检查轨迹是否碰撞
        Args:
            qpos_array (np.ndarray): 10ms间隔的轨迹
            left_arm_start_ee_qpos (np.ndarray|list[float]): 左臂末端执行器起始位置
            right_arm_start_ee_qpos (np.ndarray|list[float]): 右臂末端执行器起始位置
            left_arm_ee_trigger_list (list[tuple[int, list]], optional): 左臂末端执行器触发列表，格式[(idx, qpos), ...]. Defaults to None.
            right_arm_ee_trigger_list (list[tuple[int, list]], optional): 右臂末端执行器触发列表，格式[(idx, qpos), ...]. Defaults to None.
            step (int, optional): 检测步长，每步10ms. Defaults to 5.
        Returns:
            StatusCode: 
        
        Notes:
            - if there is no ee client, left_arm_start_ee_qpos and right_arm_start_ee_qpos should be None
        """
        # NOTE: 最大速度 伸直构型下 仅基座关节运动导致的末端位移 5 * 0.01s * 1rad/s * 0.7m = 3.5cm

        # if self._has_arm_ee_client("left_arm_ee",
        #                            False) and self._has_arm_ee_client(
        #                                "right_arm_ee", False):
        #     if left_arm_start_ee_qpos is None or not self._check_qpos_dimension(
        #             "left_arm_ee", left_arm_start_ee_qpos):
        #         raise ValueError(
        #             "Left arm start ee qpos is required and should have correct dimension."
        #         )
        #     if right_arm_start_ee_qpos is None or not self._check_qpos_dimension(
        #             "right_arm_ee", right_arm_start_ee_qpos):
        #         raise ValueError(
        #             "Right arm start ee qpos is required and should have correct dimension."
        #         )
        #     cur_left_ee_qpos = np.asarray(left_arm_start_ee_qpos)
        #     cur_right_ee_qpos = np.asarray(right_arm_start_ee_qpos)

        #     if left_arm_ee_trigger_list is None:
        #         left_arm_ee_qpos_array = np.tile(cur_left_ee_qpos,
        #                                          (qpos_array.shape[0], 1))
        #     else:
        #         left_arm_ee_qpos_array = np.tile(cur_left_ee_qpos,
        #                                          (qpos_array.shape[0], 1))
        #         for idx, target_qpos in left_arm_ee_trigger_list:
        #             left_arm_ee_qpos_array[idx:, :] = np.tile(
        #                 np.asarray(target_qpos), (qpos_array.shape[0] - idx, 1))
        #     if right_arm_ee_trigger_list is None:
        #         right_arm_ee_qpos_array = np.tile(cur_right_ee_qpos,
        #                                           (qpos_array.shape[0], 1))
        #     else:
        #         right_arm_ee_qpos_array = np.tile(cur_right_ee_qpos,
        #                                           (qpos_array.shape[0], 1))
        #         for idx, target_qpos in right_arm_ee_trigger_list:
        #             right_arm_ee_qpos_array[idx:, :] = np.tile(
        #                 np.asarray(target_qpos), (qpos_array.shape[0] - idx, 1))
        #     qpos_array_check = np.vstack((qpos_array[::step], qpos_array[-1]))
        #     left_arm_ee_qpos_array_check = np.vstack(
        #         (left_arm_ee_qpos_array[::step], left_arm_ee_qpos_array[-1]))
        #     right_arm_ee_qpos_array_check = np.vstack(
        #         (right_arm_ee_qpos_array[::step], right_arm_ee_qpos_array[-1]))
        #     for idx in range(qpos_array_check.shape[0]):
        #         if self.is_collision(
        #                 qpos_array_check[idx],
        #                 left_arm_ee_qpos_array_check[idx],
        #                 right_arm_ee_qpos_array_check[idx],
        #                 save_scene=save_scene) == StatusCode.COLLISION_FAIL:
        #             self.logger.error(
        #                 f"[{StatusCode.COLLISION_FAIL.value}]:Traj is collision at idx {idx}, qpos {qpos_array_check[idx]}, do not executed!"
        #             )
        #             return StatusCode.COLLISION_FAIL
        #     return StatusCode.SUCCESS
        # else:
        qpos_array_check = np.vstack((qpos_array[::step], qpos_array[-1]))
        for qpos in qpos_array_check:
            if self.is_collision(
                    qpos, save_scene=save_scene) == StatusCode.COLLISION_FAIL:
                self.logger.error(
                    f"[{StatusCode.COLLISION_FAIL.value}]:Traj is collision at {qpos}, do not executed!"
                )
                return StatusCode.COLLISION_FAIL
        return StatusCode.SUCCESS

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
        if not self._check_name(name):
            return StatusCode.NAME_MISMATCH, None
        if not self._check_qpos_limits(name, qpos_array):
            return StatusCode.EXCEED_LIMIT, None

        index = self.w1_params.index_infos[name]["index"]
        if qvel_limits_array is None:
            qvel_limits_array = np.array([self.w1_params.default_qvel[index]] *
                                         qpos_array.shape[0])

        # case: 发两个完全一样的点
        step = weighted_norm(np.deg2rad([1.] * qpos_array[0].shape[0]))
        if qpos_array.shape[0] == 2 and weighted_norm(qpos_array[1] -
                                                      qpos_array[0]) < step:
            return StatusCode.SUCCESS, qpos_array

        if name == "full_body":
            Rn = R20
            RnTangent = R20Tangent
        elif name == "torso":
            Rn = R4
            RnTangent = R4Tangent
        elif name == "head":
            Rn = R2
            RnTangent = R2Tangent
        elif name == "left_arm":
            Rn = R7
            RnTangent = R7Tangent
        elif name == "right_arm":
            Rn = R7
            RnTangent = R7Tangent

        waypoints = [Rn(qpos) for qpos in qpos_array]
        blend_type = BlendType.Bezier5th
        blend_tolerance = 0.005
        if USE_TOPPRA and is_sparse:
            self.logger.debug("Use Toppra for joint traj")
            res1, path = create_blend_path(waypoints, blend_type,
                                           blend_tolerance)
            if res1 != StatusType.Success:
                status = StatusCode.PLAN_PATH_FAIL
                self.logger.error(f"[{status.value}]:Plan path failed!")
                return status, None
            path_point_list = path.sample_by_step(step)
            qpos_sample_array = np.array(
                [pp.config.coeffs() for pp in path_point_list])

            # vel align
            prefix_dist_list = [0.]
            for idx in range(len(qpos_array) - 1):
                dist = weighted_norm(qpos_array[idx + 1] - qpos_array[idx])
                prefix_dist_list.append(prefix_dist_list[-1] + dist)

            idx = 0
            qvel_limits_list = [qvel_limits_array[0]]
            cur_qpos = qpos_sample_array[0]
            cur_dist = 0.
            for qpos in qpos_sample_array[1:-1]:
                cur_dist += weighted_norm(qpos - cur_qpos)
                if cur_dist >= prefix_dist_list[idx] + 1e-6:
                    idx += 1
                qvel_limits_list.append(qvel_limits_array[idx])
                cur_qpos = qpos
            qvel_limits_list.append(qvel_limits_array[-1])

            qpos_array = qpos_sample_array
            qvel_limits_array = np.array(qvel_limits_list)

        elif not USE_TOPPRA and is_sparse:
            path_vel_start = 0.0
            path_acc_start = 0.0
            path_vel_end = 0.0
            path_acc_end = 0.0
            traj_constraints = []
            acc_limits = self.w1_params.max_qacc[index]
            jerk_limits = self.w1_params.max_qjerk[index]
            for vel_limits in qvel_limits_array:
                traj_constraints.append(
                    TrajectoryConstraint(RnTangent(vel_limits.tolist()),
                                         RnTangent(acc_limits.tolist()),
                                         RnTangent(jerk_limits.tolist())))
            res, path, traj = create_double_s_traj_along_blend_path(
                waypoints, blend_type, blend_tolerance, path_vel_start,
                path_acc_start, path_vel_end, path_acc_end, traj_constraints)
            if res != StatusType.Success:
                status = StatusCode.PLAN_PATH_FAIL
                self.logger.error(f"[{status.value}]:Plan path failed!")
                return status, None
            traj_point_list = traj.sample_by_step(self.TIME_STEP)
            pos_array = np.array([tp.pos.coeffs() for tp in traj_point_list])
            return StatusCode.SUCCESS, pos_array
        self.logger.debug("Use Toppra for joint traj")
        dofs = self.w1_params.index_infos[name]["dofs"]
        index = self.w1_params.index_infos[name]["index"]
        pos_limits = np.vstack(
            (self.w1_params.qpos_lb[index], self.w1_params.qpos_ub[index]))
        vel_limits = self.w1_params.max_qvel[index]
        acc_limits = self.w1_params.max_qacc[index]
        traj_planner = ToppraTrajectoryPlanner(dofs, pos_limits, vel_limits,
                                               acc_limits)
        res, pos_array, _, _, _ = traj_planner.plan(qpos_array,
                                                    qvel_limits_array,
                                                    self.TIME_STEP)
        if not res:
            status = StatusCode.PLAN_TRAJ_FAIL
            self.logger.error(f"[{status.value}]:Plan trajectory failed!")
            return status, None
        return StatusCode.SUCCESS, pos_array

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
        arm_kine = self._get_arm_kine(name)
        if arm_kine is None:
            return StatusCode.NAME_MISMATCH, None

        # case: 发两个完全一样的点
        if len(xpos_list) == 2 and np.allclose(
                xpos_list[0], xpos_list[1], atol=1e-4):
            res, qpos_list = arm_kine.calc_nearest_iks(qpos_seed, xpos_list)
            if res != StatusType.Success:
                status = StatusCode.IK_FAIL
                self.logger.error(
                    f"[{status.value}]:Inverse kinematics failed!")
                return status, None
            return self.plan_joint_traj(name, np.array(qpos_list), False,
                                        qvel_limits_array)

        index = self.w1_params.index_infos[name]["index"]
        if qvel_limits_array is None:
            qvel_limits_array = np.array([self.w1_params.default_qvel[index]] *
                                         len(xpos_list))

        waypoints = [R3xSO3(SE3(xpos).coeffs()) for xpos in xpos_list]
        blend_type = BlendType.Bezier5th
        blend_tolerance = 0.005

        if USE_TOPPRA:
            self.logger.debug("Use Toppra for cartesian traj")
            # cartesian interpolation
            res1, path = create_blend_path(waypoints, blend_type,
                                           blend_tolerance)
            if res1 != StatusType.Success:
                status = StatusCode.PLAN_PATH_FAIL
                self.logger.error(f"[{status.value}]:Plan path failed!")
                return status, None
            path_point_list = path.sample_by_step(cartesian_step)
            xpos_sample_array = np.array(
                [SE3(pp.config.coeffs()).transform() for pp in path_point_list])

            prefix_dist_list = [0.]
            for idx in range(len(xpos_list) - 1):
                dist = np.linalg.norm(xpos_list[idx + 1][:3, 3] -
                                      xpos_list[idx][:3, 3])
                prefix_dist_list.append(prefix_dist_list[-1] + dist)

            # vel align
            idx = 0
            qvel_limits_list_ = [qvel_limits_array[idx]]
            cur_xpos = xpos_sample_array[0]
            cur_dist = 0.
            for xpos in xpos_sample_array[1:-1]:
                dist = weighted_norm(xpos[:3, 3] - cur_xpos[:3, 3])
                cur_dist += dist
                if cur_dist >= prefix_dist_list[idx] + 1e-6:
                    idx += 1
                qvel_limits_list_.append(qvel_limits_array[idx])
                cur_xpos = xpos
            qvel_limits_list_.append(qvel_limits_array[-1])
            qvel_limits_array_ = np.array(qvel_limits_list_)

            # solve ik
            res2, qpos_list = arm_kine.calc_nearest_iks(qpos_seed,
                                                        xpos_sample_array)
            if res2 != StatusType.Success:
                status = StatusCode.IK_FAIL
                self.logger.error(
                    f"[{status.value}]:Inverse kinematics failed!")
                return status, None

            return self.plan_joint_traj(name, np.array(qpos_list), False,
                                        qvel_limits_array_)
        else:
            self.logger.debug("Use Double-S for cartesian traj")
            path_vel_start = 0.0
            path_acc_start = 0.0
            path_vel_end = 0.0
            path_acc_end = 0.0
            traj_constraints = []
            for qvel_limits in qvel_limits_array:
                speed_ratio = qvel_limits[0] / self.w1_params.default_qvel[
                    index][0]
                traj_constraints.append(
                    TrajectoryConstraint(
                        R3xSO3Tangent(speed_ratio * self.w1_params.default_vel),
                        R3xSO3Tangent(self.w1_params.max_acc),
                        R3xSO3Tangent(self.w1_params.max_jerk)))
            res1, path, traj = create_double_s_traj_along_blend_path(
                waypoints, blend_type, blend_tolerance, path_vel_start,
                path_acc_start, path_vel_end, path_acc_end, traj_constraints)
            if res1 != StatusType.Success:
                status = StatusCode.PLAN_TRAJ_FAIL
                self.logger.error(f"[{status.value}]:Plan trajectory failed!")
                return status, None
            traj_point_list = traj.sample_by_step(self.TIME_STEP)
            pose_array = np.array(
                [SE3(tp.pos.coeffs()).transform() for tp in traj_point_list])
            res2, qpos_traj = arm_kine.calc_nearest_iks(qpos_seed, pose_array)
            if res2 != StatusType.Success:
                status = StatusCode.IK_FAIL
                self.logger.error(
                    f"[{status.value}]:Inverse kinematics failed!")
                return status, None
            return StatusCode.SUCCESS, np.array(qpos_traj)

    def plan_arm_ee_trigger(self, ee_name: str,
                            arm_ee_state_list: list) -> tuple[StatusCode, list]:
        # TODO: 存在延时的才分段，否则不分段且全部waypoint ee_qpos都到点触发？
        if not self._check_arm_ee_states(ee_name, arm_ee_state_list):
            return StatusCode.INPUT_INVALID, None
        ee_qpos_array = np.array(
            [ee_qpos for _, ee_qpos, _ in arm_ee_state_list])
        if not self._check_qpos_limits(ee_name, ee_qpos_array):
            return StatusCode.EXCEED_LIMIT, None

        segment_list = []
        start_idx = 0
        for i in range(1, len(arm_ee_state_list)):
            pre_qpos = arm_ee_state_list[i - 1][1]
            pre_delay, qpos, post_delay = arm_ee_state_list[i]
            if pre_delay != 0. or post_delay != 0. or not np.allclose(
                    qpos, pre_qpos, atol=np.deg2rad(0.5)):
                segment_list.append([start_idx, i])
                start_idx = i

        if start_idx < len(arm_ee_state_list) - 1:
            segment_list.append([start_idx, len(arm_ee_state_list) - 1])
        self.logger.debug(
            f"plan_arm_ee_trigger {ee_name} result: {segment_list}.")
        return StatusCode.SUCCESS, segment_list

    def execute_joint_traj(
            self,
            qpos_array: np.ndarray,
            left_arm_ee_trigger_list: list = None,
            right_arm_ee_trigger_list: list = None) -> StatusCode:
        """按10ms间隔执行关节轨迹

        Args:
            qpos_array (np.ndarray): 关节轨迹
            left_arm_ee_trigger_list (list, optional): 左末端执行器触发序列. Defaults to None.
            right_arm_ee_trigger_list (list, optional): 右末端执行器触发序列. Defaults to None.

        Returns:
            StatusCode: 
        """
        self.logger.debug("Execute joint traj")
        # check enable
        if self.robot_client.status != Status.Idle:
            status = StatusCode.ROBOT_STATUS_INVALID
            self.logger.error(
                f"[{status.value}]:Robot client not in idle before moving. (Current status: {self.robot_client.status})"
            )
            return status

        # reset stop move flag
        if self._stop_move_event.is_set():
            self._stop_move_event.clear()

        # check current qpos and start qpos
        res, cur_qpos = self.get_current_qpos()
        if res.value < 0:
            return res
        if not np.allclose(cur_qpos, qpos_array[0], atol=np.deg2rad(1)):
            status = StatusCode.NOT_AT_START
            self.logger.error(
                f"[{status.value}]:cur_qpos ({cur_qpos}) is not equal to start_qpos ({qpos_array[0]})."
            )
            return status

        # ee trigger
        ee_thread_error = False

        def ee_trigger(ee_client, ee_trigger_list, count_queue, ee_name):
            nonlocal ee_thread_error
            ee_trigger_list_ = [
                [cnt, ee_qpos, False] for cnt, ee_qpos in ee_trigger_list
            ]
            idx = 0
            while True:
                # 触发完正常退出
                if idx >= len(ee_trigger_list):
                    break
                # 异常退出
                if self._stop_move_event.is_set():
                    break

                try:
                    local_count = count_queue.get(timeout=self.TIME_STEP / 5.)
                except Empty:
                    continue

                target_count, target_ee_qpos, _ = ee_trigger_list_[idx]
                if local_count < target_count:
                    continue
                elif local_count == target_count:
                    ee_client.set_pos(target_ee_qpos)
                    ee_trigger_list_[idx][2] = True
                    self.logger.debug(
                        f"{ee_name} has triggered: {target_count} {target_ee_qpos}."
                    )
                    idx += 1
                else:  # local_count > target_count
                    # 漏了触发
                    self.logger.error(
                        f"[{StatusCode.EE_MOVE_FAIL.value}]:{ee_name} {target_count} haven't been triggered."
                    )
                    ee_thread_error = True
                    return

            not_triggered_counts = [
                cnt for cnt, _, trig in ee_trigger_list_ if not trig
            ]
            # 被stop_move中断了
            if self._stop_move_event.is_set():
                self.logger.warning(
                    f"{ee_name} {not_triggered_counts} haven't been triggered.")
                return
            # 全部触发，执行正常
            if len(not_triggered_counts) == 0:
                self.logger.debug(f"{ee_name} completes trigger.")
            else:
                self.logger.error(
                    f"[{StatusCode.EE_MOVE_FAIL.value}]:{ee_name} {not_triggered_counts} haven't been triggered."
                )
                ee_thread_error = True

        def stop_ee_trigger_threads():
            self._stop_move_event.set()
            if left_arm_ee_thread is not None:
                left_arm_ee_thread.join()
            if right_arm_ee_thread is not None:
                right_arm_ee_thread.join()

        left_arm_ee_thread = None
        left_count_queue = Queue()
        if self._has_arm_ee_client(
                "left_arm_ee", False) and left_arm_ee_trigger_list is not None:
            left_arm_ee_thread = threading.Thread(
                target=ee_trigger,
                args=(self.left_ee_client, left_arm_ee_trigger_list,
                      left_count_queue, "left_arm_ee"),
                daemon=True)
            left_arm_ee_thread.start()

        right_arm_ee_thread = None
        right_count_queue = Queue()
        if self._has_arm_ee_client(
                "right_arm_ee",
                False) and right_arm_ee_trigger_list is not None:
            right_arm_ee_thread = threading.Thread(
                target=ee_trigger,
                args=(self.right_ee_client, right_arm_ee_trigger_list,
                      right_count_queue, "right_arm_ee"),
                daemon=True)
            right_arm_ee_thread.start()

        # move traj async
        endpoint_index = [qpos_array.shape[0]]
        for temp in [left_arm_ee_trigger_list, right_arm_ee_trigger_list]:
            if temp is not None and len(temp) > 0:
                endpoint_index += [count for count, _ in temp]
        endpoint_index = list(set(endpoint_index))
        endpoint_index.sort()

        self.logger.info("Start moving.")
        t0 = time.time()
        end_id = self.robot_client.move_traj_async(
            motion_target=qpos_array.flatten().tolist(),
            motion_group=MotionGroup.WholeBody,
            endpoint_index=endpoint_index,
            interval=self.TIME_STEP)
        if end_id == -1:
            status = StatusCode.ROBOT_MOVE_FAIL
            self.logger.error(f"[{status.value}]:Start moving failed.")
            stop_ee_trigger_threads()
            return status

        # wait move traj finished
        traj_duration = (qpos_array.shape[0] - 1) * self.TIME_STEP
        last_finished_id = end_id - len(endpoint_index)
        for next_finished_id in range(last_finished_id + 1, end_id + 1):
            if self._stop_move_event.is_set():
                self.logger.info(f"Stop moving")
                stop_ee_trigger_threads()
                return StatusCode.STOP_MOVE
            self.robot_client.wait_request_finish(request_id=next_finished_id,
                                                  timeout_sec=max(
                                                      30., traj_duration + 2.))
            global_count = endpoint_index.pop(0)
            left_count_queue.put(global_count)
            right_count_queue.put(global_count)

        if left_arm_ee_thread is not None:
            left_arm_ee_thread.join()
        if right_arm_ee_thread is not None:
            right_arm_ee_thread.join()

        self.logger.info("Finish moving. " +
                         f"Ideal duration should be {traj_duration}s, " +
                         f"and actual duration is {time.time() - t0}s.")

        # check reach qpos
        targt_qpos = qpos_array[-1]
        res, cur_qpos = self.get_current_qpos()
        if res.value < 0:
            return res
        if not np.allclose(cur_qpos, targt_qpos, atol=np.deg2rad(5)):
            status = StatusCode.GOAL_NOT_REACH
            self.logger.error(
                f"[{status.value}]:Current qpos does NOT reach the target qpos, "
                + f"qpos error(deg): {np.rad2deg(cur_qpos - targt_qpos)}.")
            return status
        if not np.allclose(cur_qpos, targt_qpos, atol=np.deg2rad(0.5)):
            self.logger.warning(
                f"Current qpos does NOT close to the target qpos, " +
                f"qpos error(deg): {np.rad2deg(cur_qpos - targt_qpos)}.")
        return StatusCode.SUCCESS if (
            not ee_thread_error) else StatusCode.EE_MOVE_FAIL

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
        res, cur_xpos = self.get_current_xpos(name, is_local)
        if res.value < 0:
            return res, None
        return res, xpos_to_user_pose(cur_xpos)

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
        res, cur_xpos = self.get_xpos(name, qpos, is_local)
        if res.value < 0:
            return res, None
        return res, xpos_to_user_pose(cur_xpos)

    def _parse_waypoints_from_dict(
        self,
        name: str,
        waypoints_dict: dict,
        speed_ratio: float,
        is_cartesian: bool = False
    ) -> tuple[StatusCode, np.ndarray, np.ndarray, list]:
        """解析并检查输入  
            StatusCode: """
        pos_list = waypoints_dict.get(name)
        if not pos_list:  # None or []
            self.logger.debug(f"No waypoints in {name}.")
            return StatusCode.INPUT_EMPTY, None, None, None

        if not all(isinstance(pos, list) for pos in pos_list):
            status = StatusCode.INPUT_INVALID
            self.logger.error(f"[{status.value}]:{pos_list} not a 2d-list.")
            return status, None, None, None
        if not is_cartesian and not self._check_qpos_dimension(name, pos_list):
            return StatusCode.DIM_MISMATCH, None, None, None
        if is_cartesian and not all(len(pos) == 6 for pos in pos_list):
            status = StatusCode.DIM_MISMATCH
            self.logger.error(
                f"[{status.value}]:Dimension of {pos_list[0]} != 6.")
            return status, None, None, None

        speed_ratio_list = waypoints_dict.get(f"{name}_speed_ratio")
        if not speed_ratio_list:  # None or []
            speed_ratio_list = [speed_ratio] * len(pos_list)
        elif len(speed_ratio_list) != len(pos_list):
            status = StatusCode.DIM_MISMATCH
            self.logger.error(
                f"[{status.value}]:{name} qpos and speed_ratio dimension mismatch."
            )
            return status, None, None, None
        # 将None元素替换为global_speed_ratio
        speed_ratio_list = [
            sr if sr is not None else speed_ratio for sr in speed_ratio_list
        ]

        default_qvel = self.w1_params.default_qvel[
            self.w1_params.index_infos[name]["index"]]
        min_qvel = self.w1_params.min_qvel[self.w1_params.index_infos[name]
                                           ["index"]]
        max_qvel = self.w1_params.max_qvel[self.w1_params.index_infos[name]
                                           ["index"]]
        qvel_limits_array = np.clip(np.outer(speed_ratio_list, default_qvel),
                                    min_qvel, max_qvel)

        # 额外读取ee的输入
        arm_ee_state_list = None
        if name in self.w1_params.arm_names:
            arm_ee_state_list = waypoints_dict.get(f"{name}_ee")
            if not arm_ee_state_list:  # None or []
                arm_ee_state_list = None
            elif len(arm_ee_state_list) != len(pos_list):
                status = StatusCode.DIM_MISMATCH
                self.logger.error(
                    f"[{status.value}]:{name} qpos and arm_ee_state dimension mismatch."
                )
                return status, None, None, None
            elif not self._check_arm_ee_states(f"{name}_ee", arm_ee_state_list):
                return StatusCode.INPUT_INVALID, None, None, None

        return StatusCode.SUCCESS, np.array(
            pos_list), qvel_limits_array, arm_ee_state_list

    def _plan_to_waypoint(
            self,
            name: str,
            cur_qpos: np.ndarray,
            waypoints_dict: dict,
            speed_ratio: float,
            is_cartesian: bool = False,
            qpos_seed: np.ndarray = None,
            cartesian_step: float = None,
            is_local: bool = True,
            is_sparse: bool = True) -> tuple[StatusCode, np.ndarray]:
        if not is_local:
            qpos_array = cur_qpos[self.w1_params.index_infos[name]["index"]]
        else:
            qpos_array = cur_qpos
        (res, pos_raw_array, qvel_limits_array,
         _) = self._parse_waypoints_from_dict(name, waypoints_dict, speed_ratio,
                                              is_cartesian)
        if res == StatusCode.INPUT_EMPTY:  # 输入为空
            return StatusCode.SUCCESS, qpos_array
        elif res.value < 0:  # 输入不合法
            return res, None
        # 输入合法
        qvel_limits_array_ = np.vstack([
            self.w1_params.default_qvel[self.w1_params.index_infos[name]
                                        ["index"]], qvel_limits_array
        ])
        if is_cartesian:
            if not is_local:
                # Transfer global frame to local frame
                ref_torso_qpos = cur_qpos[self.w1_params.index_infos["torso"]
                                          ["index"]]
                ref_qpos = np.zeros(self.w1_params.full_body_dofs)
                ref_qpos[self.w1_params.index_infos["torso"]
                         ["index"]] = ref_torso_qpos
                res_fk, T_chassis_armbase = self.get_xpos(f"{name}_base",
                                                          ref_qpos,
                                                          is_local=False)
                if res_fk.value < 0:
                    self.logger.error(f"{name}_base solve fk failed!")
                    return res_fk, None
                temp = [
                    np.linalg.inv(T_chassis_armbase) @ user_pose_to_xpos(
                        pose[:3].tolist(), pose[3:].tolist())
                    for pose in pos_raw_array
                ]
                pos_raw_array = [
                    np.array(xpos_to_user_pose(xpos)) for xpos in temp
                ]
                cur_arm_qpos = cur_qpos[self.w1_params.index_infos[name]
                                        ["index"]]
            else:
                cur_arm_qpos = cur_qpos

            # In local frame
            cur_xpos = self._get_arm_kine(name).get_fk(cur_arm_qpos)
            xpos_raw_list = [
                user_pose_to_xpos(pose[:3].tolist(), pose[3:].tolist())
                for pose in pos_raw_array
            ]
            xpos_raw_list_ = [cur_xpos] + xpos_raw_list
            res_plan, qpos_array = self.plan_cartesian_traj(
                name, xpos_raw_list_, qpos_seed, qvel_limits_array_,
                cartesian_step)
            if res_plan.value < 0:
                return res_plan, None
        else:
            qpos_raw_array_ = np.vstack([cur_qpos, pos_raw_array])
            res_plan, qpos_array = self.plan_joint_traj(name, qpos_raw_array_,
                                                        is_sparse,
                                                        qvel_limits_array_)
            if res_plan.value < 0:
                return res_plan, None

        return StatusCode.SUCCESS, qpos_array

    def _plan_to_waypoint_with_ee(
            self,
            name: str,
            cur_qpos: np.ndarray,
            cur_ee_qpos: np.ndarray,
            waypoints_dict: dict,
            speed_ratio: float,
            is_cartesian: bool = False,
            qpos_seed: np.ndarray = None,
            cartesian_step: float = None,
            is_local: bool = True,
            is_sparse: bool = True) -> tuple[StatusCode, np.ndarray, list]:
        if not is_local:
            qpos_array = cur_qpos[self.w1_params.index_infos[name]["index"]]
        else:
            qpos_array = cur_qpos

        (res, pos_raw_array, qvel_limits_array,
         arm_ee_state_list) = self._parse_waypoints_from_dict(
             name, waypoints_dict, speed_ratio, is_cartesian)
        if res == StatusCode.INPUT_EMPTY:
            return StatusCode.SUCCESS, qpos_array, None
        elif res.value < 0:
            return res, None, None

        # without ee motion
        if arm_ee_state_list is None:
            res, qpos_array = self._plan_to_waypoint(name,
                                                     cur_qpos,
                                                     waypoints_dict,
                                                     speed_ratio,
                                                     is_cartesian,
                                                     qpos_seed,
                                                     cartesian_step,
                                                     is_local,
                                                     is_sparse=is_sparse)
            return res, qpos_array, None

        # with ee motion
        if not is_cartesian:
            pos_raw_array_ = np.vstack([cur_qpos, pos_raw_array])
        else:
            if not is_local:
                # Transfer global frame to local frame
                ref_torso_qpos = cur_qpos[self.w1_params.index_infos["torso"]
                                          ["index"]]
                ref_qpos = np.zeros(self.w1_params.full_body_dofs)
                ref_qpos[self.w1_params.index_infos["torso"]
                         ["index"]] = ref_torso_qpos
                res_fk, T_chassis_armbase = self.get_xpos(f"{name}_base",
                                                          ref_qpos,
                                                          is_local=False)
                if res_fk.value < 0:
                    self.logger.error(f"{name}_base solve fk failed!")
                    return res_fk, None, None
                temp = [
                    np.linalg.inv(T_chassis_armbase) @ user_pose_to_xpos(
                        pose[:3].tolist(), pose[3:].tolist())
                    for pose in pos_raw_array
                ]
                pos_raw_array = [
                    np.array(xpos_to_user_pose(xpos)) for xpos in temp
                ]
                cur_arm_qpos = cur_qpos[self.w1_params.index_infos[name]
                                        ["index"]]
            else:
                cur_arm_qpos = cur_qpos

            cur_xpos = self._get_arm_kine(name).get_fk(cur_arm_qpos)
            pos_raw_array_ = np.vstack(
                [xpos_to_user_pose(cur_xpos), pos_raw_array])

        qvel_limits_array_ = np.vstack([
            self.w1_params.default_qvel[self.w1_params.index_infos[name]
                                        ["index"]], qvel_limits_array
        ])
        cur_ee_state = [0., cur_ee_qpos.tolist(), 0.]
        arm_ee_state_list_ = [cur_ee_state] + [
            ee_state if ee_state is not None else cur_ee_state
            for ee_state in arm_ee_state_list
        ]

        # segment raw waypoint
        arm_ee_trigger_list = []  # (count, ee_qpos)
        res, segment_list = self.plan_arm_ee_trigger(f"{name}_ee",
                                                     arm_ee_state_list_)
        if res.value < 0:
            return res, None, None

        total_count = 0
        total_qpos_array = None
        if is_cartesian:
            cur_qpos_seed = qpos_seed

        for start_idx, end_idx in segment_list:
            seg_pos_array = pos_raw_array_[start_idx:end_idx + 1]
            seg_qvel_limits_array = qvel_limits_array_[start_idx:end_idx + 1]
            if is_cartesian:
                seg_xpos_list = [
                    user_pose_to_xpos(pose[:3].tolist(), pose[3:].tolist())
                    for pose in seg_pos_array
                ]
                res_plan, seg_qpos_samples_array = self.plan_cartesian_traj(
                    name, seg_xpos_list, cur_qpos_seed, seg_qvel_limits_array,
                    cartesian_step)
            else:
                res_plan, seg_qpos_samples_array = self.plan_joint_traj(
                    name, seg_pos_array, is_sparse, seg_qvel_limits_array)
            if res_plan.value < 0:
                return res_plan, None, None

            # pre delay
            pre_delay_time = arm_ee_state_list_[end_idx][0]
            if pre_delay_time >= self.TIME_STEP:
                pre_delay_count = max(
                    1, int(np.ceil(pre_delay_time / self.TIME_STEP)))
                seg_qpos_samples_array = np.vstack([
                    seg_qpos_samples_array,
                    np.repeat(seg_qpos_samples_array[-1:],
                              pre_delay_count,
                              axis=0)
                ])
            # ee trigger
            arm_ee_trigger_list.append(
                (total_count + seg_qpos_samples_array.shape[0] - 1,
                 arm_ee_state_list_[end_idx][1]))
            # post delay
            post_delay_time = arm_ee_state_list_[end_idx][2]
            if post_delay_time >= self.TIME_STEP:
                post_delay_count = max(
                    1, int(np.ceil(post_delay_time / self.TIME_STEP)))
                seg_qpos_samples_array = np.vstack([
                    seg_qpos_samples_array,
                    np.repeat(seg_qpos_samples_array[-1:],
                              post_delay_count,
                              axis=0)
                ])

            if total_qpos_array is None:
                total_qpos_array = seg_qpos_samples_array
            else:
                total_qpos_array = np.vstack(
                    ([total_qpos_array, seg_qpos_samples_array[1:]]))

            if is_cartesian:
                cur_qpos_seed = total_qpos_array[-1]

            total_count = total_qpos_array.shape[0]

        return StatusCode.SUCCESS, total_qpos_array, arm_ee_trigger_list

    def _cal_path_seg_interpolate_num(self,
                                      waypoint_list: list[np.ndarray],
                                      cartesian_step: float = 0.005):
        interpolate_num_list = []
        for idx in range(len(waypoint_list) - 1):
            dist = np.linalg.norm(waypoint_list[idx + 1][:3, 3] -
                                  waypoint_list[idx][:3, 3])
            num_sample = max(2, int(np.ceil(dist / cartesian_step)))
            interpolate_num_list.append(num_sample)
        return interpolate_num_list

    @time_consuming
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
        left_arm_kine = self._get_arm_kine("left_arm")
        if left_arm_kine is None:
            return StatusCode.NAME_MISMATCH, None, None
        right_arm_kine = self._get_arm_kine("right_arm")
        if right_arm_kine is None:
            return StatusCode.NAME_MISMATCH, None, None
        # cartesian interpolation
        left_prefix_dist_list = [0.]
        for idx in range(len(left_xpos_list) - 1):
            dist = np.linalg.norm(left_xpos_list[idx + 1][:3, 3] -
                                  left_xpos_list[idx][:3, 3])
            left_prefix_dist_list.append(left_prefix_dist_list[-1] + dist)
        left_total_dist = left_prefix_dist_list[-1]
        right_prefix_dist_list = [0.]
        for idx in range(len(right_xpos_list) - 1):
            dist = np.linalg.norm(right_xpos_list[idx + 1][:3, 3] -
                                  right_xpos_list[idx][:3, 3])
            right_prefix_dist_list.append(right_prefix_dist_list[-1] + dist)
        right_total_dist = right_prefix_dist_list[-1]

        left_interpolate_num_list = self._cal_path_seg_interpolate_num(
            left_xpos_list, cartesian_step)
        right_interpolate_num_list = self._cal_path_seg_interpolate_num(
            right_xpos_list, cartesian_step)
        if len(left_interpolate_num_list) != len(right_interpolate_num_list):
            status = StatusCode.INPUT_INVALID
            self.logger.error(
                f"[{status.value}]:Left and right arm waypoint segment number mismatch."
            )
            return status, None, None

        interpolate_num_list = []
        for i in range(len(left_interpolate_num_list)):
            left_interpolate_num = left_interpolate_num_list[i]
            right_interpolate_num = right_interpolate_num_list[i]
            seg_num_sample = max(left_interpolate_num, right_interpolate_num)
            interpolate_num_list.append(seg_num_sample)

        left_bezier_planner = PathR3xSO3Bezier(left_xpos_list)
        if not left_bezier_planner.calc_path(PathType.Bezier5thCartesian, 0.01):
            status = StatusCode.PLAN_PATH_FAIL
            self.logger.error(
                f"[{status.value}]:Left arm bezier path planning failed.")
            return status, None, None

        right_bezier_planner = PathR3xSO3Bezier(right_xpos_list)
        if not right_bezier_planner.calc_path(PathType.Bezier5thCartesian,
                                              0.01):
            status = StatusCode.PLAN_PATH_FAIL
            self.logger.error(
                f"[{status.value}]:Right arm bezier path planning failed.")
            return status, None, None

        left_path_seg_list, right_path_seg_list = left_bezier_planner.get_path_segments(
        ), right_bezier_planner.get_path_segments()
        if len(left_path_seg_list) != 2 * len(left_xpos_list) - 3 or \
              len(right_path_seg_list) != 2 * len(right_xpos_list) - 3:
            status = StatusCode.INPUT_INVALID
            self.logger.error(
                f"[{status.value}]:Left and right arm path segment number mismatch with waypoint number."
            )
            return status, None, None

        left_linear_seg_list, left_blend_seg_list = [], []
        right_linear_seg_list, right_blend_seg_list = [], []
        for i in range(len(left_path_seg_list)):
            if i % 2 == 0:
                left_linear_seg_list.append(left_path_seg_list[i])
                right_linear_seg_list.append(right_path_seg_list[i])
            else:
                left_blend_seg_list.append(left_path_seg_list[i])
                right_blend_seg_list.append(right_path_seg_list[i])

        left_rearrange_seg_list, right_rearrange_seg_list = [], []
        if len(left_blend_seg_list) != 0 and len(right_blend_seg_list) != 0:
            for i in range(len(left_linear_seg_list)):
                if i == 0:
                    left_rearrange_seg_list.append(
                        ([], left_linear_seg_list[i], left_blend_seg_list[i]))
                elif i == len(left_linear_seg_list) - 1:
                    left_rearrange_seg_list.append(
                        (left_blend_seg_list[i - 1], left_linear_seg_list[i],
                         []))
                else:
                    left_rearrange_seg_list.append(
                        (left_blend_seg_list[i - 1], left_linear_seg_list[i],
                         left_blend_seg_list[i]))
            for i in range(len(right_linear_seg_list)):
                if i == 0:
                    right_rearrange_seg_list.append(
                        ([], right_linear_seg_list[i], right_blend_seg_list[i]))
                elif i == len(right_linear_seg_list) - 1:
                    right_rearrange_seg_list.append(
                        (right_blend_seg_list[i - 1], right_linear_seg_list[i],
                         []))
                else:
                    right_rearrange_seg_list.append(
                        (right_blend_seg_list[i - 1], right_linear_seg_list[i],
                         right_blend_seg_list[i]))
        else:
            left_rearrange_seg_list.append(([], left_linear_seg_list[0], []))
            right_rearrange_seg_list.append(([], right_linear_seg_list[0], []))

        left_xpos_sample_array, right_xpos_sample_array = [], []
        for idx, (pre_blend_seg, linear_seg,
                  post_blend_seg) in enumerate(left_rearrange_seg_list):
            interpolate_num = interpolate_num_list[idx]
            pre_blend_seg_length = pre_blend_seg.get_length(
            ) if pre_blend_seg != [] else 0.0
            linear_seg_length = linear_seg.get_length()
            post_blend_seg_length = post_blend_seg.get_length(
            ) if post_blend_seg != [] else 0.0

            # pre_blend_interpolate_num = max(0, int(np.ceil(interpolate_num * (pre_blend_seg_length / 2) / (pre_blend_seg_length / 2 + linear_seg_length + post_blend_seg_length / 2))))
            # linear_interpolate_num = max(0, int(np.ceil(interpolate_num * (linear_seg_length) / (pre_blend_seg_length / 2 + linear_seg_length + post_blend_seg_length / 2))))
            # post_blend_interpolate_num = max(0, int(np.ceil(interpolate_num * (post_blend_seg_length / 2) / (pre_blend_seg_length / 2 + linear_seg_length + post_blend_seg_length / 2))))

            weights = np.array([
                pre_blend_seg_length / 2, linear_seg_length,
                post_blend_seg_length / 2
            ])
            den = weights.sum()
            if den <= 0:
                pre_blend_interpolate_num = linear_interpolate_num = post_blend_interpolate_num = 0
            else:
                raw = interpolate_num * weights / den
                base = np.floor(raw).astype(int)
                remain = int(interpolate_num - base.sum())
                frac_order = np.argsort(raw - base)[::-1]
                base[frac_order[:remain]] += 1
                pre_blend_interpolate_num, linear_interpolate_num, post_blend_interpolate_num = base.tolist(
                )

            sample_seg_list = []
            pre_blend_s = np.linspace(
                pre_blend_seg.get_s0() + pre_blend_seg.get_length() / 2,
                pre_blend_seg.get_s0() + pre_blend_seg.get_length(),
                pre_blend_interpolate_num) if pre_blend_seg != [] else None
            linear_s = np.linspace(
                linear_seg.get_s0(),
                linear_seg.get_s0() + linear_seg.get_length(),
                linear_interpolate_num)
            post_blend_s = np.linspace(
                post_blend_seg.get_s0(),
                post_blend_seg.get_s0() + post_blend_seg.get_length() / 2,
                post_blend_interpolate_num) if post_blend_seg != [] else None

            if pre_blend_s is not None:
                for s in pre_blend_s:
                    if s == pre_blend_s[0]:
                        continue
                    sample_seg_list.append(pre_blend_seg.get_config(s))
            for s in linear_s:
                if pre_blend_s is not None and s == linear_s[0]:
                    continue
                sample_seg_list.append(linear_seg.get_config(s))
            if post_blend_s is not None:
                for s in post_blend_s:
                    if s == post_blend_s[0]:
                        continue
                    sample_seg_list.append(post_blend_seg.get_config(s))

            left_xpos_sample_array.append(sample_seg_list)

        for idx, (pre_blend_seg, linear_seg,
                  post_blend_seg) in enumerate(right_rearrange_seg_list):
            interpolate_num = interpolate_num_list[idx]
            pre_blend_seg_length = pre_blend_seg.get_length(
            ) if pre_blend_seg != [] else 0.0
            linear_seg_length = linear_seg.get_length()
            post_blend_seg_length = post_blend_seg.get_length(
            ) if post_blend_seg != [] else 0.0

            # pre_blend_interpolate_num = max(0, int(np.ceil(interpolate_num * (pre_blend_seg_length / 2) / (pre_blend_seg_length / 2 + linear_seg_length + post_blend_seg_length / 2))))
            # linear_interpolate_num = max(0, int(np.ceil(interpolate_num * (linear_seg_length) / (pre_blend_seg_length / 2 + linear_seg_length + post_blend_seg_length / 2))))
            # post_blend_interpolate_num = max(0, int(np.ceil(interpolate_num * (post_blend_seg_length / 2) / (pre_blend_seg_length / 2 + linear_seg_length + post_blend_seg_length / 2))))

            weights = np.array([
                pre_blend_seg_length / 2, linear_seg_length,
                post_blend_seg_length / 2
            ])
            den = weights.sum()
            if den <= 0:
                pre_blend_interpolate_num = linear_interpolate_num = post_blend_interpolate_num = 0
            else:
                raw = interpolate_num * weights / den
                base = np.floor(raw).astype(int)
                remain = int(interpolate_num - base.sum())
                frac_order = np.argsort(raw - base)[::-1]
                base[frac_order[:remain]] += 1
                pre_blend_interpolate_num, linear_interpolate_num, post_blend_interpolate_num = base.tolist(
                )

            sample_seg_list = []
            pre_blend_s = np.linspace(
                pre_blend_seg.get_s0() + pre_blend_seg.get_length() / 2,
                pre_blend_seg.get_s0() + pre_blend_seg.get_length(),
                pre_blend_interpolate_num) if pre_blend_seg != [] else None
            linear_s = np.linspace(
                linear_seg.get_s0(),
                linear_seg.get_s0() + linear_seg.get_length(),
                linear_interpolate_num)
            post_blend_s = np.linspace(
                post_blend_seg.get_s0(),
                post_blend_seg.get_s0() + post_blend_seg.get_length() / 2,
                post_blend_interpolate_num) if post_blend_seg != [] else None

            if pre_blend_s is not None:
                for s in pre_blend_s:
                    if s == pre_blend_s[0]:
                        continue
                    sample_seg_list.append(pre_blend_seg.get_config(s))
            for s in linear_s:
                if s == linear_s[0] and pre_blend_s is not None:
                    continue
                sample_seg_list.append(linear_seg.get_config(s))
            if post_blend_s is not None:
                for s in post_blend_s:
                    if s == post_blend_s[0]:
                        continue
                    sample_seg_list.append(post_blend_seg.get_config(s))

            right_xpos_sample_array.append(sample_seg_list)

        left_xpos_sample_array = [
            xpos for seg in left_xpos_sample_array for xpos in seg
        ]
        right_xpos_sample_array = [
            xpos for seg in right_xpos_sample_array for xpos in seg
        ]

        # vel align
        if left_qvel_limits_array is None:
            left_qvel_limits_array_ = None
        else:
            idx = 0
            left_qvel_limits_list_ = [left_qvel_limits_array[idx]]
            cur_left_xpos = left_xpos_sample_array[0]
            cur_dist = 0.
            for left_xpos in left_xpos_sample_array[1:-1]:
                cur_dist += np.linalg.norm(left_xpos[:3, 3] -
                                           cur_left_xpos[:3, 3])
                if cur_dist >= left_prefix_dist_list[idx]:
                    idx += 1
                left_qvel_limits_list_.append(left_qvel_limits_array[idx])
                cur_left_xpos = left_xpos
            left_qvel_limits_list_.append(left_qvel_limits_array[-1])
        left_qvel_limits_array_ = np.array(left_qvel_limits_list_)

        if right_qvel_limits_array is None:
            right_qvel_limits_array_ = None
        else:
            idx = 0
            right_qvel_limits_list_ = [right_qvel_limits_array[idx]]
            cur_right_xpos = right_xpos_sample_array[0]
            cur_dist = 0.
            for right_xpos in right_xpos_sample_array[1:-1]:
                cur_dist += np.linalg.norm(right_xpos[:3, 3] -
                                           cur_right_xpos[:3, 3])
                if cur_dist >= right_prefix_dist_list[idx]:
                    idx += 1
                right_qvel_limits_list_.append(right_qvel_limits_array[idx])
                cur_right_xpos = right_xpos
            right_qvel_limits_list_.append(right_qvel_limits_array[-1])
        right_qvel_limits_array_ = np.array(right_qvel_limits_list_)

        qvel_limits_array_ = np.hstack(
            [left_qvel_limits_array_, right_qvel_limits_array_])

        # solve ik
        res_left, left_qpos_list = left_arm_kine.calc_nearest_iks(
            left_qpos_seed, left_xpos_sample_array)
        if res_left != StatusType.Success:
            status = StatusCode.IK_FAIL
            self.logger.error(
                f"[{status.value}]:Left arm inverse kinematics failed!")
            return status, None, None
        left_qpos_array = np.array(left_qpos_list)

        res_right, right_qpos_list = right_arm_kine.calc_nearest_iks(
            right_qpos_seed, right_xpos_sample_array)
        if res_right != StatusType.Success:
            status = StatusCode.IK_FAIL
            self.logger.error(
                f"[{status.value}]:Right arm inverse kinematics failed!")
            return status, None, None
        right_qpos_array = np.array(right_qpos_list)

        qpos_array = np.hstack([left_qpos_array, right_qpos_array])

        dofs = 2 * self.w1_params.index_infos["left_arm"]["dofs"]
        left_index = self.w1_params.index_infos["left_arm"]["index"]
        right_index = self.w1_params.index_infos["right_arm"]["index"]
        pos_lb_limits = np.hstack((self.w1_params.qpos_lb[left_index],
                                   self.w1_params.qpos_lb[right_index]))
        pos_ub_limits = np.hstack((self.w1_params.qpos_ub[left_index],
                                   self.w1_params.qpos_ub[right_index]))
        pos_limits = np.vstack([pos_lb_limits, pos_ub_limits])
        left_vel_limits = self.w1_params.max_qvel[left_index]
        right_vel_limits = self.w1_params.max_qvel[right_index]
        vel_limits = np.concatenate([left_vel_limits, right_vel_limits])
        left_acc_limits = self.w1_params.max_qacc[left_index]
        right_acc_limits = self.w1_params.max_qacc[right_index]
        acc_limits = np.concatenate([left_acc_limits, right_acc_limits])
        traj_planner = ToppraTrajectoryPlanner(dofs, pos_limits, vel_limits,
                                               acc_limits)
        res, qpos_array, _, _, _ = traj_planner.plan(qpos_array,
                                                     qvel_limits_array_,
                                                     self.TIME_STEP)

        if not res:
            status = StatusCode.PLAN_TRAJ_FAIL
            self.logger.error(
                f"[{status.value}]:Dual arm cartesian traj planning failed.")
            return status, None, None
        left_qpos_array = qpos_array[:, :self.w1_params.
                                     index_infos["left_arm"]["dofs"]]
        right_qpos_array = qpos_array[:, self.w1_params.
                                      index_infos["left_arm"]["dofs"]:]

        return StatusCode.SUCCESS, left_qpos_array, right_qpos_array

    def _plan_linear_dual_arm_sync_point_by_point(
            self,
            cur_left_qpos: np.ndarray,
            cur_right_qpos: np.ndarray,
            waypoints_dict: dict,
            left_speed_ratio: float,
            right_speed_ratio: float,
            left_qpos_seed: np.ndarray = None,
            right_qpos_seed: np.ndarray = None,
            cartesian_step: float = None,
            is_local: bool = True):
        if not is_local:
            left_qpos_array = cur_left_qpos[
                self.w1_params.index_infos["left_arm"]["index"]]
            right_qpos_array = cur_right_qpos[
                self.w1_params.index_infos["right_arm"]["index"]]
        else:
            left_qpos_array = cur_left_qpos
            right_qpos_array = cur_right_qpos
        left_parse_res, left_pos_raw_array, left_qvel_limits_array, _ = self._parse_waypoints_from_dict(
            "left_arm", waypoints_dict, left_speed_ratio, is_cartesian=True)
        right_parse_res, right_pos_raw_array, right_qvel_limits_array, _ = self._parse_waypoints_from_dict(
            "right_arm", waypoints_dict, right_speed_ratio, is_cartesian=True)
        # TODO: 这里res=0的情况判断需要完善，比如一个手臂有输入，一个手臂无输入
        if left_parse_res == StatusCode.INPUT_EMPTY or right_parse_res == StatusCode.INPUT_EMPTY:
            return StatusCode.SUCCESS, left_qpos_array, right_qpos_array
        elif left_parse_res.value < 0 or right_parse_res.value < 0:
            return StatusCode.FAIL, None, None

        left_qvel_limits_array_ = np.vstack([
            self.w1_params.default_qvel[self.w1_params.index_infos["left_arm"]
                                        ["index"]], left_qvel_limits_array
        ])
        right_qvel_limits_array_ = np.vstack([
            self.w1_params.default_qvel[self.w1_params.index_infos["right_arm"]
                                        ["index"]], right_qvel_limits_array
        ])

        if not is_local:
            # Transfer global frame to local frame
            ref_torso_qpos = cur_left_qpos[self.w1_params.index_infos["torso"]
                                           ["index"]]
            ref_qpos = np.zeros(self.w1_params.full_body_dofs)
            ref_qpos[self.w1_params.index_infos["torso"]
                     ["index"]] = ref_torso_qpos
            res_left_fk, T_chassis_left_armbase = self.get_xpos("left_arm_base",
                                                                ref_qpos,
                                                                is_local=False)
            if res_left_fk.value < 0:
                self.logger.error("left_arm_base solve fk failed!")
                return res_left_fk, None, None
            res_right_fk, T_chassis_right_armbase = self.get_xpos(
                "right_arm_base", ref_qpos, is_local=False)
            if res_right_fk.value < 0:
                self.logger.error("right_arm_base solve fk failed!")
                return res_right_fk, None, None
            left_temp = [
                np.linalg.inv(T_chassis_left_armbase) @ user_pose_to_xpos(
                    pose[:3].tolist(), pose[3:].tolist())
                for pose in left_pos_raw_array
            ]
            right_temp = [
                np.linalg.inv(T_chassis_right_armbase) @ user_pose_to_xpos(
                    pose[:3].tolist(), pose[3:].tolist())
                for pose in right_pos_raw_array
            ]
            left_pos_raw_array = [
                np.array(xpos_to_user_pose(xpos) for xpos in left_temp)
            ]
            right_pos_raw_array = [
                np.array(xpos_to_user_pose(xpos) for xpos in right_temp)
            ]
            cur_left_arm_qpos = cur_left_qpos[
                self.w1_params.index_infos["left_arm"]["index"]]
            cur_right_arm_qpos = cur_right_qpos[
                self.w1_params.index_infos["right_arm"]["index"]]
        else:
            cur_left_arm_qpos = cur_left_qpos
            cur_right_arm_qpos = cur_right_qpos

        cur_left_xpos = self._get_arm_kine("left_arm").get_fk(cur_left_arm_qpos)
        cur_right_xpos = self._get_arm_kine("right_arm").get_fk(
            cur_right_arm_qpos)
        left_xpos_raw_list = [
            user_pose_to_xpos(pose[:3].tolist(), pose[3:].tolist())
            for pose in left_pos_raw_array
        ]
        right_xpos_raw_list = [
            user_pose_to_xpos(pose[:3].tolist(), pose[3:].tolist())
            for pose in right_pos_raw_array
        ]
        left_xpos_raw_list_ = [cur_left_xpos] + left_xpos_raw_list
        right_xpos_raw_list_ = [cur_right_xpos] + right_xpos_raw_list

        if len(left_xpos_raw_list_) != len(right_xpos_raw_list_):
            status = StatusCode.DIM_MISMATCH
            self.logger.error(
                f"[{status.value}]:Left and right arm waypoint number mismatch."
            )
            return status, None, None

        res_plan, left_qpos_array, right_qpos_array = self.plan_dual_arm_cartesian_traj(
            left_xpos_raw_list_, right_xpos_raw_list_, left_qpos_seed,
            right_qpos_seed, left_qvel_limits_array_, right_qvel_limits_array_,
            cartesian_step)
        if res_plan.value < 0:
            return res_plan, None, None
        return StatusCode.SUCCESS, left_qpos_array, right_qpos_array

    @time_consuming
    def plan_joint_full_body(
        self,
        start_qpos: np.ndarray,
        start_left_arm_ee_qpos: np.ndarray,
        start_right_arm_ee_qpos: np.ndarray,
        waypoints_dict: dict,
        speed_ratio: float = 1.0,
        is_sparse: bool = True,
        use_collision_validator: bool = True
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
        if not self._check_qpos_limits("full_body", start_qpos):
            return StatusCode.EXCEED_LIMIT, None, None, None

        start_torso_qpos = start_qpos[self.w1_params.torso_index]
        start_head_qpos = start_qpos[self.w1_params.head_index]
        start_left_arm_qpos = start_qpos[self.w1_params.left_arm_index]
        start_right_arm_qpos = start_qpos[self.w1_params.right_arm_index]

        self.logger.debug(
            f"plan_joint_full_body() input cur_qpos: {start_qpos}")
        self.logger.debug(
            f"plan_joint_full_body() input cur_ee_qpos: {start_left_arm_ee_qpos}, {start_right_arm_ee_qpos}"
        )
        self.logger.debug("plan_joint_full_body() input waypoints_dict:")
        self.logger.debug(waypoints_dict)
        self.logger.debug(
            f"plan_joint_full_body() input speed_ratio: {speed_ratio}")

        # torso
        res_torso, torso_qpos_array = self._plan_to_waypoint(
            "torso",
            start_torso_qpos,
            waypoints_dict,
            speed_ratio,
            is_sparse=is_sparse)
        if res_torso.value < 0:
            self.logger.error("Plan to torso traj failed.")
            return res_torso, None, None, None

        # head
        res_head, head_qpos_array = self._plan_to_waypoint("head",
                                                           start_head_qpos,
                                                           waypoints_dict,
                                                           speed_ratio,
                                                           is_sparse=is_sparse)
        if res_head.value < 0:
            self.logger.error("Plan to head traj failed.")
            return res_head, None, None, None

        # left_arm
        left_arm_ee_trigger_list = None
        if not self._has_arm_ee_client("left_arm_ee", False):
            (res_left_arm,
             left_arm_qpos_array) = self._plan_to_waypoint("left_arm",
                                                           start_left_arm_qpos,
                                                           waypoints_dict,
                                                           speed_ratio,
                                                           is_sparse=is_sparse)
            if res_left_arm.value < 0:
                self.logger.error("Plan to left arm traj failed.")
                return res_left_arm, None, None, None
        else:
            if not self._check_qpos_limits("left_arm_ee",
                                           start_left_arm_ee_qpos):
                return StatusCode.EXCEED_LIMIT, None, None, None
            (res_left_arm, left_arm_qpos_array,
             left_arm_ee_trigger_list) = self._plan_to_waypoint_with_ee(
                 "left_arm",
                 start_left_arm_qpos,
                 start_left_arm_ee_qpos,
                 waypoints_dict,
                 speed_ratio,
                 is_sparse=is_sparse)
            if res_left_arm.value < 0:
                self.logger.error("Plan to left arm traj with ee failed.")
                return res_left_arm, None, None, None

        # right_arm
        right_arm_ee_trigger_list = None
        if not self._has_arm_ee_client("right_arm_ee", False):
            (res_right_arm, right_arm_qpos_array) = self._plan_to_waypoint(
                "right_arm",
                start_right_arm_qpos,
                waypoints_dict,
                speed_ratio,
                is_sparse=is_sparse)
            if res_right_arm.value < 0:
                self.logger.error("Plan to right arm traj failed.")
                return res_right_arm, None, None, None
        else:
            if not self._check_qpos_limits("right_arm_ee",
                                           start_right_arm_ee_qpos):
                return StatusCode.EXCEED_LIMIT, None, None, None
            (res_right_arm, right_arm_qpos_array,
             right_arm_ee_trigger_list) = self._plan_to_waypoint_with_ee(
                 "right_arm",
                 start_right_arm_qpos,
                 start_right_arm_ee_qpos,
                 waypoints_dict,
                 speed_ratio,
                 is_sparse=is_sparse)
            if res_right_arm.value < 0:
                self.logger.error("Plan to right arm traj with ee failed.")
                return res_right_arm, None, None, None

        # align
        res_align, total_qpos_array = self._qpos_align(torso_qpos_array,
                                                       head_qpos_array,
                                                       left_arm_qpos_array,
                                                       right_arm_qpos_array)
        if not res_align:
            return StatusCode.FAIL, None, None, None

        if use_collision_validator:
            if self._is_traj_collision(qpos_array=total_qpos_array,
                                       left_arm_start_ee_qpos=None,
                                       right_arm_start_ee_qpos=None,
                                       left_arm_ee_trigger_list=None,
                                       right_arm_ee_trigger_list=None
                                      ) == StatusCode.COLLISION_FAIL:
                status = StatusCode.COLLISION_FAIL
                self.logger.error(
                    f"[{status.value}]:Full body linear traj collision detected!"
                )
                return status, None, None, None

        return StatusCode.SUCCESS, total_qpos_array, left_arm_ee_trigger_list, right_arm_ee_trigger_list

    def move_joint_full_body(
            self,
            waypoints_dict: dict,
            speed_ratio: float = 1.0,
            is_sparse: bool = True,
            use_collision_validator: bool = True) -> StatusCode:
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
        res, cur_qpos = self.get_current_qpos()
        if res.value < 0:
            return res
        cur_left_arm_ee_qpos = None
        if self._has_arm_ee_client("left_arm_ee", False):
            res, cur_left_arm_ee_qpos = self.get_current_qpos("left_arm_ee")
            if res.value < 0:
                return res
        cur_right_arm_ee_qpos = None
        if self._has_arm_ee_client("right_arm_ee", False):
            res, cur_right_arm_ee_qpos = self.get_current_qpos("right_arm_ee")
            if res.value < 0:
                return res

        # plan traj
        (res, total_qpos_array, left_arm_ee_trigger_list,
         right_arm_ee_trigger_list) = self.plan_joint_full_body(
             cur_qpos, cur_left_arm_ee_qpos, cur_right_arm_ee_qpos,
             waypoints_dict, speed_ratio, is_sparse, use_collision_validator)
        if res.value < 0:
            return res

        # # check collision
        # if self._check_traj_collision(total_qpos_array):
        #     return False

        # execute
        return self.execute_joint_traj(total_qpos_array,
                                       left_arm_ee_trigger_list,
                                       right_arm_ee_trigger_list)

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
        is_sync: bool = False,
        use_collision_validator: bool = True
    ) -> tuple[StatusCode, np.ndarray, list, list]:
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
        if not self._check_qpos_limits("full_body", start_qpos):
            return StatusCode.EXCEED_LIMIT, None, None, None

        start_torso_qpos = start_qpos[self.w1_params.torso_index]
        start_head_qpos = start_qpos[self.w1_params.head_index]
        start_left_arm_qpos = start_qpos[self.w1_params.left_arm_index]
        start_right_arm_qpos = start_qpos[self.w1_params.right_arm_index]

        if left_qpos_seed is None:
            left_qpos_seed = start_left_arm_qpos
        if right_qpos_seed is None:
            right_qpos_seed = start_right_arm_qpos

        self.logger.debug(
            f"current left arm tcp: {self.get_tcp('left_arm')[1]}")
        self.logger.debug(
            f"current right arm tcp: {self.get_tcp('right_arm')[1]}")
        self.logger.debug(
            f"plan_linear_full_body() input cur_qpos: {start_qpos}")
        self.logger.debug(
            f"plan_linear_full_body() input cur_ee_qpos: {start_left_arm_ee_qpos}, {start_right_arm_ee_qpos}"
        )
        self.logger.debug("plan_linear_full_body() input waypoints_dict:")
        self.logger.debug(waypoints_dict)
        self.logger.debug(
            f"plan_linear_full_body() input speed_ratio: {speed_ratio}")

        # torso
        res_torso, torso_qpos_array = self._plan_to_waypoint(
            "torso", start_torso_qpos, waypoints_dict, speed_ratio)
        if res_torso.value < 0:
            self.logger.error("Plan to torso traj failed.")
            return res_torso, None, None, None

        # head
        res_head, head_qpos_array = self._plan_to_waypoint(
            "head", start_head_qpos, waypoints_dict, speed_ratio)
        if res_head.value < 0:
            self.logger.error("Plan to head traj failed.")
            return res_head, None, None, None

        if not is_sync:
            # left_arm
            left_arm_ee_trigger_list = None
            if not self._has_arm_ee_client("left_arm_ee", False):
                if is_local:
                    (res_left_arm,
                     left_arm_qpos_array) = self._plan_to_waypoint(
                         "left_arm",
                         start_left_arm_qpos,
                         waypoints_dict,
                         speed_ratio,
                         is_cartesian=True,
                         qpos_seed=left_qpos_seed,
                         cartesian_step=cartesian_step,
                         is_local=True)
                else:
                    (res_left_arm,
                     left_arm_qpos_array) = self._plan_to_waypoint(
                         "left_arm",
                         start_qpos,
                         waypoints_dict,
                         speed_ratio,
                         is_cartesian=True,
                         qpos_seed=left_qpos_seed,
                         cartesian_step=cartesian_step,
                         is_local=False)
                if res_left_arm.value < 0:
                    self.logger.error("Plan to left arm traj failed.")
                    return res_left_arm, None, None, None
            else:
                if not self._check_qpos_limits("left_arm_ee",
                                               start_left_arm_ee_qpos):
                    return StatusCode.EXCEED_LIMIT, None, None, None
                if is_local:
                    (res_left_arm, left_arm_qpos_array,
                     left_arm_ee_trigger_list) = self._plan_to_waypoint_with_ee(
                         "left_arm",
                         start_left_arm_qpos,
                         start_left_arm_ee_qpos,
                         waypoints_dict,
                         speed_ratio,
                         is_cartesian=True,
                         qpos_seed=left_qpos_seed,
                         cartesian_step=cartesian_step,
                         is_local=True)
                else:
                    (res_left_arm, left_arm_qpos_array,
                     left_arm_ee_trigger_list) = self._plan_to_waypoint_with_ee(
                         "left_arm",
                         start_qpos,
                         start_left_arm_ee_qpos,
                         waypoints_dict,
                         speed_ratio,
                         is_cartesian=True,
                         qpos_seed=left_qpos_seed,
                         cartesian_step=cartesian_step,
                         is_local=False)
                if res_left_arm.value < 0:
                    self.logger.error("Plan to left arm traj with ee failed.")
                    return res_left_arm, None, None, None

            # right_arm
            right_arm_ee_trigger_list = None
            if not self._has_arm_ee_client("right_arm_ee", False):
                if is_local:
                    (res_right_arm,
                     right_arm_qpos_array) = self._plan_to_waypoint(
                         "right_arm",
                         start_right_arm_qpos,
                         waypoints_dict,
                         speed_ratio,
                         is_cartesian=True,
                         qpos_seed=right_qpos_seed,
                         cartesian_step=cartesian_step,
                         is_local=True)
                else:
                    (res_right_arm,
                     right_arm_qpos_array) = self._plan_to_waypoint(
                         "right_arm",
                         start_qpos,
                         waypoints_dict,
                         speed_ratio,
                         is_cartesian=True,
                         qpos_seed=right_qpos_seed,
                         cartesian_step=cartesian_step,
                         is_local=False)
                if res_right_arm.value < 0:
                    self.logger.error("Plan to right arm traj failed.")
                    return res_right_arm, None, None, None
            else:
                if not self._check_qpos_limits("right_arm_ee",
                                               start_right_arm_ee_qpos):
                    return StatusCode.EXCEED_LIMIT, None, None, None
                if is_local:
                    (res_right_arm, right_arm_qpos_array,
                     right_arm_ee_trigger_list
                    ) = self._plan_to_waypoint_with_ee(
                        "right_arm",
                        start_right_arm_qpos,
                        start_right_arm_ee_qpos,
                        waypoints_dict,
                        speed_ratio,
                        is_cartesian=True,
                        qpos_seed=right_qpos_seed,
                        cartesian_step=cartesian_step,
                        is_local=True)
                else:
                    (res_right_arm, right_arm_qpos_array,
                     right_arm_ee_trigger_list
                    ) = self._plan_to_waypoint_with_ee(
                        "right_arm",
                        start_qpos,
                        start_right_arm_ee_qpos,
                        waypoints_dict,
                        speed_ratio,
                        is_cartesian=True,
                        qpos_seed=right_qpos_seed,
                        cartesian_step=cartesian_step,
                        is_local=False)
                if res_right_arm.value < 0:
                    self.logger.error("Plan to right arm traj with ee failed.")
                    return res_right_arm, None, None, None
        else:
            # dual arm move line sync planning
            left_arm_ee_trigger_list, right_arm_ee_trigger_list = None, None
            if not self._has_arm_ee_client("left_arm_ee", False) or \
                not self._has_arm_ee_client("right_arm_ee", False):
                if is_local:
                    (res_dual_arm, left_arm_qpos_array, right_arm_qpos_array) \
                     = self._plan_linear_dual_arm_sync_point_by_point(
                        start_left_arm_qpos,
                        start_right_arm_qpos,
                        waypoints_dict,
                        speed_ratio,
                        speed_ratio,
                        left_qpos_seed=left_qpos_seed,
                        right_qpos_seed=right_qpos_seed,
                        cartesian_step=cartesian_step,
                        is_local=True
                    )
                    if res_dual_arm:
                        left_arm_qpos_array = np.vstack([
                            start_qpos[self.w1_params.left_arm_index],
                            left_arm_qpos_array
                        ])
                        right_arm_qpos_array = np.vstack([
                            start_qpos[self.w1_params.right_arm_index],
                            right_arm_qpos_array
                        ])
                else:
                    (res_dual_arm, left_arm_qpos_array, right_arm_qpos_array) \
                     = self._plan_linear_dual_arm_sync_point_by_point(
                        start_qpos,
                        start_qpos,
                        waypoints_dict,
                        speed_ratio,
                        speed_ratio,
                        left_qpos_seed=left_qpos_seed,
                        right_qpos_seed=right_qpos_seed,
                        cartesian_step=cartesian_step,
                        is_local=False
                    )
                    if res_dual_arm:
                        left_arm_qpos_array = np.vstack([
                            start_qpos[self.w1_params.left_arm_index],
                            left_arm_qpos_array
                        ])
                        right_arm_qpos_array = np.vstack([
                            start_qpos[self.w1_params.right_arm_index],
                            right_arm_qpos_array
                        ])
                if res_dual_arm.value < 0:
                    self.logger.error("Plan to dual arm sync traj failed.")
                    return res_dual_arm, None, None, None

            # TODO: complete this: the following part originally designed for synchronize dual arm with ee, but not implemented yet
            else:
                if is_local:
                    (res_dual_arm, left_arm_qpos_array, right_arm_qpos_array) \
                     = self._plan_linear_dual_arm_sync_point_by_point(
                        start_left_arm_qpos,
                        start_right_arm_qpos,
                        waypoints_dict,
                        speed_ratio,
                        speed_ratio,
                        left_qpos_seed=left_qpos_seed,
                        right_qpos_seed=right_qpos_seed,
                        cartesian_step=cartesian_step,
                        is_local=True
                    )
                    if res_dual_arm:
                        left_arm_qpos_array = np.vstack([
                            start_qpos[self.w1_params.left_arm_index],
                            left_arm_qpos_array
                        ])
                        right_arm_qpos_array = np.vstack([
                            start_qpos[self.w1_params.right_arm_index],
                            right_arm_qpos_array
                        ])
                else:
                    (res_dual_arm, left_arm_qpos_array, right_arm_qpos_array) \
                     = self._plan_linear_dual_arm_sync_point_by_point(
                        start_qpos,
                        start_qpos,
                        waypoints_dict,
                        speed_ratio,
                        speed_ratio,
                        left_qpos_seed=left_qpos_seed,
                        right_qpos_seed=right_qpos_seed,
                        cartesian_step=cartesian_step,
                        is_local=False
                    )
                    if res_dual_arm:
                        left_arm_qpos_array = np.vstack([
                            start_qpos[self.w1_params.left_arm_index],
                            left_arm_qpos_array
                        ])
                        right_arm_qpos_array = np.vstack([
                            start_qpos[self.w1_params.right_arm_index],
                            right_arm_qpos_array
                        ])
                if res_dual_arm.value < 0:
                    self.logger.error("Plan to dual arm sync traj failed.")
                    return res_dual_arm, None, None, None

                left_ee_target_list = waypoints_dict.get("left_arm_ee")
                right_ee_target_list = waypoints_dict.get("right_arm_ee")
                if left_ee_target_list is not None and left_ee_target_list != []:
                    left_ee_target = left_ee_target_list[-1]
                    left_arm_ee_trigger_list = [
                        (int(left_arm_qpos_array.shape[0] +
                             left_ee_target[0] / self.TIME_STEP),
                         left_ee_target[1])
                    ]
                    left_arm_qpos_array = np.vstack([
                        left_arm_qpos_array,
                        np.tile(left_arm_qpos_array[-1, :],
                                (int((left_ee_target[0] + left_ee_target[2]) /
                                     self.TIME_STEP), 1))
                    ])
                if right_ee_target_list is not None and right_ee_target_list != []:
                    right_ee_target = right_ee_target_list[-1]
                    right_arm_ee_trigger_list = [
                        (int(right_arm_qpos_array.shape[0] +
                             right_ee_target[0] / self.TIME_STEP),
                         right_ee_target[1])
                    ]
                    right_arm_qpos_array = np.vstack([
                        right_arm_qpos_array,
                        np.tile(right_arm_qpos_array[-1, :],
                                (int((right_ee_target[0] + right_ee_target[2]) /
                                     self.TIME_STEP), 1))
                    ])

        # align
        res_align, total_qpos_array = self._qpos_align(torso_qpos_array,
                                                       head_qpos_array,
                                                       left_arm_qpos_array,
                                                       right_arm_qpos_array)
        if not res_align:
            return StatusCode.FAIL, None, None, None

        if use_collision_validator:
            if self._is_traj_collision(qpos_array=total_qpos_array,
                                       left_arm_start_ee_qpos=None,
                                       right_arm_start_ee_qpos=None,
                                       left_arm_ee_trigger_list=None,
                                       right_arm_ee_trigger_list=None
                                      ) == StatusCode.COLLISION_FAIL:
                status = StatusCode.COLLISION_FAIL
                self.logger.error(
                    f"[{status.value}]:Full body linear traj collision detected!"
                )
                return status, None, None, None

        return StatusCode.SUCCESS, total_qpos_array, left_arm_ee_trigger_list, right_arm_ee_trigger_list

    def move_linear_full_body(
            self,
            waypoints_dict: dict,
            left_qpos_seed: np.ndarray = None,
            right_qpos_seed: np.ndarray = None,
            cartesian_step: float = 0.005,
            speed_ratio: float = 1.0,
            is_local: bool = True,
            is_sync: bool = False,
            use_collision_validator: bool = True) -> StatusCode:
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
        res, cur_qpos = self.get_current_qpos()
        if res.value < 0:
            return res
        cur_left_arm_ee_qpos = None
        if self._has_arm_ee_client("left_arm_ee", False):
            res, cur_left_arm_ee_qpos = self.get_current_qpos("left_arm_ee")
            if res.value < 0:
                return res
        cur_right_arm_ee_qpos = None
        if self._has_arm_ee_client("right_arm_ee", False):
            res, cur_right_arm_ee_qpos = self.get_current_qpos("right_arm_ee")
            if res.value < 0:
                return res

        # plan traj
        (res, total_qpos_array, left_arm_ee_trigger_list,
         right_arm_ee_trigger_list) = self.plan_linear_full_body(
             cur_qpos, cur_left_arm_ee_qpos, cur_right_arm_ee_qpos,
             waypoints_dict, left_qpos_seed, right_qpos_seed, cartesian_step,
             speed_ratio, is_local, is_sync, use_collision_validator)

        # TODO: just for test, comment this
        self.total_qpos_array = total_qpos_array

        if res.value < 0:
            return res

        # # check collision
        # if self._check_traj_collision(total_qpos_array):
        #     return False

        # execute
        return self.execute_joint_traj(total_qpos_array,
                                       left_arm_ee_trigger_list,
                                       right_arm_ee_trigger_list)

    def plan_linear_dual_arm_sync(self,
                                  task_dict: dict,
                                  start_qpos: np.ndarray,
                                  start_left_arm_ee_qpos: np.ndarray,
                                  start_right_arm_ee_qpos: np.ndarray,
                                  speed_ratio: float = 1.0,
                                  is_local: bool = True,
                                  use_collision_validator: bool = True,
                                  is_leg_leaning_forward: bool = True):
        """
        Fixed connection between left and right arm, plan linear trajectory for dual arm or whole body motion.

        Args:
            task_dict (dict): same as previous function parameters, but only support one arm task at the same time
            start_qpos (np.ndarray, shape=(20,)): the start joint position of robot, for w1 should be (20,)
            start_left_arm_ee_qpos (np.ndarray, shape=(N,)): the start left arm ee qpos
            start_right_arm_ee_qpos (np.ndarray, shape=(N,)): the start right arm ee qpos
            speed_ratio (float, optional): a ratio to scale the default joint velocity. Defaults to 1.0.
            is_local (bool, optional): whether the target pose in task_dict is in local frame or global frame. Defaults to True.
            use_collision_validator (bool, optional): whether to use collision validator to check the trajectory safety. Defaults to True.

        Notes:
            - Only support one arm task in task_dict at the same time
            - if there isn't any end effectors, start_left_arm_ee_qpos and start_right_arm_ee_qpos can be set to None 
        """
        self.logger.debug(
            f"current left arm tcp: {self.get_tcp('left_arm')[1]}")
        self.logger.debug(
            f"current right arm tcp: {self.get_tcp('right_arm')[1]}")
        arml_kine, armr_kine = self._get_arm_kine(
            "left_arm"), self._get_arm_kine("right_arm")

        cur_head_qpos = start_qpos[self.real_head_idx]
        cur_torso_qpos = start_qpos[self.real_torso_idx]
        cur_arml_qpos = start_qpos[self.real_arml_idx]
        cur_armr_qpos = start_qpos[self.real_armr_idx]
        cur_arml_pose = arml_kine.get_fk(cur_arml_qpos)
        cur_armr_pose = armr_kine.get_fk(cur_armr_qpos)

        # cur_arml_base_tf, cur_armr_base_tf = self._pin_kine.get_arm_base_tf(
        #     real_qpos=start_qpos)
        arml_base_res, cur_arml_base_tf = self.kine.get_fk(name="left_arm_base",
                                                           qpos=start_qpos)
        armr_base_res, cur_armr_base_tf = self.kine.get_fk(
            name="right_arm_base", qpos=start_qpos)

        if not is_local:
            cur_arml_pose = cur_arml_base_tf @ cur_arml_pose
            cur_armr_pose = cur_armr_base_tf @ cur_armr_pose

        self.cartesian_planner.set_left_right_relationship(
            left_tcp_pose=cur_arml_pose,
            right_tcp_pose=cur_armr_pose,
            is_local=is_local)

        arml_target_pose_list, armr_target_pose_list = task_dict.get(
            "left_arm"), task_dict.get("right_arm")
        arml_speed_ratio_list, armr_speed_ratio_list = task_dict.get(
            "left_arm_speed_ratio"), task_dict.get("right_arm_speed_ratio")
        if (arml_target_pose_list is None and armr_target_pose_list is None) \
            or (arml_target_pose_list == [] and armr_target_pose_list == []):
            status = StatusCode.INPUT_EMPTY
            self.logger.error(
                f"[{status.value}]:Both left arm and right arm in task_dict are None or empty."
            )
            return status, None
        if (arml_target_pose_list is not None and arml_target_pose_list != []) \
            and (armr_target_pose_list is not None and armr_target_pose_list != []):
            status = StatusCode.INPUT_INVALID
            self.logger.error(
                f"[{status.value}]:Only one arm task is allowed in move_linear_dual_arm_sync."
            )
            return status, None

        left_default_qvel = self.w1_params.default_qvel[
            self.w1_params.index_infos["left_arm"]["index"]]
        right_default_qvel = self.w1_params.default_qvel[
            self.w1_params.index_infos["right_arm"]["index"]]
        full_body_default_qvel = self.w1_params.default_qvel
        left_min_qvel = self.w1_params.min_qvel[
            self.w1_params.index_infos["left_arm"]["index"]]
        right_min_qvel = self.w1_params.min_qvel[
            self.w1_params.index_infos["right_arm"]["index"]]
        full_body_min_qvel = self.w1_params.min_qvel
        left_max_qvel = self.w1_params.max_qvel[
            self.w1_params.index_infos["left_arm"]["index"]]
        right_max_qvel = self.w1_params.max_qvel[
            self.w1_params.index_infos["right_arm"]["index"]]
        full_body_max_qvel = self.w1_params.max_qvel

        target_pose_list = []
        target_speed_ratio_list = []
        target_arm_name = None
        if arml_target_pose_list is not None and arml_target_pose_list != []:
            for user_pose in arml_target_pose_list:
                target_pose_list.append(
                    user_pose_to_xpos(user_pose[:3], user_pose[3:]))
            target_arm_name = "left_arm"
            target_pose_list.insert(0, cur_arml_pose)

            if is_local:
                target_speed_ratio_list = arml_speed_ratio_list
                target_qvel_limits_array = np.clip(
                    np.outer(target_speed_ratio_list, left_default_qvel),
                    left_min_qvel, left_max_qvel)
                target_qvel_limits_array_ = np.vstack([
                    self.w1_params.default_qvel[
                        self.w1_params.index_infos["left_arm"]["index"]],
                    target_qvel_limits_array
                ])
            else:
                target_speed_ratio_list = arml_speed_ratio_list
                target_qvel_limits_array = np.clip(
                    np.outer(target_speed_ratio_list, full_body_default_qvel),
                    full_body_min_qvel, full_body_max_qvel)
                target_qvel_limits_array_ = np.vstack(
                    [self.w1_params.default_qvel, target_qvel_limits_array])
        else:
            for user_pose in armr_target_pose_list:
                target_pose_list.append(
                    user_pose_to_xpos(user_pose[:3], user_pose[3:]))
            target_arm_name = "right_arm"
            target_pose_list.insert(0, cur_armr_pose)

            if is_local:
                target_speed_ratio_list = armr_speed_ratio_list
                target_qvel_limits_array = np.clip(
                    np.outer(target_speed_ratio_list, right_default_qvel),
                    right_min_qvel, right_max_qvel)
                target_qvel_limits_array_ = np.vstack([
                    self.w1_params.default_qvel[
                        self.w1_params.index_infos["right_arm"]["index"]],
                    target_qvel_limits_array
                ])
            else:
                target_speed_ratio_list = armr_speed_ratio_list
                target_qvel_limits_array = np.clip(
                    np.outer(target_speed_ratio_list, full_body_default_qvel),
                    full_body_min_qvel, full_body_max_qvel)
                target_qvel_limits_array_ = np.vstack(
                    [self.w1_params.default_qvel, target_qvel_limits_array])

        res, left_pose_array, right_pose_array = self.cartesian_planner.generate_line_traj_from_tcp_poses(
            name=target_arm_name,
            waypoint_list=target_pose_list,
            cartesian_step=0.005,
            is_local=is_local)

        if target_arm_name == "left_arm":
            target_pose_array = left_pose_array
        else:
            target_pose_array = right_pose_array

        # velocity align
        prefix_dist_list = [0.]
        for idx in range(len(target_pose_list) - 1):
            dist = np.linalg.norm(target_pose_list[idx + 1][:3, 3] -
                                  target_pose_list[idx][:3, 3])
            prefix_dist_list.append(prefix_dist_list[-1] + dist)

        idx = 0
        qvel_limits_list_ = [target_qvel_limits_array_[idx]]
        cur_xpos = target_pose_array[0]
        cur_dist = 0.
        for i in range(1, target_pose_array.shape[0] - 1):  # bug here
            xpos = target_pose_array[i]
            cur_dist += np.linalg.norm(xpos[:3, 3] - cur_xpos[:3, 3])
            if cur_dist >= prefix_dist_list[idx] + 1e-6:
                idx += 1
            qvel_limits_list_.append(target_qvel_limits_array_[idx])
            cur_xpos = xpos
        qvel_limits_list_.append(target_qvel_limits_array_[-1])
        qvel_limits_array_ = np.array(qvel_limits_list_)

        if is_local:
            left_res, left_joint_traj = self.cartesian_joint_planner.plan_single_arm(
                name="left_arm",
                start_qpos=cur_arml_qpos,
                end_effector_traj=left_pose_array)
            right_res, right_joint_traj = self.cartesian_joint_planner.plan_single_arm(
                name="right_arm",
                start_qpos=cur_armr_qpos,
                end_effector_traj=right_pose_array)
            if not left_res:
                status = StatusCode.PLAN_TRAJ_FAIL
                self.logger.error(
                    f"[{status.value}]:Left arm joint traj planning failed in plan_linear_dual_arm_sync."
                )
                return status, None
            if not right_res:
                status = StatusCode.PLAN_TRAJ_FAIL
                self.logger.error(
                    f"[{status.value}]:Right arm joint traj planning failed in plan_linear_dual_arm_sync."
                )
                return status, None

            res, left_joint_traj_, right_joint_traj_ = self.cartesian_joint_planner.generate_dense_dual_arm_joint_traj(
                left_joint_traj=left_joint_traj,
                right_joint_traj=right_joint_traj,
                qvel_limits_array=qvel_limits_array_,
                current_left_qpos=cur_arml_qpos,
                current_right_qpos=cur_armr_qpos)

            if not res:
                status = StatusCode.PLAN_TRAJ_FAIL
                self.logger.error(
                    f"[{status.value}]:Generate dense joint traj failed in plan_linear_dual_arm_sync."
                )
                return status, None

            res, move_arml_armr_traj = self._qpos_align(cur_torso_qpos,
                                                        cur_head_qpos,
                                                        left_joint_traj_,
                                                        right_joint_traj_)
            if not res:
                status = StatusCode.FAIL
                self.logger.error(
                    f"[{status.value}]:Align dual arm traj failed in plan_linear_dual_arm_sync."
                )
                return status, None
            if use_collision_validator:
                if self._is_traj_collision(qpos_array=move_arml_armr_traj,
                                           left_arm_start_ee_qpos=None,
                                           right_arm_start_ee_qpos=None,
                                           left_arm_ee_trigger_list=None,
                                           right_arm_ee_trigger_list=None
                                          ) == StatusCode.COLLISION_FAIL:
                    status = StatusCode.COLLISION_FAIL
                    self.logger.error(
                        f"[{status.value}]:Collision detected in plan_linear_dual_arm_sync."
                    )
                    return status, None

            return StatusCode.SUCCESS, move_arml_armr_traj

        else:
            res, full_body_joint_traj = self.cartesian_joint_planner.plan_whole_body(
                start_qpos=start_qpos,
                left_end_effector_traj=left_pose_array,
                right_end_effector_traj=right_pose_array,
                is_leg_leaning_forward=is_leg_leaning_forward)
            if not res:
                status = StatusCode.PLAN_TRAJ_FAIL
                self.logger.error(
                    f"[{status.value}]:Full body joint traj planning failed in plan_linear_dual_arm_sync."
                )
                return status, None

            res, full_body_joint_traj_ = self.cartesian_joint_planner.generate_dense_whole_body_joint_traj(
                whole_body_joint_traj=full_body_joint_traj,
                qvel_limits_array=qvel_limits_array_,
                current_qpos=start_qpos)

            if not res:
                status = StatusCode.PLAN_TRAJ_FAIL
                self.logger.error(
                    f"[{status.value}]:Generate dense whole body joint traj failed in plan_linear_dual_arm_sync."
                )
                return status, None

            move_arml_armr_traj = full_body_joint_traj_

            if use_collision_validator:
                if self._is_traj_collision(qpos_array=move_arml_armr_traj,
                                           left_arm_start_ee_qpos=None,
                                           right_arm_start_ee_qpos=None,
                                           left_arm_ee_trigger_list=None,
                                           right_arm_ee_trigger_list=None
                                          ) == StatusCode.COLLISION_FAIL:
                    status = StatusCode.COLLISION_FAIL
                    self.logger.error(
                        f"[{status.value}]:Collision detected in plan_linear_dual_arm_sync."
                    )
                    return status, None

            return StatusCode.SUCCESS, move_arml_armr_traj

    def move_linear_dual_arm_sync(self,
                                  task_dict: dict,
                                  speed_ratio: float = 1.0,
                                  is_local: bool = True,
                                  use_collision_validator: bool = True):

        res, cur_qpos = self.get_current_qpos()
        if res.value < 0:
            self.logger.error(
                "Function: move_linear_dual_arm_sync(), failed to get current qpos"
            )
            return res

        res, cur_armlee_qpos = self.get_current_qpos("left_arm_ee")
        if res.value < 0:
            self.logger.error("Get current left arm ee qpos failed.")
            return res
        res, cur_armree_qpos = self.get_current_qpos("right_arm_ee")
        if res.value < 0:
            self.logger.error("Get current right arm ee qpos failed.")
            return res

        res, full_body_traj = self.plan_linear_dual_arm_sync(
            task_dict=task_dict,
            start_qpos=cur_qpos,
            start_left_arm_ee_qpos=cur_armlee_qpos,
            start_right_arm_ee_qpos=cur_armree_qpos,
            speed_ratio=speed_ratio,
            is_local=is_local,
            use_collision_validator=use_collision_validator)

        if res.value < 0:
            self.logger.error("Plan linear dual arm sync traj failed.")
            return res

        return self.execute_joint_traj(
            qpos_array=full_body_traj,
            left_arm_ee_trigger_list=None,
            right_arm_ee_trigger_list=None), full_body_traj

    def _interpolate_bezier_path(
        self,
        path: PathR3xSO3Bezier,
        cartesian_step: float = 0.005,
        interpolate_num: int = None,
        interpolate_num_list: list = None
    ) -> tuple[StatusCode, list[np.ndarray]]:
        """
        Note:
            - The input path should be already calculated, and the path type should be PathType.Bezier5thCartesian
        """
        if interpolate_num is not None and interpolate_num_list is not None:
            status = StatusCode.INPUT_INVALID
            self.logger.error(
                f"[{status.value}]:Only one of interpolate_num and interpolate_num_list should be set."
            )
            return status, None

        if interpolate_num_list is None and interpolate_num is None:
            interpolated_path = path.get_sample_by_step(cartesian_step)
            if interpolated_path is None:
                status = StatusCode.PLAN_PATH_FAIL
                self.logger.error(
                    f"[{status.value}]:Interpolate bezier path failed.")
                return status, None
            return StatusCode.SUCCESS, interpolated_path
        elif interpolate_num is not None and interpolate_num_list is None:
            interpolated_path = path.get_sample_by_num(interpolate_num)
            if interpolated_path is None:
                status = StatusCode.PLAN_PATH_FAIL
                self.logger.error(
                    f"[{status.value}]:Interpolate bezier path failed.")
                return status, None
            return StatusCode.SUCCESS, interpolated_path
        elif interpolate_num_list is not None and interpolate_num is None:
            path_seg_list = path.get_path_segments()
            # TODO: 判断如果因为某些点距离很近导致path计算过程中滤除了这些点，然后path segments的数量和interpolate_num_list长度不一致的情况
            # 这里假设path segments都是按照直线+blend+直线的顺序
            path_linear_seg_list, path_blend_seg_list = [], []
            for i in range(len(path_seg_list)):
                if i % 2 == 0:
                    path_linear_seg_list.append(path_seg_list[i])
                else:
                    path_blend_seg_list.append(path_seg_list[i])

            path_rearrange_seg_list = []
            if len(path_blend_seg_list) != 0:
                for i in range(len(path_linear_seg_list)):
                    if i == 0:
                        path_rearrange_seg_list.append(
                            ([], path_linear_seg_list[i],
                             path_blend_seg_list[i]))
                    elif i == len(path_linear_seg_list) - 1:
                        path_rearrange_seg_list.append(
                            (path_blend_seg_list[i - 1],
                             path_linear_seg_list[i], []))
                    else:
                        path_rearrange_seg_list.append(
                            (path_blend_seg_list[i - 1],
                             path_linear_seg_list[i], path_blend_seg_list[i]))
            else:
                path_rearrange_seg_list.append(
                    ([], path_linear_seg_list[0], []))

            if len(path_rearrange_seg_list) != len(interpolate_num_list):
                status = StatusCode.INPUT_INVALID
                self.logger.error(
                    f"[{status.value}]:The length of beizer path segments mismatch with the length of interpolate_num_list."
                )
                return status, None

            interpolate_path = []
            for idx, (pre_blend_seg, linear_seg,
                      post_blend_seg) in enumerate(path_rearrange_seg_list):
                interpolate_num = interpolate_num_list[idx]
                pre_blend_seg_length = pre_blend_seg.get_length(
                ) if pre_blend_seg != [] else 0.0
                linear_seg_length = linear_seg.get_length()
                post_blend_seg_length = post_blend_seg.get_length(
                ) if post_blend_seg != [] else 0.0

                weights = np.array([
                    pre_blend_seg_length / 2, linear_seg_length,
                    post_blend_seg_length / 2
                ])
                den = weights.sum()
                if den <= 0:
                    pre_blend_interpolate_num = linear_interpolate_num = post_blend_interpolate_num = 0
                else:
                    raw = interpolate_num * weights / den
                    base = np.floor(raw).astype(int)
                    remain = int(interpolate_num - base.sum())
                    frac_order = np.argsort(raw - base)[::-1]
                    base[frac_order[:remain]] += 1
                    pre_blend_interpolate_num, linear_interpolate_num, post_blend_interpolate_num = base.tolist(
                    )

                sample_seg_list = []
                pre_blend_s = np.linspace(
                    pre_blend_seg.get_s0() + pre_blend_seg.get_length() / 2,
                    pre_blend_seg.get_s0() + pre_blend_seg.get_length(),
                    pre_blend_interpolate_num) if pre_blend_seg != [] else None
                linear_s = np.linspace(linear_seg.get_s0(),
                                       linear_seg.get_s0() + linear_seg_length,
                                       linear_interpolate_num)
                post_blend_s = np.linspace(
                    post_blend_seg.get_s0(),
                    post_blend_seg.get_s0() +
                    post_blend_seg_length / 2, post_blend_interpolate_num
                ) if post_blend_seg != [] else None

                if pre_blend_s is not None:
                    for s in pre_blend_s:
                        if s == pre_blend_s[0]:
                            continue
                        sample_seg_list.append(pre_blend_seg.get_config(s))
                for s in linear_s:
                    if pre_blend_s is not None and s == linear_s[0]:
                        continue
                    sample_seg_list.append(linear_seg.get_config(s))
                if post_blend_s is not None:
                    for s in post_blend_s:
                        if s == post_blend_s[0]:
                            continue
                        sample_seg_list.append(post_blend_seg.get_config(s))

                interpolate_path.extend(sample_seg_list)

            return StatusCode.SUCCESS, interpolate_path

    def _check_sample_step(self, path: list[np.ndarray], cartesian_step: float):
        if len(path) < 2:
            self.logger.error(
                f"The number of sample points in path should be at least 2, but got {path.shape[0]}"
            )
            return False
        if cartesian_step <= 0:
            self.logger.error(
                f"Invalid cartesian_step: {cartesian_step}, should be positive."
            )
            return False

        total_dist = 0.0
        for i in range(len(path) - 1):
            dist = np.linalg.norm(path[i + 1][:3, 3] - path[i][:3, 3])
            total_dist += dist
        if total_dist <= cartesian_step:
            return False
        return True

    def interpolate_se3_path_by_cartesian_step(
            self, path: list[np.ndarray],
            cartesian_step: float) -> list[np.ndarray]:
        if cartesian_step <= 0.:
            self.logger.error(
                f"Invalid cartesian_step: {cartesian_step}, should be positive."
            )
            return False, None
        if len(path) <= 1:
            self.logger.error(
                f"The number of points in path should be at least 2, but got {len(path)}"
            )
            return False, [poses[0].copy()] if len(path) == 1 else None

        poses = [np.asarray(p, dtype=float) for p in path]
        for idx, pose in enumerate(poses):
            if pose.shape != (4, 4):
                self.logger.error(
                    f"Invalid pose shape: {pose.shape} at index {idx}, each pose should be a 4x4 homogeneous transformation matrix."
                )
                return False, None

        interpolated_path = [poses[0].copy()]

        for idx in range(len(poses) - 1):
            pose0 = poses[idx]
            pose1 = poses[idx + 1]

            p0 = pose0[:3, 3]
            p1 = pose1[:3, 3]

            R0 = pose0[:3, :3]
            R1 = pose1[:3, :3]

            dist = np.linalg.norm(p1 - p0)

            if dist <= cartesian_step:
                interpolated_path.append(pose1.copy())
                continue

            key_rots = Rotation.from_matrix([R0, R1])
            slerp = Slerp([0.0, 1.0], key_rots)

            sample_distances = np.arange(cartesian_step, dist, cartesian_step)

            for s in sample_distances:
                ratio = s / dist

                pose = np.eye(4)
                pose[:3, 3] = p0 + ratio * (p1 - p0)
                pose[:3, :3] = slerp([ratio]).as_matrix()[0]

                interpolated_path.append(pose)

            interpolated_path.append(pose1.copy())

        return interpolated_path

    @time_consuming
    def plan_linear_whole_body_ik(self,
                                  start_qpos: np.ndarray,
                                  start_left_arm_ee_qpos: np.ndarray,
                                  start_right_arm_ee_qpos: np.ndarray,
                                  waypoints_dict: dict,
                                  cartesian_step: float = 0.005,
                                  speed_ratio: float = 1.0,
                                  use_collision_validator: bool = True,
                                  movement_mode: str = "eager",
                                  eye_focus_point: np.ndarray = None,
                                  upper_body_leaning_threshold: float = None,
                                  blend_tolerance: float = 0.005):
        """
        This function is used to plan linear trajectory of end effector in Cartesian space,
            and use whole body ik to solve the corresponding joint trajectory. 

        Notes:
            - Only execute the last end effector task in waypoints_dict
            - If there is not end-effectors, start_left_arm_ee_qpos and start_right_arm_ee_qpos can be set to None
            - Arms' teaching point should be in chassis base-link frame
        """
        self.logger.debug(
            f"current left arm tcp: {self.get_tcp('left_arm')[1]}")
        self.logger.debug(
            f"current right arm tcp: {self.get_tcp('right_arm')[1]}")
        self.logger.debug(
            f"plan_linear_whole_body_ik() input cur_qpos: {start_qpos}")
        self.logger.debug(
            f"plan_linear_whole_body_ik() input cur_ee_qpos: {start_left_arm_ee_qpos}, {start_right_arm_ee_qpos}"
        )
        self.logger.debug("plan_linear_whole_body_ik() input waypoints_dict:")
        self.logger.debug(waypoints_dict)
        self.logger.debug(
            f"plan_linear_whole_body_ik() input cartesian_step: {cartesian_step}"
        )
        self.logger.debug(
            f"plan_linear_whole_body_ik() input use_collision_validator: {use_collision_validator}"
        )
        self.logger.debug(
            f"plan_linear_whole_body_ik() input movement_mode: {movement_mode}")
        self.logger.debug(
            f"plan_linear_whole_body_ik() input eye_focus_point: {eye_focus_point}"
        )
        self.logger.debug(
            f"plan_linear_whole_body_ik() input upper_body_leaning_threshold: {upper_body_leaning_threshold}"
        )

        if movement_mode not in ["eager", "sync_endpoint"]:
            status = StatusCode.INPUT_INVALID
            self.logger.error(
                f"[{status.value}]:Unsupported movement mode: {movement_mode}, please choose from ['eager', 'sync_endpoint']"
            )
            return status, None, None, None

        left_parse_res, left_pos_raw_array, left_qvel_limits_array, _ = \
            self._parse_waypoints_from_dict("left_arm", waypoints_dict, speed_ratio, is_cartesian=True)
        right_parse_res, right_pos_raw_array, right_qvel_limits_array, _ = \
            self._parse_waypoints_from_dict("right_arm", waypoints_dict, speed_ratio, is_cartesian=True)
        left_arm_static_flag, right_arm_static_flag = False, False
        if left_parse_res == StatusCode.INPUT_EMPTY or right_parse_res == StatusCode.INPUT_EMPTY:
            if left_parse_res == StatusCode.INPUT_EMPTY:
                left_arm_static_flag = True
            if right_parse_res == StatusCode.INPUT_EMPTY:
                right_arm_static_flag = True
        elif left_parse_res.value < 0 or right_parse_res.value < 0:
            return StatusCode.FAIL, None, None, None

        res_left_fk, start_left_xpos = self.get_xpos("left_arm",
                                                     start_qpos,
                                                     is_local=False)
        res_right_fk, start_right_xpos = self.get_xpos("right_arm",
                                                       start_qpos,
                                                       is_local=False)

        if not left_arm_static_flag:
            left_pos_raw_list = [start_left_xpos] + \
                [user_pose_to_xpos(pose[:3], pose[3:]) for pose in left_pos_raw_array]
            left_qvel_limits_array = np.vstack((self.w1_params.default_qvel[
                self.w1_params.index_infos["left_arm"]["index"]],
                                                left_qvel_limits_array))
        else:
            left_pos_raw_list = [start_left_xpos]
            left_qvel_limits_array = np.atleast_2d(self.w1_params.default_qvel[
                self.w1_params.index_infos["left_arm"]["index"]])
        if not right_arm_static_flag:
            right_pos_raw_list = [start_right_xpos] + \
                [user_pose_to_xpos(pose[:3], pose[3:]) for pose in right_pos_raw_array]
            right_qvel_limits_array = np.vstack((self.w1_params.default_qvel[
                self.w1_params.index_infos["right_arm"]["index"]],
                                                 right_qvel_limits_array))
        else:
            right_pos_raw_list = [start_right_xpos]
            right_qvel_limits_array = np.atleast_2d(self.w1_params.default_qvel[
                self.w1_params.index_infos["right_arm"]["index"]])

        if movement_mode == "sync_point_by_point":
            if len(left_pos_raw_list) != len(right_pos_raw_list):
                status = StatusCode.DIM_MISMATCH
                self.logger.error(
                    f"[{status.value}]:In sync_point_by_point mode, The number of left arm waypoints and right arm waypoints should be the same."
                )
                return status, None, None, None

        if left_qvel_limits_array.shape[0] != len(
                left_pos_raw_list) or right_qvel_limits_array.shape[0] != len(
                    right_pos_raw_list):
            status = StatusCode.DIM_MISMATCH
            self.logger.error(
                f"[{status.value}]:The number of rows in qvel_limits_array should be the same as the number of waypoints."
            )
            return status, None, None, None

        left_prefix_dist_list = [0.]
        for idx in range(len(left_pos_raw_list) - 1):
            dist = np.linalg.norm(left_pos_raw_list[idx + 1][:3, 3] -
                                  left_pos_raw_list[idx][:3, 3])
            left_prefix_dist_list.append(left_prefix_dist_list[-1] + dist)
        right_prefix_dist_list = [0.]
        for idx in range(len(right_pos_raw_list) - 1):
            dist = np.linalg.norm(right_pos_raw_list[idx + 1][:3, 3] -
                                  right_pos_raw_list[idx][:3, 3])
            right_prefix_dist_list.append(right_prefix_dist_list[-1] + dist)

        if movement_mode == "sync_point_by_point":
            left_interpolate_num_list = self._cal_path_seg_interpolate_num(
                left_pos_raw_list, cartesian_step)
            right_interpolate_num_list = self._cal_path_seg_interpolate_num(
                right_pos_raw_list, cartesian_step)
            if len(left_interpolate_num_list) != len(
                    right_interpolate_num_list):
                status = StatusCode.INPUT_INVALID
                self.logger.error(
                    f"[{status.value}]:Left and right arm waypoint segment number mismatch."
                )
                return status, None, None, None

            interpolate_num_list = []
            for i in range(len(left_interpolate_num_list)):
                left_interpolate_num = left_interpolate_num_list[i]
                right_interpolate_num = right_interpolate_num_list[i]
                seg_num_sample = max(left_interpolate_num,
                                     right_interpolate_num)
                interpolate_num_list.append(seg_num_sample)

            left_bezier_planner = PathR3xSO3Bezier(left_pos_raw_list)
            if not left_bezier_planner.calc_path(PathType.Bezier5thCartesian,
                                                 0.01):
                status = StatusCode.PLAN_PATH_FAIL
                self.logger.error(
                    f"[{status.value}]:Left arm bezier path planning failed.")
                return status, None, None, None

            right_bezier_planner = PathR3xSO3Bezier(right_pos_raw_list)
            if not right_bezier_planner.calc_path(PathType.Bezier5thCartesian,
                                                  0.01):
                status = StatusCode.PLAN_PATH_FAIL
                self.logger.error(
                    f"[{status.value}]:Right arm bezier path planning failed.")
                return status, None, None, None

            res, left_xpos_sample_array = self._interpolate_bezier_path(
                left_bezier_planner,
                cartesian_step,
                interpolate_num_list=interpolate_num_list)
            if res != StatusCode.SUCCESS:
                status = StatusCode.PLAN_PATH_FAIL
                self.logger.error(
                    f"[{status.value}]:Left arm bezier path interpolation failed."
                )
                return status, None, None, None
            res, right_xpos_sample_array = self._interpolate_bezier_path(
                right_bezier_planner,
                cartesian_step,
                interpolate_num_list=interpolate_num_list)
            if res != StatusCode.SUCCESS:
                status = StatusCode.PLAN_PATH_FAIL
                self.logger.error(
                    f"[{status.value}]:Right arm bezier path interpolation failed."
                )
                return status, None, None, None

        if movement_mode == "eager":
            if not left_arm_static_flag:
                if check_SE3_sparse_waypoints(
                        left_pos_raw_list,
                        blend_tolerance) and self._check_sample_step(
                            left_pos_raw_list, cartesian_step):
                    left_bezier_planner = PathR3xSO3Bezier(left_pos_raw_list)
                    if not left_bezier_planner.calc_path(
                            PathType.Bezier5thCartesian, 0.01):
                        status = StatusCode.PLAN_PATH_FAIL
                        self.logger.error(
                            f"[{status.value}]:Left arm bezier path planning failed."
                        )
                        return status, None, None, None
                    res, left_xpos_sample_array = self._interpolate_bezier_path(
                        left_bezier_planner, cartesian_step)
                    if res != StatusCode.SUCCESS:
                        status = StatusCode.PLAN_PATH_FAIL
                        self.logger.error(
                            f"[{status.value}]:Left arm bezier path interpolation failed."
                        )
                        return status, None, None, None
                elif check_SE3_sparse_waypoints(
                        left_pos_raw_list,
                        blend_tolerance) and not self._check_sample_step(
                            left_pos_raw_list, cartesian_step):
                    left_xpos_sample_array = np.array(left_pos_raw_list)
                elif not check_SE3_sparse_waypoints(
                        left_pos_raw_list,
                        blend_tolerance) and self._check_sample_step(
                            left_pos_raw_list, cartesian_step):
                    left_xpos_sample_array = self.interpolate_se3_path_by_cartesian_step(
                        left_pos_raw_list, cartesian_step)
                    left_xpos_sample_array = np.array(left_xpos_sample_array)
                else:
                    left_xpos_sample_array = np.array(left_pos_raw_list)
            else:
                left_xpos_sample_array = np.array(left_pos_raw_list)

            if not right_arm_static_flag:
                if check_SE3_sparse_waypoints(
                        right_pos_raw_list,
                        blend_tolerance) and self._check_sample_step(
                            right_pos_raw_list, cartesian_step):
                    right_bezier_planner = PathR3xSO3Bezier(right_pos_raw_list)
                    if not right_bezier_planner.calc_path(
                            PathType.Bezier5thCartesian, 0.01):
                        status = StatusCode.PLAN_PATH_FAIL
                        self.logger.error(
                            f"[{status.value}]:Right arm bezier path planning failed."
                        )
                        return status, None, None, None

                    res, right_xpos_sample_array = self._interpolate_bezier_path(
                        right_bezier_planner, cartesian_step)
                    if res != StatusCode.SUCCESS:
                        status = StatusCode.PLAN_PATH_FAIL
                        self.logger.error(
                            f"[{status.value}]:Right arm bezier path interpolation failed."
                        )
                        return status, None, None, None
                elif check_SE3_sparse_waypoints(
                        right_pos_raw_list,
                        blend_tolerance) and not self._check_sample_step(
                            right_pos_raw_list, cartesian_step):
                    right_xpos_sample_array = np.array(right_pos_raw_list)
                elif not check_SE3_sparse_waypoints(
                        right_pos_raw_list,
                        blend_tolerance) and self._check_sample_step(
                            right_pos_raw_list, cartesian_step):
                    right_xpos_sample_array = self.interpolate_se3_path_by_cartesian_step(
                        right_pos_raw_list, cartesian_step)
                    right_xpos_sample_array = np.array(right_xpos_sample_array)
                else:
                    right_xpos_sample_array = np.array(right_pos_raw_list)
            else:
                right_xpos_sample_array = np.array(right_pos_raw_list)

            # pad the shorter one to make them have the same number of waypoints
            left_len = left_xpos_sample_array.shape[0]
            right_len = right_xpos_sample_array.shape[0]

            if left_len < right_len:
                pad = np.repeat(left_xpos_sample_array[-1:],
                                right_len - left_len,
                                axis=0)
                left_xpos_sample_array = np.concatenate(
                    [left_xpos_sample_array, pad], axis=0)
            elif right_len < left_len:
                pad = np.repeat(right_xpos_sample_array[-1:],
                                left_len - right_len,
                                axis=0)
                right_xpos_sample_array = np.concatenate(
                    [right_xpos_sample_array, pad], axis=0)

            left_xpos_sample_array = list(left_xpos_sample_array)
            right_xpos_sample_array = list(right_xpos_sample_array)

        if movement_mode == "sync_endpoint":
            if left_arm_static_flag or right_arm_static_flag:
                status = StatusCode.INPUT_INVALID
                self.logger.error(
                    f"[{status.value}]:In sync_endpoint mode, both arms should not be static."
                )
                return status, None, None, None
            if not check_SE3_sparse_waypoints(
                    left_pos_raw_list,
                    blend_tolerance) or not check_SE3_sparse_waypoints(
                        right_pos_raw_list, blend_tolerance):
                status = StatusCode.INPUT_INVALID
                self.logger.error(
                    f"[{status.value}]:In sync_endpoint mode, the waypoints should be sparse for both arms."
                )
                return status, None, None, None
            if not self._check_sample_step(
                    left_pos_raw_list,
                    cartesian_step) or not self._check_sample_step(
                        right_pos_raw_list, cartesian_step):
                status = StatusCode.INPUT_INVALID
                self.logger.error(
                    f"[{status.value}]:In sync_endpoint mode, the cartesian sample step should be smaller than arm movement distance."
                )
                return status, None, None, None
            left_interpolate_num_list = self._cal_path_seg_interpolate_num(
                left_pos_raw_list, cartesian_step)
            right_interpolate_num_list = self._cal_path_seg_interpolate_num(
                right_pos_raw_list, cartesian_step)
            left_interpolate_num = sum(left_interpolate_num_list)
            right_interpolate_num = sum(right_interpolate_num_list)
            total_interpolate_num = max(left_interpolate_num,
                                        right_interpolate_num)

            left_bezier_planner = PathR3xSO3Bezier(left_pos_raw_list)
            if not left_bezier_planner.calc_path(PathType.Bezier5thCartesian,
                                                 0.01):
                status = StatusCode.PLAN_PATH_FAIL
                self.logger.error(
                    f"[{status.value}]:Left arm bezier path planning failed.")
                return status, None, None, None

            right_bezier_planner = PathR3xSO3Bezier(right_pos_raw_list)
            if not right_bezier_planner.calc_path(PathType.Bezier5thCartesian,
                                                  0.01):
                status = StatusCode.PLAN_PATH_FAIL
                self.logger.error(
                    f"[{status.value}]:Right arm bezier path planning failed.")
                return status, None, None, None

            res, left_xpos_sample_array = self._interpolate_bezier_path(
                left_bezier_planner, interpolate_num=total_interpolate_num)
            if res != StatusCode.SUCCESS:
                status = StatusCode.PLAN_PATH_FAIL
                self.logger.error(
                    f"[{status.value}]:Left arm bezier path interpolation failed."
                )
                return status, None, None, None
            res, right_xpos_sample_array = self._interpolate_bezier_path(
                right_bezier_planner, interpolate_num=total_interpolate_num)
            if res != StatusCode.SUCCESS:
                status = StatusCode.PLAN_PATH_FAIL
                self.logger.error(
                    f"[{status.value}]:Right arm bezier path interpolation failed."
                )
                return status, None, None, None

            left_xpos_sample_array = list(left_xpos_sample_array)
            right_xpos_sample_array = list(right_xpos_sample_array)

        if len(left_xpos_sample_array) != len(right_xpos_sample_array):
            status = StatusCode.PLAN_PATH_FAIL
            self.logger.error(
                f"[{status.value}]:Left and right arm sample number mismatch after interpolation."
            )
            return status, None, None, None

        # vel align
        if left_qvel_limits_array is None:
            left_qvel_limits_array_ = None
        else:
            idx = 0
            left_qvel_limits_list_ = [left_qvel_limits_array[idx]]
            cur_left_xpos = left_xpos_sample_array[0]
            cur_dist = 0.
            for left_xpos in left_xpos_sample_array[1:-1]:
                cur_dist += np.linalg.norm(left_xpos[:3, 3] -
                                           cur_left_xpos[:3, 3])
                if cur_dist >= left_prefix_dist_list[idx] + 1e-6:
                    idx += 1
                left_qvel_limits_list_.append(left_qvel_limits_array[idx])
                cur_left_xpos = left_xpos
            left_qvel_limits_list_.append(left_qvel_limits_array[-1])
        left_qvel_limits_array_ = np.array(left_qvel_limits_list_)

        if right_qvel_limits_array is None:
            right_qvel_limits_array_ = None
        else:
            idx = 0
            right_qvel_limits_list_ = [right_qvel_limits_array[idx]]
            cur_right_xpos = right_xpos_sample_array[0]
            cur_dist = 0.
            for right_xpos in right_xpos_sample_array[1:-1]:
                cur_dist += np.linalg.norm(right_xpos[:3, 3] -
                                           cur_right_xpos[:3, 3])
                if cur_dist >= right_prefix_dist_list[idx] + 1e-6:
                    idx += 1
                right_qvel_limits_list_.append(right_qvel_limits_array[idx])
                cur_right_xpos = right_xpos
            right_qvel_limits_list_.append(right_qvel_limits_array[-1])
        right_qvel_limits_array_ = np.array(right_qvel_limits_list_)

        torso_qvel_limits_array_ = np.tile(
            self.w1_params.default_qvel[self.w1_params.index_infos["torso"]
                                        ["index"]],
            (left_qvel_limits_array_.shape[0], 1))
        head_qvel_limits_array_ = np.tile(
            self.w1_params.default_qvel[self.w1_params.index_infos["head"]
                                        ["index"]],
            (left_qvel_limits_array_.shape[0], 1))
        qvel_limits_array_ = np.hstack([
            torso_qvel_limits_array_, head_qvel_limits_array_,
            left_qvel_limits_array_, right_qvel_limits_array_
        ])

        res, full_body_joint_traj = self.cartesian_joint_planner.plan_whole_body(
            start_qpos=start_qpos,
            left_end_effector_traj=np.asarray(left_xpos_sample_array),
            right_end_effector_traj=np.asarray(right_xpos_sample_array),
            upper_body_leaning_threshold=upper_body_leaning_threshold,
            eye_focus_point=eye_focus_point,
            is_strict_solution=False)

        if not res:
            status = StatusCode.PLAN_TRAJ_FAIL
            self.logger.error(
                f"[{status.value}]:Whole body joint traj planning failed in plan_linear_whole_body_ik."
            )
            return status, None, None, None

        # resolve for the last target
        left_last_target_xpos = left_xpos_sample_array[-1]
        right_last_target_xpos = right_xpos_sample_array[-1] @ np.linalg.inv(
            self.cartesian_joint_planner.right_tcp)

        res, full_body_last_qpos = self.cartesian_joint_planner.whole_body_ik_solver.solve_ik(
            left_arm_target=left_last_target_xpos,
            right_arm_target=right_last_target_xpos,
            ref_qpos=full_body_joint_traj[-1],
            upper_body_leaning_threshold=upper_body_leaning_threshold,
            eye_focus_point=eye_focus_point,
            is_strict_solution=True)

        if not res:
            status = StatusCode.PLAN_TRAJ_FAIL
            self.logger.error(
                f"[{status.value}]:Last point strictly IK solving failed in plan_linear_whole_body_ik."
            )
            return status, None, None, None

        full_body_joint_traj = np.vstack(
            [full_body_joint_traj, full_body_last_qpos])
        qvel_limits_array_ = np.vstack(
            [qvel_limits_array_, qvel_limits_array_[-1]])

        # combine the start qpos
        full_body_joint_traj = np.vstack([start_qpos, full_body_joint_traj])
        qvel_limits_array_ = np.vstack(
            [qvel_limits_array_[0], qvel_limits_array_])

        res, full_body_joint_traj_ = self.cartesian_joint_planner.generate_dense_whole_body_joint_traj(
            whole_body_joint_traj=full_body_joint_traj,
            qvel_limits_array=qvel_limits_array_,
            current_qpos=start_qpos)
        if not res:
            status = StatusCode.PLAN_TRAJ_FAIL
            self.logger.error(
                f"[{status.value}]:Generate dense whole body joint traj failed in plan_linear_whole_body_ik."
            )
            return status, None, None, None

        # TODO: Add end effector trigger
        left_ee_target_list = waypoints_dict.get("left_arm_ee")
        right_ee_target_list = waypoints_dict.get("right_arm_ee")
        left_arm_ee_trigger_list, right_arm_ee_trigger_list = None, None
        left_wait_time, right_wait_time = 0.0, 0.0
        if left_ee_target_list is not None and left_ee_target_list != []:
            left_ee_target = left_ee_target_list[-1]
            left_arm_ee_trigger_list = [
                (full_body_joint_traj_.shape[0] +
                 int(left_ee_target[0] / self.TIME_STEP), left_ee_target[1])
            ]
            left_wait_time = left_ee_target[0] + left_ee_target[2]
        if right_ee_target_list is not None and right_ee_target_list != []:
            right_ee_target = right_ee_target_list[-1]
            right_arm_ee_trigger_list = [
                (full_body_joint_traj_.shape[0] +
                 int(right_ee_target[0] / self.TIME_STEP), right_ee_target[1])
            ]
            right_wait_time = right_ee_target[0] + right_ee_target[2]

        full_body_wait_time = max(left_wait_time, right_wait_time)
        full_body_joint_traj_ = np.vstack([
            full_body_joint_traj_,
            np.tile(full_body_joint_traj_[-1, :],
                    (int(np.ceil(full_body_wait_time / self.TIME_STEP)), 1))
        ])

        if use_collision_validator:
            if self._is_traj_collision(qpos_array=full_body_joint_traj_,
                                       left_arm_start_ee_qpos=None,
                                       right_arm_start_ee_qpos=None,
                                       left_arm_ee_trigger_list=None,
                                       right_arm_ee_trigger_list=None
                                      ) == StatusCode.COLLISION_FAIL:
                status = StatusCode.COLLISION_FAIL
                self.logger.error(
                    f"[{status.value}]:Collision detected in plan_linear_whole_body_ik."
                )
                return status, None, None, None

        return StatusCode.SUCCESS, full_body_joint_traj_, left_arm_ee_trigger_list, right_arm_ee_trigger_list
