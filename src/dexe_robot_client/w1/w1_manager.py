# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import numpy as np
np.set_printoptions(precision=4, suppress=True)
import time, threading
import pathlib, json
from copy import deepcopy
import atexit
from loop_rate_limiters import RateLimiter
import rclpy
from rclpy.executors import MultiThreadedExecutor

from ..common import time_consuming, get_logger, DATA_DIR, W1ArmType, W1Version, generate_w1_urdf
from ..end_effector_client import EndEffectorType, create_end_effector_client
from ..navigation_client import ChassisClient
from ..robot_client import RobotClientRos2 as RobotClient
from ..robot_client import MotionGroup, Status
from .path_planner.bezier_blend_path import interpolate_cartesian_path
from .trajectory_planner.toppra_trajectory_planner import ToppraTrajectoryPlanner
from .utils.w1_params import W1Params
from .utils.unit_conversion import user_pose_to_xpos, xpos_to_user_pose
from .path_planner.fcl_checker import FCLChecker
from .path_planner.planner_pinocchio import Planner_Pin
from ..common.mapping_index import *
from .path_planner.utils.robot_type import *
from .path_planner.single_planner import SinglePlanner

from dexe_rp import DexforceW1Kinematics, SRSConfig


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

        self.TIME_STEP = 0.01 # 100Hz

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

        self.w1_params = W1Params(self._arm_type, 
                                  self._w1_version, 
                                  self._end_effector_type)

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

        # for visual
        if not generate_w1_urdf(arm_type, w1_version, end_effector_type)[1]:
            raise ValueError(f"Found URDF failed.")

        # for collision check
        if self._end_effector_type is not None:
            if not generate_w1_urdf(arm_type, w1_version, None)[1]:
                raise ValueError(f"Found URDF failed.")
        # initialize fcl checker and pinocchio model and planner
        self._init_fcl_checker(arm_type=arm_type, w1_version=w1_version, end_effector_type=end_effector_type)
        self._init_pinocchio(arm_type=arm_type, w1_version=w1_version, end_effector_type=end_effector_type)
        self.real_torso_idx, self.real_head_idx, self.real_arml_idx, self.real_armr_idx = get_real_idx()
        self._init_planner(arm_type=arm_type, w1_version=w1_version, end_effector_type=end_effector_type)
 
        self._stop_move_event = threading.Event()
        self._stop_move_event.clear()


    def _init_fcl_checker(self, arm_type: W1ArmType, w1_version: W1Version = W1Version.V2, end_effector_type: EndEffectorType = None):
        if w1_version == W1Version.V2:
            if arm_type == W1ArmType.ANTHROPOMORPHIC:
                fcl_robot_type = RobotType.W1V02_1
            elif arm_type == W1ArmType.INDUSTRIAL:
                fcl_robot_type = RobotType.W1V02_2
            else:
                raise ValueError(f"Not supported arm type: {arm_type} for fcl checker")
                return
            self.fcl_checker = FCLChecker(fcl_robot_type, end_effector_type)
        else:
            raise ValueError(f"Not supported w1 version: {w1_version} for fcl checker")
        
    def load_fcl_environment(self, file_path: str = None):
        """
        file_path: the path of fcl environment json file path
        """
        if file_path is None:
            env_json_path = get_env_json_path()
        else:
            env_json_path = file_path
        self.fcl_checker.load_env_json(env_json_path)

    def _init_pinocchio(self, arm_type: W1ArmType = W1ArmType.ANTHROPOMORPHIC, 
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
                raise ValueError(f"Not supported arm type: {arm_type} for pinocchio model")
                return
            if end_effector_type is None or isinstance(end_effector_type, EndEffectorType):
                pin_ee_type = end_effector_type
            else:
                raise ValueError(f"Not supported end_effector_type: {end_effector_type} for pinocchio model")
                return
            
            self._pin_kine = Planner_Pin(robot_type=pin_robot_type, end_effector_type=pin_ee_type)
            
        else:
            raise ValueError(f"Not supported w1 version: {w1_version} for pinocchio model")
        
    def _init_planner(self, arm_type: W1ArmType, w1_version: W1Version.V2, end_effector_type: EndEffectorType = None):
        if w1_version == W1Version.V2:
            if arm_type == W1ArmType.ANTHROPOMORPHIC:
                planner_robot_type = RobotType.W1V02_1
            elif arm_type == W1ArmType.INDUSTRIAL:
                planner_robot_type = RobotType.W1V02_2
            else:
                raise ValueError(f"Not supported arm type: {arm_type} for planner")
                return
            SinglePlanner.set_arms_cfg(SINGLE_ARM_DIM, SINGLE_ARM_DIM, LARM_BOUND, RARM_BOUND)
            SinglePlanner.set_robot_cfg(robot_type=planner_robot_type, ee_type=end_effector_type)
            # SinglePlanner.set_env_json(get_env_json_path())
            self.siso_planner = SinglePlanner()

    def load_planner_environment(self, file_path: str = None):
        """
        file_path: the path of planner environment json file path
        """
        if file_path is None:
            env_json_path = get_env_json_path()
        else:
            env_json_path = file_path
        self.siso_planner.set_env_json(env_json_path)

    def _init_clients(self):
        if not rclpy.ok():
            rclpy.init()

        self._robot_client_sim = RobotClient(is_sim = True)
        self._robot_client_real = RobotClient(is_sim = False)
        self._chassis_client_real = ChassisClient(is_sim = False)
        self.robot_client = self._robot_client_sim

        if self._end_effector_type is None:
            self.logger.info(f"No movable end effector.")
            if self._use_nav:
                self._clients = [self._robot_client_sim, self._robot_client_real,
                                self._chassis_client_real]
            else:
                self._clients = [self._robot_client_sim, self._robot_client_real]
        else:
            self._left_ee_client_sim = create_end_effector_client(is_left = True, is_sim = True, 
                                                                  end_effector_type = self._end_effector_type)
            self._right_ee_client_sim = create_end_effector_client(is_left = False, is_sim = True, 
                                                                  end_effector_type = self._end_effector_type)
            self._left_ee_client_real = create_end_effector_client(is_left = True, is_sim = False, 
                                                                end_effector_type = self._end_effector_type)
            self._right_ee_client_real = create_end_effector_client(is_left = False, is_sim = False, 
                                                                end_effector_type = self._end_effector_type)
            self.left_ee_client = self._left_ee_client_sim
            self.right_ee_client = self._right_ee_client_sim

            if self._use_nav:
                self._clients = [self._robot_client_sim, self._robot_client_real, 
                                self._left_ee_client_sim, self._left_ee_client_real, 
                                self._right_ee_client_sim, self._right_ee_client_real,
                                self._chassis_client_real]
            else:
                self._clients = [self._robot_client_sim, self._robot_client_real, 
                                self._left_ee_client_sim, self._left_ee_client_real, 
                                self._right_ee_client_sim, self._right_ee_client_real]

        self._is_sim = True

        self._executor = MultiThreadedExecutor()
        for node in self._clients:
            self._executor.add_node(node)
        self._node_thread = threading.Thread(target=self._executor.spin, 
                                            daemon=True)
        self._node_thread.start()
        self.logger.info("Ros2 client initialized.")

    def _destroy_clients(self):
        try:
            if rclpy.ok():
                rclpy.shutdown() # 停止ros触发spin

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
        ik_nearst_weight = np.array([2., 2., 2., 1., 1., 1., 1.])
        self.kine = DexforceW1Kinematics(self._arm_type == W1ArmType.ANTHROPOMORPHIC)
        self.kine.left_arm_kine.set_ik_nearst_weight(ik_nearst_weight)
        self.kine.right_arm_kine.set_ik_nearst_weight(ik_nearst_weight)
        self.kine.set_user_qpos_limits(self.w1_params.qpos_lb, self.w1_params.qpos_ub)
        
        self.logger.info("Initialize kinematic solver.")

    def _has_arm_ee_client(self, name: str, verbose: bool = True) -> bool:
        if name in self.w1_params.arm_ee_names:
            if name == "left_arm_ee" and self.left_ee_client is not None:
                return True
            if name == "right_arm_ee" and self.right_ee_client is not None:
                return True
            if verbose:
                self.logger.error(f"{name} has not initialized")
            return False

        self.logger.error(f"{self.w1_params.arm_ee_names} is supported, except {name}.")
        return False
    
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
            self.logger.error(f"{self.w1_params.names} is supported, except {name}.")
            return False
        return True

    def _check_arm_name(self, name: str) -> bool:
        support_names = self.w1_params.arm_names
        if name not in support_names:
            self.logger.error(f"{support_names} is supported, except {name}.")
            return False
        return True

    def _check_qpos_dimension(self, name: str, qpos) -> bool:
        """支持component和ee的检查"""
        if name in self.w1_params.arm_ee_names:
            if not self._has_arm_ee_client(name):
                return False
            dofs = self.w1_params.arm_ee_dofs
        elif self._check_name(name):
            dofs = self.w1_params.index_infos[name]["dofs"]
        else:
            return False
        
        if isinstance(qpos, np.ndarray):
            qpos = np.atleast_2d(qpos)
            if qpos.shape[1] == dofs:
                return True
        else: # list
            if all(len(temp) == dofs for temp in qpos):
                return True

        self.logger.error(f"{name} qpos dimension mismatch")
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
            self.logger.error(f"{name} {invalid_idx}th qpos({qpos.tolist()}) " + \
                              f"excceeded limits([{qpos_lb[invalid_idx-1]}, {qpos_ub[invalid_idx-1]}]).")
            return False
        return True
    
    def _check_arm_ee_states(self, arm_ee_state_list: list) -> bool:
        if len(arm_ee_state_list) == 0:
            self.logger.error("Unexpected error, FIXME!")
            return False
        
        valid_ee_state = [ee_state for ee_state in arm_ee_state_list if ee_state is not None]
        if not all(isinstance(ee_state, (float, list, float)) for ee_state in valid_ee_state):
            self.logger.error(f"arm_ee_state should be [pre_delay, ee_qpos, post_delay].")
            return False
        if not self._check_qpos_dimension("left_arm_ee", 
                                          [ee_state[1] for ee_state in valid_ee_state]):
            return False
        return True

    def _save_home_qpos(self, file_path = DATA_DIR / "config.json") -> bool:
        file_path_ = pathlib.Path(file_path)
        if not file_path_.exists() or file_path_.stat().st_size == 0:
            self.logger.error(f"{str(file_path)} invalid.")
            return False

        with open(file_path, "r") as f:
            json_data = json.load(f)

        json_data["home_qpos"] = self.get_home_qpos().tolist()

        with open(file_path, "w") as f:
            json.dump(json_data, f, indent=4, ensure_ascii=False)
        self.w1_params.update_params(file_path)
        return True

    def _qpos_align(self, 
                    torso_qpos_array: np.ndarray, 
                    head_qpos_array: np.ndarray, 
                    left_arm_qpos_array: np.ndarray,
                    right_arm_qpos_array: np.ndarray) -> tuple[bool, np.ndarray]:
        """轨迹对齐，按先运动后等待的模式进行补齐"""
        qpos_array_list = [np.atleast_2d(torso_qpos_array),
                        np.atleast_2d(head_qpos_array), 
                        np.atleast_2d(left_arm_qpos_array),
                        np.atleast_2d(right_arm_qpos_array)]        
        if all(q.size == 0 for q in qpos_array_list):
            self.logger.error("No qpos to be aligned")
            return False, None
        
        max_count = max(q.shape[0] for q in qpos_array_list)
        
        temp_arrays = []
        for qpos_array in qpos_array_list:
            pad_count = max_count - qpos_array.shape[0]
            if pad_count > 0:
                padded = np.vstack([qpos_array,
                                    np.tile(qpos_array[-1], (pad_count, 1))])
            else:
                padded = qpos_array
            temp_arrays.append(padded)

        return True, np.hstack(temp_arrays)

    ######## 基础API #############
    def connect(self, timeout_sec: float = 0.5) -> bool:
        """连接client(sim/real)

        Returns:
            bool: 
        """
        return self.is_connected(timeout_sec)

    def is_connected(self, timeout_sec: float = 0.1) -> bool:
        """检查所有client是否处于连接状态

        Returns:
            bool: 
        """
        res = True
        if not self.robot_client.is_connected(timeout_sec):
            self.logger.info(f"{self.robot_client.node_name} is not connected!")
            res = False
        if self._has_arm_ee_client("left_arm_ee", False):
            if not self.left_ee_client.is_connected(timeout_sec):
                self.logger.info(f"{self.left_ee_client.node_name} is not connected!")
                res = False
            if not self.right_ee_client.is_connected(timeout_sec):
                self.logger.info(f"{self.right_ee_client.node_name} is not connected!")
                res = False
        if( self._use_nav and not self.chassis_client.is_connected()):
            self.logger.error(f"{self.chassis_client.node_name} is not connected!")
            res = False
        return res

    def enable(self, timeout_sec: float = 0.5) -> bool:
        self.logger.warning("Not implemented (Currently it will automatically OP).")
        # TODO: self.robot_client.enable(timeout_sec)
        return True

    def is_enabled(self) -> bool:
        """检查robot是否处于OP状态

        Returns:
            bool: 
        """
        if not self.robot_client.is_connected(0.5):
            self.logger.info(f"{self.robot_client.node_name} is not connected!")
            return False
        if self.robot_client.status == Status.Init:
            self.logger.info(f"{self.robot_client.node_name} is not enabled!")
            return False
        return True

    def clear_robot_error(self) -> bool:
        """清除机器人错误

        Returns:
            bool: 
        """
        if self._is_sim:
            self.logger.error("Only support real mode.")
            return False
        res = self.robot_client.clear_error()
        if res == -1:
            self.logger.error("Clear robot error failed!")
            return False
        return True

    def set_drag_mode(self, enable_drag: bool) -> bool:
        """开关机器人拖动示教模式

        Args:
            enable_drag (bool): 是否开启拖动示教模式

        Returns:
            bool: 
        """
        if self._is_sim:
            self.logger.error("Only support real mode.")
            return False
        res = self.robot_client.set_drag_mode(enable_drag)
        if res == -1:
            self.logger.error("Set drag mode failed!")
            return False
        return True

    def set_safe_level(self, safe_level: float) -> bool:
        """设置机器人安全等级

        Args:
            safe_level (float): 安全级别, 取值范围 [0.1, 1], 默认为0.5  
                                0.1时机械臂柔性最大,1.0时机械臂刚性最大   
                                值越小机械臂柔性越大越安全, 刚性越大运动跟踪性能越好但碰撞时容易造成机械臂损坏

        Returns:
            bool: 
        """
        if self._is_sim:
            self.logger.error("Only support real mode.")
            return False
        res = self.robot_client.set_safe_level(safe_level)
        if res == -1:
            self.logger.error("Set save level failed!")
            return False
        return True

    def set_sim_mode(self, enable_sim: bool) -> bool:
        """设置仿真/真实模式"""
        if self._is_sim == enable_sim:
            if self._is_sim:
                self.logger.info("W1 manager already in sim mode.")
            else:
                self.logger.info("W1 manager already in real mode.")
            return True
        
        if enable_sim:
            target_name = "sim"
            target_robot_client = self._robot_client_sim
            target_left_ee_client = self._left_ee_client_sim
            target_right_ee_client = self._right_ee_client_sim
            target_chassis_client = None
            if self._use_nav:
                self.logger.info("Navigation client only support real mode. use_nav set to False.")
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
            self.logger.error(f"{target_robot_client.node_name} is not connected!")
            return False
        if target_robot_client.status not in [Status.Init, Status.Idle]:
            self.logger.error(f"{target_robot_client.node_name} status: {target_robot_client.status}, can not switch mode!")
            return False

        if self._end_effector_type is not None:
            if not target_left_ee_client.is_connected():
                self.logger.error(f"{target_left_ee_client.node_name} is not connected!")
                return False
            if not target_right_ee_client.is_connected():
                self.logger.error(f"{target_right_ee_client.node_name} is not connected!")
                return False
        
        if self._use_nav and target_chassis_client is not None and not target_chassis_client.is_connected():
            self.logger.error(f"{target_chassis_client.node_name} is not connected!")
            return False
        
        # 检查current client
        if not self.robot_client.is_connected():
            self.logger.error(f"{self.robot_client.node_name} is not connected!")
            return False
        if self.robot_client.status not in [Status.Init, Status.Idle]:
            self.logger.error(f"{self.robot_client.node_name} status: {self.robot_client.status}, can not switch mode!")
            return False

        if self._end_effector_type is not None:
            if not self.left_ee_client.is_connected():
                self.logger.error(f"{self.left_ee_client.node_name} is not connected!")
                return False
            if not self.right_ee_client.is_connected():
                self.logger.error(f"{self.right_ee_client.node_name} is not connected!")
                return False
        
        if self._use_nav and target_chassis_client is not None and not target_chassis_client.is_connected():
            self.logger.error(f"{self.chassis_client.node_name} is not connected!")
            return False


        self.robot_client = target_robot_client
        if self._end_effector_type is not None:
            self.left_ee_client = target_left_ee_client
            self.right_ee_client = target_right_ee_client
        self._is_sim = enable_sim
        if self._use_nav and target_chassis_client is not None:
            self.chassis_client = target_chassis_client
        self.logger.info(f"W1 manager set to {target_name} mode.")
        return True

    def move_chassis_nav2(self, base_position: np.ndarray) -> bool:
        """移动底盘至目标位置，使用导航栈"""
        if not self._use_nav:
            self.logger.error("Navigation function is disabled.")
            return False
        if not self._has_chassis_client():
            self.logger.error("Chassis client has not initialized")
            return False
        return self._get_chassis_client().set_chassis_nav2_position(base_position)
    
    def move_chassis_ctrl(self, base_position: np.ndarray) -> bool:
        """移动底盘至目标位置，使用底盘控制接口"""
        if not self._use_nav:
            self.logger.error("Navigation function is disabled.")
            return False
        if not self._has_chassis_client():
            self.logger.error("Chassis client has not initialized")
            return False
        return self._get_chassis_client().set_chassis_control_position(base_position)
    
    def get_current_chassis_pose(self) -> tuple[bool, np.ndarray]:
        """获取底盘当前位姿[x(m), y(m), yaw(angle)]"""
        if not self._use_nav:
            self.logger.error("Navigation function is disabled.")
            return False, None
        if not self._has_chassis_client():
            self.logger.error("Chassis client has not initialized")
            return False, None
        return self._get_chassis_client().get_chassis_position()

    def set_ee_qpos(self, name: str, qpos: np.ndarray) -> bool:
        """控制end effector运动

        Args:
            name (str): ["left_arm_ee", "right_arm_ee"]
            qpos (np.ndarray): 期望位置 0~100

        Returns:
            bool
        """
        qpos_ = np.array(qpos)
        if not self._has_arm_ee_client(name, False):
            return False
        if not self._check_qpos_dimension(name, qpos_):
            return False
        return self._get_arm_ee_client(name).set_qpos(qpos_)

    def set_home_qpos(self, 
                      home_qpos: np.ndarray, 
                      save_to_file: bool = False) -> bool:
        """设置home qpos

        Args:
            home_qpos (np.ndarray): 
            save_to_file (bool, optional): 是否保存至默认路径data/config.json. Defaults to False.

        Returns:
            bool: 
        """
        if not self._check_qpos_limits("full_body", home_qpos):
            return False
        self.w1_params.home_qpos = home_qpos
        if save_to_file:
            return self._save_home_qpos()
        return True

    def get_home_qpos(self) -> np.ndarray:
        return self.w1_params.home_qpos
    
    def get_qpos_limits(self, name: str = "full_body") -> tuple[bool, np.ndarray, np.ndarray]:
        """获取特定部件(支持ee)的关节极限

        Args:
            name (str, optional): ["full_body", "torso", "head", "left_arm", "right_arm", 
                                    "left_arm_ee", "right_arm_ee"]. Defaults to "full_body".

        Returns:
            tuple[bool, np.ndarray, np.ndarray]: 状态码；下极限；上极限
        """
        if name in self.w1_params.arm_ee_names:
            if not self._has_arm_ee_client(name):
                return False, None, None
            qpos_lb = np.array([0.]*self.w1_params.arm_ee_dofs)
            qpos_ub = np.array([100.]*self.w1_params.arm_ee_dofs)

        elif not self._check_name(name):
            return False, None, None
        else:
            qpos_lb = self.w1_params.qpos_lb[self.w1_params.index_infos[name]["index"]]
            qpos_ub = self.w1_params.qpos_ub[self.w1_params.index_infos[name]["index"]]
        return True, qpos_lb, qpos_ub

    def get_current_qpos(self, name: str = "full_body") -> tuple[bool, np.ndarray]:
        """获取特定部件(支持ee)的当前关节位置

        Args:
            name (str, optional): ["full_body", "torso", "head", "left_arm", "right_arm", 
                                    "left_arm_ee", "right_arm_ee"]. Defaults to "full_body".

        Returns:
            tuple[bool, np.ndarray]
        """
        support_names = list(self.w1_params.names) + self.w1_params.arm_ee_names
        if name not in support_names:
            self.logger.error(f"{support_names} is supported, except {name}.")
            return False, None
        
        if name in self.w1_params.arm_ee_names:
            if not self._has_arm_ee_client(name, False):
                return False, None
            return self._get_arm_ee_client(name).get_qpos()
        
        robot_info = self.robot_client.get_info()
        if robot_info.joint_position is None:
            self.logger.error(f"Get current robot info failed, status: {robot_info.status}.")
            return False, None
        
        cur_qpos = np.array(robot_info.joint_position)

        if name == "full_body":
            return True, cur_qpos
        return True, cur_qpos[self.w1_params.index_infos[name]["index"]]

    def get_current_xpos(self, name: str, is_local: bool = True) -> tuple[bool, np.ndarray]:
        """is_local=True，获取left_arm/right_arm相对于arm base坐标系的笛卡尔位姿
        
            is_local=False，获取["head", "head_flange" 
                                 "left_arm_base", "left_arm", "left_arm_flange", 
                                "right_arm_base", "right_arm", "right_arm_flange"]
                                相对于chassis坐标系的笛卡尔位姿

        Args:
            name (str): ["left_arm", "right_arm"]
            is_local (bool, optional): w.r.t. arm base frame or chassis frame. Defaults to True.

        Returns:
            tuple[bool, np.ndarray]: 
        """
        if is_local:
            res, cur_qpos = self.get_current_qpos(name)
            if not res:
                return False, None
        else:
            res, cur_qpos = self.get_current_qpos("full_body")
            if not res:
                return False, None
        return self.get_xpos(name, cur_qpos, is_local)

    def get_xpos(self, name: str, qpos: np.ndarray, is_local: bool = True) -> tuple[bool, np.ndarray]:
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
            tuple[bool, np.ndarray]: 
        """        
        if is_local:
            if not self._check_qpos_dimension(name, qpos):
                return False, None
            
            arm_kine = self._get_arm_kine(name)
            if arm_kine is None:
                return False, None
            
            return arm_kine.get_fk(qpos)
        else:
            if not self._check_qpos_dimension("full_body", qpos):
                return False, None
            
            return self.kine.get_fk(name, qpos)

    def set_tcp(self, name: str, xpos:np.ndarray) -> bool:
        """设置left_arm/right_arm的tcp

        Args:
            name (str): ["left_arm", "right_arm"]
            xpos (np.ndarray): 目标tcp

        Returns:
            bool: 
        """
        # TODO: set tcp in robot client too
        return self.kine.set_tcp(name, xpos)

    def get_tcp(self, name: str) -> tuple[bool, np.ndarray]:
        """获取left_arm/right_arm的tcp

        Args:
            name (str): ["left_arm", "right_arm"]
        """
        return self.kine.get_tcp(name)

    def go_home(self, speed_ratio: float = 1.0) -> bool:
        """从当前点MoveJ移动到home qpos

        Args:
            speed_ratio (float, optional): 0.1~1.0. Defaults to 1.0.

        Returns:
            bool: 
        """
        target_qpos = deepcopy(self.w1_params.home_qpos)
        target_torso_qpos = target_qpos[self.w1_params.torso_index]
        target_head_qpos = target_qpos[self.w1_params.head_index]
        target_left_arm_qpos = target_qpos[self.w1_params.left_arm_index]
        target_right_arm_qpos = target_qpos[self.w1_params.right_arm_index]

        waypoint_dict = {"torso": [target_torso_qpos.tolist()],
                         "head": [target_head_qpos.tolist()],
                         "left_arm": [target_left_arm_qpos.tolist()],
                         "right_arm": [target_right_arm_qpos.tolist()]}

        return self.move_joint_full_body(waypoint_dict, max(0.1, min(speed_ratio, 1.0)))

    def stop_move(self) -> bool:
        """停止发送streaming指令

        Returns:
            bool: 
        """
        self._stop_move_event.set()
        self.logger.info("Stop move by user!")
        return True

    def check_collision(self, qpos: np.ndarray) -> bool:
        """检查qpos状态是否碰撞"""
        if not self._check_qpos_dimension("full_body", qpos):
            return True
        torso_tf, head_tf, arml_tf, armr_tf = self._pin_kine.get_Kinematics(qpos)
        self.fcl_checker.set_head_body_transform(head_transforms=head_tf, 
                                                        body_transforms=torso_tf)
        self.fcl_checker.update_arm_transform(arml_tf + armr_tf)
        return self.fcl_checker.check_collision()

    def _check_traj_collision(self, 
                              qpos_array: np.ndarray, 
                              step: int = 5) -> bool:
        """检查轨迹是否碰撞

        Args:
            qpos_array (np.ndarray): 10ms间隔的轨迹
            step (int, optional): 检测步长，每步10ms. Defaults to 5.

        Returns:
            bool: 
        """
        # NOTE: 最大速度 伸直构型下 仅基座关节运动导致的末端位移 5 * 0.01s * 1rad/s * 0.7m = 3.5cm
        qpos_array_check = np.vstack((qpos_array[::step], qpos_array[-1]))
        for qpos in qpos_array_check:
            if self.check_collision(qpos):
                self.logger.error(f"Traj is collision at {qpos}, do not executed!")
                return True
        return False
    
    def plan_joint_traj(self, 
                        name: str,
                        qpos_array: np.ndarray, 
                        qvel_limits_array: np.ndarray = None) -> tuple[bool, np.ndarray]:
        """对各部件进行关节轨迹规划，可指定路径点的各关节的速度约束;速度表示从上一点到当前点的最大关节速度

        Args:
            name (str): ["full_body", "torso", "head", "left_arm", "right_arm"]
            qpos_array (np.ndarray): 关节路径点  
            qvel_limits_array (np.ndarray, optional): 路径点关节速度约束，省略则按默认关节速度

        Returns:
            tuple[bool, np.ndarray]
        """
        if not self._check_name(name):
            return False, None
        if not self._check_qpos_limits(name, qpos_array):
            return False, None

        dofs = self.w1_params.index_infos[name]["dofs"]
        index = self.w1_params.index_infos[name]["index"]
        vel_limits = self.w1_params.max_qvel[index]
        acc_limits = self.w1_params.max_qacc[index]
        traj_planner = ToppraTrajectoryPlanner(dofs, vel_limits, acc_limits)
        res, pos_array, _, _, _ = traj_planner.plan(qpos_array, 
                                                    qvel_limits_array,
                                                    self.TIME_STEP)
        if not res:
            return False, None
        return True, pos_array

    def plan_cartesian_traj(self, 
                            name: str, 
                            xpos_list: list[np.ndarray], 
                            qpos_seed: np.ndarray,
                            qvel_limits_array: np.ndarray = None,
                            cartesian_step: float = 0.005) -> tuple[bool, np.ndarray]:
        """对机械臂进行笛卡尔轨迹规划，可指定路径点的各关节的速度约束；速度表示从上一点到当前点的最大关节速度

        Args:
            name (str): ["left_arm", "right_arm"]
            xpos_list (list[np.ndarray]): 笛卡尔路径点
            qpos_seed (np.ndarray): 起始参考关节seed
            qvel_limits_array (np.ndarray, optional): 路径点关节速度约束，省略则按默认关节速度
            cartesian_step (float, optional): 笛卡尔轨迹采样步长 默认0.005m

        Returns:
            tuple[bool, np.ndarray]
        """
        arm_kine = self._get_arm_kine(name)
        if arm_kine is None:
            return False, None

        # cartesian interpolation
        prefix_dist_list = [0.]
        for idx in range(len(xpos_list) - 1):
            dist = np.linalg.norm(xpos_list[idx+1][:3,3] - xpos_list[idx][:3,3])
            prefix_dist_list.append(prefix_dist_list[-1] + dist)
        total_dist = prefix_dist_list[-1]
        num_sample = max(2, int(np.ceil(total_dist / cartesian_step)))

        res, xpos_sample_array = interpolate_cartesian_path(xpos_list, 
                                                            0.005, 
                                                            num_sample)
        if not res:
            return False, None

        # vel align
        # TODO: use ee vel rather than qvel
        if qvel_limits_array is None:
            qvel_limits_array_ = None
        else:
            idx = 0
            qvel_limits_list_ = [qvel_limits_array[idx]]
            cur_xpos = xpos_sample_array[0]
            cur_dist = 0.
            for xpos in xpos_sample_array[1:-1]:
                cur_dist += np.linalg.norm(xpos[:3,3] - cur_xpos[:3,3])
                if cur_dist >= prefix_dist_list[idx]:
                    idx += 1
                qvel_limits_list_.append(qvel_limits_array[idx])
                cur_xpos = xpos
            qvel_limits_list_.append(qvel_limits_array[-1])
        qvel_limits_array_ = np.array(qvel_limits_list_)

        # solve ik
        qpos_list = []
        cur_qseed = qpos_seed
        for cur_xpos in xpos_sample_array:
            res_ik, cur_qpos = arm_kine.get_nearst_ik(xpos=cur_xpos, qpos_seed=cur_qseed)
            if not res_ik:
                self.logger.error(f"{name} solve ik failed, cur_qseed(deg): {np.rad2deg(cur_qseed)}.")
                return False, None
            if not np.allclose(cur_qpos, cur_qseed, atol=np.deg2rad(90)):
                self.logger.error(f"{name} qpos traj is not continues\n" + \
                                  f"cur_qpos(deg): {np.rad2deg(cur_qpos)}\n" + \
                                  f"cur_qseed(deg): {np.rad2deg(cur_qseed)}.")
                return False, None
            qpos_list.append(cur_qpos)
            cur_qseed = cur_qpos

        return self.plan_joint_traj(name, np.array(qpos_list), qvel_limits_array_)

    def plan_arm_ee_trigger(self, arm_ee_state_list: list) -> tuple[bool, list]:
        # TODO: 存在延时的才分段，否则不分段且全部waypoint ee_qpos都到点触发？
        if not self._check_arm_ee_states(arm_ee_state_list):
            return False, None
        ee_qpos_array = np.array([ee_qpos for _, ee_qpos, _ in arm_ee_state_list])
        if not self._check_qpos_limits("left_arm_ee", ee_qpos_array):
            return False, None

        segment_list = []
        start_idx = 0
        for i in range(1, len(arm_ee_state_list)):
            pre_qpos = arm_ee_state_list[i-1][1]
            pre_delay, qpos, post_delay = arm_ee_state_list[i]
            if pre_delay != 0. or post_delay != 0. or not np.allclose(qpos, pre_qpos, 
                                                                      atol=np.deg2rad(0.5)):
                segment_list.append([start_idx, i])
                start_idx = i

        if start_idx < len(arm_ee_state_list) - 1:
            segment_list.append([start_idx, len(arm_ee_state_list)-1])
        self.logger.debug(f"plan_arm_ee_trigger result: {segment_list}.")
        return True, segment_list

    def execute_joint_traj(self, 
                            qpos_array: np.ndarray,
                            left_arm_ee_trigger_list: list = None,
                            right_arm_ee_trigger_list: list = None,
                            reach_timeout: float = 3.0) -> bool:
        """按10ms间隔执行关节轨迹

        Args:
            qpos_array (np.ndarray): 关节轨迹
            left_arm_ee_trigger_list (list, optional): 左末端执行器触发序列. Defaults to None.
            right_arm_ee_trigger_list (list, optional): 右末端执行器触发序列. Defaults to None.
            reach_timeout (float, optional): 发送完指令后的到位检测timeout. Defaults to 3.0.

        Returns:
            bool: 
        """
        # check enable
        if self.robot_client.status in (Status.Init, Status.Running, Status.Error):
            self.logger.error(f"Robot client not in idle before moving. (Current status: {self.robot_client.status})")
            return False

        # reset stop move flag
        if self._stop_move_event.is_set():
            self._stop_move_event.clear()

        # check current qpos and start qpos
        res, cur_qpos = self.get_current_qpos()
        if not res: 
            return False
        if not np.allclose(cur_qpos, qpos_array[0], atol=np.deg2rad(1)):
            self.logger.error(f"cur_qpos ({cur_qpos}) is not equal to start_qpos ({qpos_array[0]}).")
            return False

        count = 1
        stop_ee_thread = False
        ee_thread_error = False

        # ee trigger
        def left_arm_ee_trigger():
            nonlocal count, stop_ee_thread, ee_thread_error
            left_arm_ee_trigger_list_cp = left_arm_ee_trigger_list.copy()
            left_arm_ee_trigger_list_cp.reverse()
            target_count, target_ee_qpos = left_arm_ee_trigger_list_cp.pop()
            cur_count = -1

            while not stop_ee_thread:
                if count == target_count and count != cur_count:
                    self.left_ee_client.set_qpos(np.array(target_ee_qpos))
                    self.logger.debug(f"left_arm_ee has triggered: {target_count} {target_ee_qpos}.")
                    if left_arm_ee_trigger_list_cp:
                        target_count, target_ee_qpos = left_arm_ee_trigger_list_cp.pop()
                    else:
                        self.logger.debug(f"left_arm_ee completes trigger.")
                        break
                    cur_count = count
                time.sleep(self.TIME_STEP / 10)

            if len(left_arm_ee_trigger_list_cp):
                self.logger.error(f"left_arm_ee has {len(left_arm_ee_trigger_list_cp)} states haven't been triggered.")
                self.logger.debug(f"current_count: {cur_count}, count: {count}, target_count: {target_count}")
                ee_thread_error = True

        def right_arm_ee_trigger():
            nonlocal count, stop_ee_thread, ee_thread_error
            right_arm_ee_trigger_list_cp = right_arm_ee_trigger_list.copy()
            right_arm_ee_trigger_list_cp.reverse()
            target_count, target_ee_qpos = right_arm_ee_trigger_list_cp.pop()
            cur_count = -1

            while not stop_ee_thread:
                if count == target_count and count != cur_count:
                    self.right_ee_client.set_qpos(np.array(target_ee_qpos))
                    self.logger.debug(f"right_arm_ee has triggered: {target_count} {target_ee_qpos}.")
                    if right_arm_ee_trigger_list_cp:
                        target_count, target_ee_qpos = right_arm_ee_trigger_list_cp.pop()
                    else:
                        self.logger.debug(f"right_arm_ee completes trigger.")
                        break
                    cur_count = count
                time.sleep(self.TIME_STEP / 10)

            if len(right_arm_ee_trigger_list_cp):
                self.logger.error(f"left_arm_ee has {len(right_arm_ee_trigger_list_cp)} states haven't been triggered.")
                self.logger.debug(f"current_count: {cur_count}, count: {count}, target_count: {target_count}")
                ee_thread_error = True

        left_arm_ee_trigger_thread = None
        if self._has_arm_ee_client("left_arm_ee", False) and left_arm_ee_trigger_list is not None:
            left_arm_ee_trigger_thread = threading.Thread(
                target=left_arm_ee_trigger, daemon=True
            )
            left_arm_ee_trigger_thread.start()

        right_arm_ee_trigger_thread = None
        if self._has_arm_ee_client("right_arm_ee", False) and right_arm_ee_trigger_list is not None:
            right_arm_ee_trigger_thread = threading.Thread(
                target=right_arm_ee_trigger, daemon=True
            )
            right_arm_ee_trigger_thread.start()

        # streaming
        self.logger.info("Start streaming.")
        end_iter = qpos_array.shape[0] - 1
        last_qpos = cur_qpos
        self.robot_client.start_joint_streaming_request(MotionGroup.WholeBody)
        rate = RateLimiter(frequency=1./self.TIME_STEP, warn=True)
        t0 = time.time()
        for qpos in qpos_array:
            if self._stop_move_event.is_set():
                self.robot_client.send_streaming_command(position = last_qpos.tolist(), 
                                                         streaming_finished = True)
                self.logger.info(f"Stop sending streaming command")
                stop_ee_thread = True
                if left_arm_ee_trigger_thread is not None:
                    left_arm_ee_trigger_thread.join()
                if right_arm_ee_trigger_thread is not None:
                    right_arm_ee_trigger_thread.join()
                self._stop_move_event.clear()
                return False

            if count == end_iter + 1:
                streaming_finished = True
            else:
                streaming_finished = False

            if not self.robot_client.send_streaming_command(qpos.tolist(), 
                                                            streaming_finished):
                self.logger.critical("Streaming command send failed.")
                stop_ee_thread = True
                if left_arm_ee_trigger_thread is not None:
                    left_arm_ee_trigger_thread.join()
                if right_arm_ee_trigger_thread is not None:
                    right_arm_ee_trigger_thread.join()
                return False
            
            count += 1
            last_qpos = qpos
            rate.sleep()

        self.robot_client.wait_request_finish(timeout_sec=reach_timeout)
        
        stop_ee_thread = True
        if left_arm_ee_trigger_thread is not None:
            left_arm_ee_trigger_thread.join()
        if right_arm_ee_trigger_thread is not None:
            right_arm_ee_trigger_thread.join()

        self.logger.info("Finish streaming. " +
                        f"Ideal duration should be {(count-1) * self.TIME_STEP}s, " +
                        f"and actual duration is {time.time() - t0}s.")

        # check reach qpos
        targt_qpos = qpos_array[-1]
        res, cur_qpos = self.get_current_qpos()
        if not res: 
            return False
        if not np.allclose(cur_qpos, targt_qpos, atol=np.deg2rad(5)):
            self.logger.error(f"Current qpos does NOT reach the target qpos until {reach_timeout}s, " +
                    f"qpos error(deg): {np.rad2deg(cur_qpos - targt_qpos)}.")
            return False
        if not np.allclose(cur_qpos, targt_qpos, atol=np.deg2rad(0.5)):
            self.logger.warning(f"Current qpos does NOT close to the target qpos until {reach_timeout}s, " +
                            f"qpos error(deg): {np.rad2deg(cur_qpos - targt_qpos)}.")
        return (True and (not ee_thread_error))


    ########## demo定制API ###########
    def get_current_pose(self, name: str, is_local: bool = True) -> tuple[bool, list]:
        """is_local=True，获取left_arm/right_arm当前相对于arm base坐标系的笛卡尔位姿[xyz_mm, rpy_deg]

            is_local=False，获取left_arm/right_arm当前相对于chassis坐标系的笛卡尔位姿[xyz_mm, rpy_deg]

        Args:
            name (str): ["left_arm", "right_arm"]
            is_local (bool, optional): w.r.t. arm base frame or chassis frame. Defaults to True.

        Returns:
            tuple[bool, list]: 
        """
        res, cur_xpos = self.get_current_xpos(name, is_local)
        if not res: 
            return False, None
        return True, xpos_to_user_pose(cur_xpos)

    def get_pose(self, name: str, qpos: np.ndarray, is_local: bool = True) -> tuple[bool, list]:
        """is_local=True，获取left_arm/right_arm相对于arm base坐标系的笛卡尔位姿[xyz_mm, rpy_deg]，size(qpos)=7

            is_local=False，获取left_arm/right_arm相对于chassis坐标系的笛卡尔位姿[xyz_mm, rpy_deg]，size(qpos)=20

        Args:
            name (str): ["left_arm", "right_arm"]
            qpos (np.ndarray): 
            is_local (bool, optional): w.r.t. arm base frame or chassis frame. Defaults to True.

        Returns:
            tuple[bool, list]: 
        """
        res, cur_xpos = self.get_xpos(name, qpos, is_local)
        if not res: 
            return False, None
        return True, xpos_to_user_pose(cur_xpos)

    def transfer_grasp_pose(self, 
                            name: str,
                            pre_grasp_pose: list, 
                            pre_torso_qpos: np.ndarray, 
                            target_torso_qpos: np.ndarray) -> tuple[bool, list]:
        """保持抓取点在全身坐标系的位置，获取torso运动后的相对于机械臂基座坐标系的抓取pose

        Args:
            name (str): left_arm / right_arm
            pre_grasp_pose (list): 抓取前相对于机械臂基座的pose
            pre_torso_qpos (np.ndarray): torso运动前的qpos
            target_torso_qpos (np.ndarray): torso运动后的qpos

        Returns:
            tuple[bool, list]
        """
        pre_torso_qpos = np.array(pre_torso_qpos)
        target_torso_qpos = np.array(target_torso_qpos)
        
        if not self._check_arm_name(name):
            return False, None
        if not self._check_qpos_dimension("torso", pre_torso_qpos) or \
            not self._check_qpos_dimension("torso", target_torso_qpos):
            return False, None

        T_armbase_ee_pre = user_pose_to_xpos(pre_grasp_pose[:3], pre_grasp_pose[3:])
        
        qpos_pre = np.zeros(self.w1_params.full_body_dofs)
        qpos_pre[:4] = pre_torso_qpos
        res_fk, T_chassis_armbase_pre = self.kine.get_fk(name + "_base", qpos_pre)
        if not res_fk:
            self.logger.error(f"{name} solve fk failed!")
            return False, None

        qpos_target = np.zeros(self.w1_params.full_body_dofs)
        qpos_target[:4] = target_torso_qpos
        res_fk, T_chasis_armbase_target = self.kine.get_fk(name + "_base", qpos_target)
        if not res_fk:
            self.logger.error(f"{name} solve fk failed!")
            return False, None

        T_armbase_ee_target = np.linalg.inv(T_chasis_armbase_target) @ T_chassis_armbase_pre @ T_armbase_ee_pre
        return True, xpos_to_user_pose(T_armbase_ee_target)

    def _parse_waypoints_from_dict(self, 
                                   name: str, 
                                   waypoints_dict: dict, 
                                   speed_ratio:float,
                                   is_cartesian: bool = False) -> tuple[int, np.ndarray, np.ndarray, list]:
        """解析并检查输入  
            int: -1 错误输入； 0 空输入； 1 有效输入"""
        pos_list = waypoints_dict.get(name)
        if not pos_list: # None or []
            self.logger.debug(f"No waypoints in {name}.")
            return 0, None, None, None
        
        if not all(isinstance(pos, list) for pos in pos_list):
            self.logger.error(f"{pos_list} not a 2d-list.")
            return -1, None, None, None
        if not is_cartesian and not self._check_qpos_dimension(name, pos_list):
            return -1, None, None, None
        if is_cartesian and not all(len(pos) == 6 for pos in pos_list):
            self.logger.error(f"Dimension of {pos_list[0]} != 6.")
            return -1, None, None, None

        speed_ratio_list = waypoints_dict.get(f"{name}_speed_ratio")
        if not speed_ratio_list: # None or []
            speed_ratio_list = [speed_ratio] * len(pos_list)
        elif len(speed_ratio_list) != len(pos_list):
            self.logger.error(f"{name} qpos and speed_ratio dimension mismatch.")
            return -1, None, None, None
        # 将None元素替换为global_speed_ratio
        speed_ratio_list = [sr if sr is not None else speed_ratio
                            for sr in speed_ratio_list]

        default_qvel = self.w1_params.default_qvel[self.w1_params.index_infos[name]["index"]]
        min_qvel = self.w1_params.min_qvel[self.w1_params.index_infos[name]["index"]]
        max_qvel = self.w1_params.max_qvel[self.w1_params.index_infos[name]["index"]]
        qvel_limits_array = np.clip(np.outer(speed_ratio_list, default_qvel), 
                                    min_qvel, max_qvel)

        # 额外读取ee的输入
        arm_ee_state_list = None
        if name in self.w1_params.arm_names:
            arm_ee_state_list = waypoints_dict.get(f"{name}_ee")
            if not arm_ee_state_list: # None or []
                arm_ee_state_list = None
            elif len(arm_ee_state_list) != len(pos_list):
                self.logger.error(f"{name} qpos and arm_ee_state dimension mismatch.")
                return -1, None, None, None
            elif not self._check_arm_ee_states(arm_ee_state_list):
                return -1, None, None, None
                
        return 1, np.array(pos_list), qvel_limits_array, arm_ee_state_list

    def _plan_to_waypoint(self, 
                          name: str,
                          cur_qpos: np.ndarray, 
                          waypoints_dict: dict, 
                          speed_ratio: float,
                          is_cartesian: bool = False,
                          qpos_seed: np.ndarray = None,
                          cartesian_step: float = None,
                          is_local: bool = True) -> tuple[bool, np.ndarray]:
        if not is_local:
            qpos_array = cur_qpos[self.w1_params.index_infos[name]["index"]]
        else:
            qpos_array = cur_qpos
        (int_res, 
        pos_raw_array, 
        qvel_limits_array, _) =  self._parse_waypoints_from_dict(name, 
                                                            waypoints_dict, 
                                                            speed_ratio,
                                                            is_cartesian)
        if int_res == -1: # 输入不合法
            return False, None
        elif int_res == 0: # 输入为空
            return True, qpos_array
        # 输入合法
        qvel_limits_array_ = np.vstack([self.w1_params.default_qvel[self.w1_params.index_infos[name]["index"]],
                                        qvel_limits_array])
        if is_cartesian:
            if not is_local:
                # Transfer global frame to local frame
                ref_torso_qpos = cur_qpos[self.w1_params.index_infos["torso"]["index"]]
                ref_qpos = np.zeros(self.w1_params.full_body_dofs)
                ref_qpos[self.w1_params.index_infos["torso"]["index"]] = ref_torso_qpos
                res_fk, T_chassis_armbase = self.get_xpos(f"{name}_base", ref_qpos, is_local=False)
                if not res_fk:
                    self.logger.error(f"{name}_base solve fk failed!")
                    return False, None
                temp = [np.linalg.inv(T_chassis_armbase) @ user_pose_to_xpos(pose[:3].tolist(), pose[3:].tolist()) for pose in pos_raw_array]
                pos_raw_array = [np.array(xpos_to_user_pose(xpos)) for xpos in temp]
                cur_arm_qpos = cur_qpos[self.w1_params.index_infos[name]["index"]]
            else:
                cur_arm_qpos = cur_qpos

            # In local frame
            res_fk, cur_xpos = self._get_arm_kine(name).get_fk(cur_arm_qpos)
            if not res_fk:
                self.logger.error(f"{name} solve fk failed!")
                return False, None
            xpos_raw_list = [user_pose_to_xpos(pose[:3].tolist(), pose[3:].tolist()) 
                              for pose in pos_raw_array]
            xpos_raw_list_ = [cur_xpos] + xpos_raw_list
            res_plan, qpos_array = self.plan_cartesian_traj(name, xpos_raw_list_, qpos_seed, 
                                                            qvel_limits_array_, cartesian_step)
            if not res_plan:
                return False, None
        else:
            qpos_raw_array_ = np.vstack([cur_qpos, pos_raw_array])
            res_plan, qpos_array = self.plan_joint_traj(name,
                                                        qpos_raw_array_, 
                                                        qvel_limits_array_)
            if not res_plan:
                return False, None
            
        return True, qpos_array

    def _plan_to_waypoint_with_ee(self,
                                  name: str,
                                  cur_qpos: np.ndarray, 
                                  cur_ee_qpos: np.ndarray, 
                                  waypoints_dict: dict, 
                                  speed_ratio: float,
                                  is_cartesian: bool = False,
                                  qpos_seed: np.ndarray = None,
                                  cartesian_step: float = None,
                                  is_local: bool = True) -> tuple[bool, np.ndarray, list]:
        if not is_local:
            qpos_array = cur_qpos[self.w1_params.index_infos[name]["index"]]
        else:
            qpos_array = cur_qpos

        (int_res, 
        pos_raw_array, 
        qvel_limits_array, 
        arm_ee_state_list) =  self._parse_waypoints_from_dict(name, 
                                                            waypoints_dict, 
                                                            speed_ratio,
                                                            is_cartesian)
        if int_res == -1:
            return False, None, None
        elif int_res == 0:
            return True, qpos_array, None

        # without ee motion
        if arm_ee_state_list is None:
            res, qpos_array = self._plan_to_waypoint(name, 
                                                    cur_qpos, 
                                                    waypoints_dict,
                                                    speed_ratio, 
                                                    is_cartesian, 
                                                    qpos_seed, 
                                                    cartesian_step,
                                                    is_local)
            if not res:
                return False, None, None
            return True, qpos_array, None

        # with ee motion
        if not is_cartesian:
            pos_raw_array_ = np.vstack([cur_qpos, pos_raw_array])
        else:
            if not is_local:
                # Transfer global frame to local frame
                ref_torso_qpos = cur_qpos[self.w1_params.index_infos["torso"]["index"]]
                ref_qpos = np.zeros(self.w1_params.full_body_dofs)
                ref_qpos[self.w1_params.index_infos["torso"]["index"]] = ref_torso_qpos
                res_fk, T_chassis_armbase = self.get_xpos(f"{name}_base", ref_qpos, is_local=False)
                if not res_fk:
                    self.logger.error(f"{name}_base solve fk failed!")
                    return False, None, None
                temp = [np.linalg.inv(T_chassis_armbase) @ user_pose_to_xpos(pose[:3].tolist(), pose[3:].tolist()) for pose in pos_raw_array]
                pos_raw_array = [np.array(xpos_to_user_pose(xpos)) for xpos in temp]
                cur_arm_qpos = cur_qpos[self.w1_params.index_infos[name]["index"]]
            else:
                cur_arm_qpos = cur_qpos

            res_fk, cur_xpos = self._get_arm_kine(name).get_fk(cur_arm_qpos)
            if not res_fk:
                self.logger.error(f"{name} solve fk failed!")
                return False, None, None
            pos_raw_array_ = np.vstack([xpos_to_user_pose(cur_xpos), pos_raw_array])

        qvel_limits_array_ = np.vstack([self.w1_params.default_qvel[self.w1_params.index_infos[name]["index"]],
                                        qvel_limits_array])
        cur_ee_state = [0., cur_ee_qpos.tolist(), 0.]
        arm_ee_state_list_ = [cur_ee_state] + [ee_state if ee_state is not None else cur_ee_state 
                                               for ee_state in arm_ee_state_list]

        # segment raw waypoint
        arm_ee_trigger_list = [] # (count, ee_qpos)
        res, segment_list = self.plan_arm_ee_trigger(arm_ee_state_list_)
        if not res:
            return False, None, None
        
        total_count = 0
        total_qpos_array = None
        if is_cartesian:
            cur_qpos_seed = qpos_seed

        for start_idx, end_idx in segment_list:
            seg_pos_list = pos_raw_array_[start_idx:end_idx+1]
            seg_qvel_limits_list = qvel_limits_array_[start_idx:end_idx+1]
            if is_cartesian:
                seg_xpos_list = [user_pose_to_xpos(pose[:3].tolist(), pose[3:].tolist()) 
                                for pose in seg_pos_list]
                res_plan, seg_qpos_samples_array = self.plan_cartesian_traj(name, 
                                                                            seg_xpos_list, 
                                                                            cur_qpos_seed, 
                                                                            seg_qvel_limits_list, 
                                                                            cartesian_step)
            else:
                res_plan, seg_qpos_samples_array = self.plan_joint_traj(name,
                                                                        seg_pos_list, 
                                                                        seg_qvel_limits_list)
            if not res_plan:
                return False, None, None
            
            # pre delay
            pre_delay_time = arm_ee_state_list_[end_idx][0]
            if pre_delay_time >= self.TIME_STEP:
                pre_delay_count = max(1, int(np.ceil(pre_delay_time / self.TIME_STEP)))
                seg_qpos_samples_array = np.vstack([seg_qpos_samples_array, 
                                                    np.repeat(seg_qpos_samples_array[-1:], 
                                                            pre_delay_count, axis=0)])
            # ee trigger
            arm_ee_trigger_list.append((total_count + seg_qpos_samples_array.shape[0] - 1, 
                                        arm_ee_state_list_[end_idx][1]))
            # post delay
            post_delay_time = arm_ee_state_list_[end_idx][2]
            if post_delay_time >= self.TIME_STEP:
                post_delay_count = max(1, int(np.ceil(post_delay_time / self.TIME_STEP)))
                seg_qpos_samples_array = np.vstack([seg_qpos_samples_array, 
                                                    np.repeat(seg_qpos_samples_array[-1:], 
                                                            post_delay_count, axis=0)])

            if total_qpos_array is None:
                total_qpos_array = seg_qpos_samples_array
            else:
                total_qpos_array = np.vstack(([total_qpos_array, seg_qpos_samples_array[1:]]))

            if is_cartesian:
                cur_qpos_seed = total_qpos_array[-1]

            total_count = total_qpos_array.shape[0]
        
        return True, total_qpos_array, arm_ee_trigger_list

    @time_consuming
    def plan_joint_full_body(self,
                             start_qpos: np.ndarray,
                             start_left_arm_ee_qpos: np.ndarray, 
                             start_right_arm_ee_qpos: np.ndarray,
                             waypoints_dict: dict, 
                             speed_ratio: float = 1.0) -> tuple[bool, np.ndarray, list, list]:
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

            speed_ratio (float, optional): 相对于默认关节速度的比率. Defaults to 1.0.

        Returns:
            tuple[bool, np.ndarray, list, list]: 规划成功与否，全身关节轨迹，左末端执行器轨迹，右末端执行器轨迹
        """
        if not self._check_qpos_limits("full_body", start_qpos):
            return False, None, None, None

        start_torso_qpos = start_qpos[self.w1_params.torso_index]
        start_head_qpos = start_qpos[self.w1_params.head_index]
        start_left_arm_qpos = start_qpos[self.w1_params.left_arm_index]
        start_right_arm_qpos = start_qpos[self.w1_params.right_arm_index]
        
        self.logger.debug(f"plan_joint_full_body() input cur_qpos: {start_qpos}")
        self.logger.debug(f"plan_joint_full_body() input cur_ee_qpos: {start_left_arm_ee_qpos}, {start_right_arm_ee_qpos}")
        self.logger.debug("plan_joint_full_body() input waypoints_dict:")
        self.logger.debug(waypoints_dict)
        self.logger.debug(f"plan_joint_full_body() input speed_ratio: {speed_ratio}")

        # torso
        res_torso, torso_qpos_array = self._plan_to_waypoint("torso", 
                                                             start_torso_qpos, 
                                                             waypoints_dict, 
                                                             speed_ratio)
        if not res_torso:
            self.logger.error("Plan to torso traj failed.")
            return False, None, None, None
        
        # head
        res_head, head_qpos_array = self._plan_to_waypoint("head", 
                                                             start_head_qpos, 
                                                             waypoints_dict, 
                                                             speed_ratio)
        if not res_head:
            self.logger.error("Plan to head traj failed.")
            return False, None, None, None

        # left_arm
        left_arm_ee_trigger_list = None
        if not self._has_arm_ee_client("left_arm_ee", False):
            (res_left_arm, 
            left_arm_qpos_array) = self._plan_to_waypoint("left_arm", 
                                                        start_left_arm_qpos, 
                                                        waypoints_dict, 
                                                        speed_ratio)
            if not res_left_arm:
                self.logger.error("Plan to left arm traj failed.")
                return False, None, None, None
        else:
            if not self._check_qpos_limits("left_arm_ee", start_left_arm_ee_qpos):
                return False, None, None, None
            (res_left_arm, 
            left_arm_qpos_array,
            left_arm_ee_trigger_list) = self._plan_to_waypoint_with_ee("left_arm", 
                                                                       start_left_arm_qpos, 
                                                                       start_left_arm_ee_qpos, 
                                                                       waypoints_dict, 
                                                                       speed_ratio)
            if not res_left_arm:
                self.logger.error("Plan to left arm traj with ee failed.")
                return False, None, None, None

        # right_arm
        right_arm_ee_trigger_list = None
        if not self._has_arm_ee_client("right_arm_ee", False):
            (res_right_arm, 
            right_arm_qpos_array) = self._plan_to_waypoint("right_arm", 
                                                        start_right_arm_qpos, 
                                                        waypoints_dict, 
                                                        speed_ratio)
            if not res_right_arm:
                self.logger.error("Plan to right arm traj failed.")
                return False, None, None, None
        else:
            if not self._check_qpos_limits("right_arm_ee", start_right_arm_ee_qpos):
                return False, None, None, None
            (res_right_arm, 
            right_arm_qpos_array,
            right_arm_ee_trigger_list) = self._plan_to_waypoint_with_ee("right_arm", 
                                                                       start_right_arm_qpos, 
                                                                       start_right_arm_ee_qpos, 
                                                                       waypoints_dict, 
                                                                       speed_ratio)
            if not res_right_arm:
                self.logger.error("Plan to right arm traj with ee failed.")
                return False, None, None, None

        # align
        res_align, total_qpos_array = self._qpos_align(torso_qpos_array, 
                                                       head_qpos_array, 
                                                       left_arm_qpos_array, 
                                                       right_arm_qpos_array)
        if not res_align:
            return False, None, None, None

        return True, total_qpos_array, left_arm_ee_trigger_list, right_arm_ee_trigger_list

    def move_joint_full_body(self, 
                                waypoints_dict: dict, 
                                speed_ratio: float = 1.0) -> bool:
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

            speed_ratio (float, optional): 相对于默认关节速度的比率. Defaults to 1.0.

        Returns:
            bool
        """
        res, cur_qpos = self.get_current_qpos()
        if not res:
            return False
        cur_left_arm_ee_qpos = None
        if self._has_arm_ee_client("left_arm_ee", False):
            res, cur_left_arm_ee_qpos = self.get_current_qpos("left_arm_ee")
            if not res:
                return False
        cur_right_arm_ee_qpos = None
        if self._has_arm_ee_client("right_arm_ee", False):
            res, cur_right_arm_ee_qpos = self.get_current_qpos("right_arm_ee")
            if not res:
                return False

        # plan traj
        (res, 
         total_qpos_array, 
         left_arm_ee_trigger_list, 
         right_arm_ee_trigger_list) = self.plan_joint_full_body(cur_qpos, 
                                                                cur_left_arm_ee_qpos, 
                                                                cur_right_arm_ee_qpos, 
                                                                waypoints_dict,
                                                                speed_ratio)
        if not res:
            return False
        
        # # check collision
        # if self._check_traj_collision(total_qpos_array):
        #     return False

        # execute
        return self.execute_joint_traj(total_qpos_array, 
                                       left_arm_ee_trigger_list, 
                                       right_arm_ee_trigger_list)

    @time_consuming
    def plan_linear_full_body(self,
                             start_qpos: np.ndarray,
                             start_left_arm_ee_qpos: np.ndarray, 
                             start_right_arm_ee_qpos: np.ndarray,
                             waypoints_dict: dict, 
                             left_qpos_seed: np.ndarray = None,
                             right_qpos_seed: np.ndarray = None,
                             cartesian_step: float = 0.005,
                             speed_ratio: float = 1.0,
                             is_local: bool = True) -> tuple[bool, np.ndarray, list, list]:
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
            tuple[bool, np.ndarray, list, list]: 规划成功与否，全身关节轨迹，左末端执行器轨迹，右末端执行器轨迹
        """
        if not self._check_qpos_limits("full_body", start_qpos):
            return False, None, None, None

        start_torso_qpos = start_qpos[self.w1_params.torso_index]
        start_head_qpos = start_qpos[self.w1_params.head_index]
        start_left_arm_qpos = start_qpos[self.w1_params.left_arm_index]
        start_right_arm_qpos = start_qpos[self.w1_params.right_arm_index]
        
        if left_qpos_seed is None:
            left_qpos_seed = start_left_arm_qpos
        if right_qpos_seed is None:
            right_qpos_seed = start_right_arm_qpos

        self.logger.debug(f"plan_linear_full_body() input cur_qpos: {start_qpos}")
        self.logger.debug(f"plan_linear_full_body() input cur_ee_qpos: {start_left_arm_ee_qpos}, {start_right_arm_ee_qpos}")
        self.logger.debug("plan_linear_full_body() input waypoints_dict:")
        self.logger.debug(waypoints_dict)
        self.logger.debug(f"plan_linear_full_body() input speed_ratio: {speed_ratio}")

        # torso
        res_torso, torso_qpos_array = self._plan_to_waypoint("torso", 
                                                             start_torso_qpos, 
                                                             waypoints_dict, 
                                                             speed_ratio)
        if not res_torso:
            self.logger.error("Plan to torso traj failed.")
            return False, None, None, None
        
        # head
        res_head, head_qpos_array = self._plan_to_waypoint("head", 
                                                             start_head_qpos, 
                                                             waypoints_dict, 
                                                             speed_ratio)
        if not res_head:
            self.logger.error("Plan to head traj failed.")
            return False, None, None, None

        # left_arm
        left_arm_ee_trigger_list = None
        if not self._has_arm_ee_client("left_arm_ee", False):
            if is_local:
                (res_left_arm, 
                left_arm_qpos_array) = self._plan_to_waypoint("left_arm", 
                                                            start_left_arm_qpos, 
                                                            waypoints_dict, 
                                                            speed_ratio,
                                                            is_cartesian=True,
                                                            qpos_seed=left_qpos_seed,
                                                            cartesian_step=cartesian_step,
                                                            is_local=True)
            else:
                (res_left_arm, 
                left_arm_qpos_array) = self._plan_to_waypoint("left_arm", 
                                                            start_qpos, 
                                                            waypoints_dict, 
                                                            speed_ratio,
                                                            is_cartesian=True,
                                                            qpos_seed=left_qpos_seed,
                                                            cartesian_step=cartesian_step,
                                                            is_local=False)
            if not res_left_arm:
                self.logger.error("Plan to left arm traj failed.")
                return False, None, None, None
        else:
            if not self._check_qpos_limits("left_arm_ee", start_left_arm_ee_qpos):
                return False, None, None, None
            if is_local:
                (res_left_arm, 
                left_arm_qpos_array,
                left_arm_ee_trigger_list) = self._plan_to_waypoint_with_ee("left_arm", 
                                                                        start_left_arm_qpos, 
                                                                        start_left_arm_ee_qpos, 
                                                                        waypoints_dict, 
                                                                        speed_ratio,
                                                                        is_cartesian=True,
                                                                        qpos_seed=left_qpos_seed,
                                                                        cartesian_step=cartesian_step,
                                                                        is_local=True)
            else:
                (res_left_arm, 
                left_arm_qpos_array,
                left_arm_ee_trigger_list) = self._plan_to_waypoint_with_ee("left_arm", 
                                                                        start_qpos, 
                                                                        start_left_arm_ee_qpos, 
                                                                        waypoints_dict, 
                                                                        speed_ratio,
                                                                        is_cartesian=True,
                                                                        qpos_seed=left_qpos_seed,
                                                                        cartesian_step=cartesian_step,
                                                                        is_local=False)
            if not res_left_arm:
                self.logger.error("Plan to left arm traj with ee failed.")
                return False, None, None, None

        # right_arm
        right_arm_ee_trigger_list = None
        if not self._has_arm_ee_client("right_arm_ee", False):
            if is_local:
                (res_right_arm, 
                right_arm_qpos_array) = self._plan_to_waypoint("right_arm", 
                                                            start_right_arm_qpos, 
                                                            waypoints_dict, 
                                                            speed_ratio,
                                                            is_cartesian=True,
                                                            qpos_seed=right_qpos_seed,
                                                            cartesian_step=cartesian_step,
                                                            is_local=True)
            else:
                (res_right_arm, 
                right_arm_qpos_array) = self._plan_to_waypoint("right_arm", 
                                                            start_qpos, 
                                                            waypoints_dict, 
                                                            speed_ratio,
                                                            is_cartesian=True,
                                                            qpos_seed=right_qpos_seed,
                                                            cartesian_step=cartesian_step,
                                                            is_local=False)
            if not res_right_arm:
                self.logger.error("Plan to right arm traj failed.")
                return False, None, None, None
        else:
            if not self._check_qpos_limits("right_arm_ee", start_right_arm_ee_qpos):
                return False, None, None, None
            if is_local:
                (res_right_arm, 
                right_arm_qpos_array,
                right_arm_ee_trigger_list) = self._plan_to_waypoint_with_ee("right_arm", 
                                                                        start_right_arm_qpos, 
                                                                        start_right_arm_ee_qpos, 
                                                                        waypoints_dict, 
                                                                        speed_ratio,
                                                                        is_cartesian=True,
                                                                        qpos_seed=right_qpos_seed,
                                                                        cartesian_step=cartesian_step,
                                                                        is_local=True)
            else:
                (res_right_arm, 
                right_arm_qpos_array,
                right_arm_ee_trigger_list) = self._plan_to_waypoint_with_ee("right_arm", 
                                                                        start_qpos, 
                                                                        start_right_arm_ee_qpos, 
                                                                        waypoints_dict, 
                                                                        speed_ratio,
                                                                        is_cartesian=True,
                                                                        qpos_seed=right_qpos_seed,
                                                                        cartesian_step=cartesian_step,
                                                                        is_local=False)
            if not res_right_arm:
                self.logger.error("Plan to right arm traj with ee failed.")
                return False, None, None, None

        # align
        res_align, total_qpos_array = self._qpos_align(torso_qpos_array, 
                                                       head_qpos_array, 
                                                       left_arm_qpos_array, 
                                                       right_arm_qpos_array)
        if not res_align:
            return False, None, None, None

        return True, total_qpos_array, left_arm_ee_trigger_list, right_arm_ee_trigger_list

    def move_linear_full_body(self,
                                waypoints_dict: dict, 
                                left_qpos_seed: np.ndarray = None,
                                right_qpos_seed: np.ndarray = None,
                                cartesian_step: float = 0.005,
                                speed_ratio: float = 1.0,
                                is_local: bool = True) -> bool:
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
            bool
        """
        res, cur_qpos = self.get_current_qpos()
        if not res:
            return False
        cur_left_arm_ee_qpos = None
        if self._has_arm_ee_client("left_arm_ee", False):
            res, cur_left_arm_ee_qpos = self.get_current_qpos("left_arm_ee")
            if not res:
                return False
        cur_right_arm_ee_qpos = None
        if self._has_arm_ee_client("right_arm_ee", False):
            res, cur_right_arm_ee_qpos = self.get_current_qpos("right_arm_ee")
            if not res:
                return False

        # plan traj
        (res, 
         total_qpos_array, 
         left_arm_ee_trigger_list, 
         right_arm_ee_trigger_list) = self.plan_linear_full_body(cur_qpos, 
                                                                cur_left_arm_ee_qpos, 
                                                                cur_right_arm_ee_qpos, 
                                                                waypoints_dict,
                                                                left_qpos_seed,
                                                                right_qpos_seed,
                                                                cartesian_step,
                                                                speed_ratio,
                                                                is_local)
        if not res:
            return False

        # # check collision
        # if self._check_traj_collision(total_qpos_array):
        #     return False

        # execute
        return self.execute_joint_traj(total_qpos_array, 
                                       left_arm_ee_trigger_list, 
                                       right_arm_ee_trigger_list)

    def move_torso_linear(self, 
                          delta_x_mm: float, 
                          delta_z_mm: float, 
                          cartesian_step = 0.002,
                          speed_ratio: float = 1.) -> bool:
        """腿部从当前位置直线移动相对距离

        Args:
            delta_x_mm (float): x方向(前后)直线相对运动距离
            delta_z_mm (float): z方向(上下)直线相对运动距离
            cartesian_step (float, optional): 直线离散步长. Defaults to 0.002.
            speed_ratio (float, optional): 运动速率. Defaults to 1..

        Returns:
            bool: 
        """
        res, cur_qpos = self.get_current_qpos()
        if not res:
            return False
        res_fk, cur_x, cur_z, cur_y_theta = self.kine.get_torso_fk(cur_qpos[:3])
        if not res_fk:
            return False
        target_x = cur_x + (delta_x_mm / 1000.)
        target_z = cur_z + (delta_z_mm / 1000.)
        # TODO: check com in support polyhedron

        # Create path
        p_cur = np.array([cur_x, cur_z])
        p_target = np.array([target_x, target_z])
        dist = np.linalg.norm(p_target - p_cur)
        n_steps = int(np.ceil(dist / cartesian_step))
        s_list = np.linspace(0, 1, n_steps+1)
        p_list = [p_cur + (p_target - p_cur) * s for s in s_list]
        
        # Solve ik
        qpos_list = []
        for p in p_list:
            res_ik, qpos = self.kine.get_torso_ik(p[0], p[1], cur_y_theta, cur_qpos[1] < 0)
            if not res_ik:
                return False
            qpos_list.append(np.r_[qpos, cur_qpos[3]])

        return self.move_joint_full_body(waypoints_dict={f"torso": [qpos.tolist() for qpos in qpos_list]},
                                         speed_ratio=speed_ratio)

    def move_arm_linear(self, 
                        name: str,
                        delta_trans_mm: list, 
                        cartesian_step = 0.005,
                        speed_ratio: float = 1.,
                        along_base_frame: bool = True) -> bool:
        """机械臂从当前位置直线移动相对距离

        Args:
            name (str): ["left_arm", "right_arm"]
            delta_trans_mm (list): 直线相对运动距离
            cartesian_step (float, optional): 直线离散步长. Defaults to 0.005.
            speed_ratio (float, optional): 运动速率. Defaults to 1..
            along_base_frame (bool, optional): 参考chassis_base坐标系/tcp坐标系. Defaults to True.

        Returns:
            bool: 
        """
        if not self._check_arm_name(name):
            return False
        
        res, T_armbase_ee = self.get_current_xpos(name)
        if not res:
            return False
        
        delta_xpos = user_pose_to_xpos(delta_trans_mm, [0., 0., 0.])
        if along_base_frame:
            res, T_chassis_ee = self.get_current_xpos(name, False)
            if not res:
                return False
            target_xpos = T_armbase_ee @ np.linalg.inv(T_chassis_ee) @ (delta_xpos @ T_chassis_ee)
        else:
            target_xpos = T_armbase_ee @ delta_xpos

        return self.move_linear_full_body(waypoints_dict={f"{name}": [xpos_to_user_pose(target_xpos)]},
                                        cartesian_step=cartesian_step, 
                                        speed_ratio=speed_ratio)

    def rotate_arm_linear(self, 
                        name: str,
                        delta_rot_deg: list, 
                        speed_ratio: float = 1.,
                        along_base_frame: bool = True) -> bool:
        """机械臂从当前位置旋转相对角度

        Args:
            name (str): ["left_arm", "right_arm"]
            delta_rot_deg (list): 旋转相对运动距离
            speed_ratio (float, optional): 运动速率. Defaults to 1..
            along_base_frame (bool, optional): 参考chassis_base坐标系/tcp坐标系. Defaults to True.

        Returns:
            bool: 
        """
        if not self._check_arm_name(name):
            return False
        
        res, T_armbase_ee = self.get_current_xpos(name)
        if not res:
            return False
        
        delta_xpos = user_pose_to_xpos([0., 0., 0.], delta_rot_deg)
        target_xpos = deepcopy(T_armbase_ee)

        if along_base_frame:
            res, T_chassis_ee = self.get_current_xpos(name, False)
            if not res:
                return False
            target_xpos[:3,:3] = T_armbase_ee[:3,:3] @ T_chassis_ee[:3,:3].T @ delta_xpos[:3,:3] @ T_chassis_ee[:3,:3]
        else:
            target_xpos[:3,:3] = T_armbase_ee[:3,:3] @ delta_xpos[:3,:3]
        
        return self.move_linear_full_body(waypoints_dict={f"{name}": [xpos_to_user_pose(target_xpos)]},
                                        speed_ratio=speed_ratio)
    
    def move_dual_arm_single_task(self,
                         task_dict: dict,
                         speed_ratio: float = 1.0) -> bool:
        """
        This function is used to execute a single task, which only suits for 1 torso target, 1 head target, 1 arml target, 1 armr target

        Args:
            task_dict (dict): {
                              "torso": [torso_target_qpos], "troso_speed_ratio": [torso_speed],
                              "head": [head_target_qpos], "head_speed_ratio": [head_speed],
                              "left_arm": [left_arm_target_qpos], "left_arm_speed_ratio": [left_arm_speed],
                              "left_arm_ee": [[0., left_arm_ee_qpos, 0.5]], 
                              "right_arm": [right_arm_target_qpos], "right_arm_speed_ratio": [right_arm_speed],
                              "right_arm_ee": [[0., right_arm_ee_qpos, 0.5]]
                              }
            speed_ratio (float, optional): a ratio to scale the default joint velocity. Defaults to 1.0.
            
        Notes:
            - It will execute the moving of torso and head first, then execute the moving of left and right arms including the end effectors.
        """
        current_qpos = self.get_current_qpos()
        current_qpos = current_qpos[1]
        
        cur_torso_qpos = current_qpos[self.real_torso_idx[0]:self.real_torso_idx[1]+1]
        cur_head_qpos = current_qpos[self.real_head_idx[0]:self.real_head_idx[1]+1]
        cur_arml_qpos = current_qpos[self.real_arml_idx[0]:self.real_arml_idx[1]+1]
        cur_armr_qpos = current_qpos[self.real_armr_idx[0]:self.real_armr_idx[1]+1]

        # self.logger("Current torso qpos: ", cur_torso_qpos)
        res_torso, torso_qpos_array = self._plan_to_waypoint("torso",
                                                             cur_torso_qpos,
                                                             waypoints_dict=task_dict,
                                                             speed_ratio=speed_ratio)
        
        if not res_torso:
            return False
        else:
            for i in range(torso_qpos_array.shape[0]):
                target_torso_qpos = torso_qpos_array[i]
                real_qpos = np.concatenate((target_torso_qpos, cur_head_qpos, cur_arml_qpos, cur_armr_qpos))
                if self.check_collision(real_qpos):
                    self.logger.warning("It will occur collision for this torso moving trajectory")
                    return False
                
        res_head, head_qpos_array = self._plan_to_waypoint("head",
                                                           cur_head_qpos,
                                                           waypoints_dict=task_dict,
                                                           speed_ratio=speed_ratio)
        if not res_head:
            return False
        else:
            for i in range(head_qpos_array.shape[0]):
                target_head_qpos = head_qpos_array[i]
                real_qpos = np.concatenate((cur_torso_qpos, target_head_qpos, cur_arml_qpos, cur_armr_qpos))
                if self.check_collision(real_qpos):
                    self.logger.waring("It will occur collision for this head moving trajectory")
                    return False
        
        res_align, move_torso_head_traj = self._qpos_align(torso_qpos_array, head_qpos_array, cur_arml_qpos, cur_armr_qpos)
        res_execute = self.execute_joint_traj(qpos_array=move_torso_head_traj,
                                    left_arm_ee_trigger_list=None,
                                    right_arm_ee_trigger_list=None)
        if not res_execute:
            return False

        current_qpos = self.get_current_qpos()
        current_qpos = current_qpos[1]
        cur_torso_qpos = current_qpos[self.real_torso_idx[0]:self.real_torso_idx[1]+1]
        cur_head_qpos = current_qpos[self.real_head_idx[0]:self.real_head_idx[1]+1]
        cur_arml_qpos = current_qpos[self.real_arml_idx[0]:self.real_arml_idx[1]+1]
        cur_armr_qpos = current_qpos[self.real_armr_idx[0]:self.real_armr_idx[1]+1]

        arml_kine = self._get_arm_kine("left_arm")
        target_arml_pose = task_dict.get("left_arm")

        arml_speed_ratio = task_dict.get("left_arm_speed_ratio")
        if arml_speed_ratio is not None and arml_speed_ratio != []:
            if arml_speed_ratio[0] is not None:
                arml_speed_ratio = arml_speed_ratio[0]
            else:
                arml_speed_ratio = speed_ratio
        else:
            arml_speed_ratio = speed_ratio

        left_ref_psi = arml_kine.calc_arm_angle(cur_arml_qpos)[1]
        left_qpos_seed = cur_arml_qpos

        if target_arml_pose is not None and target_arml_pose != []:
            target_arml_xpos = user_pose_to_xpos(target_arml_pose[0][:3], target_arml_pose[0][3:])

            while True:
                srs_cfg = SRSConfig(left_qpos_seed, left_ref_psi)
                res, arml_ik_qpos = arml_kine.get_ik(target_arml_xpos, srs_cfg)
                if not res:
                    left_ref_psi += 5./180.*np.pi
                    continue
                else:
                    real_qpos = np.concatenate((cur_torso_qpos, cur_head_qpos, arml_ik_qpos, cur_armr_qpos))
                    if self.check_collision(real_qpos):
                        self.logger.warning("It will occur collision for this left arm ik solution, try to find another one")
                        left_ref_psi += 5./180.*np.pi
                        continue
                    else:
                        target_arml_qpos = arml_ik_qpos
                        self.logger.info(f"Successfully solve ik for left arm with qpos: {target_arml_qpos}")
                        break
        else:
            target_arml_qpos = cur_arml_qpos

        armr_kine = self._get_arm_kine("right_arm")
        target_armr_pose = task_dict.get("right_arm")

        armr_speed_ratio = task_dict.get("right_arm_speed_ratio")
        if armr_speed_ratio is not None and armr_speed_ratio != []:
            if armr_speed_ratio[0] is not None:
                armr_speed_ratio = armr_speed_ratio[0]
            else:
                armr_speed_ratio = speed_ratio
        else:
            armr_speed_ratio = speed_ratio

        right_ref_psi = armr_kine.calc_arm_angle(cur_armr_qpos)[1]
        right_qpos_seed = cur_armr_qpos

        if target_armr_pose is not None and target_armr_pose != []:
            target_armr_xpos = user_pose_to_xpos(target_armr_pose[0][:3], target_armr_pose[0][3:])
            while True:
                srs_cfg = SRSConfig(right_qpos_seed, right_ref_psi)
                res, armr_ik_qpos = armr_kine.get_ik(target_armr_xpos, srs_cfg)
                if not res:
                    right_ref_psi += 5./180.*np.pi
                    continue
                else:
                    real_qpos = np.concatenate((cur_torso_qpos, cur_head_qpos, cur_arml_qpos, armr_ik_qpos))
                    if self.check_collision(real_qpos):
                        self.logger.warning("It will occur collision for this right arm ik solution, try to find another one")
                        right_ref_psi += 5./180.*np.pi
                        continue
                    else:
                        target_armr_qpos = armr_ik_qpos
                        self.logger.info(f"Successfully solve ik for right arm with qpos: {target_armr_qpos}")
                        break
        else:
            target_armr_qpos = cur_armr_qpos

        self.siso_planner.set_head_torso(head_qpos=cur_head_qpos, torso_qpos=cur_torso_qpos)
        self.siso_planner.set_start_goal(arml_start=cur_arml_qpos, armr_start=cur_armr_qpos,
                                         arml_goal=target_arml_qpos, armr_goal=target_armr_qpos)
        self.siso_planner.set_traj_max_vel(left_max_vel=arml_speed_ratio*0.3, right_max_vel=armr_speed_ratio*0.3)
        arml_traj, armr_traj, planning_time = self.siso_planner.plan()

        self.logger.info(f"Finish Planning, time consumption: {planning_time}, with left arm traj shape: {None if arml_traj is None else arml_traj.shape}, right arm traj shape: {None if armr_traj is None else armr_traj.shape}")

        left_gripper_task = task_dict.get("left_arm_ee")
        if left_gripper_task is not None and left_gripper_task != []:
            if left_gripper_task[0] is not None and left_gripper_task[0] != []:
                lee_pre_time = left_gripper_task[0][0]
                lee_post_time = left_gripper_task[0][2]
                lee_target = left_gripper_task[0][1]
            else:
                lee_pre_time = 0.
                lee_post_time = 0.
                lee_target = None
        else:   
            lee_pre_time = 0.
            lee_post_time = 0.
            lee_target = None

        arml_pre_insert = lee_pre_time/self.TIME_STEP
        arml_post_insert = lee_post_time/self.TIME_STEP
        
        right_gripper_task = task_dict.get("right_arm_ee")
        if right_gripper_task is not None and right_gripper_task != []:
            if right_gripper_task[0] is not None and right_gripper_task[0] != []:
                ree_pre_time = right_gripper_task[0][0]
                ree_post_time = right_gripper_task[0][2]
                ree_target = right_gripper_task[0][1]
            else:
                ree_pre_time = 0.
                ree_post_time = 0.
                ree_target = None
        else:   
            ree_pre_time = 0.
            ree_post_time = 0.
            ree_target = None

        armr_pre_insert = ree_pre_time/self.TIME_STEP
        armr_post_insert = ree_post_time/self.TIME_STEP
        
        armlee_trigger, armree_trigger = [], [] # (idx, ee_qpos)

        if arml_traj is not None and armr_traj is not None:
            if lee_target is not None:
                armlee_trigger.append((arml_traj.shape[0]+arml_pre_insert, lee_target))
            else:
                armlee_trigger = None
            if ree_target is not None:
                armree_trigger.append((armr_traj.shape[0]+armr_pre_insert, ree_target))
            else: 
                armree_trigger = None 

            arml_traj = np.vstack([arml_traj, np.tile(arml_traj[-1, :], (int(arml_pre_insert+arml_post_insert), 1))])
            armr_traj = np.vstack([armr_traj, np.tile(armr_traj[-1, :], (int(armr_pre_insert+armr_post_insert), 1))])

            res_align, move_arml_armr_traj = self._qpos_align(cur_torso_qpos, cur_head_qpos, arml_traj, armr_traj)
            return self.execute_joint_traj(qpos_array=move_arml_armr_traj,
                                        left_arm_ee_trigger_list=armlee_trigger,
                                        right_arm_ee_trigger_list=armree_trigger)

        if arml_traj is not None and armr_traj is None:
            if lee_target is not None:
                armlee_trigger.append((arml_traj.shape[0]+arml_pre_insert, lee_target))
            else:
                armlee_trigger = None
            if ree_target is not None:
                armree_trigger.append((armr_pre_insert, ree_target))
            else: 
                armree_trigger = None 

            self.logger.info(armlee_trigger, armree_trigger)
            arml_traj = np.vstack([arml_traj, np.tile(arml_traj[-1, :], (int(arml_pre_insert+arml_post_insert), 1))])

            res_align, move_arml_armr_traj = self._qpos_align(cur_torso_qpos, cur_head_qpos, arml_traj, cur_armr_qpos)
            return self.execute_joint_traj(qpos_array=move_arml_armr_traj,
                                        left_arm_ee_trigger_list=armlee_trigger,
                                        right_arm_ee_trigger_list=armree_trigger)
            
        if arml_traj is None and armr_traj is not None:
            if lee_target is not None:
                armlee_trigger.append((arml_pre_insert, lee_target))
            else:
                armlee_trigger = None
            if ree_target is not None:
                armree_trigger.append((armr_traj.shape[0]+armr_pre_insert, ree_target))
            else: 
                armree_trigger = None 

            armr_traj = np.vstack([armr_traj, np.tile(armr_traj[-1, :], (int(armr_pre_insert+armr_post_insert), 1))])

            res_align, move_arml_armr_traj = self._qpos_align(cur_torso_qpos, cur_head_qpos, cur_arml_qpos, armr_traj)
            return self.execute_joint_traj(qpos_array=move_arml_armr_traj,
                                        left_arm_ee_trigger_list=armlee_trigger,
                                        right_arm_ee_trigger_list=armree_trigger)
        
        if arml_traj is None and armr_traj is None:
            if lee_target is not None:
                armlee_trigger.append((arml_pre_insert, lee_target))
            else:
                armlee_trigger = None
            if ree_target is not None:
                armree_trigger.append((armr_pre_insert, ree_target))
            else:
                armree_trigger = None

            arml_traj = np.tile(cur_arml_qpos, (arml_pre_insert+arml_post_insert, 1))
            armr_traj = np.tile(cur_armr_qpos, (armr_pre_insert+armr_post_insert, 1))

            res_align, move_arml_armr_traj = self._qpos_align(cur_torso_qpos, cur_head_qpos, arml_traj, armr_traj)
            return self.execute_joint_traj(qpos_array=move_arml_armr_traj,
                                        left_arm_ee_trigger_list=armlee_trigger,
                                        right_arm_ee_trigger_list=armree_trigger)           

    def move_single_arm(self,
                        task_dict: dict,
                        speed_ratio: float = 1.0):
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
        arml_target_list = task_dict.get("left_arm")
        armr_target_list = task_dict.get("right_arm")
        if arml_target_list is not None and arml_target_list != [] and armr_target_list is not None and armr_target_list != []:
            self.logger.info("The input task_dict should only have one arm's target, the other arm's target should be None or []")
            return False
        
        current_qpos = self.get_current_qpos()
        current_qpos = current_qpos[1]
        cur_torso_qpos = current_qpos[self.real_torso_idx[0]:self.real_torso_idx[1]+1]
        cur_head_qpos = current_qpos[self.real_head_idx[0]:self.real_head_idx[1]+1]
        cur_arml_qpos = current_qpos[self.real_arml_idx[0]:self.real_arml_idx[1]+1]
        cur_armr_qpos = current_qpos[self.real_armr_idx[0]:self.real_armr_idx[1]+1]

        res_torso, torso_qpos_array = self._plan_to_waypoint("torso",
                                                             cur_torso_qpos,
                                                             waypoints_dict=task_dict,
                                                             speed_ratio=speed_ratio)
        if not res_torso:
            return False
        else:
            for i in range(torso_qpos_array.shape[0]):
                target_torso_qpos = torso_qpos_array[i]
                real_qpos = np.concatenate((target_torso_qpos, cur_head_qpos, cur_arml_qpos, cur_armr_qpos))
                if self.check_collision(real_qpos):
                    self.logger.warning("It will occur collision for this torso moving trajectory")
                    return False
                
        res_head, head_qpos_array = self._plan_to_waypoint("head",
                                                           cur_head_qpos,
                                                           waypoints_dict=task_dict,
                                                           speed_ratio=speed_ratio)
        if not res_head:
            return False
        else:
            for i in range(head_qpos_array.shape[0]):
                target_head_qpos = head_qpos_array[i]
                real_qpos = np.concatenate((cur_torso_qpos, target_head_qpos, cur_arml_qpos, cur_armr_qpos))
                if self.check_collision(real_qpos):
                    self.logger.info("It will occur collision for this head moving trajectory")
                    return False
        res_align, move_torso_head_traj = self._qpos_align(torso_qpos_array, head_qpos_array, cur_arml_qpos, cur_armr_qpos)
        self.execute_joint_traj(qpos_array=move_torso_head_traj,
                                left_arm_ee_trigger_list=None,
                                right_arm_ee_trigger_list=None
                                )
        
        

        if arml_target_list is not None and arml_target_list != []:
            lee_target_list = task_dict.get("left_arm_ee")
            if len(arml_target_list) != len(lee_target_list):
                self.logger.warning("The length of left_arm and left_arm_ee in task_dict should be the same")
                return False

            arml_kine = self._get_arm_kine("left_arm")
            arml_speed_ratio_list = task_dict.get("left_arm_speed_ratio")
            if arml_speed_ratio_list is not None and arml_speed_ratio_list != []:
                for i in range(len(arml_speed_ratio_list)):
                    if arml_speed_ratio_list[i] is None:
                        arml_speed_ratio_list[i] = speed_ratio
            else:
                arml_speed_ratio_list = [speed_ratio]*len(arml_target_list)

            for i in range(len(arml_target_list)):

                current_qpos = self.get_current_qpos()
                current_qpos = current_qpos[1]
                cur_torso_qpos = current_qpos[self.real_torso_idx[0]:self.real_torso_idx[1]+1]
                cur_head_qpos = current_qpos[self.real_head_idx[0]:self.real_head_idx[1]+1]
                cur_arml_qpos = current_qpos[self.real_arml_idx[0]:self.real_arml_idx[1]+1]
                cur_armr_qpos = current_qpos[self.real_armr_idx[0]:self.real_armr_idx[1]+1]

                arml_trigger = []
                target_arml_pose = arml_target_list[i]
                arml_speed_ratio = arml_speed_ratio_list[i]
                left_ref_psi = arml_kine.calc_arm_angle(cur_arml_qpos)[1]
                left_qpos_seed = cur_arml_qpos
                if target_arml_pose is not None and target_arml_pose != []:
                    target_arml_xpos = user_pose_to_xpos(target_arml_pose[:3], target_arml_pose[3:])
                    while True:
                        srs_cfg = SRSConfig(left_qpos_seed, left_ref_psi)
                        res_ik, arml_ik_qpos = arml_kine.get_ik(target_arml_xpos, srs_cfg)
                        if not res_ik:
                            left_ref_psi += 5./180.*np.pi
                            continue
                        else:
                            real_qpos = np.concatenate((cur_torso_qpos, cur_head_qpos, arml_ik_qpos, cur_armr_qpos))
                            if self.check_collision(real_qpos):
                                self.logger.warning("It will occur collision for this left arm ik solution, try to find another one")
                                left_ref_psi += 5./180.*np.pi
                                continue
                            else:
                                target_arml_qpos = arml_ik_qpos
                                self.logger.info(f"Successfully solve ik for left arm with qpos: {target_arml_qpos}")
                                break
                else:
                    self.logger.warning("invalid left arm target input")
                    return False    

                self.siso_planner.set_head_torso(head_qpos=cur_head_qpos, torso_qpos=cur_torso_qpos)
                self.siso_planner.set_start_goal(arml_start=cur_arml_qpos, armr_start=cur_armr_qpos,
                                                 arml_goal=target_arml_qpos, armr_goal=cur_armr_qpos)
                self.siso_planner.set_traj_max_vel(left_max_vel=arml_speed_ratio*0.3, right_max_vel=0.3)
                arml_traj, armr_traj, planning_time = self.siso_planner.plan()

                if arml_traj is None:
                    arml_traj = np.array([cur_arml_qpos])

                lee_pre_time = lee_target_list[i][0]
                lee_post_time = lee_target_list[i][2]
                lee_target = lee_target_list[i][1]

                arml_pre_insert = lee_pre_time/self.TIME_STEP
                arml_post_insert = lee_post_time/self.TIME_STEP

                arml_trigger.append((arml_traj.shape[0]+arml_pre_insert, lee_target))
                arml_traj = np.vstack([arml_traj, np.tile(arml_traj[-1, :], (int(arml_pre_insert+arml_post_insert), 1))])
                res_align, move_arml_armr_traj = self._qpos_align(cur_torso_qpos, cur_head_qpos, arml_traj, cur_armr_qpos)
                self.execute_joint_traj(qpos_array=move_arml_armr_traj,
                                        left_arm_ee_trigger_list=arml_trigger,
                                        right_arm_ee_trigger_list=None)
                
        elif armr_target_list is not None and armr_target_list != []:
            ree_target_list = task_dict.get("right_arm_ee")
            if len(armr_target_list) != len(ree_target_list):
                self.logger.warning("The length of right_arm and right_arm_ee in task_dict should be the same")
                return False

            armr_kine = self._get_arm_kine("right_arm")
            armr_speed_ratio_list = task_dict.get("right_arm_speed_ratio")
            if armr_speed_ratio_list is not None and armr_speed_ratio_list != []:
                for i in range(len(armr_speed_ratio_list)):
                    if armr_speed_ratio_list[i] is None:
                        armr_speed_ratio_list[i] = speed_ratio
            else:
                armr_speed_ratio_list = [speed_ratio]*len(armr_target_list)

            for i in range(len(armr_target_list)):
                current_qpos = self.get_current_qpos()
                current_qpos = current_qpos[1]
                cur_torso_qpos = current_qpos[self.real_torso_idx[0]:self.real_torso_idx[1]+1]
                cur_head_qpos = current_qpos[self.real_head_idx[0]:self.real_head_idx[1]+1]
                cur_arml_qpos = current_qpos[self.real_arml_idx[0]:self.real_arml_idx[1]+1]
                cur_armr_qpos = current_qpos[self.real_armr_idx[0]:self.real_armr_idx[1]+1]

                armr_trigger = []
                target_armr_pose = armr_target_list[i]
                armr_speed_ratio = armr_speed_ratio_list[i]

                right_ref_psi = armr_kine.calc_arm_angle(cur_armr_qpos)[1]
                right_qpos_seed = cur_armr_qpos
                if target_armr_pose is not None and target_armr_pose != []:
                    target_armr_xpos = user_pose_to_xpos(target_armr_pose[:3], target_armr_pose[3:])
                    while True:
                        srs_cfg = SRSConfig(right_qpos_seed, right_ref_psi)
                        res_ik, armr_ik_qpos = armr_kine.get_ik(target_armr_xpos, srs_cfg)
                        if not res_ik:
                            right_ref_psi += 5./180.*np.pi
                            continue
                        else:
                            real_qpos = np.concatenate((cur_torso_qpos, cur_head_qpos, cur_arml_qpos, armr_ik_qpos))
                            if self.check_collision(real_qpos):
                                self.logger.warning("It will occur collision for this right arm ik solution, try to find another one")
                                right_ref_psi += 5./180.*np.pi
                                continue
                            else:
                                target_armr_qpos = armr_ik_qpos
                                self.logger.info(f"Successfully solve ik for right arm with qpos: {target_armr_qpos}")
                                break
                else:
                    self.logger.warning("invalid right arm target")
                    return False    

                self.siso_planner.set_head_torso(head_qpos=cur_head_qpos, torso_qpos=cur_torso_qpos)
                self.siso_planner.set_start_goal(arml_start=cur_arml_qpos, armr_start=cur_armr_qpos,
                                                 arml_goal=cur_arml_qpos, armr_goal=target_armr_qpos)
                self.siso_planner.set_traj_max_vel(left_max_vel=0.3, right_max_vel=armr_speed_ratio*0.3)
                arml_traj, armr_traj, planning_time = self.siso_planner.plan()  

                if armr_traj is None:
                    armr_traj = np.array([cur_armr_qpos])

                ree_pre_time = ree_target_list[i][0]
                ree_post_time = ree_target_list[i][2]
                ree_target = ree_target_list[i][1]
                armr_pre_insert = ree_pre_time/self.TIME_STEP   
                armr_post_insert = ree_post_time/self.TIME_STEP
                armr_trigger.append((armr_traj.shape[0]+armr_pre_insert, ree_target))
                armr_traj = np.vstack([armr_traj, np.tile(armr_traj[-1, :], (int(armr_pre_insert+armr_post_insert), 1))])
                res_align, move_arml_armr_traj = self._qpos_align(cur_torso_qpos, cur_head_qpos, cur_arml_qpos, armr_traj)
                return self.execute_joint_traj(qpos_array=move_arml_armr_traj,
                                            left_arm_ee_trigger_list=None,
                                            right_arm_ee_trigger_list=armr_trigger)
      
