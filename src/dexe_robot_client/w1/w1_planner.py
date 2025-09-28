# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

from ..common import time_consuming, get_logger, DATA_DIR, W1ArmType, W1Version
from ..end_effector_client import EndEffectorType, create_end_effector_client
from ..planner_client import PlannerClient
from ..camera_client import OrbbecClient
from ..robot_client import RobotClientRos2 as RobotClient
from ..robot_client import MotionGroup, Status
from .kinemtaics.torso_kinematics import TorsoKinematics
from .kinemtaics.arm_kinematics import SRSKinematics
from .path_planner.bezier_blend_path import interpolate_cartesian_path
from .trajectory_planner.toppra_trajectory_planner import ToppraTrajectoryPlanner
from .utils.w1_params import W1Params
from .utils.unit_conversion import user_pose_to_xpos, xpos_to_user_pose
from .path_planner.utils.robot_type import *
from .path_planner.utils.mapping_index import *
from .path_planner.fcl_checker import FCLChecker
from .path_planner.planner_pinocchio import Planner_Pin
from .path_planner.single_planner import SinglePlanner
from .path_planner.vamp_planner import VampPlanner
from robot_client_interfaces.srv import ClientService

import numpy as np
import open3d as o3d
import threading
import time
import pathlib, json
from copy import deepcopy
import atexit
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from IPython import embed

class W1Planner:
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
                use_rlia: bool = True):
        self.logger = get_logger()
        self.TIME_STEP = 0.01  # 100Hz

        self._is_sim = is_sim
        self._arm_type = arm_type
        self._w1_version = w1_version
        self._end_effector_type = end_effector_type

        self.w1_params = W1Params(self._arm_type, 
                                  self._w1_version, 
                                  self._end_effector_type)
        if not rclpy.ok():
            rclpy.init()
        self.executor = MultiThreadedExecutor()

        self._init_service_clients()
        self._init_planner_clients()

        self._init_ros_thread()

        self._use_rlia = use_rlia
        self._init_kine_solver()
        self._init_fcl_checker(arm_type=arm_type, w1_version=w1_version, end_effector_type=end_effector_type)
        self._init_pinocchio(arm_type=arm_type, w1_version=w1_version, end_effector_type=end_effector_type)
        self.real_torso_idx, self.real_head_idx, self.real_arml_idx, self.real_armr_idx = get_real_idx()
        self._init_planner(arm_type=arm_type, w1_version=w1_version, end_effector_type=end_effector_type)


    def _init_service_clients(self):
        if not rclpy.ok():
            rclpy.init()
        self.service_executor = MultiThreadedExecutor()

        self.service_node = Node("w1_planner_service_node")

        self._has_arm_ee_client_client = self.service_node.create_client(ClientService,
                                                                        "_has_arm_ee_client")
        while not self._has_arm_ee_client_client.wait_for_service(timeout_sec=1.0):
            self.logger.warning("_has_arm_ee_client service not available, waiting again...")
        self.connect_client = self.service_node.create_client(ClientService,
                                                              "connect")
        while not self.connect_client.wait_for_service(timeout_sec=1.0):
            self.logger.warning("connect service not available, waiting again...")
        self.is_connected_client = self.service_node.create_client(ClientService,
                                                                  "is_connected")
        while not self.is_connected_client.wait_for_service(timeout_sec=1.0):
            self.logger.warning("is_connected service not available, waiting again...")
        self.clear_robot_error_client = self.service_node.create_client(ClientService,
                                                                      "clear_robot_error")
        while not self.clear_robot_error_client.wait_for_service(timeout_sec=1.0):
            self.logger.warning("clear_robot_error service not available, waiting again...")
        self.set_drag_mode_client = self.service_node.create_client(ClientService,
                                                                   "set_drag_mode")
        while not self.set_drag_mode_client.wait_for_service(timeout_sec=1.0):
            self.logger.warning("set_drag_mode service not available, waiting again...")
        self.set_safe_level_client = self.service_node.create_client(ClientService,
                                                                    "set_safe_level")
        while not self.set_safe_level_client.wait_for_service(timeout_sec=1.0):
            self.logger.warning("set_safe_level service not available, waiting again...")
        self.set_sim_mode_client = self.service_node.create_client(ClientService,
                                                                  "set_sim_mode")
        while not self.set_sim_mode_client.wait_for_service(timeout_sec=1.0):
            self.logger.warning("set_sim_mode service not available, waiting again...")
        self.set_ee_qpos_client = self.service_node.create_client(ClientService,
                                                                 "set_ee_qpos")
        while not self.set_ee_qpos_client.wait_for_service(timeout_sec=1.0):
            self.logger.warning("set_ee_qpos service not available, waiting again...")
        self.go_home_client = self.service_node.create_client(ClientService,    
                                                             "go_home")
        while not self.go_home_client.wait_for_service(timeout_sec=1.0):
            self.logger.warning("go_home service not available, waiting again...")
        self.stop_move_client = self.service_node.create_client(ClientService,
                                                               "stop_move")
        while not self.stop_move_client.wait_for_service(timeout_sec=1.0):
            self.logger.warning("stop_move service not available, waiting again...")
        self.set_home_qpos_client = self.service_node.create_client(ClientService,
                                                                   "set_home_qpos")
        while not self.set_home_qpos_client.wait_for_service(timeout_sec=1.0):
            self.logger.warning("set_home_qpos service not available, waiting again...")
        self.execute_joint_traj_client = self.service_node.create_client(ClientService,
                                                                       "execute_joint_traj")
        while not self.execute_joint_traj_client.wait_for_service(timeout_sec=1.0):
            self.logger.warning("execute_joint_traj service not available, waiting again...")

        self.start_moving_client = self.service_node.create_client(ClientService,
                                                                  "start_moving")
        while not self.start_moving_client.wait_for_service(timeout_sec=1.0):
            self.logger.warning("start_moving service not available, waiting again...")


        self.logger.info("Connected to controller service.")

    def _init_planner_clients(self):
        self.planner_client = PlannerClient(
            is_sim=self._is_sim,
            end_effector_type=self._end_effector_type
        )
        self.executor.add_node(self.planner_client)

    def _init_ros_thread(self):
        if not rclpy.ok():
            rclpy.init()
        self.ros_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.ros_thread.start()


    def _init_kine_solver(self):
        self.torso_kine_solver = TorsoKinematics()

        ik_nearst_weight = np.array([2., 2., 2., 1., 1., 1., 1.])
        if self._use_rlia:
            import rlia
            from rlia.kit.drive_controllers import W1DriveController
            rlia_w1_version = rlia.kit.drive_controllers.w1_drive_controller.W1Version.V2
            if self._arm_type == W1ArmType.INDUSTRIAL:
                rlia_arm_type = rlia.kit.drive_controllers.w1_drive_controller.W1ArmType.INDUSTRIAL
            else:
                rlia_arm_type = rlia.kit.drive_controllers.w1_drive_controller.W1ArmType.ANTHROPOMORPHIC

            self.left_arm_kine = W1DriveController(is_left_arm = True, 
                                                    type_version = rlia_w1_version, 
                                                    arm_type = rlia_arm_type,
                                                    is_yaik_solver=True)
            self.left_arm_kine.set_joint_limits(upper_limits = self.w1_params.left_arm_ub,
                                                    lower_limits = self.w1_params.left_arm_lb)
            self.left_arm_kine.set_user_joint_limits(upper_limits = self.w1_params.left_arm_ub,
                                                    lower_limits = self.w1_params.left_arm_lb)
            self.left_arm_kine.set_ik_nearst_weight(ik_nearst_weight)

            self.right_arm_kine = W1DriveController(is_left_arm = False, 
                                                    type_version = rlia_w1_version, 
                                                    arm_type = rlia_arm_type,
                                                    is_yaik_solver=True)
            self.right_arm_kine.set_joint_limits(upper_limits = self.w1_params.right_arm_ub,
                                                    lower_limits = self.w1_params.right_arm_lb)
            self.right_arm_kine.set_user_joint_limits(upper_limits = self.w1_params.right_arm_ub,
                                                    lower_limits = self.w1_params.right_arm_lb)
            self.right_arm_kine.set_ik_nearst_weight(ik_nearst_weight)

            self.logger.info("Initialize rlia kinematic solver.")
        
        else:
            self.left_arm_kine = SRSKinematics(self.w1_params._left_arm_kine_params)
            self.left_arm_kine.set_joint_limits(lower_bound = self.w1_params.left_arm_lb,
                                                upper_bound = self.w1_params.left_arm_ub)
            self.left_arm_kine.set_user_joint_limits(lower_bound = self.w1_params.left_arm_lb,
                                                     upper_bound = self.w1_params.left_arm_ub)
            self.left_arm_kine.set_ik_nearst_weight(ik_nearst_weight)

            self.right_arm_kine = SRSKinematics(self.w1_params._right_arm_kine_params)
            self.right_arm_kine.set_joint_limits(lower_bound = self.w1_params.right_arm_lb,
                                                upper_bound = self.w1_params.right_arm_ub)
            self.right_arm_kine.set_user_joint_limits(lower_bound = self.w1_params.right_arm_lb,
                                                     upper_bound = self.w1_params.right_arm_ub)
            self.right_arm_kine.set_ik_nearst_weight(ik_nearst_weight)

            self.logger.info("Initialize srs-python kinematic solver.")

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
            env_json_path = get_env_json_path()
            # self.fcl_checker.load_env_json(env_json_path)
        else:
            raise ValueError(f"Not supported w1 version: {w1_version} for fcl checker")
        
    def _init_pinocchio(self, arm_type: W1ArmType,
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
            SinglePlanner.set_env_json(get_env_json_path())
            self.siso_planner = SinglePlanner()

    def _get_arm_kine(self, name: str):
        if not self._check_arm_name(name):
            return None
        if name == "left_arm":
            return self.left_arm_kine
        elif name == "right_arm":
            return self.right_arm_kine

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
        #TODO: 检查是否真的同步或者异步
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
            self.logger.error(f"{name} {invalid_idx+1}th qpos({qpos.tolist()}) " + \
                              f"excceeded limits([{qpos_lb[invalid_idx]}, {qpos_ub[invalid_idx]}]).")
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
    
    def _has_arm_ee_client(self, name: str, verbose: bool = True) -> bool:
        req = ClientService.Request()
        req.request = json.dumps({"name": name, "verbose": verbose})
        future = self._has_arm_ee_client_client.call_async(req)
        rclpy.spin_until_future_complete(self.service_node, future)
        res = json.loads(future.result().response)["has_arm_ee_client"]
        return res
    

    ######## 基础API #############
    def connect(self, timeout_sec: float = 0.5) -> bool:
        """连接client(sim/real)

        Returns:
            bool: 
        """
        req = ClientService.Request()
        req.request = json.dumps({"timeout_sec": timeout_sec})
        future = self.connect_client.call_async(req)
        rclpy.spin_until_future_complete(self.service_node, future)
        res = json.loads(future.result().response)["is_connected"]
        return res
    
    def is_connected(self, timeout_sec: float = 0.1) -> bool:
        """检查所有client是否处于连接状态

        Returns:
            bool: 
        """
        req = ClientService.Request()
        req.request = json.dumps({"timeout_sec": timeout_sec})
        future = self.is_connected_client.call_async(req)
        rclpy.spin_until_future_complete(self.service_node, future)
        res = json.loads(future.result().response)["is_connected"]
        return res
    
    def clear_robot_error(self) -> bool:
        """清除机器人错误

        Returns:
            bool: 
        """
        req = ClientService.Request()
        req.request = json.dumps({})
        future = self.clear_robot_error_client.call_async(req)
        rclpy.spin_until_future_complete(self.service_node, future)
        res = json.loads(future.result().response)["is_cleared"]
        return res
    
    def set_drag_mode(self, enable_drag: bool) -> bool:
        """开关机器人拖动示教模式

        Args:
            enable_drag (bool): 是否开启拖动示教模式

        Returns:
            bool: 
        """
        req = ClientService.Request()
        req.request = json.dumps({"enable_drag": enable_drag})
        future = self.set_drag_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self.service_node, future)
        res = json.loads(future.result().response)["is_set"]
        return res
    
    def set_safe_level(self, safe_level: float) -> bool:
        """设置机器人安全等级

        Args:
            safe_level (float): 安全级别, 取值范围 [0.1, 1], 默认为0.5  
                                0.1时机械臂柔性最大,1.0时机械臂刚性最大   
                                值越小机械臂柔性越大越安全, 刚性越大运动跟踪性能越好但碰撞时容易造成机械臂损坏

        Returns:
            bool: 
        """
        req = ClientService.Request()
        req.request = json.dumps({"safe_level": safe_level})
        future = self.set_safe_level_client.call_async(req)
        rclpy.spin_until_future_complete(self.service_node, future)
        res = json.loads(future.result().response)["is_set"]
        return res
    
    def set_sim_mode(self, enable_sim: bool) -> bool:
        req = ClientService.Request()
        req.request = json.dumps({"enable_sim": enable_sim})
        future = self.set_sim_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self.service_node, future)
        res = json.loads(future.result().response)["is_set"]
        return res
    
    def set_ee_qpos(self, name: str, qpos: np.ndarray) -> bool:
        """控制end effector运动

        Args:
            name (str): ["left_arm", "right_arm"]
            qpos (np.ndarray): 期望位置 0~100

        Returns:
            bool
        """
        if not self._check_qpos_dimension(name, np.array(qpos)):
            return False
        req = ClientService.Request()
        req.request = json.dumps({"name": name, "qpos": qpos.tolist()})
        future = self.set_ee_qpos_client.call_async(req)
        rclpy.spin_until_future_complete(self.service_node, future)
        res = json.loads(future.result().response)["is_set"]
        return res
    
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
        req = ClientService.Request()
        req.request = json.dumps({})
        future = self.stop_move_client.call_async(req)
        rclpy.spin_until_future_complete(self.service_node, future)
        res = json.loads(future.result().response)["is_stopped"]
        return res



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

        req = ClientService.Request()
        req.request = json.dumps({"home_qpos": home_qpos.tolist(), "save_to_file": save_to_file})
        future = self.set_home_qpos_client.call_async(req)
        rclpy.spin_until_future_complete(self.service_node, future)
        res = json.loads(future.result().response)["is_set"]

        if save_to_file:
            return self._save_home_qpos()
        
        return res

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
            if name == "left_arm_ee":
                return self.planner_client.get_left_ee_state()
            elif name == "right_arm_ee":
                return self.planner_client.get_right_ee_state()
        
        res, robot_status, robot_joint_position = self.planner_client.get_state()
        if not res:
            return False, None
        
        cur_qpos = np.array(robot_joint_position)

        if name == "full_body":
            return True, cur_qpos
        return True, cur_qpos[self.w1_params.index_infos[name]["index"]]
    
    def get_current_xpos(self, name: str) -> tuple[bool, np.ndarray]:
        """获取left_arm/right_arm的当前笛卡尔位置

        Args:
            name (str): ["left_arm", "right_arm"]

        Returns:
            tuple[bool, np.ndarray]: 
        """
        res, cur_qpos = self.get_current_qpos(name)
        if not res:
            return False, None
        return self.get_xpos(name, cur_qpos)
    
    def get_xpos(self, name: str, qpos: np.ndarray) -> tuple[bool, np.ndarray]:
        """获取left_arm/right_arm的目标qpos的笛卡尔位置

        Args:
            name (str): ["left_arm", "right_arm"]
            qpos (np.ndarray): 目标关节角

        Returns:
            tuple[bool, np.ndarray]: 
        """
        if not self._check_qpos_dimension(name, qpos):
            return False, None
        
        arm_kine = self._get_arm_kine(name)
        if arm_kine is None:
            return False, None
        
        res, cur_xpos = arm_kine.get_fk(qpos=qpos)
        if not res:
            self.logger.error(f"{name} solve fk failed")
            return False, None
        return True, cur_xpos

    def set_tcp(self, name: str, xpos:np.ndarray) -> bool:
        # TODO: set tcp in robot client too
        arm_kine = self._get_arm_kine(name)
        if arm_kine is None:
            return False
        arm_kine.set_tcp(xpos=xpos)
        return True
    
    def get_tcp(self, name: str) -> tuple[bool, np.ndarray]:
        arm_kine = self._get_arm_kine(name)
        if arm_kine is None:
            return False, None
        return True, arm_kine.get_tcp()
    
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
            if self._use_rlia:
                res_ik, cur_qpos = arm_kine.get_ik(xpos=cur_xpos, 
                                                    qpos_seed=cur_qseed)
            else:
                res_ik, cur_qpos = arm_kine.get_nearst_ik(xpos=cur_xpos, 
                                                    qpos_seed=cur_qseed)
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
                           reach_timeout: float = 1.0) -> bool:
        """按10ms间隔执行关节轨迹

        Args:
            qpos_array (np.ndarray): 关节轨迹
            left_arm_ee_trigger_list (list, optional): 左末端执行器触发序列. Defaults to None.
            right_arm_ee_trigger_list (list, optional): 右末端执行器触发序列. Defaults to None.
            reach_timeout (float, optional): 发送完指令后的到位检测timeout. Defaults to 2.0.

        Returns:
            bool: 
        """

        req = ClientService.Request()
        req.request = json.dumps({"qpos_array": qpos_array.tolist(), 
                                  "left_arm_ee_trigger_list": left_arm_ee_trigger_list,
                                  "right_arm_ee_trigger_list": right_arm_ee_trigger_list,
                                  "reach_timeout": reach_timeout})
        future = self.execute_joint_traj_client.call_async(req)
        rclpy.spin_until_future_complete(self.service_node, future)
        res = json.loads(future.result().response)["is_executed"]
        return res
    
    ########## demo定制API ###########

    def get_current_pose(self, name: str) -> tuple[bool, list]:
        """获取left_arm/right_arm的当前笛卡尔位置[xyz_mm, rpy_deg]

        Args:
            name (str): ["left_arm", "right_arm"]

        Returns:
            tuple[bool, list]: 
        """
        res, cur_xpos = self.get_current_xpos(name)
        if not res: 
            return False, None
        return True, xpos_to_user_pose(cur_xpos)

    def get_pose(self, name: str, qpos: np.ndarray) -> tuple[bool, list]:
        """获取left_arm/right_arm目标qpos的笛卡尔位置[xyz_mm, rpy_deg]

        Args:
            name (str): ["left_arm", "right_arm"]
            qpos (np.ndarray): 目标关节角

        Returns:
            tuple[bool, list]: 
        """
        res, cur_xpos = self.get_xpos(name, qpos)
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

        if name == "left_arm":
            is_left_base = True
        elif name == "right_arm":
            is_left_base = False

        pre_xpos = user_pose_to_xpos(pre_grasp_pose[:3], pre_grasp_pose[3:])
        pre_xpos_torso_armbase = self.torso_kine_solver.get_fk(pre_torso_qpos, is_left_base)
        target_xpos_torso_armbase = self.torso_kine_solver.get_fk(target_torso_qpos, is_left_base)
        target_xpos = np.linalg.inv(target_xpos_torso_armbase) @ pre_xpos_torso_armbase @ pre_xpos
        return True, xpos_to_user_pose(target_xpos)

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
                          cartesian_step: float = None) -> tuple[bool, np.ndarray]:
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
            res_fk, cur_xpos = self._get_arm_kine(name).get_fk(cur_qpos)
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
                                  cartesian_step: float = None) -> tuple[bool, np.ndarray, list]:
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
                                                    cartesian_step)
            if not res:
                return False, None, None
            return True, qpos_array, None

        # with ee motion
        if not is_cartesian:
            pos_raw_array_ = np.vstack([cur_qpos, pos_raw_array])
        else:
            res_fk, cur_xpos = self._get_arm_kine(name).get_fk(cur_qpos)
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
                self.logger.error("Start left arm ee qpos out of limit.")
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
                self.logger.error("Start right arm ee qpos out of limit.")
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
            self.logger.error("Align full body traj failed.")
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
                             speed_ratio: float = 1.0) -> tuple[bool, np.ndarray, list, list]:
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
            
            cartesian_step (float, optional): cartesian sample step. Defaults to 0.05.

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
        
        if left_qpos_seed is None:
            left_qpos_seed = start_left_arm_qpos
        if right_qpos_seed is None:
            right_qpos_seed = start_right_arm_qpos

        self.logger.debug(f"plan_joint_full_body() input cur_qpos: {start_qpos}")
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
                                                        speed_ratio,
                                                        is_cartesian=True,
                                                        qpos_seed=left_qpos_seed,
                                                        cartesian_step=cartesian_step)
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
                                                                       speed_ratio,
                                                                       is_cartesian=True,
                                                                       qpos_seed=left_qpos_seed,
                                                                       cartesian_step=cartesian_step)
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
                                                        speed_ratio,
                                                        is_cartesian=True,
                                                        qpos_seed=right_qpos_seed,
                                                        cartesian_step=cartesian_step)
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
                                                                       speed_ratio,
                                                                       is_cartesian=True,
                                                                       qpos_seed=right_qpos_seed,
                                                                       cartesian_step=cartesian_step)
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
                              speed_ratio: float = 1.0) -> bool:
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

            cartesian_step (float, optional): cartesian sample step. Defaults to 0.05.

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
         right_arm_ee_trigger_list) = self.plan_linear_full_body(cur_qpos, 
                                                                cur_left_arm_ee_qpos, 
                                                                cur_right_arm_ee_qpos, 
                                                                waypoints_dict,
                                                                left_qpos_seed,
                                                                right_qpos_seed,
                                                                cartesian_step,
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

    def move_arm_linear(self, 
                        name: str,
                        delta_trans_mm: list, 
                        speed_ratio: float = 1.,
                        along_base_frame: bool = True) -> bool:
        if not self._check_arm_name(name):
            return False
        res, cur_xpos = self.get_current_xpos(name)
        if not res:
            return False
        delta_xpos = user_pose_to_xpos(delta_trans_mm, [0., 0., 0.])
        if along_base_frame:
            target_xpos = delta_xpos @ cur_xpos
        else:
            target_xpos = cur_xpos @ delta_xpos
        
        return self.move_linear_full_body(waypoints_dict={f"{name}": [xpos_to_user_pose(target_xpos)]},
                                        speed_ratio=speed_ratio)

    def rotate_arm_linear(self, 
                        name: str,
                        delta_rot_deg: list, 
                        speed_ratio: float = 1.,
                        along_base_frame: bool = True) -> bool:
        if not self._check_arm_name(name):
            return False
        res, cur_xpos = self.get_current_xpos(name)
        if not res:
            return False
        delta_xpos = user_pose_to_xpos([0., 0., 0.], delta_rot_deg)
        target_xpos = deepcopy(cur_xpos)
        if along_base_frame:
            target_xpos[:3,:3] = delta_xpos[:3,:3] @ cur_xpos[:3,:3]
        else:
            target_xpos[:3,:3] = cur_xpos[:3,:3] @ delta_xpos[:3,:3]
        
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
                # torso_tf, head_tf, arml_tf, armr_tf = self._pin_kine.get_Kinematics(real_qpos=real_qpos)
                # self.fcl_checker.set_head_body_transform(head_transforms=head_tf, body_transforms=torso_tf)
                # self.fcl_checker.update_arm_transform(arml_tf+armr_tf)
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
                # torso_tf, head_tf, arml_tf, armr_tf = self._pin_kine.get_Kinematics(real_qpos=real_qpos)
                # self.fcl_checker.set_head_body_transform(head_transforms=head_tf, body_transforms=torso_tf)
                # self.fcl_checker.update_arm_transform(arml_tf+armr_tf)
                if self.check_collision(real_qpos):
                    self.logger.waring("It will occur collision for this head moving trajectory")
                    return False
        
        res_align, move_torso_head_traj = self._qpos_align(torso_qpos_array, head_qpos_array, cur_arml_qpos, cur_armr_qpos)
        self.execute_joint_traj(qpos_array=move_torso_head_traj,
                                left_arm_ee_trigger_list=None,
                                right_arm_ee_trigger_list=None
                                )
    
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


        left_ref_psi = arml_kine._calc_arm_angle(cur_arml_qpos)[1]
        left_qpos_seed = cur_arml_qpos

        if target_arml_pose is not None and target_arml_pose != []:
            target_arml_xpos = user_pose_to_xpos(target_arml_pose[0][:3], target_arml_pose[0][3:])

            while True:
                res, arml_ik_qpos = arml_kine.get_ik(xpos=target_arml_xpos, arm_angle=left_ref_psi, config=np.sign(left_qpos_seed[[1,3,5]]))
                if not res:
                    left_ref_psi += 5./180.*np.pi
                    continue
                else:
                    real_qpos = np.concatenate((cur_torso_qpos, cur_head_qpos, arml_ik_qpos, cur_armr_qpos))
                    # torso_tf, head_tf, arml_tf, armr_tf = self._pin_kine.get_Kinematics(real_qpos=real_qpos)
                    # self.fcl_checker.set_head_body_transform(head_transforms=head_tf, body_transforms=torso_tf)
                    # self.fcl_checker.update_arm_transform(arml_tf+armr_tf)
                    if self.check_collision(real_qpos):
                        self.logger.warning("It will occur collision for this left arm ik solution, try to find another one")
                        left_ref_psi += 5./180.*np.pi
                        continue
                    else:
                        target_arml_qpos = arml_ik_qpos
                        self.logger.info(f"Successfully solve ik for left arm with qpos: {target_arml_qpos}")
                        break
            # res, arml_ik_qpos = arml_kine.get_ik(xpos=target_arml_xpos, arm_angle=left_ref_psi, config=np.sign(left_qpos_seed[[1,3,5]]))
            # if not res:
            #     print("Failed to solve ik for left arm before planning trajectory")
            #     return False
            # else:
            #     while True:
            #         real_qpos = np.concatenate((cur_torso_qpos, cur_head_qpos, arml_ik_qpos, cur_armr_qpos))
            #         torso_tf, head_tf, arml_tf, armr_tf = self._pin_kine.get_Kinematics(real_qpos=real_qpos)
            #         self.fcl_checker.set_head_body_transform(head_transforms=head_tf, body_transforms=torso_tf)
            #         self.fcl_checker.update_arm_transform(arml_tf+armr_tf)
            #         if self.fcl_checker.check_collision():
            #             print("It will occur collision for this left arm ik solution, try to find another one")
            #             left_ref_psi += 5./180.*np.pi
            #             res, arml_ik_qpos = arml_kine.get_ik(xpos=target_arml_xpos, arm_angle=left_ref_psi, config=np.sign(left_qpos_seed[[1,3,5]]))
            #             if not res:
            #                 print("Failed to solve ik for left arm before planning trajectory")
            #                 return False
            #         else:
            #             target_arml_qpos = arml_ik_qpos
            #             print(f"Successfully solve ik for left arm with qpos: {target_arml_pose}")
            #             break
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

        right_ref_psi = armr_kine._calc_arm_angle(cur_armr_qpos)[1]
        right_qpos_seed = cur_armr_qpos

        if target_armr_pose is not None and target_armr_pose != []:
            target_armr_xpos = user_pose_to_xpos(target_armr_pose[0][:3], target_armr_pose[0][3:])
            while True:
                res, armr_ik_qpos = armr_kine.get_ik(xpos=target_armr_xpos, arm_angle=right_ref_psi, config=np.sign(right_qpos_seed[[1,3,5]]))
                if not res:
                    right_ref_psi += 5./180.*np.pi
                    continue
                else:
                    real_qpos = np.concatenate((cur_torso_qpos, cur_head_qpos, cur_arml_qpos, armr_ik_qpos))
                    # torso_tf, head_tf, arml_tf, armr_tf = self._pin_kine.get_Kinematics(real_qpos=real_qpos)
                    # self.fcl_checker.set_head_body_transform(head_transforms=head_tf, body_transforms=torso_tf)
                    # self.fcl_checker.update_arm_transform(arml_tf+armr_tf)
                    if self.check_collision(real_qpos):
                        self.logger.warning("It will occur collision for this right arm ik solution, try to find another one")
                        right_ref_psi += 5./180.*np.pi
                        continue
                    else:
                        target_armr_qpos = armr_ik_qpos
                        self.logger.info(f"Successfully solve ik for right arm with qpos: {target_armr_qpos}")
                        break
            # res, armr_ik_qpos = armr_kine.get_ik(xpos=target_armr_xpos, arm_angle=right_ref_psi, config=np.sign(right_qpos_seed[[1,3,5]]))
            # if not res:
            #     print("Failed to solve ik for right arm before planning trajectory")
            #     return False
            # else:
            #     while True:
            #         real_qpos = np.concatenate((cur_torso_qpos, cur_head_qpos, cur_arml_qpos, armr_ik_qpos))
            #         torso_tf, head_tf, arml_tf, armr_tf = self._pin_kine.get_Kinematics(real_qpos=real_qpos)
            #         self.fcl_checker.set_head_body_transform(head_transforms=head_tf, body_transforms=torso_tf)
            #         self.fcl_checker.update_arm_transform(arml_tf+armr_tf)
            #         if self.fcl_checker.check_collision():
            #             print("It will occur collision for this right arm ik solution, try to find another one")
            #             right_ref_psi += 5./180.*np.pi
            #             res, armr_ik_qpos = armr_kine.get_ik(xpos=target_armr_xpos, arm_angle=right_ref_psi, config=np.sign(right_qpos_seed[[1,3,5]]))
            #             if not res:
            #                 print("Failed to solve ik for right arm before planning trajectory")
            #                 return False
            #         else:
            #             target_armr_qpos = armr_ik_qpos
            #             print(f"Successfully solve ik for right arm with qpos: {target_armr_qpos}")
            #             break
        else:
            target_armr_qpos = cur_armr_qpos

        self.siso_planner.set_head_torso(head_qpos=cur_head_qpos, torso_qpos=cur_torso_qpos)
        self.siso_planner.set_start_goal(arml_start=cur_arml_qpos, armr_start=cur_armr_qpos,
                                         arml_goal=target_arml_qpos, armr_goal=target_armr_qpos)
        self.siso_planner.set_traj_max_vel(left_max_vel=arml_speed_ratio*0.3, right_max_vel=armr_speed_ratio*0.3)
        arml_traj, armr_traj, planning_time = self.siso_planner.plan()

        self.logger.info(f"Finish Planning, time consumption: {planning_time}, with left arm traj shape: {None if arml_traj is None else arml_traj.shape}, right arm traj shape: {None if armr_traj is None else armr_traj.shape}")
        # input("Press any key to continue...")
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
            self.execute_joint_traj(qpos_array=move_arml_armr_traj,
                                    left_arm_ee_trigger_list=armlee_trigger,
                                    right_arm_ee_trigger_list=armree_trigger)
            # self.logger.info(armlee_trigger, armree_trigger)
            return
        
        if arml_traj is not None and armr_traj is None:
            if lee_target is not None:
                armlee_trigger.append((arml_traj.shape[0]+arml_pre_insert, lee_target))
            else:
                armlee_trigger = None
            if ree_target is not None:
                armree_trigger.append((armr_pre_insert, ree_target))
            else: 
                armree_trigger = None 

            arml_traj = np.vstack([arml_traj, np.tile(arml_traj[-1, :], (int(arml_pre_insert+arml_post_insert), 1))])

            res_align, move_arml_armr_traj = self._qpos_align(cur_torso_qpos, cur_head_qpos, arml_traj, cur_armr_qpos)
            self.execute_joint_traj(qpos_array=move_arml_armr_traj,
                                    left_arm_ee_trigger_list=armlee_trigger,
                                    right_arm_ee_trigger_list=armree_trigger)
            self.logger.info(armlee_trigger, armree_trigger)
            return
        
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
            self.execute_joint_traj(qpos_array=move_arml_armr_traj,
                                    left_arm_ee_trigger_list=armlee_trigger,
                                    right_arm_ee_trigger_list=armree_trigger)
            # self.logger.info(armlee_trigger, armree_trigger)
            return
        
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
            self.execute_joint_traj(qpos_array=move_arml_armr_traj,
                                    left_arm_ee_trigger_list=armlee_trigger,
                                    right_arm_ee_trigger_list=armree_trigger)
            # self.logger.info(armlee_trigger, armree_trigger)
            return
        
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
                # torso_tf, head_tf, arml_tf, armr_tf = self._pin_kine.get_Kinematics(real_qpos=real_qpos)
                # self.fcl_checker.set_head_body_transform(head_transforms=head_tf, body_transforms=torso_tf)
                # self.fcl_checker.update_arm_transform(arml_tf+armr_tf)
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
                # torso_tf, head_tf, arml_tf, armr_tf = self._pin_kine.get_Kinematics(real_qpos=real_qpos)
                # self.fcl_checker.set_head_body_transform(head_transforms=head_tf, body_transforms=torso_tf)
                # self.fcl_checker.update_arm_transform(arml_tf+armr_tf)
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
                left_ref_psi = arml_kine._calc_arm_angle(cur_arml_qpos)[1]
                left_qpos_seed = cur_arml_qpos
                if target_arml_pose is not None and target_arml_pose != []:
                    target_arml_xpos = user_pose_to_xpos(target_arml_pose[:3], target_arml_pose[3:])
                    while True:
                        res_ik, arml_ik_qpos = arml_kine.get_ik(xpos=target_arml_xpos, arm_angle=left_ref_psi, config=np.sign(left_qpos_seed[[1,3,5]]))
                        if not res_ik:
                            left_ref_psi += 5./180.*np.pi
                            continue
                        else:
                            real_qpos = np.concatenate((cur_torso_qpos, cur_head_qpos, arml_ik_qpos, cur_armr_qpos))
                            # torso_tf, head_tf, arml_tf, armr_tf = self._pin_kine.get_Kinematics(real_qpos=real_qpos)
                            # self.fcl_checker.set_head_body_transform(head_transforms=head_tf, body_transforms=torso_tf)
                            # self.fcl_checker.update_arm_transform(arml_tf+armr_tf)
                            if self.check_collision(real_qpos):
                                self.logger.warning("It will occur collision for this left arm ik solution, try to find another one")
                                left_ref_psi += 5./180.*np.pi
                                continue
                            else:
                                target_arml_qpos = arml_ik_qpos
                                self.logger.info(f"Successfully solve ik for left arm with qpos: {target_arml_qpos}")
                                break
                    # res_ik, arml_ik_qpos = arml_kine.get_ik(xpos=target_arml_xpos, arm_angle=left_ref_psi, config=np.sign(left_qpos_seed[[1,3,5]]))
                    # if not res_ik:
                    #     print("Failed to solve ik for left arm before planning trajectory")
                    #     return False
                    # else:
                    #     while True:
                    #         real_qpos = np.concatenate((cur_torso_qpos, cur_head_qpos, arml_ik_qpos, cur_armr_qpos))
                    #         torso_tf, head_tf, arml_tf, armr_tf = self._pin_kine.get_Kinematics(real_qpos=real_qpos)
                    #         self.fcl_checker.set_head_body_transform(head_transforms=head_tf, body_transforms=torso_tf)
                    #         self.fcl_checker.update_arm_transform(arml_tf+armr_tf)
                    #         if self.fcl_checker.check_collision():
                    #             print("It will occur collision for this left arm ik solution, try to find another one")
                    #             left_ref_psi += 5./180.*np.pi
                    #             res_ik, arml_ik_qpos = arml_kine.get_ik(xpos=target_arml_xpos, arm_angle=left_ref_psi, config=np.sign(left_qpos_seed[[1,3,5]]))
                    #             if not res_ik:
                    #                 print("Failed to solve ik for left arm before planning trajectory")
                    #                 return False
                    #         else:
                    #             target_arml_qpos = arml_ik_qpos
                    #             print(f"Successfully solve ik for left arm with qpos: {target_arml_qpos}")
                    #             break
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

                right_ref_psi = armr_kine._calc_arm_angle(cur_armr_qpos)[1]
                right_qpos_seed = cur_armr_qpos
                if target_armr_pose is not None and target_armr_pose != []:
                    target_armr_xpos = user_pose_to_xpos(target_armr_pose[:3], target_armr_pose[3:])
                    while True:
                        res_ik, armr_ik_qpos = armr_kine.get_ik(xpos=target_armr_xpos, arm_angle=right_ref_psi, config=np.sign(right_qpos_seed[[1,3,5]]))
                        if not res_ik:
                            right_ref_psi += 5./180.*np.pi
                            continue
                        else:
                            real_qpos = np.concatenate((cur_torso_qpos, cur_head_qpos, cur_arml_qpos, armr_ik_qpos))
                            # torso_tf, head_tf, arml_tf, armr_tf = self._pin_kine.get_Kinematics(real_qpos=real_qpos)
                            # self.fcl_checker.set_head_body_transform(head_transforms=head_tf, body_transforms=torso_tf)
                            # self.fcl_checker.update_arm_transform(arml_tf+armr_tf)
                            if self.check_collision(real_qpos):
                                self.logger.warning("It will occur collision for this right arm ik solution, try to find another one")
                                right_ref_psi += 5./180.*np.pi
                                continue
                            else:
                                target_armr_qpos = armr_ik_qpos
                                self.logger.info(f"Successfully solve ik for right arm with qpos: {target_armr_qpos}")
                                break
                    # res_ik, armr_ik_qpos = armr_kine.get_ik(xpos=target_armr_xpos, arm_angle=right_ref_psi, config=np.sign(right_qpos_seed[[1,3,5]]))
                    # if not res_ik:
                    #     print("Failed to solve ik for right arm before planning trajectory")
                    #     return False
                    # else:
                    #     while True:
                    #         real_qpos = np.concatenate((cur_torso_qpos, cur_head_qpos, cur_arml_qpos, armr_ik_qpos))
                    #         torso_tf, head_tf, arml_tf, armr_tf = self._pin_kine.get_Kinematics(real_qpos=real_qpos)
                    #         self.fcl_checker.set_head_body_transform(head_transforms=head_tf, body_transforms=torso_tf)
                    #         self.fcl_checker.update_arm_transform(arml_tf+armr_tf)
                    #         if self.fcl_checker.check_collision():
                    #             print("It will occur collision for this right arm ik solution, try to find another one")
                    #             right_ref_psi += 5./180.*np.pi
                    #             res_ik, armr_ik_qpos = armr_kine.get_ik(xpos=target_armr_xpos, arm_angle=right_ref_psi, config=np.sign(right_qpos_seed[[1,3,5]]))
                    #             if not res_ik:
                    #                 print("Failed to solve ik for right arm before planning trajectory")
                    #                 return False
                    #         else:
                    #             target_armr_qpos = armr_ik_qpos
                    #             print(f"Successfully solve ik for right arm with qpos: {target_armr_qpos}")
                    #             break
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
                self.execute_joint_traj(qpos_array=move_arml_armr_traj,
                                        left_arm_ee_trigger_list=None,
                                        right_arm_ee_trigger_list=armr_trigger)


    ######## Dynamic Planner functions ########
    def init_dynamic_planner(self,
                             robot_type: RobotType = None,
                             end_effector_type: EndEffectorType = None,
                             cam_arml_extrinsics: np.ndarray = None
                             ):
        if cam_arml_extrinsics is None or cam_arml_extrinsics.shape != (4,4):
            raise ValueError("Invalid camera to left arm extrinsics input, it should be a 4x4 numpy array")
        self.cam_arml_extrinsics = cam_arml_extrinsics

        self._dynamics_planner_executor = MultiThreadedExecutor()
        
        # just for update dexsim server environment
        self._dexsim_pcd_node = Node("dexsim_pcd_node")
        self.pcd_env_publisher = self._dexsim_pcd_node.create_publisher(
            String, "/dexe_robot_client/dexsim_server/update_env", 10
        )
        self._dynamics_planner_executor.add_node(self._dexsim_pcd_node)

        self._init_camera()
        self._init_dynamic_ros_thread()
        self._check_connection()

        self.vamp_planner = VampPlanner(robot_type=robot_type, end_effector_type=end_effector_type)
        self.start_qpos, self.target_qpos = None, None

        self.last_velocity = None
        self.last_traj = None
        self.is_first_plan = True

    def _init_camera(self):
        if not rclpy.ok():
            rclpy.init()
        self.camera_node = OrbbecClient()
        self._dynamics_planner_executor.add_node(self.camera_node)

    def _init_dynamic_ros_thread(self):
        if not rclpy.ok():
            rclpy.init()
        
        self.ros_thread = threading.Thread(target=self._dynamics_planner_executor.spin, daemon=True)
        self.ros_thread.start()

    def _check_connection(self):
        if not self.camera_node.is_connected():
            time.sleep(0.5)

    def set_start_target_qpos(self, start_qpos: np.ndarray, target_qpos: np.ndarray):
        if target_qpos is None or target_qpos.shape[0] != SINGLE_ARM_DIM:
            self.logger.error("Invalid target_qpos shape")
            return False
        if start_qpos is None or start_qpos.shape[0] != SINGLE_ARM_DIM:
            self.logger.error("Invalid start_qpos shape")
            return False
        self.start_qpos = start_qpos
        self.target_qpos = target_qpos
        return True

    def _get_base_link_T_arm_base(self):
        current_qpos = self.get_current_qpos()[1]
        cur_arml_base_tf, cur_armr_base_tf = self._pin_kine.get_arm_base_tf(real_qpos=current_qpos)
        return True, cur_arml_base_tf, cur_armr_base_tf
    
    def _get_waist_T_arm_base(self):
        # FIXME: Since current single arm urdf is based on waist joint, we need transfer the point cloud to waist frame
        current_qpos = self.get_current_qpos()[1]
        torso_tf, _, _, _ = self._pin_kine.get_Kinematics(real_qpos=current_qpos)
        cur_arml_base_tf, cur_armr_base_tf = self._pin_kine.get_arm_base_tf(real_qpos=current_qpos)
        base_link_T_waist = torso_tf[-1]
        waist_T_arml_base = np.linalg.inv(base_link_T_waist) @ cur_arml_base_tf
        waist_T_armr_base = np.linalg.inv(base_link_T_waist) @ cur_armr_base_tf
        return True, waist_T_arml_base, waist_T_armr_base
    
    def _get_current_arm_qpos(self):
        current_qpos = self.get_current_qpos()[1]
        cur_arml_qpos = current_qpos[self.real_arml_idx[0]:self.real_arml_idx[1]+1]
        cur_armr_qpos = current_qpos[self.real_armr_idx[0]:self.real_armr_idx[1]+1]
        return True, cur_arml_qpos, cur_armr_qpos
    
    def _get_current_full_qpos(self):
        current_qpos = self.get_current_qpos()[1]
        cur_torso_qpos = current_qpos[self.real_torso_idx[0]:self.real_torso_idx[1]+1]
        cur_head_qpos = current_qpos[self.real_head_idx[0]:self.real_head_idx[1]+1]
        cur_arml_qpos = current_qpos[self.real_arml_idx[0]:self.real_arml_idx[1]+1]
        cur_armr_qpos = current_qpos[self.real_armr_idx[0]:self.real_armr_idx[1]+1]
        return True, cur_torso_qpos, cur_head_qpos, cur_arml_qpos, cur_armr_qpos
    
    def _get_qpos_from_topic(self):
        current_pub_qpos, current_pub_vel = self.planner_client.get_streaming_cmd()
        if current_pub_qpos is None:
            self.logger.warning("No current qpos data from topic")
            return True, self._get_current_arm_qpos()[1], np.zeros(7)
        current_arml_qpos = np.array(current_pub_qpos[self.real_arml_idx[0]:self.real_arml_idx[1]+1])
        current_arml_vel = np.array(current_pub_vel[self.real_arml_idx[0]:self.real_arml_idx[1]+1])
        return True, current_arml_qpos, current_arml_vel
    
    def _voxel_down_sample_numpy(self, points, voxel_size):
        coords = np.floor(points / voxel_size).astype(np.int32)
        _, idx = np.unique(coords, axis=0, return_index=True)
        return points[idx]

    def _reload_environment(self):
        current_pcd = self.camera_node.get_pcd_numpy()[1]
        # start_time = time.time()
        if current_pcd is None or current_pcd.shape[0] == 0:
            self.logger.warning("No point cloud data from camera")
            return False, None
        ## This is for dexsim instance environment update
        # if self.dexsim_env is not None and self.dexsim_articulation is not None:
        #     if self.dexsim_pcd_handle is not None:
        #         self.dexsim_env.remove_point_cloud(self.dexsim_pcd_handle)
        #         self.dexsim_pcd_handle = None
        #     vis_pcd = o3d.geometry.PointCloud()
        #     vis_pcd.points = o3d.utility.Vector3dVector(current_pcd)
        #     o3d_point_cloud = o3d_point_cloud.voxel_down_sample(voxel_size=0.01)
        #     res, base_link_T_arml_base, base_link_T_armr_base = self._get_base_link_T_arm_base()
        #     if not res:
        #         return False, None
        #     # choose one arm to transform the point cloud to arm base frame
        #     # TODO: check if base_link_T_camera got from left arm or right arm are same
        #     base_link_T_camera = base_link_T_arml_base @ self.cam_arml_extrinsics
        #     vis_pcd.transform(base_link_T_camera)
        #     # FIXME: Modify the path
        #     saved_pcd_path = "/home/young/workspace/dexe_robot_client/examples/output/dexsim_vis_pcd.ply"
        #     o3d.io.write_point_cloud(saved_pcd_path, vis_pcd)
        #     self.dexsim_pcd_handle = self.dexsim_env.load_point_cloud(saved_pcd_path)
        #     self.pcd_env_publisher.publish(String(data="update"))

        # vis_pcd = o3d.geometry.PointCloud()
        # vis_pcd.points = o3d.utility.Vector3dVector(current_pcd)
        # vis_pcd = vis_pcd.voxel_down_sample(voxel_size=0.01)

        # vis_pcd = deepcopy(current_pcd)
        # vis_pcd = self._voxel_down_sample_numpy(vis_pcd, voxel_size=0.01)

        # res, base_link_T_arml_base, base_link_T_armr_base = self._get_base_link_T_arm_base()
        # if not res:
        #     return False, None
        # # choose one arm to transform the point cloud to arm base frame
        # # TODO: check if base_link_T_camera got from left arm or right arm are same
        # base_link_T_camera = base_link_T_arml_base @ self.cam_arml_extrinsics

        # vis_pcd_h = np.hstack([vis_pcd, np.ones((vis_pcd.shape[0], 1))])
        # vis_pcd = (base_link_T_camera @ vis_pcd_h.T).T[:, :3]
        # # print(f"process pcd time: {time.time()-start_time}")
        # # vis_pcd.transform(base_link_T_camera)
        # # FIXME: Modify the path
        # saved_pcd = o3d.geometry.PointCloud()
        # saved_pcd.points = o3d.utility.Vector3dVector(vis_pcd)
        # saved_pcd_path = "/home/young/workspace/dexe_robot_client/examples/output/dexsim_vis_pcd.ply"
        # o3d.io.write_point_cloud(saved_pcd_path, saved_pcd)
        # # print(f"save pcd time: {time.time()-start_time}")
        # # self.dexsim_pcd_handle = self.dexsim_env.load_point_cloud(saved_pcd_path)
        # self.pcd_env_publisher.publish(String(data="update"))


        # planner_pcd = o3d.geometry.PointCloud()
        # planner_pcd.points = o3d.utility.Vector3dVector(current_pcd)
        # planner_pcd = planner_pcd.voxel_down_sample(voxel_size=0.05)

        planner_pcd = deepcopy(current_pcd)
        planner_pcd = self._voxel_down_sample_numpy(planner_pcd, voxel_size=0.05)

        res, waist_T_arml_base, waist_T_armr_base = self._get_waist_T_arm_base()
        if not res:
            return False, None
        # choose one arm to transform the point cloud to waist frame
        # planner_pcd.transform(waist_T_arml_base @ self.cam_arml_extrinsics)

        planner_pcd_h = np.hstack([planner_pcd, np.ones((planner_pcd.shape[0], 1))])
        planner_pcd = (waist_T_arml_base @ self.cam_arml_extrinsics @ planner_pcd_h.T).T[:, :3]

        return True, planner_pcd
    
    def _check_start_goal_validity(self):
        # res, cur_arml_qpos, cur_armr_qpos = self._get_current_arm_qpos()
        res, cur_arml_qpos, cur_arml_vel = self._get_qpos_from_topic()
        if not res:
            self.logger.warning("Get current qpos failed during checking start and goal validity")
            return False
        if self.vamp_planner.vamp_module.validate(cur_arml_qpos.tolist()) \
            and self.vamp_planner.vamp_module.validate(self.target_qpos.tolist()):
            return True
        else:
            self.logger.warning("Start or goal qpos is invalid for some reason")
            return False
        
    def _gen_trajectory_with_toppra(self, path, max_v = 0.1, last_velocity = np.zeros(SINGLE_ARM_DIM)):
        dofs = SINGLE_ARM_DIM
        vel_limits = [max_v] * dofs
        acc_limits = [0.5] * dofs
        traj_planner = ToppraTrajectoryPlanner(dofs, vel_limits, acc_limits)
        traj_planner.set_last_joint_vel(last_velocity)
        res, pos_array, vel_array, acc_array, duration = traj_planner.plan_with_last_vel(path)
        if not res:
            res, pos_array, vel_array, acc_array, duration = traj_planner.plan(path)
        return pos_array, vel_array, acc_array, duration

    def plan(self, update_frequency: float = 5.0):
        # TODO: currently use time.sleep to simulate the update frequency, need to be improved
        # Try to figure out apscheduler method or ROS timer method or other methods
        # TODO: currently only support left arm planning
        if self.target_qpos is None:
            self.logger.error("Target qpos not set, please set it using set_target_qpos function")
            return False, None
        is_first_plan = True
        planning_time = 0
        while True:
            time.sleep((1.0 / update_frequency)-planning_time)
            planning_start_time = time.time()
            res, cur_arml_qpos, cur_armr_qpos = self._get_current_arm_qpos()
            res, planner_pcd = self._reload_environment()
            # print("reload env time:", time.time()-planning_start_time)
            planner_pcd = self.vamp_planner.vamp_module.filter_self_from_pointcloud(planner_pcd,
                                                                                    0.0025,
                                                                                    cur_arml_qpos,
                                                                                    self.vamp_planner.env)
            # print(f"filter self time {time.time()-planning_start_time}")
            if not res:
                self.logger.warning("Reload environment failed")
                continue
            self.vamp_planner.reset_env()
            # self.vamp_planner.add_pcd(point_cloud=np.asarray(planner_pcd.points).tolist())
            self.vamp_planner.add_pcd(point_cloud=planner_pcd)

            if not self._check_start_goal_validity():
                continue
            # res, cur_arml_qpos, cur_armr_qpos = self._get_current_arm_qpos()
            res, cur_arml_qpos, cur_arml_vel = self._get_qpos_from_topic()
            start_time = time.time()
            # self.logger.debug(f"current arml qpos: {cur_arml_qpos}, current arml vel: {cur_arml_vel}")
            # self.logger.debug(f"target arml qpos: {self.target_qpos}")
            if not res:
                self.logger.warning("Get current qpos failed")
                continue
            if np.allclose(cur_arml_qpos, self.target_qpos, rtol=1e-03, atol=1e-03): # FIXME: need to tune the threshold
                self.logger.info("Current qpos is close to target qpos, no need to plan")
                break

            if is_first_plan:
                is_first_plan = False
                self.vamp_planner.set_start_goal(start=self.start_qpos, goal=self.target_qpos)
            else:
                self.vamp_planner.set_start_goal(start=cur_arml_qpos, goal=self.target_qpos)
            
            # TODO: change to dual arm planning with result is 14 dim
            result = self.vamp_planner.plan()
            
            if result is not None:
                # if self.is_first_plan:
                #     self.is_first_plan = False
                #     self.last_velocity = np.zeros(SINGLE_ARM_DIM)
                # else:
                #     stop_idx = int((1.0/update_frequency) / 0.01)
                #     if self.last_traj is not None:
                #         self.last_velocity = (self.last_traj[stop_idx] - self.last_traj[stop_idx-1]) / 0.01
                #     else:
                #         self.last_velocity = np.zeros(SINGLE_ARM_DIM)
                self.last_velocity = cur_arml_vel
                traj_result = self._gen_trajectory_with_toppra(result, max_v=0.3, last_velocity=self.last_velocity)[0]
                self.last_traj = traj_result

                res, cur_arml_qpos, cur_arml_vel = self._get_qpos_from_topic()
                idx = np.argmin(np.linalg.norm(traj_result - cur_arml_qpos, axis=1))
                traj_result = traj_result[idx+6:]
                # publish the result trajectory
                full_traj = np.zeros((traj_result.shape[0], 20))
                res, cur_torso_qpos, cur_head_qpos, cur_arml_qpos, cur_armr_qpos = self._get_current_full_qpos()
                for i in range(traj_result.shape[0]):
                    full_qpos = np.concatenate([cur_torso_qpos, cur_head_qpos, traj_result[i], cur_armr_qpos])
                    full_traj[i] = full_qpos

                self.planner_client.publish_trajectory(trajectory=full_traj)
                self.logger.info(f"Planning time: {time.time() - start_time}s")
                self.logger.info(f"current publish time: {time.time()}")
                self.logger.info("Planning succeeded, publishing trajectory")

                planning_time = time.time() - planning_start_time
                # self.logger.info(f"planning time: {planning_time}s")
            else:
                self.logger.warning("Planning failed, no trajectory to publish")
                planning_time = time.time() - planning_start_time

        self.planner_client.publish_trajectory(trajectory=None)

    def start_moving(self):
        req = ClientService.Request()
        req.request = json.dumps({})
        future = self.start_moving_client.call_async(req)
        return future