# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

from ..common import time_consuming, get_logger, DATA_DIR, W1ArmType, W1Version, generate_w1_urdf
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

# for rviz point cloud visualization
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

import numpy as np
import open3d as o3d
from sklearn.neighbors import NearestNeighbors, KDTree
from sklearn.cluster import DBSCAN
import threading
import time
import pathlib, json
from copy import deepcopy
import atexit
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from collections import deque
import trimesh
from scipy.spatial.transform import Rotation as R
import vamp
from visualization_msgs.msg import MarkerArray, Marker

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
        
        # # for visual
        # if not generate_w1_urdf(arm_type, w1_version, end_effector_type)[1]:
        #     raise ValueError(f"Found URDF failed.")

        # # for collision check
        # if self._end_effector_type is not None:
        #     if not generate_w1_urdf(arm_type, w1_version, None)[1]:
        #         raise ValueError(f"Found URDF failed.")

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

        self.set_ee_pose_with_time_client = self.service_node.create_client(ClientService,
                                                                            "set_ee_pose_with_time")
        while not self.set_ee_pose_with_time_client.wait_for_service(timeout_sec=1.0):
            self.logger.warning("set_ee_pose_with_time service not available, waiting again...")


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
            # SinglePlanner.set_env_json(get_env_json_path())
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
    
    def set_ee_pose_with_time(self, name: str, ee_task: list) -> bool:
        """
        Control end effector movement with pre-waited time and post-waited time

        Args:
            name (str): "left_arm_ee" or "right_arm_ee"
            ee_task (list): [pre_time: float, ee_pose: list (6,), post_time: float]
        """
        if not self._check_qpos_dimension(name, np.array(ee_task[1])):
            return False
        req = ClientService.Request()
        req.request = json.dumps({"name": name, "ee_task": ee_task})
        future = self.set_ee_pose_with_time_client.call_async(req)
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
        """设置left_arm/right_arm的tcp

        Args:
            name (str): ["left_arm", "right_arm"]
            xpos (np.ndarray): 目标tcp

        Returns:
            bool: 
        """
        # TODO: set tcp in robot client too
        arm_kine = self._get_arm_kine(name)
        if arm_kine is None:
            return False
        arm_kine.set_tcp(xpos=xpos)
        return True
    
    def get_tcp(self, name: str) -> tuple[bool, np.ndarray]:
        """获取left_arm/right_arm的tcp

        Args:
            name (str): ["left_arm", "right_arm"]
        """
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
        """从当前位置直线移动相对距离

        Args:
            name (str): ["left_arm", "right_arm"]
            delta_trans_mm (list): 直线相对运动距离
            speed_ratio (float, optional): 运动速率. Defaults to 1..
            along_base_frame (bool, optional): 参考机械臂base坐标系/tcp坐标系. Defaults to True.

        Returns:
            bool: 
        """
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
        """从当前位置旋转相对距离

        Args:
            name (str): ["left_arm", "right_arm"]
            delta_rot_deg (list): 旋转相对运动距离
            speed_ratio (float, optional): 运动速率. Defaults to 1..
            along_base_frame (bool, optional): 参考机械臂base坐标系/tcp坐标系. Defaults to True.

        Returns:
            bool: 
        """
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
                             robot_type: RobotType = RobotType.W1V02_1,
                             end_effector_type: EndEffectorType = None,
                             cam_arml_extrinsics: np.ndarray = None,
                             cam_armr_extrinsics: np.ndarray = None
                             ):
        # if cam_arml_extrinsics is None or cam_arml_extrinsics.shape != (4,4):
        #     raise ValueError("Invalid camera to left arm extrinsics input, it should be a 4x4 numpy array")
        # if cam_armr_extrinsics is None or cam_armr_extrinsics.shape != (4,4):
        #     raise ValueError("Invalid camera to right arm extrinsics input, it should be a 4x4 numpy array")

        self.cam_arml_extrinsics = cam_arml_extrinsics
        self.cam_armr_extrinsics = cam_armr_extrinsics

        self._dynamics_planner_executor = MultiThreadedExecutor()
        
        # This node is used to publish pcd to rviz(might for dexsim in the future)
        self._dexsim_pcd_node = Node("dexsim_pcd_node")
        self.pcd_env_publisher = self._dexsim_pcd_node.create_publisher(
            PointCloud2, "/dexe_robot_client/rviz2/point_cloud", 10
        )

        # This publisher is used to publish the torso sphere collision to rviz
        self.torso_collision_publisher = self._dexsim_pcd_node.create_publisher(
            MarkerArray, "dexe_robot_client/rviz2/torso_collision", 10
        )

        self._dynamics_planner_executor.add_node(self._dexsim_pcd_node)

        self._init_camera()
        self._init_dynamic_ros_thread()
        self._check_connection()

        self.arml_vamp_planner = VampPlanner(robot_type=robot_type, 
                                             end_effector_type=end_effector_type, 
                                             is_left_arm=True
                                             )
        self.armr_vamp_planner = VampPlanner(robot_type=robot_type, end_effector_type=end_effector_type, is_left_arm=False)
        self.dynamic_arml_start_qpos, self.dynamic_arml_target_qpos = None, None
        self.dynamic_armr_start_qpos, self.dynamic_armr_target_qpos = None, None

        self.arml_last_velocity, self.armr_last_velocity = None, None
        self.is_first_plan = True

        self.pcd_buffer = deque(maxlen=6)

        # self.add_torso_collision_to_env()
        self.first_init_torso_collision = True
        self.armlee_attached = False
        self.armree_attached = False

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
        while True:
            if not self.camera_node.is_connected():
                time.sleep(0.5)
                self.logger.warning("Waiting for camera connection...")
            else:
                self.logger.info("Camera connected!")
                break

    def set_dynamic_start_target_qpos(self, 
                                      left_start_qpos: np.ndarray, left_target_qpos: np.ndarray,
                                      right_start_qpos: np.ndarray, right_target_qpos: np.ndarray):
        if left_start_qpos is None or np.atleast_2d(left_start_qpos).shape[1] != SINGLE_ARM_DIM:
            self.logger.error("Invalid left_start_qpos shape")
            return False
        if left_target_qpos is None or np.atleast_2d(left_target_qpos).shape[1] != SINGLE_ARM_DIM:
            self.logger.error("Invalid left_target_qpos shape")
            return False
        if right_start_qpos is None or np.atleast_2d(right_start_qpos).shape[1] != SINGLE_ARM_DIM:
            self.logger.error("Invalid right_start_qpos shape")
            return False
        if right_target_qpos is None or np.atleast_2d(right_target_qpos).shape[1] != SINGLE_ARM_DIM:
            self.logger.error("Invalid right_target_qpos shape")
            return False
        self.dynamic_arml_start_qpos, self.dynamic_arml_target_qpos = left_start_qpos, left_target_qpos
        self.dynamic_armr_start_qpos, self.dynamic_armr_target_qpos = right_start_qpos, right_target_qpos
        return True

    def set_cam_neck2_extrinsics(self, cam_neck2_extrinsics):
        self.cam_neck2_extrinsics = cam_neck2_extrinsics

    def set_cam_base_link_extrinsics(self, cam_base_link_extrinsics):
        self.cam_base_link_extrinsics = cam_base_link_extrinsics

    def _get_base_link_T_arm_base(self):
        current_qpos = self.get_current_qpos()[1]
        cur_arml_base_tf, cur_armr_base_tf = self._pin_kine.get_arm_base_tf(real_qpos=current_qpos)
        return True, cur_arml_base_tf, cur_armr_base_tf
    
    def _get_base_link_T_neck2(self):
        current_qpos = self.get_current_qpos()[1]

        current_arml_qpos = self._get_qpos_from_topic()[1]
        current_qpos[6:13] = current_arml_qpos

        torso_tf, head_tf, arml_tf, armr_tf = self._pin_kine.get_Kinematics(real_qpos=current_qpos)
        base_link_T_neck2 = head_tf[-1]
        return True, base_link_T_neck2

    def _get_waist_T_neck2(self):
        current_qpos = self.get_current_qpos()[1]

        current_arml_qpos = self._get_qpos_from_topic()[1]
        current_qpos[6:13] = current_arml_qpos

        torso_tf, head_tf, arml_tf, armr_tf = self._pin_kine.get_Kinematics(real_qpos=current_qpos)
        base_link_T_waist = torso_tf[-1]
        base_link_T_neck2 = head_tf[-1]
        waist_T_neck2 = np.linalg.inv(base_link_T_waist) @ base_link_T_neck2
        return True, waist_T_neck2

    def _get_waist_T_base_link(self):
        current_qpos = self.get_current_qpos()[1]
        torso_tf, head_tf, arml_tf, armr_tf = self._pin_kine.get_Kinematics(real_qpos=current_qpos)
        base_link_T_waist = torso_tf[-1]
        waist_T_base_link = np.linalg.inv(base_link_T_waist)
        return True, waist_T_base_link


    def _get_base_link_T_waist(self):
        current_qpos = self.get_current_qpos()[1]

        # current_arml_qpos = self._get_qpos_from_topic()[1]
        # current_qpos[6:13] = current_arml_qpos

        torso_tf, head_tf, arml_tf, armr_tf = self._pin_kine.get_Kinematics(real_qpos=current_qpos)
        base_link_T_waist = torso_tf[-1]
        return True, base_link_T_waist

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
        current_pub_qpos, current_pub_vel, get_streaming_time, current_streaming_history = self.planner_client.get_streaming_cmd()
        # self.logger.debug(f"Get qpos time stamp: {get_streaming_time:.6f}s")
        if current_pub_qpos is None:
            # self.logger.warning("No current qpos data from topic")
            res, current_arml_qpos, current_armr_qpos = self._get_current_arm_qpos()
            return True, current_arml_qpos, np.zeros(7), current_armr_qpos, np.zeros(7), None, None
        current_arml_qpos = np.array(current_pub_qpos[self.real_arml_idx[0]:self.real_arml_idx[1]+1])
        current_arml_vel = np.array(current_pub_vel[self.real_arml_idx[0]:self.real_arml_idx[1]+1])
        current_armr_qpos = np.array(current_pub_qpos[self.real_armr_idx[0]:self.real_armr_idx[1]+1])
        current_armr_vel = np.array(current_pub_vel[self.real_armr_idx[0]:self.real_armr_idx[1]+1])
        return True, current_arml_qpos, current_arml_vel, current_armr_qpos, current_armr_vel, get_streaming_time, current_streaming_history
    
    def _voxel_down_sample_numpy(self, points, voxel_size):
        coords = np.floor(points / voxel_size).astype(np.int64)
        # _, idx = np.unique(coords, axis=0, return_index=True)
        keys = (
        (coords[:, 0] << 42) +
        (coords[:, 1] << 21) +
        coords[:, 2]
        )
        _, idx = np.unique(keys, return_index=True)
        return points[idx]

    def augment_pointcloud(self, pointcloud: np.ndarray, step: float = 0.02, max_range: float = 0.2) -> np.ndarray:
        """
        Augment pointcloud by extending points along the ray from origin to each point
        
        Args:
            pointcloud (nd.array, shape=(N, 3)): original pointcloud
            step (float): step size for augmentation
            max_range (float): maximum range for augmentation

        """
        # start_time = time.time()
        # center = np.zeros((1, 3))
        # vectors = pointcloud - center
        # distances = np.linalg.norm(vectors, axis=1)
        # directions = vectors / (distances[:, np.newaxis] + 1e-8)

        # augmented_points = []
        # for i in range(pointcloud.shape[0]):
        #     num_steps = int((max_range) / step)
        #     for j in range(1, num_steps + 1):
        #         new_point = pointcloud[i] + directions[i] * step * j
        #         augmented_points.append(new_point)
        # if len(augmented_points) > 0:
        #     augmented_points = np.array(augmented_points)
        #     combined_pointcloud = np.vstack((pointcloud, augmented_points))
        # else:
        #     combined_pointcloud = pointcloud

        # self.logger.debug(f"Pointcloud augmentation time: {time.time()-start_time:.6f}s, original points: {pointcloud.shape[0]}, augmented points: {combined_pointcloud.shape[0]}")
        
        center = np.zeros((1, 3))
        vectors = pointcloud - center
        distances = np.linalg.norm(vectors, axis=1, keepdims=True)
        directions = vectors / (distances + 1e-8)

        num_steps = int(max_range / step)
        scales = step * np.arange(1, num_steps +1, dtype=vectors.dtype)
        new_points = vectors[:, None, :] + directions[:, None, :] * scales[None, :, None]
        new_points = new_points.reshape(-1, 3)
        combined_pointcloud = np.vstack((pointcloud, new_points))


        return combined_pointcloud

    def numpy_to_pointcloud2(self, points: np.ndarray, frame_id="base_link") -> PointCloud2:
        """
        numpy points: shape (N, 3), dtype float32
        """
        header = Header()
        header.stamp = rclpy.clock.Clock().now().to_msg()
        header.frame_id = frame_id

        return point_cloud2.create_cloud_xyz32(header, points.tolist())

    def radius_outlier_removal(self, points, radius=0.05, min_neighbors=5):
        """
        半径邻域去噪 (Radius Outlier Removal)
        :param points: numpy array (N,3)，点云数据
        :param radius: 半径范围，比如0.05米
        :param min_neighbors: 半径内至少有多少个邻居才能保留
        :return: 去噪后的点云 numpy (M,3)
        """
        if len(points) == 0:
            return points

        tree = KDTree(points)
        # 查询每个点在radius范围内的邻居数量
        ind = tree.query_radius(points, r=radius, count_only=False)

        mask = np.array([len(i) > min_neighbors for i in ind])
        filtered_points = points[mask]
        return filtered_points

    def _reload_environment(self):
        start_time = time.time()
        res, current_pcd, get_pcd_time = self.camera_node.get_pcd_numpy()
        self.time_dict["original pcd"] = current_pcd.shape[0] if current_pcd is not None else 0
        self.time_dict["reading pcd msg"] = time.time() - start_time
        # self.logger.debug(f"Get pcd time stamp: {get_pcd_time:.6f}s")
        if current_pcd is None or current_pcd.shape[0] == 0:
            self.logger.warning("No point cloud data from camera")
            return False, None, None
        start_time = time.time()
        mask = (
            (current_pcd[:,0] >= -2) & (current_pcd[:,0] <= 2) &
            (current_pcd[:,1] >= -2) & (current_pcd[:,1] <= 2) &
            (current_pcd[:,2] >= -0.5) & (current_pcd[:,2] <= 1)
        )
        current_pcd = current_pcd[mask]

        self.time_dict["ROI crop"] = time.time() - start_time
        start_time = time.time()
        planner_pcd = current_pcd
        planner_pcd = self._voxel_down_sample_numpy(planner_pcd, voxel_size=0.01)
        self.time_dict["downsample1"] = time.time() - start_time
        # self.logger.debug(f"downsampled time: {time.time() - start_time}")
        # planner_pcd = self.augment_pointcloud(planner_pcd, 0.02, 0.2)
        # planner_pcd = self._voxel_down_sample_numpy(planner_pcd, voxel_size=0.01)

        # res, waist_T_neck2 = self._get_waist_T_neck2()
        res, waist_T_base_link = self._get_waist_T_base_link()
        start_time = time.time()
        planner_pcd_h = np.hstack([planner_pcd, np.ones((planner_pcd.shape[0], 1))])
        # planner_pcd = (waist_T_neck2 @ self.cam_neck2_extrinsics @ planner_pcd_h.T).T[:, :3]
        planner_pcd = (waist_T_base_link @ self.cam_base_link_extrinsics @ planner_pcd_h.T).T[:, :3]

        self.time_dict["pose transform"] = time.time() - start_time
        
        # print(f"process planner pcd time: {time.time()-start_time}")

        # # Visulaize for the filtered pcd

        # current_qpos = self.get_current_qpos()[1]
        # cur_arml_qpos = current_qpos[self.real_arml_idx[0]:self.real_arml_idx[1]+1]

        # # self.arml_vamp_planner.reset_env()
        # # tf = np.eye(4, dtype=np.float32)
        # # tf[:3, 3] = np.array([0, 0, 0.2])
        # # attachment = vamp.Attachment(tf)
        # # attachment.add_spheres([vamp.Sphere([0, 0, 0], 0.2)])
        # # attachment.set_ee_pose(self.arml_vamp_planner.vamp_module.eefk(cur_arml_qpos))
        # # self.arml_vamp_planner.env.attach(attachment)

        # # self.arml_vamp_planner.env.set_attachment_pose(self.arml_vamp_planner.vamp_module.eefk(cur_arml_qpos))
        # # attachment.set_ee_pose(self.arml_vamp_planner.vamp_module.eefk(cur_arml_qpos))
        # # print(attachment.posed_spheres[0].position)
        # # print(self.arml_vamp_planner.vamp_module.eefk(cur_arml_qpos))
        # # torso_tf, head_tf, arml_tf, armr_tf = self._pin_kine.get_Kinematics(real_qpos=current_qpos)
        # # base_link_T_waist = torso_tf[-1]
        # # base_link_T_left_ee = arml_tf[-1]
        # # waist_T_left_ee = np.linalg.inv(base_link_T_waist) @ base_link_T_left_ee
        # # print(waist_T_left_ee)
        # # self.arml_vamp_planner.env.add_sphere(vamp.Sphere(waist_T_left_ee[:3, 3], 0.1))

        # # embed()
        

        # # self.arml_vamp_planner.env.add_sphere(vamp.Sphere([0,0,0.2], 0.5))

        # filtered_planner_pcd = self.arml_vamp_planner.vamp_module.filter_self_from_pointcloud(planner_pcd,
        #                                                                 0.02,
        #                                                                 cur_arml_qpos,
        #                                                                 self.arml_vamp_planner.env)

        # # filtered_planner_pcd = deepcopy(planner_pcd)
        # # filtered_planner_pcd = self.radius_outlier_removal(filtered_planner_pcd, radius=0.03, min_neighbors=15)

        # # self.logger.debug(f"planner pcd length: {planner_pcd.shape[0]} filtered pcd length: {len(filtered_planner_pcd)}")
        
        # rviz_planner_pcd = np.asarray(deepcopy(filtered_planner_pcd))
        # torso_tf, head_tf, arml_tf, armr_tf = self._pin_kine.get_Kinematics(real_qpos=current_qpos)
        # base_link_T_waist = torso_tf[-1]
        # rviz_pcd = np.hstack([rviz_planner_pcd, np.ones((rviz_planner_pcd.shape[0], 1))])
        # rviz_pcd = (base_link_T_waist @ rviz_pcd.T).T[:, :3]
        # rviz_pcd = self.numpy_to_pointcloud2(rviz_pcd, frame_id="base_link")
        # self.pcd_env_publisher.publish(rviz_pcd)


        return True, planner_pcd, get_pcd_time
    
    def _check_start_goal_validity(self):
        res, cur_arml_qpos, cur_arml_vel, cur_armr_qpos, cur_armr_vel, _, _ = self._get_qpos_from_topic()
        if not res:
            self.logger.warning("Get current qpos failed during checking start and goal validity")
            return False
        start_time = time.time()
        if self.arml_vamp_planner.vamp_module.validate(cur_arml_qpos.tolist(), self.arml_vamp_planner.env) \
            and self.arml_vamp_planner.vamp_module.validate(self.dynamic_arml_target_qpos.tolist(), self.arml_vamp_planner.env) \
            and self.armr_vamp_planner.vamp_module.validate(cur_armr_qpos.tolist(), self.armr_vamp_planner.env) \
            and self.armr_vamp_planner.vamp_module.validate(self.dynamic_armr_target_qpos.tolist(), self.arml_vamp_planner.env):
            self.logger.info(f"Start and goal qpos are valid, check time: {time.time()-start_time:.8f}s")
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
    
    def _sync_pcd_and_stream(self, pcd_time, stream_time, stream_history):
        if pcd_time is None or stream_time is None or stream_history is None:
            return False, None, None, None, None
        # self.logger.debug(f"pcd time: {pcd_time:.6f}s stream time: {stream_time:.6f}s")

        # sync_steps = (stream_time - pcd_time) / self.TIME_STEP
        sync_steps = (time.time() - pcd_time) / self.TIME_STEP

        if sync_steps <= 0:
            # cur_arml_qpos = stream_history[-1]["position"][self.real_arml_idx[0]:self.real_arml_idx[1]+1]
            # cur_arml_vel = stream_history[-1]["velocity"][self.real_arml_idx[0]:self.real_arml_idx[1]+1]
            # cur_armr_qpos = stream_history[-1]["position"][self.real_armr_idx[0]:self.real_armr_idx[1]+1]
            # cur_armr_vel = stream_history[-1]["velocity"][self.real_armr_idx[0]:self.real_armr_idx[1]+1]
            # return True, cur_arml_qpos, cur_arml_vel, cur_armr_qpos, cur_armr_vel
            return False, None, None, None, None
        sync_steps = int(sync_steps)
        # self.logger.debug(f"Sync steps: {sync_steps}")
        if sync_steps >= len(stream_history):
            cur_arml_qpos = stream_history[0]["position"][self.real_arml_idx[0]:self.real_arml_idx[1]+1]
            cur_arml_vel = stream_history[0]["velocity"][self.real_arml_idx[0]:self.real_arml_idx[1]+1]
            cur_armr_qpos = stream_history[0]["position"][self.real_armr_idx[0]:self.real_armr_idx[1]+1]
            cur_armr_vel = stream_history[0]["velocity"][self.real_armr_idx[0]:self.real_armr_idx[1]+1]
            return True, cur_arml_qpos, cur_arml_vel, cur_armr_qpos, cur_armr_vel
        cur_arml_qpos = stream_history[-(sync_steps+1)]["position"][self.real_arml_idx[0]:self.real_arml_idx[1]+1]
        cur_arml_vel = stream_history[-(sync_steps+1)]["velocity"][self.real_arml_idx[0]:self.real_arml_idx[1]+1]
        cur_armr_qpos = stream_history[-(sync_steps+1)]["position"][self.real_armr_idx[0]:self.real_armr_idx[1]+1]
        cur_armr_vel = stream_history[-(sync_steps+1)]["velocity"][self.real_armr_idx[0]:self.real_armr_idx[1]+1]

        return True, cur_arml_qpos, cur_arml_vel, cur_armr_qpos, cur_armr_vel

    def _predict_trajectory_start(self, original_arml_trajectory, original_armr_trajectory, qpos_diff_weight = [1, 1, 1, 1, 1, 1, 1]):
        
        # TODO: decide whether to use the closed point to crop
        res, cur_arml_qpos, _, cur_armr_qpos, _, _, _ = self._get_qpos_from_topic()
        # arml_idx = np.argmin(np.linalg.norm(original_arml_trajectory - cur_arml_qpos, axis=1))
        # armr_idx = np.argmin(np.linalg.norm(original_armr_trajectory - cur_armr_qpos, axis=1))
        arml_diff = original_arml_trajectory - cur_arml_qpos
        armr_diff = original_armr_trajectory - cur_armr_qpos
        arml_cost = (arml_diff * arml_diff * qpos_diff_weight).sum(axis=1)
        armr_cost = (armr_diff * armr_diff * qpos_diff_weight).sum(axis=1)
        arml_idx = np.argmin(arml_cost)
        # print(f"------------{arml_idx}--------------")
        armr_idx = np.argmin(armr_cost)
        if arml_idx < original_arml_trajectory.shape[0]:
            original_arml_trajectory = original_arml_trajectory[arml_idx:, :]
        if armr_idx < original_armr_trajectory.shape[0]:
            original_armr_trajectory = original_armr_trajectory[armr_idx:, :]

        return original_arml_trajectory, original_armr_trajectory

    def _merge_pcd(self, pcd):
        if len(self.pcd_buffer) == 0:
            return pcd
        merged_pcd = np.vstack(self.pcd_buffer)
        merged_pcd = self._voxel_down_sample_numpy(merged_pcd, voxel_size=0.01)
        return merged_pcd

    def plan(self, update_frequency: float = 5.0):
        if self.dynamic_arml_target_qpos is None or self.dynamic_armr_target_qpos is None:
            self.logger.error("Target qpos not set, please set it using set_dynamic_start_target_qpos function")
            return False, None
        
        self.start_moving()

        is_first_plan = True
        planning_time = 0
        while True:
            if planning_time < (1.0 / update_frequency):
                time.sleep((1.0 / update_frequency)-planning_time)
            planning_start_time = time.time()
            
            res, planner_pcd, get_pcd_time = self._reload_environment()
            if not res:
                self.logger.warning("Reload environment failed")
                planning_time = 0
                self.planner_client.publish_trajectory(trajectory=None)
                continue
            res, cur_arml_qpos, cur_arml_vel, cur_armr_qpos, cur_armr_vel, get_stream_time, cur_stream_history = self._get_qpos_from_topic()

            arml_start = cur_arml_qpos
            armr_start = cur_armr_qpos
            sync_res, sync_arml_qpos, sync_arml_vel, sync_armr_qpos, sync_armr_vel = \
                                    self._sync_pcd_and_stream(get_pcd_time, get_stream_time, cur_stream_history)   
            if sync_res:
                # self.logger.info("Successfully sync pcd and streaming qpos")
                cur_arml_qpos = sync_arml_qpos
                cur_armr_qpos = sync_armr_qpos

            self.arml_vamp_planner.reset_env()
            self.armr_vamp_planner.reset_env()
            self.add_torso_collision_to_env()
            # print("reload env time:", time.time()-planning_start_time)
            if planner_pcd is not None and planner_pcd.shape[0] != 0:
                if sync_arml_vel is not None and sync_armr_vel is not None:
                    arml_filter_radius = max(0.02, 0.02 + 0.01 * np.linalg.norm(sync_arml_vel))
                    armr_filter_radius = max(0.02, 0.02 + 0.01 * np.linalg.norm(sync_armr_vel))
                    # print(f"Left arm filter radius: {arml_filter_radius:.4f}m, right arm filter radius: {armr_filter_radius:.4f}m")
                    arml_planner_pcd = self.arml_vamp_planner.vamp_module.filter_self_from_pointcloud(planner_pcd,
                                                                                            arml_filter_radius,
                                                                                            cur_arml_qpos,
                                                                                            self.arml_vamp_planner.env)
                    armr_planner_pcd = self.armr_vamp_planner.vamp_module.filter_self_from_pointcloud(planner_pcd,
                                                                                            armr_filter_radius,
                                                                                            cur_armr_qpos,
                                                                                            self.armr_vamp_planner.env)
                else:
                    arml_planner_pcd = self.arml_vamp_planner.vamp_module.filter_self_from_pointcloud(planner_pcd,
                                                                                            0.02,
                                                                                            cur_arml_qpos,
                                                                                            self.arml_vamp_planner.env)
                    armr_planner_pcd = self.armr_vamp_planner.vamp_module.filter_self_from_pointcloud(planner_pcd,
                                                                                            0.02,
                                                                                            cur_armr_qpos,
                                                                                            self.armr_vamp_planner.env)
                

                arml_planner_pcd = self.radius_outlier_removal(np.asarray(arml_planner_pcd), radius=0.03, min_neighbors=15).tolist()
                armr_planner_pcd = self.radius_outlier_removal(np.asarray(armr_planner_pcd), radius=0.03, min_neighbors=15).tolist()

                self.pcd_buffer.append(np.asarray(arml_planner_pcd))
                # arml_planner_pcd = self._merge_pcd(np.asarray(arml_planner_pcd)).tolist()


                print(f"get pcd and filter self time {time.time()-planning_start_time}")

                # Visulaize for the filtered pcd
                # vis_pcd = self.armr_vamp_planner.vamp_module.filter_self_from_pointcloud(np.asarray(arml_planner_pcd),
                #                                                                         0.02,
                #                                                                         cur_armr_qpos,
                #                                                                         self.armr_vamp_planner.env)

                vis_pcd = deepcopy(arml_planner_pcd)

                if vis_pcd is not None and len(vis_pcd) != 0:
                    current_qpos = self.get_current_qpos()[1]
                    rviz_planner_pcd = np.asarray(deepcopy(vis_pcd))
                    torso_tf, head_tf, arml_tf, armr_tf = self._pin_kine.get_Kinematics(real_qpos=current_qpos)
                    base_link_T_waist = torso_tf[-1]
                    rviz_pcd = np.hstack([rviz_planner_pcd, np.ones((rviz_planner_pcd.shape[0], 1))])
                    rviz_pcd = (base_link_T_waist @ rviz_pcd.T).T[:, :3]
                    rviz_pcd = self.numpy_to_pointcloud2(rviz_pcd, frame_id="base_link")
                    self.pcd_env_publisher.publish(rviz_pcd)

                

                self.arml_vamp_planner.add_pcd(point_cloud=arml_planner_pcd, pcd_radius=0.02)
                self.armr_vamp_planner.add_pcd(point_cloud=armr_planner_pcd, pcd_radius=0.02)

            # if not self._check_start_goal_validity():
            #     self.logger.warning("Start or goal qpos is invalid, replan later")
            #     self.planner_client.publish_trajectory(trajectory=None)
            #     continue

            start_time = time.time()
            
            if np.allclose(arml_start, self.dynamic_arml_target_qpos, rtol=1e-03, atol=1e-03) \
                and np.allclose(armr_start, self.dynamic_armr_target_qpos, rtol=1e-03, atol=1e-03): # FIXME: need to tune the threshold
                self.logger.info("Current qpos is close to target qpos, no need to plan")
                self.planner_client.publish_trajectory(trajectory=None)
                self.planner_client.stop_dynamic_moving_publisher.publish(String(data="stop"))
                break

            if is_first_plan:
                is_first_plan = False
                self.arml_vamp_planner.set_start_goal(start=self.dynamic_arml_start_qpos, goal=self.dynamic_arml_target_qpos)
                self.armr_vamp_planner.set_start_goal(start=self.dynamic_armr_start_qpos, goal=self.dynamic_armr_target_qpos)
            else:
                self.arml_vamp_planner.set_start_goal(start=arml_start, goal=self.dynamic_arml_target_qpos)
                self.armr_vamp_planner.set_start_goal(start=armr_start, goal=self.dynamic_armr_target_qpos)
            
            arml_result = self.arml_vamp_planner.plan()
            armr_result = self.armr_vamp_planner.plan()
            
            if arml_result is not None and armr_result is not None:
                self.arml_last_velocity = cur_arml_vel
                self.armr_last_velocity = cur_armr_vel

                self.logger.debug(f"arml result shape {arml_result.shape}, armr result shape: {armr_result.shape}")

                arml_traj_result = self._gen_trajectory_with_toppra(arml_result, max_v=0.3, last_velocity=self.arml_last_velocity)[0]
                armr_traj_result = self._gen_trajectory_with_toppra(armr_result, max_v=0.3, last_velocity=self.armr_last_velocity)[0]

                qpos_diff_weight = [2.0, 1.8, 1.6, 1.3, 1.0, 0.8, 0.7]
                arml_traj_result, armr_traj_result = self._predict_trajectory_start(arml_traj_result, armr_traj_result, qpos_diff_weight)

                # publish the result trajectory
                res, cur_torso_qpos, cur_head_qpos, cur_arml_qpos, cur_armr_qpos = self._get_current_full_qpos()
                self.logger.debug(f"arml_traj_result shape {arml_traj_result.shape}, armr_traj_result shape: {armr_traj_result.shape}")
                res_align, full_traj = self._qpos_align(cur_torso_qpos, cur_head_qpos, arml_traj_result, armr_traj_result)


                self.planner_client.publish_trajectory(trajectory=full_traj)
                self.logger.info(f"Total planning cost time: {time.time() - start_time}s")
                # self.logger.info("Planning succeeded, publishing trajectory")

                planning_time = time.time() - planning_start_time
                self.logger.info(f"Total replan(include obtaining data) cost time: {planning_time}s")
            else:
                self.logger.warning("Planning failed, no trajectory to publish")
                planning_time = time.time() - planning_start_time
                self.planner_client.publish_trajectory(trajectory=None)

        self.planner_client.publish_trajectory(trajectory=None)

    def start_moving(self):
        req = ClientService.Request()
        req.request = json.dumps({})
        future = self.start_moving_client.call_async(req)
        return future

    def check_traj_validity(self, last_arml_traj: np.ndarray, last_armr_traj: np.ndarray):
        if last_arml_traj is None or last_arml_traj.shape[0] == 0 or last_armr_traj is None or last_armr_traj.shape[0] == 0:
            # self.logger.error("Invalid trajectory for validity check")
            return False
        res, cur_arml_qpos, cur_arml_vel, cur_armr_qpos, cur_armr_vel, get_stream_time, cur_stream_history = self._get_qpos_from_topic()
        arml_idx = np.where(np.linalg.norm(last_arml_traj - cur_arml_qpos, axis=1) < 1e-6)[0]
        armr_idx = np.where(np.linalg.norm(last_armr_traj - cur_armr_qpos, axis=1) < 1e-6)[0]
        if len(arml_idx) > 0:
            last_arml_traj = last_arml_traj[arml_idx[0]:, :]
        if len(armr_idx) > 0:
            last_armr_traj = last_armr_traj[armr_idx[0]:, :]
        last_arml_traj = last_arml_traj[::2]
        last_armr_traj = last_armr_traj[::2]
        start_time = time.time()
        qpos_cnt = 0
        for qpos in last_arml_traj:
            qpos_cnt += 1
            if not self.arml_vamp_planner.vamp_module.validate(qpos.tolist(), self.arml_vamp_planner.env):
                print(f"---Collision at step {qpos_cnt-1}---")
                self.logger.warning("Left arm trajectory invalid")
                return False
        # self.logger.info(f"average checking time: {(time.time()-start_time) / len(last_arml_traj):.8f}s with trajectory length {len(last_arml_traj)}")
        self.time_dict["average checking time"] = (time.time()-start_time) / len(last_arml_traj)
        for qpos in last_armr_traj:
            if not self.armr_vamp_planner.vamp_module.validate(qpos.tolist(), self.armr_vamp_planner.env):
                self.logger.warning("Right arm trajectory invalid")
                return False
        # self.logger.info("Trajectory is valid")
        return True

    def plan_keep_checking(self, update_frequency: float = 5.0):
        self.time_dict_list = []
        if self.dynamic_arml_target_qpos is None or self.dynamic_armr_target_qpos is None:
            self.logger.error("Target qpos not set, please set it using set_dynamic_start_target_qpos function")
            return False, None
        
        self.start_moving()

        is_first_plan = True
        total_time = 0

        last_arml_trajectory = None
        last_armr_trajectory = None

        self.pcd_buffer = deque(maxlen=6)

        while True:
            self.time_dict = {}
            if total_time < (1.0 / update_frequency):
                time.sleep((1.0 / update_frequency)-total_time)
            total_start_time = time.time()

            res, planner_pcd, get_pcd_time = self._reload_environment()
            if not res:
                self.logger.warning("Reload environment failed")
                total_time = 0
                # self.planner_client.publish_trajectory(trajectory=None)
                continue
            res, cur_arml_qpos, cur_arml_vel, cur_armr_qpos, cur_armr_vel, get_stream_time, cur_stream_history = self._get_qpos_from_topic()

            arml_start = cur_arml_qpos
            armr_start = cur_armr_qpos
            start_time = time.time()
            sync_res, sync_arml_qpos, sync_arml_vel, sync_armr_qpos, sync_armr_vel = \
                                    self._sync_pcd_and_stream(get_pcd_time, get_stream_time, cur_stream_history)   
            if sync_res:
                # self.logger.info("Successfully sync pcd and streaming qpos")
                cur_arml_qpos = sync_arml_qpos
                cur_armr_qpos = sync_armr_qpos
            self.time_dict["sync pcd and feedback"] = time.time() - start_time
            start_time = time.time()
            self.arml_vamp_planner.reset_env()
            self.armr_vamp_planner.reset_env()
            cur_qpos = self.get_current_qpos()[1]
            self.arml_vamp_planner.set_head_body_qpos(head_qpos=cur_qpos[4:6], body_qpos=cur_qpos[:4])
            self.armr_vamp_planner.set_head_body_qpos(head_qpos=cur_qpos[4:6], body_qpos=cur_qpos[:4])
            self.add_torso_collision_to_env()
            self.arml_vamp_planner.add_cuboid(np.array([0.6, 0, 0.72]), np.array([0.0, 0.0, 0.0]), np.array([0.3, 0.6, 0.02]))
            self.armr_vamp_planner.add_cuboid(np.array([0.6, 0, 0.72]), np.array([0.0, 0.0, 0.0]), np.array([0.3, 0.6, 0.02]))
            self.time_dict["reset env & add torso"] = time.time() - start_time

            # test attachment
            if self.armlee_attached:
                arml_attachment = vamp.Attachment(self.arml_attachment_tf)
                for sphere in self.arml_attachment_spheres:
                    arml_attachment.add_spheres([vamp.Sphere(sphere[0], sphere[1])])
                arml_attachment.set_ee_pose(self.arml_vamp_planner.vamp_module.eefk(cur_arml_qpos))
                self.arml_vamp_planner.env.attach(arml_attachment)

            if self.armree_attached:
                armr_attachment = vamp.Attachment(self.armr_attachment_tf)
                for sphere in self.armr_attachment_spheres:
                    armr_attachment.add_spheres([vamp.Sphere(sphere[0], sphere[1])])
                armr_attachment.set_ee_pose(self.armr_vamp_planner.vamp_module.eefk(cur_armr_qpos))
                self.armr_vamp_planner.env.attach(armr_attachment)
            # tf = np.identity(4)
            # tf[:3, 3] = np.array([0, 0, 0.106])

            # arml_attachment = vamp.Attachment(tf)
            # arml_attachment.add_spheres([vamp.Sphere([0.05, 0.0, 0], 0.035)])
            # arml_attachment.add_spheres([vamp.Sphere([0.025, 0.0, 0], 0.035)])
            # arml_attachment.add_spheres([vamp.Sphere([0, 0.0, 0], 0.035)])
            # arml_attachment.add_spheres([vamp.Sphere([-0.05, 0.0, 0], 0.035)])
            # arml_attachment.add_spheres([vamp.Sphere([-0.075, 0.0, 0], 0.035)])
            # arml_attachment.add_spheres([vamp.Sphere([-0.1, 0.0, 0], 0.035)])
            # # arml_attachment.add_spheres([vamp.Sphere([0.15, 0.045, 0], 0.05)])
            # arml_attachment.set_ee_pose(self.arml_vamp_planner.vamp_module.eefk(cur_arml_qpos))
            # self.arml_vamp_planner.env.attach(arml_attachment)
            

            # armr_attachment = vamp.Attachment(tf)
            # armr_attachment.add_spheres([vamp.Sphere([0, 0, 0], 0.2)])
            # self.armr_vamp_planner.env.attach(armr_attachment)
            # armr_attachment.set_ee_pose(self.armr_vamp_planner.vamp_module.eefk(cur_armr_qpos))

            # self.arml_vamp_planner.env.set_attachment_pose(self.arml_vamp_planner.vamp_module.eefk(cur_arml_qpos))


            start_time = time.time()
            if planner_pcd is not None and planner_pcd.shape[0] != 0:
                if sync_arml_vel is not None and sync_armr_vel is not None:
                    arml_filter_radius = max(0.02, 0.02 + 0.01 * np.linalg.norm(sync_arml_vel))
                    armr_filter_radius = max(0.02, 0.02 + 0.01 * np.linalg.norm(sync_armr_vel))
                    # print(f"Left arm filter radius: {arml_filter_radius:.4f}m, right arm filter radius: {armr_filter_radius:.4f}m")
                    arml_planner_pcd = self.arml_vamp_planner.vamp_module.filter_self_from_pointcloud(planner_pcd,
                                                                                            arml_filter_radius,
                                                                                            cur_arml_qpos,
                                                                                            self.arml_vamp_planner.env)
                    armr_planner_pcd = self.armr_vamp_planner.vamp_module.filter_self_from_pointcloud(planner_pcd,
                                                                                            armr_filter_radius,
                                                                                            cur_armr_qpos,
                                                                                            self.armr_vamp_planner.env)
                else:
                    arml_planner_pcd = self.arml_vamp_planner.vamp_module.filter_self_from_pointcloud(planner_pcd,
                                                                                            0.02,
                                                                                            cur_arml_qpos,
                                                                                            self.arml_vamp_planner.env)
                    armr_planner_pcd = self.armr_vamp_planner.vamp_module.filter_self_from_pointcloud(planner_pcd,
                                                                                            0.02,
                                                                                            cur_armr_qpos,
                                                                                            self.armr_vamp_planner.env)
                
                self.time_dict["filter self"] = time.time() - start_time
                start_time = time.time()
                arml_planner_pcd = self.radius_outlier_removal(np.asarray(arml_planner_pcd), radius=0.03, min_neighbors=25).tolist()
                armr_planner_pcd = self.radius_outlier_removal(np.asarray(armr_planner_pcd), radius=0.03, min_neighbors=25).tolist()
                self.time_dict["remove noise"] = time.time() - start_time

                # augment_start_time = time.time()
                # arml_planner_pcd = self.augment_pointcloud(np.asarray(arml_planner_pcd), step=0.05, max_range=0.2)
                # armr_planner_pcd = self.augment_pointcloud(np.asarray(armr_planner_pcd), step=0.05, max_range=0.2)
                # arml_planner_pcd = self._voxel_down_sample_numpy(arml_planner_pcd, voxel_size=0.01).tolist()
                # armr_planner_pcd = self._voxel_down_sample_numpy(armr_planner_pcd, voxel_size=0.01).tolist()
                # self.logger.info(f"After downsample and augmentation, pointcloud points: {len(arml_planner_pcd)} and {len(armr_planner_pcd)}")
                # self.logger.info(f"Aumentation time: {time.time() - augment_start_time}")

                # self.pcd_buffer.append(np.asarray(arml_planner_pcd))
                # arml_planner_pcd = self._merge_pcd(np.asarray(arml_planner_pcd)).tolist()

                # self.logger.info(f"get pcd and filter self time {time.time()-total_start_time}")
                self.arml_vamp_planner.add_pcd(point_cloud=arml_planner_pcd, pcd_radius=0.02)
                self.armr_vamp_planner.add_pcd(point_cloud=armr_planner_pcd, pcd_radius=0.02)
                
                vis_pcd = deepcopy(armr_planner_pcd)

                if vis_pcd is not None and len(vis_pcd) != 0:
                    current_qpos = self.get_current_qpos()[1]
                    rviz_planner_pcd = np.asarray(deepcopy(vis_pcd))
                    torso_tf, head_tf, arml_tf, armr_tf = self._pin_kine.get_Kinematics(real_qpos=current_qpos)
                    base_link_T_waist = torso_tf[-1]
                    rviz_pcd = np.hstack([rviz_planner_pcd, np.ones((rviz_planner_pcd.shape[0], 1))])
                    rviz_pcd = (base_link_T_waist @ rviz_pcd.T).T[:, :3]
                    rviz_pcd = self.numpy_to_pointcloud2(rviz_pcd, frame_id="base_link")
                    self.pcd_env_publisher.publish(rviz_pcd)

            # if not self._check_start_goal_validity():
            #     self.logger.warning("Start or goal qpos is invalid, replan later")
            #     self.planner_client.publish_trajectory(trajectory=None)
            #     continue

            if np.allclose(arml_start, self.dynamic_arml_target_qpos, rtol=1e-03, atol=1e-03) \
                    and np.allclose(armr_start, self.dynamic_armr_target_qpos, rtol=1e-03, atol=1e-03): # FIXME: need to tune the threshold
                    self.logger.info("Current qpos is close to target qpos, no need to plan")
                    self.planner_client.publish_trajectory(trajectory=None)
                    self.planner_client.stop_dynamic_moving_publisher.publish(String(data="stop"))
                    break

            self.time_dict["Result left pcd"] = len(arml_planner_pcd)
            checking_start_time = time.time()
            last_traj_validity = self.check_traj_validity(last_arml_trajectory, last_armr_trajectory)
            checking_time = time.time() - checking_start_time
            self.time_dict["Total trajectory checking time"] = checking_time
            total_time = time.time() - total_start_time
            # self.logger.info(f"Trajectory validity checking time: {checking_time}s, validity: {last_traj_validity}")
            if not last_traj_validity:
                self.planner_client.publish_trajectory(trajectory=None)
                res, cur_arml_qpos, cur_arml_vel, cur_armr_qpos, cur_armr_vel, get_stream_time, cur_stream_history = self._get_qpos_from_topic()
                arml_start = cur_arml_qpos
                armr_start = cur_armr_qpos
                
                if is_first_plan:
                    is_first_plan = False
                    self.arml_vamp_planner.set_start_goal(start=self.dynamic_arml_start_qpos, goal=self.dynamic_arml_target_qpos)
                    self.armr_vamp_planner.set_start_goal(start=self.dynamic_armr_start_qpos, goal=self.dynamic_armr_target_qpos)
                else:
                    self.arml_vamp_planner.set_start_goal(start=arml_start, goal=self.dynamic_arml_target_qpos)
                    self.armr_vamp_planner.set_start_goal(start=armr_start, goal=self.dynamic_armr_target_qpos)
                
                planning_start_time = time.time()
                arml_result = self.arml_vamp_planner.plan()
                armr_result = self.armr_vamp_planner.plan()
                
                if arml_result is not None and armr_result is not None:
                    self.arml_last_velocity = cur_arml_vel
                    self.armr_last_velocity = cur_armr_vel

                    # self.logger.debug(f"arml result shape {arml_result.shape}, armr result shape: {armr_result.shape}")

                    arml_traj_result = self._gen_trajectory_with_toppra(arml_result, max_v=0.3, last_velocity=self.arml_last_velocity)[0]
                    armr_traj_result = self._gen_trajectory_with_toppra(armr_result, max_v=0.3, last_velocity=self.armr_last_velocity)[0]

                    qpos_diff_weight = [2.0, 1.8, 1.6, 1.3, 1.0, 0.8, 0.7]
                    arml_traj_result, armr_traj_result = self._predict_trajectory_start(arml_traj_result, armr_traj_result, qpos_diff_weight)
                    last_arml_trajectory = arml_traj_result
                    last_armr_trajectory = armr_traj_result
                    # publish the result trajectory
                    res, cur_torso_qpos, cur_head_qpos, cur_arml_qpos, cur_armr_qpos = self._get_current_full_qpos()
                    # self.logger.debug(f"arml_traj_result shape {arml_traj_result.shape}, armr_traj_result shape: {armr_traj_result.shape}")
                    res_align, full_traj = self._qpos_align(cur_torso_qpos, cur_head_qpos, arml_traj_result, armr_traj_result)


                    self.planner_client.publish_trajectory(trajectory=full_traj)
                    # self.logger.info(f"Total planning cost time: {time.time() - planning_start_time}s")
                    # self.logger.info("Planning succeeded, publishing trajectory")

                    total_time = time.time() - total_start_time
                    # self.logger.info(f"Total replan(include obtaining data) cost time: {total_time}s")
                else:
                    self.logger.warning("Planning failed, no trajectory to publish")
                    total_time = time.time() - total_start_time
                    self.planner_client.publish_trajectory(trajectory=None)
                    last_arml_trajectory = None
                    last_armr_trajectory = None
            # self.print_time_consuming(self.time_dict)
            self.time_dict_list.append(self.time_dict)
        self.planner_client.publish_trajectory(trajectory=None)

    # The following functions are for armlee and armree attachment
    def attach_left_arm(self, attachment_frame_tf: np.ndarray, spheres: list):
        """
        Args:
            attachment_frame (np.ndarray, shape=(4,4)): left_ee_frame_T_attachment_frame
            spheres (list): list of tuples, each tuple is (center: list of float (3,), radius: float)
        """
        self.armlee_attached = True
        self.arml_attachment_tf = attachment_frame_tf
        self.arml_attachment_spheres = spheres

    def attach_right_arm(self, attachment_frame_tf: np.ndarray, spheres: list):
        """
        Args:
            attachment_frame (np.ndarray, shape=(4,4)): right_ee_frame_T_attachment_frame
            spheres (list): list of tuples, each tuple is (center: list of float (3,), radius: float)
        """
        self.armree_attached = True
        self.armr_attachment_tf = attachment_frame_tf
        self.armr_attachment_spheres = spheres

    def detach_left_arm(self):
        self.armlee_attached = False
        self.arml_attachment_tf = None
        self.arml_attachment_spheres = []

    def detach_right_arm(self):
        self.armree_attached = False
        self.armr_attachment_tf = None
        self.armr_attachment_spheres = []

    # The following functions are for torso 1 2 3 collision checking
    def _compute_waist_T_torso(self):
        res, cur_qpos = self.get_current_qpos()
        torso_tf, head_tf, arml_tf, armr_tf = self._pin_kine.get_Kinematics(real_qpos=cur_qpos)

        waist_T_ankle = np.linalg.inv(torso_tf[-1]) @ torso_tf[0]
        waist_T_knee = np.linalg.inv(torso_tf[-1]) @ torso_tf[1]
        waist_T_buttock = np.linalg.inv(torso_tf[-1]) @ torso_tf[2]

        

        return True, waist_T_ankle, waist_T_knee, waist_T_buttock
    
    def _init_torso_collision(self):
        self.first_init_torso_collision = False
        self.ankle_collision, self.knee_collision, self.buttock_collision = [], [], []
        self.ankle_collision.append(([-4.5980105860658e-49, 0.00470564178275118, 0.04483259610327928], 0.0908840518267283))
        self.ankle_collision.append(([-1.540743955509789e-33, 0.0039850179034444655, -0.04363911150875806], 0.0908840518267283))
        self.ankle_collision.append(([1.5407439555097882e-33, 0.11961474737979472, 0.05144951411198579], 0.0908840518267283))
        self.ankle_collision.append(([-4.5980105860658e-49, 0.12158115599155228, -0.043608343328251045 ], 0.0908840518267283))
        self.ankle_collision.append(([-4.5980105860658e-49, 0.23781198482994792, 0.05635133629925315], 0.0908840518267283))
        self.ankle_collision.append(([-4.5980105860658e-49, 0.23792534680573463, -0.047245838917225495], 0.0908840518267283))
        self.ankle_collision.append(([-4.622231866529367e-33, 0.3281024661610352, 0.05403185307674402 ], 0.0908840518267283))
        self.ankle_collision.append(([-1.540743955509789e-33, 0.3277440920416459, -0.05500361962402418], 0.0908840518267283))

        self.knee_collision.append(([-0.004351185184101105, 0.005811189676413475, -0.0018416225282883465], 0.09561191828866984))
        self.knee_collision.append(([-5.473822126268818e-48, 0.12358733190849125, -2.465190328815662e-32], 0.09561191828866984))
        self.knee_collision.append(([-2.7369110631344096e-48, 0.23981320458230027, -2.465190328815662e-32], 0.09561191828866984))
        self.knee_collision.append(([-0.0018484470498033522, 0.33948860189689456, 0.052893369619988764], 0.08054239262315771))
        self.knee_collision.append(([-2.7369110631344096e-48, 0.34255584518233007, -0.0522664330570803], 0.08054239262315771))

        self.buttock_collision.append(([0.0, 0.0, 0.0], 0.09917384684490473))
        self.buttock_collision.append(([-8.149862198248183e-95, 0.13283260321417567, 0.0], 0.06872532507502535))
        self.buttock_collision.append(([5.473822126268817e-48, 0.09311773980968707, 0.0], 0.06991851138466271))

        

    def add_torso_collision_to_env(self):
        if self.first_init_torso_collision:
            self._init_torso_collision()
            res, self.waist_T_ankle, self.waist_T_knee, self.waist_T_buttock = self._compute_waist_T_torso()
            # input("press Enter to publish the torso collision markers...")
            res, cur_qpos = self.get_current_qpos()
            torso_tf, head_tf, arml_tf, armr_tf = self._pin_kine.get_Kinematics(real_qpos=cur_qpos)
            ma = MarkerArray()
            for e in self.ankle_collision:
                center, radius = e
                center_h = np.array([center[0], center[1], center[2], 1.0])
                center_in_world = torso_tf[0] @ center_h
                marker = Marker()
                marker.header.frame_id = "base_link"
                marker.ns = "torso_ankle_collision"
                marker.id = len(ma.markers)
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD

                marker.pose.position.x = center_in_world[0]
                marker.pose.position.y = center_in_world[1]
                marker.pose.position.z = center_in_world[2]
                marker.pose.orientation.w = 1.0

                marker.scale.x = radius * 2
                marker.scale.y = radius * 2
                marker.scale.z = radius * 2

                marker.color.a = 0.5
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0

                ma.markers.append(marker)
            
            self.torso_collision_publisher.publish(ma)

        start_time = time.time()
        
        for e in self.ankle_collision:
            center, radius = e
            center_h = np.array([center[0], center[1], center[2], 1.0])
            center_in_waist = self.waist_T_ankle @ center_h
            self.arml_vamp_planner.env.add_sphere(vamp.Sphere(center_in_waist[:3].tolist(), radius))
            self.armr_vamp_planner.env.add_sphere(vamp.Sphere(center_in_waist[:3].tolist(), radius))

        for e in self.knee_collision:
            center, radius = e
            center_h = np.array([center[0], center[1], center[2], 1.0])
            center_in_waist = self.waist_T_knee @ center_h
            self.arml_vamp_planner.env.add_sphere(vamp.Sphere(center_in_waist[:3].tolist(), radius))
            self.armr_vamp_planner.env.add_sphere(vamp.Sphere(center_in_waist[:3].tolist(), radius))

        # print(f"add torso collision time: {time.time()-start_time}s")

        # for e in self.buttock_collision:
        #     center, radius = e
        #     center_h = np.array([center[0], center[1], center[2], 1.0])
        #     center_in_waist = waist_T_buttock @ center_h
        #     self.arml_vamp_planner.env.add_sphere(vamp.Sphere(center_in_waist[:3].tolist(), radius))
        #     self.armr_vamp_planner.env.add_sphere(vamp.Sphere(center_in_waist[:3].tolist(), radius))

    def print_time_consuming(self, time_dict):
        print("┌─────────────────────────────┬──────────────┐")
        print("│ Task                        │ Time (s)     │")
        print("├─────────────────────────────┼──────────────┤")
        for k, v in time_dict.items():
            print(f"│ {k:<25} │ {v:>10.6f} │")
        print("└─────────────────────────────┴──────────────┘")
        pass

    