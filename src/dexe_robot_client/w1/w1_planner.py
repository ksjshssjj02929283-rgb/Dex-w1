# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

from ..common import time_consuming, get_logger, DATA_DIR, W1ArmType, W1Version
from ..end_effector_client import EndEffectorType, create_end_effector_client
from ..trajectory_client import TrajectoryPublisher, TrajectorySubscriber
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

        self._is_sim = None
        self._arm_type = arm_type
        self._w1_version = w1_version
        self._end_effector_type = end_effector_type

        self.w1_params = W1Params(self._arm_type, 
                                  self._w1_version, 
                                  self._end_effector_type)
        self._init_service_clients()

        self._use_rlia = use_rlia
        self._init_kine_solver()
        self._init_collision_checker()
        self.real_torso_idx, self.real_head_idx, self.real_arml_idx, self.real_armr_idx = get_real_idx()
        self._init_planner(arm_type=arm_type, w1_version=w1_version, end_effector_type=end_effector_type)


    def _init_service_clients(self):
        if not rclpy.ok():
            rclpy.init()
        self.service_executor = MultiThreadedExecutor()

        self.service_node = Node("w1_planner_service_node")
        self._has_arm_ee_client_client = self.service_node.create_client(ClientService, "has_arm_ee_client")
        while not self._has_arm_ee_client_client.wait_for_service(timeout_sec=1.0):
            self.logger.warning("service not available, waiting again...")
        self.logger.info("Connected to has_arm_ee_client service.")


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

    def _init_collision_checker(self):
        if self._arm_type == W1ArmType.ANTHROPOMORPHIC:
            robot_type = RobotType.W1V02_1
        else:
            robot_type = RobotType.W1V02_2
        self._pin_kine = Planner_Pin(robot_type, 
                                     self._end_effector_type)
        self._collision_checker = FCLChecker(robot_type, 
                                             None)
        self.logger.info("Initialize FCL collision checker.")

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


    def _check_name(self, name: str) -> bool:
        """仅支持component检查，不包含ee"""
        if name not in self.w1_params.names:
            self.logger.error(f"{self.w1_params.names} is supported, except {name}.")
            return False
        return True

    def _check_qpos_dimension(self, name: str, qpos) -> bool:
        """支持component和ee的检查"""
        if name in self.w1_params.arm_ee_names:
            req = ClientService.Request()
            req.request = json.dumps({"name": name})
            future = self._has_arm_ee_client_client.call_async(req)
            rclpy.spin_until_future_complete(self.service_node, future)
            res = json.loads(future.result().response)["has_arm_ee_client"]
            if not res:
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

    def get_qpos_limits(self, name: str = "full_body") -> tuple[bool, np.ndarray, np.ndarray]:
        """获取特定部件(支持ee)的关节极限

        Args:
            name (str, optional): ["full_body", "torso", "head", "left_arm", "right_arm", 
                                    "left_arm_ee", "right_arm_ee"]. Defaults to "full_body".

        Returns:
            tuple[bool, np.ndarray, np.ndarray]: 状态码；下极限；上极限
        """
        if name in self.w1_params.arm_ee_names:
            req = ClientService.Request()
            req.request = json.dumps({"name": name})
            future = self._has_arm_ee_client_client.call_async(req)
            rclpy.spin_until_future_complete(self.service_node, future)
            res = json.loads(future.result().response)["has_arm_ee_client"]
            if not res:
                return False, None, None
            qpos_lb = np.array([0.]*self.w1_params.arm_ee_dofs)
            qpos_ub = np.array([100.]*self.w1_params.arm_ee_dofs)

        elif not self._check_name(name):
            return False, None, None
        else:
            qpos_lb = self.w1_params.qpos_lb[self.w1_params.index_infos[name]["index"]]
            qpos_ub = self.w1_params.qpos_ub[self.w1_params.index_infos[name]["index"]]
        return True, qpos_lb, qpos_ub
    
    def get_home_qpos(self) -> np.ndarray:
        return self.w1_params.home_qpos
    


    

