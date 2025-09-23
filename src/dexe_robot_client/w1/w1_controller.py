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
from std_srvs.srv import SetBool

from IPython import embed

class W1Controller:
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

        self._clients = []

        self.robot_client = None
        self._robot_client_sim = None
        self._robot_client_real = None

        self.left_ee_client = None
        self._left_ee_client_sim = None
        self._left_ee_client_real = None

        self.right_ee_client = None
        self._right_ee_client_sim = None
        self._right_ee_client_real = None

        self._executor = None
        self._node_thread = None

        self._is_sim = None
        self._arm_type = arm_type
        self._w1_version = w1_version
        self._end_effector_type = end_effector_type

        self.w1_params = W1Params(self._arm_type, 
                                  self._w1_version, 
                                  self._end_effector_type)
        
        atexit.register(self._destroy_clients)

        try:
            self._init_clients()
        except Exception:
            raise RuntimeError("Ros2 node create failed")
        
        self._init_service_servers()

        self._use_rlia = use_rlia
        self._init_kine_solver()

        if not self.set_sim_mode(is_sim):
            if is_sim:
                raise RuntimeError("Set sim mode failed")
            raise RuntimeError("Set real mode failed")
        
        self._stop_move_event = threading.Event()
        self._stop_move_event.clear()
        
    def _init_service_servers(self):
        if not rclpy.ok():
            rclpy.init()
        self.service_executor = MultiThreadedExecutor()
        
        self.service_node = Node("w1_controller_service_node")
        self._has_arm_ee_client_server = self.service_node.create_service(ClientService, "has_arm_ee_client", self._has_arm_ee_client_callback)



        self.service_executor.add_node(self.service_node)
        self._service_thread = threading.Thread(target=self.service_executor.spin, 
                                            daemon=True)
        self._service_thread.start()
        self.logger.info("W1 controller service server initialized.")

    def _init_clients(self):
        if not rclpy.ok():
            rclpy.init()

        self._robot_client_sim = RobotClient(is_sim = True)
        self._robot_client_real = RobotClient(is_sim = False)
        self.robot_client = self._robot_client_sim

        if self._end_effector_type is None:
            self.logger.info(f"No movable end effector.")
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


    def set_sim_mode(self, enable_sim: bool) -> bool:
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
        else:
            target_name = "real"
            target_robot_client = self._robot_client_real
            target_left_ee_client = self._left_ee_client_real
            target_right_ee_client = self._right_ee_client_real

        # 检查target client
        # if target_robot_client is None:
        #     self.logger.error(f"Robot_client_{target_name} is not initialized.")
        #     return False
        if not target_robot_client.is_connected(timeout_sec=0.1):
            self.logger.error(f"{target_robot_client.node_name} is not connected!")
            return False
        if target_robot_client.status not in [Status.Init, Status.Idle]:
            self.logger.error(f"{target_robot_client.node_name} status: {target_robot_client.status}, can not switch mode!")
            return False

        # if target_left_ee_client is None:
        #     self.logger.error(f"Left_ee_client_{target_name} is not initialized.")
        #     return False
        if not target_left_ee_client.is_connected():
            self.logger.error(f"{target_left_ee_client.node_name} is not connected!")
            return False
        # if target_right_ee_client is None:
        #     self.logger.error(f"Right_ee_client_{target_name} is not initialized.")
        #     return False
        if not target_right_ee_client.is_connected():
            self.logger.error(f"{target_right_ee_client.node_name} is not connected!")
            return False

        # 检查current client
        if not self.robot_client.is_connected():
            self.logger.error(f"{self.robot_client.node_name} is not connected!")
            return False
        if self.robot_client.status not in [Status.Init, Status.Idle]:
            self.logger.error(f"{self.robot_client.node_name} status: {self.robot_client.status}, can not switch mode!")
            return False

        if not self.left_ee_client.is_connected():
            self.logger.error(f"{self.left_ee_client.node_name} is not connected!")
            return False
        if not self.right_ee_client.is_connected():
            self.logger.error(f"{self.right_ee_client.node_name} is not connected!")
            return False

        self.robot_client = target_robot_client
        self.left_ee_client = target_left_ee_client
        self.right_ee_client = target_right_ee_client
        self._is_sim = enable_sim
        self.logger.info(f"W1 manager set to {target_name} mode.")
        return True
    

    def _has_arm_ee_client(self, name: str, verbose: bool = True) -> bool:
        if name in self.w1_params.arm_ee_names:
            if name == "left_arm_ee" and self.left_ee_client is not None:
                return True
            if name == "right_arm_ee" and self.right_ee_client is not None:
                return True
            if verbose:
                self.logger.error(f"{name} has not initialized")
            return False

    def _has_arm_ee_client_callback(self, request, response):
        """
        This function is a service callback of function _has_arm_ee_client
        """
        data = json.loads(request.request)
        if "name" in data:
            name = data["name"]
            self.logger.info(f"Check has_arm_ee_client: {name}")

            result = self._has_arm_ee_client(name)
            response.response = json.dumps(
                {
                    "success": True,
                    "has_arm_ee_client": result
                }
            )
        else:
            self.logger.error(f"Check has_arm_ee_client failed, no name in request")
            response.response = json.dumps(
                {
                    "success": False,
                    "has_arm_ee_client": False
                }
            )
        return response