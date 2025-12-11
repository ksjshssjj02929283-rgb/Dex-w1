# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

from ..common import time_consuming, get_logger, DATA_DIR, W1ArmType, W1Version
from ..end_effector_client import EndEffectorType, create_end_effector_client
from ..controller_client import ControllerClient
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
                end_effector_type: EndEffectorType = None):
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

        if not self.set_sim_mode(is_sim):
            if is_sim:
                raise RuntimeError("Set sim mode failed")
            raise RuntimeError("Set real mode failed")
        
        self._stop_move_event = threading.Event()
        self._stop_move_event.clear()

        self._stop_dynamic_moving = False

        
    def _init_service_servers(self):
        if not rclpy.ok():
            rclpy.init()
        self.service_executor = MultiThreadedExecutor()
        
        self.service_node = Node("w1_controller_service_node")
        self._has_arm_ee_client_server = self.service_node.create_service(ClientService,
                                                                          "_has_arm_ee_client",
                                                                           self._has_arm_ee_client_callback)
        self.connect_server = self.service_node.create_service(ClientService,
                                                                "connect",
                                                                self.connect_callback)
        self.is_connected_server = self.service_node.create_service(ClientService,
                                                                    "is_connected",
                                                                    self.is_connected_callback)
        self.clear_robot_error_server = self.service_node.create_service(ClientService,
                                                                        "clear_robot_error",
                                                                        self.clear_robot_error_callback)
        self.set_drag_mode_server = self.service_node.create_service(ClientService,
                                                                    "set_drag_mode",
                                                                    self.set_drag_mode_callback)
        self.set_safe_level_server = self.service_node.create_service(ClientService,
                                                                    "set_safe_level",
                                                                    self.set_safe_level_callback)
        self.set_sim_mode_server = self.service_node.create_service(ClientService,
                                                                    "set_sim_mode",
                                                                    self.set_sim_mode_callback)
        self.set_ee_qpos_server = self.service_node.create_service(ClientService,
                                                                   "set_ee_qpos",
                                                                   self.set_ee_qpos_callback)
        self.go_home_server = self.service_node.create_service(ClientService,
                                                               "go_home",
                                                               self.go_home_callback)
        self.stop_move_server = self.service_node.create_service(ClientService,
                                                                "stop_move",
                                                                self.stop_move_callback)
        self.set_home_qpos_server = self.service_node.create_service(ClientService,
                                                                    "set_home_qpos",
                                                                    self.set_home_qpos_callback)
        self.execute_joint_traj_server = self.service_node.create_service(ClientService,
                                                                        "execute_joint_traj",
                                                                        self.execute_joint_traj_callback)
        self.start_moving_server = self.service_node.create_service(ClientService,
                                                                    "start_moving",
                                                                    self.start_moving_callback)
        
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

        self.controller_client = ControllerClient()
        

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
        self._clients.append(self.controller_client)
        
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
        This function is a service callback of function _has_arm_ee_client()
        """
        data = json.loads(request.request)
        if "name" in data:
            name = data["name"]
            self.logger.info(f"Check has_arm_ee_client: {name}")

            if "verbose" in data:
                verbose = data["verbose"]
                result = self._has_arm_ee_client(name, verbose=verbose)
            else:
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
    
    def _get_arm_ee_client(self, name: str):
        if not self._has_arm_ee_client(name):
            return None
        if name == "left_arm_ee":
            return self.left_ee_client
        elif name == "right_arm_ee":
            return self.right_ee_client

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

    ######## 基础API #############

    def connect(self, timeout_sec: float = 0.5) -> bool:
        """连接client(sim/real)

        Returns:
            bool: 
        """
        return self.is_connected(timeout_sec)
    
    def connect_callback(self, request, response):
        """
        This function is a service callback of function connect()
        """
        data = json.loads(request.request)
        if "timeout_sec" in data:
            timeout_sec = data["timeout_sec"]
            self.logger.info(f"Connect with timeout_sec: {timeout_sec}")
            result = self.connect(timeout_sec)
            response.response = json.dumps(
                {
                    "success": True,
                    "is_connected": result
                }
            )
        else:
            self.logger.error(f"Connect failed, no timeout_sec in request")
            response.response = json.dumps(
                {
                    "success": False,
                    "is_connected": False
                }
            )
        return response
    
    def is_connected(self, timeout_sec: float = 0.1) -> bool:
        """检查所有client是否处于连接状态

        Returns:
            bool: 
        """
        res = True
        if not self.robot_client.is_connected(timeout_sec):
            self.logger.error(f"{self.robot_client.node_name} is not connected!")
            res = False
        if self._has_arm_ee_client("left_arm_ee", False):
            if not self.left_ee_client.is_connected(timeout_sec):
                self.logger.error(f"{self.left_ee_client.node_name} is not connected!")
                res = False
            if not self.right_ee_client.is_connected(timeout_sec):
                self.logger.error(f"{self.right_ee_client.node_name} is not connected!")
                res = False
        return res
    
    def is_connected_callback(self, request, response):
        """
        This function is a service callback of function is_connected()
        """
        data = json.loads(request.request)
        if "timeout_sec" in data:
            timeout_sec = data["timeout_sec"]
            self.logger.info(f"Check is_connected with timeout_sec: {timeout_sec}")
            result = self.is_connected(timeout_sec)
            response.response = json.dumps(
                {
                    "success": True,
                    "is_connected": result
                }
            )
        else:
            self.logger.error(f"Check is_connected failed, no timeout_sec in request")
            response.response = json.dumps(
                {
                    "success": False,
                    "is_connected": False
                }
            )
        return response
    
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
    
    def clear_robot_error_callback(self, request, response):
        """
        This function is a service callback of function clear_robot_error()
        """
        self.logger.info(f"Clear robot error")
        result = self.clear_robot_error()
        response.response = json.dumps(
            {
                "success": True,
                "is_cleared": result
            }
        )
        return response
    
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
    
    def set_drag_mode_callback(self, request, response):
        """
        This function is a service callback of function set_drag_mode()
        """
        data = json.loads(request.request)
        if "enable_drag" in data:
            enable_drag = data["enable_drag"]
            self.logger.info(f"Set drag mode: {enable_drag}")
            result = self.set_drag_mode(enable_drag)
            response.response = json.dumps(
                {
                    "success": True,
                    "is_set": result
                }
            )
        else:
            self.logger.error(f"Set drag mode failed, no enable_drag in request")
            response.response = json.dumps(
                {
                    "success": False,
                    "is_set": False
                }
            )
        return response
    
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
    
    def set_safe_level_callback(self, request, response):
        """
        This function is a service callback of function set_safe_level()
        """
        data = json.loads(request.request)
        if "safe_level" in data:
            safe_level = data["safe_level"]
            self.logger.info(f"Set safe level: {safe_level}")
            result = self.set_safe_level(safe_level)
            response.response = json.dumps(
                {
                    "success": True,
                    "is_set": result
                }
            )
        else:
            self.logger.error(f"Set safe level failed, no safe_level in request")
            response.response = json.dumps(
                {
                    "success": False,
                    "is_set": False
                }
            )
        return response

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

        if self._end_effector_type is not None:
            if not target_left_ee_client.is_connected():
                self.logger.error(f"{target_left_ee_client.node_name} is not connected!")
                return False
        # if target_right_ee_client is None:
        #     self.logger.error(f"Right_ee_client_{target_name} is not initialized.")
        #     return False
        if self._end_effector_type is not None:
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

        if self._end_effector_type is not None:
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
    
    def set_sim_mode_callback(self, request, response):
        """
        This function is a service callback of function set_sim_mode()
        """
        data = json.loads(request.request)
        if "enable_sim" in data:
            enable_sim = data["enable_sim"]
            self.logger.info(f"Set sim mode: {enable_sim}")
            result = self.set_sim_mode(enable_sim)
            response.response = json.dumps(
                {
                    "success": True,
                    "is_set": result
                }
            )
        else:
            self.logger.error(f"Set sim mode failed, no enable_sim in request")
            response.response = json.dumps(
                {
                    "success": False,
                    "is_set": False
                }
            )
        return response
    
    def set_ee_qpos(self, name: str, qpos: np.ndarray) -> bool:
        """控制end effector运动

        Args:
            name (str): ["left_arm", "right_arm"]
            qpos (np.ndarray): 期望位置 0~100

        Returns:
            bool
        """
        qpos_ = np.array(qpos)
        if not self._has_arm_ee_client(name, False):
            return False
        # if not self._check_qpos_dimension(name, qpos_):
        #     return False
        return self._get_arm_ee_client(name).set_pos(qpos_)
    
    def set_ee_qpos_callback(self, request, response):
        """
        This function is a service callback of function set_ee_qpos()

        Note:
            -qpos in the request should be list, remember before sending the request, convert
            np.ndarray to list.
            -remember to use _check_qpos_dimension() to check the dimension of qpos before send request
        """
        data = json.loads(request.request)
        if "name" in data and "qpos" in data:
            name = data["name"]
            qpos = np.array(data["qpos"])
            self.logger.info(f"Set {name} ee qpos: {qpos}")
            result = self.set_ee_qpos(name, qpos)
            response.response = json.dumps(
                {
                    "success": True,
                    "is_set": result
                }
            )
        else:
            self.logger.error(f"Set ee qpos failed, no name or qpos in request")
            response.response = json.dumps(
                {
                    "success": False,
                    "is_set": False
                }
            )
        return response
    
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
    
    def go_home_callback(self, request, response):
        """
        This function is a service callback of function go_home()
        """
        data = json.loads(request.request)
        if "speed_ratio" in data:
            speed_ratio = data["speed_ratio"]
            self.logger.info(f"Go home with speed_ratio: {speed_ratio}")
            result = self.go_home(speed_ratio)
            response.response = json.dumps(
                {
                    "success": True,
                    "is_moved": result
                }
            )
        else:
            self.logger.error(f"Go home failed, no speed_ratio in request")
            response.response = json.dumps(
                {
                    "success": False,
                    "is_moved": False
                }
            )
        return response

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
        # if not self._check_qpos_limits("full_body", home_qpos):
        #     return False
        self.w1_params.home_qpos = home_qpos
        if save_to_file:
            return self._save_home_qpos()
        return True
    
    def set_home_qpos_callback(self, request, response):
        """
        This function is a service callback of function set_home_qpos()

        Note:
            -home_qpos in the request should be list, remember before sending the request, convert
            np.ndarray to list.
            -remember to use _check_qpos_limits() to check the limits of home_qpos before send request
        """
        data = json.loads(request.request)
        if "home_qpos" in data:
            home_qpos = np.array(data["home_qpos"])
            if "save_to_file" in data:
                save_to_file = data["save_to_file"]
                result = self.set_home_qpos(home_qpos, save_to_file)
                self.logger.info(f"Set home qpos: {home_qpos}, save_to_file: {save_to_file}")
            else:
                result = self.set_home_qpos(home_qpos)
                self.logger.info(f"Set home qpos: {home_qpos}")
            
            response.response = json.dumps(
                {
                    "success": True,
                    "is_set": result
                }
            )
        else:
            self.logger.error(f"Set home qpos failed, no home_qpos in request")
            response.response = json.dumps(
                {
                    "success": False,
                    "is_set": False
                }
            )
        return response
    
    def get_home_qpos(self) -> np.ndarray:
        return self.w1_params.home_qpos
    
    def stop_move(self) -> bool:
        """停止发送streaming指令

        Returns:
            bool: 
        """
        self._stop_move_event.set()
        self.logger.info("Stop move by user!")
        return True
    
    def stop_move_callback(self, request, response):
        """
        This function is a service callback of function stop_move()
        """
        self.logger.info(f"Stop move")
        result = self.stop_move()
        response.response = json.dumps(
            {
                "success": True,
                "is_stopped": result
            }
        )
        return response
    


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
        # check enable
        if self.robot_client.status in (Status.Init, Status.Running, Status.Error):
            self.logger.error(f"Robot client not in idle. ({self.robot_client.status})")
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

        # ee trigger
        def left_arm_ee_trigger():
            nonlocal count, stop_ee_thread
            left_arm_ee_trigger_list_cp = deepcopy(left_arm_ee_trigger_list)
            left_arm_ee_trigger_list_cp.reverse()
            target_count, target_ee_qpos = left_arm_ee_trigger_list_cp.pop()
            cur_count = -1
            while not stop_ee_thread:
                if count == target_count and count != cur_count:
                    self.left_ee_client.set_pos(np.array(target_ee_qpos))
                    self.logger.debug(f"left_arm_ee has triggered: {target_count} {target_ee_qpos}.")
                    if left_arm_ee_trigger_list_cp:
                        target_count, target_ee_qpos = left_arm_ee_trigger_list_cp.pop()
                    cur_count = count
                time.sleep(self.TIME_STEP / 10)

        def right_arm_ee_trigger():
            nonlocal count, stop_ee_thread
            right_arm_ee_trigger_list_cp = deepcopy(right_arm_ee_trigger_list)
            right_arm_ee_trigger_list_cp.reverse()
            target_count, target_ee_qpos = right_arm_ee_trigger_list_cp.pop()
            cur_count = -1
            while not stop_ee_thread:
                if count == target_count and count != cur_count:
                    self.right_ee_client.set_pos(np.array(target_ee_qpos))
                    self.logger.debug(f"right_arm_ee has triggered: {target_count} {target_ee_qpos}.")
                    if right_arm_ee_trigger_list_cp:
                        target_count, target_ee_qpos = right_arm_ee_trigger_list_cp.pop()
                    cur_count = count
                time.sleep(self.TIME_STEP / 10)

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
        end_iter = qpos_array.shape[0] - 1
        self.robot_client.start_joint_streaming_request(MotionGroup.WholeBody)
        self.logger.info("Start streaming.")
        t0 = time.time()
        last_qpos = cur_qpos
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

            # self.logger.debug(f"planner_cnt: {self.planner_cnt}")
            if not self.robot_client.send_streaming_command(qpos.tolist(), 
                                                            streaming_finished):
                self.logger.critical("Streaming command send failed.")
                return False
            self.robot_client.wait_second(self.TIME_STEP)
            count += 1
            last_qpos = qpos
        self.robot_client.wait_request_finish()

        self.logger.info("Finish streaming. " +
                        f"Ideal duration should be {(count-1) * self.TIME_STEP}s, " +
                        f"and actual duration is {time.time() - t0}s.")
        
        stop_ee_thread = True
        if left_arm_ee_trigger_thread is not None:
            left_arm_ee_trigger_thread.join()
        if right_arm_ee_trigger_thread is not None:
            right_arm_ee_trigger_thread.join()

        # check reach within timeout
        targt_qpos = qpos_array[-1]
        t1 = time.time()
        while time.time() - t1 < reach_timeout:
            res, cur_qpos = self.get_current_qpos()
            if not res: 
                return False
            if np.allclose(cur_qpos, targt_qpos, atol=np.deg2rad(1)):
                return True
            time.sleep(5*self.TIME_STEP)

        self.logger.error(f"Current qpos does NOT reach the target qpos until {reach_timeout}s, " +
                          f"qpos error(deg): {np.rad2deg(cur_qpos - targt_qpos)}.")
        return False
    
    def execute_joint_traj_callback(self, request, response):
        data = json.loads(request.request)
        if "qpos_array" in data:
            qpos_array = np.array(data["qpos_array"])
            if "left_arm_ee_trigger_list" in data:
                left_arm_ee_trigger_list = data["left_arm_ee_trigger_list"]
            else:
                left_arm_ee_trigger_list = None
            if "right_arm_ee_trigger_list" in data:
                right_arm_ee_trigger_list = data["right_arm_ee_trigger_list"]
            else:
                right_arm_ee_trigger_list = None
            if "reach_timeout" in data:
                reach_timeout = data["reach_timeout"]
            else:
                reach_timeout = 1.0
            
            result = self.execute_joint_traj(qpos_array, 
                                             left_arm_ee_trigger_list, 
                                             right_arm_ee_trigger_list,
                                             reach_timeout)
            self.logger.info("Execute joint traj")
            response.response = json.dumps(
                {
                    "success": True,
                    "is_executed": result
                }
            )
        else:
            self.logger.error(f"Execute joint traj failed, no qpos_array in request")
            response.response = json.dumps(
                {
                    "success": False,
                    "is_executed": False
                }
            )
        return response
    
    def start_moving(self):

        self.controller_client.reset_stop_dynamic_moving_flag()

        start_qpos = self.get_current_qpos()[1]
        self.robot_client.send_streaming_command(position = start_qpos.tolist(),
                                                             streaming_finished = True)
        if self.robot_client.status in (Status.Init, Status.Running, Status.Error):
            self.logger.error(f"Robot client not in idle. ({self.robot_client.status})")
            return False
        self.robot_client.start_joint_streaming_request(MotionGroup.WholeBody)
        record_new_traj_time = None
        current_traj_finish = False
        last_qpos = None
        # input("press enter to continue")
        while True:
            if self.controller_client.get_stop_dynamic_moving_flag():
                self.logger.info("Stop dynamic moving by controller client!")
                last_qpos = self.get_current_qpos()[1]
                self.robot_client.send_streaming_command(position = last_qpos.tolist(),
                                                             streaming_finished = True)
                self.robot_client.wait_request_finish()
                break

            # start_time = time.time()
            received_traj = self.controller_client.get_trajectory_numpy()
            
            # self.logger.debug(f"last qpos: {last_qpos[6:13] if last_qpos is not None else None}")
            # self.logger.debug(f"received first qpos: {received_traj[0, 6:13] if received_traj is not None else None}")
            # self.logger.debug(f"current received time: {time.time()}")
            if received_traj is None:
                # self.logger.debug("No new trajectory.")
                current_traj_finish = False
                continue
            else:
                current_traj_finish = False
                self.logger.debug(f"Received new trajectory with length {received_traj.shape}")

            if current_traj_finish:
                self.logger.debug("current traj finish True")
                continue

            if self._stop_move_event.is_set():
                self._stop_move_event.clear()

            res, cur_qpos = self.get_current_qpos()
            if not res:
                self.logger.debug("get qpos failed")
                continue

            

            # TODO: check first waypoint is close to current qpos
            # if not np.allclose(cur_qpos, received_traj[0], atol=np.deg2rad(1)):
            #     self.logger.error(f"cur_qpos ({cur_qpos}) is not equal to start_qpos ({received_traj[0]}).")
            #     continue


            count = 1
            end_iter = received_traj.shape[0] - 1
            # self.robot_client.start_joint_streaming_request(MotionGroup.WholeBody)
            self.logger.info("Start streaming.")

            last_qpos = cur_qpos
            # self.logger.debug(f"preparing time: {time.time() - start_time}s")
            # if record_new_traj_time is not None:
            #     self.logger.debug(f"wait new traj time: {time.time() - record_new_traj_time}s")
            #     record_new_traj_time = None

            if self.robot_client.status != Status.Running:
                self.robot_client.start_joint_streaming_request(MotionGroup.WholeBody)
                self.logger.info("Robot client start streaming request again.")

            for qpos in received_traj:
                # self.logger.debug("enter the received traj loop and publish point")
                is_new_traj = self.controller_client.get_is_new_trajectory()
                if is_new_traj:
                    # self.logger.info("Received new trajectory, stop current trajectory execution.")
                    # record_new_traj_time = time.time()
                    # self.robot_client.send_streaming_command(qpos.tolist(),
                    #                                             streaming_finished = False)
                    # # self.robot_client.send_streaming_command(position = last_qpos.tolist(),
                    # #                                          streaming_finished = True)
                    # self.robot_client.wait_second(self.TIME_STEP)
                    # self.robot_client.wait_request_finish()
                    break
                if self._stop_move_event.is_set():
                    self.robot_client.send_streaming_command(position = last_qpos.tolist(),
                                                             streaming_finished = True)
                    self.logger.info(f"Stop sending streaming command")
                    self.robot_client.wait_request_finish()
                    # self.robot_client.wait_second(self.TIME_STEP)
                    # self._stop_move_event.clear()
                    break
                
                if count == end_iter + 1:
                    streaming_finished = True
                else:
                    streaming_finished = False

                if not self.robot_client.send_streaming_command(qpos.tolist(),
                                                                streaming_finished):
                    self.logger.critical("Streaming command send failed.")
                    # current_traj_finish = True
                    # self.robot_client.start_joint_streaming_request(MotionGroup.WholeBody)
                    break
                if streaming_finished:
                    current_traj_finish = True
                    self.robot_client.start_joint_streaming_request(MotionGroup.WholeBody)
                    self.logger.info("Finish current trajectory streaming, wait for new trajectory.")
                self.robot_client.wait_second(self.TIME_STEP)
                count += 1
                last_qpos = qpos

    def start_moving_callback(self, request, response):
        """
        This function is a service callback of function start_moving()
        """
        
        self.start_moving()
        response.response = json.dumps(
            {
                "success": True,
                "is_finished": True
            }
        )
        return response
    
