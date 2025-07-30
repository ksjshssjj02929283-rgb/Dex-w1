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
import rclpy

from rlia.kit.drive_controllers import W1DriveController
from rlia.kit.drive_controllers.w1_drive_controller import W1Version, W1ArmType

from dexe_robot_client.common import g_data_dir, get_logger
from dexe_robot_client.end_effector_client import (EndEffectorType, 
                          LeftBrainCoHadnClient, RightBrainCoHadnClient, 
                          LeftPGCGripperClient, RightPGCGripperClient)
from dexe_robot_client.robot_client import RobotClientRos2 as RobotClient
from dexe_robot_client.robot_client import MotionGroup

from dexe_robot_client.w1 import TorsoKinmatics, ToppraTrajectoryPlanner, W1ComponentManager
from dexe_robot_client.w1 import interpolate_cartesian_path
from dexe_robot_client.w1 import user_pose_to_xpos, xpos_to_user_pose


class W1Manager:
    _instance = None
    _lock = threading.Lock()

    def __new__(cls, *args, **kwargs):
        with cls._lock:
            if not cls._instance:
                cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self, 
                arm_type: W1ArmType, 
                w1_version: W1Version = W1Version.V2,
                end_effector_type: EndEffectorType = None,
                is_sim: bool = True):
        self.logger = get_logger()

        self.TIME_STEP = 0.01 # 100Hz

        self.component_manager = W1ComponentManager()
        self.arm_ee_names = ["left_arm_ee", "right_arm_ee"]
        self.arm_ee_dofs = None
        self.arm_names = ["left_arm", "right_arm"]

        self.robot_client = None
        self._robot_client_sim = None
        self._robot_client_real = None
        self.left_arm_ee_client = None
        self.right_arm_ee_client = None

        self._is_sim = is_sim
        self._arm_type = arm_type
        self._w1_version = w1_version
        self._end_effector_type = end_effector_type

        self._init_clients()
        self._init_kine_solver()
        self.set_use_sim(is_sim)

        self._home_qpos = np.deg2rad([45, -90, 45, 0, 0, 0,
                                      5, -45, -5, -90, 0, -15, 0,
                                      -5, 45, 5, 90, 0, 15, 0])
        if not self._load_home_qpos():
            raise ValueError
        
        self._stop_move_flag = False
        self._stop_move_lock = threading.Lock()


    def _init_clients(self):
        try:
            if not rclpy.ok():
                rclpy.init()

            if self._is_sim:
                self._robot_client_sim = RobotClient(is_sim = True)
            else:
                self._robot_client_real = RobotClient(is_sim = False)

            if self._end_effector_type == EndEffectorType.BRAINCO_HAND:
                self.left_arm_ee_client = LeftBrainCoHadnClient()
                self.right_arm_ee_client = RightBrainCoHadnClient()
                self.arm_ee_dofs = 6
            elif self._end_effector_type == EndEffectorType.PGC_GRIPPER:
                self.left_arm_ee_client = LeftPGCGripperClient()
                self.right_arm_ee_client = RightPGCGripperClient()
                self.arm_ee_dofs = 1
            elif self._end_effector_type is None:
                self.logger.info(f"No movable end effector")
            else:
                raise ValueError(f"Unsupport end effector type: {self._end_effector_type}")

        except Exception as e:
            self.logger.error(f"ROS2 node create failed: {str(e)}")
            self._destroy_clients()
            raise RuntimeError("Failed to initialize clients") from e
        
        self.logger.info("Initialize ROS2 client")

    def _destroy_clients(self):
        try:
            if self._robot_client_sim is not None:
                self._robot_client_sim.destroy_node()
            if self._robot_client_real is not None:
                self._robot_client_real.destroy_node()
            if self.left_arm_ee_client is not None:
                self.left_arm_ee_client.destroy_node()
            if self.right_arm_ee_client is not None:
                self.right_arm_ee_client.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            self.logger.warning(f"ROS2 node destory failed: {str(e)}")

    def _init_kine_solver(self):
        # TODO: params read from json
        if self._w1_version != W1Version.V2:
            raise ValueError(f"W1Version {self._w1_version} are not support")
        
        # fmt: off
        self.torso_ub = np.deg2rad([75., 5., 90., 90.])
        self.torso_lb = np.deg2rad([-5., -150., -100., -90.])
        self.head_ub = np.deg2rad([90., 45.])
        self.head_lb = np.deg2rad([-90., -25.])
        if self._arm_type == W1ArmType.INDUSTRIAL:
            # left arm
            self.left_arm_ub = np.deg2rad([90., 45., 170., 5., 170., 89., 170.])
            self.left_arm_lb = np.deg2rad([-170., -120., -170., -135., -170., -89., -170.])
            # right arm
            self.right_arm_ub = np.deg2rad([170., 120., 170., 135., 170., 89., 170.])
            self.right_arm_lb = np.deg2rad([-90., -45., -170., -5., -170., -89., -170.])
        elif self._arm_type == W1ArmType.ANTHROPOMORPHIC:
            # left arm
            self.left_arm_ub = np.deg2rad([90., 45., 170., 5., 170., 48., 36.])
            self.left_arm_lb = np.deg2rad([-170., -115., -170., -130., -170., -35., -100.])
            # right arm
            self.right_arm_ub = np.deg2rad([170., 115., 170., 130., 170., 35., 100.])
            self.right_arm_lb = np.deg2rad([-90., -45., -170., -5., -170., -48., -36.])
        else:
            raise ValueError(f"W1ArmType {self._arm_type} are not support")
        # fmt: on
        self.qpos_ub = np.r_[self.torso_ub, 
                            self.head_ub, 
                            self.left_arm_ub, 
                            self.right_arm_ub]
        self.qpos_lb = np.r_[self.torso_lb,
                            self.head_lb,
                            self.left_arm_lb,
                            self.right_arm_lb]

        _ik_nearst_weight = [2., 2., 2., 1., 1., 1., 1.]
        _ik_sample_num = 1600
        # _arm_angle_sample_range = np.deg2rad(45)

        self.torso_kine_solver = TorsoKinmatics()
        self.left_arm_controller = W1DriveController(is_left_arm = True, 
                                                    type_version = self._w1_version, 
                                                    arm_type = self._arm_type)
        self.left_arm_controller.set_joint_limits(upper_limits = self.left_arm_ub,
                                                   lower_limits = self.left_arm_lb)
        self.left_arm_controller.set_ik_nearst_weight(_ik_nearst_weight)
        left_kin_solver = self.left_arm_controller.solver.get_kin_solver()
        left_kin_solver.set_num_samples(_ik_sample_num)
        # left_kin_solver.set_small_range_qt(_arm_angle_sample_range)

        self.right_arm_controller = W1DriveController(is_left_arm = False, 
                                                    type_version = self._w1_version, 
                                                    arm_type = self._arm_type)
        self.right_arm_controller.set_joint_limits(upper_limits = self.right_arm_ub,
                                                   lower_limits = self.right_arm_lb)
        self.right_arm_controller.set_ik_nearst_weight(_ik_nearst_weight)
        right_kin_solver = self.right_arm_controller.solver.get_kin_solver()
        right_kin_solver.set_num_samples(_ik_sample_num)
        # right_kin_solver.set_small_range_qt(_arm_angle_sample_range)

        # fmt: off
        self.default_qvel = np.array([0.3]*self.component_manager.full_body_dofs)
        self.min_qvel = np.array([0.05]*self.component_manager.full_body_dofs)
        self.max_qvel = np.array([1.0]*self.component_manager.full_body_dofs)
        self.max_qacc = np.array([1., 1., 1., 1.,
                                5., 5.,
                                3., 3., 3., 5., 10., 10., 10.,
                                3., 3., 3., 5., 10., 10., 10.])
        # fmt: on
        self.logger.info("Initialize kinematic solver")

    def _has_arm_ee_client(self, name: str) -> bool:
        if name in self.arm_ee_names:
            if name == "left_arm_ee" and self.left_arm_ee_client is not None:
                return True
            if name == "right_arm_ee" and self.right_arm_ee_client is not None:
                return True
            # self.logger.warning(f"{name} has not initialized")
            return False

        self.logger.error(f"{self.arm_ee_names} is supported, except {name}")
        return False

    def _get_arm_controller(self, name: str):
        if not self._check_arm_name(name):
            return None
        if name == "left_arm":
            return self.left_arm_controller
        elif name == "right_arm":
            return self.right_arm_controller

    def _get_arm_ee_client(self, name: str):
        if not self._has_arm_ee_client(name):
            return None
        if name == "left_arm_ee":
            return self.left_arm_ee_client
        elif name == "right_arm_ee":
            return self.right_arm_ee_client

    def _check_name(self, name: str) -> bool:
        """仅支持component检查，不包含ee"""
        if name not in self.component_manager.names:
            self.logger.error(f"{self.component_manager.names} is supported, except {name}")
            return False
        return True

    def _check_arm_name(self, name: str) -> bool:
        support_names = self.arm_names
        if name not in support_names:
            self.logger.error(f"{support_names} is supported, except {name}")
            return False
        return True

    def _check_qpos_dimension(self, name: str, qpos) -> bool:
        """支持component和ee的检查"""
        if name in self.arm_ee_names:
            if not self._has_arm_ee_client(name):
                self.logger.error(f"{name} has not initialized")
                return False
            dofs = self.arm_ee_dofs
        elif self._check_name(name):
            dofs = self.component_manager.infos[name]["dofs"]
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
            self.logger.error(f"{name} qpos excceeded limits")
            return False
        return True
    
    def _check_arm_ee_states(self, arm_ee_state_list: list) -> bool:
        if len(arm_ee_state_list) == 0:
            self.logger.error("FIXME")
            return False
        if not all(isinstance(ee_state, (float, list, float)) for ee_state in arm_ee_state_list):
            self.logger.error(f"arm_ee state should be [pre_delay, ee_qpos, post_delay]")
            return False
        if not self._check_qpos_dimension("left_arm_ee", 
                                          [ee_state[1] for ee_state in arm_ee_state_list]):
            return False
        return True

    def _save_home_qpos(self, file_path: str = f"{g_data_dir}/config.json") -> bool:
        file_path_ = pathlib.Path(file_path)
        if not file_path_.exists() or file_path_.stat().st_size == 0:
            file_path_.write_text("{}")

        with open(file_path, "r") as f:
            json_data = json.load(f)

        json_data["home_qpos_deg"] = np.rad2deg(self.get_home_qpos()).tolist()

        with open(file_path, "w") as f:
            json.dump(json_data, f, indent=4)
        
        return True

    def _load_home_qpos(self, file_path: str = f"{g_data_dir}/config.json") -> bool:
        file_path_ = pathlib.Path(file_path)
        if not file_path_.exists() or file_path_.stat().st_size == 0:
            self.logger.warning(f"{file_path} does not exist, would be created automatically")
            if self._save_home_qpos(file_path):
               return self._load_home_qpos(file_path) 
            return False

        with open(file_path, "r") as f:
            json_data = json.load(f)
            home_qpos_deg = json_data.get("home_qpos_deg")
            if home_qpos_deg is not None:
                self.logger.info(f"Load home qpos from {file_path}")
                return self.set_home_qpos(np.deg2rad(home_qpos_deg), False)

        return False

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
            self.logger.error("no qpos to be aligned")
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
    def set_use_sim(self, is_sim) -> bool:
        self._is_sim = is_sim
        if self._is_sim:
            if self._robot_client_sim is None:
                self._robot_client_sim = RobotClient(is_sim=True)
            if self._robot_client_sim.is_connected():
                self.robot_client = self._robot_client_sim
        else:
            if self._robot_client_real is None:
                self._robot_client_real = RobotClient(is_sim=False)
            if self._robot_client_real.is_connected():
                self.robot_client = self._robot_client_real
        return True

    def set_ee_state(self, name: str, state: np.ndarray) -> bool:
        """控制end effector运动

        Args:
            name (str): ["left_arm", "right_arm"]
            state (np.ndarray): 期望位置 0~1

        Returns:
            bool
        """
        state_ = np.array(state)
        if not self._has_arm_ee_client(name):
            return False
        if not self._check_qpos_dimension(name, state_):
            return False
        return self._get_arm_ee_client(name).set_state(state_)

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
        self._home_qpos = home_qpos
        if save_to_file:
            return self._save_home_qpos()
        return True

    def get_home_qpos(self) -> np.ndarray:
        return self._home_qpos
    
    def get_qpos_limits(self, name: str = "full_body") -> tuple[bool, np.ndarray, np.ndarray]:
        """获取特定部件(支持ee)的关节极限

        Args:
            name (str, optional): ["full_body", "torso", "head", "left_arm", "right_arm", 
                                    "left_arm_ee", "right_arm_ee"]. Defaults to "full_body".

        Returns:
            tuple[bool, np.ndarray, np.ndarray]: 状态码；下极限；上极限
        """
        if name in self.arm_ee_names:
            if not self._has_arm_ee_client(name):
                return False, None, None
            qpos_lb = np.array([0.]*self.arm_ee_dofs)
            qpos_ub = np.array([1.]*self.arm_ee_dofs)

        elif not self._check_name(name):
            return False, None, None
        else:
            qpos_lb = self.qpos_lb[self.component_manager.infos[name]["index"]]
            qpos_ub = self.qpos_ub[self.component_manager.infos[name]["index"]]
        return True, qpos_lb, qpos_ub

    def get_current_qpos(self, name: str = "full_body") -> tuple[bool, np.ndarray]:
        """获取特定部件(支持ee)的当前关节位置

        Args:
            name (str, optional): ["full_body", "torso", "head", "left_arm", "right_arm", 
                                    "left_arm_ee", "right_arm_ee"]. Defaults to "full_body".

        Returns:
            tuple[bool, np.ndarray]
        """
        support_names = list(self.component_manager.names) + self.arm_ee_names
        if name not in support_names:
            self.logger.error(f"{support_names} is supported, except {name}")
            return False, None
        
        if name in self.arm_ee_names:
            if not self._has_arm_ee_client(name):
                return False, None
            return self._get_arm_ee_client(name).get_state()
        
        robot_info = self.robot_client.get_info()
        cur_qpos = np.array(robot_info.joint_position)
        if cur_qpos is None:
            self.logger.error(f"Get current robot info failed: {robot_info.status}")
            return False, None
        
        if name == "full_body":
            return True, cur_qpos
        return True, cur_qpos[self.component_manager.infos[name]["index"]]

    def get_current_xpos(self, name: str) -> tuple[bool, np.ndarray]:
        arm_controller = self._get_arm_controller(name)
        if arm_controller is None:
            return False, None

        res, cur_qpos = self.get_current_qpos(name)
        if not res:
            return False, None
        
        res, cur_xpos = arm_controller.get_fk(qpos=cur_qpos)
        if not res:
            self.logger.error(f"{name} solve fk failed")
            return False, None
        return True, cur_xpos

    def set_tcp(self, name: str, xpos:np.ndarray) -> bool:
        # TODO: set tcp in robot client too
        arm_controller = self._get_arm_controller(name)
        if arm_controller is None:
            return False
        arm_controller.set_tcp(xpos=xpos)
        return True

    def get_tcp(self, name: str) -> tuple[bool, np.ndarray]:
        arm_controller = self._get_arm_controller(name)
        if arm_controller is None:
            return False, None
        return True, arm_controller.get_tcp()

    def go_home(self, speed_ratio: float = 1.0) -> bool:
        """从当前点MoveJ移动到home qpos

        Args:
            speed_ratio (float, optional): 0.1~1.0. Defaults to 1.0.

        Returns:
            bool: 
        """
        target_qpos = deepcopy(self._home_qpos)
        target_torso_qpos = target_qpos[self.component_manager.torso_index]
        target_head_qpos = target_qpos[self.component_manager.head_index]
        target_left_arm_qpos = target_qpos[self.component_manager.left_arm_index]
        target_right_arm_qpos = target_qpos[self.component_manager.right_arm_index]

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
        with self._stop_move_lock:
            self._stop_move_flag = True
        self.logger.info("Stop move by user")
        return True

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

        dofs = self.component_manager.infos[name]["dofs"]
        index = self.component_manager.infos[name]["index"]
        vel_limits = self.max_qvel[index]
        acc_limits = self.max_qacc[index]
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
        arm_controller = self._get_arm_controller(name)
        if arm_controller is None:
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
        for i, cur_xpos in enumerate(xpos_sample_array):
            # if i == 1:
            #     arm_controller.set_use_small_range_first(True)
            res_ik, cur_qpos = arm_controller.get_ik(xpos=cur_xpos, 
                                                     qpos_seed=cur_qseed)
            if not np.allclose(cur_qpos, cur_qseed, atol=np.deg2rad(90)):
                self.logger.error(f"qpos traj is not continues\n" + \
                                  f"cur_qpos: {np.rad2deg(cur_qpos)}\n" + \
                                  f"cur_qseed: {np.rad2deg(cur_qseed)}")
                return False, None
            if not res_ik:
                self.logger.error(f"{name} solve ik failed, cur_qseed: {np.rad2deg(cur_qseed)}")
                return False, None
            qpos_list.append(cur_qpos)
            cur_qseed = cur_qpos
        # arm_controller.set_use_small_range_first(False)

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
        self.logger.debug(f"plan_arm_ee_trigger result: {segment_list}")
        return True, segment_list

    def execute_joint_traj(self, 
                           qpos_array: np.ndarray,
                           left_arm_ee_trigger_list: list = None,
                           right_arm_ee_trigger_list: list = None,
                           reach_timeout: float = 2.0) -> bool:
        """按10ms间隔执行关节轨迹

        Args:
            qpos_array (np.ndarray): 关节轨迹
            left_arm_ee_trigger_list (list, optional): 左末端执行器触发序列. Defaults to None.
            right_arm_ee_trigger_list (list, optional): 右末端执行器触发序列. Defaults to None.
            reach_timeout (float, optional): 发送完指令后的到位检测timeout. Defaults to 2.0.

        Returns:
            bool: 
        """
        # reset stop move flag
        self._stop_move_flag = False

        # check current qpos and start qpos
        res, cur_qpos = self.get_current_qpos()
        if not res: 
            return False
        if not np.allclose(cur_qpos, qpos_array[0], atol=np.deg2rad(1)):
            self.logger.error(f"cur_qpos ({cur_qpos}) is not equal to start_qpos ({qpos_array[0]})")
            return False

        count = 1
        stop_thread = False

        # ee trigger
        def left_arm_ee_trigger():
            nonlocal count, stop_thread
            left_arm_ee_trigger_list_cp = deepcopy(left_arm_ee_trigger_list)
            left_arm_ee_trigger_list_cp.reverse()
            target_count, target_ee_qpos = left_arm_ee_trigger_list_cp.pop()
            cur_count = -1
            while not stop_thread:
                if count == target_count and count != cur_count:
                    self.left_arm_ee_client.set_state(np.array(target_ee_qpos))
                    self.logger.debug(f"left_arm_ee has triggered: {target_count} {target_ee_qpos}")
                    if left_arm_ee_trigger_list_cp:
                        target_count, target_ee_qpos = left_arm_ee_trigger_list_cp.pop()
                    cur_count = count
                time.sleep(self.TIME_STEP / 10)

        def right_arm_ee_trigger():
            nonlocal count, stop_thread
            right_arm_ee_trigger_list_cp = deepcopy(right_arm_ee_trigger_list)
            right_arm_ee_trigger_list_cp.reverse()
            target_count, target_ee_qpos = right_arm_ee_trigger_list_cp.pop()
            cur_count = -1
            while not stop_thread:
                if count == target_count and count != cur_count:
                    self.right_arm_ee_client.set_state(np.array(target_ee_qpos))
                    self.logger.debug(f"right_arm_ee has triggered: {target_count} {target_ee_qpos}")
                    if right_arm_ee_trigger_list_cp:
                        target_count, target_ee_qpos = right_arm_ee_trigger_list_cp.pop()
                    cur_count = count
                time.sleep(self.TIME_STEP / 10)

        if self._has_arm_ee_client("left_arm_ee") and left_arm_ee_trigger_list is not None:
            if self._has_arm_ee_client("left_arm_ee"):
                left_arm_ee_trigger_thread = threading.Thread(
                    target=left_arm_ee_trigger, daemon=True
                )
                left_arm_ee_trigger_thread.start()

        if self._has_arm_ee_client("right_arm_ee") and right_arm_ee_trigger_list is not None:
            if self._has_arm_ee_client("right_arm_ee"):
                right_arm_ee_trigger_thread = threading.Thread(
                    target=right_arm_ee_trigger, daemon=True
                )
                right_arm_ee_trigger_thread.start()

        # streaming
        end_iter = qpos_array.shape[0] - 1
        self.robot_client.start_joint_streaming_request(MotionGroup.WholeBody)
        self.logger.info("Start streaming")
        t0 = time.time()
        for qpos in qpos_array:
            if self._stop_move_flag:
                with self._stop_move_lock:
                    self._stop_move_flag = False
                # TODO: use pre_qpos
                self.robot_client.send_streaming_command(qpos.tolist(), True)
                self.logger.info(f"Stop sending streaming command")
                stop_thread = True
                return False

            if count == end_iter + 1:
                streaming_finished = True
            else:
                streaming_finished = False
            self.robot_client.send_streaming_command(qpos.tolist(), 
                                                     streaming_finished)
            self.robot_client.wait_second(self.TIME_STEP)
            count += 1
        self.logger.info(f"Ideal duration should be {(count-1) * self.TIME_STEP}s, " +
                            f"and actual duration is {time.time() - t0}s")
        self.logger.info("Finish streaming")
        
        stop_thread = True
        # TODO: join thread?

        # check reach within timeout
        targt_qpos = qpos_array[-1]
        t1 = time.time()
        while time.time() - t1 < reach_timeout:
            res, cur_qpos = self.get_current_qpos()
            if not res: 
                continue
            if np.allclose(cur_qpos, targt_qpos, atol=np.deg2rad(1)):
                return True
            time.sleep(5*self.TIME_STEP)

        self.logger.error(f"Current qpos does NOT reach the target qpos until {reach_timeout}s, " +
                          f"qpos error: {np.rad2deg(cur_qpos - targt_qpos)} deg")
        return False


    ########## demo定制API ###########
    def get_current_pose(self, name: str) -> tuple[bool, np.ndarray]:
        res, cur_xpos = self.get_current_xpos(name)
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
            self.logger.debug(f"No waypoints in {name}")
            return 0, None, None, None
        
        if not all(isinstance(pos, list) for pos in pos_list):
            self.logger.error(f"{pos_list} not a 2d-list")
            return -1, None, None, None
        if not is_cartesian and not self._check_qpos_dimension(name, pos_list):
            return -1, None, None, None
        if is_cartesian and not all(len(pos) == 6 for pos in pos_list):
            self.logger.error(f"Dimension of {pos_list[0]} != 6")
            return -1, None, None, None

        speed_ratio_list = waypoints_dict.get(f"{name}_speed_ratio")
        if not speed_ratio_list: # None or []
            speed_ratio_list = [speed_ratio] * len(pos_list)
        elif len(speed_ratio_list) != len(pos_list):
            self.logger.error(f"{name} qpos and speed_ratio dimension mismatch")
            return -1, None, None, None
        # 将None元素替换为global_speed_ratio
        speed_ratio_list = [sr if sr is not None else speed_ratio
                            for sr in speed_ratio_list]

        default_qvel = self.default_qvel[self.component_manager.infos[name]["index"]]
        min_qvel = self.min_qvel[self.component_manager.infos[name]["index"]]
        max_qvel = self.max_qvel[self.component_manager.infos[name]["index"]]
        qvel_limits_array = np.clip(np.outer(speed_ratio_list, default_qvel), 
                                    min_qvel, max_qvel)

        # 额外读取ee的输入
        arm_ee_state_list = None
        if name in self.arm_names:
            arm_ee_state_list = waypoints_dict.get(f"{name}_ee")
            if not arm_ee_state_list: # None or []
                arm_ee_state_list = None
            elif len(arm_ee_state_list) != len(pos_list):
                self.logger.error(f"{name} qpos and arm_ee_state dimension mismatch")
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
        qvel_limits_array_ = np.vstack([self.default_qvel[self.component_manager.infos[name]["index"]],
                                        qvel_limits_array])
        if is_cartesian:
            res_fk, cur_xpos = self._get_arm_controller(name).get_fk(cur_qpos)
            if not res_fk:
                self.logger.error(f"{name} solve fk failed")
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
            res_fk, cur_xpos = self._get_arm_controller(name).get_fk(cur_qpos)
            if not res_fk:
                self.logger.error(f"{name} solve fk failed")
                return False, None, None
            pos_raw_array_ = np.vstack([xpos_to_user_pose(cur_xpos), pos_raw_array])

        qvel_limits_array_ = np.vstack([self.default_qvel[self.component_manager.infos[name]["index"]],
                                        qvel_limits_array])
        arm_ee_state_list_ = [[0., cur_ee_qpos.tolist(), 0.]] + arm_ee_state_list

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
                                                                            qvel_limits_array_, 
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

    def move_joint_full_body(self, 
                             waypoints_dict: dict, 
                             speed_ratio: float = 1.0) -> bool:
        """规划各部件独立关节空间运动
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
        self.logger.debug(f"waypoints_dict: {waypoints_dict}")

        res, cur_qpos = self.get_current_qpos()
        if not res:
            return False
        cur_torso_qpos = cur_qpos[self.component_manager.torso_index]
        cur_head_qpos = cur_qpos[self.component_manager.head_index]
        cur_left_arm_qpos = cur_qpos[self.component_manager.left_arm_index]
        cur_right_arm_qpos = cur_qpos[self.component_manager.right_arm_index]
        
        # torso
        res_torso, torso_qpos_array = self._plan_to_waypoint("torso", 
                                                             cur_torso_qpos, 
                                                             waypoints_dict, 
                                                             speed_ratio)
        if not res_torso:
            return False
        
        # head
        res_head, head_qpos_array = self._plan_to_waypoint("head", 
                                                             cur_head_qpos, 
                                                             waypoints_dict, 
                                                             speed_ratio)
        if not res_head:
            return False

        # left_arm
        left_arm_ee_trigger_list = None
        if not self._has_arm_ee_client("left_arm_ee"):
            (res_left_arm, 
            left_arm_qpos_array) = self._plan_to_waypoint("left_arm", 
                                                        cur_left_arm_qpos, 
                                                        waypoints_dict, 
                                                        speed_ratio)
            if not res_left_arm:
                return False
        else:
            res, cur_left_arm_ee_qpos = self.get_current_qpos("left_arm_ee")
            if not res:
                return False
            (res_left_arm, 
            left_arm_qpos_array,
            left_arm_ee_trigger_list) = self._plan_to_waypoint_with_ee("left_arm", 
                                                                       cur_left_arm_qpos, 
                                                                       cur_left_arm_ee_qpos, 
                                                                       waypoints_dict, 
                                                                       speed_ratio)
            if not res_left_arm:
                return False

        # right_arm
        right_arm_ee_trigger_list = None
        if not self._has_arm_ee_client("right_arm_ee"):
            (res_right_arm, 
            right_arm_qpos_array) = self._plan_to_waypoint("right_arm", 
                                                        cur_right_arm_qpos, 
                                                        waypoints_dict, 
                                                        speed_ratio)
            if not res_right_arm:
                return False
        else:
            res, cur_right_arm_ee_qpos = self.get_current_qpos("right_arm_ee")
            if not res:
                return False
            (res_right_arm, 
            right_arm_qpos_array,
            right_arm_ee_trigger_list) = self._plan_to_waypoint_with_ee("right_arm", 
                                                                       cur_right_arm_qpos, 
                                                                       cur_right_arm_ee_qpos, 
                                                                       waypoints_dict, 
                                                                       speed_ratio)
            if not res_right_arm:
                return False

        # align
        res_align, total_qpos_array = self._qpos_align(torso_qpos_array, 
                                                       head_qpos_array, 
                                                       left_arm_qpos_array, 
                                                       right_arm_qpos_array)
        if not res_align:
            return False
        # execute
        return self.execute_joint_traj(total_qpos_array, 
                                       left_arm_ee_trigger_list, 
                                       right_arm_ee_trigger_list)

    def move_linear_full_body(self,
                              waypoints_dict: dict, 
                              left_qpos_seed: np.ndarray = None,
                              right_qpos_seed: np.ndarray = None,
                              cartesian_step: float = 0.05,
                              speed_ratio: float = 1.0) -> bool:
        """规划各部件独立关节空间运动，arm执行笛卡尔空间运动
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
        self.logger.debug(f"waypoints_dict: {waypoints_dict}")

        res, cur_qpos = self.get_current_qpos()
        if not res:
            return False
        cur_torso_qpos = cur_qpos[self.component_manager.torso_index]
        cur_head_qpos = cur_qpos[self.component_manager.head_index]
        cur_left_arm_qpos = cur_qpos[self.component_manager.left_arm_index]
        cur_right_arm_qpos = cur_qpos[self.component_manager.right_arm_index]

        if left_qpos_seed is None:
            left_qpos_seed = cur_left_arm_qpos
        if right_qpos_seed is None:
            right_qpos_seed = cur_right_arm_qpos

        # torso
        res_torso, torso_qpos_array = self._plan_to_waypoint("torso", 
                                                             cur_torso_qpos, 
                                                             waypoints_dict, 
                                                             speed_ratio)
        if not res_torso:
            return False
        
        # head
        res_head, head_qpos_array = self._plan_to_waypoint("head", 
                                                             cur_head_qpos, 
                                                             waypoints_dict, 
                                                             speed_ratio)
        if not res_head:
            return False

        # left_arm
        left_arm_ee_trigger_list = None
        if not self._has_arm_ee_client("left_arm_ee"):
            (res_left_arm, 
            left_arm_qpos_array) = self._plan_to_waypoint("left_arm", 
                                                        cur_left_arm_qpos, 
                                                        waypoints_dict, 
                                                        speed_ratio,
                                                        is_cartesian=True,
                                                        qpos_seed=left_qpos_seed,
                                                        cartesian_step=cartesian_step)
            if not res_left_arm:
                return False
        else:
            res, cur_left_arm_ee_qpos = self.get_current_qpos("left_arm_ee")
            if not res:
                return False
            (res_left_arm, 
            left_arm_qpos_array,
            left_arm_ee_trigger_list) = self._plan_to_waypoint_with_ee("left_arm", 
                                                                       cur_left_arm_qpos, 
                                                                       cur_left_arm_ee_qpos, 
                                                                       waypoints_dict, 
                                                                       speed_ratio,
                                                                       is_cartesian=True,
                                                                       qpos_seed=left_qpos_seed,
                                                                       cartesian_step=cartesian_step)
            if not res_left_arm:
                return False

        # right_arm
        right_arm_ee_trigger_list = None
        if not self._has_arm_ee_client("right_arm_ee"):
            (res_right_arm, 
            right_arm_qpos_array) = self._plan_to_waypoint("right_arm", 
                                                        cur_right_arm_qpos, 
                                                        waypoints_dict, 
                                                        speed_ratio,
                                                        is_cartesian=True,
                                                        qpos_seed=right_qpos_seed,
                                                        cartesian_step=cartesian_step)
            if not res_right_arm:
                return False
        else:
            res, cur_right_arm_ee_qpos = self.get_current_qpos("right_arm_ee")
            if not res:
                return False
            (res_right_arm, 
            right_arm_qpos_array,
            right_arm_ee_trigger_list) = self._plan_to_waypoint_with_ee("right_arm", 
                                                                       cur_right_arm_qpos, 
                                                                       cur_right_arm_ee_qpos, 
                                                                       waypoints_dict, 
                                                                       speed_ratio,
                                                                       is_cartesian=True,
                                                                       qpos_seed=right_qpos_seed,
                                                                       cartesian_step=cartesian_step)
            if not res_right_arm:
                return False

        # align
        res_align, total_qpos_array = self._qpos_align(torso_qpos_array, 
                                                       head_qpos_array, 
                                                       left_arm_qpos_array, 
                                                       right_arm_qpos_array)
        if not res_align:
            return False
        
        # execute
        return self.execute_joint_traj(total_qpos_array, 
                                       left_arm_ee_trigger_list, 
                                       right_arm_ee_trigger_list)


if __name__ == "__main__":
    w1_manager = W1Manager(W1ArmType.ANTHROPOMORPHIC, 
                           W1Version.V2, 
                           EndEffectorType.BRAINCO_HAND,
                           is_sim=True)
    # w1_manager.set_ee_state("left_arm_ee", np.array([0.]*6))
    # w1_manager.set_ee_state("right_arm_ee", np.array([0.]*6))
    # w1_manager.go_home(0.5)
    from IPython import embed; embed()