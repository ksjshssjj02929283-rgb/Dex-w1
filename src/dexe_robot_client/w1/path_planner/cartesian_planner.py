'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

import numpy as np

from ...common import get_logger, RobotType
from ..utils.w1_params import W1Params
from ..path_planner.blend_path import PathR3xSO3Bezier
from .utils.robot_type import *
from .planner_pinocchio import Planner_Pin

from dexe_rp import DexforceW1Kinematics, RobotVersion, EndEffectorType


class CartesianPlanner:

    def __init__(
            self,
            robot_version: RobotVersion,
            end_effector_type: EndEffectorType = EndEffectorType.BrainCo_Revo1_R
    ):
        self.logger = get_logger()

        self._robot_version = robot_version
        self._end_effector_type = end_effector_type

        self.w1_params = W1Params(self._robot_version, self._end_effector_type)

        self._init_kine_solver()
        self._init_pinocchio(self._robot_version, self._end_effector_type)
        self._init_arml_base_T_armr_base()

    def _init_kine_solver(self):
        self.kine = DexforceW1Kinematics()
        self.kine.set_user_qpos_limits(self.w1_params.qpos_lb,
                                       self.w1_params.qpos_ub)

        self.logger.info(f"Initialize kinematic solver for Cartesian Planner")

    def _init_pinocchio(self,
                        robot_version: RobotVersion,
                        end_effector_type: EndEffectorType = None):
        """
        Initialize the pinocchio model and data for the robot.
        """
        pin_robot_type = None
        if robot_version.version_value.startswith("v02"):
            if robot_version.version_value.endswith("i"):
                pin_robot_type = RobotType.W1V02_2
            else:
                pin_robot_type = RobotType.W1V02_1
        elif robot_version.version_value.startswith("v03"):
            pin_robot_type = RobotType.W1V03_1

        self._pin_kine = Planner_Pin(pin_robot_type, end_effector_type)

    def set_arm_tcp(self, name: str, pose: np.ndarray):
        return self.kine.set_tcp(name, pose)

    def _get_arm_kine(self, name: str):
        if name == "left_arm":
            return self.kine.left_arm_kine
        elif name == "right_arm":
            return self.kine.right_arm_kine
        else:
            raise ValueError(f"Invalid arm name: {name}")

    def _init_arml_base_T_armr_base(self):
        real_qpos = np.zeros(20)
        torso_tf, head_tf, arml_tf, armr_tf = self._pin_kine.get_Kinematics(
            real_qpos=real_qpos)
        arml_base_tf, armr_base_tf = self._pin_kine.get_arm_base_tf(
            real_qpos=real_qpos)
        self.waist_T_arml_base = np.linalg.inv(torso_tf[-1]) @ arml_base_tf
        self.waist_T_armr_base = np.linalg.inv(torso_tf[-1]) @ armr_base_tf
        self.armr_base_T_arml_base = np.linalg.inv(
            self.waist_T_armr_base) @ self.waist_T_arml_base
        self.arml_base_T_armr_base = np.linalg.inv(self.armr_base_T_arml_base)

    def set_left_right_relationship(self,
                                    left_tcp_pose: np.ndarray,
                                    right_tcp_pose: np.ndarray,
                                    is_local: bool = True):
        if is_local:
            waist_T_left_tcp = self.waist_T_arml_base @ left_tcp_pose
            waist_T_right_tcp = self.waist_T_armr_base @ right_tcp_pose

            self.left_tcp_T_right_tcp = np.linalg.inv(
                waist_T_left_tcp) @ waist_T_right_tcp
            self.right_tcp_T_left_tcp = np.linalg.inv(self.left_tcp_T_right_tcp)
        else:
            self.left_tcp_T_right_tcp = np.linalg.inv(
                left_tcp_pose) @ right_tcp_pose
            self.right_tcp_T_left_tcp = np.linalg.inv(self.left_tcp_T_right_tcp)

    def generate_right_arm_traj_from_left(self,
                                          left_arm_ee_traj: np.ndarray,
                                          is_local: bool = True):
        if is_local:
            right_arm_ee_traj = np.zeros_like(left_arm_ee_traj)
            for i in range(left_arm_ee_traj.shape[0]):
                left_tcp_pose = left_arm_ee_traj[i]
                right_tcp_pose = self.armr_base_T_arml_base @ left_tcp_pose @ self.left_tcp_T_right_tcp
                right_arm_ee_traj[i] = right_tcp_pose
        else:
            right_arm_ee_traj = np.zeros_like(left_arm_ee_traj)
            for i in range(left_arm_ee_traj.shape[0]):
                left_tcp_pose = left_arm_ee_traj[i]
                right_tcp_pose = left_tcp_pose @ self.left_tcp_T_right_tcp
                right_arm_ee_traj[i] = right_tcp_pose
        return right_arm_ee_traj

    def generate_left_arm_traj_from_right(self,
                                          right_arm_ee_traj: np.ndarray,
                                          is_local: bool = True):
        if is_local:
            left_arm_ee_traj = np.zeros_like(right_arm_ee_traj)
            for i in range(right_arm_ee_traj.shape[0]):
                right_tcp_pose = right_arm_ee_traj[i]
                left_tcp_pose = self.arml_base_T_armr_base @ right_tcp_pose @ self.right_tcp_T_left_tcp
                left_arm_ee_traj[i] = left_tcp_pose
        else:
            left_arm_ee_traj = np.zeros_like(right_arm_ee_traj)
            for i in range(right_arm_ee_traj.shape[0]):
                right_tcp_pose = right_arm_ee_traj[i]
                left_tcp_pose = right_tcp_pose @ self.right_tcp_T_left_tcp
                left_arm_ee_traj[i] = left_tcp_pose
        return left_arm_ee_traj

    def generate_line_traj_from_tcp_poses(self,
                                          name: str,
                                          waypoint_list: list[np.ndarray],
                                          cartesian_step: float = 0.005,
                                          is_local: bool = True):
        if name not in ["left_arm", "right_arm"]:
            self.logger.error(f"Invalid arm name: {name}")
            return False, None, None

        # prefix_dist_list = [0.]
        # for idx in range(len(waypoint_list) - 1):
        #     dist = np.linalg.norm(waypoint_list[idx + 1][:3, 3] -
        #                           waypoint_list[idx][:3, 3])
        #     prefix_dist_list.append(prefix_dist_list[-1] + dist)
        # total_dist = prefix_dist_list[-1]

        path = PathR3xSO3Bezier(np.array(waypoint_list))
        if not path.calc_path():
            return False, None, None
        pose_sample_array = path.get_sample_by_step(cartesian_step)
        if name == "left_arm":
            left_pose_sample_array = pose_sample_array
            right_pose_sample_array = self.generate_right_arm_traj_from_left(
                left_pose_sample_array, is_local)
        else:
            right_pose_sample_array = pose_sample_array
            left_pose_sample_array = self.generate_left_arm_traj_from_right(
                right_pose_sample_array, is_local)

        return True, left_pose_sample_array, right_pose_sample_array
