'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

import os
import time
import numpy as np
import json
from copy import deepcopy

from .fcl_checker import FCLChecker
from .ompl_plan import OMPLPlanSingle, OMPLPlanCentralized, OMPLPlanDecoupled
from .traj_coordination import Coordination
from .utils.robot_type import RobotURDF, RobotCollision, get_robot_urdf, get_robot_mesh_path
from .planner_pinocchio import Planner_Pin
from ...common import get_logger, RobotType

from dexe_rp import EndEffectorType

HEAD_DIM = 2
TORSO_DIM = 4


class HybridPlanner:
    """
    High-level motion planning interface for single arm or dual arm planning.

    This class provides a unified planning API that integrates FCL for collision checking,
        OMPL for sampling-based motion planning, and coordination algorithms for multi-arm
        planning. It supports both single-arm and dual-arm motion and trajectory planning.
    """
    arml_dim, armr_dim = None, None
    arml_bounds, armr_bounds = None, None
    ee_type = None
    use_ee = None
    env_json_path = None
    urdf_path = None
    robot_type = None

    @classmethod
    def set_arms_cfg(cls, arml_dim: int, armr_dim: int, arml_bounds: np.ndarray,
                     armr_bounds: np.ndarray):
        """
        Set the configuration for the arms, including dimensions and bounds.

        Args:
            arml_dim (int): Degrees of freedom for the left arm excluding the end effector(Usually 7 for W1)
            armr_dim (int): Degrees of freedom for the right arm excluding the end effector(Usually 7 for W1)
            arml_bounds (np.ndarray, size: 2xN): Bounds for the left arm's joint angles
            armr_bounds (np.ndarray, size: 2xN): Bounds for the right arm's joint angles

        Notes:
            - The bounds should be in the format [[min1, min2, ...], [max1, max2, ...]].
            - The bounds are in radians for joint angles.
        """
        cls.arml_dim = arml_dim
        cls.armr_dim = armr_dim
        if arml_bounds.shape[1] != arml_dim or arml_bounds.shape[0] != 2:
            raise ValueError(
                f"Left arm bounds should be of shape (2, {arml_dim}) but got {arml_bounds.shape}."
            )
        if armr_bounds.shape[1] != armr_dim or armr_bounds.shape[0] != 2:
            raise ValueError(
                f"Right arm bounds should be of shape (2, {armr_dim}) but got {armr_bounds.shape}."
            )

        cls.arml_bounds = arml_bounds
        cls.armr_bounds = armr_bounds

    @classmethod
    def set_robot_cfg(cls,
                      robot_type: RobotType = None,
                      ee_type: EndEffectorType = None):
        """
        Set the configuration for the robot, including the robot type and end effector type.

        Args:
            robot_type (RobotType): The type of the robot (RobotType.W1V02_1 or RobotType.W1V02_2)
            ee_type (EndEffectorType): The type of the end effector

        Notes:
            - If ee_type is None, it will not use the end effector
        """
        if ee_type is not None:
            if ee_type in [
                    EndEffectorType.DH_PGC_140_50,
                    EndEffectorType.BrainCo_Revo1_R, EndEffectorType.Linker_L6
            ]:
                cls.ee_type = ee_type
                cls.use_ee = True
            else:
                cls.ee_type = None
                cls.use_ee = False
        else:
            cls.ee_type = None
            cls.use_ee = False

        if robot_type is not None:
            if robot_type not in RobotType:
                raise ValueError(
                    f"Unsupported robot type: {robot_type}. Please use one of {list(RobotType)}."
                )
            cls.urdf_path = get_robot_urdf(robot_type, ee_type=ee_type)
            cls.robot_type = robot_type
        else:
            raise ValueError(
                f"Robot type must be specified. Please use  one of {list(RobotType)}."
            )

    @classmethod
    def set_env_json(cls, env_json_path: str):
        """
        Set the path of environment JSON file, which will be used to load the environment for fcl checker
        """
        if not os.path.exists(env_json_path):
            raise ValueError(f"Couldn't find the path {env_json_path}")
        cls.env_json_path = env_json_path

    def __init__(self):

        self._init_fcl_checker(self.robot_type, self.ee_type)
        self._init_planner_pinocchio(self.robot_type, self.ee_type)
        self.arml_start, self.armr_start, self.arml_goal, self.armr_goal = None, None, None, None
        self.armlee_wait_time, self.armree_wait_time = 0, 0
        self.armlee_exec_time, self.armree_exec_time = 0, 0
        self.head_qpos = np.zeros(HEAD_DIM)
        self.torso_qpos = np.zeros(TORSO_DIM)

        self.left_traj_max_vel = 0.3
        self.right_traj_max_vel = 0.3

        self.logger = get_logger()

    def _init_fcl_checker(self, robot_type: RobotType,
                          ee_type: EndEffectorType):
        self.fcl_checker = FCLChecker(robot_type, ee_type)
        self.fcl_checker.enable_visualize()

        if self.env_json_path is not None:
            self.fcl_checker.load_env_json(self.env_json_path)

    def _init_planner_pinocchio(self, robot_type: RobotType,
                                ee_type: EndEffectorType):
        self.planner_pin = Planner_Pin(robot_type, ee_type)

    def set_arm_bounds(self,
                       arml_bounds: np.ndarray = None,
                       armr_bounds: np.ndarray = None):
        """
        This function is used to update the arm bounds after the class instance is created.

        Args:
            arml_bounds (np.ndarray, size: 2xN): Bounds for the left arm's joint angles
            armr_bounds (np.ndarray, size: 2xN): Bounds for the right arm's joint angles
        
        Notes:
            - The bounds should be in the format [[min1, min2, ...], [max1, max2, ...]].
            - The bounds are in radians for joint angles.
        """
        if arml_bounds is not None:
            if arml_bounds.shape[1] != self.arml_dim or arml_bounds.shape[
                    0] != 2:
                raise ValueError(
                    f"Left arm bounds should be of shape (2, {self.arml_dim}) but got {arml_bounds.shape}."
                )
            self.arml_bounds = arml_bounds
        if armr_bounds is not None:
            if armr_bounds.shape[1] != self.armr_dim or armr_bounds.shape[
                    0] != 2:
                raise ValueError(
                    f"Right arm bounds should be of shape (2, {self.armr_dim}) but got {armr_bounds.shape}."
                )
            self.armr_bounds = armr_bounds

    def update_fcl_env(self, env_json_path: str):
        self.fcl_checker.load_env_json(env_json_path)

    def set_start_goal(self, arml_start: np.ndarray, armr_start: np.ndarray,
                       arml_goal: np.ndarray, armr_goal: np.ndarray):
        """
        Set the start and goal configurations for the left and right arms.

        Args: 
            arml_start (np.ndarray): Start configuration for the left arm (shape: (arml_dim,))
            armr_start (np.ndarray): Start configuration for the right arm (shape: (armr_dim,))
            arml_goal (np.ndarray): Goal configuration for the left arm (shape: (arml_dim,))
            armr_goal (np.ndarray): Goal configuration for the right arm (shape: (armr_dim,)) 
        """

        self.arml_start, self.armr_start, self.arml_goal, self.armr_goal = arml_start, armr_start, arml_goal, armr_goal

    def set_head_torso(self, head_qpos: np.ndarray, torso_qpos: np.ndarray):

        if head_qpos.shape[0] != HEAD_DIM:
            raise ValueError(
                f"Head position should be of shape ({HEAD_DIM},) but got {head_qpos.shape}."
            )
        if torso_qpos.shape[0] != TORSO_DIM:
            raise ValueError(
                f"Torso position should be of shape ({TORSO_DIM},) but got {torso_qpos.shape}."
            )
        self.head_qpos = head_qpos
        self.torso_qpos = torso_qpos

    def set_end_effector_qpos(self, armlee_qpos: np.ndarray,
                              armree_qpos: np.ndarray):
        if self.use_ee:
            if self.ee_type == EndEffectorType.BrainCo_Revo1_R:
                if armlee_qpos.shape != (6,) or armree_qpos.shape != (6,):
                    raise ValueError(
                        f"For BrainCo_Revo1_R, end_effector_qpos must be a 6-element array, got {armlee_qpos.shape} and {armree_qpos.shape}"
                    )
            if self.ee_type == EndEffectorType.DH_PGC_140_50:
                if armlee_qpos.shape != (2,) or armree_qpos.shape != (2,):
                    raise ValueError(
                        f"For DH_PGC_140_50, end_effector_qpos must be a 2-element array, got {armlee_qpos.shape} and {armree_qpos.shape}"
                    )
            self.armlee_qpos = armlee_qpos
            self.armree_qpos = armree_qpos

    def set_ee_time(self, armlee_wait_time: np.ndarray,
                    armree_wait_time: np.ndarray, armlee_exec_time: np.ndarray,
                    armree_exec_time: np.ndarray):
        """
        Set the time periods for arms

        Args:
            armlee_wait_time (double): waiting time after left arm reach it's goal position
            armree_wait_time (double): waiting time after right arm reach it's goal position
            armlee_exec_time (double): executing time of left arm end effector
            armree_exec_time (double): executing time of right arm end effector
        """
        self.armlee_wait_time, self.armree_wait_time = armlee_wait_time, armree_wait_time
        self.armlee_exec_time, self.armree_exec_time = armlee_exec_time, armree_exec_time

    def set_traj_limits(self, arml_pos_limits, armr_pos_limits, arml_vel_limits,
                        armr_vel_limits, arml_acc_limits, armr_acc_limits):
        if (arml_pos_limits.shape[0] == self.arml_dim and arml_pos_limits.shape[1] == 2) or \
            (arml_pos_limits.shape[0] == 2 and arml_pos_limits.shape[1] == self.arml_dim):
            self.arml_pos_limits = arml_pos_limits
        else:
            raise ValueError(
                f"Left Arm Position limits must be a {self.arml_dim}x2 or 2x{self.arml_dim} array, got {arml_pos_limits.shape}"
            )
        if (armr_pos_limits.shape[0] == self.armr_dim and armr_pos_limits.shape[1] == 2) or \
            (armr_pos_limits.shape[0] == 2 and armr_pos_limits.shape[1] == self.armr_dim):
            self.armr_pos_limits = armr_pos_limits
        else:
            raise ValueError(
                f"Right Arm Position limits must be a {self.armr_dim}x2 or 2x{self.armr_dim} array, got {armr_pos_limits.shape}"
            )

        if arml_vel_limits.shape[0] == self.arml_dim:
            self.arml_vel_limits = arml_vel_limits
        else:
            raise ValueError(
                f"Left Arm Velocity limits must be a {self.arml_dim}-element array, got {arml_vel_limits.shape}"
            )
        if armr_vel_limits.shape[0] == self.armr_dim:
            self.armr_vel_limits = armr_vel_limits
        else:
            raise ValueError(
                f"Right Arm Velocity limits must be a {self.armr_dim}-element array, got {armr_vel_limits.shape}"
            )

        if arml_acc_limits.shape[0] == self.arml_dim:
            self.arml_acc_limits = arml_acc_limits
        else:
            raise ValueError(
                f"Left Arm Acceleration limits must be a {self.arml_dim}-element array, got {arml_acc_limits.shape}"
            )
        if armr_acc_limits.shape[0] == self.armr_dim:
            self.armr_acc_limits = armr_acc_limits
        else:
            raise ValueError(
                f"Right Arm Acceleration limits must be a {self.armr_dim}-element array, got {armr_acc_limits.shape}"
            )

    def attach_object(self,
                      obj_name: str,
                      which_arm: str,
                      arm7_T_object: np.ndarray = None):
        if which_arm not in ['left', 'right']:
            self.logger.error("which_arm must be either 'left' or 'right'.")
            return False
        if which_arm == "left":
            res_attach = self.fcl_checker.attach_object_arml(
                obj_name=obj_name, arml7_T_object=arm7_T_object)
            return res_attach
        if which_arm == "right":
            res_attach = self.fcl_checker.attach_object_armr(
                obj_name=obj_name, armr7_T_object=arm7_T_object)
            return res_attach

    def detach_object(self, obj_name: str, which_arm: str,
                      arm7_transform: np.ndarray):
        if which_arm not in ['left', 'right']:
            self.logger.error("which_arm must be either 'left' or 'right'.")
            return False
        if which_arm == "left":
            res_detach = self.fcl_checker.detach_object_arml(
                obj_name=obj_name, arml7_transform=arm7_transform)
            return res_detach
        if which_arm == "right":
            res_detach = self.fcl_checker.detach_object_armr(
                obj_name=obj_name, armr7_transform=arm7_transform)
            return res_detach

    def transform_obstacle(self, name: str, new_center: np.ndarray,
                           new_quaternion: np.ndarray):
        """
        Transform an existing obstacle in the FCL environment.\
        
        Args:
            name (str): The name of the obstacle to be transformed
            new_center (np.ndarray): The new center position of the obstacle (shape: (3,))
            new_quaternion (np.ndarray): The new orientation of the obstacle as a quaternion (shape: (4,))
        """

        res = self.fcl_checker.transform_obstacle(name=name,
                                                  new_center=new_center,
                                                  new_quaternion=new_quaternion)
        return res

    def disable_obstacle(self, name: str):
        """
        Disable an existing obstacle in the FCL environment.\
        
        Args:
            name (str): The name of the obstacle to be disabled
        """

        res = self.fcl_checker.disable_obstacle(name=name)
        return res

    def enable_obstacle(self, name: str):
        """
        Enable an existing obstacle in the FCL environment.\
        
        Args:
            name (str): The name of the obstacle to be enabled
        """

        res = self.fcl_checker.enable_obstacle(name=name)
        return res

    def plan(self):
        """
        Solve the planning problem which will decide which planner to use based on the start and goal configurations of the arms.

        Returns:
            left_trajectory (np.ndarray): Planned trajectory for the left arm, shape (N, arml_dim)
            right_trajectory (np.ndarray): Planned trajectory for the right arm, shape (N, armr_dim)
            plan_time (float): Total time taken for the planning process
        """
        is_arml_plan = True if np.max(np.abs(self.arml_goal -
                                             self.arml_start)) > 1e-4 else False
        is_armr_plan = True if np.max(np.abs(self.armr_goal -
                                             self.armr_start)) > 1e-4 else False
        if is_arml_plan and not is_armr_plan:
            self.logger.info(
                "----------------using Left arm Single Planner----------------")
            ompl_planner = OMPLPlanSingle(self.arml_dim,
                                          self.arml_bounds,
                                          self.arml_start,
                                          self.arml_goal,
                                          self.fcl_checker,
                                          planner_pin=self.planner_pin,
                                          which_arm='left',
                                          other_arm_qpos=self.armr_start,
                                          use_ee=self.use_ee)
            ompl_planner.set_head_body(self.head_qpos, self.torso_qpos)
            ompl_planner.set_end_effector_qpos(self.armlee_qpos,
                                               self.armree_qpos)
            ompl_planner.set_traj_limits(self.arml_pos_limits,
                                         self.arml_vel_limits,
                                         self.arml_acc_limits)
            left_path, left_trajectory, left_plan_time = ompl_planner.plan(
                planning_time=2.0)
            if left_path is not None:
                return left_trajectory, None, left_plan_time
            else:
                return None, None, None

        if is_armr_plan and not is_arml_plan:
            self.logger.info(
                "----------------using Right arm Single Planner----------------"
            )
            ompl_planner = OMPLPlanSingle(self.armr_dim,
                                          self.armr_bounds,
                                          self.armr_start,
                                          self.armr_goal,
                                          self.fcl_checker,
                                          planner_pin=self.planner_pin,
                                          which_arm='right',
                                          other_arm_qpos=self.arml_start,
                                          use_ee=self.use_ee)
            ompl_planner.set_head_body(self.head_qpos, self.torso_qpos)
            ompl_planner.set_end_effector_qpos(self.armlee_qpos,
                                               self.armree_qpos)
            ompl_planner.set_traj_limits(self.armr_pos_limits,
                                         self.armr_vel_limits,
                                         self.armr_acc_limits)
            right_path, right_trajectory, right_plan_time = ompl_planner.plan(
                planning_time=2.0)
            if right_path is not None:
                return None, right_trajectory, right_plan_time
            else:
                return None, None, None

        if is_arml_plan and is_armr_plan:
            self.logger.info(
                "----------------using Decoupled Planner----------------")
            ompl_planner = OMPLPlanDecoupled(self.arml_dim,
                                             self.armr_dim,
                                             self.arml_bounds,
                                             self.armr_bounds,
                                             self.arml_start,
                                             self.armr_start,
                                             self.arml_goal,
                                             self.armr_goal,
                                             self.fcl_checker,
                                             planner_pin=self.planner_pin,
                                             use_ee=self.use_ee)
            ompl_planner.set_head_body(self.head_qpos, self.torso_qpos)
            ompl_planner.set_traj_limits(self.arml_pos_limits,
                                         self.armr_pos_limits,
                                         self.arml_vel_limits,
                                         self.armr_vel_limits,
                                         self.arml_acc_limits,
                                         self.armr_acc_limits)
            ompl_planner.set_end_effector_qpos(self.armlee_qpos,
                                               self.armree_qpos)
            coordinated_left, coordinated_right, plan_time = ompl_planner.plan()
            if coordinated_left is not None and coordinated_right is not None:
                return coordinated_left, coordinated_right, plan_time
            else:
                self.logger.warning(
                    "Failed to find a trajectory for dual arm Decoupled planning"
                )
                return None, None, None

        if not is_arml_plan and not is_armr_plan:
            self.logger.info(
                "No planning needed, both arms are already at their goal positions."
            )
            return np.atleast_2d(self.arml_start), np.atleast_2d(
                self.armr_start), 0.0

    def coordinate_trajectories_with_EE(
            self, left_traj_list: list[np.ndarray],
            right_traj_list: list[np.ndarray],
            left_ee_trigger_list: list[tuple[int, np.ndarray]],
            right_ee_trigger_list: list[tuple[int, np.ndarray]]):
        """
        Connect multiple trajectories for single arm individually with end effector time consideration 
            and return the coordinated dual arm trajectories with connected trajectories.

        Args:
            left_traj_list (list[np.ndarray]): A list of left arm trajectories to be connected
            right_traj_list (list[np.ndarray]): A list of right arm trajectories to be connected
            left_ee_triger_list (list[tuple[int, np.ndarray]]): A list of tuples for left arm end effector trigger points.
                                                                Each tuple contains (index in trajectory, end effector qpos)
            right_ee_triger_list (list[tuple[int, np.ndarray]]): A list of tuples for right arm end effector trigger points.
                                                                 Each tuple contains (index in trajectory,

        """

        if len(left_traj_list) != len(left_ee_trigger_list):
            raise ValueError(
                f"Count of left arm trajectories {len(left_traj_list)} does not match count of left arm ee trigger points {len(left_ee_trigger_list)}"
            )
        if len(right_traj_list) != len(right_ee_trigger_list):
            raise ValueError(
                f"Count of right arm trajectories {len(right_traj_list)} does not match count of right arm ee trigger points {len(right_ee_trigger_list)}"
            )

        arml_trajectory = np.vstack(left_traj_list)
        armr_trajectory = np.vstack(right_traj_list)
        left_traj_cnt, right_traj_cnt = 0, 0
        left_ee_identifier_list, right_ee_identifier_list = [], []
        for i in range(len(left_ee_trigger_list)):
            left_ee_identifier_list.append(
                (left_traj_cnt + left_ee_trigger_list[i][0],
                 np.asarray(left_ee_trigger_list[i][1])))
            left_traj_cnt += left_traj_list[i].shape[0]
        for i in range(len(right_ee_trigger_list)):
            right_ee_identifier_list.append(
                (right_traj_cnt + right_ee_trigger_list[i][0],
                 np.asarray(right_ee_trigger_list[i][1])))
            right_traj_cnt += right_traj_list[i].shape[0]
        coordinator = Coordination(left_traj=arml_trajectory,
                                   right_traj=armr_trajectory,
                                   fcl_checker=self.fcl_checker,
                                   planner_pin=self.planner_pin,
                                   coordination_res=0.1,
                                   traj_res=0.01,
                                   use_ee=self.use_ee)
        coordinator.set_head_body(self.head_qpos, self.torso_qpos)
        coordinator.set_ee_identifier(
            armlee_start_qpos=self.armlee_qpos,
            armree_start_qpos=self.armree_qpos,
            left_identifier_list=left_ee_identifier_list,
            right_identifier_list=right_ee_identifier_list)
        coordinated_left, coordinated_right, coordinated_left_ee, coordinated_right_ee = coordinator.Coordinate_RRT_with_EE(
        )

        return coordinated_left, coordinated_right, coordinated_left_ee, coordinated_right_ee
