import os
import time
import numpy as np
import json
from copy import deepcopy

from dexe_robot_client.w1.path_planner.fcl_checker import FCLChecker
from dexe_robot_client.w1.path_planner.ompl_plan import OMPLPlanSingle, OMPLPlanCentralized, OMPLPlanDecoupled
from dexe_robot_client.w1.path_planner.utils.robot_type import RobotType, RobotURDF, RobotCollision, get_robot_urdf, get_robot_mesh_path
from dexe_robot_client.end_effector_client import EndEffectorType 

HEAD_DIM = 2
TORSO_DIM = 4


class SinglePlanner:
    arml_dim, armr_dim = None, None
    arml_bounds, armr_bounds = None, None
    ee_type = None
    use_ee = None
    env_json_path = None
    urdf_path = None
    robot_type = None

    @classmethod
    def set_arms_cfg(cls, arml_dim: int, armr_dim: int, arml_bounds: np.ndarray, armr_bounds: np.ndarray):
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
            raise ValueError(f"Left arm bounds should be of shape (2, {arml_dim}) but got {arml_bounds.shape}.")
        if armr_bounds.shape[1] != armr_dim or armr_bounds.shape[0] != 2:
            raise ValueError(f"Right arm bounds should be of shape (2, {armr_dim}) but got {armr_bounds.shape}.")

        cls.arml_bounds = arml_bounds
        cls.armr_bounds = armr_bounds

    @classmethod
    def set_robot_cfg(cls, robot_type: RobotType = None, ee_type: EndEffectorType = None):
        """
        Set the configuration for the robot, including the robot type and end effector type.

        Args:
            robot_type (RobotType): The type of the robot (RobotType.W1V02_1 or RobotType.W1V02_2)
            ee_type (EndEffectorType): The type of the end effector (EndEffectorType.PGC_GRIPPER or EndEffectorType.BRAINCO_HAND)

        Notes:
            - If ee_type is None, it will not use the end effector
        """
        if ee_type is not None:
            cls.ee_type = ee_type
            cls.use_ee = True
        else:
            cls.ee_type = None
            cls.use_ee = False
        
        if robot_type is not None:
            if robot_type not in RobotType:
                raise ValueError(f"Unsupported robot type: {robot_type}. Please use one of {list(RobotType)}.")
            cls.urdf_path = get_robot_urdf(robot_type)
            cls.robot_type = robot_type
        else:
            raise ValueError(f"Robot type must be specified. Please use  one of {list(RobotType)}.")

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
        self.arml_start, self.armr_start, self.arml_goal, self.armr_goal = None, None, None, None
        self.armlee_wait_time, self.armree_wait_time = 0, 0
        self.armlee_exec_time, self.armree_exec_time = 0, 0
        self.head_qpos = np.zeros(HEAD_DIM)
        self.torso_qpos = np.zeros(TORSO_DIM)

        
    def _init_fcl_checker(self, robot_type: RobotType, ee_type: EndEffectorType):
        self.fcl_checker = FCLChecker(robot_type, ee_type)
        # self.fcl_checker.create_robot_obj()
        # self.fcl_checker.create_robot_manager()
        if self.env_json_path is not None:
            self.fcl_checker.load_env_json(self.env_json_path)

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
            raise ValueError(f"Head position should be of shape ({HEAD_DIM},) but got {head_qpos.shape}.")
        if torso_qpos.shape[0] != TORSO_DIM:
            raise ValueError(f"Torso position should be of shape ({TORSO_DIM},) but got {torso_qpos.shape}.")
        self.head_qpos = head_qpos
        self.torso_qpos = torso_qpos

    def set_ee_time(self, armlee_wait_time: np.ndarray, armree_wait_time: np.ndarray, 
                    armlee_exec_time: np.ndarray, armree_exec_time: np.ndarray):
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

    def attach_object(self, obj_name: str, which_arm: str, flange_T_object: np.ndarray = None):
        if which_arm not in ['left', 'right']:
            raise ValueError("which_arm must be either 'left' or 'right'.")
        if which_arm == "left":
            self.fcl_checker.attach_object_arml(obj_name=obj_name, flange_T_object=flange_T_object)
        if which_arm == "right":
            self.fcl_checker.attach_object_armr(obj_name=obj_name, flange_T_object=flange_T_object)

    def detach_object(self, obj_name: str, which_arm: str, arm7_transform: np.ndarray):
        if which_arm not in ['left', 'right']:
            raise ValueError("which_arm must be either 'left' or 'right'.")
        if which_arm == "left":
            self.fcl_checker.detach_object_arml(obj_name=obj_name, arml7_transform=arm7_transform)
        if which_arm == "right":    
            self.fcl_checker.detach_object_armr(obj_name=obj_name, armr7_transform=arm7_transform)
    def plan(self):
        """
        Solve the planning problem which will decide which planner to use based on the start and goal configurations of the arms.

        Returns:
            left_trajectory (np.ndarray): Planned trajectory for the left arm, shape (N, arml_dim)
            right_trajectory (np.ndarray): Planned trajectory for the right arm, shape (N, armr_dim)
            plan_time (float): Total time taken for the planning process
        """
        is_arml_plan = True if np.max(np.abs(self.arml_goal - self.arml_start)) > 1e-4 else False
        is_armr_plan = True if np.max(np.abs(self.armr_goal - self.armr_start)) > 1e-4 else False
        if is_arml_plan and not is_armr_plan:
            print("----------------using Left arm Single Planner----------------")
            ompl_planner = OMPLPlanSingle(self.arml_dim, self.arml_bounds, self.arml_start, self.arml_goal,
                                          self.fcl_checker, urdf_path=self.urdf_path,
                                          which_arm='left', other_arm_qpos=self.armr_start,
                                          use_ee=self.use_ee)
            ompl_planner.set_head_body(self.head_qpos, self.torso_qpos)
            left_path, left_trajectory, left_plan_time = ompl_planner.plan(planning_time=2.0)
            if left_path is not None:
                return left_trajectory, None, left_plan_time
            else:
                return None, None, None
            
        if is_armr_plan and not is_arml_plan:
            print("----------------using Right arm Single Planner----------------")
            ompl_planner = OMPLPlanSingle(self.armr_dim, self.armr_bounds, self.armr_start, self.armr_goal,
                                          self.fcl_checker, urdf_path=self.urdf_path,
                                          which_arm='right', other_arm_qpos=self.arml_start,
                                          use_ee=self.use_ee)
            ompl_planner.set_head_body(self.head_qpos, self.torso_qpos)
            right_path, right_trajectory, right_plan_time = ompl_planner.plan(planning_time=2.0)
            if right_path is not None:
                return None, right_trajectory, right_plan_time
            else:
                return None, None, None
            
        if is_arml_plan and is_armr_plan:
            print("----------------using Decoupled Planner----------------")
            ompl_planner = OMPLPlanDecoupled(self.arml_dim, self.armr_dim, 
                                             self.arml_bounds, self.armr_bounds,
                                             self.arml_start, self.armr_start,
                                             self.arml_goal, self.armr_goal,
                                             self.fcl_checker,
                                             urdf_path=self.urdf_path,
                                             use_ee=self.use_ee)
            ompl_planner.set_head_body(self.head_qpos, self.torso_qpos)
            coordinated_left, coordinated_right, plan_time = ompl_planner.plan()
            if coordinated_left is not None and coordinated_right is not None:
                return coordinated_left, coordinated_right, plan_time
            else:
                print("Failed to find a trajectory for dual arm Decoupled planning")
                return None, None, None
            
        if not is_arml_plan and not is_armr_plan:
            print("No planning needed, both arms are already at their goal positions.")
            return None, None, 0.0