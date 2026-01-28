# ----------------------------------------------------------------------------
# Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import numpy as np
from .utils.unit_conversion import user_pose_to_xpos, xpos_to_user_pose
from .trajectory_planner.toppra_trajectory_planner import ToppraTrajectoryPlanner
from .path_planner.bezier_blend_path import interpolate_cartesian_path
from ..common import get_logger


class SupervisorViewW1(object):

    def __init__(self, supervisor_in, w1_params_in, kine_in: DexforceW1Kinematics) -> None:
        self.supervisor = supervisor_in
        self.logger = get_logger()
        self.w1_params = w1_params_in
        self.kine = kine_in

    def _has_arm_ee_client(self, name: str, verbose: bool = True) -> bool:
        if name in self.w1_params.arm_ee_names:
            if name == "left_arm_ee" and 'left_arm_ee' in self.supervisor.client_map:
                return True
            if name == "right_arm_ee" and 'left_arm_ee' in self.supervisor.client_map:
                return True
            if verbose:
                self.logger.error(f"{name} has not initialized")
            return False

        self.logger.error(
            f"{self.w1_params.arm_ee_names} is supported, except {name}.")
        return False
    
    def _get_arm_ee_client(self, name: str):
        if not self._has_arm_ee_client(name):
            return None
        client = self.supervisor.client_map(name)
        return client
    
    def _check_name(self, name: str) -> bool:
        """仅支持component检查，不包含ee"""
        if name not in self.w1_params.names:
            self.logger.error(
                f"{self.w1_params.names} is supported, except {name}.")
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
        else:  # list
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

        valid_ee_state = [
            ee_state for ee_state in arm_ee_state_list if ee_state is not None
        ]
        if not all(
                isinstance(ee_state, (float, list, float))
                for ee_state in valid_ee_state):
            self.logger.error(
                f"arm_ee_state should be [pre_delay, ee_qpos, post_delay].")
            return False
        if not self._check_qpos_dimension(
                "left_arm_ee", [ee_state[1] for ee_state in valid_ee_state]):
            return False
        return True
    
    def get_qpos_limits(
            self,
            name: str = "full_body") -> tuple[bool, np.ndarray, np.ndarray]:
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
            qpos_lb = np.array([0.] * self.w1_params.arm_ee_dofs)
            qpos_ub = np.array([100.] * self.w1_params.arm_ee_dofs)

        elif not self._check_name(name):
            return False, None, None
        else:
            qpos_lb = self.w1_params.qpos_lb[self.w1_params.index_infos[name]
                                             ["index"]]
            qpos_ub = self.w1_params.qpos_ub[self.w1_params.index_infos[name]
                                             ["index"]]
        return True, qpos_lb, qpos_ub
    
    def get_current_qpos(self,
                         name: str = "full_body") -> tuple[bool, np.ndarray]:
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
            return self._get_arm_ee_client(name).peek_status()

        robot_info = self.supervisor.robot_client.query_state()
        if robot_info.joint_position is None:
            self.logger.error(
                f"Get current robot info failed, status: {robot_info.status}.")
            return False, None

        cur_qpos = np.array(robot_info.joint_position)

        if name == "full_body":
            return True, cur_qpos
        return True, cur_qpos[self.w1_params.index_infos[name]["index"]]
    
    def _get_arm_kine(self, name: str):
        if not self._check_arm_name(name):
            return None
        if name == "left_arm":
            return self.kine.left_arm_kine
        elif name == "right_arm":
            return self.kine.right_arm_kine
        
    def get_current_xpos(self,
                         name: str,
                         is_local: bool = True) -> tuple[bool, np.ndarray]:
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

    def get_xpos(self,
                 name: str,
                 qpos: np.ndarray,
                 is_local: bool = True) -> tuple[bool, np.ndarray]:
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
        
    def execute_joint_traj(self,
                           qpos_array: np.ndarray,
                           left_arm_ee_trigger_list: list[Tuple[int, List[float]]] = None,
                           right_arm_ee_trigger_list: list[Tuple[int, List[float]]] = None) -> bool:
        """Execute joint trajectory with 10ms interval

        Args:
            qpos_array (np.ndarray): joint trajectory
            left_arm_ee_trigger_list (list[Tuple[int, List[float]]], optional): left end-effector trigger sequence. Defaults to None.
            right_arm_ee_trigger_list (list[Tuple[int, List[float]]], optional): right end-effector trigger sequence. Defaults to None.

        Returns:
            StatusCode: Execution status
        """
        try:
            self.logger.info("Starting joint trajectory execution...")
            move_id, end_id = self.supervisor.move_joint_trajectory_async(
                qpos_array, interval=0.01)

            if left_arm_ee_trigger_list:
                for trigger in left_arm_ee_trigger_list:
                    action, sync_id = trigger
                    self.logger.info(f"Triggering left arm EE action: {action}")
                    self.supervisor.call_srv_async(
                        "left_arm_ee", {"action": action},
                        MoveSyncOption.sync_w_explicit_id(sync_id))

            if right_arm_ee_trigger_list:
                for trigger in right_arm_ee_trigger_list:
                    action, sync_id = trigger
                    self.logger.info(f"Triggering right arm EE action: {action}")
                    self.supervisor.call_srv_async(
                        "right_arm_ee", {"action": action},
                        MoveSyncOption.sync_w_explicit_id(sync_id))

            self.logger.info("Waiting for all movements to complete...")
            self.supervisor.wait_move()

            self.logger.info("Joint trajectory execution completed successfully.")
            return True

        except Exception as e:
            self.logger.error(f"Error during joint trajectory execution: {e}")
            return False
