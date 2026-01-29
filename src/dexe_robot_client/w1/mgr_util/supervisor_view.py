# ----------------------------------------------------------------------------
# Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import numpy as np
from ..utils.unit_conversion import user_pose_to_xpos, xpos_to_user_pose
from ..trajectory_planner.toppra_trajectory_planner import ToppraTrajectoryPlanner
from ...common import get_logger, StatusCode
from typing import Tuple, List

class SupervisorViewW1(object):

    def __init__(self, supervisor, w1_params, kine_solver, pin_kin_solver,collision_checker, time_step : float = 0.01) -> None:
        self.supervisor = supervisor
        self.logger = get_logger()
        self.w1_params = w1_params
        self.kine = kine_solver
        self.pin_kine = pin_kin_solver
        self.collision_checker = collision_checker
        self.TIME_STEP = time_step  # removed after mgr_util refactor

    def _has_arm_ee_client(self, name: str, verbose: bool = True) -> bool:
        if name in self.w1_params.arm_ee_names:
            if name == "left_arm_ee" and 'left_arm_ee' in self.supervisor.client_map:
                return True
            if name == "right_arm_ee" and 'right_arm_ee' in self.supervisor.client_map:
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
        client = self.supervisor.client_map.get(name)
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
            name: str = "full_body") -> tuple[StatusCode, np.ndarray, np.ndarray]:
        """获取特定部件(支持ee)的关节极限

        Args:
            name (str, optional): ["full_body", "torso", "head", "left_arm", "right_arm", 
                                    "left_arm_ee", "right_arm_ee"]. Defaults to "full_body".

        Returns:
            tuple[StatusCode, np.ndarray, np.ndarray]: 状态码；下极限；上极限
        """
        if name in self.w1_params.arm_ee_names:
            if not self._has_arm_ee_client(name):
                return StatusCode.FAIL, None, None
            qpos_lb = np.array([0.] * self.w1_params.arm_ee_dofs)
            qpos_ub = np.array([100.] * self.w1_params.arm_ee_dofs)

        elif not self._check_name(name):
            return StatusCode.NAME_MISMATCH, None, None
        else:
            qpos_lb = self.w1_params.qpos_lb[self.w1_params.index_infos[name]
                                             ["index"]]
            qpos_ub = self.w1_params.qpos_ub[self.w1_params.index_infos[name]
                                             ["index"]]
        return StatusCode.SUCCESS, qpos_lb, qpos_ub
    
    def get_current_qpos(self,
                         name: str = "full_body") -> tuple[StatusCode, np.ndarray]:
        """获取特定部件(支持ee)的当前关节位置

        Args:
            name (str, optional): ["full_body", "torso", "head", "left_arm", "right_arm", 
                                    "left_arm_ee", "right_arm_ee"]. Defaults to "full_body".

        Returns:
            tuple[StatusCode, np.ndarray]
        """
        support_names = list(self.w1_params.names) + self.w1_params.arm_ee_names
        if name not in support_names:
            self.logger.error(f"{support_names} is supported, except {name}.")
            return StatusCode.NAME_MISMATCH, None

        if name in self.w1_params.arm_ee_names:
            if not self._has_arm_ee_client(name, False):
                return StatusCode.FAIL, None
            res, value = self._get_arm_ee_client(name).get_pos()
            return (StatusCode.SUCCESS if res else StatusCode.FAIL), value

        robot_info = self.supervisor.robot_client.get_info()
        if robot_info.joint_position is None:
            self.logger.error(
                f"Get current robot info failed, status: {robot_info.status}.")
            return StatusCode.ROBOT_STATUS_INVALID, None

        cur_qpos = np.array(robot_info.joint_position)

        if name == "full_body":
            return StatusCode.SUCCESS, cur_qpos
        return StatusCode.SUCCESS, cur_qpos[self.w1_params.index_infos[name]
                                            ["index"]]
    
    def _get_arm_kine(self, name: str):
        if not self._check_arm_name(name):
            return None
        if name == "left_arm":
            return self.kine.left_arm_kine
        elif name == "right_arm":
            return self.kine.right_arm_kine
        
    def get_current_xpos(self,
                         name: str,
                         is_local: bool = True) -> tuple[StatusCode, np.ndarray]:
        """is_local=True，获取left_arm/right_arm相对于arm base坐标系的笛卡尔位姿
        
            is_local=False，获取["head", "head_flange" 
                                 "left_arm_base", "left_arm", "left_arm_flange", 
                                "right_arm_base", "right_arm", "right_arm_flange"]
                                相对于chassis坐标系的笛卡尔位姿

        Args:
            name (str): ["left_arm", "right_arm"]
            is_local (bool, optional): w.r.t. arm base frame or chassis frame. Defaults to True.

        Returns:
            tuple[StatusCode, np.ndarray]:
        """
        if is_local:
            res, cur_qpos = self.get_current_qpos(name)
            if res.value < 0:
                return res, None
        else:
            res, cur_qpos = self.get_current_qpos("full_body")
            if res.value < 0:
                return res, None
        return self.get_xpos(name, cur_qpos, is_local)

    def get_xpos(self,
                 name: str,
                 qpos: np.ndarray,
                 is_local: bool = True) -> tuple[StatusCode, np.ndarray]:
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
            tuple[StatusCode, np.ndarray]:
        """
        if is_local:
            if not self._check_qpos_dimension(name, qpos):
                return StatusCode.DIM_MISMATCH, None

            arm_kine = self._get_arm_kine(name)
            if arm_kine is None:
                return StatusCode.NAME_MISMATCH, None

            res, value = arm_kine.get_fk(qpos)
            return (StatusCode.SUCCESS if res else StatusCode.FK_FAIL), value
        else:
            if not self._check_qpos_dimension("full_body", qpos):
                return StatusCode.DIM_MISMATCH, None

            res, value = self.kine.get_fk(name, qpos)
            return (StatusCode.SUCCESS if res else StatusCode.FK_FAIL), value

    def is_collision(self,
                     qpos: np.ndarray,
                     armlee_qpos: np.ndarray = None,
                     armree_qpos: np.ndarray = None,
                     robot_name: str = "whole_body") -> StatusCode:
        """检查qpos状态是否碰撞"""
        if not self._check_qpos_dimension("full_body", qpos):
            return StatusCode.DIM_MISMATCH
        if armlee_qpos is not None and not self._check_qpos_dimension(
                "left_arm_ee", armlee_qpos):
            return StatusCode.DIM_MISMATCH
        if armree_qpos is not None and not self._check_qpos_dimension(
                "right_arm_ee", armree_qpos):
            return StatusCode.DIM_MISMATCH
        if self._has_arm_ee_client("left_arm_ee",
                                   False) and self._has_arm_ee_client(
                                       "right_arm_ee", False):
            torso_tf, head_tf, arml_tf, armr_tf, armlee_tf, armree_tf = self.pin_kine.get_Kinematics(
                qpos,
                left_ee_qpos=armlee_qpos,
                right_ee_qpos=armree_qpos,
                compute_ee=True)
        else:
            torso_tf, head_tf, arml_tf, armr_tf = self.pin_kine.get_Kinematics(
                qpos, left_ee_qpos=None, right_ee_qpos=None, compute_ee=False)
            armlee_tf, armree_tf = None, None

        self.collision_checker.set_head_body_transform(head_transforms=head_tf,
                                                 body_transforms=torso_tf)
        self.collision_checker.update_arm_transform(arml_tf + armr_tf, armlee_tf,
                                              armree_tf)
        res = self.collision_checker.check_collision(robot_name=robot_name)
        return (StatusCode.SUCCESS if res else StatusCode.FAIL)

    def is_traj_collision(self,
                           qpos_array: np.ndarray,
                           left_arm_ee_trigger_list: list[tuple[int,
                                                                list]] = None,
                           right_arm_ee_trigger_list: list[tuple[int,
                                                                 list]] = None,
                           step: int = 5) -> StatusCode:
        """检查轨迹是否碰撞
        Args:
            qpos_array (np.ndarray): 10ms间隔的轨迹
            left_arm_ee_trigger_list (list[tuple[int, list]], optional): 左臂末端执行器触发列表，格式[(idx, qpos), ...]. Defaults to None.
            right_arm_ee_trigger_list (list[tuple[int, list]], optional): 右臂末端执行器触发列表，格式[(idx, qpos), ...]. Defaults to None.
            left_arm_ee_trigger_list (list[tuple[int, list]], optional): 左臂末端执行器触发列表，格式[(idx, qpos), ...]. Defaults to None.
            right_arm_ee_trigger_list (list[tuple[int, list]], optional): 右臂末端执行器触发列表，格式[(idx, qpos), ...]. Defaults to None.
            step (int, optional): 检测步长，每步10ms. Defaults to 5.
        Returns:
            StatusCode: 
        """
        # NOTE: 最大速度 伸直构型下 仅基座关节运动导致的末端位移 5 * 0.01s * 1rad/s * 0.7m = 3.5cm

        if self._has_arm_ee_client("left_arm_ee",
                                   False) and self._has_arm_ee_client(
                                       "right_arm_ee", False):
            cur_left_ee_qpos = self.get_current_qpos("left_arm_ee")[1]
            cur_right_ee_qpos = self.get_current_qpos("right_arm_ee")[1]
            if left_arm_ee_trigger_list is None:
                left_arm_ee_qpos_array = np.tile(cur_left_ee_qpos,
                                                 (qpos_array.shape[0], 1))
            else:
                left_arm_ee_qpos_array = np.tile(cur_left_ee_qpos,
                                                 (qpos_array.shape[0], 1))
                for idx, target_qpos in left_arm_ee_trigger_list:
                    left_arm_ee_qpos_array[idx:, :] = np.tile(
                        np.asarray(target_qpos), (qpos_array.shape[0] - idx, 1))
            if right_arm_ee_trigger_list is None:
                right_arm_ee_qpos_array = np.tile(cur_right_ee_qpos,
                                                  (qpos_array.shape[0], 1))
            else:
                right_arm_ee_qpos_array = np.tile(cur_right_ee_qpos,
                                                  (qpos_array.shape[0], 1))
                for idx, target_qpos in right_arm_ee_trigger_list:
                    right_arm_ee_qpos_array[idx:, :] = np.tile(
                        np.asarray(target_qpos), (qpos_array.shape[0] - idx, 1))
            qpos_array_check = np.vstack((qpos_array[::step], qpos_array[-1]))
            left_arm_ee_qpos_array_check = np.vstack(
                (left_arm_ee_qpos_array[::step], left_arm_ee_qpos_array[-1]))
            right_arm_ee_qpos_array_check = np.vstack(
                (right_arm_ee_qpos_array[::step], right_arm_ee_qpos_array[-1]))
            for idx in range(qpos_array_check.shape[0]):
                if self.is_collision(
                        qpos_array_check[idx],
                        left_arm_ee_qpos_array_check[idx],
                        right_arm_ee_qpos_array_check[idx]) != StatusCode.FAIL:
                    self.logger.error(
                        f"Traj is collision at idx {idx}, qpos {qpos_array_check[idx]}, do not executed!"
                    )
                    return StatusCode.SUCCESS
            return StatusCode.FAIL
        else:
            qpos_array_check = np.vstack((qpos_array[::step], qpos_array[-1]))
            for qpos in qpos_array_check:
                if self.is_collision(qpos) != StatusCode.FAIL:
                    self.logger.error(
                        f"Traj is collision at {qpos}, do not executed!")
                    return StatusCode.SUCCESS
            return StatusCode.FAIL

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
        # try:
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

        # except Exception as e:
        #     self.logger.error(f"Error during joint trajectory execution: {e}")
        #     return False
