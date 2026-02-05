# ----------------------------------------------------------------------------
# Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import time
import threading
import numpy as np
from ..utils.unit_conversion import user_pose_to_xpos, xpos_to_user_pose
from ..trajectory_planner.toppra_trajectory_planner import ToppraTrajectoryPlanner
from ...common import get_logger, StatusCode
from typing import Tuple, List

from ...client_supervisor.utils.move_sync_option import MoveSyncOption
from ...robot_client import MotionGroup, Status


class SupervisorViewW1(object):

    def __init__(self,
                 supervisor,
                 w1_params,
                 kine_solver,
                 pin_kine_solver,
                 collision_checker,
                 hybrid_planner,
                 cartesian_planner,
                 cartesian_joint_planner,
                 stop_move_event: threading.Event,
                 time_step: float = 0.01) -> None:
        self.supervisor = supervisor
        self.w1_params = w1_params
        self.kine_solver = kine_solver
        self.pin_kine_solver = pin_kine_solver  # Use 'whole_body_kin_solver'?
        self.collision_checker = collision_checker
        self.hybrid_planner = hybrid_planner
        self.cartesian_planner = cartesian_planner
        self.cartesian_joint_planner = cartesian_joint_planner
        self.stop_move_event = stop_move_event
        self.TIME_STEP = time_step  # removed after mgr_util refactor

        self.logger = get_logger()

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

    def get_arm_ee_client(self, name: str):
        if not self._has_arm_ee_client(name):
            return None
        client = self.supervisor.client_map.get(name, None)
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
            name: str = "full_body"
    ) -> tuple[StatusCode, np.ndarray, np.ndarray]:
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
                         name: str = "full_body"
                        ) -> tuple[StatusCode, np.ndarray]:
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
            status = self.get_arm_ee_client(name).peek_status()
            res = status.get("res", False)
            action = status.get("action", None)

            return (StatusCode.SUCCESS if res else StatusCode.FAIL), action

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
            return self.kine_solver.left_arm_kine
        elif name == "right_arm":
            return self.kine_solver.right_arm_kine

    def get_current_xpos(
            self,
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

    def get_current_pose(self,
                         name: str,
                         is_local: bool = True) -> tuple[StatusCode, list]:
        """is_local=True，获取left_arm/right_arm当前相对于arm base坐标系的笛卡尔位姿[xyz_mm, rpy_deg]

            is_local=False，获取left_arm/right_arm当前相对于chassis坐标系的笛卡尔位姿[xyz_mm, rpy_deg]

        Args:
            name (str): ["left_arm", "right_arm"]
            is_local (bool, optional): w.r.t. arm base frame or chassis frame. Defaults to True.

        Returns:
            tuple[StatusCode, list]: 
        """
        res, cur_xpos = self.get_current_xpos(name, is_local)
        if res.value < 0:
            return res, None
        return res, xpos_to_user_pose(cur_xpos)

    def get_pose(self,
                 name: str,
                 qpos: np.ndarray,
                 is_local: bool = True) -> tuple[StatusCode, list]:
        """is_local=True，获取left_arm/right_arm相对于arm base坐标系的笛卡尔位姿[xyz_mm, rpy_deg]，size(qpos)=7

            is_local=False，获取left_arm/right_arm相对于chassis坐标系的笛卡尔位姿[xyz_mm, rpy_deg]，size(qpos)=20

        Args:
            name (str): ["left_arm", "right_arm"]
            qpos (np.ndarray): 
            is_local (bool, optional): w.r.t. arm base frame or chassis frame. Defaults to True.

        Returns:
            tuple[StatusCode, list]: 
        """
        res, cur_xpos = self.get_xpos(name, qpos, is_local)
        if res.value < 0:
            return res, None
        return res, xpos_to_user_pose(cur_xpos)

    def transfer_grasp_pose(
            self, name: str, pre_grasp_pose: list, pre_torso_qpos: np.ndarray,
            target_torso_qpos: np.ndarray) -> tuple[StatusCode, list]:
        """保持抓取点在全身坐标系的位置，获取torso运动后的相对于机械臂基座坐标系的抓取pose

        Args:
            name (str): left_arm / right_arm
            pre_grasp_pose (list): 抓取前相对于机械臂基座的pose
            pre_torso_qpos (np.ndarray): torso运动前的qpos
            target_torso_qpos (np.ndarray): torso运动后的qpos

        Returns:
            tuple[StatusCode, list]
        """
        pre_torso_qpos = np.array(pre_torso_qpos)
        target_torso_qpos = np.array(target_torso_qpos)

        if not self._check_arm_name(name):
            return StatusCode.NAME_MISMATCH, None
        if not self._check_qpos_dimension("torso", pre_torso_qpos) or \
            not self._check_qpos_dimension("torso", target_torso_qpos):
            return StatusCode.DIM_MISMATCH, None

        T_armbase_ee_pre = user_pose_to_xpos(pre_grasp_pose[:3],
                                             pre_grasp_pose[3:])

        qpos_pre = np.zeros(self.w1_params.full_body_dofs)
        qpos_pre[:4] = pre_torso_qpos
        res_fk, T_chassis_armbase_pre = self.kine_solver.get_fk(
            name + "_base", qpos_pre)
        if not res_fk:
            self.logger.error(f"{name} solve fk failed!")
            return StatusCode.IK_FAIL, None

        qpos_target = np.zeros(self.w1_params.full_body_dofs)
        qpos_target[:4] = target_torso_qpos
        res_fk, T_chasis_armbase_target = self.kine_solver.get_fk(
            name + "_base", qpos_target)
        if not res_fk:
            self.logger.error(f"{name} solve fk failed!")
            return StatusCode.FK_FAIL, None

        T_armbase_ee_target = np.linalg.inv(
            T_chasis_armbase_target) @ T_chassis_armbase_pre @ T_armbase_ee_pre
        return StatusCode.SUCCESS, xpos_to_user_pose(T_armbase_ee_target)

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

            res, value = self.kine_solver.get_fk(name, qpos)
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
            torso_tf, head_tf, arml_tf, armr_tf, armlee_tf, armree_tf = self.pin_kine_solver.get_Kinematics(
                qpos,
                left_ee_qpos=armlee_qpos,
                right_ee_qpos=armree_qpos,
                compute_ee=True)
        else:
            torso_tf, head_tf, arml_tf, armr_tf = self.pin_kine_solver.get_Kinematics(
                qpos, left_ee_qpos=None, right_ee_qpos=None, compute_ee=False)
            armlee_tf, armree_tf = None, None

        self.collision_checker.set_head_body_transform(head_transforms=head_tf,
                                                       body_transforms=torso_tf)
        self.collision_checker.update_arm_transform(arml_tf + armr_tf,
                                                    armlee_tf, armree_tf)
        res = self.collision_checker.naive_check_collision(
            robot_name=robot_name)
        if res:
            self.collision_checker.update_vis_transform(
                torso_tf + head_tf + arml_tf + armr_tf, armlee_tf, armree_tf)
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

    def stop_move(self, timeout_sec: float = 3.0) -> StatusCode:
        """停止运动

        Args:
            timeout_sec (float, optional): 等待停止超时时间. Defaults to 3.0.

        Returns:
            StatusCode: 
        """
        self.logger.info("Stop move by user!")
        self.stop_move_event.set()
        if self.supervisor.robot_client.stop_move() == -1:
            self.logger.error("Stop move by user failed!")
            return StatusCode.FAIL
        t0 = time.time()
        while self.supervisor.robot_client._client.status != Status.Idle:
            if time.time() - t0 > timeout_sec:
                self.logger.error("Stop move wait timeout!")
                return StatusCode.TIME_OUT
            time.sleep(2 * self.TIME_STEP)
        return StatusCode.SUCCESS

    def execute_joint_traj(
        self,
        qpos_array: np.ndarray,
        left_arm_ee_trigger_list: list[Tuple[int, List[float]]] = None,
        right_arm_ee_trigger_list: list[Tuple[int,
                                              List[float]]] = None) -> bool:
        """Execute joint trajectory with 10ms interval

        Args:
            qpos_array (np.ndarray): joint trajectory
            left_arm_ee_trigger_list (list[Tuple[int, List[float]]], optional): left end-effector trigger sequence. Defaults to None.
            right_arm_ee_trigger_list (list[Tuple[int, List[float]]], optional): right end-effector trigger sequence. Defaults to None.

        Returns:
            StatusCode: Execution status
        """
        self.logger.debug("Starting joint trajectory execution...")

        robot_client = self.supervisor.robot_client._client
        # check enable
        if robot_client.status != Status.Idle:
            self.logger.error(
                f"Robot client not in idle before moving. (Current status: {robot_client.status})"
            )
            return StatusCode.ROBOT_STATUS_INVALID

        # reset stop move flag
        if self.stop_move_event.is_set():
            self.stop_move_event.clear()

        # check current qpos and start qpos
        res, cur_qpos = self.get_current_qpos()
        if res.value < 0:
            return res
        if not np.allclose(cur_qpos, qpos_array[0], atol=np.deg2rad(1)):
            self.logger.error(
                f"cur_qpos ({cur_qpos}) is not equal to start_qpos ({qpos_array[0]})."
            )
            return StatusCode.NOT_AT_START

        # Validate inputs
        if not isinstance(qpos_array, np.ndarray):
            self.logger.error("qpos_array must be a numpy array.")
            return StatusCode.EE_MOVE_FAIL

        # Ensure trigger lists are valid or default to empty lists
        left_arm_ee_trigger_list = left_arm_ee_trigger_list or []
        right_arm_ee_trigger_list = right_arm_ee_trigger_list or []

        # Extract unique sync IDs
        raw_endpoint_index = [
            sync_id for sync_id, _ in left_arm_ee_trigger_list
        ]
        raw_endpoint_index += [
            sync_id for sync_id, _ in right_arm_ee_trigger_list
        ]
        unique_sync_ids = sorted(
            set(raw_endpoint_index)) if raw_endpoint_index else None

        t0 = time.time()
        try:
            move_id, end_id = self.supervisor.move_joint_trajectory_async(
                qpos_array,
                interval=self.TIME_STEP,
                endpoint_index=unique_sync_ids)
            self.logger.debug(
                f"Trajectory execution started: move_id={move_id}, end_id={end_id}"
            )
        except Exception as e:
            self.logger.error(f"Failed to start joint trajectory: {e}")
            return False

        # Process trigger lists
        def process_trigger_list(trigger_list, unique_sync_ids, move_id):
            if not trigger_list or not unique_sync_ids:
                return []
            sync_id_map = {
                sync_id: move_id + i + 1
                for i, sync_id in enumerate(unique_sync_ids)
            }
            new_trigger_list = [(sync_id_map[sync_id], action)
                                for sync_id, action in trigger_list
                                if sync_id in sync_id_map]
            new_trigger_list.sort(key=lambda x: x[0])
            return new_trigger_list

        left_arm_ee_trigger_list_new = process_trigger_list(
            left_arm_ee_trigger_list, unique_sync_ids, move_id)
        right_arm_ee_trigger_list_new = process_trigger_list(
            right_arm_ee_trigger_list, unique_sync_ids, move_id)

        # Trigger end-effector actions
        def trigger_ee_actions(trigger_list, arm_name):
            for sync_id, action in trigger_list:
                try:
                    self.supervisor.call_srv_async(
                        arm_name, {"action": action},
                        MoveSyncOption.sync_w_explicit_id(sync_id))
                    self.logger.debug(
                        f"Triggered action for {arm_name}: sync_id={sync_id}, action={action}"
                    )
                except Exception as e:
                    self.logger.error(
                        f"Failed to trigger action for {arm_name}: {e}")

        if left_arm_ee_trigger_list_new:
            trigger_ee_actions(left_arm_ee_trigger_list_new, "left_arm_ee")
        if right_arm_ee_trigger_list_new:
            trigger_ee_actions(right_arm_ee_trigger_list_new, "right_arm_ee")

        # Wait for completion
        self.logger.debug("Waiting for all movements to complete...")

        traj_duration = (qpos_array.shape[0] - 1) * self.TIME_STEP
        self.supervisor.wait_move(time_out=max(5, traj_duration))

        self.logger.info("Finish moving. " +
                         f"Ideal duration should be {traj_duration}s, " +
                         f"and actual duration is {time.time() - t0}s.")

        # check reach qpos
        target_qpos = qpos_array[-1]
        res, cur_qpos = self.get_current_qpos()
        if res.value < 0:
            return res
        if not np.allclose(cur_qpos, target_qpos, atol=np.deg2rad(5)):
            self.logger.error(
                f"Current qpos[{cur_qpos}] does NOT reach the target qpos[{target_qpos}], "
                + f"qpos error(deg): {np.rad2deg(cur_qpos - target_qpos)}.")
            return StatusCode.GOAL_NOT_REACH
        if not np.allclose(cur_qpos, target_qpos, atol=np.deg2rad(0.5)):
            self.logger.warning(
                f"Current qpos[{cur_qpos}] does NOT close to the target qpos[{target_qpos}], "
                + f"qpos error(deg): {np.rad2deg(cur_qpos - target_qpos)}.")

        self.logger.debug("Joint trajectory execution completed successfully.")
        return StatusCode.SUCCESS
