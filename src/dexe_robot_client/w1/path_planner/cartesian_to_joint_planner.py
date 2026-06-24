'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

import numpy as np
from copy import deepcopy

from ...common import get_logger, W1ArmType, W1Version
from ..utils.w1_params import W1Params
from ...end_effector_client import EndEffectorType
from ..trajectory_planner.toppra_trajectory_planner import ToppraTrajectoryPlanner
from ..kinematics.pin_ik_whole_body import PinIKWholeBody
from .utils.robot_type import *
from dexe_rp import DexforceW1Kinematics, StatusType, calc_nearest_iks


class CartesianToJointPlanner:

    def __init__(
            self,
            arm_type: W1ArmType = W1ArmType.ANTHROPOMORPHIC,
            w1_version: W1Version = W1Version.V2,
            end_effector_type: EndEffectorType = EndEffectorType.BRAINCO_HAND):
        self.logger = get_logger()

        self._arm_type = arm_type
        self._w1_version = w1_version
        self._end_effector_type = end_effector_type

        self.w1_params = W1Params(self._arm_type, self._w1_version,
                                  self._end_effector_type)
        self._init_kine_solver()

        self.left_tcp = np.eye(4)
        self.right_tcp = np.eye(4)

    def _init_kine_solver(self):
        # ik_nearst_weight = np.array([2., 2., 2., 0., 1., 1., 1.])
        self.kine = DexforceW1Kinematics(
            self._arm_type == W1ArmType.ANTHROPOMORPHIC)
        # self.kine.left_arm_kine.set_ik_nearest_weight(ik_nearst_weight)
        # self.kine.right_arm_kine.set_ik_nearest_weight(ik_nearst_weight)
        self.kine.set_user_qpos_limits(self.w1_params.qpos_lb,
                                       self.w1_params.qpos_ub)

        if self._w1_version == W1Version.V2:
            if self._arm_type == W1ArmType.ANTHROPOMORPHIC:
                robot_type = RobotType.W1V02_1
            elif self._arm_type == W1ArmType.INDUSTRIAL:
                robot_type = RobotType.W1V02_2
            else:
                raise ValueError(f"Unsupported arm type: {self._arm_type}")
        else:
            raise ValueError(f"Unsupported W1 version: {self._w1_version}")
        urdf_path = get_robot_urdf(robot_type=robot_type)

        self.whole_body_ik_solver = PinIKWholeBody(urdf_path,
                                                   track_type="standard",
                                                   cost_type="standard")

        self.logger.info(
            f"Initialize kinematic solver for Cartesian to Joint Planner")

    def set_whole_body_ik_cost_type(self, cost_type: str):
        self.whole_body_ik_solver.set_cost_function(cost_type=cost_type)

    def set_whole_body_ik_focus_point(self, focus_point: np.ndarray):
        self.whole_body_ik_solver.set_focus_point(focus_point)

    def set_whole_body_ik_lean_backward_threshold(self, threshold: float):
        self.whole_body_ik_solver.set_lean_backward_threshold(threshold)

    def set_arm_tcp(self, name: str, pose: np.ndarray):
        if name == "left_arm":
            self.left_tcp = pose
        elif name == "right_arm":
            self.right_tcp = pose
        else:
            raise ValueError(f"Invalid arm name: {name}")
        return self.kine.set_tcp(name, pose)

    def _get_arm_kine(self, name: str):
        if name == "left_arm":
            return self.kine.left_arm_kine
        elif name == "right_arm":
            return self.kine.right_arm_kine
        else:
            raise ValueError(f"Invalid arm name: {name}")

    def plan_single_arm(self, name: str, start_qpos: np.ndarray,
                        end_effector_traj: np.ndarray):
        """
        Plan joint trajectory from end-effector Cartesian trajectory.

        Args:
            name (str): "left_arm" or "right_arm"
            start_qpos (np.ndarray, shape=(7,)): Starting joint positions
            end_effector_traj (np.ndarray, shape=(N,4,4)): End effector trajectory in Cartesian 
        
        Returns:
            success (bool): The planning success flag
            joint_qpos_list (List[np.ndarray], shape=(N,7)): The solved joint trajectory
        Note:
            - Suppose the input trajectory is dense enough.
        """
        # cur_qseed = deepcopy(start_qpos)
        # joint_qpos_list = []
        arm_kine = self._get_arm_kine(name)
        # for pose in end_effector_traj:
        #     res_ik, cur_qpos = arm_kine.get_nearest_ik(xpos=pose,
        #                                                qpos_seed=cur_qseed)
        #     if not res_ik:
        #         self.logger.error(
        #             f"{name} Cartesian_to_joint_planner : solve ik failed, cur_qseed(deg): {np.rad2deg(cur_qseed)}."
        #         )
        #         return False, None
        #     if not np.allclose(cur_qpos, cur_qseed, atol=np.deg2rad(90.0)):
        #         self.logger.error(f"{name} Cartesian_to_joint_planner, solving ik failed since qpos traj is not continues\n" + \
        #                           f"cur_qpos(deg): {np.rad2deg(cur_qpos)}\n" + \
        #                           f"cur_qseed(deg): {np.rad2deg(cur_qseed)}.")
        #         return False, None
        #     joint_qpos_list.append(cur_qpos)
        #     cur_qseed = cur_qpos
        # joint_qpos_array = np.array(joint_qpos_list)
        # return True, joint_qpos_array
        res, qpos_list = calc_nearest_iks(arm_kine, start_qpos,
                                          end_effector_traj)
        if res.type != StatusType.Success:
            return False, None
        return True, np.array(qpos_list)

    from dexe_robot_client.common import time_consuming

    @time_consuming
    def plan_whole_body(self, start_qpos: np.ndarray,
                        left_end_effector_traj: np.ndarray,
                        right_end_effector_traj: np.ndarray):
        """
        This function will solve whole body IK by tracking the dual arm end-effector poses
        Args:
            start_qpos (np.ndarray, shape=(20,)): The starting whole body joint positions
            left_end_effector_traj (np.ndarray, shape=(N,4,4)): Left arm end-effector trajectory in homogeneous matrix
            right_end_effector_traj (np.ndarray, shape=(N,4,4)): Right arm end-effector trajectory in homogeneous matrix
        
        Notes:
            - The pose of end-effector is in the robot base-link frame.
        """
        if left_end_effector_traj.shape[0] != right_end_effector_traj.shape[0]:
            self.logger.error(
                "Left and right end-effector traj must have the same length")
            return False, None

        for idx, left_pose in enumerate(left_end_effector_traj):
            left_end_effector_traj[idx] = left_pose @ np.linalg.inv(
                self.left_tcp)
        for idx, right_pose in enumerate(right_end_effector_traj):
            right_end_effector_traj[idx] = right_pose @ np.linalg.inv(
                self.right_tcp)

        current_qpos = deepcopy(start_qpos)
        joint_qpos_list = []
        for left_pose, right_pose in zip(left_end_effector_traj,
                                         right_end_effector_traj):
            res_ik, result_qpos = self.whole_body_ik_solver.solve_ik(
                left_ee_target=left_pose,
                right_ee_target=right_pose,
                cur_qpos=current_qpos)
            if not res_ik:
                self.logger.error(
                    f"Whole body Cartesian_to_joint_planner: solve ik failed, cur_qpos(deg): {np.rad2deg(current_qpos)}"
                )
                self.logger.error(
                    f"cartesian trajectory length {left_end_effector_traj.shape[0]}, and solve failed at index {len(joint_qpos_list)}."
                )
                return False, None
            if not np.allclose(result_qpos, current_qpos,
                               atol=np.deg2rad(90.0)):
                self.logger.error(f"Whole body Cartesian_to_joint_planner, solving ik failed since qpos traj is not continues\n" + \
                                  f"result_qpos(deg): {np.rad2deg(result_qpos)}\n" + \
                                  f"current_qpos(deg): {np.rad2deg(current_qpos)}.")
                return False, None
            joint_qpos_list.append(result_qpos)
            current_qpos = result_qpos
        joint_qpos_array = np.array(joint_qpos_list)
        return True, joint_qpos_array

    def generate_dense_dual_arm_joint_traj(
            self,
            left_joint_traj: np.ndarray = None,
            right_joint_traj: np.ndarray = None,
            qvel_limits_array: np.ndarray = None,
            current_left_qpos: np.ndarray = None,
            current_right_qpos: np.ndarray = None):
        dofs = self.w1_params.arm_dofs
        dual_arm_dofs = dofs * 2

        left_index = self.w1_params.index_infos["left_arm"]["index"]
        right_index = self.w1_params.index_infos["right_arm"]["index"]
        left_pos_limits = np.vstack((self.w1_params.qpos_lb[left_index],
                                     self.w1_params.qpos_ub[left_index]))
        right_pos_limits = np.vstack((self.w1_params.qpos_lb[right_index],
                                      self.w1_params.qpos_ub[right_index]))
        dual_arm_pos_limits = np.hstack([left_pos_limits, right_pos_limits])
        left_vel_limits = self.w1_params.max_qvel[left_index]
        right_vel_limits = self.w1_params.max_qvel[right_index]
        dual_arm_vel_limits = np.concatenate(
            [left_vel_limits, right_vel_limits], axis=0)
        left_acc_limits = self.w1_params.max_qacc[left_index]
        right_acc_limits = self.w1_params.max_qacc[right_index]
        dual_arm_acc_limits = np.concatenate(
            [left_acc_limits, right_acc_limits], axis=0)

        if left_joint_traj is None and right_joint_traj is None:
            self.logger.error(
                "At least one of left_joint_traj and right_joint_traj should be provided"
            )
            return False, None, None

        if (left_joint_traj is not None) != (right_joint_traj is not None):
            if left_joint_traj is not None:
                if left_joint_traj.ndim != 2 or left_joint_traj.shape[1] != dofs:
                    self.logger.error(
                        f"left_joint_traj must be of shape (N, {dofs})")
                    return False, None, None
                if current_left_qpos is not None:
                    left_joint_traj = np.vstack(
                        [current_left_qpos, left_joint_traj])
                    qvel_limits_array = np.vstack(
                        [qvel_limits_array[0], qvel_limits_array])
                traj_planner = ToppraTrajectoryPlanner(dofs, left_pos_limits,
                                                       left_vel_limits,
                                                       left_acc_limits)
                res, pos_array, _, _, _ = traj_planner.plan(
                    left_joint_traj, qvel_limits_array)
                if not res:
                    self.logger.error(
                        "left arm joint trajectory generation failed")
                    return False, None, None
                return True, pos_array, None
            if right_joint_traj is not None:
                if right_joint_traj.ndim != 2 or right_joint_traj.shape[
                        1] != dofs:
                    self.logger.error(
                        f"right_joint_traj must be of shape (N, {dofs})")
                    return False, None, None
                if current_right_qpos is not None:
                    right_joint_traj = np.vstack(
                        [current_right_qpos, right_joint_traj])
                    qvel_limits_array = np.vstack(
                        [qvel_limits_array[0], qvel_limits_array])
                traj_planner = ToppraTrajectoryPlanner(dofs, right_pos_limits,
                                                       right_vel_limits,
                                                       right_acc_limits)
                res, pos_array, _, _, _ = traj_planner.plan(
                    right_joint_traj, qvel_limits_array)
                if not res:
                    self.logger.error(
                        "right arm joint trajectory generation failed")
                    return False, None, None
                return True, None, pos_array

        if left_joint_traj is not None and right_joint_traj is not None:
            if left_joint_traj.ndim != 2 or left_joint_traj.shape[1] != dofs:
                self.logger.error(
                    f"left_joint_traj must be of shape (N, {dofs})")
                return False, None, None
            if right_joint_traj.ndim != 2 or right_joint_traj.shape[1] != dofs:
                self.logger.error(
                    f"right_joint_traj must be of shape (N, {dofs})")
                return False, None, None
            if left_joint_traj.shape[0] != right_joint_traj.shape[0]:
                self.logger.error(
                    "left_joint_traj and right_joint_traj must have the same length"
                )
                return False, None, None
            combined_traj = np.concatenate([left_joint_traj, right_joint_traj],
                                           axis=1)
            traj_planner = ToppraTrajectoryPlanner(dual_arm_dofs,
                                                   dual_arm_pos_limits,
                                                   dual_arm_vel_limits,
                                                   dual_arm_acc_limits)
            qvel_limits_array = np.hstack(
                [qvel_limits_array, qvel_limits_array])
            if current_left_qpos is not None and current_right_qpos is not None:
                combined_traj = np.vstack([
                    np.hstack([current_left_qpos, current_right_qpos]),
                    combined_traj
                ])
                qvel_limits_array = np.vstack(
                    [qvel_limits_array[0], qvel_limits_array])
            res, pos_array, _, _, _ = traj_planner.plan(combined_traj,
                                                        qvel_limits_array)

            if not res:
                self.logger.error("dual arm joint trajectory generation failed")
                return False, None, None
            left_qpos_array = pos_array[:, :dofs]
            right_qpos_array = pos_array[:, dofs:]
            return True, left_qpos_array, right_qpos_array

    def generate_dense_whole_body_joint_traj(
            self,
            whole_body_joint_traj: np.ndarray = None,
            qvel_limits_array: np.ndarray = None,
            current_qpos: np.ndarray = None):
        if whole_body_joint_traj.shape[0] != qvel_limits_array.shape[0]:
            self.logger.error(
                "whole_body_joint_traj and qvel_limits_array must have the same length"
            )
            return False, None
        if whole_body_joint_traj.shape[1] != self.w1_params.full_body_dofs:
            self.logger.error(
                f"whole_body_joint_traj must be of shape (N, {self.w1_params.full_body_dofs})"
            )
            return False, None
        if qvel_limits_array.shape[1] != self.w1_params.full_body_dofs:
            self.logger.error(
                f"qvel_limits_array must be of shape (N, {self.w1_params.full_body_dofs})"
            )
            return False, None

        if current_qpos is not None:
            whole_body_joint_traj = np.vstack(
                [current_qpos, whole_body_joint_traj])
            qvel_limits_array = np.vstack(
                [qvel_limits_array[0], qvel_limits_array])

        dofs = self.w1_params.full_body_dofs
        pos_limits = np.vstack((self.w1_params.qpos_lb, self.w1_params.qpos_ub))
        vel_limits = self.w1_params.max_qvel
        acc_limits = self.w1_params.max_qacc

        traj_planner = ToppraTrajectoryPlanner(dofs, pos_limits, vel_limits,
                                               acc_limits)
        res, qpos_array, _, _, _ = traj_planner.plan(whole_body_joint_traj,
                                                     qvel_limits_array)
        if not res:
            self.logger.error(
                "whole body joint dense trajectory generation failed")
            return False, None
        return True, qpos_array

    def check_joint_traj_collision(self):
        pass

    def check_end_effector_traj_collision(self):
        # TODO: implement this in cartesian planner?
        pass

    def check_joint_traj_continuity(self):
        pass
