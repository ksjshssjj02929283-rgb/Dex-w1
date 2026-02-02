# ----------------------------------------------------------------------------
# Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import threading
import numpy as np
from copy import deepcopy
from ..utils.unit_conversion import user_pose_to_xpos, xpos_to_user_pose
from ..trajectory_planner.toppra_trajectory_planner import ToppraTrajectoryPlanner

from ..mgr_util.supervisor_view import SupervisorViewW1
from ..mgr_util.supervisor_utils import StaticUtilityW1
from .move_dual_arm import DualArmMovementBaseW1
from ...common import get_logger, time_consuming, StatusCode

from ..path_planner.blend_path import PathType, PathRnBezier, PathR3xSO3Bezier, weighted_norm

from dexe_rp import DexforceW1Kinematics, SRSConfig


class DualArmMoveLineW1(DualArmMovementBaseW1):

    def __init__(self,
                 w1_params,
                 supervisor,
                 kine_solver,
                 pin_kine_solver,
                 collision_checker,
                 hybrid_planner,
                 cartesian_planner,
                 cartesian_joint_planner,
                 stop_move_event: threading.Event,
                 time_step: float = 0.01):
        super().__init__(w1_params,
                         supervisor,
                         kine_solver,
                         pin_kine_solver,
                         collision_checker,
                         hybrid_planner,
                         cartesian_planner,
                         cartesian_joint_planner,
                         stop_move_event,
                         time_step=time_step)

    def _plan_to_waypoint(
            self,
            name: str,
            cur_qpos: np.ndarray,
            waypoints_dict: dict,
            speed_ratio: float,
            is_cartesian: bool = False,
            qpos_seed: np.ndarray = None,
            cartesian_step: float = None,
            is_local: bool = True,
            is_sparse: bool = True) -> tuple[StatusCode, np.ndarray]:
        if not is_local:
            qpos_array = cur_qpos[self.w1_params.index_infos[name]["index"]]
        else:
            qpos_array = cur_qpos
        (res, pos_raw_array, qvel_limits_array,
         _) = StaticUtilityW1.parse_waypoints_from_dict(self.supervisor_view,
                                                        name, waypoints_dict,
                                                        speed_ratio,
                                                        is_cartesian)
        if res == StatusCode.INPUT_EMPTY:  # 输入为空
            return StatusCode.SUCCESS, qpos_array
        elif res.value < 0:  # 输入不合法
            return res, None
        # 输入合法
        qvel_limits_array_ = np.vstack([
            self.w1_params.default_qvel[self.w1_params.index_infos[name]
                                        ["index"]], qvel_limits_array
        ])
        if is_cartesian:
            if not is_local:
                # Transfer global frame to local frame
                ref_torso_qpos = cur_qpos[self.w1_params.index_infos["torso"]
                                          ["index"]]
                ref_qpos = np.zeros(self.w1_params.full_body_dofs)
                ref_qpos[self.w1_params.index_infos["torso"]
                         ["index"]] = ref_torso_qpos
                res_fk, T_chassis_armbase = self.supervisor_view.get_xpos(
                    f"{name}_base", ref_qpos, is_local=False)
                if res_fk.value < 0:
                    self.logger.error(f"{name}_base solve fk failed!")
                    return res_fk, None
                temp = [
                    np.linalg.inv(T_chassis_armbase) @ user_pose_to_xpos(
                        pose[:3].tolist(), pose[3:].tolist())
                    for pose in pos_raw_array
                ]
                pos_raw_array = [
                    np.array(xpos_to_user_pose(xpos)) for xpos in temp
                ]
                cur_arm_qpos = cur_qpos[self.w1_params.index_infos[name]
                                        ["index"]]
            else:
                cur_arm_qpos = cur_qpos

            # In local frame
            res_fk, cur_xpos = self.supervisor_view._get_arm_kine(name).get_fk(
                cur_arm_qpos)
            if not res_fk:
                self.logger.error(f"{name} solve fk failed!")
                return StatusCode.FK_FAIL, None
            xpos_raw_list = [
                user_pose_to_xpos(pose[:3].tolist(), pose[3:].tolist())
                for pose in pos_raw_array
            ]
            xpos_raw_list_ = [cur_xpos] + xpos_raw_list
            res_plan, qpos_array = StaticUtilityW1.plan_cartesian_traj(
                self.supervisor_view, name, xpos_raw_list_, qpos_seed,
                qvel_limits_array_, cartesian_step)
            if res_plan.value < 0:
                return res_plan, None
        else:
            qpos_raw_array_ = np.vstack([cur_qpos, pos_raw_array])
            res_plan, qpos_array = StaticUtilityW1.plan_joint_traj(
                self.supervisor_view, name, qpos_raw_array_, is_sparse,
                qvel_limits_array_)
            if res_plan.value < 0:
                return res_plan, None

        return StatusCode.SUCCESS, qpos_array

    def _plan_to_waypoint_with_ee(
            self,
            name: str,
            cur_qpos: np.ndarray,
            cur_ee_qpos: np.ndarray,
            waypoints_dict: dict,
            speed_ratio: float,
            is_cartesian: bool = False,
            qpos_seed: np.ndarray = None,
            cartesian_step: float = None,
            is_local: bool = True,
            is_sparse: bool = True) -> tuple[StatusCode, np.ndarray, list]:
        if not is_local:
            qpos_array = cur_qpos[self.w1_params.index_infos[name]["index"]]
        else:
            qpos_array = cur_qpos

        (res, pos_raw_array, qvel_limits_array,
         arm_ee_state_list) = StaticUtilityW1.parse_waypoints_from_dict(
             self.supervisor_view, name, waypoints_dict, speed_ratio,
             is_cartesian)
        if res == StatusCode.INPUT_EMPTY:
            return StatusCode.SUCCESS, qpos_array, None
        elif res.value < 0:
            return res, None, None

        # without ee motion
        if arm_ee_state_list is None:
            res, qpos_array = self._plan_to_waypoint(name,
                                                     cur_qpos,
                                                     waypoints_dict,
                                                     speed_ratio,
                                                     is_cartesian,
                                                     qpos_seed,
                                                     cartesian_step,
                                                     is_local,
                                                     is_sparse=is_sparse)
            return res, qpos_array, None

        # with ee motion
        if not is_cartesian:
            pos_raw_array_ = np.vstack([cur_qpos, pos_raw_array])
        else:
            if not is_local:
                # Transfer global frame to local frame
                ref_torso_qpos = cur_qpos[self.w1_params.index_infos["torso"]
                                          ["index"]]
                ref_qpos = np.zeros(self.w1_params.full_body_dofs)
                ref_qpos[self.w1_params.index_infos["torso"]
                         ["index"]] = ref_torso_qpos
                res_fk, T_chassis_armbase = self.supervisor_view.get_xpos(
                    f"{name}_base", ref_qpos, is_local=False)
                if res_fk.value < 0:
                    self.logger.error(f"{name}_base solve fk failed!")
                    return res_fk, None, None
                temp = [
                    np.linalg.inv(T_chassis_armbase) @ user_pose_to_xpos(
                        pose[:3].tolist(), pose[3:].tolist())
                    for pose in pos_raw_array
                ]
                pos_raw_array = [
                    np.array(xpos_to_user_pose(xpos)) for xpos in temp
                ]
                cur_arm_qpos = cur_qpos[self.w1_params.index_infos[name]
                                        ["index"]]
            else:
                cur_arm_qpos = cur_qpos

            res_fk, cur_xpos = self.supervisor_view._get_arm_kine(name).get_fk(
                cur_arm_qpos)
            if not res_fk:
                self.logger.error(f"{name} solve fk failed!")
                return StatusCode.FK_FAIL, None, None
            pos_raw_array_ = np.vstack(
                [xpos_to_user_pose(cur_xpos), pos_raw_array])

        qvel_limits_array_ = np.vstack([
            self.w1_params.default_qvel[self.w1_params.index_infos[name]
                                        ["index"]], qvel_limits_array
        ])
        cur_ee_state = [0., cur_ee_qpos.tolist(), 0.]
        arm_ee_state_list_ = [cur_ee_state] + [
            ee_state if ee_state is not None else cur_ee_state
            for ee_state in arm_ee_state_list
        ]

        # segment raw waypoint
        arm_ee_trigger_list = []  # (count, ee_qpos)
        res, segment_list = StaticUtilityW1.plan_arm_ee_trigger(
            self.supervisor_view, arm_ee_state_list_)
        if res.value < 0:
            return res, None, None

        total_count = 0
        total_qpos_array = None
        if is_cartesian:
            cur_qpos_seed = qpos_seed

        for start_idx, end_idx in segment_list:
            seg_pos_array = pos_raw_array_[start_idx:end_idx + 1]
            seg_qvel_limits_array = qvel_limits_array_[start_idx:end_idx + 1]
            if is_cartesian:
                seg_xpos_list = [
                    user_pose_to_xpos(pose[:3].tolist(), pose[3:].tolist())
                    for pose in seg_pos_array
                ]
                res_plan, seg_qpos_samples_array = StaticUtilityW1.plan_cartesian_traj(
                    self.supervisor_view, name, seg_xpos_list, cur_qpos_seed,
                    seg_qvel_limits_array, cartesian_step)
            else:
                res_plan, seg_qpos_samples_array = StaticUtilityW1.plan_joint_traj(
                    self.supervisor_view, name, seg_pos_array, is_sparse,
                    seg_qvel_limits_array)
            if res_plan.value < 0:
                return res_plan, None, None

            # pre delay
            pre_delay_time = arm_ee_state_list_[end_idx][0]
            if pre_delay_time >= self.supervisor_view.TIME_STEP:
                pre_delay_count = max(
                    1,
                    int(np.ceil(pre_delay_time /
                                self.supervisor_view.TIME_STEP)))
                seg_qpos_samples_array = np.vstack([
                    seg_qpos_samples_array,
                    np.repeat(seg_qpos_samples_array[-1:],
                              pre_delay_count,
                              axis=0)
                ])
            # ee trigger
            arm_ee_trigger_list.append(
                (total_count + seg_qpos_samples_array.shape[0] - 1,
                 arm_ee_state_list_[end_idx][1]))
            # post delay
            post_delay_time = arm_ee_state_list_[end_idx][2]
            if post_delay_time >= self.supervisor_view.TIME_STEP:
                post_delay_count = max(
                    1,
                    int(
                        np.ceil(post_delay_time /
                                self.supervisor_view.TIME_STEP)))
                seg_qpos_samples_array = np.vstack([
                    seg_qpos_samples_array,
                    np.repeat(seg_qpos_samples_array[-1:],
                              post_delay_count,
                              axis=0)
                ])

            if total_qpos_array is None:
                total_qpos_array = seg_qpos_samples_array
            else:
                total_qpos_array = np.vstack(
                    ([total_qpos_array, seg_qpos_samples_array[1:]]))

            if is_cartesian:
                cur_qpos_seed = total_qpos_array[-1]

            total_count = total_qpos_array.shape[0]

        return StatusCode.SUCCESS, total_qpos_array, arm_ee_trigger_list

    def _plan_linear_dual_arm_sync(self,
                                   cur_left_qpos: np.ndarray,
                                   cur_right_qpos: np.ndarray,
                                   waypoints_dict: dict,
                                   left_speed_ratio: float,
                                   right_speed_ratio: float,
                                   left_qpos_seed: np.ndarray = None,
                                   right_qpos_seed: np.ndarray = None,
                                   cartesian_step: float = None,
                                   is_local: bool = True):
        if not is_local:
            left_qpos_array = cur_left_qpos[
                self.w1_params.index_infos["left_arm"]["index"]]
            right_qpos_array = cur_right_qpos[
                self.w1_params.index_infos["right_arm"]["index"]]
        else:
            left_qpos_array = cur_left_qpos
            right_qpos_array = cur_right_qpos
        left_parse_res, left_pos_raw_array, left_qvel_limits_array, _ = StaticUtilityW1.parse_waypoints_from_dict(
            self.supervisor_view,
            "left_arm",
            waypoints_dict,
            left_speed_ratio,
            is_cartesian=True)
        right_parse_res, right_pos_raw_array, right_qvel_limits_array, _ = StaticUtilityW1.parse_waypoints_from_dict(
            self.supervisor_view,
            "right_arm",
            waypoints_dict,
            right_speed_ratio,
            is_cartesian=True)
        # TODO: 这里res=0的情况判断需要完善，比如一个手臂有输入，一个手臂无输入
        if left_parse_res == StatusCode.INPUT_EMPTY or right_parse_res == StatusCode.INPUT_EMPTY:
            return StatusCode.SUCCESS, left_qpos_array, right_qpos_array
        elif left_parse_res.value < 0 or right_parse_res.value < 0:
            return StatusCode.FAIL, None, None

        left_qvel_limits_array_ = np.vstack([
            self.w1_params.default_qvel[self.w1_params.index_infos["left_arm"]
                                        ["index"]], left_qvel_limits_array
        ])
        right_qvel_limits_array_ = np.vstack([
            self.w1_params.default_qvel[self.w1_params.index_infos["right_arm"]
                                        ["index"]], right_qvel_limits_array
        ])

        if not is_local:
            # Transfer global frame to local frame
            ref_torso_qpos = cur_left_qpos[self.w1_params.index_infos["torso"]
                                           ["index"]]
            ref_qpos = np.zeros(self.w1_params.full_body_dofs)
            ref_qpos[self.w1_params.index_infos["torso"]
                     ["index"]] = ref_torso_qpos
            res_left_fk, T_chassis_left_armbase = self.supervisor_view.get_xpos(
                "left_arm_base", ref_qpos, is_local=False)
            if res_left_fk.value < 0:
                self.logger.error("left_arm_base solve fk failed!")
                return res_left_fk, None, None
            res_right_fk, T_chassis_right_armbase = self.supervisor_view.get_xpos(
                "right_arm_base", ref_qpos, is_local=False)
            if res_right_fk.value < 0:
                self.logger.error("right_arm_base solve fk failed!")
                return res_right_fk, None, None
            left_temp = [
                np.linalg.inv(T_chassis_left_armbase) @ user_pose_to_xpos(
                    pose[:3].tolist(), pose[3:].tolist())
                for pose in left_pos_raw_array
            ]
            right_temp = [
                np.linalg.inv(T_chassis_right_armbase) @ user_pose_to_xpos(
                    pose[:3].tolist(), pose[3:].tolist())
                for pose in right_pos_raw_array
            ]
            left_pos_raw_array = [
                np.array(xpos_to_user_pose(xpos) for xpos in left_temp)
            ]
            right_pos_raw_array = [
                np.array(xpos_to_user_pose(xpos) for xpos in right_temp)
            ]
            cur_left_arm_qpos = cur_left_qpos[
                self.w1_params.index_infos["left_arm"]["index"]]
            cur_right_arm_qpos = cur_right_qpos[
                self.w1_params.index_infos["right_arm"]["index"]]
        else:
            cur_left_arm_qpos = cur_left_qpos
            cur_right_arm_qpos = cur_right_qpos

        res_left_fk, cur_left_xpos = self.supervisor_view._get_arm_kine(
            "left_arm").get_fk(cur_left_arm_qpos)
        if not res_left_fk:
            self.logger.error("left_arm solve fk failed!")
            return StatusCode.FK_FAIL, None, None
        res_right_fk, cur_right_xpos = self.supervisor_view._get_arm_kine(
            "right_arm").get_fk(cur_right_arm_qpos)
        if not res_right_fk:
            self.logger.error("right_arm solve fk failed!")
            return StatusCode.FK_FAIL, None, None
        left_xpos_raw_list = [
            user_pose_to_xpos(pose[:3].tolist(), pose[3:].tolist())
            for pose in left_pos_raw_array
        ]
        right_xpos_raw_list = [
            user_pose_to_xpos(pose[:3].tolist(), pose[3:].tolist())
            for pose in right_pos_raw_array
        ]
        left_xpos_raw_list_ = [cur_left_xpos] + left_xpos_raw_list
        right_xpos_raw_list_ = [cur_right_xpos] + right_xpos_raw_list

        if len(left_xpos_raw_list_) != len(right_xpos_raw_list_):
            self.logger.error("Left and right arm waypoint number mismatch.")
            return StatusCode.DIM_MISMATCH, None, None

        res_plan, left_qpos_array, right_qpos_array = self.plan_dual_arm_cartesian_traj(
            left_xpos_raw_list_, right_xpos_raw_list_, left_qpos_seed,
            right_qpos_seed, left_qvel_limits_array_, right_qvel_limits_array_,
            cartesian_step)
        if res_plan.value < 0:
            return res_plan, None, None
        return StatusCode.SUCCESS, left_qpos_array, right_qpos_array

    def plan_dual_arm_cartesian_traj(
        self,
        left_xpos_list: list[np.ndarray],
        right_xpos_list: list[np.ndarray],
        left_qpos_seed: np.ndarray,
        right_qpos_seed: np.ndarray,
        left_qvel_limits_array: np.ndarray = None,
        right_qvel_limits_array: np.ndarray = None,
        cartesian_step: float = 0.005
    ) -> tuple[StatusCode, np.ndarray, np.ndarray]:
        left_arm_kine = self.supervisor_view._get_arm_kine("left_arm")
        if left_arm_kine is None:
            return StatusCode.NAME_MISMATCH, None, None
        right_arm_kine = self.supervisor_view._get_arm_kine("right_arm")
        if right_arm_kine is None:
            return StatusCode.NAME_MISMATCH, None, None
        # cartesian interpolation
        left_prefix_dist_list = [0.]
        for idx in range(len(left_xpos_list) - 1):
            dist = np.linalg.norm(left_xpos_list[idx + 1][:3, 3] -
                                  left_xpos_list[idx][:3, 3])
            left_prefix_dist_list.append(left_prefix_dist_list[-1] + dist)
        left_total_dist = left_prefix_dist_list[-1]
        right_prefix_dist_list = [0.]
        for idx in range(len(right_xpos_list) - 1):
            dist = np.linalg.norm(right_xpos_list[idx + 1][:3, 3] -
                                  right_xpos_list[idx][:3, 3])
            right_prefix_dist_list.append(right_prefix_dist_list[-1] + dist)
        right_total_dist = right_prefix_dist_list[-1]

        left_interpolate_num_list = StaticUtilityW1.cal_path_seg_interpolate_num(
            self.supervisor_view, left_xpos_list, cartesian_step)
        right_interpolate_num_list = StaticUtilityW1.cal_path_seg_interpolate_num(
            self.supervisor_view, right_xpos_list, cartesian_step)
        if len(left_interpolate_num_list) != len(right_interpolate_num_list):
            self.logger.error(
                "Left and right arm waypoint segment number mismatch.")
            return StatusCode.INPUT_INVALID, None, None

        interpolate_num_list = []
        for i in range(len(left_interpolate_num_list)):
            left_interpolate_num = left_interpolate_num_list[i]
            right_interpolate_num = right_interpolate_num_list[i]
            seg_num_sample = max(left_interpolate_num, right_interpolate_num)
            interpolate_num_list.append(seg_num_sample)

        left_bezier_planner = PathR3xSO3Bezier(left_xpos_list)
        if not left_bezier_planner.calc_path(PathType.Bezier5thCartesian, 0.01):
            self.logger.error("Left arm bezier path planning failed.")
            return StatusCode.PLAN_PATH_FAIL, None, None

        right_bezier_planner = PathR3xSO3Bezier(right_xpos_list)
        if not right_bezier_planner.calc_path(PathType.Bezier5thCartesian,
                                              0.01):
            self.logger.error("Right arm bezier path planning failed.")
            return StatusCode.PLAN_PATH_FAIL, None, None

        left_path_seg_list, right_path_seg_list = left_bezier_planner.get_path_segments(
        ), right_bezier_planner.get_path_segments()
        if len(left_path_seg_list) != 2 * len(left_xpos_list) - 3 or \
              len(right_path_seg_list) != 2 * len(right_xpos_list) - 3:
            self.logger.error(
                "Left and right arm path segment number mismatch with waypoint number."
            )
            return StatusCode.INPUT_INVALID, None, None

        left_linear_seg_list, left_blend_seg_list = [], []
        right_linear_seg_list, right_blend_seg_list = [], []
        for i in range(len(left_path_seg_list)):
            if i % 2 == 0:
                left_linear_seg_list.append(left_path_seg_list[i])
                right_linear_seg_list.append(right_path_seg_list[i])
            else:
                left_blend_seg_list.append(left_path_seg_list[i])
                right_blend_seg_list.append(right_path_seg_list[i])

        left_rearrange_seg_list, right_rearrange_seg_list = [], []
        if len(left_blend_seg_list) != 0 and len(right_blend_seg_list) != 0:
            for i in range(len(left_linear_seg_list)):
                if i == 0:
                    left_rearrange_seg_list.append(
                        ([], left_linear_seg_list[i], left_blend_seg_list[i]))
                elif i == len(left_linear_seg_list) - 1:
                    left_rearrange_seg_list.append(
                        (left_blend_seg_list[i - 1], left_linear_seg_list[i],
                         []))
                else:
                    left_rearrange_seg_list.append(
                        (left_blend_seg_list[i - 1], left_linear_seg_list[i],
                         left_blend_seg_list[i]))
            for i in range(len(right_linear_seg_list)):
                if i == 0:
                    right_rearrange_seg_list.append(
                        ([], right_linear_seg_list[i], right_blend_seg_list[i]))
                elif i == len(right_linear_seg_list) - 1:
                    right_rearrange_seg_list.append(
                        (right_blend_seg_list[i - 1], right_linear_seg_list[i],
                         []))
                else:
                    right_rearrange_seg_list.append(
                        (right_blend_seg_list[i - 1], right_linear_seg_list[i],
                         right_blend_seg_list[i]))
        else:
            left_rearrange_seg_list.append(([], left_linear_seg_list[0], []))
            right_rearrange_seg_list.append(([], right_linear_seg_list[0], []))

        left_xpos_sample_array, right_xpos_sample_array = [], []
        for idx, (pre_blend_seg, linear_seg,
                  post_blend_seg) in enumerate(left_rearrange_seg_list):
            interpolate_num = interpolate_num_list[idx]
            pre_blend_seg_length = pre_blend_seg.get_length(
            ) if pre_blend_seg != [] else 0.0
            linear_seg_length = linear_seg.get_length()
            post_blend_seg_length = post_blend_seg.get_length(
            ) if post_blend_seg != [] else 0.0

            # pre_blend_interpolate_num = max(0, int(np.ceil(interpolate_num * (pre_blend_seg_length / 2) / (pre_blend_seg_length / 2 + linear_seg_length + post_blend_seg_length / 2))))
            # linear_interpolate_num = max(0, int(np.ceil(interpolate_num * (linear_seg_length) / (pre_blend_seg_length / 2 + linear_seg_length + post_blend_seg_length / 2))))
            # post_blend_interpolate_num = max(0, int(np.ceil(interpolate_num * (post_blend_seg_length / 2) / (pre_blend_seg_length / 2 + linear_seg_length + post_blend_seg_length / 2))))

            weights = np.array([
                pre_blend_seg_length / 2, linear_seg_length,
                post_blend_seg_length / 2
            ])
            den = weights.sum()
            if den <= 0:
                pre_blend_interpolate_num = linear_interpolate_num = post_blend_interpolate_num = 0
            else:
                raw = interpolate_num * weights / den
                base = np.floor(raw).astype(int)
                remain = int(interpolate_num - base.sum())
                frac_order = np.argsort(raw - base)[::-1]
                base[frac_order[:remain]] += 1
                pre_blend_interpolate_num, linear_interpolate_num, post_blend_interpolate_num = base.tolist(
                )

            sample_seg_list = []
            pre_blend_s = np.linspace(
                pre_blend_seg.get_s0() + pre_blend_seg.get_length() / 2,
                pre_blend_seg.get_s0() + pre_blend_seg.get_length(),
                pre_blend_interpolate_num) if pre_blend_seg != [] else None
            linear_s = np.linspace(
                linear_seg.get_s0(),
                linear_seg.get_s0() + linear_seg.get_length(),
                linear_interpolate_num)
            post_blend_s = np.linspace(
                post_blend_seg.get_s0(),
                post_blend_seg.get_s0() + post_blend_seg.get_length() / 2,
                post_blend_interpolate_num) if post_blend_seg != [] else None

            if pre_blend_s is not None:
                for s in pre_blend_s:
                    if s == pre_blend_s[0]:
                        continue
                    sample_seg_list.append(pre_blend_seg.get_config(s))
            for s in linear_s:
                if pre_blend_s is not None and s == linear_s[0]:
                    continue
                sample_seg_list.append(linear_seg.get_config(s))
            if post_blend_s is not None:
                for s in post_blend_s:
                    if s == post_blend_s[0]:
                        continue
                    sample_seg_list.append(post_blend_seg.get_config(s))

            left_xpos_sample_array.append(sample_seg_list)

        for idx, (pre_blend_seg, linear_seg,
                  post_blend_seg) in enumerate(right_rearrange_seg_list):
            interpolate_num = interpolate_num_list[idx]
            pre_blend_seg_length = pre_blend_seg.get_length(
            ) if pre_blend_seg != [] else 0.0
            linear_seg_length = linear_seg.get_length()
            post_blend_seg_length = post_blend_seg.get_length(
            ) if post_blend_seg != [] else 0.0

            # pre_blend_interpolate_num = max(0, int(np.ceil(interpolate_num * (pre_blend_seg_length / 2) / (pre_blend_seg_length / 2 + linear_seg_length + post_blend_seg_length / 2))))
            # linear_interpolate_num = max(0, int(np.ceil(interpolate_num * (linear_seg_length) / (pre_blend_seg_length / 2 + linear_seg_length + post_blend_seg_length / 2))))
            # post_blend_interpolate_num = max(0, int(np.ceil(interpolate_num * (post_blend_seg_length / 2) / (pre_blend_seg_length / 2 + linear_seg_length + post_blend_seg_length / 2))))

            weights = np.array([
                pre_blend_seg_length / 2, linear_seg_length,
                post_blend_seg_length / 2
            ])
            den = weights.sum()
            if den <= 0:
                pre_blend_interpolate_num = linear_interpolate_num = post_blend_interpolate_num = 0
            else:
                raw = interpolate_num * weights / den
                base = np.floor(raw).astype(int)
                remain = int(interpolate_num - base.sum())
                frac_order = np.argsort(raw - base)[::-1]
                base[frac_order[:remain]] += 1
                pre_blend_interpolate_num, linear_interpolate_num, post_blend_interpolate_num = base.tolist(
                )

            sample_seg_list = []
            pre_blend_s = np.linspace(
                pre_blend_seg.get_s0() + pre_blend_seg.get_length() / 2,
                pre_blend_seg.get_s0() + pre_blend_seg.get_length(),
                pre_blend_interpolate_num) if pre_blend_seg != [] else None
            linear_s = np.linspace(
                linear_seg.get_s0(),
                linear_seg.get_s0() + linear_seg.get_length(),
                linear_interpolate_num)
            post_blend_s = np.linspace(
                post_blend_seg.get_s0(),
                post_blend_seg.get_s0() + post_blend_seg.get_length() / 2,
                post_blend_interpolate_num) if post_blend_seg != [] else None

            if pre_blend_s is not None:
                for s in pre_blend_s:
                    if s == pre_blend_s[0]:
                        continue
                    sample_seg_list.append(pre_blend_seg.get_config(s))
            for s in linear_s:
                if s == linear_s[0] and pre_blend_s is not None:
                    continue
                sample_seg_list.append(linear_seg.get_config(s))
            if post_blend_s is not None:
                for s in post_blend_s:
                    if s == post_blend_s[0]:
                        continue
                    sample_seg_list.append(post_blend_seg.get_config(s))

            right_xpos_sample_array.append(sample_seg_list)

        left_xpos_sample_array = [
            xpos for seg in left_xpos_sample_array for xpos in seg
        ]
        right_xpos_sample_array = [
            xpos for seg in right_xpos_sample_array for xpos in seg
        ]

        # vel align
        if left_qvel_limits_array is None:
            left_qvel_limits_array_ = None
        else:
            idx = 0
            left_qvel_limits_list_ = [left_qvel_limits_array[idx]]
            cur_left_xpos = left_xpos_sample_array[0]
            cur_dist = 0.
            for left_xpos in left_xpos_sample_array[1:-1]:
                cur_dist += np.linalg.norm(left_xpos[:3, 3] -
                                           cur_left_xpos[:3, 3])
                if cur_dist >= left_prefix_dist_list[idx]:
                    idx += 1
                left_qvel_limits_list_.append(left_qvel_limits_array[idx])
                cur_left_xpos = left_xpos
            left_qvel_limits_list_.append(left_qvel_limits_array[-1])
        left_qvel_limits_array_ = np.array(left_qvel_limits_list_)

        if right_qvel_limits_array is None:
            right_qvel_limits_array_ = None
        else:
            idx = 0
            right_qvel_limits_list_ = [right_qvel_limits_array[idx]]
            cur_right_xpos = right_xpos_sample_array[0]
            cur_dist = 0.
            for right_xpos in right_xpos_sample_array[1:-1]:
                cur_dist += np.linalg.norm(right_xpos[:3, 3] -
                                           cur_right_xpos[:3, 3])
                if cur_dist >= right_prefix_dist_list[idx]:
                    idx += 1
                right_qvel_limits_list_.append(right_qvel_limits_array[idx])
                cur_right_xpos = right_xpos
            right_qvel_limits_list_.append(right_qvel_limits_array[-1])
        right_qvel_limits_array_ = np.array(right_qvel_limits_list_)

        qvel_limits_array_ = np.hstack(
            [left_qvel_limits_array_, right_qvel_limits_array_])

        # solve ik
        left_qpos_list = []
        cur_left_qseed = left_qpos_seed
        for cur_left_xpos in left_xpos_sample_array:
            res_left_ik, cur_left_qpos = left_arm_kine.get_nearest_ik(
                xpos=cur_left_xpos, qpos_seed=cur_left_qseed)
            if not res_left_ik:
                self.logger.error(
                    f"Left arm solve ik failed, cur_qseed(deg): {np.rad2deg(cur_left_qseed)}."
                )
                return StatusCode.IK_FAIL, None, None
            if not np.allclose(
                    cur_left_qpos, cur_left_qseed, atol=np.deg2rad(90)):
                self.logger.error(f"Left arm qpos traj is not continus\n" + \
                                  f"cur_qpos(deg): {np.rad2deg(cur_left_qpos)}\n" + \
                                  f"cur_qseed(deg): {np.rad2deg(cur_left_qseed)}.")
                return StatusCode.IK_FAIL, None, None
            left_qpos_list.append(cur_left_qpos)
            cur_left_qseed = cur_left_qpos

        left_qpos_array = np.array(left_qpos_list)

        right_qpos_list = []
        cur_right_qseed = right_qpos_seed
        for cur_right_xpos in right_xpos_sample_array:
            res_right_ik, cur_right_qpos = right_arm_kine.get_nearest_ik(
                xpos=cur_right_xpos, qpos_seed=cur_right_qseed)
            if not res_right_ik:
                self.logger.error(
                    f"Right arm solve ik failed, cur_qseed(deg): {np.rad2deg(cur_right_qseed)}."
                )
                return StatusCode.IK_FAIL, None, None
            if not np.allclose(
                    cur_right_qpos, cur_right_qseed, atol=np.deg2rad(90)):
                self.logger.error(f"Right arm qpos traj is not continus\n" + \
                                  f"cur_qpos(deg): {np.rad2deg(cur_right_qpos)}\n" + \
                                  f"cur_qseed(deg): {np.rad2deg(cur_right_qseed)}.")
                return StatusCode.IK_FAIL, None, None
            right_qpos_list.append(cur_right_qpos)
            cur_right_qseed = cur_right_qpos

        right_qpos_array = np.array(right_qpos_list)

        qpos_array = np.hstack([left_qpos_array, right_qpos_array])

        dofs = 2 * self.w1_params.index_infos["left_arm"]["dofs"]
        left_index = self.w1_params.index_infos["left_arm"]["index"]
        right_index = self.w1_params.index_infos["right_arm"]["index"]
        left_vel_limits = self.w1_params.max_qvel[left_index]
        right_vel_limits = self.w1_params.max_qvel[right_index]
        vel_limits = np.concatenate([left_vel_limits, right_vel_limits])
        left_acc_limits = self.w1_params.max_qacc[left_index]
        right_acc_limits = self.w1_params.max_qacc[right_index]
        acc_limits = np.concatenate([left_acc_limits, right_acc_limits])
        traj_planner = ToppraTrajectoryPlanner(dofs, vel_limits, acc_limits)
        res, qpos_array, _, _, _ = traj_planner.plan(qpos_array,
                                                     qvel_limits_array_,
                                                     self.TIME_STEP)

        if not res:
            self.logger.error("Dual arm cartesian traj planning failed.")
            return StatusCode.PLAN_TRAJ_FAIL, None, None
        left_qpos_array = qpos_array[:, :self.w1_params.
                                     index_infos["left_arm"]["dofs"]]
        right_qpos_array = qpos_array[:, self.w1_params.
                                      index_infos["left_arm"]["dofs"]:]

        return StatusCode.SUCCESS, left_qpos_array, right_qpos_array

    @time_consuming
    def plan_linear_full_body(
            self,
            start_qpos: np.ndarray,
            start_left_arm_ee_qpos: np.ndarray,
            start_right_arm_ee_qpos: np.ndarray,
            waypoints_dict: dict,
            left_qpos_seed: np.ndarray = None,
            right_qpos_seed: np.ndarray = None,
            cartesian_step: float = 0.005,
            speed_ratio: float = 1.0,
            is_local: bool = True,
            is_sync: bool = False) -> tuple[bool, np.ndarray, list, list]:
        """规划各部件独立关节空间运动，arm执行笛卡尔空间运动
        - 每个key接收多个qpos路径点，key省略视为不控制
        - 同一部件之间的key需要对齐：torso、head、left_xx和right_xx
        - 未指定速度的路径点(None)按speed_ratio补全
        - left_arm_ee/right_arm_ee的每个路径点输入格式为[pre_delay_time, ee_qpos, post_delay_time]
        - key对应的value为 None / [] 视为不控制，其余特殊输入视为不支持

        Args:
            start_qpos (np.ndarray): 规划起始的全身qpos
            start_left_arm_ee_qpos (np.ndarray): 规划起始的left_arm_ee_qpos
            start_right_arm_ee_qpos (np.ndarray): 规划起始的right_arm_ee_qpos   
            waypoints_dict (dict): {"torso": [torso_qpos1,torso_qpos2,], "torso_speed_ratio": [torso_speed1, torso_speed2,],   
                                    "head": [head_qpos1,], "head_speed_ratio": [head_speed1,],   
                                    "left_arm": [left_arm_qpos1,], "left_arm_speed_ratio": [left_arm_speed1,], 
                                    "left_arm_ee": [[0.,left_arm_ee_qpos1,0.5],],  
                                    "right_arm": [right_arm_qpos1,], "right_arm_speed_ratio": [right_arm_speed1], 
                                    "right_arm_ee": [[0.,right_arm_ee_qpos1,0.5],]}
            left_qpos_seed (np.ndarray, optional): ik reference seed. Defaults to None.
            right_qpos_seed (np.ndarray, optional): ik reference seed. Defaults to None.
            cartesian_step (float, optional): cartesian sample step. Defaults to 0.005.
            speed_ratio (float, optional): 相对于默认关节速度的比率. Defaults to 1.0.
            is_local (bool, optional): w.r.t. arm base frame or chassis frame. Defaults to True.
            is_sync (bool, optional): dual arm move line sync planning. Defaults to False.

        Returns:
            tuple[StatusCode, np.ndarray, list, list]: 规划成功与否，全身关节轨迹，左末端执行器轨迹，右末端执行器轨迹
        """
        if not self.supervisor_view._check_qpos_limits("full_body", start_qpos):
            return StatusCode.EXCEED_LIMIT, None, None, None

        start_torso_qpos = start_qpos[
            self.supervisor_view.w1_params.torso_index]
        start_head_qpos = start_qpos[self.supervisor_view.w1_params.head_index]
        start_left_arm_qpos = start_qpos[
            self.supervisor_view.w1_params.left_arm_index]
        start_right_arm_qpos = start_qpos[
            self.supervisor_view.w1_params.right_arm_index]

        if left_qpos_seed is None:
            left_qpos_seed = start_left_arm_qpos
        if right_qpos_seed is None:
            right_qpos_seed = start_right_arm_qpos

        self.supervisor_view.logger.debug(
            f"plan_linear_full_body() input cur_qpos: {start_qpos}")
        self.supervisor_view.logger.debug(
            f"plan_linear_full_body() input cur_ee_qpos: {start_left_arm_ee_qpos}, {start_right_arm_ee_qpos}"
        )
        self.supervisor_view.logger.debug(
            "plan_linear_full_body() input waypoints_dict:")
        self.supervisor_view.logger.debug(waypoints_dict)
        self.supervisor_view.logger.debug(
            f"plan_linear_full_body() input speed_ratio: {speed_ratio}")

        # torso
        res_torso, torso_qpos_array = self._plan_to_waypoint(
            "torso", start_torso_qpos, waypoints_dict, speed_ratio)
        if res_torso.value < 0:
            self.supervisor_view.logger.error("Plan to torso traj failed.")
            return res_torso, None, None, None

        # head
        res_head, head_qpos_array = self._plan_to_waypoint(
            "head", start_head_qpos, waypoints_dict, speed_ratio)
        if res_head.value < 0:
            self.supervisor_view.logger.error("Plan to head traj failed.")
            return res_head, None, None, None

        if not is_sync:
            # left_arm
            left_arm_ee_trigger_list = None
            if not self.supervisor_view._has_arm_ee_client(
                    "left_arm_ee", False):
                if is_local:
                    (res_left_arm,
                     left_arm_qpos_array) = self._plan_to_waypoint(
                         "left_arm",
                         start_left_arm_qpos,
                         waypoints_dict,
                         speed_ratio,
                         is_cartesian=True,
                         qpos_seed=left_qpos_seed,
                         cartesian_step=cartesian_step,
                         is_local=True)
                else:
                    (res_left_arm,
                     left_arm_qpos_array) = self._plan_to_waypoint(
                         "left_arm",
                         start_qpos,
                         waypoints_dict,
                         speed_ratio,
                         is_cartesian=True,
                         qpos_seed=left_qpos_seed,
                         cartesian_step=cartesian_step,
                         is_local=False)
                if res_left_arm.value < 0:
                    self.supervisor_view.logger.error(
                        "Plan to left arm traj failed.")
                    return res_left_arm, None, None, None
            else:
                if not self.supervisor_view._check_qpos_limits(
                        "left_arm_ee", start_left_arm_ee_qpos):
                    return StatusCode.EXCEED_LIMIT, None, None, None
                if is_local:
                    (res_left_arm, left_arm_qpos_array,
                     left_arm_ee_trigger_list) = self._plan_to_waypoint_with_ee(
                         "left_arm",
                         start_left_arm_qpos,
                         start_left_arm_ee_qpos,
                         waypoints_dict,
                         speed_ratio,
                         is_cartesian=True,
                         qpos_seed=left_qpos_seed,
                         cartesian_step=cartesian_step,
                         is_local=True)
                else:
                    (res_left_arm, left_arm_qpos_array,
                     left_arm_ee_trigger_list) = self._plan_to_waypoint_with_ee(
                         "left_arm",
                         start_qpos,
                         start_left_arm_ee_qpos,
                         waypoints_dict,
                         speed_ratio,
                         is_cartesian=True,
                         qpos_seed=left_qpos_seed,
                         cartesian_step=cartesian_step,
                         is_local=False)
                if res_left_arm.value < 0:
                    self.supervisor_view.logger.error(
                        "Plan to left arm traj with ee failed.")
                    return res_left_arm, None, None, None

            # right_arm
            right_arm_ee_trigger_list = None
            if not self.supervisor_view._has_arm_ee_client(
                    "right_arm_ee", False):
                if is_local:
                    (res_right_arm,
                     right_arm_qpos_array) = self._plan_to_waypoint(
                         "right_arm",
                         start_right_arm_qpos,
                         waypoints_dict,
                         speed_ratio,
                         is_cartesian=True,
                         qpos_seed=right_qpos_seed,
                         cartesian_step=cartesian_step,
                         is_local=True)
                else:
                    (res_right_arm,
                     right_arm_qpos_array) = self._plan_to_waypoint(
                         "right_arm",
                         start_qpos,
                         waypoints_dict,
                         speed_ratio,
                         is_cartesian=True,
                         qpos_seed=right_qpos_seed,
                         cartesian_step=cartesian_step,
                         is_local=False)
                if res_right_arm.value < 0:
                    self.supervisor_view.logger.error(
                        "Plan to right arm traj failed.")
                    return res_right_arm, None, None, None
            else:
                if not self.supervisor_view._check_qpos_limits(
                        "right_arm_ee", start_right_arm_ee_qpos):
                    return StatusCode.EXCEED_LIMIT, None, None, None
                if is_local:
                    (res_right_arm, right_arm_qpos_array,
                     right_arm_ee_trigger_list
                    ) = self._plan_to_waypoint_with_ee(
                        "right_arm",
                        start_right_arm_qpos,
                        start_right_arm_ee_qpos,
                        waypoints_dict,
                        speed_ratio,
                        is_cartesian=True,
                        qpos_seed=right_qpos_seed,
                        cartesian_step=cartesian_step,
                        is_local=True)
                else:
                    (res_right_arm, right_arm_qpos_array,
                     right_arm_ee_trigger_list
                    ) = self._plan_to_waypoint_with_ee(
                        "right_arm",
                        start_qpos,
                        start_right_arm_ee_qpos,
                        waypoints_dict,
                        speed_ratio,
                        is_cartesian=True,
                        qpos_seed=right_qpos_seed,
                        cartesian_step=cartesian_step,
                        is_local=False)
                if res_right_arm.value < 0:
                    self.supervisor_view.logger.error(
                        "Plan to right arm traj with ee failed.")
                    return res_right_arm, None, None, None
        else:
            # dual arm move line sync planning
            left_arm_ee_trigger_list, right_arm_ee_trigger_list = None, None
            if not self.supervisor_view._has_arm_ee_client("left_arm_ee", False) or \
                not self.supervisor_view._has_arm_ee_client("right_arm_ee", False):
                if is_local:
                    (res_dual_arm, left_arm_qpos_array, right_arm_qpos_array) \
                     = self.supervisor_view._plan_linear_dual_arm_sync(
                        start_left_arm_qpos,
                        start_right_arm_qpos,
                        waypoints_dict,
                        speed_ratio,
                        speed_ratio,
                        left_qpos_seed=left_qpos_seed,
                        right_qpos_seed=right_qpos_seed,
                        cartesian_step=cartesian_step,
                        is_local=True
                    )
                    if res_dual_arm:
                        left_arm_qpos_array = np.vstack([
                            start_qpos[
                                self.supervisor_view.w1_params.left_arm_index],
                            left_arm_qpos_array
                        ])
                        right_arm_qpos_array = np.vstack([
                            start_qpos[
                                self.supervisor_view.w1_params.right_arm_index],
                            right_arm_qpos_array
                        ])
                else:
                    (res_dual_arm, left_arm_qpos_array, right_arm_qpos_array) \
                     = self.supervisor_view._plan_linear_dual_arm_sync(
                        start_qpos,
                        start_qpos,
                        waypoints_dict,
                        speed_ratio,
                        speed_ratio,
                        left_qpos_seed=left_qpos_seed,
                        right_qpos_seed=right_qpos_seed,
                        cartesian_step=cartesian_step,
                        is_local=False
                    )
                    if res_dual_arm:
                        left_arm_qpos_array = np.vstack([
                            start_qpos[
                                self.supervisor_view.w1_params.left_arm_index],
                            left_arm_qpos_array
                        ])
                        right_arm_qpos_array = np.vstack([
                            start_qpos[
                                self.supervisor_view.w1_params.right_arm_index],
                            right_arm_qpos_array
                        ])
                if res_dual_arm.value < 0:
                    self.supervisor_view.logger.error(
                        "Plan to dual arm sync traj failed.")
                    return res_dual_arm, None, None, None

            # TODO: complete this: the following part originally designed for synchronize dual arm with ee, but not implemented yet
            else:
                if is_local:
                    (res_dual_arm, left_arm_qpos_array, right_arm_qpos_array) \
                     = self.supervisor_view._plan_linear_dual_arm_sync(
                        start_left_arm_qpos,
                        start_right_arm_qpos,
                        waypoints_dict,
                        speed_ratio,
                        speed_ratio,
                        left_qpos_seed=left_qpos_seed,
                        right_qpos_seed=right_qpos_seed,
                        cartesian_step=cartesian_step,
                        is_local=True
                    )
                    if res_dual_arm:
                        left_arm_qpos_array = np.vstack([
                            start_qpos[
                                self.supervisor_view.w1_params.left_arm_index],
                            left_arm_qpos_array
                        ])
                        right_arm_qpos_array = np.vstack([
                            start_qpos[
                                self.supervisor_view.w1_params.right_arm_index],
                            right_arm_qpos_array
                        ])
                else:
                    (res_dual_arm, left_arm_qpos_array, right_arm_qpos_array) \
                     = self.supervisor_view._plan_linear_dual_arm_sync(
                        start_qpos,
                        start_qpos,
                        waypoints_dict,
                        speed_ratio,
                        speed_ratio,
                        left_qpos_seed=left_qpos_seed,
                        right_qpos_seed=right_qpos_seed,
                        cartesian_step=cartesian_step,
                        is_local=False
                    )
                    if res_dual_arm:
                        left_arm_qpos_array = np.vstack([
                            start_qpos[
                                self.supervisor_view.w1_params.left_arm_index],
                            left_arm_qpos_array
                        ])
                        right_arm_qpos_array = np.vstack([
                            start_qpos[
                                self.supervisor_view.w1_params.right_arm_index],
                            right_arm_qpos_array
                        ])
                if res_dual_arm.value < 0:
                    self.supervisor_view.logger.error(
                        "Plan to dual arm sync traj failed.")
                    return res_dual_arm, None, None, None

                left_ee_target_list = waypoints_dict.get("left_arm_ee")
                right_ee_target_list = waypoints_dict.get("right_arm_ee")
                if left_ee_target_list is not None and left_ee_target_list != []:
                    left_ee_target = left_ee_target_list[-1]
                    left_arm_ee_trigger_list = [
                        (left_arm_qpos_array.shape[0] + left_ee_target[0],
                         left_ee_target[1])
                    ]
                    left_arm_qpos_array = np.vstack([
                        left_arm_qpos_array,
                        np.tile(left_arm_qpos_array[-1, :],
                                (int(left_ee_target[0] + left_ee_target[2]), 1))
                    ])
                if right_ee_target_list is not None and right_ee_target_list != []:
                    right_ee_target = right_ee_target_list[-1]
                    right_arm_ee_trigger_list = [
                        (right_arm_qpos_array.shape[0] + right_ee_target[0],
                         right_ee_target[1])
                    ]
                    right_arm_qpos_array = np.vstack([
                        right_arm_qpos_array,
                        np.tile(
                            right_arm_qpos_array[-1, :],
                            (int(right_ee_target[0] + right_ee_target[2]), 1))
                    ])

        # align
        res_align, total_qpos_array = StaticUtilityW1.qpos_align(
            self.supervisor_view, torso_qpos_array, head_qpos_array,
            left_arm_qpos_array, right_arm_qpos_array)
        if not res_align:
            return StatusCode.FAIL, None, None, None

        if self.supervisor_view.is_traj_collision(
                qpos_array=total_qpos_array,
                left_arm_ee_trigger_list=left_arm_ee_trigger_list,
                right_arm_ee_trigger_list=right_arm_ee_trigger_list
        ) != StatusCode.FAIL:
            self.supervisor_view.logger.error(
                "Full body linear traj collision detected!")
            return StatusCode.COLLISION_FAIL, None, None, None

        return StatusCode.SUCCESS, total_qpos_array, left_arm_ee_trigger_list, right_arm_ee_trigger_list

    def move_linear_full_body(self,
                              waypoints_dict: dict,
                              left_qpos_seed: np.ndarray = None,
                              right_qpos_seed: np.ndarray = None,
                              cartesian_step: float = 0.005,
                              speed_ratio: float = 1.0,
                              is_local: bool = True,
                              is_sync: bool = False) -> StatusCode:
        """从当前位置规划各部件独立关节空间运动，arm执行笛卡尔空间运动
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

            cartesian_step (float, optional): cartesian sample step. Defaults to 0.005.

            speed_ratio (float, optional): 相对于默认关节速度的比率. Defaults to 1.0.

            is_local (bool, optional): w.r.t. arm base frame or chassis frame. Defaults to True.

        Returns:
            StatusCode
        """
        res, cur_qpos = self.supervisor_view.get_current_qpos()
        if res.value < 0:
            return res
        cur_left_arm_ee_qpos = None
        if self.supervisor_view._has_arm_ee_client("left_arm_ee", False):
            res, cur_left_arm_ee_qpos = self.supervisor_view.get_current_qpos(
                "left_arm_ee")
            if res.value < 0:
                return res
        cur_right_arm_ee_qpos = None
        if self.supervisor_view._has_arm_ee_client("right_arm_ee", False):
            res, cur_right_arm_ee_qpos = self.supervisor_view.get_current_qpos(
                "right_arm_ee")
            if res.value < 0:
                return res

        # plan traj
        (res, total_qpos_array, left_arm_ee_trigger_list,
         right_arm_ee_trigger_list) = self.plan_linear_full_body(
             cur_qpos, cur_left_arm_ee_qpos, cur_right_arm_ee_qpos,
             waypoints_dict, left_qpos_seed, right_qpos_seed, cartesian_step,
             speed_ratio, is_local, is_sync)

        # TODO: just for test, comment this
        self.total_qpos_array = total_qpos_array

        if res.value < 0:
            return res

        # # check collision
        # if self._check_traj_collision(total_qpos_array):
        #     return False

        # execute
        return self.supervisor_view.execute_joint_traj(
            total_qpos_array, left_arm_ee_trigger_list,
            right_arm_ee_trigger_list)

    def move_arm_linear(self,
                        name: str,
                        delta_trans_mm: list,
                        cartesian_step=0.005,
                        speed_ratio: float = 1.,
                        along_base_frame: bool = True) -> StatusCode:
        """机械臂从当前位置直线移动相对距离

        Args:
            name (str): ["left_arm", "right_arm"]
            delta_trans_mm (list): 直线相对运动距离
            cartesian_step (float, optional): 直线离散步长. Defaults to 0.005.
            speed_ratio (float, optional): 运动速率. Defaults to 1..
            along_base_frame (bool, optional): 参考chassis_base坐标系/tcp坐标系. Defaults to True.

        Returns:
            StatusCode: 
        """
        if not self.supervisor_view._check_arm_name(name):
            return StatusCode.NAME_MISMATCH

        res, T_armbase_ee = self.supervisor_view.get_current_xpos(name)
        if res.value < 0:
            return res

        delta_xpos = user_pose_to_xpos(delta_trans_mm, [0., 0., 0.])
        if along_base_frame:
            res, T_chassis_ee = self.supervisor_view.get_current_xpos(
                name, False)
            if res.value < 0:
                return res
            target_xpos = T_armbase_ee @ np.linalg.inv(T_chassis_ee) @ (
                delta_xpos @ T_chassis_ee)
        else:
            target_xpos = T_armbase_ee @ delta_xpos

        return self.move_linear_full_body(
            waypoints_dict={f"{name}": [xpos_to_user_pose(target_xpos)]},
            cartesian_step=cartesian_step,
            speed_ratio=speed_ratio)

    def rotate_arm_linear(self,
                          name: str,
                          delta_rot_deg: list,
                          speed_ratio: float = 1.,
                          along_base_frame: bool = True) -> bool:
        """机械臂从当前位置旋转相对角度

        Args:
            name (str): ["left_arm", "right_arm"]
            delta_rot_deg (list): 旋转相对运动距离
            speed_ratio (float, optional): 运动速率. Defaults to 1..
            along_base_frame (bool, optional): 参考chassis_base坐标系/tcp坐标系. Defaults to True.

        Returns:
            bool: 
        """
        if not self.supervisor_view._check_arm_name(name):
            return StatusCode.NAME_MISMATCH

        res, T_armbase_ee = self.supervisor_view.get_current_xpos(name)
        if res.value < 0:
            return res

        delta_xpos = user_pose_to_xpos([0., 0., 0.], delta_rot_deg)
        target_xpos = deepcopy(T_armbase_ee)

        if along_base_frame:
            res, T_chassis_ee = self.supervisor_view.get_current_xpos(
                name, False)
            if res.value < 0:
                return res
            target_xpos[:3,:3] = T_armbase_ee[:3,:3] \
                                @ T_chassis_ee[:3,:3].T \
                                @ delta_xpos[:3,:3] \
                                @ T_chassis_ee[:3,:3]
        else:
            target_xpos[:3, :3] = T_armbase_ee[:3, :3] @ delta_xpos[:3, :3]

        arm_kine = self.supervisor_view._get_arm_kine(name)
        if arm_kine is None:
            return StatusCode.NAME_MISMATCH

        path = PathR3xSO3Bezier(np.array([T_armbase_ee, target_xpos]))
        if not path.calc_path():
            return StatusCode.PLAN_PATH_FAIL
        xpos_sample_array = path.get_sample_by_step(0.005)
        if xpos_sample_array is None:
            return StatusCode.PLAN_PATH_FAIL

        # solve ik
        qpos_list = []
        res, cur_qpos = self.supervisor_view.get_current_qpos()
        if res.value < 0:
            return res
        cur_qseed = cur_qpos[self.supervisor_view.w1_params.index_infos[name]
                             ["index"]]
        for cur_xpos in xpos_sample_array:
            res_ik, qpos = arm_kine.get_nearest_ik(xpos=cur_xpos,
                                                   qpos_seed=cur_qseed)
            if not res_ik:
                self.logger.error(
                    f"{name} solve ik failed, cur_qseed(deg): {np.rad2deg(cur_qseed)}."
                )
                return StatusCode.IK_FAIL
            if not np.allclose(qpos, cur_qseed, atol=np.deg2rad(90)):
                self.logger.error(f"{name} qpos traj is not continues\n" + \
                                  f"cur_qpos(deg): {np.rad2deg(cur_qpos)}\n" + \
                                  f"cur_qseed(deg): {np.rad2deg(cur_qseed)}.")
                return StatusCode.IK_FAIL
            qpos_list.append(qpos)
            cur_qseed = qpos

        speed_ratio_list = [speed_ratio] * len(qpos_list)
        default_qvel = self.supervisor_view.w1_params.default_qvel[
            self.supervisor_view.w1_params.index_infos[name]["index"]]
        min_qvel = self.supervisor_view.w1_params.min_qvel[
            self.supervisor_view.w1_params.index_infos[name]["index"]]
        max_qvel = self.supervisor_view.w1_params.max_qvel[
            self.supervisor_view.w1_params.index_infos[name]["index"]]
        qvel_limits_array = np.clip(np.outer(speed_ratio_list, default_qvel),
                                    min_qvel, max_qvel)

        res_plan, arm_qpos_array = StaticUtilityW1.plan_joint_traj(
            self.supervisor_view, name, np.array(qpos_list), False,
            qvel_limits_array)
        if res_plan.value < 0:
            return res_plan, None

        # align
        if name == "left_arm":
            res_align, total_qpos_array = StaticUtilityW1.qpos_align(
                self.supervisor_view,
                cur_qpos[self.supervisor_view.w1_params.torso_index],
                cur_qpos[self.supervisor_view.w1_params.head_index],
                arm_qpos_array,
                cur_qpos[self.supervisor_view.w1_params.right_arm_index])
        else:
            res_align, total_qpos_array = StaticUtilityW1.qpos_align(
                self.supervisor_view,
                cur_qpos[self.supervisor_view.w1_params.torso_index],
                cur_qpos[self.supervisor_view.w1_params.head_index],
                cur_qpos[self.supervisor_view.w1_params.left_arm_index],
                arm_qpos_array)
        if not res_align:
            return StatusCode.FAIL

        return self.supervisor_view.execute_joint_traj(total_qpos_array)

    # TODO: optimize this function to reduce code redundancy with move_linear_full_body
    def plan_linear_dual_arm_sync(self,
                                  task_dict: dict,
                                  speed_ratio: float = 1.0,
                                  is_local: bool = True):
        arml_kine, armr_kine = self.supervisor_view._get_arm_kine(
            "left_arm"), self.supervisor_view._get_arm_kine("right_arm")

        res, cur_qpos = self.supervisor_view.get_current_qpos()
        cur_head_qpos = cur_qpos[self.real_head_idx]
        cur_torso_qpos = cur_qpos[self.real_torso_idx]
        cur_arml_qpos = cur_qpos[self.real_arml_idx]
        cur_armr_qpos = cur_qpos[self.real_armr_idx]
        res, cur_arml_pose = arml_kine.get_fk(cur_arml_qpos)
        res, cur_armr_pose = armr_kine.get_fk(cur_armr_qpos)

        cur_arml_base_tf, cur_armr_base_tf = self.supervisor_view.pin_kine_solver.get_arm_base_tf(
            real_qpos=cur_qpos)
        if not is_local:
            cur_arml_pose = cur_arml_base_tf @ cur_arml_pose
            cur_armr_pose = cur_armr_base_tf @ cur_armr_pose

        self.supervisor_view.cartesian_planner.set_left_right_relationship(
            left_tcp_pose=cur_arml_pose,
            right_tcp_pose=cur_armr_pose,
            is_local=is_local)

        arml_target_pose_list, armr_target_pose_list = task_dict.get(
            "left_arm"), task_dict.get("right_arm")
        arml_speed_ratio_list, armr_speed_ratio_list = task_dict.get(
            "left_arm_speed_ratio"), task_dict.get("right_arm_speed_ratio")
        if (arml_target_pose_list is None and armr_target_pose_list is None) \
            or (arml_target_pose_list == [] and armr_target_pose_list == []):
            self.logger.error(
                "Both left arm and right arm in task_dict are None or empty.")
            return StatusCode.INPUT_EMPTY, None
        if (arml_target_pose_list is not None and arml_target_pose_list != []) \
            and (armr_target_pose_list is not None and armr_target_pose_list != []):
            self.logger.error(
                "Only one arm task is allowed in move_linear_dual_arm_sync.")
            return StatusCode.INPUT_INVALID, None

        left_default_qvel = self.w1_params.default_qvel[
            self.w1_params.index_infos["left_arm"]["index"]]
        right_default_qvel = self.w1_params.default_qvel[
            self.w1_params.index_infos["right_arm"]["index"]]
        full_body_default_qvel = self.w1_params.default_qvel
        left_min_qvel = self.w1_params.min_qvel[
            self.w1_params.index_infos["left_arm"]["index"]]
        right_min_qvel = self.w1_params.min_qvel[
            self.w1_params.index_infos["right_arm"]["index"]]
        full_body_min_qvel = self.w1_params.min_qvel
        left_max_qvel = self.w1_params.max_qvel[
            self.w1_params.index_infos["left_arm"]["index"]]
        right_max_qvel = self.w1_params.max_qvel[
            self.w1_params.index_infos["right_arm"]["index"]]
        full_body_max_qvel = self.w1_params.max_qvel

        target_pose_list = []
        target_speed_ratio_list = []
        target_arm_name = None
        if arml_target_pose_list is not None and arml_target_pose_list != []:
            for user_pose in arml_target_pose_list:
                target_pose_list.append(
                    user_pose_to_xpos(user_pose[:3], user_pose[3:]))
            target_arm_name = "left_arm"
            target_pose_list.insert(0, cur_arml_pose)

            if is_local:
                target_speed_ratio_list = arml_speed_ratio_list
                target_qvel_limits_array = np.clip(
                    np.outer(target_speed_ratio_list, left_default_qvel),
                    left_min_qvel, left_max_qvel)
                target_qvel_limits_array_ = np.vstack([
                    self.w1_params.default_qvel[
                        self.w1_params.index_infos["left_arm"]["index"]],
                    target_qvel_limits_array
                ])
            else:
                target_speed_ratio_list = arml_speed_ratio_list
                target_qvel_limits_array = np.clip(
                    np.outer(target_speed_ratio_list, full_body_default_qvel),
                    full_body_min_qvel, full_body_max_qvel)
                target_qvel_limits_array_ = np.vstack(
                    [self.w1_params.default_qvel, target_qvel_limits_array])
        else:
            for user_pose in armr_target_pose_list:
                target_pose_list.append(
                    user_pose_to_xpos(user_pose[:3], user_pose[3:]))
            target_arm_name = "right_arm"
            target_pose_list.insert(0, cur_armr_pose)

            if is_local:
                target_speed_ratio_list = armr_speed_ratio_list
                target_qvel_limits_array = np.clip(
                    np.outer(target_speed_ratio_list, right_default_qvel),
                    right_min_qvel, right_max_qvel)
                target_qvel_limits_array_ = np.vstack([
                    self.w1_params.default_qvel[
                        self.w1_params.index_infos["right_arm"]["index"]],
                    target_qvel_limits_array
                ])
            else:
                target_speed_ratio_list = armr_speed_ratio_list
                target_qvel_limits_array = np.clip(
                    np.outer(target_speed_ratio_list, full_body_default_qvel),
                    full_body_min_qvel, full_body_max_qvel)
                target_qvel_limits_array_ = np.vstack(
                    [self.w1_params.default_qvel, target_qvel_limits_array])

        res, left_pose_array, right_pose_array = self.supervisor_view.cartesian_planner.generate_line_traj_from_tcp_poses(
            name=target_arm_name,
            waypoint_list=target_pose_list,
            cartesian_step=0.005,
            is_local=is_local)

        if target_arm_name == "left_arm":
            target_pose_array = left_pose_array
        else:
            target_pose_array = right_pose_array

        # velocity align
        prefix_dist_list = [0.]
        for idx in range(len(target_pose_list) - 1):
            dist = np.linalg.norm(target_pose_list[idx + 1][:3, 3] -
                                  target_pose_list[idx][:3, 3])
            prefix_dist_list.append(prefix_dist_list[-1] + dist)

        idx = 0
        qvel_limits_list_ = [target_qvel_limits_array_[idx]]
        cur_xpos = target_pose_array[0]
        cur_dist = 0.
        for i in range(1, target_pose_array.shape[0] - 1):  # bug here
            xpos = target_pose_array[i]
            cur_dist += np.linalg.norm(xpos[:3, 3] - cur_xpos[:3, 3])
            if cur_dist >= prefix_dist_list[idx] + 1e-6:
                idx += 1
            qvel_limits_list_.append(target_qvel_limits_array_[idx])
            cur_xpos = xpos
        qvel_limits_list_.append(target_qvel_limits_array_[-1])
        qvel_limits_array_ = np.array(qvel_limits_list_)

        if is_local:
            left_res, left_joint_traj = self.supervisor_view.cartesian_joint_planner.plan_single_arm(
                name="left_arm",
                start_qpos=cur_arml_qpos,
                end_effector_traj=left_pose_array)
            right_res, right_joint_traj = self.supervisor_view.cartesian_joint_planner.plan_single_arm(
                name="right_arm",
                start_qpos=cur_armr_qpos,
                end_effector_traj=right_pose_array)
            if not left_res:
                self.logger.error(
                    "Left arm joint traj planning failed in plan_linear_dual_arm_sync."
                )
                return StatusCode.PLAN_TRAJ_FAIL, None
            if not right_res:
                self.logger.error(
                    "Right arm joint traj planning failed in plan_linear_dual_arm_sync."
                )
                return StatusCode.PLAN_TRAJ_FAIL, None

            res, left_joint_traj_, right_joint_traj_ = self.supervisor_view.cartesian_joint_planner.generate_dense_dual_arm_joint_traj(
                left_joint_traj=left_joint_traj,
                right_joint_traj=right_joint_traj,
                qvel_limits_array=qvel_limits_array_,
                current_left_qpos=cur_arml_qpos,
                current_right_qpos=cur_armr_qpos)

            if not res:
                self.logger.error(
                    "Generate dense joint traj failed in plan_linear_dual_arm_sync."
                )
                return StatusCode.PLAN_TRAJ_FAIL, None

            res, move_arml_armr_traj = StaticUtilityW1.qpos_align(
                self.supervisor_view, cur_torso_qpos, cur_head_qpos,
                left_joint_traj_, right_joint_traj_)
            if not res:
                self.logger.error(
                    "Align dual arm traj failed in plan_linear_dual_arm_sync.")
                return StatusCode.FAIL, None

            if self._is_traj_collision(
                    qpos_array=move_arml_armr_traj,
                    left_arm_ee_trigger_list=None,
                    right_arm_ee_trigger_list=None) != StatusCode.FAIL:
                self.logger.error(
                    "Collision detected in plan_linear_dual_arm_sync.")
                return StatusCode.COLLISION_FAIL, None

            return StatusCode.SUCCESS, move_arml_armr_traj

        else:
            res, full_body_joint_traj = self.supervisor_view.cartesian_joint_planner.plan_whole_body(
                start_qpos=cur_qpos,
                left_end_effector_traj=left_pose_array,
                right_end_effector_traj=right_pose_array)
            if not res:
                self.logger.error(
                    "Full body joint traj planning failed in plan_linear_dual_arm_sync."
                )
                return StatusCode.PLAN_TRAJ_FAIL, None

            res, full_body_joint_traj_ = self.supervisor_view.cartesian_joint_planner.generate_dense_whole_body_joint_traj(
                whole_body_joint_traj=full_body_joint_traj,
                qvel_limits_array=qvel_limits_array_,
                current_qpos=cur_qpos)

            if not res:
                self.logger.error(
                    "Generate dense whole body joint traj failed in plan_linear_dual_arm_sync."
                )
                return StatusCode.PLAN_TRAJ_FAIL, None

            move_arml_armr_traj = full_body_joint_traj_

            if self._is_traj_collision(
                    qpos_array=move_arml_armr_traj,
                    left_arm_ee_trigger_list=None,
                    right_arm_ee_trigger_list=None) != StatusCode.FAIL:
                self.logger.error(
                    "Collision detected in plan_linear_dual_arm_sync.")
                return StatusCode.COLLISION_FAIL, None

            return StatusCode.SUCCESS, move_arml_armr_traj

    def move_linear_dual_arm_sync(self,
                                  task_dict: dict,
                                  speed_ratio: float = 1.0,
                                  is_local: bool = True):

        res, full_body_traj = self.plan_linear_dual_arm_sync(
            task_dict=task_dict, speed_ratio=speed_ratio, is_local=is_local)

        if res.value < 0:
            self.logger.error("Plan linear dual arm sync traj failed.")
            return res

        return self.supervisor_view.execute_joint_traj(
            qpos_array=full_body_traj,
            left_arm_ee_trigger_list=None,
            right_arm_ee_trigger_list=None), full_body_traj

    def plan_cartesian_traj(
            self,
            name: str,
            xpos_list: list[np.ndarray],
            qpos_seed: np.ndarray,
            qvel_limits_array: np.ndarray = None,
            cartesian_step: float = 0.005) -> tuple[StatusCode, np.ndarray]:
        """对机械臂进行笛卡尔轨迹规划，可指定路径点的各关节的速度约束；速度表示从上一点到当前点的最大关节速度

        Args:
            name (str): ["left_arm", "right_arm"]
            xpos_list (list[np.ndarray]): 笛卡尔路径点
            qpos_seed (np.ndarray): 起始参考关节seed
            qvel_limits_array (np.ndarray, optional): 路径点关节速度约束，省略则按默认关节速度
            cartesian_step (float, optional): 笛卡尔轨迹采样步长 默认0.005m

        Returns:
            tuple[StatusCode, np.ndarray]
        """
        return StaticUtilityW1.plan_cartesian_traj(self.supervisor_view, name,
                                                   xpos_list, qpos_seed,
                                                   qvel_limits_array,
                                                   cartesian_step)
