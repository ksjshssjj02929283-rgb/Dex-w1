# ----------------------------------------------------------------------------
# Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import numpy as np
from ..utils.unit_conversion import user_pose_to_xpos, xpos_to_user_pose
from ..trajectory_planner.toppra_trajectory_planner import ToppraTrajectoryPlanner

from ..mgr_util.supervisor_view import SupervisorViewW1
from ..mgr_util.supervisor_utils import StaticUtilityW1
from ...common import get_logger, time_consuming

from ..path_planner.blend_path import PathType, PathRnBezier, PathR3xSO3Bezier, weighted_norm

from dexe_rp import DexforceW1Kinematics, SRSConfig

class DualArmMovementBaseW1(object):

    def __init__(self, w1_params, supervisor, kine_solver, pin_kin_solver, collision_checker, time_step: float = 0.01):
        self.w1_params = w1_params
        self.supervisor_view = SupervisorViewW1(supervisor=supervisor, w1_params=w1_params, kine_solver=kine_solver, pin_kin_solver=pin_kin_solver, collision_checker=collision_checker, time_step=time_step)
        self.logger = get_logger()
    
    def _plan_to_waypoint(self,
                          name: str,
                          cur_qpos: np.ndarray,
                          waypoints_dict: dict,
                          speed_ratio: float,
                          is_cartesian: bool = False,
                          qpos_seed: np.ndarray = None,
                          cartesian_step: float = None,
                          is_local: bool = True) -> tuple[bool, np.ndarray]:
        if not is_local:
            qpos_array = cur_qpos[self.w1_params.index_infos[name]["index"]]
        else:
            qpos_array = cur_qpos
        (int_res, pos_raw_array, qvel_limits_array,
         _) = StaticUtilityW1.parse_waypoints_from_dict(self.supervisor_view, name, waypoints_dict, speed_ratio,
                                              is_cartesian)
        if int_res == -1:  # 输入不合法
            return False, None
        elif int_res == 0:  # 输入为空
            return True, qpos_array
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
                res_fk, T_chassis_armbase = self.supervisor_view.get_xpos(f"{name}_base",
                                                          ref_qpos,
                                                          is_local=False)
                if not res_fk:
                    self.logger.error(f"{name}_base solve fk failed!")
                    return False, None
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
            res_fk, cur_xpos = self.supervisor_view._get_arm_kine(name).get_fk(cur_arm_qpos)
            if not res_fk:
                self.logger.error(f"{name} solve fk failed!")
                return False, None
            xpos_raw_list = [
                user_pose_to_xpos(pose[:3].tolist(), pose[3:].tolist())
                for pose in pos_raw_array
            ]
            xpos_raw_list_ = [cur_xpos] + xpos_raw_list
            res_plan, qpos_array = self.plan_cartesian_traj(
                name, xpos_raw_list_, qpos_seed, qvel_limits_array_,
                cartesian_step)
            if not res_plan:
                return False, None
        else:
            qpos_raw_array_ = np.vstack([cur_qpos, pos_raw_array])
            res_plan, qpos_array = StaticUtilityW1.plan_joint_traj(self.supervisor_view, name, qpos_raw_array_,
                                                        qvel_limits_array_)
            if not res_plan:
                return False, None

        return True, qpos_array
    
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
            is_local: bool = True) -> tuple[bool, np.ndarray, list]:
        if not is_local:
            qpos_array = cur_qpos[self.w1_params.index_infos[name]["index"]]
        else:
            qpos_array = cur_qpos

        (int_res, pos_raw_array, qvel_limits_array,
         arm_ee_state_list) = self._parse_waypoints_from_dict(
             name, waypoints_dict, speed_ratio, is_cartesian)
        if int_res == -1:
            return False, None, None
        elif int_res == 0:
            return True, qpos_array, None

        # without ee motion
        if arm_ee_state_list is None:
            res, qpos_array = self._plan_to_waypoint(name, cur_qpos,
                                                     waypoints_dict,
                                                     speed_ratio, is_cartesian,
                                                     qpos_seed, cartesian_step,
                                                     is_local)
            if not res:
                return False, None, None
            return True, qpos_array, None

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
                res_fk, T_chassis_armbase = self.supervisor_view.get_xpos(f"{name}_base",
                                                          ref_qpos,
                                                          is_local=False)
                if not res_fk:
                    self.logger.error(f"{name}_base solve fk failed!")
                    return False, None, None
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

            res_fk, cur_xpos = self.supervisor_view._get_arm_kine(name).get_fk(cur_arm_qpos)
            if not res_fk:
                self.logger.error(f"{name} solve fk failed!")
                return False, None, None
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
        res, segment_list = StaticUtilityW1.plan_arm_ee_trigger(self.supervisor_view, arm_ee_state_list_)
        if not res:
            return False, None, None

        total_count = 0
        total_qpos_array = None
        if is_cartesian:
            cur_qpos_seed = qpos_seed

        for start_idx, end_idx in segment_list:
            seg_pos_list = pos_raw_array_[start_idx:end_idx + 1]
            seg_qvel_limits_list = qvel_limits_array_[start_idx:end_idx + 1]
            if is_cartesian:
                seg_xpos_list = [
                    user_pose_to_xpos(pose[:3].tolist(), pose[3:].tolist())
                    for pose in seg_pos_list
                ]
                res_plan, seg_qpos_samples_array = self.plan_cartesian_traj(
                    name, seg_xpos_list, cur_qpos_seed, seg_qvel_limits_list,
                    cartesian_step)
            else:
                res_plan, seg_qpos_samples_array = StaticUtilityW1.plan_joint_traj(
                    self.supervisor_view,
                    name, seg_pos_list, seg_qvel_limits_list)
            if not res_plan:
                return False, None, None

            # pre delay
            pre_delay_time = arm_ee_state_list_[end_idx][0]
            if pre_delay_time >= self.TIME_STEP:
                pre_delay_count = max(
                    1, int(np.ceil(pre_delay_time / self.TIME_STEP)))
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
            if post_delay_time >= self.TIME_STEP:
                post_delay_count = max(
                    1, int(np.ceil(post_delay_time / self.TIME_STEP)))
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

        return True, total_qpos_array, arm_ee_trigger_list

    def plan_cartesian_traj(
            self,
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
            tuple[StatusCode, np.ndarray]
        """
        arm_kine = self.supervisor_view._get_arm_kine(name)
        if arm_kine is None:
            return StatusCode.NAME_MISMATCH, None

        # cartesian interpolation
        path = PathR3xSO3Bezier(np.array(xpos_list))
        if not path.calc_path():
            return StatusCode.PLAN_PATH_FAIL, None
        if len(xpos_list) == 2 and path.get_length() < cartesian_step:
            xpos_sample_array = np.array(xpos_list)
        else:
            xpos_sample_array = path.get_sample_by_step(cartesian_step)
            if xpos_sample_array is None:
                return StatusCode.PLAN_PATH_FAIL, None

        prefix_dist_list = [0.]
        for idx in range(len(xpos_list) - 1):
            dist = np.linalg.norm(xpos_list[idx + 1][:3, 3] -
                                  xpos_list[idx][:3, 3])
            prefix_dist_list.append(prefix_dist_list[-1] + dist)

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
                dist = weighted_norm(xpos[:3, 3] - cur_xpos[:3, 3])
                cur_dist += dist
                if cur_dist >= prefix_dist_list[idx] + 1e-6:
                    idx += 1
                qvel_limits_list_.append(qvel_limits_array[idx])
                cur_xpos = xpos
            qvel_limits_list_.append(qvel_limits_array[-1])
        qvel_limits_array_ = np.array(qvel_limits_list_)

        # solve ik
        qpos_list = []
        cur_qseed = qpos_seed
        for i, cur_xpos in enumerate(xpos_sample_array):
            use_nearest = (i != 0)

            if not use_nearest:
                res, ref_psi = arm_kine.calc_arm_angle(cur_qseed)
                if res:
                    srs_cfg = SRSConfig(cur_qseed, ref_psi)
                    res_ik, cur_qpos = arm_kine.get_ik(cur_xpos, srs_cfg)
                    if not res_ik:
                        use_nearest = True
                else:
                    use_nearest = True

            if use_nearest:
                res_ik, cur_qpos = arm_kine.get_nearest_ik(xpos=cur_xpos,
                                                           qpos_seed=cur_qseed)

            if not res_ik:
                self.logger.error(
                    f"{name} solve ik failed, cur_qseed(deg): {np.rad2deg(cur_qseed)}."
                )
                return StatusCode.IK_FAIL, None
            if not np.allclose(cur_qpos, cur_qseed, atol=np.deg2rad(90)):
                self.logger.error(f"{name} qpos traj is not continues\n" + \
                                  f"cur_qpos(deg): {np.rad2deg(cur_qpos)}\n" + \
                                  f"cur_qseed(deg): {np.rad2deg(cur_qseed)}.")
                return StatusCode.IK_FAIL, None
            qpos_list.append(cur_qpos)
            cur_qseed = cur_qpos

        return StaticUtilityW1.plan_joint_traj(self.supervisor_view, name, np.array(qpos_list), False,
                                    qvel_limits_array_)

