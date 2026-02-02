# ----------------------------------------------------------------------------
# Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import time
import numpy as np
from ..utils.unit_conversion import user_pose_to_xpos, xpos_to_user_pose
from ..trajectory_planner.toppra_trajectory_planner import ToppraTrajectoryPlanner
from ...common import get_logger, StatusCode
from .supervisor_view import SupervisorViewW1

from ..path_planner.blend_path import PathType, PathRnBezier, PathR3xSO3Bezier, weighted_norm
from dexe_rp import DexforceW1Kinematics, SRSConfig


class StaticUtilityW1(object):

    @staticmethod
    def parse_waypoints_from_dict(
        supervisor: SupervisorViewW1,
        name: str,
        waypoints_dict: dict,
        speed_ratio: float,
        is_cartesian: bool = False
    ) -> tuple[StatusCode, np.ndarray, np.ndarray, list]:
        """解析并检查输入  
            StatusCode: """
        pos_list = waypoints_dict.get(name)
        if not pos_list:  # None or []
            supervisor.logger.debug(f"No waypoints in {name}.")
            return StatusCode.INPUT_EMPTY, None, None, None

        if not all(isinstance(pos, list) for pos in pos_list):
            supervisor.logger.error(f"{pos_list} not a 2d-list.")
            return StatusCode.INPUT_INVALID, None, None, None
        if not is_cartesian and not supervisor._check_qpos_dimension(
                name, pos_list):
            return StatusCode.DIM_MISMATCH, None, None, None
        if is_cartesian and not all(len(pos) == 6 for pos in pos_list):
            supervisor.logger.error(f"Dimension of {pos_list[0]} != 6.")
            return StatusCode.DIM_MISMATCH, None, None, None

        speed_ratio_list = waypoints_dict.get(f"{name}_speed_ratio")
        if not speed_ratio_list:  # None or []
            speed_ratio_list = [speed_ratio] * len(pos_list)
        elif len(speed_ratio_list) != len(pos_list):
            self.logger.error(
                f"{name} qpos and speed_ratio dimension mismatch.")
            return StatusCode.DIM_MISMATCH, None, None, None
        # 将None元素替换为global_speed_ratio
        speed_ratio_list = [
            sr if sr is not None else speed_ratio for sr in speed_ratio_list
        ]

        default_qvel = supervisor.w1_params.default_qvel[
            supervisor.w1_params.index_infos[name]["index"]]
        min_qvel = supervisor.w1_params.min_qvel[
            supervisor.w1_params.index_infos[name]["index"]]
        max_qvel = supervisor.w1_params.max_qvel[
            supervisor.w1_params.index_infos[name]["index"]]
        qvel_limits_array = np.clip(np.outer(speed_ratio_list, default_qvel),
                                    min_qvel, max_qvel)

        # 额外读取ee的输入
        arm_ee_state_list = None
        if name in supervisor.w1_params.arm_names:
            arm_ee_state_list = waypoints_dict.get(f"{name}_ee")
            if not arm_ee_state_list:  # None or []
                arm_ee_state_list = None
            elif len(arm_ee_state_list) != len(pos_list):
                supervisor.logger.error(
                    f"{name} qpos and arm_ee_state dimension mismatch.")
                return StatusCode.DIM_MISMATCH, None, None, None
            elif not supervisor._check_arm_ee_states(arm_ee_state_list):
                return StatusCode.INPUT_INVALID, None, None, None

        return StatusCode.SUCCESS, np.array(
            pos_list), qvel_limits_array, arm_ee_state_list

    @staticmethod
    def plan_joint_traj(
            supervisor: SupervisorViewW1,
            name: str,
            qpos_array: np.ndarray,
            is_sparse: bool,
            qvel_limits_array: np.ndarray = None
    ) -> tuple[StatusCode, np.ndarray]:
        """对各部件进行关节轨迹规划，可指定路径点的各关节的速度约束;速度表示从上一点到当前点的最大关节速度

        Args:
            supervisor (SupervisorViewW1): 管理器视图
            name (str): ["full_body", "torso", "head", "left_arm", "right_arm"]
            qpos_array (np.ndarray): 关节路径点  
            is_sparse (bool): 是否为稀疏路径点
            qvel_limits_array (np.ndarray, optional): 路径点关节速度约束，省略则按默认关节速度

        Returns:
            tuple[StatusCode, np.ndarray]
        """
        if not supervisor._check_name(name):
            return StatusCode.NAME_MISMATCH, None
        if not supervisor._check_qpos_limits(name, qpos_array):
            return StatusCode.EXCEED_LIMIT, None

        step = weighted_norm(np.deg2rad([1.] * qpos_array[0].shape[0]))
        if qpos_array.shape[0] == 2 and weighted_norm(qpos_array[1] -
                                                      qpos_array[0]) < step:
            return StatusCode.SUCCESS, qpos_array

        if is_sparse:
            path = PathRnBezier(qpos_array)
            if not path.calc_path():
                return StatusCode.PLAN_PATH_FAIL, None
            qpos_sample_array = path.get_sample_by_step(step)
            if qpos_sample_array is None:
                return StatusCode.PLAN_PATH_FAIL, None

            # vel align
            prefix_dist_list = [0.]
            for idx in range(len(qpos_array) - 1):
                dist = weighted_norm(qpos_array[idx + 1] - qpos_array[idx])
                prefix_dist_list.append(prefix_dist_list[-1] + dist)

            idx = 0
            qvel_limits_list = [qvel_limits_array[0]]
            cur_qpos = qpos_sample_array[0]
            cur_dist = 0.
            for qpos in qpos_sample_array[1:-1]:
                cur_dist += weighted_norm(qpos - cur_qpos)
                if cur_dist >= prefix_dist_list[idx]:
                    idx += 1
                qvel_limits_list.append(qvel_limits_array[idx])
                cur_qpos = qpos
            qvel_limits_list.append(qvel_limits_array[-1])

            qpos_array = qpos_sample_array
            qvel_limits_array = np.array(qvel_limits_list)

        dofs = supervisor.w1_params.index_infos[name]["dofs"]
        index = supervisor.w1_params.index_infos[name]["index"]
        vel_limits = supervisor.w1_params.max_qvel[index]
        acc_limits = supervisor.w1_params.max_qacc[index]
        traj_planner = ToppraTrajectoryPlanner(dofs, vel_limits, acc_limits)
        res, pos_array, _, _, _ = traj_planner.plan(qpos_array,
                                                    qvel_limits_array,
                                                    supervisor.TIME_STEP)
        if not res:
            return StatusCode.PLAN_TRAJ_FAIL, None
        return StatusCode.SUCCESS, pos_array

    @staticmethod
    def plan_cartesian_traj(
            supervisor: SupervisorViewW1,
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
        arm_kine = supervisor._get_arm_kine(name)
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
                supervisor.logger.error(
                    f"{name} solve ik failed, cur_qseed(deg): {np.rad2deg(cur_qseed)}."
                )
                return StatusCode.IK_FAIL, None
            if not np.allclose(cur_qpos, cur_qseed, atol=np.deg2rad(90)):
                supervisor.logger.error(f"{name} qpos traj is not continues\n" + \
                                  f"cur_qpos(deg): {np.rad2deg(cur_qpos)}\n" + \
                                  f"cur_qseed(deg): {np.rad2deg(cur_qseed)}.")
                return StatusCode.IK_FAIL, None
            qpos_list.append(cur_qpos)
            cur_qseed = cur_qpos

        return StaticUtilityW1.plan_joint_traj(supervisor, name,
                                               np.array(qpos_list), False,
                                               qvel_limits_array_)

    @staticmethod
    def plan_arm_ee_trigger(supervisor: SupervisorViewW1,
                            arm_ee_state_list: list) -> tuple[StatusCode, list]:
        # TODO: 存在延时的才分段，否则不分段且全部waypoint ee_qpos都到点触发？
        if not supervisor._check_arm_ee_states(arm_ee_state_list):
            return StatusCode.INPUT_INVALID, None
        ee_qpos_array = np.array(
            [ee_qpos for _, ee_qpos, _ in arm_ee_state_list])
        if not supervisor._check_qpos_limits("left_arm_ee", ee_qpos_array):
            return StatusCode.EXCEED_LIMIT, None

        segment_list = []
        start_idx = 0
        for i in range(1, len(arm_ee_state_list)):
            pre_qpos = arm_ee_state_list[i - 1][1]
            pre_delay, qpos, post_delay = arm_ee_state_list[i]
            if pre_delay != 0. or post_delay != 0. or not np.allclose(
                    qpos, pre_qpos, atol=np.deg2rad(0.5)):
                segment_list.append([start_idx, i])
                start_idx = i

        if start_idx < len(arm_ee_state_list) - 1:
            segment_list.append([start_idx, len(arm_ee_state_list) - 1])
        supervisor.logger.debug(f"plan_arm_ee_trigger result: {segment_list}.")
        return StatusCode.SUCCESS, segment_list

    @staticmethod
    def qpos_align(supervisor: SupervisorViewW1, torso_qpos_array: np.ndarray,
                   head_qpos_array: np.ndarray, left_arm_qpos_array: np.ndarray,
                   right_arm_qpos_array: np.ndarray) -> tuple[bool, np.ndarray]:
        """轨迹对齐，按先运动后等待的模式进行补齐"""
        qpos_array_list = [
            np.atleast_2d(torso_qpos_array),
            np.atleast_2d(head_qpos_array),
            np.atleast_2d(left_arm_qpos_array),
            np.atleast_2d(right_arm_qpos_array)
        ]
        if all(q.size == 0 for q in qpos_array_list):
            supervisor.logger.error("No qpos to be aligned")
            return False, None

        max_count = max(q.shape[0] for q in qpos_array_list)

        temp_arrays = []
        for qpos_array in qpos_array_list:
            pad_count = max_count - qpos_array.shape[0]
            if pad_count > 0:
                padded = np.vstack(
                    [qpos_array,
                     np.tile(qpos_array[-1], (pad_count, 1))])
            else:
                padded = qpos_array
            temp_arrays.append(padded)

        return True, np.hstack(temp_arrays)

    @staticmethod
    def cal_path_seg_interpolate_num(waypoint_list: list[np.ndarray],
                                     cartesian_step: float = 0.005):
        interpolate_num_list = []
        for idx in range(len(waypoint_list) - 1):
            dist = np.linalg.norm(waypoint_list[idx + 1][:3, 3] -
                                  waypoint_list[idx][:3, 3])
            num_sample = max(2, int(np.ceil(dist / cartesian_step)))
            interpolate_num_list.append(num_sample)
        return interpolate_num_list
