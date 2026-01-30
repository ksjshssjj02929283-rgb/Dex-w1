import numpy as np
from ..utils.unit_conversion import user_pose_to_xpos, xpos_to_user_pose
from ..trajectory_planner.toppra_trajectory_planner import ToppraTrajectoryPlanner

from ..mgr_util.supervisor_view import SupervisorViewW1
from ..mgr_util.supervisor_utils import StaticUtilityW1
from .move_dual_arm import DualArmMovementBaseW1
from ...common import get_logger, time_consuming, StatusCode

from ..path_planner.blend_path import PathType, PathRnBezier, PathR3xSO3Bezier, weighted_norm

from dexe_rp import DexforceW1Kinematics, SRSConfig

class DualArmMoveLineW1(DualArmMovementBaseW1):

    def __init__(self, w1_params, supervisor, kine_solver, pin_kin_solver, collision_checker, time_step: float = 0.01):
        super().__init__(w1_params, supervisor, kine_solver, pin_kin_solver, collision_checker, time_step=time_step)

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
         _) = StaticUtilityW1.parse_waypoints_from_dict(self.supervisor_view, name, waypoints_dict, speed_ratio,
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
                res_fk, T_chassis_armbase = self.supervisor_view.get_xpos(f"{name}_base",
                                                          ref_qpos,
                                                          is_local=False)
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
            res_fk, cur_xpos = self.supervisor_view._get_arm_kine(name).get_fk(cur_arm_qpos)
            if not res_fk:
                self.logger.error(f"{name} solve fk failed!")
                return StatusCode.FK_FAIL, None
            xpos_raw_list = [
                user_pose_to_xpos(pose[:3].tolist(), pose[3:].tolist())
                for pose in pos_raw_array
            ]
            xpos_raw_list_ = [cur_xpos] + xpos_raw_list
            res_plan, qpos_array = self.plan_cartesian_traj(
                name, xpos_raw_list_, qpos_seed, qvel_limits_array_,
                cartesian_step)
            if res_plan.value < 0:
                return res_plan, None
        else:
            qpos_raw_array_ = np.vstack([cur_qpos, pos_raw_array])
            res_plan, qpos_array = self.plan_joint_traj(name, qpos_raw_array_,
                                                        is_sparse,
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
             self.supervisor_view, name, waypoints_dict, speed_ratio, is_cartesian)
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
                res_fk, T_chassis_armbase = self.supervisor_view.get_xpos(f"{name}_base",
                                                          ref_qpos,
                                                          is_local=False)
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

            res_fk, cur_xpos = self.supervisor_view._get_arm_kine(name).get_fk(cur_arm_qpos)
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
        res, segment_list = StaticUtilityW1.plan_arm_ee_trigger(self.supervisor_view, arm_ee_state_list_)
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
                res_plan, seg_qpos_samples_array = self.plan_cartesian_traj(
                    name, seg_xpos_list, cur_qpos_seed, seg_qvel_limits_array,
                    cartesian_step)
            else:
                res_plan, seg_qpos_samples_array = self.plan_joint_traj(
                    name, seg_pos_array, is_sparse, seg_qvel_limits_array)
            if res_plan.value < 0:
                return res_plan, None, None

            # pre delay
            pre_delay_time = arm_ee_state_list_[end_idx][0]
            if pre_delay_time >= self.supervisor_view.TIME_STEP:
                pre_delay_count = max(
                    1, int(np.ceil(pre_delay_time / self.supervisor_view.TIME_STEP)))
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
                    1, int(np.ceil(post_delay_time / self.supervisor_view.TIME_STEP)))
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

        start_torso_qpos = start_qpos[self.supervisor_view.w1_params.torso_index]
        start_head_qpos = start_qpos[self.supervisor_view.w1_params.head_index]
        start_left_arm_qpos = start_qpos[self.supervisor_view.w1_params.left_arm_index]
        start_right_arm_qpos = start_qpos[self.supervisor_view.w1_params.right_arm_index]

        if left_qpos_seed is None:
            left_qpos_seed = start_left_arm_qpos
        if right_qpos_seed is None:
            right_qpos_seed = start_right_arm_qpos

        self.supervisor_view.logger.debug(
            f"plan_linear_full_body() input cur_qpos: {start_qpos}")
        self.supervisor_view.logger.debug(
            f"plan_linear_full_body() input cur_ee_qpos: {start_left_arm_ee_qpos}, {start_right_arm_ee_qpos}"
        )
        self.supervisor_view.logger.debug("plan_linear_full_body() input waypoints_dict:")
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
            if not self.supervisor_view._has_arm_ee_client("left_arm_ee", False):
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
                    self.supervisor_view.logger.error("Plan to left arm traj failed.")
                    return res_left_arm, None, None, None
            else:
                if not self.supervisor_view._check_qpos_limits("left_arm_ee",
                                               start_left_arm_ee_qpos):
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
                    self.supervisor_view.logger.error("Plan to left arm traj with ee failed.")
                    return res_left_arm, None, None, None

            # right_arm
            right_arm_ee_trigger_list = None
            if not self.supervisor_view._has_arm_ee_client("right_arm_ee", False):
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
                    self.supervisor_view.logger.error("Plan to right arm traj failed.")
                    return res_right_arm, None, None, None
            else:
                if not self.supervisor_view._check_qpos_limits("right_arm_ee",
                                               start_right_arm_ee_qpos):
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
                    self.supervisor_view.logger.error("Plan to right arm traj with ee failed.")
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
                            start_qpos[self.supervisor_view.w1_params.left_arm_index],
                            left_arm_qpos_array
                        ])
                        right_arm_qpos_array = np.vstack([
                            start_qpos[self.supervisor_view.w1_params.right_arm_index],
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
                            start_qpos[self.supervisor_view.w1_params.left_arm_index],
                            left_arm_qpos_array
                        ])
                        right_arm_qpos_array = np.vstack([
                            start_qpos[self.supervisor_view.w1_params.right_arm_index],
                            right_arm_qpos_array
                        ])
                if res_dual_arm.value < 0:
                    self.supervisor_view.logger.error("Plan to dual arm sync traj failed.")
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
                            start_qpos[self.supervisor_view.w1_params.left_arm_index],
                            left_arm_qpos_array
                        ])
                        right_arm_qpos_array = np.vstack([
                            start_qpos[self.supervisor_view.w1_params.right_arm_index],
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
                            start_qpos[self.supervisor_view.w1_params.left_arm_index],
                            left_arm_qpos_array
                        ])
                        right_arm_qpos_array = np.vstack([
                            start_qpos[self.supervisor_view.w1_params.right_arm_index],
                            right_arm_qpos_array
                        ])
                if res_dual_arm.value < 0:
                    self.supervisor_view.logger.error("Plan to dual arm sync traj failed.")
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
        res_align, total_qpos_array = StaticUtilityW1.qpos_align(self.supervisor_view, torso_qpos_array,
                                                       head_qpos_array,
                                                       left_arm_qpos_array,
                                                       right_arm_qpos_array)
        if not res_align:
            return StatusCode.FAIL, None, None, None

        if self.supervisor_view.is_traj_collision(
                qpos_array=total_qpos_array,
                left_arm_ee_trigger_list=left_arm_ee_trigger_list,
                right_arm_ee_trigger_list=right_arm_ee_trigger_list
        ) != StatusCode.FAIL:
            self.supervisor_view.logger.error("Full body linear traj collision detected!")
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
            res, cur_left_arm_ee_qpos = self.supervisor_view.get_current_qpos("left_arm_ee")
            if res.value < 0:
                return res
        cur_right_arm_ee_qpos = None
        if self.supervisor_view._has_arm_ee_client("right_arm_ee", False):
            res, cur_right_arm_ee_qpos = self.supervisor_view.get_current_qpos("right_arm_ee")
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
        return self.supervisor_view.execute_joint_traj(total_qpos_array,
                                       left_arm_ee_trigger_list,
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
            res, T_chassis_ee = self.supervisor_view.get_current_xpos(name, False)
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
            res, T_chassis_ee = self.supervisor_view.get_current_xpos(name, False)
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
        cur_qseed = cur_qpos[self.supervisor_view.w1_params.index_infos[name]["index"]]
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
        default_qvel =  self.supervisor_view.w1_params.default_qvel[
            self.supervisor_view.w1_params.index_infos[name]["index"]]
        min_qvel = self.supervisor_view.w1_params.min_qvel[self.supervisor_view.w1_params.index_infos[name]
                                           ["index"]]
        max_qvel = self.supervisor_view.w1_params.max_qvel[self.supervisor_view.w1_params.index_infos[name]
                                           ["index"]]
        qvel_limits_array = np.clip(np.outer(speed_ratio_list, default_qvel),
                                    min_qvel, max_qvel)

        res_plan, arm_qpos_array = self.supervisor_view.plan_joint_traj(name,
                                                        np.array(qpos_list),
                                                        False,
                                                        qvel_limits_array)
        if res_plan.value < 0:
            return res_plan, None

        # align
        if name == "left_arm":
            res_align, total_qpos_array = StaticUtilityW1.qpos_align(
                self.supervisor_view,
                cur_qpos[self.supervisor_view.w1_params.torso_index],
                cur_qpos[self.supervisor_view.w1_params.head_index], arm_qpos_array,
                cur_qpos[self.supervisor_view.w1_params.right_arm_index])
        else:
            res_align, total_qpos_array = StaticUtilityW1.qpos_align(
                self.supervisor_view,
                cur_qpos[self.supervisor_view.w1_params.torso_index],
                cur_qpos[self.supervisor_view.w1_params.head_index],
                cur_qpos[self.supervisor_view.w1_params.left_arm_index], arm_qpos_array)
        if not res_align:
            return StatusCode.FAIL

        return self.supervisor_view.execute_joint_traj(total_qpos_array)
