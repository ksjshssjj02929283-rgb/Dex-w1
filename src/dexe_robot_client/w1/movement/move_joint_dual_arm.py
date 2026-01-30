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
from .move_dual_arm import DualArmMovementBaseW1
from ...common import get_logger, time_consuming, StatusCode

from ..path_planner.blend_path import PathType, PathRnBezier, PathR3xSO3Bezier, weighted_norm

from dexe_rp import DexforceW1Kinematics, SRSConfig


class DualArmMoveJointW1(DualArmMovementBaseW1):

    def __init__(self,
                 w1_params,
                 supervisor,
                 kine_solver,
                 pin_kin_solver,
                 collision_checker,
                 time_step: float = 0.01):
        super().__init__(w1_params,
                         supervisor,
                         kine_solver,
                         pin_kin_solver,
                         collision_checker,
                         time_step=time_step)
        pass

    @time_consuming
    def plan_joint_full_body(
            self,
            start_qpos: np.ndarray,
            start_left_arm_ee_qpos: np.ndarray,
            start_right_arm_ee_qpos: np.ndarray,
            waypoints_dict: dict,
            speed_ratio: float = 1.0,
            is_sparse: bool = True
    ) -> tuple[StatusCode, np.ndarray, list, list]:
        """规划各部件独立关节空间运动
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
            speed_ratio (float, optional): 相对于默认关节速度的比率. Defaults to 1.0,
                        is_sparse (bool, optional): 输入路径点是否稀疏（点间距是否小于路径blend过渡阈值）.

        Returns:
            tuple[bool, np.ndarray, list, list]: 规划成功与否，全身关节轨迹，左末端执行器轨迹，右末端执行器轨迹
        """
        if not self.supervisor_view._check_qpos_limits("full_body", start_qpos):
            return StatusCode.EXCEED_LIMIT, None, None, None

        start_torso_qpos = start_qpos[self.w1_params.torso_index]
        start_head_qpos = start_qpos[self.w1_params.head_index]
        start_left_arm_qpos = start_qpos[self.w1_params.left_arm_index]
        start_right_arm_qpos = start_qpos[self.w1_params.right_arm_index]

        self.logger.debug(
            f"plan_joint_full_body() input cur_qpos: {start_qpos}")
        self.logger.debug(
            f"plan_joint_full_body() input cur_ee_qpos: {start_left_arm_ee_qpos}, {start_right_arm_ee_qpos}"
        )
        self.logger.debug("plan_joint_full_body() input waypoints_dict:")
        self.logger.debug(waypoints_dict)
        self.logger.debug(
            f"plan_joint_full_body() input speed_ratio: {speed_ratio}")

        # torso
        res_torso, torso_qpos_array = self._plan_to_waypoint(
            "torso",
            start_torso_qpos,
            waypoints_dict,
            speed_ratio,
            is_sparse=is_sparse)
        if res_torso.value < 0:
            self.logger.error("Plan to torso traj failed.")
            return res_torso, None, None, None

        # head
        res_head, head_qpos_array = self._plan_to_waypoint("head",
                                                           start_head_qpos,
                                                           waypoints_dict,
                                                           speed_ratio,
                                                           is_sparse=is_sparse)
        if res_head.value < 0:
            self.logger.error("Plan to head traj failed.")
            return res_head, None, None, None

        # left_arm
        left_arm_ee_trigger_list = None
        if not self.supervisor_view._has_arm_ee_client("left_arm_ee", False):
            (res_left_arm,
             left_arm_qpos_array) = self._plan_to_waypoint("left_arm",
                                                           start_left_arm_qpos,
                                                           waypoints_dict,
                                                           speed_ratio,
                                                           is_sparse=is_sparse)
            if res_left_arm.value < 0:
                self.logger.error("Plan to left arm traj failed.")
                return res_left_arm, None, None, None
        else:
            if not self.supervisor_view._check_qpos_limits("left_arm_ee",
                                           start_left_arm_ee_qpos):
                return StatusCode.EXCEED_LIMIT, None, None, None
            (res_left_arm, left_arm_qpos_array,
             left_arm_ee_trigger_list) = self._plan_to_waypoint_with_ee(
                 "left_arm",
                 start_left_arm_qpos,
                 start_left_arm_ee_qpos,
                 waypoints_dict,
                 speed_ratio,
                 is_sparse=is_sparse)
            if res_left_arm.value < 0:
                self.logger.error("Plan to left arm traj with ee failed.")
                return res_left_arm, None, None, None

        # right_arm
        right_arm_ee_trigger_list = None
        if not self.supervisor_view._has_arm_ee_client("right_arm_ee", False):
            (res_right_arm, right_arm_qpos_array) = self._plan_to_waypoint(
                "right_arm",
                start_right_arm_qpos,
                waypoints_dict,
                speed_ratio,
                is_sparse=is_sparse)
            if res_right_arm.value < 0:
                self.logger.error("Plan to right arm traj failed.")
                return res_right_arm, None, None, None
        else:
            if not self.supervisor_view._check_qpos_limits("right_arm_ee",
                                           start_right_arm_ee_qpos):
                return StatusCode.EXCEED_LIMIT, None, None, None
            (res_right_arm, right_arm_qpos_array,
             right_arm_ee_trigger_list) = self._plan_to_waypoint_with_ee(
                 "right_arm",
                 start_right_arm_qpos,
                 start_right_arm_ee_qpos,
                 waypoints_dict,
                 speed_ratio,
                 is_sparse=is_sparse)
            if res_right_arm.value < 0:
                self.logger.error("Plan to right arm traj with ee failed.")
                return res_right_arm, None, None, None

        # align
        res_align, total_qpos_array = StaticUtilityW1.qpos_align(
                                                       self.supervisor_view,
                                                       torso_qpos_array,
                                                       head_qpos_array,
                                                       left_arm_qpos_array,
                                                       right_arm_qpos_array)
        if not res_align:
            return StatusCode.FAIL, None, None, None

        return StatusCode.SUCCESS, total_qpos_array, left_arm_ee_trigger_list, right_arm_ee_trigger_list

    def move_joint_full_body(self,
                             waypoints_dict: dict,
                             speed_ratio: float = 1.0,
                             is_sparse: bool = True) -> StatusCode:
        """从当前位置规划并执行各部件独立关节空间运动
        - 每个key接收多个qpos路径点，key省略视为不控制
        - 同一部件之间的key需要对齐：torso、head、left_xx和right_xx
        - 未指定速度的路径点(None)按speed_ratio补全
        - left_arm_ee/right_arm_ee的每个路径点输入格式为[pre_delay_time, ee_qpos, post_delay_time]
        - key对应的value为 None / [] 视为不控制，其余特殊输入视为不支持

        Args:
            waypoints_dict (dict): {"torso": [torso_qpos1,torso_qpos2,], "torso_speed_ratio": [torso_speed1, torso_speed2,],   
                                    "head": [head_qpos1,], "head_speed_ratio": [head_speed1,],   
                                    "left_arm": [left_arm_qpos1,], "left_arm_speed_ratio": [left_arm_speed1,], 
                                    "left_arm_ee": [[0.,left_arm_ee_qpos1,0.5],],  
                                    "right_arm": [right_arm_qpos1,], "right_arm_speed_ratio": [right_arm_speed1], 
                                    "right_arm_ee": [[0.,right_arm_ee_qpos1,0.5],]}

            speed_ratio (float, optional): 相对于默认关节速度的比率. Defaults to 1.0,
            is_sparse (bool, optional): 输入路径点是否稀疏（点间距是否小于路径blend过渡阈值）.

        Returns:
            StatusCode: 规划并执行成功与否
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
         right_arm_ee_trigger_list) = self.supervisor_view.plan_joint_full_body(
             cur_qpos, cur_left_arm_ee_qpos, cur_right_arm_ee_qpos,
             waypoints_dict, speed_ratio, is_sparse=is_sparse)
        if res.value < 0:
            return res

        # # check collision
        # if self._check_traj_collision(total_qpos_array):
        #     return False

        # execute
        return self.supervisor_view.execute_joint_traj(
            total_qpos_array, left_arm_ee_trigger_list,
            right_arm_ee_trigger_list)

    def move_torso_linear(self,
                          delta_x_mm: float,
                          delta_z_mm: float,
                          cartesian_step=0.002,
                          speed_ratio: float = 1.) -> bool:
        """腿部从当前位置直线移动相对距离

        Args:
            delta_x_mm (float): x方向(前后)直线相对运动距离
            delta_z_mm (float): z方向(上下)直线相对运动距离
            cartesian_step (float, optional): 直线离散步长. Defaults to 0.002.
            speed_ratio (float, optional): 运动速率. Defaults to 1..

        Returns:
            bool: 
        """
        res, cur_qpos = self.get_current_qpos()
        if not res:
            return False
        res_fk, cur_x, cur_z, cur_y_theta = self.kine.get_torso_fk(cur_qpos[:3])
        if not res_fk:
            return False
        target_x = cur_x + (delta_x_mm / 1000.)
        target_z = cur_z + (delta_z_mm / 1000.)
        # TODO: check com in support polyhedron

        # Create path
        p_cur = np.array([cur_x, cur_z])
        p_target = np.array([target_x, target_z])
        dist = np.linalg.norm(p_target - p_cur)
        n_steps = int(np.ceil(dist / cartesian_step))
        s_list = np.linspace(0, 1, n_steps + 1)
        p_list = [p_cur + (p_target - p_cur) * s for s in s_list]

        # Solve ik
        qpos_list = []
        for p in p_list:
            res_ik, qpos = self.kine.get_torso_ik(p[0], p[1], cur_y_theta,
                                                  cur_qpos[1] < 0)
            if not res_ik:
                return False
            qpos_list.append(np.r_[qpos, cur_qpos[3]])

        return self.move_joint_full_body(
            waypoints_dict={f"torso": [qpos.tolist() for qpos in qpos_list]},
            speed_ratio=speed_ratio)


    def plan_joint_traj(
            self,
            name: str,
            qpos_array: np.ndarray,
            is_sparse: bool,
            qvel_limits_array: np.ndarray = None
    ) -> tuple[StatusCode, np.ndarray]:
        """对各部件进行关节轨迹规划，可指定路径点的各关节的速度约束;速度表示从上一点到当前点的最大关节速度

        Args:
            name (str): ["full_body", "torso", "head", "left_arm", "right_arm"]
            qpos_array (np.ndarray): 关节路径点  

            qvel_limits_array (np.ndarray, optional): 路径点关节速度约束，省略则按默认关节速度

        Returns:
            tuple[StatusCode, np.ndarray]
        """
        if not self.supervisor_view._check_name(name):
            return StatusCode.NAME_MISMATCH, None
        if not self.supervisor_view._check_qpos_limits(name, qpos_array):
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

        dofs = self.w1_params.index_infos[name]["dofs"]
        index = self.w1_params.index_infos[name]["index"]
        vel_limits = self.w1_params.max_qvel[index]
        acc_limits = self.w1_params.max_qacc[index]
        traj_planner = ToppraTrajectoryPlanner(dofs, vel_limits, acc_limits)
        res, pos_array, _, _, _ = traj_planner.plan(qpos_array,
                                                    qvel_limits_array,
                                                    self.supervisor_view.TIME_STEP)
        if not res:
            return StatusCode.PLAN_TRAJ_FAIL, None
        return StatusCode.SUCCESS, pos_array

