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

    def __init__(self, w1_params, supervisor, kine_solver, pin_kin_solver, collision_checker, time_step: float = 0.01):
        super().__init__(w1_params, supervisor, kine_solver, pin_kin_solver, collision_checker, time_step=time_step)

    def plan_joint_full_body(
            self,
            start_qpos: np.ndarray,
            start_left_arm_ee_qpos: np.ndarray,
            start_right_arm_ee_qpos: np.ndarray,
            waypoints_dict: dict,
            speed_ratio: float = 1.0) -> tuple[bool, np.ndarray, list, list]:
        if not self.supervisor_view._check_qpos_limits("full_body", start_qpos):
            return False, None, None, None

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
            "torso", start_torso_qpos, waypoints_dict, speed_ratio)
        if not res_torso:
            self.logger.error("Plan to torso traj failed.")
            return False, None, None, None

        # head
        res_head, head_qpos_array = self._plan_to_waypoint(
            "head", start_head_qpos, waypoints_dict, speed_ratio)
        if not res_head:
            self.logger.error("Plan to head traj failed.")
            return False, None, None, None

        # left_arm
        left_arm_ee_trigger_list = None
        if not self._has_arm_ee_client("left_arm_ee", False):
            (res_left_arm, left_arm_qpos_array) = self._plan_to_waypoint(
                "left_arm", start_left_arm_qpos, waypoints_dict, speed_ratio)
            if not res_left_arm:
                self.logger.error("Plan to left arm traj failed.")
                return False, None, None, None
        else:
            if not self._check_qpos_limits("left_arm_ee",
                                           start_left_arm_ee_qpos):
                return False, None, None, None
            (res_left_arm, left_arm_qpos_array,
             left_arm_ee_trigger_list) = self._plan_to_waypoint_with_ee(
                 "left_arm", start_left_arm_qpos, start_left_arm_ee_qpos,
                 waypoints_dict, speed_ratio)
            if not res_left_arm:
                self.logger.error("Plan to left arm traj with ee failed.")
                return False, None, None, None

        # right_arm
        right_arm_ee_trigger_list = None
        if not self._has_arm_ee_client("right_arm_ee", False):
            (res_right_arm, right_arm_qpos_array) = self._plan_to_waypoint(
                "right_arm", start_right_arm_qpos, waypoints_dict, speed_ratio)
            if not res_right_arm:
                self.logger.error("Plan to right arm traj failed.")
                return False, None, None, None
        else:
            if not self._check_qpos_limits("right_arm_ee",
                                           start_right_arm_ee_qpos):
                return False, None, None, None
            (res_right_arm, right_arm_qpos_array,
             right_arm_ee_trigger_list) = self._plan_to_waypoint_with_ee(
                 "right_arm", start_right_arm_qpos, start_right_arm_ee_qpos,
                 waypoints_dict, speed_ratio)
            if not res_right_arm:
                self.logger.error("Plan to right arm traj with ee failed.")
                return False, None, None, None

        # align
        res_align, total_qpos_array = StaticUtilityW1.qpos_align(torso_qpos_array,
                                                       head_qpos_array,
                                                       left_arm_qpos_array,
                                                       right_arm_qpos_array)
        if not res_align:
            return False, None, None, None

        return True, total_qpos_array, left_arm_ee_trigger_list, right_arm_ee_trigger_list
    
    def move_joint_full_body(self, waypoints_dict: dict, speed_ratio: float = 1.0):
        res, cur_qpos = self.get_current_qpos()
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
         right_arm_ee_trigger_list) = self.supervisor_view.plan_joint_full_body(
             cur_qpos, cur_left_arm_ee_qpos, cur_right_arm_ee_qpos,
             waypoints_dict, speed_ratio, is_sparse)
        if res.value < 0:
            return res

        # # check collision
        # if self._check_traj_collision(total_qpos_array):
        #     return False

        # execute
        return self.supervisor_view.execute_joint_traj(total_qpos_array,
                                       left_arm_ee_trigger_list,
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
