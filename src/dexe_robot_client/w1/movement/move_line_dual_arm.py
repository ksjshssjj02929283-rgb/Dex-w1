import numpy as np
from .utils.unit_conversion import user_pose_to_xpos, xpos_to_user_pose
from .trajectory_planner.toppra_trajectory_planner import ToppraTrajectoryPlanner
from .path_planner.bezier_blend_path import interpolate_cartesian_path
from .w1_mgr_util.supervisor_view import SupervisorViewW1
from .w1_mgr_util.supervisor_util import StaticUtilityW1
from .w1_mgr_util.move_dual_arm import DualArmMovementBaseW1
from ..common import get_logger


class DualArmMoveLineW1(DualArmMovementBaseW1):

    def __init__(self, w1_params_in, supervisor_in, kine_in):
        super().__init__(w1_params_in, supervisor_in, kine_in)

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
            is_local: bool = True) -> tuple[bool, np.ndarray, list, list]:
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

        Returns:
            tuple[bool, np.ndarray, list, list]: 规划成功与否，全身关节轨迹，左末端执行器轨迹，右末端执行器轨迹
        """
        if not self.supervisor._check_qpos_limits("full_body", start_qpos):
            return False, None, None, None

        start_torso_qpos = start_qpos[self.w1_params.torso_index]
        start_head_qpos = start_qpos[self.w1_params.head_index]
        start_left_arm_qpos = start_qpos[self.w1_params.left_arm_index]
        start_right_arm_qpos = start_qpos[self.w1_params.right_arm_index]

        if left_qpos_seed is None:
            left_qpos_seed = start_left_arm_qpos
        if right_qpos_seed is None:
            right_qpos_seed = start_right_arm_qpos

        self.logger.debug(
            f"plan_linear_full_body() input cur_qpos: {start_qpos}")
        self.logger.debug(
            f"plan_linear_full_body() input cur_ee_qpos: {start_left_arm_ee_qpos}, {start_right_arm_ee_qpos}"
        )
        self.logger.debug("plan_linear_full_body() input waypoints_dict:")
        self.logger.debug(waypoints_dict)
        self.logger.debug(
            f"plan_linear_full_body() input speed_ratio: {speed_ratio}")

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
            if is_local:
                (res_left_arm, left_arm_qpos_array) = self._plan_to_waypoint(
                    "left_arm",
                    start_left_arm_qpos,
                    waypoints_dict,
                    speed_ratio,
                    is_cartesian=True,
                    qpos_seed=left_qpos_seed,
                    cartesian_step=cartesian_step,
                    is_local=True)
            else:
                (res_left_arm, left_arm_qpos_array) = self._plan_to_waypoint(
                    "left_arm",
                    start_qpos,
                    waypoints_dict,
                    speed_ratio,
                    is_cartesian=True,
                    qpos_seed=left_qpos_seed,
                    cartesian_step=cartesian_step,
                    is_local=False)
            if not res_left_arm:
                self.logger.error("Plan to left arm traj failed.")
                return False, None, None, None
        else:
            if not self.supervisor._check_qpos_limits("left_arm_ee",
                                           start_left_arm_ee_qpos):
                return False, None, None, None
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
            if not res_left_arm:
                self.logger.error("Plan to left arm traj with ee failed.")
                return False, None, None, None

        # right_arm
        right_arm_ee_trigger_list = None
        if not self.supervisor._has_arm_ee_client("right_arm_ee", False):
            if is_local:
                (res_right_arm, right_arm_qpos_array) = self._plan_to_waypoint(
                    "right_arm",
                    start_right_arm_qpos,
                    waypoints_dict,
                    speed_ratio,
                    is_cartesian=True,
                    qpos_seed=right_qpos_seed,
                    cartesian_step=cartesian_step,
                    is_local=True)
            else:
                (res_right_arm, right_arm_qpos_array) = self._plan_to_waypoint(
                    "right_arm",
                    start_qpos,
                    waypoints_dict,
                    speed_ratio,
                    is_cartesian=True,
                    qpos_seed=right_qpos_seed,
                    cartesian_step=cartesian_step,
                    is_local=False)
            if not res_right_arm:
                self.logger.error("Plan to right arm traj failed.")
                return False, None, None, None
        else:
            if not self.supervisor._check_qpos_limits("right_arm_ee",
                                           start_right_arm_ee_qpos):
                return False, None, None, None
            if is_local:
                (res_right_arm, right_arm_qpos_array,
                 right_arm_ee_trigger_list) = self._plan_to_waypoint_with_ee(
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
                 right_arm_ee_trigger_list) = self._plan_to_waypoint_with_ee(
                     "right_arm",
                     start_qpos,
                     start_right_arm_ee_qpos,
                     waypoints_dict,
                     speed_ratio,
                     is_cartesian=True,
                     qpos_seed=right_qpos_seed,
                     cartesian_step=cartesian_step,
                     is_local=False)
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

    def move_linear_full_body(self,
                              waypoints_dict: dict,
                              left_qpos_seed: np.ndarray = None,
                              right_qpos_seed: np.ndarray = None,
                              cartesian_step: float = 0.005,
                              speed_ratio: float = 1.0,
                              is_local: bool = True) -> bool:
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
            bool
        """
        res, cur_qpos = self.get_current_qpos()
        if not res:
            return False
        cur_left_arm_ee_qpos = None
        if self._has_arm_ee_client("left_arm_ee", False):
            res, cur_left_arm_ee_qpos = self.get_current_qpos("left_arm_ee")
            if not res:
                return False
        cur_right_arm_ee_qpos = None
        if self._has_arm_ee_client("right_arm_ee", False):
            res, cur_right_arm_ee_qpos = self.get_current_qpos("right_arm_ee")
            if not res:
                return False

        # plan traj
        (res, total_qpos_array, left_arm_ee_trigger_list,
         right_arm_ee_trigger_list) = self.plan_linear_full_body(
             cur_qpos, cur_left_arm_ee_qpos, cur_right_arm_ee_qpos,
             waypoints_dict, left_qpos_seed, right_qpos_seed, cartesian_step,
             speed_ratio, is_local)
        if not res:
            return False

        # # check collision
        # if self._check_traj_collision(total_qpos_array):
        #     return False

        # execute
        return self.supervisor.execute_joint_traj(total_qpos_array,
                                       left_arm_ee_trigger_list,
                                       right_arm_ee_trigger_list)

    def move_arm_linear(self,
                        name: str,
                        delta_trans_mm: list,
                        cartesian_step=0.005,
                        speed_ratio: float = 1.,
                        along_base_frame: bool = True) -> bool:
        """机械臂从当前位置直线移动相对距离

        Args:
            name (str): ["left_arm", "right_arm"]
            delta_trans_mm (list): 直线相对运动距离
            cartesian_step (float, optional): 直线离散步长. Defaults to 0.005.
            speed_ratio (float, optional): 运动速率. Defaults to 1..
            along_base_frame (bool, optional): 参考chassis_base坐标系/tcp坐标系. Defaults to True.

        Returns:
            bool: 
        """
        if not self._check_arm_name(name):
            return False

        res, T_armbase_ee = self.get_current_xpos(name)
        if not res:
            return False

        delta_xpos = user_pose_to_xpos(delta_trans_mm, [0., 0., 0.])
        if along_base_frame:
            res, T_chassis_ee = self.get_current_xpos(name, False)
            if not res:
                return False
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
        if not self._check_arm_name(name):
            return False

        res, T_armbase_ee = self.get_current_xpos(name)
        if not res:
            return False

        delta_xpos = user_pose_to_xpos([0., 0., 0.], delta_rot_deg)
        target_xpos = deepcopy(T_armbase_ee)

        if along_base_frame:
            res, T_chassis_ee = self.get_current_xpos(name, False)
            if not res:
                return False
            target_xpos[:3, :
                        3] = T_armbase_ee[:3, :
                                          3] @ T_chassis_ee[:3, :
                                                            3].T @ delta_xpos[:
                                                                              3, :
                                                                              3] @ T_chassis_ee[:
                                                                                                3, :
                                                                                                3]
        else:
            target_xpos[:3, :3] = T_armbase_ee[:3, :3] @ delta_xpos[:3, :3]

        return self.move_linear_full_body(
            waypoints_dict={f"{name}": [xpos_to_user_pose(target_xpos)]},
            speed_ratio=speed_ratio)
