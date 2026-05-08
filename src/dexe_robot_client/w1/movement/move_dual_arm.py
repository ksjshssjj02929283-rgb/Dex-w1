# ----------------------------------------------------------------------------
# Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import threading
import numpy as np
from ..utils.unit_conversion import user_pose_to_xpos, xpos_to_user_pose
from ..trajectory_planner.toppra_trajectory_planner import ToppraTrajectoryPlanner

from ..mgr_util.supervisor_view import SupervisorViewW1
from ..mgr_util.supervisor_utils import StaticUtilityW1
from ...common import get_logger, time_consuming, StatusCode
from ...common.mapping_index import get_real_idx

from ..path_planner.blend_path import PathType, PathRnBezier, PathR3xSO3Bezier, weighted_norm

from dexe_rp import DexforceW1Kinematics, SRSAKinematics


class DualArmMovementBaseW1(object):

    def __init__(self, supervisor_view: SupervisorViewW1):
        self.supervisor_view = supervisor_view
        self.logger = get_logger()

        self.real_torso_idx, self.real_head_idx, self.real_arml_idx, self.real_armr_idx = get_real_idx(
        )

    def move_dual_arm_single_task(self,
                                  task_dict: dict,
                                  speed_ratio: float = 1.0) -> StatusCode:
        """
        This function is used to execute a single task, which only suits for 1 torso target, 1 head target, 1 arml target, 1 armr target

        Args:
            task_dict (dict): {
                              "torso": [torso_target_qpos], "troso_speed_ratio": [torso_speed],
                              "head": [head_target_qpos], "head_speed_ratio": [head_speed],
                              "left_arm": [left_arm_target_qpos], "left_arm_speed_ratio": [left_arm_speed],
                              "left_arm_ee": [[float., left_arm_ee_qpos, float]], 
                              "right_arm": [right_arm_target_qpos], "right_arm_speed_ratio": [right_arm_speed],
                              "right_arm_ee": [[float., right_arm_ee_qpos, float]]
                              }
            speed_ratio (float, optional): a ratio to scale the default joint velocity. Defaults to 1.0.
            
        Notes:
            - It will execute the moving of torso and head first, then execute the moving of left and right arms including the end effectors.
        """
        res, current_qpos = self.supervisor_view.get_current_qpos()
        if res.value < 0:
            self.logger.error(
                "Function: move_dual_arm_single_task(), failed to get current qpos"
            )
            return res

        cur_torso_qpos = current_qpos[self.real_torso_idx]
        cur_head_qpos = current_qpos[self.real_head_idx]
        cur_arml_qpos = current_qpos[self.real_arml_idx]
        cur_armr_qpos = current_qpos[self.real_armr_idx]

        res, cur_armlee_qpos = self.supervisor_view.get_current_qpos(
            "left_arm_ee")
        if res.value < 0:
            self.logger.error("Get current left arm ee qpos failed.")
            return res
        res, cur_armree_qpos = self.supervisor_view.get_current_qpos(
            "right_arm_ee")
        if res.value < 0:
            self.logger.error("Get current right arm ee qpos failed.")
            return res

        res_torso, torso_qpos_array = self._plan_to_waypoint(
            "torso",
            cur_torso_qpos,
            waypoints_dict=task_dict,
            speed_ratio=speed_ratio)
        if res_torso.value < 0:
            self.logger.error(
                "Function: move_dual_arm_single_task(), failed to plan torso traj"
            )
            return res_torso
        else:
            for i in range(torso_qpos_array.shape[0]):
                target_torso_qpos = torso_qpos_array[i]
                real_qpos = np.concatenate((target_torso_qpos, cur_head_qpos,
                                            cur_arml_qpos, cur_armr_qpos))
                if self.supervisor_view.is_collision(
                        real_qpos,
                        armlee_qpos=cur_armlee_qpos,
                        armree_qpos=cur_armree_qpos) != StatusCode.FAIL:
                    # torso_tf, head_tf, arml_tf, armr_tf, armlee_tf, armree_tf = self._pin_kine.get_Kinematics(real_qpos, cur_armlee_qpos, cur_armree_qpos, compute_ee=True)
                    # self.fcl_checker.update_vis_transform(transforms=torso_tf + head_tf + arml_tf + armr_tf,
                    #                               armlee_transforms=armlee_tf,
                    #                               armree_transforms=armree_tf)
                    self.logger.warning(
                        "It will occur collision for this torso moving trajectory"
                    )
                    return StatusCode.COLLISION_FAIL

        res_head, head_qpos_array = self._plan_to_waypoint(
            "head",
            cur_head_qpos,
            waypoints_dict=task_dict,
            speed_ratio=speed_ratio)
        if res_head.value < 0:
            self.logger.error(
                "Function: move_dual_arm_single_task(), failed to plan head traj"
            )
            return res_head
        else:
            for i in range(head_qpos_array.shape[0]):
                target_head_qpos = head_qpos_array[i]
                real_qpos = np.concatenate((cur_torso_qpos, target_head_qpos,
                                            cur_arml_qpos, cur_armr_qpos))
                if self.supervisor_view.is_collision(
                        real_qpos,
                        armlee_qpos=cur_armlee_qpos,
                        armree_qpos=cur_armree_qpos) != StatusCode.FAIL:
                    self.logger.warning(
                        "It will occur collision for this head moving trajectory"
                    )
                    return StatusCode.COLLISION_FAIL

        res_align, move_torso_head_traj = StaticUtilityW1.qpos_align(
            self.supervisor_view, torso_qpos_array, head_qpos_array,
            cur_arml_qpos, cur_armr_qpos)
        if not res_align:
            self.logger.error(
                "Function: move_dual_arm_single_task(), failed to align torso and head traj"
            )
            return StatusCode.FAIL

        res_execute = self.supervisor_view.execute_joint_traj(
            qpos_array=move_torso_head_traj,
            left_arm_ee_trigger_list=None,
            right_arm_ee_trigger_list=None)
        if res_execute.value < 0:
            self.logger.error("Execute torso and head moving traj failed.")
            return res_execute

        res, current_qpos = self.supervisor_view.get_current_qpos()
        if res.value < 0:
            self.logger.error(
                "Function: move_dual_arm_single_task(), failed to get current qpos"
            )
            return res
        cur_torso_qpos = current_qpos[self.real_torso_idx]
        cur_head_qpos = current_qpos[self.real_head_idx]
        cur_arml_qpos = current_qpos[self.real_arml_idx]
        cur_armr_qpos = current_qpos[self.real_armr_idx]

        res, cur_armlee_qpos = self.supervisor_view.get_current_qpos(
            "left_arm_ee")
        if res.value < 0:
            self.logger.error("Get current left arm ee qpos failed.")
            return res
        res, cur_armree_qpos = self.supervisor_view.get_current_qpos(
            "right_arm_ee")
        if res.value < 0:
            self.logger.error("Get current right arm ee qpos failed.")
            return res

        arml_kine = self.supervisor_view._get_arm_kine("left_arm")
        target_arml_pose = task_dict.get("left_arm")

        arml_speed_ratio = task_dict.get("left_arm_speed_ratio")
        if arml_speed_ratio is not None and arml_speed_ratio != []:
            if arml_speed_ratio[0] is not None:
                arml_speed_ratio = arml_speed_ratio[0]
            else:
                arml_speed_ratio = speed_ratio
        else:
            arml_speed_ratio = speed_ratio

        left_ref_psi = arml_kine.calc_arm_angle(cur_arml_qpos)[1]
        init_left_ref_psi = left_ref_psi
        left_qpos_seed = cur_arml_qpos

        if target_arml_pose is not None and target_arml_pose != []:
            target_arml_xpos = user_pose_to_xpos(target_arml_pose[0][:3],
                                                 target_arml_pose[0][3:])

            while True:
                if left_ref_psi - init_left_ref_psi > 2 * np.pi:
                    self.logger.error(
                        "Cannot find valid ik solution for left arm target pose in move_dual_arm_single_task"
                    )
                    return StatusCode.IK_FAIL
                srs_cfg = SRSAKinematics.Config(is_left=True,
                                                qpos=left_qpos_seed,
                                                arm_angle=left_ref_psi)
                res, arml_ik_qpos = arml_kine.get_ik(target_arml_xpos, srs_cfg)
                if not res:
                    left_ref_psi += 5. / 180. * np.pi
                    continue
                else:
                    real_qpos = np.concatenate((cur_torso_qpos, cur_head_qpos,
                                                arml_ik_qpos, cur_armr_qpos))
                    if self.supervisor_view.is_collision(
                            real_qpos,
                            armlee_qpos=cur_armlee_qpos,
                            armree_qpos=cur_armree_qpos) != StatusCode.FAIL:
                        self.logger.warning(
                            "It will occur collision for this left arm ik solution, try to find another one"
                        )
                        left_ref_psi += 5. / 180. * np.pi
                        continue
                    else:
                        target_arml_qpos = arml_ik_qpos
                        self.logger.info(
                            f"Successfully solve ik for left arm with qpos: {target_arml_qpos}"
                        )
                        break
        else:
            target_arml_qpos = cur_arml_qpos

        armr_kine = self.supervisor_view._get_arm_kine("right_arm")
        target_armr_pose = task_dict.get("right_arm")

        armr_speed_ratio = task_dict.get("right_arm_speed_ratio")
        if armr_speed_ratio is not None and armr_speed_ratio != []:
            if armr_speed_ratio[0] is not None:
                armr_speed_ratio = armr_speed_ratio[0]
            else:
                armr_speed_ratio = speed_ratio
        else:
            armr_speed_ratio = speed_ratio

        right_ref_psi = armr_kine.calc_arm_angle(cur_armr_qpos)[1]
        init_right_ref_psi = right_ref_psi
        right_qpos_seed = cur_armr_qpos

        if target_armr_pose is not None and target_armr_pose != []:
            target_armr_xpos = user_pose_to_xpos(target_armr_pose[0][:3],
                                                 target_armr_pose[0][3:])
            while True:
                if right_ref_psi - init_right_ref_psi > 2 * np.pi:
                    self.logger.error(
                        "Cannot find valid ik solution for right arm target pose in move_dual_arm_single_task"
                    )
                    return StatusCode.IK_FAIL
                srs_cfg = SRSAKinematics.Config(is_left=False,
                                                qpos=right_qpos_seed,
                                                arm_angle=right_ref_psi)
                res, armr_ik_qpos = armr_kine.get_ik(target_armr_xpos, srs_cfg)
                if not res:
                    right_ref_psi += 5. / 180. * np.pi
                    continue
                else:
                    real_qpos = np.concatenate((cur_torso_qpos, cur_head_qpos,
                                                cur_arml_qpos, armr_ik_qpos))
                    if self.supervisor_view.is_collision(
                            real_qpos,
                            armlee_qpos=cur_armlee_qpos,
                            armree_qpos=cur_armree_qpos) != StatusCode.FAIL:
                        self.logger.warning(
                            "It will occur collision for this right arm ik solution, try to find another one"
                        )
                        right_ref_psi += 5. / 180. * np.pi
                        continue
                    else:
                        target_armr_qpos = armr_ik_qpos
                        self.logger.info(
                            f"Successfully solve ik for right arm with qpos: {target_armr_qpos}"
                        )
                        break
        else:
            target_armr_qpos = cur_armr_qpos

        self.supervisor_view.hybrid_planner.set_head_torso(
            head_qpos=cur_head_qpos, torso_qpos=cur_torso_qpos)
        self.supervisor_view.hybrid_planner.set_end_effector_qpos(
            armlee_qpos=cur_armlee_qpos, armree_qpos=cur_armree_qpos)
        self.supervisor_view.hybrid_planner.set_start_goal(
            arml_start=cur_arml_qpos,
            armr_start=cur_armr_qpos,
            arml_goal=target_arml_qpos,
            armr_goal=target_armr_qpos)
        self.supervisor_view.hybrid_planner.set_traj_max_vel(
            left_max_vel=arml_speed_ratio * 0.3,
            right_max_vel=armr_speed_ratio * 0.3)
        arml_traj, armr_traj, planning_time = self.supervisor_view.hybrid_planner.plan(
        )

        self.logger.info(
            f"Finish Planning, time consumption: {planning_time}, with left arm traj shape: {None if arml_traj is None else arml_traj.shape}, right arm traj shape: {None if armr_traj is None else armr_traj.shape}"
        )

        left_gripper_task = task_dict.get("left_arm_ee")
        if left_gripper_task is not None and left_gripper_task != []:
            if left_gripper_task[0] is not None and left_gripper_task[0] != []:
                lee_pre_time = left_gripper_task[0][0]
                lee_post_time = left_gripper_task[0][2]
                lee_target = left_gripper_task[0][1]
            else:
                lee_pre_time = 0.
                lee_post_time = 0.
                lee_target = None
        else:
            lee_pre_time = 0.
            lee_post_time = 0.
            lee_target = None

        arml_pre_insert = lee_pre_time / self.supervisor_view.TIME_STEP
        arml_post_insert = lee_post_time / self.supervisor_view.TIME_STEP

        right_gripper_task = task_dict.get("right_arm_ee")
        if right_gripper_task is not None and right_gripper_task != []:
            if right_gripper_task[0] is not None and right_gripper_task[
                    0] != []:
                ree_pre_time = right_gripper_task[0][0]
                ree_post_time = right_gripper_task[0][2]
                ree_target = right_gripper_task[0][1]
            else:
                ree_pre_time = 0.
                ree_post_time = 0.
                ree_target = None
        else:
            ree_pre_time = 0.
            ree_post_time = 0.
            ree_target = None

        armr_pre_insert = ree_pre_time / self.supervisor_view.TIME_STEP
        armr_post_insert = ree_post_time / self.supervisor_view.TIME_STEP

        armlee_trigger, armree_trigger = [], []  # (idx, ee_qpos)

        if arml_traj is not None and armr_traj is not None:
            if lee_target is not None:
                armlee_trigger.append(
                    (arml_traj.shape[0] + arml_pre_insert, lee_target))
            else:
                armlee_trigger = None
            if ree_target is not None:
                armree_trigger.append(
                    (armr_traj.shape[0] + armr_pre_insert, ree_target))
            else:
                armree_trigger = None

            arml_traj = np.vstack([
                arml_traj,
                np.tile(arml_traj[-1, :],
                        (max(1, int(arml_pre_insert + arml_post_insert)), 1))
            ])
            armr_traj = np.vstack([
                armr_traj,
                np.tile(armr_traj[-1, :],
                        (max(1, int(armr_pre_insert + armr_post_insert)), 1))
            ])

            res_align, move_arml_armr_traj = StaticUtilityW1.qpos_align(
                self.supervisor_view, cur_torso_qpos, cur_head_qpos, arml_traj,
                armr_traj)

            # # check trajectory collision
            if lee_target is None:
                lee_target = cur_armlee_qpos
            if ree_target is None:
                ree_target = cur_armree_qpos
            for qpos in move_arml_armr_traj:
                if self.supervisor_view.is_collision(
                        qpos,
                        armlee_qpos=cur_armlee_qpos,
                        armree_qpos=cur_armree_qpos) != StatusCode.FAIL:
                    # torso_tf, head_tf, arml_tf, armr_tf, armlee_tf, armree_tf = self._pin_kine.get_Kinematics(qpos, cur_armlee_qpos, cur_armree_qpos, compute_ee=True)
                    # self.fcl_checker.update_vis_transform(transforms=torso_tf + head_tf + arml_tf + armr_tf,
                    #                               armlee_transforms=armlee_tf,
                    #                               armree_transforms=armree_tf)
                    self.logger.warning(
                        "It will occur collision for this dual arm moving trajectory"
                    )
                    break
                    # return False

            return self.supervisor_view.execute_joint_traj(
                qpos_array=move_arml_armr_traj,
                left_arm_ee_trigger_list=armlee_trigger,
                right_arm_ee_trigger_list=armree_trigger)

        if arml_traj is not None and armr_traj is None:
            if lee_target is not None:
                armlee_trigger.append(
                    (arml_traj.shape[0] + arml_pre_insert, lee_target))
            else:
                armlee_trigger = None
            if ree_target is not None:
                armree_trigger.append((armr_pre_insert, ree_target))
            else:
                armree_trigger = None

            self.logger.info(armlee_trigger, armree_trigger)
            arml_traj = np.vstack([
                arml_traj,
                np.tile(arml_traj[-1, :],
                        (max(1, int(arml_pre_insert + arml_post_insert)), 1))
            ])

            res_align, move_arml_armr_traj = StaticUtilityW1.qpos_align(
                self.supervisor_view, cur_torso_qpos, cur_head_qpos, arml_traj,
                cur_armr_qpos)

            # check trajectory collision
            if lee_target is None:
                lee_target = cur_armlee_qpos
            if ree_target is None:
                ree_target = cur_armree_qpos
            for qpos in move_arml_armr_traj:
                if self.supervisor_view.is_collision(
                        qpos,
                        armlee_qpos=cur_armlee_qpos,
                        armree_qpos=cur_armree_qpos) != StatusCode.FAIL:
                    # torso_tf, head_tf, arml_tf, armr_tf, armlee_tf, armree_tf = self._pin_kine.get_Kinematics(qpos, cur_armlee_qpos, cur_armree_qpos, compute_ee=True)
                    # self.fcl_checker.update_vis_transform(transforms=torso_tf + head_tf + arml_tf + armr_tf,
                    #                               armlee_transforms=armlee_tf,
                    #                               armree_transforms=armree_tf)
                    self.logger.warning(
                        "It will occur collision for this dual arm moving trajectory"
                    )
                    break
                    # return False

            return self.supervisor_view.execute_joint_traj(
                qpos_array=move_arml_armr_traj,
                left_arm_ee_trigger_list=armlee_trigger,
                right_arm_ee_trigger_list=armree_trigger)

        if arml_traj is None and armr_traj is not None:
            if lee_target is not None:
                armlee_trigger.append((arml_pre_insert, lee_target))
            else:
                armlee_trigger = None
            if ree_target is not None:
                armree_trigger.append(
                    (armr_traj.shape[0] + armr_pre_insert, ree_target))
            else:
                armree_trigger = None

            armr_traj = np.vstack([
                armr_traj,
                np.tile(armr_traj[-1, :],
                        (max(1, int(armr_pre_insert + armr_post_insert)), 1))
            ])

            res_align, move_arml_armr_traj = StaticUtilityW1.qpos_align(
                self.supervisor_view, cur_torso_qpos, cur_head_qpos,
                cur_arml_qpos, armr_traj)

            # check trajectory collision
            if lee_target is None:
                lee_target = cur_armlee_qpos
            if ree_target is None:
                ree_target = cur_armree_qpos
            for qpos in move_arml_armr_traj:
                if self.supervisor_view.is_collision(
                        qpos,
                        armlee_qpos=cur_armlee_qpos,
                        armree_qpos=cur_armree_qpos) != StatusCode.FAIL:
                    # torso_tf, head_tf, arml_tf, armr_tf, armlee_tf, armree_tf = self._pin_kine.get_Kinematics(qpos, cur_armlee_qpos, cur_armree_qpos, compute_ee=True)
                    # self.fcl_checker.update_vis_transform(transforms=torso_tf + head_tf + arml_tf + armr_tf,
                    #                               armlee_transforms=armlee_tf,
                    #                               armree_transforms=armree_tf)
                    self.logger.warning(
                        "It will occur collision for this dual arm moving trajectory"
                    )
                    break
                    # return False

            return self.supervisor_view.execute_joint_traj(
                qpos_array=move_arml_armr_traj,
                left_arm_ee_trigger_list=armlee_trigger,
                right_arm_ee_trigger_list=armree_trigger)

        if arml_traj is None and armr_traj is None:
            if lee_target is not None:
                armlee_trigger.append((arml_pre_insert, lee_target))
            else:
                armlee_trigger = None
            if ree_target is not None:
                armree_trigger.append((armr_pre_insert, ree_target))
            else:
                armree_trigger = None

            arml_traj = np.tile(
                cur_arml_qpos,
                (max(1, int(arml_pre_insert + arml_post_insert)), 1))
            armr_traj = np.tile(
                cur_armr_qpos,
                (max(1, int(armr_pre_insert + armr_post_insert)), 1))

            res_align, move_arml_armr_traj = StaticUtilityW1.qpos_align(
                self.supervisor_view, cur_torso_qpos, cur_head_qpos, arml_traj,
                armr_traj)

            # check trajectory collision
            if lee_target is None:
                lee_target = cur_armlee_qpos
            if ree_target is None:
                ree_target = cur_armree_qpos
            for qpos in move_arml_armr_traj:
                if self.supervisor_view.is_collision(
                        qpos,
                        armlee_qpos=cur_armlee_qpos,
                        armree_qpos=cur_armree_qpos) != StatusCode.FAIL:
                    # torso_tf, head_tf, arml_tf, armr_tf, armlee_tf, armree_tf = self._pin_kine.get_Kinematics(qpos, cur_armlee_qpos, cur_armree_qpos, compute_ee=True)
                    # self.fcl_checker.update_vis_transform(transforms=torso_tf + head_tf + arml_tf + armr_tf,
                    #                               armlee_transforms=armlee_tf,
                    #                               armree_transforms=armree_tf)
                    self.logger.warning(
                        "It will occur collision for this dual arm moving trajectory"
                    )
                    break
                    # return False

            return self.supervisor_view.execute_joint_traj(
                qpos_array=move_arml_armr_traj,
                left_arm_ee_trigger_list=armlee_trigger,
                right_arm_ee_trigger_list=armree_trigger)

    def move_single_arm(self, task_dict: dict, speed_ratio: float = 1.0):
        """
        This function is used to execute a single arm task, when the other arm is static

        Args:
            task_dict (dict): {
                              "torso": [torso_target_qpos1, torso_target_qpos2,], "troso_speed_ratio": [torso_speed1, torso_speed2,],
                              "head": [head_target_qpos1, head,target_qpos2,], "head_speed_ratio": [head_speed1, head_speed2,],
                              "left_arm": [left_arm_target_qpos1, left_arm_target_qpos2,], "left_arm_speed_ratio": [left_arm_speed1, left_arm_speed2,],
                              "left_arm_ee": [[float, left_arm_ee_qpos1, float], [float, left_arm_ee_qpos2, float]], 
                              "right_arm": [right_arm_target_qpos1, right_arm_target_qpos2,], "right_arm_speed_ratio": [right_arm_speed1, right_arm_speed2,],
                              "right_arm_ee": [[float, right_arm_ee_qpos1, float], [float, right_arm_ee_qpos2, float]]
                              }
            speed_ratio (float, optional): a ratio to scale the default joint velocity. Defaults to 1.0.

        Notes:
            - The parameter task_dict should only have one arm's target, the other arm's target should be None or []
        """
        arml_target_list = task_dict.get("left_arm")
        armr_target_list = task_dict.get("right_arm")
        if arml_target_list is not None and arml_target_list != [] and armr_target_list is not None and armr_target_list != []:
            self.logger.info(
                "The input task_dict should only have one arm's target, the other arm's target should be None or []"
            )
            return StatusCode.INPUT_INVALID

        res, current_qpos = self.supervisor_view.get_current_qpos()
        if res.value < 0:
            self.logger.error(
                "Function: move_single_arm(), failed to get current qpos")
            return res
        cur_torso_qpos = current_qpos[self.real_torso_idx]
        cur_head_qpos = current_qpos[self.real_head_idx]
        cur_arml_qpos = current_qpos[self.real_arml_idx]
        cur_armr_qpos = current_qpos[self.real_armr_idx]

        res, cur_armlee_qpos = self.supervisor_view.get_current_qpos(
            "left_arm_ee")
        if res.value < 0:
            self.logger.error("Get current left arm ee qpos failed.")
            return res
        res, cur_armree_qpos = self.supervisor_view.get_current_qpos(
            "right_arm_ee")
        if res.value < 0:
            self.logger.error("Get current right arm ee qpos failed.")
            return res

        res_torso, torso_qpos_array = self._plan_to_waypoint(
            "torso",
            cur_torso_qpos,
            waypoints_dict=task_dict,
            speed_ratio=speed_ratio)
        if res_torso.value < 0:
            return res_torso
        else:
            for i in range(torso_qpos_array.shape[0]):
                target_torso_qpos = torso_qpos_array[i]
                real_qpos = np.concatenate((target_torso_qpos, cur_head_qpos,
                                            cur_arml_qpos, cur_armr_qpos))
                if self.supervisor_view.is_collision(
                        real_qpos,
                        armlee_qpos=cur_armlee_qpos,
                        armree_qpos=cur_armree_qpos) != StatusCode.FAIL:
                    self.logger.warning(
                        "It will occur collision for this torso moving trajectory"
                    )
                    return StatusCode.COLLISION_FAIL

        res_head, head_qpos_array = self._plan_to_waypoint(
            "head",
            cur_head_qpos,
            waypoints_dict=task_dict,
            speed_ratio=speed_ratio)
        if res_head.value < 0:
            return res_head
        else:
            for i in range(head_qpos_array.shape[0]):
                target_head_qpos = head_qpos_array[i]
                real_qpos = np.concatenate((cur_torso_qpos, target_head_qpos,
                                            cur_arml_qpos, cur_armr_qpos))
                if self.supervisor_view.is_collision(
                        real_qpos,
                        armlee_qpos=cur_armlee_qpos,
                        armree_qpos=cur_armree_qpos) != StatusCode.FAIL:
                    self.logger.info(
                        "It will occur collision for this head moving trajectory"
                    )
                    return StatusCode.COLLISION_FAIL
        res_align, move_torso_head_traj = StaticUtilityW1.qpos_align(
            self.supervisor_view, torso_qpos_array, head_qpos_array,
            cur_arml_qpos, cur_armr_qpos)
        self.supervisor_view.execute_joint_traj(qpos_array=move_torso_head_traj,
                                                left_arm_ee_trigger_list=None,
                                                right_arm_ee_trigger_list=None)

        if arml_target_list is not None and arml_target_list != []:
            lee_target_list = task_dict.get("left_arm_ee")
            if len(arml_target_list) != len(lee_target_list):
                self.logger.warning(
                    "The length of left_arm and left_arm_ee in task_dict should be the same"
                )
                return StatusCode.DIM_MISMATCH

            arml_kine = self.supervisor_view._get_arm_kine("left_arm")
            arml_speed_ratio_list = task_dict.get("left_arm_speed_ratio")
            if arml_speed_ratio_list is not None and arml_speed_ratio_list != []:
                for i in range(len(arml_speed_ratio_list)):
                    if arml_speed_ratio_list[i] is None:
                        arml_speed_ratio_list[i] = speed_ratio
            else:
                arml_speed_ratio_list = [speed_ratio] * len(arml_target_list)

            for i in range(len(arml_target_list)):

                res, current_qpos = self.supervisor_view.get_current_qpos()
                if res.value < 0:
                    self.logger.error(
                        "Function: move_single_arm(), failed to get current qpos"
                    )
                    return res
                cur_torso_qpos = current_qpos[self.real_torso_idx]
                cur_head_qpos = current_qpos[self.real_head_idx]
                cur_arml_qpos = current_qpos[self.real_arml_idx]
                cur_armr_qpos = current_qpos[self.real_armr_idx]

                res, cur_armlee_qpos = self.supervisor_view.get_current_qpos(
                    "left_arm_ee")
                if res.value < 0:
                    self.logger.error("Get current left arm ee qpos failed.")
                    return res
                res, cur_armree_qpos = self.supervisor_view.get_current_qpos(
                    "right_arm_ee")
                if res.value < 0:
                    self.logger.error("Get current right arm ee qpos failed.")
                    return res

                arml_trigger = []
                target_arml_pose = arml_target_list[i]
                arml_speed_ratio = arml_speed_ratio_list[i]
                left_ref_psi = arml_kine.calc_arm_angle(cur_arml_qpos)[1]
                init_left_ref_psi = left_ref_psi
                left_qpos_seed = cur_arml_qpos
                if target_arml_pose is not None and target_arml_pose != []:
                    target_arml_xpos = user_pose_to_xpos(
                        target_arml_pose[:3], target_arml_pose[3:])
                    while True:
                        if left_ref_psi - init_left_ref_psi > 2 * np.pi:
                            self.logger.error(
                                "Cannot find valid ik solution for left arm target pose in move_single_arm"
                            )
                            return StatusCode.IK_FAIL
                        srs_cfg = SRSAKinematics.Config(is_left=True,
                                                        qpos=left_qpos_seed,
                                                        arm_angle=left_ref_psi)
                        res_ik, arml_ik_qpos = arml_kine.get_ik(
                            target_arml_xpos, srs_cfg)
                        if not res_ik:
                            left_ref_psi += 5. / 180. * np.pi
                            continue
                        else:
                            real_qpos = np.concatenate(
                                (cur_torso_qpos, cur_head_qpos, arml_ik_qpos,
                                 cur_armr_qpos))
                            if self.supervisor_view.is_collision(
                                    real_qpos,
                                    armlee_qpos=cur_armlee_qpos,
                                    armree_qpos=cur_armree_qpos
                            ) != StatusCode.FAIL:
                                self.logger.warning(
                                    "It will occur collision for this left arm ik solution, try to find another one"
                                )
                                left_ref_psi += 5. / 180. * np.pi
                                continue
                            else:
                                target_arml_qpos = arml_ik_qpos
                                self.logger.info(
                                    f"Successfully solve ik for left arm with qpos: {target_arml_qpos}"
                                )
                                break
                else:
                    self.logger.warning("invalid left arm target input")
                    return StatusCode.INPUT_INVALID

                self.supervisor_view.hybrid_planner.set_head_torso(
                    head_qpos=cur_head_qpos, torso_qpos=cur_torso_qpos)
                self.supervisor_view.hybrid_planner.set_end_effector_qpos(
                    armlee_qpos=cur_armlee_qpos, armree_qpos=cur_armree_qpos)
                self.supervisor_view.hybrid_planner.set_start_goal(
                    arml_start=cur_arml_qpos,
                    armr_start=cur_armr_qpos,
                    arml_goal=target_arml_qpos,
                    armr_goal=cur_armr_qpos)
                self.supervisor_view.hybrid_planner.set_traj_max_vel(
                    left_max_vel=arml_speed_ratio * 0.3, right_max_vel=0.3)
                arml_traj, armr_traj, planning_time = self.supervisor_view.hybrid_planner.plan(
                )

                if arml_traj is None:
                    arml_traj = np.array([cur_arml_qpos])

                lee_pre_time = lee_target_list[i][0]
                lee_post_time = lee_target_list[i][2]
                lee_target = lee_target_list[i][1]

                arml_pre_insert = lee_pre_time / self.supervisor_view.TIME_STEP
                arml_post_insert = lee_post_time / self.supervisor_view.TIME_STEP

                arml_trigger.append(
                    (arml_traj.shape[0] + arml_pre_insert, lee_target))
                arml_traj = np.vstack([
                    arml_traj,
                    np.tile(
                        arml_traj[-1, :],
                        (max(1, int(arml_pre_insert + arml_post_insert)), 1))
                ])
                res_align, move_arml_armr_traj = StaticUtilityW1.qpos_align(
                    self.supervisor_view, cur_torso_qpos, cur_head_qpos,
                    arml_traj, cur_armr_qpos)
                self.supervisor_view.execute_joint_traj(
                    qpos_array=move_arml_armr_traj,
                    left_arm_ee_trigger_list=arml_trigger,
                    right_arm_ee_trigger_list=None)

        elif armr_target_list is not None and armr_target_list != []:
            ree_target_list = task_dict.get("right_arm_ee")
            if len(armr_target_list) != len(ree_target_list):
                self.logger.warning(
                    "The length of right_arm and right_arm_ee in task_dict should be the same"
                )
                return StatusCode.DIM_MISMATCH

            armr_kine = self.supervisor_view._get_arm_kine("right_arm")
            armr_speed_ratio_list = task_dict.get("right_arm_speed_ratio")
            if armr_speed_ratio_list is not None and armr_speed_ratio_list != []:
                for i in range(len(armr_speed_ratio_list)):
                    if armr_speed_ratio_list[i] is None:
                        armr_speed_ratio_list[i] = speed_ratio
            else:
                armr_speed_ratio_list = [speed_ratio] * len(armr_target_list)

            for i in range(len(armr_target_list)):
                res, current_qpos = self.supervisor_view.get_current_qpos()
                if res.value < 0:
                    self.logger.error(
                        "Function: move_single_arm(), failed to get current qpos"
                    )
                    return res
                cur_torso_qpos = current_qpos[self.real_torso_idx]
                cur_head_qpos = current_qpos[self.real_head_idx]
                cur_arml_qpos = current_qpos[self.real_arml_idx]
                cur_armr_qpos = current_qpos[self.real_armr_idx]

                res, cur_arml_qpos = self.supervisor_view.get_current_qpos(
                    "left_arm_ee")
                if res.value < 0:
                    self.logger.error("Get current left arm ee qpos failed.")
                    return res
                res, cur_armr_qpos = self.supervisor_view.get_current_qpos(
                    "right_arm_ee")
                if res.value < 0:
                    self.logger.error("Get current right arm ee qpos failed.")
                    return res

                armr_trigger = []
                target_armr_pose = armr_target_list[i]
                armr_speed_ratio = armr_speed_ratio_list[i]

                right_ref_psi = armr_kine.calc_arm_angle(cur_armr_qpos)[1]
                init_right_ref_psi = right_ref_psi
                right_qpos_seed = cur_armr_qpos
                if target_armr_pose is not None and target_armr_pose != []:
                    target_armr_xpos = user_pose_to_xpos(
                        target_armr_pose[:3], target_armr_pose[3:])
                    while True:
                        if right_ref_psi - init_right_ref_psi > 2 * np.pi:
                            self.logger.error(
                                "Cannot find valid ik solution for right arm target pose in move_single_arm"
                            )
                            return StatusCode.IK_FAIL
                        srs_cfg = SRSAKinematics.Config(is_left=False,
                                                        qpos=right_qpos_seed,
                                                        arm_angle=right_ref_psi)
                        res_ik, armr_ik_qpos = armr_kine.get_ik(
                            target_armr_xpos, srs_cfg)
                        if not res_ik:
                            right_ref_psi += 5. / 180. * np.pi
                            continue
                        else:
                            real_qpos = np.concatenate(
                                (cur_torso_qpos, cur_head_qpos, cur_arml_qpos,
                                 armr_ik_qpos))
                            if self.supervisor_view.is_collision(
                                    real_qpos,
                                    armlee_qpos=cur_armlee_qpos,
                                    armree_qpos=cur_armree_qpos
                            ) != StatusCode.FAIL:
                                self.logger.warning(
                                    "It will occur collision for this right arm ik solution, try to find another one"
                                )
                                right_ref_psi += 5. / 180. * np.pi
                                continue
                            else:
                                target_armr_qpos = armr_ik_qpos
                                self.logger.info(
                                    f"Successfully solve ik for right arm with qpos: {target_armr_qpos}"
                                )
                                break
                else:
                    self.logger.warning("invalid right arm target")
                    return StatusCode.INPUT_INVALID

                self.supervisor_view.hybrid_planner.set_head_torso(
                    head_qpos=cur_head_qpos, torso_qpos=cur_torso_qpos)
                self.supervisor_view.hybrid_planner.set_end_effector_qpos(
                    armlee_qpos=cur_armlee_qpos, armree_qpos=cur_armree_qpos)
                self.supervisor_view.hybrid_planner.set_start_goal(
                    arml_start=cur_arml_qpos,
                    armr_start=cur_armr_qpos,
                    arml_goal=cur_arml_qpos,
                    armr_goal=target_armr_qpos)
                self.supervisor_view.hybrid_planner.set_traj_max_vel(
                    left_max_vel=0.3, right_max_vel=armr_speed_ratio * 0.3)
                arml_traj, armr_traj, planning_time = self.supervisor_view.hybrid_planner.plan(
                )

                if armr_traj is None:
                    armr_traj = np.array([cur_armr_qpos])

                ree_pre_time = ree_target_list[i][0]
                ree_post_time = ree_target_list[i][2]
                ree_target = ree_target_list[i][1]
                armr_pre_insert = ree_pre_time / self.supervisor_view.TIME_STEP
                armr_post_insert = ree_post_time / self.supervisor_view.TIME_STEP
                armr_trigger.append(
                    (armr_traj.shape[0] + armr_pre_insert, ree_target))
                armr_traj = np.vstack([
                    armr_traj,
                    np.tile(
                        armr_traj[-1, :],
                        (max(1, int(armr_pre_insert + armr_post_insert)), 1))
                ])
                res_align, move_arml_armr_traj = StaticUtilityW1.qpos_align(
                    self.supervisor_view, cur_torso_qpos, cur_head_qpos,
                    cur_arml_qpos, armr_traj)
                return self.supervisor_view.execute_joint_traj(
                    qpos_array=move_arml_armr_traj,
                    left_arm_ee_trigger_list=None,
                    right_arm_ee_trigger_list=armr_trigger)

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
            qpos_array = cur_qpos[
                self.supervisor_view.w1_params.index_infos[name]["index"]]
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
            self.supervisor_view.w1_params.default_qvel[
                self.supervisor_view.w1_params.index_infos[name]["index"]],
            qvel_limits_array
        ])
        if is_cartesian:
            if not is_local:
                # Transfer global frame to local frame
                ref_torso_qpos = cur_qpos[self.supervisor_view.w1_params.
                                          index_infos["torso"]["index"]]
                ref_qpos = np.zeros(
                    self.supervisor_view.w1_params.full_body_dofs)
                ref_qpos[self.supervisor_view.w1_params.index_infos["torso"]
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
                cur_arm_qpos = cur_qpos[
                    self.supervisor_view.w1_params.index_infos[name]["index"]]
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
            qpos_array = cur_qpos[
                self.supervisor_view.w1_params.index_infos[name]["index"]]
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
                ref_torso_qpos = cur_qpos[self.supervisor_view.w1_params.
                                          index_infos["torso"]["index"]]
                ref_qpos = np.zeros(
                    self.supervisor_view.w1_params.full_body_dofs)
                ref_qpos[self.supervisor_view.w1_params.index_infos["torso"]
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
                cur_arm_qpos = cur_qpos[
                    self.supervisor_view.w1_params.index_infos[name]["index"]]
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
            self.supervisor_view.w1_params.default_qvel[
                self.supervisor_view.w1_params.index_infos[name]["index"]],
            qvel_limits_array
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

    def plan_arm_ee_trigger(self,
                            arm_ee_state_list: list) -> tuple[StatusCode, list]:
        return StaticUtilityW1.plan_arm_ee_trigger(self.supervisor_view,
                                                   arm_ee_state_list)

    def move_torso_linear(self,
                          delta_x_mm: float,
                          delta_z_mm: float,
                          cartesian_step=0.002,
                          speed_ratio: float = 1.) -> StatusCode:
        """腿部从当前位置直线移动相对距离

        Args:
            delta_x_mm (float): x方向(前后)直线相对运动距离
            delta_z_mm (float): z方向(上下)直线相对运动距离
            cartesian_step (float, optional): 直线离散步长. Defaults to 0.002.
            speed_ratio (float, optional): 运动速率. Defaults to 1..

        Returns:
            StatusCode: 
        """
        res, cur_qpos = self.supervisor_view.get_current_qpos()
        if res.value < 0:
            return res
        res_fk, cur_x, cur_z, cur_y_theta = self.supervisor_view.kine_solver.get_torso_fk(
            cur_qpos[:3])
        if not res_fk:
            return StatusCode.FK_FAIL
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
            res_ik, qpos = self.supervisor_view.kine_solver.get_torso_ik(
                p[0], p[1], cur_y_theta, cur_qpos[1] < 0)
            if not res_ik:
                return StatusCode.IK_FAIL
            qpos_list.append(np.r_[qpos, cur_qpos[3]])

        speed_ratio_list = [speed_ratio] * len(qpos_list)
        default_qvel = self.supervisor_view.w1_params.default_qvel[
            self.supervisor_view.w1_params.index_infos["torso"]["index"]]
        min_qvel = self.supervisor_view.w1_params.min_qvel[
            self.supervisor_view.w1_params.index_infos["torso"]["index"]]
        max_qvel = self.supervisor_view.w1_params.max_qvel[
            self.supervisor_view.w1_params.index_infos["torso"]["index"]]
        qvel_limits_array = np.clip(np.outer(speed_ratio_list, default_qvel),
                                    min_qvel, max_qvel)

        res_plan, torso_qpos_array = StaticUtilityW1.plan_joint_traj(
            self.supervisor_view, "torso", np.array(qpos_list), False,
            qvel_limits_array)
        if res_plan.value < 0:
            return res_plan, None

        # align
        res_align, total_qpos_array = StaticUtilityW1.qpos_align(
            self.supervisor_view, torso_qpos_array,
            cur_qpos[self.supervisor_view.w1_params.head_index],
            cur_qpos[self.supervisor_view.w1_params.left_arm_index],
            cur_qpos[self.supervisor_view.w1_params.right_arm_index])
        if not res_align:
            return StatusCode.FAIL

        return self.supervisor_view.execute_joint_traj(total_qpos_array)
