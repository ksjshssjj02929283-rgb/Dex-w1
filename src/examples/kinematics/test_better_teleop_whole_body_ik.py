'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

from pathlib import Path
import numpy as np
from IPython import embed

from dexe_robot_client.w1.kinematics.whole_body_ik import *
from dexe_rp import W1Vis, pos_wxyz_to_homogeneous, Box, Sphere, Mesh, Material, rotation_to_wxyz
from loop_rate_limiters import RateLimiter


def _load_viser_environment(vis):
    scene_handle_list = []
    workobjects_handle_list = []

    return scene_handle_list, workobjects_handle_list


def load_viser(file_path: str = None):
    vis = W1Vis()
    vis.set_qpos("robot", home_qpos)
    vis.set_ee_pos("left_ee", left_ee_home_qpos)
    vis.set_ee_pos("right_ee", right_ee_home_qpos)

    scene_handle_list, workobjects_handle_list = _load_viser_environment(vis)

    return vis, scene_handle_list, workobjects_handle_list


def viser_execute_traj(vis,
                       trajectory,
                       armlee_trigger=None,
                       armree_trigger=None):
    rate = RateLimiter(frequency=100, warn=True)
    print(f"trajectory length: {len(trajectory)}")
    idx = 0
    lee_idx = 0
    ree_idx = 0
    while True:
        if idx < len(trajectory):
            full_qpos = trajectory[idx]
            vis.set_qpos("robot", full_qpos)
            if armlee_trigger is not None and lee_idx < len(
                    armlee_trigger) and idx == armlee_trigger[lee_idx][0]:
                vis.set_ee_pos("left_ee",
                               np.asarray(armlee_trigger[lee_idx][1]))
                lee_idx += 1
            if armree_trigger is not None and ree_idx < len(
                    armree_trigger) and idx == armree_trigger[ree_idx][0]:
                vis.set_ee_pos("right_ee",
                               np.asarray(armree_trigger[ree_idx][1]))
                ree_idx += 1
            idx += 1
        else:
            break
        rate.sleep()


# customize teleoperate whole body IK

if __name__ == "__main__":
    # yapf: disable
    home_qpos = np.array([
        0.785, -1.57, 0.9, 0.0,
        0.0, -0.4,
        -1.57, -0.785, 1.57, -0.785, 0.0, 0.0, 0.0,
        1.57, 0.785, -1.57, 0.785, 0.0, 0.0, 0.0
    ])
    # yapf: enable

    left_ee_home_qpos = np.array([0.0] * 6)
    right_ee_home_qpos = np.array([0.0] * 6)

    vis, scene_handle_list, workobjects_handle_list = load_viser()

    urdf_path = Path.home() / ".cache/dexe_data/assembled/w1/robot.urdf"
    left_j7_T_tracking = np.array([[0, 0.707, 0.707, -0.15], [1, 0, 0, 0],
                                   [0, 0.707, -0.707, 0], [0, 0, 0, 1]])
    right_j7_T_tracking = np.array([[0, 0.707, 0.707, -0.15], [1, 0, 0, 0],
                                    [0, 0.707, -0.707, 0], [0, 0, 0, 1]])
    neck2_T_tracking = np.array([[0, 0, -1, 0], [0, -1, 0, -0.1], [-1, 0, 0, 0],
                                 [0, 0, 0, 1]])

    model = pinocchio.buildModelFromUrdf(urdf_path)
    data = model.createData()
    pinocchio.forwardKinematics(model, data, home_qpos)
    pinocchio.updateFramePlacements(model, data)
    left_j7_id = model.getFrameId("LEFT_J7")
    right_j7_id = model.getFrameId("RIGHT_J7")
    neck2_id = model.getFrameId("NECK2")

    left_ee_tracking_frame = TrackingFrame(frame_name="left_arm_tracking_frame",
                                           parent_joint_name="LEFT_J7",
                                           parent_T_frame=left_j7_T_tracking)
    right_ee_tracking_frame = TrackingFrame(
        frame_name="right_arm_tracking_frame",
        parent_joint_name="RIGHT_J7",
        parent_T_frame=right_j7_T_tracking)
    head_tracking_frame = TrackingFrame(frame_name="head_tracking_frame",
                                        parent_joint_name="NECK2",
                                        parent_T_frame=neck2_T_tracking)
    tracking_frame_config = TrackingFrameConfig(
        left_ee_frame=left_ee_tracking_frame,
        right_ee_frame=right_ee_tracking_frame,
        head_ee_frame=head_tracking_frame)

    joint_name_config = JointNameConfig.init_from_robot(robot="W1V02X")

    ik_solver = WholeBodyIK(urdf_path=urdf_path,
                            tracking_frame_config=tracking_frame_config,
                            joint_name_config=joint_name_config,
                            solver_options=IpoptOptions(),
                            weight_config=WeightConfig(),
                            posture_config=PostureConfig(),
                            freezed_joint_dict=None)

    def translation_cost(context: IKBuildContext) -> casadi.SX:
        parts = []
        parts.append(
            ik_solver._bind_q(
                "tele_left_arm_translation_func", ik_solver.cdata.oMf[
                    ik_solver.left_arm_tracking_id].translation, context.q) -
            context.left_arm_target[:3, 3])
        parts.append(
            ik_solver._bind_q(
                "tele_right_arm_translation_func", ik_solver.cdata.oMf[
                    ik_solver.right_arm_tracking_id].translation, context.q) -
            context.right_arm_target[:3, 3])
        parts.append(
            ik_solver._bind_q(
                "tele_head_translation_func", ik_solver.cdata.oMf[
                    ik_solver.head_tracking_id].translation[0], context.q) -
            context.head_target[0, 3])
        parts.append(
            ik_solver._bind_q(
                "tele_head_translation_func", ik_solver.cdata.oMf[
                    ik_solver.head_tracking_id].translation[2], context.q) -
            context.head_target[2, 3])
        return casadi.sumsqr(casadi.vertcat(*parts))

    def rotation_cost(context: IKBuildContext) -> casadi.SX:
        parts = []

        left_arm_current = ik_solver.cdata.oMf[
            ik_solver.left_arm_tracking_id].rotation
        left_arm_current = ik_solver._bind_q("tele_left_arm_rotation_func",
                                             left_arm_current, context.q)
        left_arm_target = context.left_arm_target[:3, :3]
        left_arm_rotation_error = [
            left_arm_current[:, 0] - left_arm_target[:, 0],
            left_arm_current[:, 1] - left_arm_target[:, 1],
            left_arm_current[:, 2] - left_arm_target[:, 2]
        ]
        parts.extend(left_arm_rotation_error)

        right_arm_current = ik_solver.cdata.oMf[
            ik_solver.right_arm_tracking_id].rotation
        right_arm_current = ik_solver._bind_q("tele_right_arm_rotation_func",
                                              right_arm_current, context.q)
        right_arm_target = context.right_arm_target[:3, :3]
        right_arm_rotation_error = [
            right_arm_current[:, 0] - right_arm_target[:, 0],
            right_arm_current[:, 1] - right_arm_target[:, 1],
            right_arm_current[:, 2] - right_arm_target[:, 2]
        ]
        parts.extend(right_arm_rotation_error)

        head_current = ik_solver.cdata.oMf[ik_solver.head_tracking_id].rotation
        head_current = ik_solver._bind_q("tele_head_rotation_func",
                                         head_current, context.q)
        head_target = context.head_target[:3, :3]
        head_rotation_error = [
            head_current[:, 0] - head_target[:, 0],
            head_current[:, 1] - head_target[:, 1],
            head_current[:, 2] - head_target[:, 2]
        ]
        parts.extend(head_rotation_error)

        return casadi.sumsqr(casadi.vertcat(*parts))

    def leg_cost(context: IKBuildContext) -> IKCostTerm:

        def cost_builder(context: IKBuildContext) -> casadi.SX:
            ankle_type, ankle_q = ik_solver.joint_q_or_locked_value(
                context, ik_solver.joint_name_config.ankle_joint)
            knee_type, knee_q = ik_solver.joint_q_or_locked_value(
                context, ik_solver.joint_name_config.knee_joint)
            return casadi.sumsqr(ankle_q + 0.5 * knee_q)

        return IKCostTerm(name=f"leg_cost",
                          expression=cost_builder,
                          weight=10.0)

    def build_teleoperate_problem(context: IKBuildContext) -> IKProblemSpec:
        ik_problem_spec = IKProblemSpec(
            name="teleoperate",
            costs=[
                IKCostTerm(name="translational_tracking_cost",
                           expression=translation_cost,
                           weight=50),
                IKCostTerm(name="rotational_tracking_cost",
                           expression=rotation_cost,
                           weight=1),
                # use joint name index to replace the accurate index number
                IKCostTerm(name="regularization_cost",
                           expression=lambda context: casadi.sumsqr(context.q) +
                           10 * casadi.sumsqr(context.q[0:2]) + casadi.sumsqr(
                               context.q[2]) + casadi.sumsqr(context.q[3]),
                           weight=0.02),
                ik_solver._smooth_cost(context),
                leg_cost(context)
            ],
            constraints=[
                ik_solver._joint_limit_constraint(
                    context,
                    margin_ratio=ik_solver.posture_config.
                    joint_limits_contract_ratio),
                ik_solver._com_constraint(context)
            ])

        return ik_problem_spec

    ik_solver.add_customize_problem("teleoperate", build_teleoperate_problem)
    ik_solver.set_problem("teleoperate")

    cur_qpos = vis.get_qpos()
    cur_arml_j7_xpos = data.oMf[left_j7_id].homogeneous
    cur_armr_j7_xpos = data.oMf[right_j7_id].homogeneous
    left_handle_xpos = cur_arml_j7_xpos @ left_j7_T_tracking
    right_handle_xpos = cur_armr_j7_xpos @ right_j7_T_tracking
    head_handle_xpos = data.oMf[neck2_id].homogeneous @ neck2_T_tracking
    h_target1 = vis.server.scene.add_transform_controls(
        name="/target1",
        scale=0.2,
        position=left_handle_xpos[:3, 3],
        wxyz=rotation_to_wxyz(left_handle_xpos[:3, :3]))
    h_target2 = vis.server.scene.add_transform_controls(
        name="/target2",
        scale=0.2,
        position=right_handle_xpos[:3, 3],
        wxyz=rotation_to_wxyz(right_handle_xpos[:3, :3]))
    h_target3 = vis.server.scene.add_transform_controls(
        name="/target3",
        scale=0.2,
        position=head_handle_xpos[:3, 3],
        wxyz=rotation_to_wxyz(head_handle_xpos[:3, :3]))

    rate = RateLimiter(frequency=100, warn=True)

    input("Press Enter to start the IK control loop...")
    while True:
        T_world_target1 = pos_wxyz_to_homogeneous(h_target1.position,
                                                  h_target1.wxyz)
        T_world_target2 = pos_wxyz_to_homogeneous(h_target2.position,
                                                  h_target2.wxyz)
        T_world_target3 = pos_wxyz_to_homogeneous(h_target3.position,
                                                  h_target3.wxyz)

        cur_qpos = vis.get_qpos()
        res, result_qpos = ik_solver.solve_ik(left_arm_target=T_world_target1,
                                              right_arm_target=T_world_target2,
                                              head_target=T_world_target3,
                                              ref_qpos=cur_qpos)
        if res:
            vis.set_qpos("robot", result_qpos)
            rate.sleep()
        else:
            continue

    embed()
