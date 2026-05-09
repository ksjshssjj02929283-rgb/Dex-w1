import casadi
import numpy as np
import pinocchio
import os
import time
from copy import deepcopy
from pinocchio import casadi as cpin
from pinocchio.robot_wrapper import RobotWrapper
from ...common import get_logger


class TrackFrameTF:
    TELEOP_LEFT_EE = np.array([[0, 0.707, 0.707, -0.15], [1, 0, 0, 0],
                               [0, 0.707, -0.707, 0], [0, 0, 0, 1]])
    TELEOP_RIGHT_EE = np.array([[0, 0.707, 0.707, -0.15], [1, 0, 0, 0],
                                [0, 0.707, -0.707, 0], [0, 0, 0, 1]])
    TELEOP_HEAD_EE = np.array([[0, 0, -1, 0], [0, -1, 0, -0.1], [-1, 0, 0, 0],
                               [0, 0, 0, 1]])
    STANDARD_LEFT_EE = np.array(
        [[2.22044605e-16, 4.37905770e-47, -1.00000000e+00, -6.60000000e-02],
         [-1.42829983e-31, 1.00000000e+00, 3.69778549e-32, 0.00000000e+00],
         [1.00000000e+00, -1.74073880e-32, 2.22044605e-16, 0.00000000e+00],
         [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
    STANDARD_RIGHT_EE = np.array(
        [[2.22044605e-16, 6.56858655e-47, -1.00000000e+00, -6.60000000e-02],
         [-4.42223702e-32, 1.00000000e+00, 3.69778549e-32, -2.22044605e-16],
         [1.00000000e+00, -1.74073880e-32, 2.22044605e-16, 4.93038066e-32],
         [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

    STANDARD_HEAD_EE = np.array([[1, 0, 0, 0.091], [0, 1, 0, -0.051],
                                 [0, 0, 1, 0.0], [0, 0, 0, 1]])


class PinIKWholeBody:

    def __init__(self,
                 urdf_path: str,
                 track_type: str = "teleoperate",
                 cost_type: str = "teleoperate_straight",
                 kept_joint_names: list = None,
                 custom_left_ee_tf: np.ndarray = None,
                 custom_right_ee_tf: np.ndarray = None,
                 custom_head_ee_tf: np.ndarray = None):
        """
        Initialize the Pinocchio-based whole body inverse kinematics solver.

        Args:
            urdf_path (str): Path to the URDF file of the robot.
            track_type (str): Type of tracking frame configuration. Options are "teleoperate", "standard", or "custom".
            cost_type (str): Type of cost function to use. Options are "teleoperate_straight", "teleoperate_bend", "standard", "standard_bend_backward".
            kept_joint_names (list[str]): List of joint names to keep free during IK. If None, all joints are free.
            custom_left_ee_tf (np.ndarray, shape=(4,4)): Custom transformation matrix for the left end-effector.
            custom_right_ee_tf (np.ndarray, shape=(4,4)): Custom transformation matrix for the right end-effector.
            custom_head_ee_tf (np.ndarray, shape=(4,4)): Custom transformation matrix for the head end-effector.

        Notes:
            - If track_type is "custom", please provide custom transformation matrices for all end-effectors.
        """
        self.logger = get_logger()
        self.urdf_path = urdf_path
        self.urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
        if track_type in ["teleoperate", "standard", "custom"]:
            self.track_type = track_type
        else:
            raise ValueError(f"Invalid track_type: {track_type}")

        self.supported_cost_types = [
            "teleoperate_straight", "teleoperate_bend", "standard",
            "standard_bend_backward", "standard_strict"
        ]

        if cost_type in self.supported_cost_types:
            self.cost_type = cost_type
        else:
            raise ValueError(f"Invalid cost_type: {cost_type}")

        robot = pinocchio.RobotWrapper.BuildFromURDF(self.urdf_path,
                                                     self.urdf_dir)
        if kept_joint_names is not None:
            mixed_jointsToLockIDs = [
                joint_name for joint_name in robot.model.names
                if joint_name not in kept_joint_names and joint_name !=
                "universe" and any(frame.name == joint_name and
                                   frame.type == pinocchio.FrameType.JOINT
                                   for frame in robot.model.frames)
            ]
            self.robot_ = robot.buildReducedRobot(
                list_of_joints_to_lock=mixed_jointsToLockIDs,
                reference_configuration=np.array([0.0] * robot.model.nq))
        else:
            self.robot_ = robot

        self._add_track_frame(custom_left_ee_tf=custom_left_ee_tf,
                              custom_right_ee_tf=custom_right_ee_tf,
                              custom_head_ee_tf=custom_head_ee_tf)

        self.focus_point = None

        self._create_pin_casadi()
        self._create_trans_rot_error()
        self.set_cost_function(cost_type=self.cost_type)

    def _add_track_frame(self,
                         custom_left_ee_tf: np.ndarray = None,
                         custom_right_ee_tf: np.ndarray = None,
                         custom_head_ee_tf: np.ndarray = None):
        if self.track_type == "teleoperate":
            target_left_ee, target_right_ee, target_head_ee = \
                TrackFrameTF.TELEOP_LEFT_EE, TrackFrameTF.TELEOP_RIGHT_EE, TrackFrameTF.TELEOP_HEAD_EE
        elif self.track_type == "standard":
            target_left_ee, target_right_ee, target_head_ee = \
                TrackFrameTF.STANDARD_LEFT_EE, TrackFrameTF.STANDARD_RIGHT_EE, TrackFrameTF.STANDARD_HEAD_EE
        elif self.track_type == "custom":
            if custom_left_ee_tf is None or custom_right_ee_tf is None or custom_head_ee_tf is None:
                raise ValueError(
                    "Custom track type requires custom_left_ee_tf, custom_right_ee_tf, and custom_head_ee_tf to be provided."
                )
            target_left_ee, target_right_ee, target_head_ee = \
                custom_left_ee_tf, custom_right_ee_tf, custom_head_ee_tf
        else:
            raise ValueError(f"Invalid track_type: {self.track_type}")

        self.robot_.model.addFrame(
            pinocchio.Frame(
                'tracking_left_ee', self.robot_.model.getJointId("LEFT_J7"),
                pinocchio.SE3(target_left_ee[:3, :3], target_left_ee[:3, 3]),
                pinocchio.FrameType.OP_FRAME))
        self.robot_.model.addFrame(
            pinocchio.Frame(
                'tracking_right_ee', self.robot_.model.getJointId("RIGHT_J7"),
                pinocchio.SE3(target_right_ee[:3, :3], target_right_ee[:3, 3]),
                pinocchio.FrameType.OP_FRAME))
        self.robot_.model.addFrame(
            pinocchio.Frame(
                'tracking_head_ee', self.robot_.model.getJointId("NECK2"),
                pinocchio.SE3(target_head_ee[:3, :3], target_head_ee[:3, 3]),
                pinocchio.FrameType.OP_FRAME))

        self.left_id = self.robot_.model.getFrameId("tracking_left_ee")
        self.right_id = self.robot_.model.getFrameId("tracking_right_ee")
        self.head_id = self.robot_.model.getFrameId("tracking_head_ee")

    def _create_pin_casadi(self):
        self.cmodel = cpin.Model(self.robot_.model)
        self.cdata = self.cmodel.createData()

        self.cq = casadi.SX.sym("q", self.robot_.model.nq, 1)
        self.chead_tf = casadi.SX.sym("head_tf", 4, 4)
        self.cleft_tf = casadi.SX.sym("left_tf", 4, 4)
        self.cright_tf = casadi.SX.sym("right_tf", 4, 4)
        self.reference_cq = casadi.SX.sym("ref_q", self.robot_.model.nq, 1)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

    def _create_trans_rot_error(self):
        calc_head_xz_trans_err = [
            self.cdata.oMf[self.left_id].translation - self.cleft_tf[:3, 3],
            self.cdata.oMf[self.right_id].translation - self.cright_tf[:3, 3],
            self.cdata.oMf[self.head_id].translation[0] - self.chead_tf[0, 3],
            self.cdata.oMf[self.head_id].translation[2] - self.chead_tf[2, 3]
        ]

        calc_head_z_trans_err = [
            self.cdata.oMf[self.left_id].translation - self.cleft_tf[:3, 3],
            self.cdata.oMf[self.right_id].translation - self.cright_tf[:3, 3],
            self.cdata.oMf[self.head_id].translation[2] - self.chead_tf[2, 3]
        ]

        calc_no_head_trans_err = [
            self.cdata.oMf[self.left_id].translation - self.cleft_tf[:3, 3],
            self.cdata.oMf[self.right_id].translation - self.cright_tf[:3, 3]
        ]

        self.translational_error_dict = {
            "standard":
                casadi.Function(f"standard_dual_arm_translation_err",
                                [self.cq, self.cleft_tf, self.cright_tf],
                                [casadi.vertcat(*calc_no_head_trans_err)]),
            "teleoperate_bend":
                casadi.Function(
                    f"teleop_head_xz_translation_err",
                    [self.cq, self.cleft_tf, self.cright_tf, self.chead_tf],
                    [casadi.vertcat(*calc_head_xz_trans_err)]),
            "teleoperate_straight":
                casadi.Function(
                    f"teleop_head_z_translation_err",
                    [self.cq, self.cleft_tf, self.cright_tf, self.chead_tf],
                    [casadi.vertcat(*calc_head_z_trans_err)])
        }

        calc_head_rot_err = [
            cpin.log3(self.cdata.oMf[self.left_id].rotation
                      @ self.cleft_tf[:3, :3].T),
            cpin.log3(self.cdata.oMf[self.right_id].rotation
                      @ self.cright_tf[:3, :3].T),
            cpin.log3(
                self.cdata.oMf[self.head_id].rotation @ self.chead_tf[:3, :3].T)
        ]

        calc_no_head_rot_err = [
            cpin.log3(self.cdata.oMf[self.left_id].rotation
                      @ self.cleft_tf[:3, :3].T),
            cpin.log3(self.cdata.oMf[self.right_id].rotation
                      @ self.cright_tf[:3, :3].T)
        ]

        self.rotational_error_dict = {
            "standard":
                casadi.Function(f"standard_dual_arm_rotation_err",
                                [self.cq, self.cleft_tf, self.cright_tf],
                                [casadi.vertcat(*calc_no_head_rot_err)]),
            "teleoperate":
                casadi.Function(
                    f"teleop_head_rotation_err",
                    [self.cq, self.cleft_tf, self.cright_tf, self.chead_tf],
                    [casadi.vertcat(*calc_head_rot_err)])
        }

    def set_focus_point(self, position: np.ndarray):
        if position is None:
            self.focus_point = None
            self.set_cost_function(cost_type=self.cost_type)
            return True

        if np.atleast_2d(position).shape != (1, 3):
            self.logger.error("Focus point must be a 3D position vector.")
            self.focus_point = None
            return False
        self.focus_point = position
        self.set_cost_function(cost_type=self.cost_type)
        return True

    def set_cost_function(self, cost_type: str):
        """
        Set the cost function of the IK numerical solver's optimization problem

        Args:
            cost_type (str): The type of predefined cost function. Options are "teleoperate_straight", "teleoperate_bend", "standard", "standard_bend_backward".
        """
        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.robot_.model.nq)
        self.var_q_last = self.opti.parameter(self.robot_.model.nq)

        self.param_left_tf = self.opti.parameter(4, 4)
        self.param_right_tf = self.opti.parameter(4, 4)
        self.param_head_tf = self.opti.parameter(4, 4)

        opts = {
            'ipopt': {
                'print_level': 0,
                'max_iter': 100,
                'tol': 1e-4
            },
            'print_time': False,
            'calc_lam_p': False,
        }
        self.opti.solver("ipopt", opts)

        self.solver_dict = {
            "opti": self.opti,
            "var_q": self.var_q,
            "var_q_last": self.var_q_last,
            "param_left_tf": self.param_left_tf,
            "param_right_tf": self.param_right_tf,
            "param_head_tf": self.param_head_tf
        }

        if cost_type not in self.supported_cost_types:
            self.logger.error(
                f"Invalid cost type. Supported cost types are: {self.supported_cost_types}"
            )
            raise ValueError(
                f"Invalid cost type. Supported cost types are: {self.supported_cost_types}"
            )
        else:
            self.cost_type = cost_type

        if cost_type == "standard" or cost_type == "standard_bend_backward" or cost_type == "standard_strict":
            self.translational_cost = casadi.sumsqr(
                self.translational_error_dict["standard"](self.var_q,
                                                          self.param_left_tf,
                                                          self.param_right_tf))
            self.rotation_cost = casadi.sumsqr(
                self.rotational_error_dict["standard"](self.var_q,
                                                       self.param_left_tf,
                                                       self.param_right_tf))
        else:
            self.translational_cost = casadi.sumsqr(
                self.translational_error_dict[cost_type](self.var_q,
                                                         self.param_left_tf,
                                                         self.param_right_tf,
                                                         self.param_head_tf))
            self.rotation_cost = casadi.sumsqr(
                self.rotational_error_dict["teleoperate"](self.var_q,
                                                          self.param_left_tf,
                                                          self.param_right_tf,
                                                          self.param_head_tf))

        self.smooth_cost = casadi.sumsqr(self.var_q - self.var_q_last)
        if cost_type == "teleoperate_bend":
            regularization_cost = (casadi.sumsqr(self.var_q) +
                                   10 * casadi.sumsqr(self.var_q[0:2]) +
                                   1 * casadi.sumsqr(self.var_q[2]) +
                                   10 * casadi.sumsqr(self.var_q[3]))
            leg_cost1 = casadi.sumsqr(self.var_q[0] + 0.5 * self.var_q[1])
            leg_cost2 = casadi.sumsqr(self.var_q[0] + self.var_q[1] +
                                      1 * self.var_q[2])
            freeze_ankle_knee_cost = casadi.sumsqr(self.var_q[0:2] -
                                                   self.var_q_last[0:2])

            self.opti.subject_to(
                self.opti.bounded(self.robot_.model.lowerPositionLimit,
                                  self.var_q,
                                  self.robot_.model.upperPositionLimit))
            self.opti.minimize(50 * self.translational_cost +
                               self.rotation_cost + 0.02 * regularization_cost +
                               0.1 * self.smooth_cost + 10 * leg_cost1)
        if cost_type == "teleoperate_straight":
            regularization_cost = casadi.sumsqr(
                self.var_q) + 10 * casadi.sumsqr(self.var_q[3])
            leg_cost1 = casadi.sumsqr(self.var_q[0] - self.var_q[2])
            leg_cost2 = casadi.sumsqr(self.var_q[0] + self.var_q[1] +
                                      self.var_q[2])
            freeze_ankle_knee_cost = casadi.sumsqr(self.var_q[0:2] -
                                                   self.var_q_last[0:2])

            self.opti.subject_to(
                self.opti.bounded(self.robot_.model.lowerPositionLimit,
                                  self.var_q,
                                  self.robot_.model.upperPositionLimit))
            self.opti.minimize(50 * self.translational_cost +
                               self.rotation_cost + 0.02 * regularization_cost +
                               0.1 * self.smooth_cost + 10 * leg_cost1 +
                               10 * leg_cost2)

        if cost_type == "standard":
            # TODO： tune the weight of waist joint regularization
            regularization_cost = 10 * casadi.sumsqr(self.var_q[3])

            self.opti.subject_to(
                self.opti.bounded(self.robot_.model.lowerPositionLimit,
                                  self.var_q,
                                  self.robot_.model.upperPositionLimit))

            self.opti.subject_to(self.var_q[0] + self.var_q[1] + self.var_q[2]
                                 >= 0)  # torso leaning forward constraint
            self.opti.subject_to(self.var_q[0]
                                 >= 0)  # legs bending forward constraint
            self.opti.subject_to(self.var_q[1] <= 0)
            self.opti.subject_to(self.var_q[2] >= 0)
            self.opti.subject_to(self.var_q[9]
                                 <= 0)  # elbow joint inward constraint
            self.opti.subject_to(self.var_q[16] >= 0)

            self.opti.subject_to(self.var_q[0] + 0.5 * self.var_q[1] == 0)

            # Center of mass cost computation
            cpin.centerOfMass(self.cmodel, self.cdata, self.cq)
            self.com_world = self.cdata.com[0]  # shape (3,)
            self.base_frame_id = self.cmodel.getFrameId("base_link")
            self.T_w_b = self.cdata.oMf[self.base_frame_id]
            self.com_base = self.T_w_b.inverse() * self.com_world  # shape (3,)
            self.com_position = casadi.Function("com_position", [self.cq],
                                                [self.com_base])

            com = self.com_position(self.var_q)

            self.opti.subject_to(com[0] >= -0.25)
            self.opti.subject_to(com[0] <= 0.25)
            self.opti.subject_to(com[1] >= -0.25)
            self.opti.subject_to(com[1] <= 0.25)

            # self.opti.subject_to(self.translational_cost <= 0.000001)
            # self.opti.subject_to(self.rotation_cost <= 0.00005)

            arml6_cost = 1 / (1 + casadi.exp(
                -50 *
                (self.var_q[11] - self.robot_.model.upperPositionLimit[11])
            )) + 1 / (1 + casadi.exp(
                -50 *
                (self.robot_.model.lowerPositionLimit[11] - self.var_q[11])))
            armr6_cost = 1 / (1 + casadi.exp(
                -50 *
                (self.var_q[18] - self.robot_.model.upperPositionLimit[18])
            )) + 1 / (1 + casadi.exp(
                -50 *
                (self.robot_.model.lowerPositionLimit[18] - self.var_q[18])))
            self.arm6_cost = casadi.sumsqr(
                casadi.vertcat(arml6_cost, armr6_cost))

            arml7_cost = 1 / (1 + casadi.exp(
                -50 *
                (self.var_q[12] - self.robot_.model.upperPositionLimit[12])
            )) + 1 / (1 + casadi.exp(
                -50 *
                (self.robot_.model.lowerPositionLimit[12] - self.var_q[12])))
            armr7_cost = 1 / (1 + casadi.exp(
                -50 *
                (self.var_q[19] - self.robot_.model.upperPositionLimit[19])
            )) + 1 / (1 + casadi.exp(
                -50 *
                (self.robot_.model.lowerPositionLimit[19] - self.var_q[19])))
            self.arm7_cost = casadi.sumsqr(
                casadi.vertcat(arml7_cost, armr7_cost))

            arml4_cost = 1 / (1 + casadi.exp(
                -50 *
                (self.robot_.model.lowerPositionLimit[9] - self.var_q[9])))
            armr4_cost = 1 / (1 + casadi.exp(
                -50 *
                (self.var_q[16] - self.robot_.model.upperPositionLimit[16])))
            self.arm4_cost = casadi.sumsqr(
                casadi.vertcat(arml4_cost, armr4_cost))

            left_angle_cost = 1 / (1 + casadi.exp(-25 * (-2 - self.cq[7])))
            right_angle_cost = 1 / (1 + casadi.exp(-25 * (self.cq[14] - 2)))
            angle_cost = [left_angle_cost, right_angle_cost]
            self.angle_cost_func = casadi.Function(
                "angle_cost", [self.cq], [casadi.vertcat(*angle_cost)])

            self.waist_straight_weight = 5 / (1 + casadi.exp(
                -50 *
                (0.55 -
                 (self.param_left_tf[0, 3] + self.param_right_tf[0, 3]) / 2)))

            # focus point cost
            if self.focus_point is None:
                self.focus_point_cost_func = casadi.Function(
                    "focus_point_cost", [self.cq], [0])
            else:
                focus_direction = self.focus_point - self.cdata.oMf[
                    self.head_id].translation
                focus_direction = focus_direction / (
                    casadi.norm_2(focus_direction) + 1e-6)
                head_forward = self.cdata.oMf[self.head_id].rotation[:, 0]
                focus_point_cost = 10 * (
                    1 - casadi.dot(focus_direction, head_forward))
                self.focus_point_cost_func = casadi.Function(
                    "focus_point_cost", [self.cq], [focus_point_cost])

            # yapf: disable
            self.opti.minimize(
                800 * self.translational_cost
                + 10 * self.rotation_cost
                + 0.02 * regularization_cost
                + 5 * self.smooth_cost
                + self.waist_straight_weight * casadi.sumsqr(self.var_q[0] + self.var_q[1] + self.var_q[2])  # upper body straight cost
                + 10 * casadi.sumsqr(self.var_q[0:3] -
                                   self.var_q_last[0:3])  # move arms first cost
                + 10 * (self.arm6_cost + self.arm7_cost + self.arm4_cost)
                + 10 * casadi.sumsqr(self.angle_cost_func(self.var_q))
                + 10 * casadi.sumsqr(self.focus_point_cost_func(self.var_q))
            )
            # yapf: enable

        if cost_type == "standard_bend_backward":
            regularization_cost = 10 * casadi.sumsqr(self.var_q[3])

            self.opti.subject_to(
                self.opti.bounded(self.robot_.model.lowerPositionLimit,
                                  self.var_q,
                                  self.robot_.model.upperPositionLimit))

            self.opti.subject_to(self.var_q[0] + self.var_q[1] + self.var_q[2]
                                 >= 0)  # torso leaning forward constraint
            self.opti.subject_to(self.var_q[0]
                                 <= 0)  # legs bending forward constraint
            self.opti.subject_to(self.var_q[1] >= 0)
            self.opti.subject_to(self.var_q[2] <= 0)
            self.opti.subject_to(self.var_q[9]
                                 <= 0)  # elbow joint inward constraint
            self.opti.subject_to(self.var_q[16] >= 0)

            self.opti.subject_to(self.var_q[0] + 0.5 * self.var_q[1] == 0)

            # Center of mass cost computation
            cpin.centerOfMass(self.cmodel, self.cdata, self.cq)
            self.com_world = self.cdata.com[0]  # shape (3,)
            self.base_frame_id = self.cmodel.getFrameId("base_link")
            self.T_w_b = self.cdata.oMf[self.base_frame_id]
            self.com_base = self.T_w_b.inverse() * self.com_world  # shape (3,)
            self.com_position = casadi.Function("com_position", [self.cq],
                                                [self.com_base])

            com = self.com_position(self.var_q)

            self.opti.subject_to(com[0] >= -0.25)
            self.opti.subject_to(com[0] <= 0.25)
            self.opti.subject_to(com[1] >= -0.25)
            self.opti.subject_to(com[1] <= 0.25)

            arml6_cost = 1 / (1 + casadi.exp(
                -50 *
                (self.var_q[11] - self.robot_.model.upperPositionLimit[11])
            )) + 1 / (1 + casadi.exp(
                -50 *
                (self.robot_.model.lowerPositionLimit[11] - self.var_q[11])))
            armr6_cost = 1 / (1 + casadi.exp(
                -50 *
                (self.var_q[18] - self.robot_.model.upperPositionLimit[18])
            )) + 1 / (1 + casadi.exp(
                -50 *
                (self.robot_.model.lowerPositionLimit[18] - self.var_q[18])))
            self.arm6_cost = casadi.sumsqr(
                casadi.vertcat(arml6_cost, armr6_cost))

            arml7_cost = 1 / (1 + casadi.exp(
                -50 *
                (self.var_q[12] - self.robot_.model.upperPositionLimit[12])
            )) + 1 / (1 + casadi.exp(
                -50 *
                (self.robot_.model.lowerPositionLimit[12] - self.var_q[12])))
            armr7_cost = 1 / (1 + casadi.exp(
                -50 *
                (self.var_q[19] - self.robot_.model.upperPositionLimit[19])
            )) + 1 / (1 + casadi.exp(
                -50 *
                (self.robot_.model.lowerPositionLimit[19] - self.var_q[19])))
            self.arm7_cost = casadi.sumsqr(
                casadi.vertcat(arml7_cost, armr7_cost))

            arml4_cost = 1 / (1 + casadi.exp(
                -50 *
                (self.robot_.model.lowerPositionLimit[9] - self.var_q[9])))
            armr4_cost = 1 / (1 + casadi.exp(
                -50 *
                (self.var_q[16] - self.robot_.model.upperPositionLimit[16])))
            self.arm4_cost = casadi.sumsqr(
                casadi.vertcat(arml4_cost, armr4_cost))

            left_angle_cost = 1 / (1 + casadi.exp(-25 * (-2 - self.cq[7])))
            right_angle_cost = 1 / (1 + casadi.exp(-25 * (self.cq[14] - 2)))
            angle_cost = [left_angle_cost, right_angle_cost]
            self.angle_cost_func = casadi.Function(
                "angle_cost", [self.cq], [casadi.vertcat(*angle_cost)])

            waist_straight_weight = 5 / (1 + casadi.exp(
                -50 *
                (0.6 -
                 (self.param_left_tf[0, 3] + self.param_right_tf[0, 3]) / 2)))

            # focus point cost
            if self.focus_point is None:
                self.focus_point_cost_func = casadi.Function(
                    "focus_point_cost", [self.cq], [0])
            else:
                focus_direction = self.focus_point - self.cdata.oMf[
                    self.head_id].translation
                focus_direction = focus_direction / (
                    casadi.norm_2(focus_direction) + 1e-6)
                head_forward = self.cdata.oMf[self.head_id].rotation[:, 0]
                focus_point_cost = 10 * (
                    1 - casadi.dot(focus_direction, head_forward))
                self.focus_point_cost_func = casadi.Function(
                    "focus_point_cost", [self.cq], [focus_point_cost])

            # yapf: disable
            self.opti.minimize(
                800 * self.translational_cost
                + 10 * self.rotation_cost
                + 0.02 * regularization_cost
                + 5 * self.smooth_cost
                + waist_straight_weight * casadi.sumsqr(self.var_q[0] + self.var_q[1] + self.var_q[2])  # upper body straight cost
                + 10 * casadi.sumsqr(self.var_q[0:3] -
                                   self.var_q_last[0:3])  # move arms first cost
                + 10 * (self.arm6_cost + self.arm7_cost + self.arm4_cost)
                + 10 * casadi.sumsqr(self.angle_cost_func(self.var_q))
                + 10 * casadi.sumsqr(self.focus_point_cost_func(self.var_q))
            )
            # yapf: enable

        if cost_type == "standard_strict":
            # TODO： tune the weight of waist joint regularization
            regularization_cost = 10 * casadi.sumsqr(self.var_q[3])

            self.opti.subject_to(
                self.opti.bounded(self.robot_.model.lowerPositionLimit,
                                  self.var_q,
                                  self.robot_.model.upperPositionLimit))

            self.opti.subject_to(self.var_q[0] + self.var_q[1] + self.var_q[2]
                                 >= 0)  # torso leaning forward constraint
            self.opti.subject_to(self.var_q[0]
                                 >= 0)  # legs bending forward constraint
            self.opti.subject_to(self.var_q[1] <= 0)
            self.opti.subject_to(self.var_q[2] >= 0)
            self.opti.subject_to(self.var_q[9]
                                 <= 0)  # elbow joint inward constraint
            self.opti.subject_to(self.var_q[16] >= 0)

            self.opti.subject_to(self.var_q[0] + 0.5 * self.var_q[1] == 0)

            # Center of mass cost computation
            cpin.centerOfMass(self.cmodel, self.cdata, self.cq)
            self.com_world = self.cdata.com[0]  # shape (3,)
            self.base_frame_id = self.cmodel.getFrameId("base_link")
            self.T_w_b = self.cdata.oMf[self.base_frame_id]
            self.com_base = self.T_w_b.inverse() * self.com_world  # shape (3,)
            self.com_position = casadi.Function("com_position", [self.cq],
                                                [self.com_base])

            com = self.com_position(self.var_q)

            self.opti.subject_to(com[0] >= -0.25)
            self.opti.subject_to(com[0] <= 0.25)
            self.opti.subject_to(com[1] >= -0.25)
            self.opti.subject_to(com[1] <= 0.25)

            self.opti.subject_to(self.translational_cost <= 0.000001)
            self.opti.subject_to(self.rotation_cost <= 0.00005)

            arml6_cost = 1 / (1 + casadi.exp(
                -50 *
                (self.var_q[11] - self.robot_.model.upperPositionLimit[11])
            )) + 1 / (1 + casadi.exp(
                -50 *
                (self.robot_.model.lowerPositionLimit[11] - self.var_q[11])))
            armr6_cost = 1 / (1 + casadi.exp(
                -50 *
                (self.var_q[18] - self.robot_.model.upperPositionLimit[18])
            )) + 1 / (1 + casadi.exp(
                -50 *
                (self.robot_.model.lowerPositionLimit[18] - self.var_q[18])))
            self.arm6_cost = casadi.sumsqr(
                casadi.vertcat(arml6_cost, armr6_cost))

            arml7_cost = 1 / (1 + casadi.exp(
                -50 *
                (self.var_q[12] - self.robot_.model.upperPositionLimit[12])
            )) + 1 / (1 + casadi.exp(
                -50 *
                (self.robot_.model.lowerPositionLimit[12] - self.var_q[12])))
            armr7_cost = 1 / (1 + casadi.exp(
                -50 *
                (self.var_q[19] - self.robot_.model.upperPositionLimit[19])
            )) + 1 / (1 + casadi.exp(
                -50 *
                (self.robot_.model.lowerPositionLimit[19] - self.var_q[19])))
            self.arm7_cost = casadi.sumsqr(
                casadi.vertcat(arml7_cost, armr7_cost))

            arml4_cost = 1 / (1 + casadi.exp(
                -50 *
                (self.robot_.model.lowerPositionLimit[9] - self.var_q[9])))
            armr4_cost = 1 / (1 + casadi.exp(
                -50 *
                (self.var_q[16] - self.robot_.model.upperPositionLimit[16])))
            self.arm4_cost = casadi.sumsqr(
                casadi.vertcat(arml4_cost, armr4_cost))

            left_angle_cost = 1 / (1 + casadi.exp(-25 * (-2 - self.cq[7])))
            right_angle_cost = 1 / (1 + casadi.exp(-25 * (self.cq[14] - 2)))
            angle_cost = [left_angle_cost, right_angle_cost]
            self.angle_cost_func = casadi.Function(
                "angle_cost", [self.cq], [casadi.vertcat(*angle_cost)])

            self.waist_straight_weight = 5 / (1 + casadi.exp(
                -50 *
                (0.55 -
                 (self.param_left_tf[0, 3] + self.param_right_tf[0, 3]) / 2)))

            # focus point cost
            if self.focus_point is None:
                self.focus_point_cost_func = casadi.Function(
                    "focus_point_cost", [self.cq], [0])
            else:
                focus_direction = self.focus_point - self.cdata.oMf[
                    self.head_id].translation
                focus_direction = focus_direction / (
                    casadi.norm_2(focus_direction) + 1e-6)
                head_forward = self.cdata.oMf[self.head_id].rotation[:, 0]
                focus_point_cost = 10 * (
                    1 - casadi.dot(focus_direction, head_forward))
                self.focus_point_cost_func = casadi.Function(
                    "focus_point_cost", [self.cq], [focus_point_cost])

            # yapf: disable
            self.opti.minimize(
                1000 * self.translational_cost
                + 20 * self.rotation_cost
                + 0.02 * regularization_cost
                + 5 * self.smooth_cost
                + self.waist_straight_weight * casadi.sumsqr(self.var_q[0] + self.var_q[1] + self.var_q[2])  # upper body straight cost
                + 10 * casadi.sumsqr(self.var_q[0:3] -
                                   self.var_q_last[0:3])  # move arms first cost
                + 10 * (self.arm6_cost + self.arm7_cost + self.arm4_cost)
                + 10 * casadi.sumsqr(self.angle_cost_func(self.var_q))
                + 10 * casadi.sumsqr(self.focus_point_cost_func(self.var_q))
            )

    def solve_ik(self,
                 left_ee_target: np.ndarray,
                 right_ee_target: np.ndarray,
                 head_ee_target: np.ndarray = None,
                 cur_qpos: np.ndarray = None):

        if cur_qpos is None:
            self.init_data = np.zeros(self.robot_.model.nq)
        else:
            self.init_data = cur_qpos

        self.opti.set_initial(self.var_q, self.init_data)

        self.opti.set_value(self.param_left_tf, left_ee_target)
        self.opti.set_value(self.param_right_tf, right_ee_target)
        if self.cost_type != "standard" and self.cost_type != "standard_bend_backward" and self.cost_type != "standard_strict":
            if head_ee_target is None:
                self.logger.error(
                    "head_ee_target must be provided for teleoperate cost types."
                )
                return False, None
            self.opti.set_value(self.param_head_tf, head_ee_target)
        self.opti.set_value(self.var_q_last, self.init_data)

        try:
            solution = self.opti.solve()
            solution_q = solution.value(self.var_q)
            return True, solution_q
        except Exception as e:
            self.logger.error(f"IK solver failed: {e}")
            self.opti.debug.show_infeasibilities()
            print(f"var_q: {self.opti.debug.value(self.var_q)}")
            print(f"left_ee_target: {left_ee_target}")
            print(f"right_ee_target: {right_ee_target}")
            print(f"translation_cost: {self.opti.debug.value(self.translational_cost)}")
            print(f"rotation_cost: {self.opti.debug.value(self.rotation_cost)}")
            print(f"smooth_cost: {self.opti.debug.value(self.smooth_cost)}")
            print(f"arm6 arm7 cost: {self.opti.debug.value(self.arm6_cost)}  {self.opti.debug.value(self.arm7_cost)}")
            print(f"angle_cost: {self.opti.debug.value(self.angle_cost_func(self.var_q))}")
            print(f"waist_straight_weight: {self.opti.debug.value(self.waist_straight_weight)}")

            rot_err = self.rotational_error_dict["standard"](
                self.var_q, self.param_left_tf, self.param_right_tf
            )
            print(f"rotational error: {self.opti.debug.value(rot_err)}")

            return False, None
