'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

import os
from copy import deepcopy
from dataclasses import dataclass, field
from abc import ABC, abstractmethod
from typing import Dict, Callable, Iterable, List, Union, Any
from enum import Enum, auto

import casadi
import numpy as np
import pinocchio
from pinocchio import casadi as pin_casadi

from ..collision.signed_distance_field.sdf_map import load_sdf_map, create_casadi_sdf_function

from ...common import get_logger

CasadiExpr = Union[casadi.SX, casadi.MX, casadi.DM, float]
ProblemBuilder = Callable[["IKBuildContext"], "IKProblemSpec"]
CostBuilder = Callable[["IKBuildContext"], CasadiExpr]
ConstraintBuilder = Callable[["IKBuildContext"], Iterable[CasadiExpr]]


@dataclass
class TrackingFrame:
    frame_name: str
    parent_joint_name: str
    parent_T_frame: np.ndarray


@dataclass
class TrackingFrameConfig:
    left_ee_frame: TrackingFrame = None
    right_ee_frame: TrackingFrame = None
    head_ee_frame: TrackingFrame = None
    extra_tracking_frames: Dict[str,
                                TrackingFrame] = field(default_factory=dict)

    @classmethod
    def init_from_urdf(cls, robot: str,
                       urdf_path: str) -> "TrackingFrameConfig":
        if robot not in ["W1V02X", "W1V03X", "A1", "L1"]:
            raise ValueError(f"Unsupported robot: {robot}")
        if not os.path.exists(urdf_path):
            raise ValueError(f"URDF file does not exist: {urdf_path}")
        model = pinocchio.buildModelFromUrdf(urdf_path)
        data = model.createData()
        q_zeros = np.zeros(model.nq)
        pinocchio.forwardKinematics(model, data, q_zeros)
        pinocchio.updateFramePlacements(model, data)
        left_ee_name, right_ee_name, head_ee_name = None, None, None
        left_parent_name, right_parent_name, head_parent_name = None, None, None
        if robot == "W1V02X":
            left_ee_name, right_ee_name, head_ee_name = "LEFT_EE", "RIGHT_EE", "EYES"
            left_parent_name, right_parent_name, head_parent_name = "LEFT_J7", "RIGHT_J7", "NECK2"
        elif robot == "W1V03X":
            left_ee_name, right_ee_name, head_ee_name = "LEFT_EE", "RIGHT_EE", "EYES"
            left_parent_name, right_parent_name, head_parent_name = "WRIST_ROLL_L_J7", "WRIST_ROLL_R_J7", "HEAD_PITCH_J2"
        elif robot == "A1":
            # TODO: fix the names according to the A1 URDF
            left_ee_name, right_ee_name, head_ee_name = "LEFT_EE", "RIGHT_EE", "EYES"
            left_parent_name, right_parent_name, head_parent_name = "LEFT_J7", "RIGHT_J7", "NECK2"
        elif robot == "L1":
            # TODO: fix the names according to the L1 URDF
            left_ee_name, right_ee_name, head_ee_name = "LEFT_EE", "RIGHT_EE", "EYES"
            left_parent_name, right_parent_name, head_parent_name = "LEFT_J7", "RIGHT_J7", "NECK2"

        left_ee_id, left_parent_id = model.getFrameId(
            left_ee_name), model.getFrameId(left_parent_name)
        right_ee_id, right_parent_id = model.getFrameId(
            right_ee_name), model.getFrameId(right_parent_name)
        head_ee_id, head_parent_id = model.getFrameId(
            head_ee_name), model.getFrameId(head_parent_name)

        left_parent_T_ee = np.linalg.inv(data.oMf[left_parent_id].homogeneous
                                        ) @ data.oMf[left_ee_id].homogeneous
        right_parent_T_ee = np.linalg.inv(data.oMf[right_parent_id].homogeneous
                                         ) @ data.oMf[right_ee_id].homogeneous
        head_parent_T_ee = np.linalg.inv(data.oMf[head_parent_id].homogeneous
                                        ) @ data.oMf[head_ee_id].homogeneous

        left_tracking_frame = TrackingFrame(
            frame_name="left_arm_tracking_frame",
            parent_joint_name=left_parent_name,
            parent_T_frame=left_parent_T_ee)
        right_tracking_frame = TrackingFrame(
            frame_name="right_arm_tracking_frame",
            parent_joint_name=right_parent_name,
            parent_T_frame=right_parent_T_ee)
        head_tracking_frame = TrackingFrame(frame_name="head_tracking_frame",
                                            parent_joint_name=head_parent_name,
                                            parent_T_frame=head_parent_T_ee)
        return cls(left_ee_frame=left_tracking_frame,
                   right_ee_frame=right_tracking_frame,
                   head_ee_frame=head_tracking_frame)


@dataclass
class JointNameConfig:
    ankle_joint: str
    knee_joint: str
    buttock_joint: str
    waist_joint: str
    neck1_joint: str
    neck2_joint: str
    left_arm_joints: list[str]
    right_arm_joints: list[str]
    extra_joints: Dict[str, str] = field(default_factory=dict)

    @classmethod
    def init_from_robot(cls, robot: str) -> "JointNameConfig":
        if robot not in ["W1V02X", "W1V03X", "A1", "L1"]:
            raise ValueError(f"Unsupported robot: {robot}")
        # yapf: disable
        if robot == "W1V02X":
            return cls(
                ankle_joint="ANKLE", knee_joint="KNEE", buttock_joint="BUTTOCK", waist_joint="WAIST",
                neck1_joint="NECK1", neck2_joint="NECK2",
                left_arm_joints=["LEFT_J1", "LEFT_J2", "LEFT_J3", "LEFT_J4", "LEFT_J5", "LEFT_J6", "LEFT_J7"],
                right_arm_joints=["RIGHT_J1", "RIGHT_J2", "RIGHT_J3", "RIGHT_J4", "RIGHT_J5", "RIGHT_J6", "RIGHT_J7"]
            )
        elif robot == "W1V03X":
            return cls(
                ankle_joint="ANKLE_PITCH", knee_joint="KNEE_PITCH", buttock_joint="BUTTOCK_PITCH", waist_joint="WAIST_YAW",
                neck1_joint="HEAD_YAW_J1", neck2_joint="HEAD_PITCH_J2",
                left_arm_joints=["SHOULDER_PITCH_L_J1", "SHOULDER_ROLL_L_J2", "ELBOW_PITCH_L_J3", "ELBOW_YAW_L_J4", "WRIST_PITCH_L_J5", "WRIST_YAW_L_J6", "WRIST_ROLL_L_J7"],
                right_arm_joints=["SHOULDER_PITCH_R_J1", "SHOULDER_ROLL_R_J2", "ELBOW_PITCH_R_J3", "ELBOW_YAW_R_J4", "WRIST_PITCH_R_J5", "WRIST_YAW_R_J6", "WRIST_ROLL_R_J7"]
            )
        elif robot == "A1":
            # TODO: fix the joint names according to the A1 URDF
            return cls(
                ankle_joint=None, knee_joint=None, buttock_joint=None, waist_joint=None,
                neck1_joint="NECK1", neck2_joint="NECK2",
                left_arm_joints=["LEFT_J1", "LEFT_J2", "LEFT_J3", "LEFT_J4", "LEFT_J5", "LEFT_J6", "LEFT_J7"],
                right_arm_joints=["RIGHT_J1", "RIGHT_J2", "RIGHT_J3", "RIGHT_J4", "RIGHT_J5", "RIGHT_J6", "RIGHT_J7"]
            )
        elif robot == "L1":
            # TODO: fix the joint names according to the L1 URDF
            return cls(
                ankle_joint=None, knee_joint=None, buttock_joint=None, waist_joint=None,
                neck1_joint="HEAD_YAW_J1", neck2_joint="HEAD_PITCH_J2",
                left_arm_joints=["SHOULDER_PITCH_L_J1", "SHOULDER_ROLL_L_J2", "ELBOW_PITCH_L_J3", "ELBOW_YAW_L_J4", "WRIST_PITCH_L_J5", "WRIST_YAW_L_J6", "WRIST_ROLL_L_J7"],
                right_arm_joints=["SHOULDER_PITCH_R_J1", "SHOULDER_ROLL_R_J2", "ELBOW_PITCH_R_J3", "ELBOW_YAW_R_J4", "WRIST_PITCH_R_J5", "WRIST_YAW_R_J6", "WRIST_ROLL_R_J7"],
                extra_joints={"left_leg_joints": ["HIP_PITCH_L_J1", "HIP_ROLL_L_J2", "HIP_YAW_L_J3", "KNEE_PITCH_L_J4", "ANKLE_PITCH_L_J5", "ANKLE_ROLL_L_J6", "FOOT_L_J7"],
                              "right_leg_joints": ["HIP_PITCH_R_J1", "HIP_ROLL_R_J2", "HIP_YAW_R_J3", "KNEE_PITCH_R_J4", "ANKLE_PITCH_R_J5", "ANKLE_ROLL_R_J6", "FOOT_R_J7"]}
            )

@dataclass(frozen=True)
class CollisionSphere:
    """A sphere rigidly attached to a Pinocchio BODY frame.

    ``center`` is expressed in ``frame_name`` coordinates.  The sphere is an
    IK-only collision primitive: no frame or geometry is added to the URDF.
    """

    name: str
    frame_name: str
    center: tuple[float, float, float]
    radius: float
    weight: float = 1.0

    def __post_init__(self):
        center = np.asarray(self.center, dtype=np.float64)
        if center.shape != (3,) or not np.all(np.isfinite(center)):
            raise ValueError(
                f"collision sphere {self.name} center must contain three finite values"
            )
        radius = float(self.radius)
        if not np.isfinite(radius) or radius <= 0.0:
            raise ValueError(
                f"collision sphere {self.name} radius must be finite and positive"
            )
        weight = float(self.weight)
        if not np.isfinite(weight) or weight < 0.0:
            raise ValueError(
                f"collision sphere {self.name} weight must be finite and non-negative"
            )
        object.__setattr__(self, "center", tuple(float(value) for value in center))
        object.__setattr__(self, "radius", radius)
        object.__setattr__(self, "weight", weight)

@dataclass
class CollisionSphereModelConfig:
    """Robot collision spheres and the common frame used for pair distances.

    Sphere geometry is independent of any collision consumer.  The torso SDF
    and inter-arm collision terms can therefore share one sphere model without
    requiring either feature to be enabled.
    """

    spheres: list[CollisionSphere] = field(default_factory=list)

    def __post_init__(self):
        self.spheres = list(self.spheres)
        if not self.spheres:
            raise ValueError("at least one collision sphere is required")
        if not all(isinstance(sphere, CollisionSphere) for sphere in self.spheres):
            raise ValueError("spheres must contain CollisionSphere objects")
        names = [sphere.name for sphere in self.spheres]
        if len(set(names)) != len(names):
            raise ValueError("collision sphere names must be unique")

    @property
    def sphere_names(self) -> tuple[str, ...]:
        return tuple(sphere.name for sphere in self.spheres)

    @classmethod
    def init_from_robot(cls, robot: str) -> "CollisionSphereModelConfig":
        """
        Reference eight-sphere fit for both W1V02X/W1V03X forearms.
        """
        if robot not in ["W1V02X", "W1V03X", "A1", "L1"]:
            raise ValueError(f"Unsupported robot: {robot}")
        if robot == "W1V02X":
            spheres = [
                CollisionSphere(
                    name="left_j4_sphere_0",
                    frame_name="left_j4",
                    center=(0.0, 0.005198024034396342, 0.013764243491729777),
                    radius=0.05,
                ),
                CollisionSphere(
                    name="left_j4_sphere_1",
                    frame_name="left_j4",
                    center=(0.0, 0.0774246319826733, 0.013013430258358882),
                    radius=0.05,
                ),
                CollisionSphere(
                    name="left_j5_sphere_0",
                    frame_name="left_j5",
                    center=(0.0, 0.006146499074118017, 0.0),
                    radius=0.073,
                ),
                CollisionSphere(
                    name="left_j5_sphere_1",
                    frame_name="left_j5",
                    center=(0.0, 0.008443805315081043, 0.07204693684518815),
                    radius=0.073,
                ),
                CollisionSphere(
                    name="right_j4_sphere_0",
                    frame_name="right_j4",
                    center=(0.0, -0.002170583599583109, -0.013277989523093447),
                    radius=0.05,
                ),
                CollisionSphere(
                    name="right_j4_sphere_1",
                    frame_name="right_j4",
                    center=(0.0, 0.0798450068846504, -0.018604816259440105),
                    radius=0.05,
                ),
                CollisionSphere(
                    name="right_j5_sphere_0",
                    frame_name="right_j5",
                    center=(0.0, -0.010745722554156956, 0.0),
                    radius=0.073,
                ),
                CollisionSphere(
                    name="right_j5_sphere_1",
                    frame_name="right_j5",
                    center=(0.0, -0.00755619484590242, 0.06617731783309722),
                    radius=0.073,
                ),
            ]
            return cls(spheres=spheres)
        elif robot == "W1V03X":
            spheres = [
                CollisionSphere(
                    name="elbow_yaw_l_j4_sphere_0",
                    frame_name="elbow_yaw_l_j4_link",
                    center=(0.0, 0.0, 0.0),
                    radius=0.068,
                ),
                CollisionSphere(
                    name="elbow_yaw_l_j4_sphere_1",
                    frame_name="elbow_yaw_l_j4_link",
                    center=(-0.016354907285170625, 0.09797667436914287, 0.0),
                    radius=0.06,
                ),
                CollisionSphere(
                    name="wrist_pitch_l_j5_sphere_0",
                    frame_name="wrist_pitch_l_j5_link",
                    center=(0.0, 0.02623301030282288, 0.0),
                    radius=0.06,
                ),
                CollisionSphere(
                    name="wrist_pitch_l_j5_sphere_1",
                    frame_name="wrist_pitch_l_j5_link",
                    center=(0.0, 0.09072714569980178, 0.0),
                    radius=0.06,
                ),
                CollisionSphere(
                    name="wrist_yaw_l_j6_sphere_0",
                    frame_name="wrist_yaw_l_j6_link",
                    center=(0.0, 0.08525212808751759, -0.005273968312190291),
                    radius=0.06,
                ),
                CollisionSphere(
                    name="elbow_yaw_r_j4_sphere_0",
                    frame_name="elbow_yaw_r_j4_link",
                    center=(0.0, 0.0, 0.0),
                    radius=0.068,
                ),
                CollisionSphere(
                    name="elbow_yaw_r_j4_sphere_1",
                    frame_name="elbow_yaw_r_j4_link",
                    center=(-0.018672402547112693, -0.09758516071329221, 0.0),
                    radius=0.06,
                ),
                CollisionSphere(
                    name="wrist_pitch_r_j5_sphere_0",
                    frame_name="wrist_pitch_r_j5_link",
                    center=(0.0, -0.02572970514618833, 0.0),
                    radius=0.06,
                ),
                CollisionSphere(
                    name="wrist_pitch_r_j5_sphere_1",
                    frame_name="wrist_pitch_r_j5_link",
                    center=(0.0, -0.08960481987966695, 0.0),
                    radius=0.06,
                ),
                CollisionSphere(
                    name="wrist_yaw_r_j6_sphere_0",
                    frame_name="wrist_yaw_r_j6_link",
                    center=(0.0, -0.07164586268679396, 0.0),
                    radius=0.06,
                ),
            ]
            return cls(spheres=spheres)
        elif robot == "A1":
            pass
        elif robot == "L1":
            pass

@dataclass
class TorsoSDFCollisionConfig:
    """Static sphere-vs-torso SDF collision configuration.

    ``sphere_names`` selects spheres from ``CollisionSphereModelConfig``; an
    empty tuple selects every configured sphere.  ``torso_T_sdf`` is the fixed
    pose of the SDF/OBJ frame in the torso BODY frame.
    """

    sdf_path: str
    torso_frame_name: str
    sphere_names: list[str] = field(default_factory=list)
    torso_T_sdf: np.ndarray = np.eye(4, dtype=np.float64)
    safety_margin: float = 0.0
    activation_distance: float = 0.03
    smooth_epsilon: float = 5.0e-4
    cost_weight: float = 1.0
    hard_constraint: bool = False

    def __post_init__(self):
        if not os.path.exists(self.sdf_path):
            raise ValueError("sdf_path does not exist: {}".format(self.sdf_path))

        self.sphere_names = list(self.sphere_names)
        if len(set(self.sphere_names)) != len(self.sphere_names):
            raise ValueError("sphere_names must be unique")

        transform = np.asarray(self.torso_T_sdf, dtype=np.float64)
        if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
            raise ValueError("torso_T_sdf must be a finite 4x4 transform")

        self.torso_T_sdf = transform.copy()

        if self.safety_margin < 0.0:
            raise ValueError("safety_margin must be non-negative")
        if self.activation_distance <= 0.0:
            raise ValueError("activation_distance must be positive")
        if self.smooth_epsilon <= 0.0:
            raise ValueError("smooth_epsilon must be positive")
        if self.cost_weight < 0.0:
            raise ValueError("cost_weight must be non-negative")

@dataclass
class InterArmSphereCollisionConfig:
    """Soft collision cost for the Cartesian product of two sphere groups.

    Each pair uses the geometric mean of its two sphere weights, so equal
    per-sphere weights retain the same scale and a zero weight disables only
    that pair's cost contribution.
    """

    left_sphere_names: list[str]
    right_sphere_names: list[str]
    safety_margin: float = 0.0
    activation_distance: float = 0.03
    smooth_epsilon: float = 5.0e-4
    cost_weight: float = 2.5

    def __post_init__(self):
        self.left_sphere_names = list(self.left_sphere_names)
        self.right_sphere_names = list(self.right_sphere_names)
        for field_name in ("left_sphere_names", "right_sphere_names"):
            names = getattr(self, field_name)
            if not names:
                raise ValueError(f"{field_name} must not be empty")
            if len(set(names)) != len(names):
                raise ValueError(f"{field_name} must contain unique sphere names")
        overlap = set(self.left_sphere_names) & set(self.right_sphere_names)
        if overlap:
            raise ValueError(
                "left and right sphere groups must be disjoint, got "
                f"{sorted(overlap)}"
            )

        if self.safety_margin < 0.0:
            raise ValueError("safety_margin must be non-negative")
        if self.activation_distance <= 0.0:
            raise ValueError("activation_distance must be positive")
        if self.smooth_epsilon <= 0.0:
            raise ValueError("smooth_epsilon must be positive")
        if self.cost_weight < 0.0:
            raise ValueError("cost_weight must be non-negative")

    @classmethod
    def init_from_robot(cls, robot: str) -> "InterArmSphereCollisionConfig":
        if robot not in ["W1V02X", "W1V03X", "A1", "L1"]:
            raise ValueError(f"Unsupported robot: {robot}")
        if robot == "W1V02X":
            left_sphere_names = ["left_j4_sphere_0", "left_j4_sphere_1", "left_j5_sphere_0", "left_j5_sphere_1"]
            right_sphere_names = ["right_j4_sphere_0", "right_j4_sphere_1", "right_j5_sphere_0", "right_j5_sphere_1"]

            return cls(left_sphere_names=left_sphere_names, right_sphere_names=right_sphere_names)
        elif robot == "W1V03X":
            left_sphere_names = ["elbow_yaw_l_j4_sphere_0", "elbow_yaw_l_j4_sphere_1", "wrist_pitch_l_j5_sphere_0", "wrist_pitch_l_j5_sphere_1", "wrist_yaw_l_j6_sphere_0"]
            right_sphere_names = ["elbow_yaw_r_j4_sphere_0", "elbow_yaw_r_j4_sphere_1", "wrist_pitch_r_j5_sphere_0", "wrist_pitch_r_j5_sphere_1", "wrist_yaw_r_j6_sphere_0"]
            return cls(left_sphere_names=left_sphere_names, right_sphere_names=right_sphere_names)
        elif robot == "A1":
            pass
        elif robot == "L1":
            pass

class JointIndexResolver:
    def __init__(self, model):
        self.model = model

    def q_index(self, joint_name: str) -> int | None:
        if not self.model.existJointName(joint_name):
            return None
        joint_id = self.model.getJointId(joint_name)
        joint = self.model.joints[joint_id]
        if joint.nq != 1:
            raise ValueError(f"Joint {joint_name} has nq={joint.nq}, expected 1")

        return joint.idx_q

    def q(self, q_var, joint_name: str):
        idx = self.q_index(joint_name)
        if idx is None:
            return None
        return q_var[idx]

@dataclass
class SolverOptions(ABC):
    name: str = ""
    print_time: bool = False
    calc_lam_p: bool = False

    @abstractmethod
    def to_casadi_options(self) -> dict:
        pass

@dataclass
class IpoptOptions(SolverOptions):
    name: str = "ipopt"
    ipopt_print_level: int = 0
    ipopt_max_iter: int = 100
    ipopt_tol: float = 1e-4

    print_time: bool = False
    calc_lam_p: bool = False

    def to_casadi_options(self):
        opts = {
            "ipopt": {
                "print_level": self.ipopt_print_level,
                "max_iter": self.ipopt_max_iter,
                "tol": self.ipopt_tol,
            },
            "print_time": self.print_time,
            "calc_lam_p": self.calc_lam_p,
        }
        return opts

@dataclass
class SqpMethodOptions(SolverOptions):
    name: str = "sqpmethod"
    max_iter: int = 100
    tol_pr: float = 1e-6
    tol_du: float = 1e-6

    def to_casadi_options(self):
        return {
            "qpsol": "qrqp",
            "max_iter": self.max_iter,
            "tol_pr": self.tol_pr,
            "tol_du": self.tol_du,
            "print_header": False,
            "print_iteration": False,
            "print_status": False,
            "print_time": False,
            "qpsol_options": {
                "print_header": False,
                "print_iter": False,
                "print_info": False,
            },
        }

@dataclass
class WeightConfig:
    translation: float = 800.0
    rotation: float = 10.0
    regularization: float = 0.02
    smoothness: float = 5.0
    upper_body_straightness: float = 5.0
    leg_heaviness: float = 10.0
    waist_straightness: float = 1.0
    shoulder_angle_barrier: float = 10.0
    eye_focus_point: float = 10.0
    extra_weights: Dict[str, float] = field(default_factory=dict)

@dataclass
class PostureConfig:
    # the threshold of the average x position of the two end effector flanges to start encouraging leaning forward. The value is determined empirically based on the W1V02X robot dimensions and kinematics, and may need to be adjusted for other robots.
    dual_ee_flange_centerX_threshold: float = 0.55

    translation_error_threshold: float = 1e-6
    rotation_error_threshold: float = 5e-5

    # dynamic knots: (lower, upper)
    left_arm_j1_dynamic_knots: tuple[float, float] = (-1.8, 0.0)
    left_arm_j2_dynamic_knots: tuple[float, float] = (-2.0, -1.63)
    right_arm_j1_dynamic_knots: tuple[float, float] = (0.0, 1.8)
    right_arm_j2_dynamic_knots: tuple[float, float] = (1.63, 2.0)

    joint_limits_contract_ratio: float = 0.01

    leg_leaning_forward: bool = True

    com_x_bound: tuple[float, float] = (-0.25, 0.25)
    com_y_bound: tuple[float, float] = (-0.25, 0.25)

    extra_postures: Dict[str, Any] = field(default_factory=dict)

@dataclass
class IKBuildContext:
    solver: "WholeBodyIK"
    opti: casadi.Opti
    q: casadi.MX
    q_last: casadi.MX
    left_arm_target: casadi.MX
    right_arm_target: casadi.MX
    head_target: casadi.MX

    upper_body_leaning_threshold: casadi.MX = None
    eye_focus_point: casadi.MX = None
    eye_focus_weight: casadi.MX = None
    translation_error_threshold: casadi.MX = None
    rotation_error_threshold: casadi.MX = None

    extra_targets: Dict[str, casadi.MX] = field(default_factory=dict)
    extra_contexts: Dict[str, Any] = field(default_factory=dict)

    @property
    def model(self):
        return self.solver.robot_.model

@dataclass
class IKCostTerm:
    name: str
    expression: CostBuilder
    weight: float = 1.0

@dataclass
class IKConstraint:
    name: str
    expressions: ConstraintBuilder

@dataclass
class IKProblemSpec:
    name: str
    costs: List[IKCostTerm] = field(default_factory=list)
    constraints: List[IKConstraint] = field(default_factory=list)

class JointQType(Enum):
    VARIABLE = auto()
    LOCKED = auto()
    MISSING = auto()

class WholeBodyIK:
    def __init__(self,
                 urdf_path: str,
                 tracking_frame_config: TrackingFrameConfig,
                 joint_name_config: JointNameConfig,
                 solver_options: SolverOptions = None,
                 weight_config: WeightConfig = None,
                 posture_config: PostureConfig = None,
                 freezed_joint_dict: dict[str, float] = None,
                 collision_sphere_model_config: CollisionSphereModelConfig = None,
                 torso_sdf_collision_config: TorsoSDFCollisionConfig = None,
                 inter_arm_collision_config: InterArmSphereCollisionConfig = None):
        self.logger = get_logger()
        if not os.path.exists(urdf_path):
            self.logger.error(f"URDF file does not exist: {urdf_path}")
            raise ValueError(f"URDF file does not exist: {urdf_path}")
        self.urdf_path = urdf_path
        self.urdf_dir = os.path.dirname(os.path.abspath(urdf_path))

        robot = pinocchio.RobotWrapper.BuildFromURDF(self.urdf_path, self.urdf_dir)
        if freezed_joint_dict is not None and freezed_joint_dict != {}:
            reference_configuration = np.zeros(robot.model.nq)
            joints_to_lock = []
            for joint_name, joint_angle in freezed_joint_dict.items():
                if not robot.model.existJointName(joint_name):
                    raise ValueError(f"Joint name {joint_name} does not exist in the robot model.")
                joint_id = robot.model.getJointId(joint_name)
                joint = robot.model.joints[joint_id]
                if joint.nq != 1:
                    raise ValueError(f"Joint {joint_name} is not a 1-DoF joint and cannot be locked.")
                q_index = joint.idx_q
                reference_configuration[q_index] = joint_angle
                joints_to_lock.append(joint_name)

            self.robot_ = robot.buildReducedRobot(
                list_of_joints_to_lock=[robot.model.getJointId(joint_name) for joint_name in joints_to_lock],
                reference_configuration=reference_configuration
            )

            self.freezed_joint_dict = freezed_joint_dict

        else:
            self.robot_ = robot
            self.freezed_joint_dict = None

        self.tracking_frame_config = tracking_frame_config
        self.joint_name_config = joint_name_config
        if solver_options is None:
            self.solver_options = IpoptOptions()
        else:
            self.solver_options = solver_options

        self.joint_index_resolver = JointIndexResolver(self.robot_.model)
        if weight_config is None:
            self.weight_config = WeightConfig()
        else:
            self.weight_config = weight_config
        if posture_config is None:
            self.posture_config = PostureConfig()
        else:
            self.posture_config = posture_config

        self.cache_left_arm_target = None
        self.cache_right_arm_target = None
        self.cache_head_target = None
        self.cache_extra_targets = {}
        self.cache_solution_q = None

        self.collision_sphere_model_config = collision_sphere_model_config
        self.torso_sdf_collision_config = torso_sdf_collision_config
        self.inter_arm_collision_config = inter_arm_collision_config

        self._set_tracking_frame(self.tracking_frame_config)
        self._create_symbolic_model()
        self._initialize_collision_spheres()
        self._initialize_torso_collision()
        self._initialize_inter_arm_collision()

        self.problem_builders: Dict[str, ProblemBuilder] = {}
        self.problem_overlays: Dict[str, IKProblemSpec] = {}
        self._register_default_problems()
        self.set_problem("standard")

    def _set_tracking_frame(self, tracking_frame_config: TrackingFrameConfig):
        # TODO: temporarily regard the locked frame cannot be used for tracking's parent joint, but can deal with the locked joint's parent joint as tracking frame's parent joint.
        if tracking_frame_config.left_ee_frame is not None:
            if not self.robot_.model.existJointName(tracking_frame_config.left_ee_frame.parent_joint_name):
                raise ValueError(f"Left EE parent joint {tracking_frame_config.left_ee_frame.parent_joint_name} does not exist in the robot model.")
            self.robot_.model.addFrame(
                pinocchio.Frame(
                    tracking_frame_config.left_ee_frame.frame_name,
                    self.robot_.model.getJointId(tracking_frame_config.left_ee_frame.parent_joint_name),
                    pinocchio.SE3(tracking_frame_config.left_ee_frame.parent_T_frame[:3, :3], tracking_frame_config.left_ee_frame.parent_T_frame[:3, 3]),
                    pinocchio.FrameType.OP_FRAME
                )
            )
        if tracking_frame_config.right_ee_frame is not None:
            if not self.robot_.model.existJointName(tracking_frame_config.right_ee_frame.parent_joint_name):
                raise ValueError(f"Right EE parent joint {tracking_frame_config.right_ee_frame.parent_joint_name} does not exist in the robot model.")
            self.robot_.model.addFrame(
                pinocchio.Frame(
                    tracking_frame_config.right_ee_frame.frame_name,
                    self.robot_.model.getJointId(tracking_frame_config.right_ee_frame.parent_joint_name),
                    pinocchio.SE3(tracking_frame_config.right_ee_frame.parent_T_frame[:3, :3], tracking_frame_config.right_ee_frame.parent_T_frame[:3, 3]),
                    pinocchio.FrameType.OP_FRAME
                )
            )
        if tracking_frame_config.head_ee_frame is not None:
            if not self.robot_.model.existJointName(tracking_frame_config.head_ee_frame.parent_joint_name):
                raise ValueError(f"Head EE parent joint {tracking_frame_config.head_ee_frame.parent_joint_name} does not exist in the robot model.")
            self.robot_.model.addFrame(
                pinocchio.Frame(
                    tracking_frame_config.head_ee_frame.frame_name,
                    self.robot_.model.getJointId(tracking_frame_config.head_ee_frame.parent_joint_name),
                    pinocchio.SE3(tracking_frame_config.head_ee_frame.parent_T_frame[:3, :3], tracking_frame_config.head_ee_frame.parent_T_frame[:3, 3]),
                    pinocchio.FrameType.OP_FRAME
                )
            )

        if tracking_frame_config.extra_tracking_frames != {} and tracking_frame_config.extra_tracking_frames is not None:
            for name, tracking_frame in tracking_frame_config.extra_tracking_frames.items():
                if not self.robot_.model.existJointName(tracking_frame.parent_joint_name):
                    raise ValueError(f"Extra tracking frame {name} parent joint {tracking_frame.parent_joint_name} does not exist in the robot model.")
                self.robot_.model.addFrame(
                    pinocchio.Frame(
                        tracking_frame.frame_name,
                        self.robot_.model.getJointId(tracking_frame.parent_joint_name),
                        pinocchio.SE3(tracking_frame.parent_T_frame[:3, :3], tracking_frame.parent_T_frame[:3, 3]),
                        pinocchio.FrameType.OP_FRAME
                    )
                )

        self.robot_.data = self.robot_.model.createData()

        self._update_tracking_frame_id(tracking_frame_config)

    def _update_tracking_frame_id(self, tracking_frame_config: TrackingFrameConfig):
        if tracking_frame_config.left_ee_frame is not None:
            self.left_arm_tracking_id = self.robot_.model.getFrameId(tracking_frame_config.left_ee_frame.frame_name)
        else:
            self.left_arm_tracking_id = None
        if tracking_frame_config.right_ee_frame is not None:
            self.right_arm_tracking_id = self.robot_.model.getFrameId(tracking_frame_config.right_ee_frame.frame_name)
        else:
            self.right_arm_tracking_id = None
        if tracking_frame_config.head_ee_frame is not None:
            self.head_tracking_id = self.robot_.model.getFrameId(tracking_frame_config.head_ee_frame.frame_name)
        else:
            self.head_tracking_id = None

        self.extra_tracking_ids = {}
        if self.tracking_frame_config.extra_tracking_frames != {} and self.tracking_frame_config.extra_tracking_frames is not None:
            for name, tracking_frame in self.tracking_frame_config.extra_tracking_frames.items():
                self.extra_tracking_ids[name] = self.robot_.model.getFrameId(tracking_frame.frame_name)
        else:
            self.extra_tracking_ids = None

    def _create_symbolic_model(self):
        self.cmodel = pin_casadi.Model(self.robot_.model)
        self.cdata = self.cmodel.createData()
        self.cq = casadi.SX.sym("q", self.robot_.model.nq, 1)

        pin_casadi.framesForwardKinematics(self.cmodel, self.cdata, self.cq)
        pin_casadi.centerOfMass(self.cmodel, self.cdata, self.cq)

    @property
    def collision_sphere_model_enabled(self) -> bool:
        return self.collision_sphere_model_config is not None

    @property
    def torso_collision_enabled(self) -> bool:
        return self.torso_sdf_collision_config is not None

    @property
    def inter_arm_collision_enabled(self) -> bool:
        return self.inter_arm_collision_config is not None

    def _validate_body_frame(self, frame_name: str):
        model = self.robot_.model
        if not model.existFrame(frame_name):
            raise ValueError(
                f"collision frame {frame_name!r} does not exist in the Pinocchio model"
            )
        frame_id = model.getFrameId(frame_name)
        if model.frames[frame_id].type != pinocchio.FrameType.BODY:
            raise ValueError(
                f"collision frame {frame_name!r} must be a BODY/link frame, "
                f"got {model.frames[frame_id].type}"
            )

    def _resolve_collision_spheres(
            self,
            sphere_names: Iterable[str],
            owner: str) -> tuple[tuple[CollisionSphere, ...], tuple[int, ...]]:
        if not self.collision_sphere_model_enabled:
            raise ValueError(
                f"{owner} requires collision_sphere_model_config"
            )
        names = tuple(sphere_names)
        missing_names = [
            name for name in names
            if name not in self.collision_sphere_name_to_index
        ]
        if missing_names:
            raise ValueError(
                f"{owner} references unknown collision spheres: {missing_names}"
            )
        indices = tuple(
            self.collision_sphere_name_to_index[name] for name in names
        )
        spheres = tuple(self.collision_spheres[index] for index in indices)
        return spheres, indices

    def _initialize_collision_spheres(self):
        """Build shared collision-sphere kinematics independently of consumers."""

        if not self.collision_sphere_model_enabled:
            if self.torso_collision_enabled or self.inter_arm_collision_enabled:
                raise ValueError(
                    "collision_sphere_model_config is required when collision "
                    "costs are configured"
                )
            return

        config = self.collision_sphere_model_config
        self.collision_spheres = config.spheres
        self.collision_sphere_name_to_index = {
            sphere.name: index
            for index, sphere in enumerate(self.collision_spheres)
        }

        required_frames = []
        required_frames.extend(
            sphere.frame_name for sphere in self.collision_spheres
        )
        for frame_name in dict.fromkeys(required_frames):
            self._validate_body_frame(frame_name)

        centers_world = []
        for sphere in self.collision_spheres:
            frame_id = self.cmodel.getFrameId(sphere.frame_name)
            center_link = casadi.SX(list(sphere.center))
            center_world = self.cdata.oMf[frame_id] * center_link
            centers_world.append(center_world)

        centers_world_expression = casadi.horzcat(*centers_world)

        self._collision_sphere_centers_world_expression = centers_world_expression

    def _initialize_torso_collision(self):
        """Build differentiable sphere/SDF functions once during construction."""

        if not self.torso_collision_enabled:
            return

        config = self.torso_sdf_collision_config
        sphere_names = (
            config.sphere_names
            if config.sphere_names
            else self.collision_sphere_model_config.sphere_names
        )
        self.torso_collision_spheres, sphere_indices = (
            self._resolve_collision_spheres(sphere_names, "torso collision")
        )
        sdf_path = os.fspath(config.sdf_path)
        if not os.path.isabs(sdf_path):
            sdf_path = os.path.join(self.urdf_dir, sdf_path)
        self.torso_sdf_map = load_sdf_map(sdf_path)

        self._validate_body_frame(config.torso_frame_name)

        boundary_clearance = (
            float(self.torso_sdf_map.metadata["boundary_min_sdf"])
            - self.torso_sdf_map.interpolation_error_bound
        )
        required_boundary_clearance = max(
            sphere.radius + config.safety_margin + config.activation_distance
            for sphere in self.torso_collision_spheres
        )
        if boundary_clearance <= required_boundary_clearance:
            raise ValueError(
                "torso SDF boundary is too close for the configured spheres: "
                f"minimum conservative boundary SDF={boundary_clearance:.6g}, "
                f"required > {required_boundary_clearance:.6g} (maximum sphere "
                "radius + safety margin + activation distance)"
            )

        self.torso_sdf_function = create_casadi_sdf_function(
            self.torso_sdf_map,
            name="torso_collision_sdf",
            conservative=True,
        )

        torso_frame_id = self.cmodel.getFrameId(config.torso_frame_name)
        world_M_torso = self.cdata.oMf[torso_frame_id]
        torso_R_sdf = casadi.SX(config.torso_T_sdf[:3, :3])
        torso_p_sdf = casadi.SX(config.torso_T_sdf[:3, 3])

        centers_sdf = []
        for sphere_index in sphere_indices:
            center_world = self._collision_sphere_centers_world_expression[
                :, sphere_index
            ]
            center_torso = world_M_torso.inverse() * center_world
            # torso_T_sdf maps SDF coordinates into torso coordinates.
            center_sdf = torso_R_sdf.T @ (center_torso - torso_p_sdf)
            centers_sdf.append(center_sdf)

        centers_sdf_expression = casadi.horzcat(*centers_sdf)

        sphere_count = len(self.torso_collision_spheres)
        mapped_sdf = self.torso_sdf_function.map(sphere_count, "serial")
        sdf_values_expression = mapped_sdf(centers_sdf_expression).T
        radii = casadi.SX([
            sphere.radius for sphere in self.torso_collision_spheres
        ])
        clearances_expression = (
            sdf_values_expression - radii - config.safety_margin
        )

        activated = config.activation_distance - clearances_expression
        smooth_violation = 0.5 * (
            activated
            + casadi.sqrt(activated**2 + config.smooth_epsilon**2)
        )
        normalized_violation = smooth_violation / config.activation_distance
        sphere_weights = casadi.SX([
            sphere.weight for sphere in self.torso_collision_spheres
        ])
        collision_cost_expression = casadi.dot(
            sphere_weights, normalized_violation**2
        )

        self.torso_collision_clearance_function = casadi.Function(
            "torso_collision_clearances",
            [self.cq],
            [clearances_expression],
            ["q"],
            ["clearances"],
        )

        self.torso_collision_cost_function = casadi.Function(
            "torso_collision_cost",
            [self.cq],
            [collision_cost_expression],
            ["q"],
            ["cost"],
        )

    @staticmethod
    def _sphere_pair_soft_collision_terms(
            squared_distance: CasadiExpr,
            contact_distance: float,
            activation_distance: float,
            smooth_epsilon: float) -> tuple[CasadiExpr, CasadiExpr, CasadiExpr]:
        """Return separation, activation, and normalized smooth violations.

        All three expressions avoid ``sqrt(squared_distance)``.  The first two
        retain metre-like scaling at their respective zero crossings, while the
        normalized violation is approximately one at sphere contact.
        """

        activation_center_distance = contact_distance + activation_distance
        # separation_residual = (
        #     squared_distance - contact_distance**2
        # ) / (2.0 * contact_distance)
        activation_residual = (
            activation_center_distance**2 - squared_distance
        ) / (2.0 * activation_center_distance)
        contact_activation_residual = (
            activation_center_distance**2 - contact_distance**2
        ) / (2.0 * activation_center_distance)
        smooth_violation = 0.5 * (
            activation_residual
            + casadi.sqrt(activation_residual**2 + smooth_epsilon**2)
        )
        normalized_violation = smooth_violation / contact_activation_residual
        return normalized_violation


    def _initialize_inter_arm_collision(self):
        """Build a smooth, sqrt-free cost for every left/right sphere pair."""

        if not self.inter_arm_collision_enabled:
            return

        config = self.inter_arm_collision_config
        left_spheres, left_indices = self._resolve_collision_spheres(
            config.left_sphere_names, "inter-arm collision left group"
        )
        right_spheres, right_indices = self._resolve_collision_spheres(
            config.right_sphere_names, "inter-arm collision right group"
        )

        normalized_violations = []
        pair_weights = []
        for left_sphere, left_index in zip(left_spheres, left_indices):
            left_center = self._collision_sphere_centers_world_expression[
                :, left_index
            ]
            for right_sphere, right_index in zip(right_spheres, right_indices):
                right_center = self._collision_sphere_centers_world_expression[
                    :, right_index
                ]
                delta = left_center - right_center
                squared_distance = casadi.dot(delta, delta)

                contact_distance = (
                    left_sphere.radius
                    + right_sphere.radius
                    + config.safety_margin
                )
                normalized_violation = WholeBodyIK._sphere_pair_soft_collision_terms(
                    squared_distance=squared_distance,
                    contact_distance=contact_distance,
                    activation_distance=config.activation_distance,
                    smooth_epsilon=config.smooth_epsilon,
                )

                normalized_violations.append(normalized_violation)
                pair_weights.append(
                    np.sqrt(left_sphere.weight * right_sphere.weight)
                )

        normalized_violations_expression = casadi.vertcat(*normalized_violations)
        pair_weights_expression = casadi.SX(pair_weights)
        collision_cost_expression = casadi.dot(
            pair_weights_expression, normalized_violations_expression**2
        )

        self.inter_arm_collision_cost_function = casadi.Function(
            "inter_arm_collision_cost",
            [self.cq],
            [collision_cost_expression],
            ["q"],
            ["cost"],
        )


    def _register_default_problems(self):
        self.register_problem("standard", self._build_standard_problem)

    def register_problem(self, name: str, builder: ProblemBuilder):
        if name in self.problem_builders:
            raise ValueError(f"Problem {name} already exists.")
        self.problem_builders[name] = builder
        self.supported_problems = list(self.problem_builders.keys())

    def add_customize_problem(self, name: str, problem_builder: ProblemBuilder):
        if name in self.problem_builders:
            self.logger.error(f"Problem builder {name} already exists.")
            return False
        self.problem_builders[name] = problem_builder
        self.supported_problems = list(self.problem_builders.keys())
        return True

    def add_problem_overlay(self,
                            name: str,
                            costs: List[IKCostTerm] = None,
                            constraints: List[IKConstraint] = None):
        if name not in self.problem_builders:
            self.logger.error(f"Base problem {name} does not exist, cannot add overlay.")
            return False
        overlay = self.problem_overlays.setdefault(name, IKProblemSpec(name=name))
        if costs is not None:
            overlay.costs.extend(costs)
        if constraints is not None:
            overlay.constraints.extend(constraints)
        return True

    def _build_standard_problem(self, context: IKBuildContext) -> IKProblemSpec:
        translation_cost = self._translational_tracking_cost(context)
        rotation_cost = self._rotational_tracking_cost(context)
        # ik_problem_spec = IKProblemSpec(
        #     name="standard",
        #     costs=[
        #         IKCostTerm(
        #             name="translational_tracking_cost",
        #             # expression=self._translational_tracking_cost,
        #             expression=lambda context: translation_cost,
        #             weight=self.weight_config.translation
        #         ),
        #         IKCostTerm(
        #             name="rotational_tracking_cost",
        #             # expression=self._rotational_tracking_cost,
        #             expression=lambda context: rotation_cost,
        #             weight=self.weight_config.rotation
        #         ),
        #         self._regularization_cost(context),
        #         self._smooth_cost(context),
        #         self._upper_body_leaning_cost(context),
        #         self._leg_heaviness_cost(context),
        #         self._shoulder_angle_barrier_cost(context),
        #         self._eye_focus_point_cost(context)
        #     ],
        #     constraints=[
        #         self._joint_limit_constraint(context, margin_ratio=self.posture_config.joint_limits_contract_ratio),
        #         self._upper_body_leaning_constraint(context),
        #         self._buttock_upright_constraint(context),
        #         self._leg_leaning_constraint(context, is_leaning_forward=self.posture_config.leg_leaning_forward),
        #         self._elbow_inward_constraint(context),
        #         self._com_constraint(context),
        #         self._strict_tracking_constraint(context, translation_cost, rotation_cost)
        #     ]
        # )
        costs = [
            IKCostTerm(
                name="translational_tracking_cost",
                expression=lambda context: translation_cost,
                weight=self.weight_config.translation
            ),
            IKCostTerm(
                name="rotational_tracking_cost",
                expression=lambda context: rotation_cost,
                weight=self.weight_config.rotation
            ),
            self._regularization_cost(context),
            self._smooth_cost(context),
            self._upper_body_leaning_cost(context),
            self._leg_heaviness_cost(context),
            self._shoulder_angle_barrier_cost(context),
            self._eye_focus_point_cost(context)
        ]
        constraints = [
            self._joint_limit_constraint(context, margin_ratio=self.posture_config.joint_limits_contract_ratio),
            self._upper_body_leaning_constraint(context),
            self._buttock_upright_constraint(context),
            self._leg_leaning_constraint(context, is_leaning_forward=self.posture_config.leg_leaning_forward),
            self._elbow_inward_constraint(context),
            self._com_constraint(context),
            self._strict_tracking_constraint(context, translation_cost, rotation_cost)
        ]
        if self.torso_collision_enabled:
            costs.append(self._torso_collision_cost(context))
            if self.torso_sdf_collision_config.hard_constraint:
                constraints.append(self._torso_collision_constraint(context))
        if self.inter_arm_collision_enabled:
            costs.append(self._inter_arm_collision_cost(context))

        ik_problem_spec = IKProblemSpec(
            name="standard",
            costs=costs,
            constraints=constraints,
        )
        return ik_problem_spec

    def _bind_q(self, name: str, expr: CasadiExpr, q: CasadiExpr):
        func = casadi.Function(
            f"{name}",
            [self.cq],
            [expr]
        )
        return func(q)

    def joint_q_or_locked_value(self, context: IKBuildContext, joint_name: str):
        idx = self.joint_index_resolver.q_index(joint_name)

        if idx is not None:
            return JointQType.VARIABLE, context.q[idx]
        else:
            if self.freezed_joint_dict is not None and joint_name in self.freezed_joint_dict:
                return JointQType.LOCKED, self.freezed_joint_dict[joint_name]
            else:
                return JointQType.MISSING, None

    def joint_q_last_or_locked_value(self, context: IKBuildContext, joint_name: str):
        idx = self.joint_index_resolver.q_index(joint_name)

        if idx is not None:
            return JointQType.VARIABLE, context.q_last[idx]
        else:
            if self.freezed_joint_dict is not None and joint_name in self.freezed_joint_dict:
                return JointQType.LOCKED, self.freezed_joint_dict[joint_name]
            else:
                return JointQType.MISSING, None

    @staticmethod
    def _upper_side_barrier(q: CasadiExpr, upper_bound: float, gain: float = 50.0):
        return 1 / (1 + casadi.exp(-gain*(q - upper_bound)))

    @staticmethod
    def _lower_side_barrier(q: CasadiExpr, lower_bound: float, gain: float = 50.0):
        return 1 / (1 + casadi.exp(-gain*(lower_bound - q)))

    @staticmethod
    def _two_sided_barrier(q: CasadiExpr, lower_bound: float, upper_bound: float, gain: float = 50.0):
        return WholeBodyIK._upper_side_barrier(q, upper_bound, gain) * WholeBodyIK._lower_side_barrier(q, lower_bound, gain)

    def _translational_tracking_cost(self, context: IKBuildContext) -> CasadiExpr:
        parts = []
        if self.left_arm_tracking_id is not None:
            parts.append(
                self._bind_q("left_arm_translation_func", self.cdata.oMf[self.left_arm_tracking_id].translation, context.q) - context.left_arm_target[:3, 3]
            )
        if self.right_arm_tracking_id is not None:
            parts.append(
                self._bind_q("right_arm_translation_func", self.cdata.oMf[self.right_arm_tracking_id].translation, context.q) - context.right_arm_target[:3, 3]
            )

        if not parts:
            self.logger.warning("No tracking frames available for translational tracking cost.")
            return casadi.MX(0)

        return casadi.sumsqr(casadi.vertcat(*parts))

    def _rotational_tracking_cost(self, context: IKBuildContext) -> CasadiExpr:
        parts = []
        if self.left_arm_tracking_id is not None:
            left_arm_current = self.cdata.oMf[self.left_arm_tracking_id].rotation
            left_arm_current = self._bind_q("left_arm_rotation_func", left_arm_current, context.q)
            left_arm_target = context.left_arm_target[:3, :3]
            left_arm_rotation_error = [
                left_arm_current[:, 0] - left_arm_target[:, 0],
                left_arm_current[:, 1] - left_arm_target[:, 1],
                left_arm_current[:, 2] - left_arm_target[:, 2]
            ]
            parts.extend(left_arm_rotation_error)
        if self.right_arm_tracking_id is not None:
            right_arm_current = self.cdata.oMf[self.right_arm_tracking_id].rotation
            right_arm_current = self._bind_q("right_arm_rotation_func", right_arm_current, context.q)
            right_arm_target = context.right_arm_target[:3, :3]
            right_arm_rotation_error = [
                right_arm_current[:, 0] - right_arm_target[:, 0],
                right_arm_current[:, 1] - right_arm_target[:, 1],
                right_arm_current[:, 2] - right_arm_target[:, 2]
            ]
            parts.extend(right_arm_rotation_error)
        if not parts:
            self.logger.warning("No tracking frames available for rotational tracking cost.")
            return casadi.MX(0)
        return casadi.sumsqr(casadi.vertcat(*parts))

    def _regularization_cost(self, context: IKBuildContext) -> CasadiExpr:
        def cost_builder(context: IKBuildContext) -> CasadiExpr:
            waist_type, waist_q = self.joint_q_or_locked_value(context, self.joint_name_config.waist_joint)
            if waist_q is None:
                self.logger.warning("Waist joint is not available, skipping regularization cost.")
                return 0.0
            return 10 * casadi.sumsqr(waist_q)

        return IKCostTerm(
            name=f"regularization_cost",
            expression=cost_builder,
            weight=self.weight_config.regularization
        )

    def _smooth_cost(self, context: IKBuildContext) -> IKCostTerm:
        def cost_builder(context: IKBuildContext) -> CasadiExpr:
            return casadi.sumsqr(context.q - context.q_last)

        return IKCostTerm(
            name=f"smooth_cost",
            expression=cost_builder,
            weight=self.weight_config.smoothness
        )

    def _upper_body_leaning_cost(self, context: IKBuildContext) -> IKCostTerm:
        def cost_builder(context: IKBuildContext) -> CasadiExpr:
            ankle_type, ankle_q = self.joint_q_or_locked_value(context, self.joint_name_config.ankle_joint)
            knee_type, knee_q = self.joint_q_or_locked_value(context, self.joint_name_config.knee_joint)
            buttock_type, buttock_q = self.joint_q_or_locked_value(context, self.joint_name_config.buttock_joint)
            if any(q is None for q in [ankle_q, knee_q, buttock_q]):
                self.logger.warning("One of the leg joints is not available, skipping upper body leaning cost.")
                return 0.0
            upper_body_leaning = ankle_q + knee_q + buttock_q
            if self.left_arm_tracking_id is not None:
                left_target_x = context.left_arm_target[0, 3]
            else:
                left_target_x = 0.0
            if self.right_arm_tracking_id is not None:
                right_target_x = context.right_arm_target[0, 3]
            else:
                right_target_x = 0.0
            # upper_body_leaning_weight = WholeBodyIK._lower_side_barrier((context.left_arm_target[0, 3] + context.right_arm_target[0, 3])/2, lower_bound=self.posture_config.dual_ee_flange_centerX_threshold, gain=50)
            upper_body_leaning_weight = WholeBodyIK._lower_side_barrier((left_target_x + right_target_x)/2, lower_bound=self.posture_config.dual_ee_flange_centerX_threshold, gain=50)

            # Keep the upright preference at full strength when every collision
            # sphere is outside the activation band.  As soon as any sphere
            # enters that band, smoothly reduce (but never remove) this cost so
            # whole-body redundancy can be used for avoidance.
            collision_posture_scale = 1.0
            if self.torso_collision_enabled:
                clearances = self.torso_collision_clearance_function(context.q)
                activation_distance = self.torso_sdf_collision_config.activation_distance
                transition_width = 0.1 * activation_distance
                sphere_activations = 0.5 * (
                    1.0
                    + casadi.tanh(
                        (activation_distance - clearances) / transition_width
                    )
                )

                # Smooth OR: zero when all spheres are inactive and approaches
                # one when at least one sphere is well inside the activation band.
                all_spheres_inactive = 1.0
                for sphere_index in range(len(self.torso_collision_spheres)):
                    all_spheres_inactive *= 1.0 - sphere_activations[sphere_index]
                collision_activation = 1.0 - all_spheres_inactive

                minimum_posture_scale = 0.1
                collision_posture_scale = (
                    1.0
                    - (1.0 - minimum_posture_scale) * collision_activation
                )

            return (
                upper_body_leaning_weight
                * collision_posture_scale
                * casadi.sumsqr(
                    upper_body_leaning - context.upper_body_leaning_threshold
                )
            )

            # return upper_body_leaning_weight * casadi.sumsqr(upper_body_leaning - context.upper_body_leaning_threshold)

        return IKCostTerm(
            name=f"upper_body_leaning_cost",
            expression=cost_builder,
            weight=self.weight_config.upper_body_straightness
        )

    def _leg_heaviness_cost(self, context: IKBuildContext) -> IKCostTerm:
        def cost_builder(context: IKBuildContext) -> CasadiExpr:
            ankle_type, ankle_q = self.joint_q_or_locked_value(context, self.joint_name_config.ankle_joint)
            knee_type, knee_q = self.joint_q_or_locked_value(context, self.joint_name_config.knee_joint)
            buttock_type, buttock_q = self.joint_q_or_locked_value(context, self.joint_name_config.buttock_joint)
            ankle_last_type, ankle_q_last = self.joint_q_last_or_locked_value(context, self.joint_name_config.ankle_joint)
            knee_last_type, knee_q_last = self.joint_q_last_or_locked_value(context, self.joint_name_config.knee_joint)
            buttock_last_type, buttock_q_last = self.joint_q_last_or_locked_value(context, self.joint_name_config.buttock_joint)
            if any(q is None for q in [ankle_q, knee_q, buttock_q, ankle_q_last, knee_q_last, buttock_q_last]):
                self.logger.warning("One of the leg joints is not available, skipping leg heaviness cost.")
                return 0.0
            leg_heaviness = casadi.sumsqr(ankle_q - ankle_q_last) + casadi.sumsqr(knee_q - knee_q_last) + casadi.sumsqr(buttock_q - buttock_q_last)
            return leg_heaviness

        return IKCostTerm(
            name=f"leg_heaviness_cost",
            expression=cost_builder,
            weight=self.weight_config.leg_heaviness
        )

    def _j2_j1_dynamic_lower_bound(self, j1_jps, j1_low, j1_high, j2_low, j2_high):
        ratio = (j1_jps - j1_low) / (j1_high - j1_low)
        ratio = casadi.fmax(0.0, casadi.fmin(1.0, ratio))
        return j2_low + ratio * (j2_high - j2_low)

    def _shoulder_angle_barrier_cost(self, context: IKBuildContext) -> IKCostTerm:
        def cost_builder(context: IKBuildContext) -> CasadiExpr:
            left_shoulder_j1_type, left_shoulder_j1 = self.joint_q_or_locked_value(context, self.joint_name_config.left_arm_joints[0])
            left_shoulder_j2_type, left_shoulder_j2 = self.joint_q_or_locked_value(context, self.joint_name_config.left_arm_joints[1])
            right_shoulder_j1_type, right_shoulder_j1 = self.joint_q_or_locked_value(context, self.joint_name_config.right_arm_joints[0])
            right_shoulder_j2_type, right_shoulder_j2 = self.joint_q_or_locked_value(context, self.joint_name_config.right_arm_joints[1])
            costs = []
            if left_shoulder_j1 is not None and left_shoulder_j2 is not None:
                left_j2_lower_bound = self._j2_j1_dynamic_lower_bound(left_shoulder_j1,
                                                                      self.posture_config.left_arm_j1_dynamic_knots[0],
                                                                      self.posture_config.left_arm_j1_dynamic_knots[1],
                                                                      self.posture_config.left_arm_j2_dynamic_knots[0],
                                                                      self.posture_config.left_arm_j2_dynamic_knots[1])

                costs.append(self._lower_side_barrier(left_shoulder_j2, lower_bound=left_j2_lower_bound, gain=25))
            if right_shoulder_j1 is not None and right_shoulder_j2 is not None:
                right_j2_lower_bound = self._j2_j1_dynamic_lower_bound(right_shoulder_j1,
                                                                       self.posture_config.right_arm_j1_dynamic_knots[0],
                                                                       self.posture_config.right_arm_j1_dynamic_knots[1],
                                                                       self.posture_config.right_arm_j2_dynamic_knots[0],
                                                                       self.posture_config.right_arm_j2_dynamic_knots[1])
                costs.append(self._upper_side_barrier(right_shoulder_j2, upper_bound=right_j2_lower_bound, gain=25))
            if costs:
                return casadi.sumsqr(casadi.vertcat(*costs))
            else:
                return 0.0

        return IKCostTerm(
            name=f"shoulder_angle_barrier_cost",
            expression=cost_builder,
            weight=self.weight_config.shoulder_angle_barrier
        )

    def _eye_focus_point_cost(self, context: IKBuildContext) -> IKCostTerm:
        def cost_builder(context: IKBuildContext) -> CasadiExpr:
            if self.head_tracking_id is None:
                self.logger.warning("Head tracking frame is not available, skipping eye focus point cost.")
                return 0.0
            eye_position = self._bind_q("eye_position_func", self.cdata.oMf[self.head_tracking_id].translation, context.q)
            focus_direction = context.eye_focus_point - eye_position
            focus_direction = focus_direction / (casadi.norm_2(focus_direction) + 1e-6)
            eye_forward = self._bind_q("eye_forward_func", -self.cdata.oMf[self.head_tracking_id].rotation[:, 2], context.q)
            eye_focus_cost = 10 * context.eye_focus_weight * (1 - casadi.dot(focus_direction, eye_forward))
            return casadi.sumsqr(eye_focus_cost)

        return IKCostTerm(
            name=f"eye_focus_point_cost",
            expression=cost_builder,
            weight=self.weight_config.eye_focus_point
        )

    def _joint_limit_constraint(self, context: IKBuildContext, margin_ratio: float) -> IKConstraint:
        def constraint_builder(context: IKBuildContext) -> Iterable[CasadiExpr]:
            constraints = []
            lower = context.model.lowerPositionLimit
            uppper = context.model.upperPositionLimit
            margin = (uppper - lower) * margin_ratio
            constraints.append(
                context.opti.bounded(lower + margin, context.q, uppper - margin)
            )
            return constraints

        return IKConstraint(
            name=f"joint_limit_constraint",
            expressions=constraint_builder
        )

    def _upper_body_leaning_constraint(self, context: IKBuildContext) -> IKConstraint:
        def constraint_builder(context: IKBuildContext) -> Iterable[CasadiExpr]:
            constraints = []
            ankle_type, ankle_q = self.joint_q_or_locked_value(context, self.joint_name_config.ankle_joint)
            knee_type, knee_q = self.joint_q_or_locked_value(context, self.joint_name_config.knee_joint)
            buttock_type, buttock_q = self.joint_q_or_locked_value(context, self.joint_name_config.buttock_joint)
            if any(q is None for q in [ankle_q, knee_q, buttock_q]):
                self.logger.warning("One of the leg joints is not available, skipping upper body leaning constraint.")
                return []
            if ankle_type == JointQType.LOCKED and knee_type == JointQType.LOCKED and buttock_type == JointQType.LOCKED:
                self.logger.warning("All leg joints are locked, skipping upper body leaning constraint.")
                return []
            upper_body_leaning = ankle_q + knee_q + buttock_q
            # if context.upper_body_leaning_threshold is not None:
            #     constraints.append(upper_body_leaning >= context.upper_body_leaning_threshold)
            # else:
            #     constraints.append(upper_body_leaning >= 0.0)
            maximum_backward_lean = np.deg2rad(30.0)
            constraints.append(upper_body_leaning >= -maximum_backward_lean)

            return constraints

        return IKConstraint(
            name=f"upper_body_leaning_constraint",
            expressions=constraint_builder
        )

    def _buttock_upright_constraint(self, context: IKBuildContext) -> IKConstraint:
        def constraint_build(context: IKBuildContext) -> Iterable[CasadiExpr]:
            constraints = []
            ankle_type, ankle_q = self.joint_q_or_locked_value(context, self.joint_name_config.ankle_joint)
            knee_type, knee_q = self.joint_q_or_locked_value(context, self.joint_name_config.knee_joint)
            if any(q is None for q in [ankle_q, knee_q]):
                self.logger.warning("Buttock joint is not available, skipping buttock upright constraint.")
                return []
            if ankle_type == JointQType.LOCKED and knee_type == JointQType.LOCKED:
                self.logger.warning("Both ankle and knee joints are locked, skipping buttock upright constraint.")
                return []
            constraints.append(ankle_q + 0.5 * knee_q == 0.0)
            return constraints

        return IKConstraint(
            name=f"buttock_upright_constraint",
            expressions=constraint_build
        )

    def _leg_leaning_constraint(self, context: IKBuildContext, is_leaning_forward: bool) -> IKConstraint:
        def constraint_build(context: IKBuildContext) -> Iterable[CasadiExpr]:
            constraints = []
            ankle_type, ankle_q = self.joint_q_or_locked_value(context, self.joint_name_config.ankle_joint)
            knee_type, knee_q = self.joint_q_or_locked_value(context, self.joint_name_config.knee_joint)
            if any(q is None for q in [ankle_q, knee_q]):
                self.logger.warning("One of the leg joints is not available, skipping leg leaning constraint.")
                return []
            if is_leaning_forward:
                if ankle_type == JointQType.VARIABLE:
                    constraints.append(ankle_q >= 0.0)
                if knee_type == JointQType.VARIABLE:
                    constraints.append(knee_q <= 0.0)
            else:
                if ankle_type == JointQType.VARIABLE:
                    constraints.append(ankle_q <= 0.0)
                if knee_type == JointQType.VARIABLE:
                    constraints.append(knee_q >= 0.0)
            return constraints

        return IKConstraint(
            name=f"leg_leaning_constraint",
            expressions=constraint_build
        )

    def _elbow_inward_constraint(self, context: IKBuildContext) -> IKConstraint:
        def constraint_build(context: IKBuildContext) -> Iterable[CasadiExpr]:
            constraints = []
            left_elbow_j4_type, left_elbow_j4 = self.joint_q_or_locked_value(context, self.joint_name_config.left_arm_joints[3])
            right_elbow_j4_type, right_elbow_j4 = self.joint_q_or_locked_value(context, self.joint_name_config.right_arm_joints[3])
            if any(q is None for q in [left_elbow_j4, right_elbow_j4]):
                self.logger.warning("Shoulder roll joint is not available, skipping elbow inward constraint.")
                return []
            if left_elbow_j4_type == JointQType.VARIABLE:
                constraints.append(left_elbow_j4 <= 0.0)
            if right_elbow_j4_type == JointQType.VARIABLE:
                constraints.append(right_elbow_j4 >= 0.0)
            return constraints

        return IKConstraint(
            name=f"elbow_inward_constraint",
            expressions=constraint_build
        )

    def _com_constraint(self, context: IKBuildContext) -> IKConstraint:
        def constraint_build(context: IKBuildContext) -> Iterable[CasadiExpr]:
            constraints = []
            base_frame_id = self.cmodel.getFrameId("base_link")
            world_T_base_link = self.cdata.oMf[base_frame_id]
            com_base = world_T_base_link.inverse() * self.cdata.com[0]
            com = self._bind_q("com_func", com_base, context.q)
            constraints.append(com[0] >= self.posture_config.com_x_bound[0])
            constraints.append(com[0] <= self.posture_config.com_x_bound[1])
            constraints.append(com[1] >= self.posture_config.com_y_bound[0])
            constraints.append(com[1] <= self.posture_config.com_y_bound[1])
            return constraints

        return IKConstraint(
            name=f"com_constraint",
            expressions=constraint_build
        )

    def _strict_tracking_constraint(self, context: IKBuildContext, translation_cost: CasadiExpr, rotation_cost: CasadiExpr) -> IKConstraint:
        def constraint_build(context: IKBuildContext) -> Iterable[CasadiExpr]:
            constraints = []
            if context.translation_error_threshold is not None:
                constraints.append(translation_cost <= context.translation_error_threshold)
            if context.rotation_error_threshold is not None:
                constraints.append(rotation_cost <= context.rotation_error_threshold)
            return constraints

        return IKConstraint(
            name=f"strict_tracking_constraint",
            expressions=constraint_build
        )

    def _torso_collision_cost(self, context: IKBuildContext) -> IKCostTerm:
        return IKCostTerm(
            name="torso_collision_cost",
            expression=lambda build_context: self.torso_collision_cost_function(
                build_context.q
            ),
            weight=self.torso_sdf_collision_config.cost_weight,
        )

    def _torso_collision_constraint(self, context: IKBuildContext) -> IKConstraint:
        return IKConstraint(
            name="torso_collision_constraint",
            expressions=lambda build_context: [
                self.torso_collision_clearance_function(build_context.q) >= 0.0
            ],
        )

    def _inter_arm_collision_cost(self, context: IKBuildContext) -> IKCostTerm:
        return IKCostTerm(
            name="inter_arm_collision_cost",
            expression=lambda build_context: self.inter_arm_collision_cost_function(
                build_context.q
            ),
            weight=self.inter_arm_collision_config.cost_weight,
        )

    def set_problem(self, problem_name: str = "standard", extra_parameter_shapes: Dict[str, tuple[int, ...]] = None):
        if problem_name not in self.problem_builders:
            self.logger.error(f"Problem {problem_name} is not registered. Supported problems are: {self.supported_problems}")
            raise ValueError(f"Problem {problem_name} is not registered. Supported problems are: {self.supported_problems}")

        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.robot_.model.nq)
        self.var_q_last = self.opti.parameter(self.robot_.model.nq)
        if self.left_arm_tracking_id is not None:
            self.param_left_arm_tf = self.opti.parameter(4, 4)
        else:
            self.param_left_arm_tf = None
        if self.right_arm_tracking_id is not None:
            self.param_right_arm_tf = self.opti.parameter(4, 4)
        else:
            self.param_right_arm_tf = None
        if self.head_tracking_id is not None:
            self.param_head_tf = self.opti.parameter(4, 4)
        else:
            self.param_head_tf = None

        self.param_upper_body_leaning_threshold = self.opti.parameter()
        self.param_eye_focus_point = self.opti.parameter(3, 1)
        self.param_eye_focus_weight = self.opti.parameter()
        self.param_translation_error_threshold = self.opti.parameter()
        self.param_rotation_error_threshold = self.opti.parameter()

        self.param_extra_tracking_tfs = {}
        if self.extra_tracking_ids:
            for name in self.extra_tracking_ids.keys():
                self.param_extra_tracking_tfs[name] = self.opti.parameter(4, 4)

        self.extra_parameter_specs = {}
        if extra_parameter_shapes is not None:
            for name, shape in extra_parameter_shapes.items():
                if shape == () or shape is None:
                    self.extra_parameter_specs[name] = self.opti.parameter()
                else:
                    self.extra_parameter_specs[name] = self.opti.parameter(*shape)

        self.opti.solver(
            self.solver_options.name,
            self.solver_options.to_casadi_options()
        )

        context = self._context()

        self.current_problem_spec = self.problem_builders[problem_name](context)
        self.current_problem_overlay = self.problem_overlays.get(problem_name, None)
        if self.current_problem_overlay is not None:
            self.current_problem_spec.costs.extend(self.current_problem_overlay.costs)
            self.current_problem_spec.constraints.extend(self.current_problem_overlay.constraints)

        total_cost = 0
        for term in self.current_problem_spec.costs:
            cost_expr = term.expression(context)
            total_cost += term.weight * cost_expr
        for constraint in self.current_problem_spec.constraints:
            constraint_exprs = constraint.expressions(context)
            for expr in constraint_exprs:
                self.opti.subject_to(expr)

        self.opti.minimize(total_cost)

    def _context(self) -> IKBuildContext:
        return IKBuildContext(
            solver=self,
            opti=self.opti,
            q=self.var_q,
            q_last=self.var_q_last,
            left_arm_target=self.param_left_arm_tf,
            right_arm_target=self.param_right_arm_tf,
            head_target=self.param_head_tf,
            upper_body_leaning_threshold=self.param_upper_body_leaning_threshold,
            eye_focus_point=self.param_eye_focus_point,
            eye_focus_weight=self.param_eye_focus_weight,
            translation_error_threshold=self.param_translation_error_threshold,
            rotation_error_threshold=self.param_rotation_error_threshold,
            extra_targets=self.param_extra_tracking_tfs,
            extra_contexts=self.extra_parameter_specs
        )

    def _pose_close(self, target: np.ndarray, cached: np.ndarray, trans_tol: float = 5e-4, rot_tol: float = np.deg2rad(1.0)):
        trans_err = np.linalg.norm(target[:3, 3] - cached[:3, 3])
        T_err = np.linalg.inv(target) @ cached
        rot_err = np.linalg.norm(pinocchio.log3(T_err[:3, :3]))
        return trans_err < trans_tol and rot_err < rot_tol

    def _check_same_targets(self, left_arm_target, right_arm_target, head_target=None, extra_targets=None):
        if self.cache_left_arm_target is None:
            if self.left_arm_tracking_id is not None:
                return False
        else:
            if self.left_arm_tracking_id is not None and not self._pose_close(left_arm_target, self.cache_left_arm_target):
                return False
        if self.cache_right_arm_target is None:
            if self.right_arm_tracking_id is not None:
                return False
        else:
            if self.right_arm_tracking_id is not None and not self._pose_close(right_arm_target, self.cache_right_arm_target):
                return False
        if self.cache_head_target is None:
            if head_target is not None:
                if self.head_tracking_id is not None:
                    return False
        else:
            if head_target is not None:
                if self.head_tracking_id is not None and not self._pose_close(head_target, self.cache_head_target):
                    return False
        if self.cache_extra_targets is None or self.cache_extra_targets == {}:
            if extra_targets is not None:
                if self.extra_tracking_ids is not None:
                    return False
        else:
            if extra_targets is not None:
                if self.extra_tracking_ids is not None:
                    for name, target in extra_targets.items():
                        if name in self.cache_extra_targets:
                            if not self._pose_close(target, self.cache_extra_targets[name]):
                                return False
                        else:
                            return False
        return True

    def solve_ik(
            self,
            left_arm_target: np.ndarray,
            right_arm_target: np.ndarray,
            head_target: np.ndarray = None,
            ref_qpos: np.ndarray = None,
            upper_body_leaning_threshold: float = None,
            eye_focus_point: np.ndarray = None,
            is_strict_solution: bool = False,
            extra_targets: Dict[str, np.ndarray] = None,
            extra_parameters: Dict[str, np.ndarray] = None,
            use_same_targets_filter: bool = False
    ):
        if self.left_arm_tracking_id is not None and left_arm_target is None:
            self.logger.error("Left arm tracking frame is registered, but no left arm target is provided.")
            return False, None
        if self.right_arm_tracking_id is not None and right_arm_target is None:
            self.logger.error("Right arm tracking frame is registered, but no right arm target is provided.")
            return False, None



        if ref_qpos is not None:
            if np.atleast_2d(ref_qpos).shape != (1, self.robot_.model.nq) and np.atleast_2d(ref_qpos).shape != (self.robot_.model.nq, 1):
                self.logger.error(f"Reference qpos has incorrect shape. Expected {self.robot_.model.nq}, got {np.atleast_2d(ref_qpos).shape[0]}.")
                return False, None
        init_data = deepcopy(ref_qpos) if ref_qpos is not None else np.zeros(self.robot_.model.nq)
        self.opti.set_initial(self.var_q, init_data)
        self.opti.set_value(self.var_q_last, init_data)

        if self.left_arm_tracking_id is not None:
            self.opti.set_value(self.param_left_arm_tf, left_arm_target)
        if self.right_arm_tracking_id is not None:
            self.opti.set_value(self.param_right_arm_tf, right_arm_target)
        if self.head_tracking_id is not None:
            if head_target is not None:
                self.opti.set_value(self.param_head_tf, head_target)

        if self.param_extra_tracking_tfs:
            if extra_targets is None:
                self.logger.error("Extra tracking frames are registered, but no extra targets are provided.")
                return False, None
            for name, param in self.param_extra_tracking_tfs.items():
                if name in extra_targets:
                    self.opti.set_value(param, extra_targets[name])
                else:
                    self.logger.error(f"Extra tracking frame {name}'s target is not provided.")
                    return False, None

        if self.extra_parameter_specs:
            if extra_parameters is None:
                self.logger.error("Extra parameters are registered, but no extra parameters are provided.")
                return False, None
            for name, param in self.extra_parameter_specs.items():
                if name in extra_parameters:
                    self.opti.set_value(param, extra_parameters[name])
                else:
                    self.logger.error(f"Extra parameter {name} is not provided.")
                    return False, None

        if use_same_targets_filter:
            if not is_strict_solution:
                if self.cache_solution_q is not None:
                    if self._check_same_targets(left_arm_target, right_arm_target, head_target, extra_targets):
                        self.logger.info("Targets are the same as previous solution, returning cached qpos.")
                        return True, self.cache_solution_q

        if upper_body_leaning_threshold is not None:
            self.opti.set_value(self.param_upper_body_leaning_threshold, upper_body_leaning_threshold)
        else:
            self.opti.set_value(self.param_upper_body_leaning_threshold, 0.0)

        if eye_focus_point is not None:
            eye_focus_point = np.asarray(eye_focus_point).reshape(3, 1)
            self.opti.set_value(self.param_eye_focus_point, eye_focus_point)
            self.opti.set_value(self.param_eye_focus_weight, 1.0)
        else:
            self.opti.set_value(self.param_eye_focus_point, np.array([[0.0, 0.0, 0.0]]))
            self.opti.set_value(self.param_eye_focus_weight, 0.0)

        if is_strict_solution:
            self.opti.set_value(self.param_translation_error_threshold, 1e-6)
            self.opti.set_value(self.param_rotation_error_threshold, 5e-5)
        else:
            self.opti.set_value(self.param_translation_error_threshold, 1e6)
            self.opti.set_value(self.param_rotation_error_threshold, 1e5)

        try:
            solution = self.opti.solve()
            solution_q = solution.value(self.var_q)
            if self.left_arm_tracking_id is not None:
                self.cache_left_arm_target = deepcopy(left_arm_target)
            if self.right_arm_tracking_id is not None:
                self.cache_right_arm_target = deepcopy(right_arm_target)
            if head_target is not None:
                if self.head_tracking_id is not None:
                    self.cache_head_target = deepcopy(head_target)
            if extra_targets is not None:
                if self.extra_tracking_ids is not None:
                    self.cache_extra_targets = {name: deepcopy(target) for name, target in extra_targets.items()}
            self.cache_solution_q = deepcopy(solution_q)
            return True, solution_q
        except Exception as e:
            self.logger.error(f"IK solver failed: {e}")

            return False, None
