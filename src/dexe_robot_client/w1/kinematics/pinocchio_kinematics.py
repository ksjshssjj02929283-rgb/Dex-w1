'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

import numpy as np
from pathlib import Path
import pinocchio
from dexe_rp import RobotVersion, EndEffectorType
from ...common import get_logger
from dataclasses import dataclass
import xml.etree.ElementTree as ET
from copy import deepcopy

from dexe_rp.toolkits.data import get_data_path
from dexe_rp.toolkits.urdf_assembly.type import MeshType
from dexe_rp.toolkits.urdf_assembly.ee_config import END_EFFECTORS_URDF_PATHS


@dataclass(frozen=True)
class MimicInfo:
    joint_name: str
    mimicked_joint_name: str
    multiplier: float
    offset: float


REAL_ROBOT_JOINT_ORDERS_V02X = [
    "ANKLE",
    "KNEE",
    "BUTTOCK",
    "WAIST",
    "NECK1",
    "NECK2",
    "LEFT_J1",
    "LEFT_J2",
    "LEFT_J3",
    "LEFT_J4",
    "LEFT_J5",
    "LEFT_J6",
    "LEFT_J7",
    "RIGHT_J1",
    "RIGHT_J2",
    "RIGHT_J3",
    "RIGHT_J4",
    "RIGHT_J5",
    "RIGHT_J6",
    "RIGHT_J7",
]

REAL_ROBOT_JOINT_ORDERS_V03X = [
    "ANKLE_PITCH",
    "KNEE_PITCH",
    "BUTTOCK_PITCH",
    "WAIST_YAW",
    "HEAD_YAW_J1",
    "HEAD_PITCH_J2",
    "SHOULDER_PITCH_L_J1",
    "SHOULDER_ROLL_L_J2",
    "ELBOW_PITCH_L_J3",
    "ELBOW_YAW_L_J4",
    "WRIST_PITCH_L_J5",
    "WRIST_YAW_L_J6",
    "WRIST_ROLL_L_J7",
    "SHOULDER_PITCH_R_J1",
    "SHOULDER_ROLL_R_J2",
    "ELBOW_PITCH_R_J3",
    "ELBOW_YAW_R_J4",
    "WRIST_PITCH_R_J5",
    "WRIST_YAW_R_J6",
    "WRIST_ROLL_R_J7",
]

BRAINCO_REVO1_R_REAL_JOINT_ORDERS = [
    "T_MCP", "T_CMC_YAW", "IF_MCP_PITCH", "MF_MCP_PITCH", "RF_MCP_PITCH",
    "LF_MCP_PITCH"
]
LINKER_L6_REAL_JOINT_ORDERS = [
    "T_MCP", "T_CMC_YAW", "IF_MCP_PITCH", "MF_MCP_PITCH", "RF_MCP_PITCH",
    "LF_MCP_PITCH"
]
LINKER_L20_REAL_JOINT_ORDERS = [
    "T_CMC_ROLL", "T_CMC_YAW", "T_CMC_PITCH", "T_MCP", "IF_MCP_ROLL",
    "IF_MCP_PITCH", "IF_PIP", "MF_MCP_ROLL", "MF_MCP_PITCH", "MF_PIP",
    "RF_MCP_ROLL", "RF_MCP_PITCH", "RF_PIP", "LF_MCP_ROLL", "LF_MCP_PITCH",
    "LF_PIP"
]
PAXINI_DEX_H13_REAL_JOINT_ORDERS = [
    "IF_MCP_ROLL", "IF_MCP_PITCH", "IF_PIP", "MF_MCP_ROLL", "MF_MCP_PITCH",
    "MF_PIP", "RF_MCP_ROLL", "RF_MCP_PITCH", "RF_PIP", "T_CMC_ROLL",
    "T_CMC_YAW", "T_MCP", "T_IP"
]
DH_AG_160_95_REAL_JOINT_ORDERS = ["OUTER_KNUCKLE_1"]
DH_PGC_140_50_REAL_JOINT_ORDERS = ["FINGER_1"]
AGILEX_PIPER_L100_REAL_JOINT_ORDERS = ["FINGER_1"]


class PinocchioKinematics:
    logger = get_logger()

    def __init__(self, urdf_path: str | Path, verbose: bool = False):
        # self.logger = get_logger()

        if not Path(urdf_path).exists():
            self.logger.error(f"URDF file not found: {urdf_path}")
            raise FileNotFoundError(f"URDF file not found: {urdf_path}")

        self.urdf_path = urdf_path
        self.model_ = self._build_model_with_urdf_mimic(urdf_path,
                                                        verbose=verbose)
        # self.model_ = pinocchio.buildModelFromUrdf(str(urdf_path), True)
        self.data_ = self.model_.createData()

    @property
    def model(self) -> pinocchio.Model:
        return self.model_

    @property
    def data(self) -> pinocchio.Data:
        return self.data_

    def _parse_urdf_mimic_infos(self,
                                urdf_path: str |
                                Path = None) -> list[MimicInfo]:
        if urdf_path is None:
            urdf_path = deepcopy(self.urdf_path)
        root = ET.parse(urdf_path).getroot()
        mimic_infos = []

        for joint_element in root.findall("joint"):
            mimic_element = joint_element.find("mimic")
            if mimic_element is None:
                continue

            mimic_infos.append(
                MimicInfo(
                    joint_name=joint_element.attrib["name"],
                    mimicked_joint_name=mimic_element.attrib["joint"],
                    multiplier=float(mimic_element.attrib.get(
                        "multiplier", 1.0)),
                    offset=float(mimic_element.attrib.get("offset", 0.0)),
                ))

        return mimic_infos

    @staticmethod
    def print_model_joint_info(model: pinocchio.Model, title: str):
        get_logger().debug(f"{title}")
        get_logger().debug(
            f"model nq: {model.nq}, nv: {model.nv}, njoints: {model.njoints}")
        get_logger().debug("joint names:")
        for joint_id, name in enumerate(model.names):
            joint = model.joints[joint_id]
            get_logger().debug(
                f"  [{joint_id:03d}] {name}: "
                f"type={joint.shortname()}, nq={joint.nq}, nv={joint.nv}")

    def _build_model_with_urdf_mimic(self,
                                     urdf_path: str | Path,
                                     verbose: bool = False) -> pinocchio.Model:
        model = pinocchio.buildModelFromUrdf(str(urdf_path))

        if verbose:
            self.print_model_joint_info(model, "Before setting mimic")

        mimic_infos = self._parse_urdf_mimic_infos(urdf_path)

        if verbose:
            self.logger.debug("URDF mimic infos:")

        if not mimic_infos:
            if verbose:
                self.logger.debug("  no mimic joints found")
            return model

        if verbose:
            for mimic_info in mimic_infos:
                self.logger.debug(
                    f"  {mimic_info.joint_name} mimics "
                    f"{mimic_info.mimicked_joint_name}, "
                    f"multiplier={mimic_info.multiplier}, offset={mimic_info.offset}"
                )

        for mimic_info in mimic_infos:
            if not model.existJointName(mimic_info.mimicked_joint_name):
                raise ValueError(
                    f"Mimicked joint '{mimic_info.mimicked_joint_name}' "
                    f"not found in Pinocchio model.")
            if not model.existJointName(mimic_info.joint_name):
                raise ValueError(f"Mimic joint '{mimic_info.joint_name}' "
                                 f"not found in Pinocchio model.")

            mimicked_joint_id = model.getJointId(mimic_info.mimicked_joint_name)
            mimic_joint_id = model.getJointId(mimic_info.joint_name)
            model = pinocchio.transformJointIntoMimic(
                model,
                mimicked_joint_id,
                mimic_joint_id,
                mimic_info.multiplier,
                mimic_info.offset,
            )

        if verbose:
            self.print_model_joint_info(model, "After setting mimic")
        return model

    @staticmethod
    def _normalize_link_name(link_name: str,
                             strip_side_prefix: bool = False) -> str:
        normalized_name = link_name
        if strip_side_prefix:
            for side_prefix in ("left_", "right_"):
                if normalized_name.startswith(side_prefix):
                    normalized_name = normalized_name[len(side_prefix):]
                    break
        return normalized_name

    def get_frame_pose(self, frame_name: str) -> np.ndarray:
        frame_id = self.resolve_frame_id(frame_name)
        return self.get_frame_pose_by_id(frame_id)

    def resolve_frame_id(self, frame_name: str) -> int:
        if not self.model_.existFrame(frame_name):
            raise ValueError(
                f"Frame '{frame_name}' not found in Pinocchio model.")

        return self.model_.getFrameId(frame_name)

    def get_frame_pose_by_id(self, frame_id: int) -> np.ndarray:
        return self.data_.oMf[frame_id].homogeneous.copy()

    def get_ordered_poses_by_frame_ids(
            self, ordered_frame_ids: list[int]) -> list[np.ndarray]:
        return [
            self.get_frame_pose_by_id(frame_id)
            for frame_id in ordered_frame_ids
        ]

    def update_frame_placements(self, q: np.ndarray):
        if q.shape[0] != self.model_.nq:
            raise ValueError(
                f"q shape mismatch, expected {self.model_.nq}, got {q.shape[0]}."
            )

        pinocchio.forwardKinematics(self.model_, self.data_, q)
        pinocchio.updateFramePlacements(self.model_, self.data_)

    def resolve_link_frame_name(self,
                                link_name: str,
                                strip_side_prefix: bool = False) -> str:
        normalized_link_name = self._normalize_link_name(
            link_name,
            strip_side_prefix=strip_side_prefix,
        )

        if self.model_.existFrame(normalized_link_name):
            return normalized_link_name

        raise ValueError(
            f"URDF link frame '{normalized_link_name}' (from COAL name "
            f"'{link_name}') not found in Pinocchio model.")

    def resolve_ordered_link_frame_ids(
            self,
            ordered_link_names: list[str],
            strip_side_prefix: bool = False) -> list[int]:
        return [
            self.model_.getFrameId(
                self.resolve_link_frame_name(
                    link_name,
                    strip_side_prefix=strip_side_prefix,
                )) for link_name in ordered_link_names
        ]

    def get_ordered_link_poses(self,
                               ordered_link_names: list[str],
                               strip_side_prefix: bool = False,
                               verbose: bool = False) -> list[np.ndarray]:
        ordered_poses = []
        if verbose:
            self.logger.debug("Ordered coal link -> Pinocchio frame mapping:")
        for link_name in ordered_link_names:
            frame_name = self.resolve_link_frame_name(
                link_name,
                strip_side_prefix=strip_side_prefix,
            )
            pose = self.get_frame_pose(frame_name)
            ordered_poses.append(pose)
            if verbose:
                self.logger.debug(f"  {link_name} -> {frame_name}")

        return ordered_poses

    def get_ordered_link_poses_by_component_from_pinocchio(
            self,
            q: np.ndarray,
            ordered_component_link_names: dict[str, list[str]],
            strip_side_prefix: bool = False,
            verbose: bool = False) -> dict[str, list[np.ndarray]]:
        self.update_frame_placements(q)
        return {
            component_name:
                self.get_ordered_link_poses(
                    ordered_link_names,
                    strip_side_prefix=strip_side_prefix,
                    verbose=verbose,
                ) for component_name, ordered_link_names in
            ordered_component_link_names.items()
        }

    def get_ordered_poses_by_component_frame_ids_from_pinocchio(
        self, q: np.ndarray, ordered_component_frame_ids: dict[str, list[int]]
    ) -> dict[str, list[np.ndarray]]:
        self.update_frame_placements(q)
        return {
            component_name: self.get_ordered_poses_by_frame_ids(frame_ids) for
            component_name, frame_ids in ordered_component_frame_ids.items()
        }

    def get_ordered_link_poses_from_pinocchio(
            self,
            q: np.ndarray,
            ordered_link_names: list[str],
            strip_side_prefix: bool = False,
            verbose: bool = False) -> list[np.ndarray]:
        self.update_frame_placements(q)
        return self.get_ordered_link_poses(
            ordered_link_names,
            strip_side_prefix=strip_side_prefix,
            verbose=verbose,
        )


@dataclass(frozen=True)
class EndEffectorAttachInfo:
    end_effector_type: EndEffectorType | None
    is_left: bool
    flange_T_ee: np.ndarray


class EndEffectorAttachManager:
    """Maintain flange-to-end-effector attachment transforms."""

    def __init__(self, robot_version: RobotVersion):
        self.robot_version = robot_version
        self.version_value = robot_version.version_value.lower()

    @staticmethod
    def _create_z_rotation(degrees: float) -> np.ndarray:
        theta = np.deg2rad(degrees)
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        return np.array([
            [cos_theta, -sin_theta, 0.0],
            [sin_theta, cos_theta, 0.0],
            [0.0, 0.0, 1.0],
        ])

    def _create_offset_attach_xpos(self) -> np.ndarray:
        z_offset = self._get_attach_z_offset()

        flange_T_ee = np.eye(4)
        flange_T_ee[2, 3] = z_offset
        return flange_T_ee

    def _get_attach_z_offset(self) -> float:
        if self.version_value.startswith("v02") and self.version_value.endswith(
                "f"):
            return 0.0
        elif self.version_value.startswith("v02"):
            return 0.012
        elif self.version_value.startswith("v03"):
            return 0.0
        else:
            raise ValueError(f"Unsupported robot version: {self.robot_version}")

    def _create_parallel_gripper_attach_xpos(
            self,
            is_left: bool,
            right_hand_flip_180: bool = True) -> np.ndarray:
        flange_T_ee = np.eye(4)
        flange_T_ee[2, 3] = self._get_attach_z_offset()
        angle = 0.0 if is_left or not right_hand_flip_180 else 180.0
        flange_T_ee[:3, :3] = self._create_z_rotation(angle)
        return flange_T_ee

    def get_attach_xpos(self, end_effector_type: EndEffectorType | None,
                        is_left: bool) -> np.ndarray:
        if end_effector_type in [
                EndEffectorType.BrainCo_Revo1_R,
                EndEffectorType.Linker_L6,
                EndEffectorType.Linker_L20,
        ]:
            return self._create_offset_attach_xpos()

        if end_effector_type in [
                EndEffectorType.DH_PGC_140_50,
                EndEffectorType.DH_AG_160_95,
        ]:
            return self._create_parallel_gripper_attach_xpos(
                is_left,
                right_hand_flip_180=True,
            )

        if end_effector_type in [
                EndEffectorType.AGILEX_PiPER_L100,
                EndEffectorType.AGILEX_Pika,
        ]:
            return self._create_parallel_gripper_attach_xpos(
                is_left,
                right_hand_flip_180=False,
            )

        return np.eye(4)

    def get_attach_info(self, end_effector_type: EndEffectorType | None,
                        is_left: bool) -> EndEffectorAttachInfo:
        return EndEffectorAttachInfo(
            end_effector_type=end_effector_type,
            is_left=is_left,
            flange_T_ee=self.get_attach_xpos(end_effector_type, is_left),
        )

    def transform_ee_link_poses_to_world(self, world_T_flange: np.ndarray,
                                         ee_link_poses: list[np.ndarray],
                                         end_effector_type: EndEffectorType |
                                         None,
                                         is_left: bool) -> list[np.ndarray]:
        flange_T_ee = self.get_attach_xpos(end_effector_type, is_left)
        world_T_ee = world_T_flange @ flange_T_ee
        return [world_T_ee @ ee_T_link for ee_T_link in ee_link_poses]


def get_end_effector_attach_xpos(robot_version: RobotVersion,
                                 end_effector_type: EndEffectorType | None,
                                 is_left: bool) -> np.ndarray:
    return EndEffectorAttachManager(robot_version).get_attach_xpos(
        end_effector_type,
        is_left,
    )


class PinocchioMappingGenerator:

    def __init__(self,
                 robot_version: RobotVersion,
                 left_end_effector_type: EndEffectorType = None,
                 right_end_effector_type: EndEffectorType = None,
                 verbose: bool = False):
        self.robot_version = robot_version
        self.robot_family = robot_version.robot_family
        self.robot_version_value = robot_version.version_value
        self.left_ee_type = left_end_effector_type
        self.right_ee_type = right_end_effector_type

        self.robot_urdf_path = get_data_path(
            str(Path(self.robot_family) / "robot.urdf"),
            self.robot_version_value)
        self.robot_kine = PinocchioKinematics(self.robot_urdf_path,
                                              verbose=verbose)

        ee_urdf_paths = END_EFFECTORS_URDF_PATHS.get(MeshType.DEFAULT, {})
        if left_end_effector_type is not None:
            left_ee_urdf_path = ee_urdf_paths.get(left_end_effector_type,
                                                  {}).get("left")
            self.left_ee_urdf_path = get_data_path(left_ee_urdf_path,
                                                   self.robot_version_value)
            self.left_ee_kine = PinocchioKinematics(self.left_ee_urdf_path,
                                                    verbose=verbose)
        else:
            self.left_ee_urdf_path = None
            self.left_ee_kine = None

        if right_end_effector_type is not None:
            right_ee_urdf_path = ee_urdf_paths.get(right_end_effector_type,
                                                   {}).get("right")
            self.right_ee_urdf_path = get_data_path(right_ee_urdf_path,
                                                    self.robot_version_value)
            self.right_ee_kine = PinocchioKinematics(self.right_ee_urdf_path,
                                                     verbose=verbose)
        else:
            self.right_ee_urdf_path = None
            self.right_ee_kine = None

    @staticmethod
    def _canonical_joint_name(joint_name: str,
                              strip_side_prefix: bool = False) -> str:
        canonical_name = joint_name.upper()
        if strip_side_prefix:
            for prefix in ("LEFT_", "RIGHT_"):
                if canonical_name.startswith(prefix):
                    canonical_name = canonical_name[len(prefix):]
                    break
        return canonical_name

    @staticmethod
    def _active_pinocchio_qpos_joints(
            model: pinocchio.Model,
            strip_side_prefix: bool = False) -> dict[str, tuple[int, str]]:
        active_joints = {}
        for joint_id in range(1, model.njoints):
            joint = model.joints[joint_id]
            if joint.nq <= 0 or joint.nv <= 0:
                continue
            if joint.nq != 1:
                raise ValueError(
                    f"Joint '{model.names[joint_id]}' has nq={joint.nq}; "
                    "only scalar joints are supported for qpos mapping now.")

            canonical_name = PinocchioMappingGenerator._canonical_joint_name(
                model.names[joint_id],
                strip_side_prefix=strip_side_prefix,
            )
            if canonical_name in active_joints:
                raise ValueError(
                    f"Duplicate active joint name after normalization: "
                    f"'{canonical_name}'.")
            active_joints[canonical_name] = (model.idx_qs[joint_id],
                                             model.names[joint_id])

        return active_joints

    @staticmethod
    def _get_robot_real_joint_orders(robot_version: RobotVersion) -> list[str]:
        version_value = robot_version.version_value.lower()
        if version_value.startswith("v02"):
            return REAL_ROBOT_JOINT_ORDERS_V02X
        if version_value.startswith("v03"):
            return REAL_ROBOT_JOINT_ORDERS_V03X
        raise ValueError(f"Unsupported robot version: {robot_version}")

    @staticmethod
    def _get_end_effector_real_joint_orders(
            end_effector_type: EndEffectorType) -> list[str]:
        if end_effector_type == EndEffectorType.BrainCo_Revo1_R:
            return BRAINCO_REVO1_R_REAL_JOINT_ORDERS
        if end_effector_type == EndEffectorType.Linker_L6:
            return LINKER_L6_REAL_JOINT_ORDERS
        if end_effector_type == EndEffectorType.Linker_L20:
            return LINKER_L20_REAL_JOINT_ORDERS
        if end_effector_type == EndEffectorType.PaXini_Dex_H13:
            return PAXINI_DEX_H13_REAL_JOINT_ORDERS
        if end_effector_type == EndEffectorType.DH_AG_160_95:
            return DH_AG_160_95_REAL_JOINT_ORDERS
        if end_effector_type == EndEffectorType.DH_PGC_140_50:
            return DH_PGC_140_50_REAL_JOINT_ORDERS
        if end_effector_type == EndEffectorType.AGILEX_PiPER_L100:
            return AGILEX_PIPER_L100_REAL_JOINT_ORDERS
        raise ValueError(f"Unsupported end effector type: {end_effector_type}")

    @staticmethod
    def _get_pin2real_qpos_idx(model: pinocchio.Model,
                               real_joint_names: list[str],
                               strip_side_prefix: bool = False) -> list[int]:
        active_joints = PinocchioMappingGenerator._active_pinocchio_qpos_joints(
            model,
            strip_side_prefix=strip_side_prefix,
        )

        pin2real_idx = []
        missing_joint_names = []
        for real_joint_name in real_joint_names:
            canonical_name = PinocchioMappingGenerator._canonical_joint_name(
                real_joint_name,
                strip_side_prefix=strip_side_prefix,
            )
            if canonical_name in active_joints:
                pin2real_idx.append(active_joints[canonical_name][0])
            else:
                missing_joint_names.append(real_joint_name)

        if missing_joint_names:
            raise ValueError(
                "Real joint names are not found in active Pinocchio joints: "
                f"{missing_joint_names}")

        return pin2real_idx

    @staticmethod
    def _get_real2pin_qpos_idx(model: pinocchio.Model,
                               real_joint_names: list[str],
                               strip_side_prefix: bool = False) -> list[int]:
        active_joints = PinocchioMappingGenerator._active_pinocchio_qpos_joints(
            model,
            strip_side_prefix=strip_side_prefix,
        )
        real_names = [
            PinocchioMappingGenerator._canonical_joint_name(
                name, strip_side_prefix=strip_side_prefix)
            for name in real_joint_names
        ]
        real_dict = {name: idx for idx, name in enumerate(real_names)}

        real2pin_idx = []
        missing_joint_names = []
        for active_joint_name in active_joints:
            if active_joint_name in real_dict:
                real2pin_idx.append(real_dict[active_joint_name])
            else:
                missing_joint_names.append(active_joint_name)

        if missing_joint_names:
            raise ValueError(
                "Active Pinocchio joints are not covered by real_joint_names: "
                f"{missing_joint_names}")

        return real2pin_idx

    @staticmethod
    def get_robot_pin2real_qpos_idx(model: pinocchio.Model,
                                    robot_version: RobotVersion) -> list[int]:
        real_joint_names = PinocchioMappingGenerator._get_robot_real_joint_orders(
            robot_version)
        return PinocchioMappingGenerator._get_pin2real_qpos_idx(
            model, real_joint_names)

    @staticmethod
    def get_robot_real2pin_qpos_idx(model: pinocchio.Model,
                                    robot_version: RobotVersion) -> list[int]:
        real_joint_names = PinocchioMappingGenerator._get_robot_real_joint_orders(
            robot_version)
        return PinocchioMappingGenerator._get_real2pin_qpos_idx(
            model, real_joint_names)

    @staticmethod
    def get_end_effector_pin2real_qpos_idx(
            model: pinocchio.Model,
            end_effector_type: EndEffectorType) -> list[int]:
        real_joint_names = PinocchioMappingGenerator._get_end_effector_real_joint_orders(
            end_effector_type)
        return PinocchioMappingGenerator._get_pin2real_qpos_idx(
            model,
            real_joint_names,
            strip_side_prefix=True,
        )

    @staticmethod
    def get_end_effector_real2pin_qpos_idx(
            model: pinocchio.Model,
            end_effector_type: EndEffectorType) -> list[int]:
        real_joint_names = PinocchioMappingGenerator._get_end_effector_real_joint_orders(
            end_effector_type)
        return PinocchioMappingGenerator._get_real2pin_qpos_idx(
            model,
            real_joint_names,
            strip_side_prefix=True,
        )

    @property
    def robot_pin2real_mapping(self) -> list[int]:
        return self.get_robot_pin2real_qpos_idx(self.robot_kine.model,
                                                self.robot_version)

    @property
    def robot_real2pin_mapping(self) -> list[int]:
        return self.get_robot_real2pin_qpos_idx(self.robot_kine.model,
                                                self.robot_version)

    @property
    def end_effector_pin2real_mapping(self) -> list[int]:
        left_ee_mapping, right_ee_mapping = None, None
        if self.left_ee_type is not None:
            left_ee_mapping = self.get_end_effector_pin2real_qpos_idx(
                self.left_ee_kine.model, self.left_ee_type)
        if self.right_ee_type is not None:
            right_ee_mapping = self.get_end_effector_pin2real_qpos_idx(
                self.right_ee_kine.model, self.right_ee_type)
        return left_ee_mapping, right_ee_mapping

    @property
    def end_effector_real2pin_mapping(self) -> list[int]:
        left_ee_mapping, right_ee_mapping = None, None
        if self.left_ee_type is not None:
            left_ee_mapping = self.get_end_effector_real2pin_qpos_idx(
                self.left_ee_kine.model, self.left_ee_type)
        if self.right_ee_type is not None:
            right_ee_mapping = self.get_end_effector_real2pin_qpos_idx(
                self.right_ee_kine.model, self.right_ee_type)
        return left_ee_mapping, right_ee_mapping

    @staticmethod
    def get_end_effector_real_order_qpos_limits(
            model: pinocchio.Model, end_effector_type: EndEffectorType
    ) -> tuple[np.ndarray, np.ndarray]:
        pin2real_idx = PinocchioMappingGenerator.get_end_effector_pin2real_qpos_idx(
            model,
            end_effector_type,
        )
        lower_limits = np.asarray(model.lowerPositionLimit)[pin2real_idx]
        upper_limits = np.asarray(model.upperPositionLimit)[pin2real_idx]
        return lower_limits, upper_limits

    @staticmethod
    def end_effector_pulse_to_pin_qpos(model: pinocchio.Model,
                                       end_effector_type: EndEffectorType,
                                       pulse_values: np.ndarray | list[float],
                                       pulse_min: float | np.ndarray |
                                       list[float] = 0.0,
                                       pulse_max: float | np.ndarray |
                                       list[float] = 100.0,
                                       clip: bool = True) -> np.ndarray:
        pulse_values = np.asarray(pulse_values, dtype=float)
        pin2real_idx = PinocchioMappingGenerator.get_end_effector_pin2real_qpos_idx(
            model,
            end_effector_type,
        )
        real2pin_idx = PinocchioMappingGenerator.get_end_effector_real2pin_qpos_idx(
            model,
            end_effector_type,
        )

        if pulse_values.shape != (len(pin2real_idx),):
            raise ValueError(
                f"pulse_values shape mismatch, expected {(len(pin2real_idx),)}, "
                f"got {pulse_values.shape}.")

        pulse_min = np.asarray(pulse_min, dtype=float)
        pulse_max = np.asarray(pulse_max, dtype=float)
        if pulse_min.ndim == 0:
            pulse_min = np.full_like(pulse_values, float(pulse_min))
        if pulse_max.ndim == 0:
            pulse_max = np.full_like(pulse_values, float(pulse_max))
        if pulse_min.shape != pulse_values.shape:
            raise ValueError(
                f"pulse_min shape mismatch, expected {pulse_values.shape}, "
                f"got {pulse_min.shape}.")
        if pulse_max.shape != pulse_values.shape:
            raise ValueError(
                f"pulse_max shape mismatch, expected {pulse_values.shape}, "
                f"got {pulse_max.shape}.")

        if np.any(pulse_max <= pulse_min):
            raise ValueError(
                "Each pulse_max value must be greater than pulse_min, got "
                f"pulse_min={pulse_min}, pulse_max={pulse_max}.")

        if clip:
            pulse_values = np.clip(pulse_values, pulse_min, pulse_max)
        elif np.any((pulse_values < pulse_min) | (pulse_values > pulse_max)):
            raise ValueError("pulse_values should be in the per-joint range "
                             f"[{pulse_min}, {pulse_max}], got {pulse_values}.")

        lower_limits, upper_limits = (
            PinocchioMappingGenerator.get_end_effector_real_order_qpos_limits(
                model,
                end_effector_type,
            ))
        pulse_ratio = (pulse_values - pulse_min) / (pulse_max - pulse_min)
        real_order_qpos = lower_limits + pulse_ratio * (upper_limits -
                                                        lower_limits)

        pin_qpos = real_order_qpos[real2pin_idx]

        return pin_qpos

    def left_end_effector_pulse_to_pin_qpos(self,
                                            pulse_values: np.ndarray |
                                            list[float],
                                            pulse_min: float | np.ndarray |
                                            list[float] = 0.0,
                                            pulse_max: float | np.ndarray |
                                            list[float] = 100.0,
                                            clip: bool = True) -> np.ndarray:
        if self.left_ee_kine is None or self.left_ee_type is None:
            raise ValueError("Left end effector is not configured.")
        return self.end_effector_pulse_to_pin_qpos(
            self.left_ee_kine.model,
            self.left_ee_type,
            pulse_values,
            pulse_min=pulse_min,
            pulse_max=pulse_max,
            clip=clip,
        )

    def right_end_effector_pulse_to_pin_qpos(self,
                                             pulse_values: np.ndarray |
                                             list[float],
                                             pulse_min: float | np.ndarray |
                                             list[float] = 0.0,
                                             pulse_max: float | np.ndarray |
                                             list[float] = 100.0,
                                             clip: bool = True) -> np.ndarray:
        if self.right_ee_kine is None or self.right_ee_type is None:
            raise ValueError("Right end effector is not configured.")
        return self.end_effector_pulse_to_pin_qpos(
            self.right_ee_kine.model,
            self.right_ee_type,
            pulse_values,
            pulse_min=pulse_min,
            pulse_max=pulse_max,
            clip=clip,
        )
