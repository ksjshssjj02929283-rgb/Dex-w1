'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''
from pathlib import Path
import numpy as np
import time
from ...common import get_logger, StatusCode, CACHE_DEXE_DATA, DATA_DIR, time_consuming
from .coal_collision_wrapper import CoalCollisionWrapper
from ..kinematics.pinocchio_kinematics import PinocchioKinematics, EndEffectorAttachManager, PinocchioMappingGenerator, get_end_effector_attach_xpos

from dexe_rp import RobotVersion, EndEffectorType
from dexe_rp.toolkits.urdf_assembly.type import MeshType
from dexe_rp.toolkits.data import get_data_path
from dexe_rp.toolkits.urdf_assembly.ee_config import END_EFFECTORS_URDF_PATHS
from dexe_rp.toolkits.data.config import BASE_ASSETS_MODEL_URL_PREFIX, _build_assets_url
from dexe_rp.toolkits.data.utils import DataUtility

END_EFFECTOR_PULSE_MIN_MAX = {
    EndEffectorType.DH_PGC_140_50: {
        "min": [0.0],
        "max": [100.0]
    },
    EndEffectorType.DH_AG_160_95: {
        "min": [0.0],
        "max": [100.0]
    },
    EndEffectorType.BrainCo_Revo1_R: {
        "min": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "max": [100.0, 100.0, 100.0, 100.0, 100.0, 100.0]
    },
    EndEffectorType.Linker_L6: {
        "min": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "max": [100.0, 100.0, 100.0, 100.0, 100.0, 100.0]
    },
    EndEffectorType.Linker_L20: {
        "min": [
            0.0, 0.0, 0.0, 0.0, -100.0, 0.0, 0.0, -100.0, 0.0, 0.0, -100.0, 0.0,
            0.0, -100.0, 0.0, 0.0
        ],
        "max": [
            100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
            100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0
        ]
    },
    EndEffectorType.PaXini_Dex_H13: {
        "min": [
            -100.0, 0.0, 0.0, -100.0, 0.0, 0.0, -100.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0
        ],
        "max": [
            100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
            100.0, 100.0, 100.0, 100.0
        ]
    },
    EndEffectorType.AGILEX_PiPER_L100: {
        "min": [0.0],
        "max": [100.0]
    },
    EndEffectorType.AGILEX_Pika: {
        "min": [0.0],
        "max": [100.0]
    }
}


class CollisionCheckerWrapper:

    def __init__(self,
                 robot_version: RobotVersion,
                 left_end_effector_type: EndEffectorType = None,
                 right_end_effector_type: EndEffectorType = None,
                 mesh_type: MeshType = MeshType.DEFAULT,
                 verbose: bool = False):
        self.logger = get_logger()
        self.robot_version = robot_version
        self.robot_family = robot_version.robot_family
        self.robot_version_value = robot_version.version_value
        self.left_ee_type = left_end_effector_type
        self.right_ee_type = right_end_effector_type

        self.robot_urdf_path = get_data_path(
            str(Path(robot_version.robot_family) / "robot.urdf"),
            robot_version.version_value)

        ee_urdf_paths = END_EFFECTORS_URDF_PATHS.get(mesh_type, {})
        left_ee_urdf_path, right_ee_urdf_path = None, None
        if left_end_effector_type is not None:
            left_ee_urdf_path = ee_urdf_paths.get(left_end_effector_type,
                                                  {}).get("left", None)
            self.left_ee_urdf_path = get_data_path(left_ee_urdf_path,
                                                   robot_version.version_value)
        if right_end_effector_type is not None:
            right_ee_urdf_path = ee_urdf_paths.get(right_end_effector_type,
                                                   {}).get("right", None)
            self.right_ee_urdf_path = get_data_path(right_ee_urdf_path,
                                                    robot_version.version_value)

        collision_pairs_json_name = "ignored_collision_pairs.json"
        ignored_collision_pairs_json_path = DATA_DIR / collision_pairs_json_name  # dexe_robot_client_data路径

        # collision_pairs_json_url = _build_assets_url(str(self.robot_family),
        #                                              collision_pairs_json_name)
        # collision_pairs_json_destination = CACHE_DEXE_DATA / collision_pairs_json_name
        # if DataUtility.file_exists(collision_pairs_json_destination):
        #     self.logger.info(
        #         f"Collision pairs JSON file already exists at {collision_pairs_json_destination}. Prioritize the use of this file."
        #     )
        # else:
        #     self.logger.info(
        #         f"Downloading collision pairs JSON file from {collision_pairs_json_url} to {collision_pairs_json_destination}."
        #     )
        #     DataUtility.make_directory_hierarchy(str(CACHE_DEXE_DATA))
        #     DataUtility.download_file(collision_pairs_json_url,
        #                               collision_pairs_json_destination)

        self.coal_collision = CoalCollisionWrapper(
            robot_version=robot_version,
            left_end_effector_type=left_end_effector_type,
            right_end_effector_type=right_end_effector_type,
            mesh_type=mesh_type,
            verbose=verbose)
        if ignored_collision_pairs_json_path.exists():
            self.coal_collision.load_ignored_collision_pairs_from_json(
                str(ignored_collision_pairs_json_path))

        self.robot_pin_kine = PinocchioKinematics(self.robot_urdf_path)
        self.left_ee_pin_kine, self.right_ee_pin_kine = None, None
        self.left_ee_attach_pose, self.right_ee_attach_pose = None, None
        self.pin_mapping_generator = PinocchioMappingGenerator(
            robot_version, left_end_effector_type, right_end_effector_type)
        self.robot_pin2real_mapping = self.pin_mapping_generator.robot_pin2real_mapping
        self.robot_real2pin_mapping = self.pin_mapping_generator.robot_real2pin_mapping
        if left_ee_urdf_path is not None:
            self.left_ee_pin_kine = PinocchioKinematics(self.left_ee_urdf_path)
            self.left_ee_attach_pose = get_end_effector_attach_xpos(
                robot_version, self.left_ee_type, True)
        if right_ee_urdf_path is not None:
            self.right_ee_pin_kine = PinocchioKinematics(
                self.right_ee_urdf_path)
            self.right_ee_attach_pose = get_end_effector_attach_xpos(
                robot_version, self.right_ee_type, False)
        self.left_ee_pin2real_mapping, self.right_ee_pin2real_mapping = self.pin_mapping_generator.end_effector_pin2real_mapping
        self.left_ee_real2pin_mapping, self.right_ee_real2pin_mapping = self.pin_mapping_generator.end_effector_real2pin_mapping

        if self.robot_family == "w1":
            self.torso_link_names, self.head_link_names, self.left_arm_link_names, self.right_arm_link_names = self.coal_collision.get_robot_link_names(
            )
        self.left_ee_link_names, self.right_ee_link_names = self.coal_collision.get_end_effector_link_names(
        )
        if self.robot_family == "w1":
            self.robot_component_frame_ids = {
                "torso":
                    self.robot_pin_kine.resolve_ordered_link_frame_ids(
                        self.torso_link_names,),
                "head":
                    self.robot_pin_kine.resolve_ordered_link_frame_ids(
                        self.head_link_names,),
                "left_arm":
                    self.robot_pin_kine.resolve_ordered_link_frame_ids(
                        self.left_arm_link_names,),
                "right_arm":
                    self.robot_pin_kine.resolve_ordered_link_frame_ids(
                        self.right_arm_link_names,),
            }
            self.left_ee_flange_frame_id = self.robot_pin_kine.resolve_frame_id(
                "LEFT_EE")
            self.right_ee_flange_frame_id = self.robot_pin_kine.resolve_frame_id(
                "RIGHT_EE")
        if self.left_ee_pin_kine is not None:
            self.left_ee_frame_ids = self.left_ee_pin_kine.resolve_ordered_link_frame_ids(
                self.left_ee_link_names,
                strip_side_prefix=True,
            )
        if self.right_ee_pin_kine is not None:
            self.right_ee_frame_ids = self.right_ee_pin_kine.resolve_ordered_link_frame_ids(
                self.right_ee_link_names,
                strip_side_prefix=True,
            )

    @property
    def collision_handler(self):
        return self.coal_collision

    def set_verbose(self, verbose: bool):
        self.coal_collision.set_verbose(verbose)

    def check_collision(self,
                        qpos: np.ndarray,
                        left_ee_pulse: np.ndarray = None,
                        right_ee_pulse: np.ndarray = None,
                        robot_name: str = "whole_body",
                        save_scene: bool = False):
        qpos = np.asarray(qpos).reshape(-1)
        if qpos.shape[0] != self.robot_pin_kine.model.nq:
            self.logger.error(
                f"Invalid qpos shape: {qpos.shape}, expected: ({self.robot_pin_kine.model.nq},)"
            )
            return StatusCode.INPUT_INVALID
        if self.left_ee_type is not None:
            if left_ee_pulse is None:
                self.logger.error(
                    "Left end effector pulse values are required but not provided."
                )
                return StatusCode.INPUT_INVALID
            left_ee_pulse = np.asarray(left_ee_pulse).reshape(-1)
            if left_ee_pulse.shape[0] != self.left_ee_pin_kine.model.nq:
                self.logger.error(
                    f"Invalid left end effector pulse shape: {left_ee_pulse.shape}, expected: ({self.left_ee_pin_kine.model.nq},)"
                )
                return StatusCode.INPUT_INVALID
            # TODO: define the pulse min and max values for left end effector based on its type and version
            left_ee_pin_qpos = self.pin_mapping_generator.left_end_effector_pulse_to_pin_qpos(
                pulse_values=left_ee_pulse,
                pulse_min=END_EFFECTOR_PULSE_MIN_MAX.get(self.left_ee_type,
                                                         {}).get("min"),
                pulse_max=END_EFFECTOR_PULSE_MIN_MAX.get(self.left_ee_type,
                                                         {}).get("max"),
                clip=True)

        if self.right_ee_type is not None:
            if right_ee_pulse is None:
                self.logger.error(
                    "Right end effector pulse values are required but not provided."
                )
                return StatusCode.INPUT_INVALID
            right_ee_pulse = np.asarray(right_ee_pulse).reshape(-1)
            if right_ee_pulse.shape[0] != self.right_ee_pin_kine.model.nq:
                self.logger.error(
                    f"Invalid right end effector pulse shape: {right_ee_pulse.shape}, expected: ({self.right_ee_pin_kine.model.nq},)"
                )
                return StatusCode.INPUT_INVALID
            right_ee_pin_qpos = self.pin_mapping_generator.right_end_effector_pulse_to_pin_qpos(
                pulse_values=right_ee_pulse,
                pulse_min=END_EFFECTOR_PULSE_MIN_MAX.get(
                    self.right_ee_type, {}).get("min"),
                pulse_max=END_EFFECTOR_PULSE_MIN_MAX.get(
                    self.right_ee_type, {}).get("max"),
                clip=True)

        robot_pin_qpos = qpos[self.robot_real2pin_mapping]
        if self.robot_family == "w1":
            robot_component_tfs = self.robot_pin_kine.get_ordered_poses_by_component_frame_ids_from_pinocchio(
                robot_pin_qpos,
                self.robot_component_frame_ids,
            )
            torso_link_tfs = robot_component_tfs["torso"]
            head_link_tfs = robot_component_tfs["head"]
            left_arm_link_tfs = robot_component_tfs["left_arm"]
            right_arm_link_tfs = robot_component_tfs["right_arm"]
            self.coal_collision.update_robot_component_pose(
                "torso", torso_link_tfs)
            self.coal_collision.update_robot_component_pose(
                "head", head_link_tfs)
            self.coal_collision.update_robot_component_pose(
                "left_arm", left_arm_link_tfs)
            self.coal_collision.update_robot_component_pose(
                "right_arm", right_arm_link_tfs)

            if self.left_ee_type is not None:
                self.left_ee_pin_kine.update_frame_placements(left_ee_pin_qpos)
                left_ee_tfs = self.left_ee_pin_kine.get_ordered_poses_by_frame_ids(
                    self.left_ee_frame_ids)
                left_ee_flange_tf = self.robot_pin_kine.get_frame_pose_by_id(
                    self.left_ee_flange_frame_id)
                left_ee_link_tfs = [
                    left_ee_flange_tf @ self.left_ee_attach_pose @ tf
                    for tf in left_ee_tfs
                ]
                self.coal_collision.update_robot_component_pose(
                    "left_end_effector", left_ee_link_tfs)
            if self.right_ee_type is not None:
                self.right_ee_pin_kine.update_frame_placements(
                    right_ee_pin_qpos)
                right_ee_tfs = self.right_ee_pin_kine.get_ordered_poses_by_frame_ids(
                    self.right_ee_frame_ids)
                right_ee_flange_tf = self.robot_pin_kine.get_frame_pose_by_id(
                    self.right_ee_flange_frame_id)
                right_ee_link_tfs = [
                    right_ee_flange_tf @ self.right_ee_attach_pose @ tf
                    for tf in right_ee_tfs
                ]
                self.coal_collision.update_robot_component_pose(
                    "right_end_effector", right_ee_link_tfs)
        elif self.robot_family == "a1":
            pass
        elif self.robot_family == "l1":
            pass

        res = self.coal_collision.check_collision(robot_name=robot_name)

        if res == StatusCode.COLLISION_FAIL:
            if save_scene:
                self.coal_collision.export_scene_to_ply(
                    str(Path.home() / f"log/collision_check_scene.ply"))

        return res

    def check_collision_between_arms(self,
                                     left_arm_qpos: np.ndarray,
                                     right_arm_qpos: np.ndarray,
                                     left_ee_pulse: np.ndarray = None,
                                     right_ee_pulse: np.ndarray = None):
        left_arm_qpos = np.asarray(left_arm_qpos).reshape(-1)
        right_arm_qpos = np.asarray(right_arm_qpos).reshape(-1)

        if self.left_ee_type is not None:
            if left_ee_pulse is None:
                self.logger.error(
                    "Left end effector pulse values are required but not provided."
                )
                return StatusCode.INPUT_INVALID
            left_ee_pulse = np.asarray(left_ee_pulse).reshape(-1)
            left_ee_pin_qpos = self.pin_mapping_generator.left_end_effector_pulse_to_pin_qpos(
                pulse_values=left_ee_pulse,
                pulse_min=END_EFFECTOR_PULSE_MIN_MAX.get(self.left_ee_type,
                                                         {}).get("min"),
                pulse_max=END_EFFECTOR_PULSE_MIN_MAX.get(self.left_ee_type,
                                                         {}).get("max"),
                clip=True)
        if self.right_ee_type is not None:
            if right_ee_pulse is None:
                self.logger.error(
                    "Right end effector pulse values are required but not provided."
                )
                return StatusCode.INPUT_INVALID
            right_ee_pulse = np.asarray(right_ee_pulse).reshape(-1)
            right_ee_pin_qpos = self.pin_mapping_generator.right_end_effector_pulse_to_pin_qpos(
                pulse_values=right_ee_pulse,
                pulse_min=END_EFFECTOR_PULSE_MIN_MAX.get(
                    self.right_ee_type, {}).get("min"),
                pulse_max=END_EFFECTOR_PULSE_MIN_MAX.get(
                    self.right_ee_type, {}).get("max"),
                clip=True)

        if self.robot_family == "w1":
            torso_qpos, head_qpos = np.zeros(4), np.zeros(2)
            robot_real_qpos = np.concatenate(
                [torso_qpos, head_qpos, left_arm_qpos, right_arm_qpos])
            robot_pin_qpos = robot_real_qpos[self.robot_real2pin_mapping]

            self.robot_pin_kine.update_frame_placements(robot_pin_qpos)
            left_arm_link_tfs = self.robot_pin_kine.get_ordered_poses_by_frame_ids(
                self.robot_component_frame_ids.get("left_arm", []))
            right_arm_link_tfs = self.robot_pin_kine.get_ordered_poses_by_frame_ids(
                self.robot_component_frame_ids.get("right_arm", []))
            self.coal_collision.update_robot_component_pose(
                "left_arm", left_arm_link_tfs)
            self.coal_collision.update_robot_component_pose(
                "right_arm", right_arm_link_tfs)

            if self.left_ee_type is not None:
                self.left_ee_pin_kine.update_frame_placements(left_ee_pin_qpos)
                left_ee_tfs = self.left_ee_pin_kine.get_ordered_poses_by_frame_ids(
                    self.left_ee_frame_ids)
                left_ee_flange_tf = self.robot_pin_kine.get_frame_pose_by_id(
                    self.left_ee_flange_frame_id)
                left_ee_link_tfs = [
                    left_ee_flange_tf @ self.left_ee_attach_pose @ tf
                    for tf in left_ee_tfs
                ]
                self.coal_collision.update_robot_component_pose(
                    "left_end_effector", left_ee_link_tfs)
            if self.right_ee_type is not None:
                self.right_ee_pin_kine.update_frame_placements(
                    right_ee_pin_qpos)
                right_ee_tfs = self.right_ee_pin_kine.get_ordered_poses_by_frame_ids(
                    self.right_ee_frame_ids)
                right_ee_flange_tf = self.robot_pin_kine.get_frame_pose_by_id(
                    self.right_ee_flange_frame_id)
                right_ee_link_tfs = [
                    right_ee_flange_tf @ self.right_ee_attach_pose @ tf
                    for tf in right_ee_tfs
                ]
                self.coal_collision.update_robot_component_pose(
                    "right_end_effector", right_ee_link_tfs)
        elif self.robot_family == "a1":
            pass
        elif self.robot_family == "l1":
            pass

        res1 = self.coal_collision.check_components_collision(
            "left_arm", "right_arm")
        if res1 == StatusCode.COLLISION_FAIL:
            return res1

        res2 = self.coal_collision.check_components_collision(
            "left_end_effector", "right_arm")
        if res2 == StatusCode.COLLISION_FAIL:
            return res2

        res3 = self.coal_collision.check_components_collision(
            "right_end_effector", "left_arm")
        if res3 == StatusCode.COLLISION_FAIL:
            return res3

        res4 = self.coal_collision.check_components_collision(
            "left_end_effector", "right_end_effector")
        if res4 == StatusCode.COLLISION_FAIL:
            return res4

        return StatusCode.SUCCESS
