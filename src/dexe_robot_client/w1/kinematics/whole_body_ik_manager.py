'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

import numpy as np
from pathlib import Path
from typing import Dict
from .whole_body_ik import *
from ...common import get_logger

from dexe_rp import RobotVersion
from dexe_rp.toolkits.data import get_data_path
from dexe_rp.toolkits.data.config import DATA_DIR, _build_assets_url
from dexe_rp.toolkits.data.utils import create_directory
from dexe_rp.toolkits.data.utils import DataUtility

SDF_MAP_DIR = DATA_DIR / "signed_distance_field"
W1V02_SDF_DIR = SDF_MAP_DIR / "w1" / "v02"
W1V03_SDF_DIR = SDF_MAP_DIR / "w1" / "v03"
create_directory(W1V02_SDF_DIR, "W1V02 Signed Distance Field Map")
create_directory(W1V03_SDF_DIR, "W1V03 Signed Distance Field Map")

class WholeBodyIKSolverManager:

    def __init__(self, robot_version: RobotVersion, enable_sdf_collision: bool = False):
        self.logger = get_logger()
        self.robot_version = robot_version
        self.robot_family = robot_version.robot_family
        self.robot_verison_value = robot_version.version_value

        self.urdf_path = get_data_path(
            str(Path(robot_version.robot_family) / "robot.urdf"),
            robot_version.version_value)

        self._download_sdf_map()
        self.use_sdf_collision = enable_sdf_collision

        self.solver_manager = {}
        self._register_standard_ipopt_solver()
        self._register_standard_sqp_solver()

    def _download_sdf_map(self):
        if self.robot_family == "w1":
            if self.robot_verison_value.startswith("v02"):
                self.sdf_map_path = W1V02_SDF_DIR / "waist_sdf.npz"
                sdf_map_url = _build_assets_url("w1", "sdf_map", "v02x", "waist_sdf.npz")
            elif self.robot_verison_value.startswith("v03"):
                self.sdf_map_path = W1V03_SDF_DIR / "waist_sdf.npz"
                sdf_map_url = _build_assets_url("w1", "sdf_map", "v03x", "waist_sdf.npz")
            else:
                self.logger.error(
                    f"Unsupported robot version value: {self.robot_verison_value}"
                )
                return False
            
            if DataUtility.file_exists(self.sdf_map_path):
                self.logger.info(f"SDF map already exists: {self.sdf_map_path}")
                return True
            else:
                self.logger.info(f"Downloading SDF map from {sdf_map_url} to {self.sdf_map_path}")
                DataUtility.make_directory_hierarchy(str(self.sdf_map_path.parent))
                DataUtility.download_file(sdf_map_url, self.sdf_map_path)
                return True
        elif self.robot_family == "a1":
            pass
        elif self.robot_family == "l1":
            pass

    def _register_standard_ipopt_solver(self):
        collision_sphere_model_config = None
        torso_sdf_collision_config = None
        inter_arm_collision_config = None
        if self.robot_family == "w1":
            if self.robot_verison_value.startswith("v02"):
                tracking_frame_config = TrackingFrameConfig.init_from_urdf(
                    robot="W1V02X", urdf_path=self.urdf_path)
                joint_name_config = JointNameConfig.init_from_robot(
                    robot="W1V02X")
                if self.use_sdf_collision:
                    collision_sphere_model_config = CollisionSphereModelConfig.init_from_robot(robot="W1V02X")
                    torso_sdf_collision_config = TorsoSDFCollisionConfig(
                            torso_frame_name="waist",
                            sdf_path=self.sdf_map_path,
                            cost_weight=2.5,
                    )
                    inter_arm_collision_config = InterArmSphereCollisionConfig.init_from_robot(robot="W1V02X")
            elif self.robot_verison_value.startswith("v03"):
                tracking_frame_config = TrackingFrameConfig.init_from_urdf(
                    robot="W1V03X", urdf_path=self.urdf_path)
                joint_name_config = JointNameConfig.init_from_robot(
                    robot="W1V03X")
                if self.use_sdf_collision:
                    collision_sphere_model_config = CollisionSphereModelConfig.init_from_robot(robot="W1V03X")
                    torso_sdf_collision_config = TorsoSDFCollisionConfig(
                            torso_frame_name="waist_yaw_link",
                            sdf_path=self.sdf_map_path,
                            cost_weight=2.5,
                    )
                    inter_arm_collision_config = InterArmSphereCollisionConfig.init_from_robot(robot="W1V03X")
            else:
                self.logger.error(
                    f"Unsupported robot version value: {self.robot_verison_value}"
                )
                return False
            solver_options = IpoptOptions()
            weight_config = WeightConfig()
            posture_config = PostureConfig()
            freezed_joint_dict = None

            ik_solver = WholeBodyIK(urdf_path=self.urdf_path,
                                    tracking_frame_config=tracking_frame_config,
                                    joint_name_config=joint_name_config,
                                    solver_options=solver_options,
                                    weight_config=weight_config,
                                    posture_config=posture_config,
                                    freezed_joint_dict=freezed_joint_dict,
                                    collision_sphere_model_config=collision_sphere_model_config,
                                    torso_sdf_collision_config=torso_sdf_collision_config,
                                    inter_arm_collision_config=inter_arm_collision_config)

            self.solver_manager["standard_ipopt"] = ik_solver
        elif self.robot_family == "a1":
            pass
        elif self.robot_family == "l1":
            pass

        return True

    def _register_standard_sqp_solver(self):
        collision_sphere_model_config = None
        torso_sdf_collision_config = None
        inter_arm_collision_config = None
        if self.robot_family == "w1":
            if self.robot_verison_value.startswith("v02"):
                tracking_frame_config = TrackingFrameConfig.init_from_urdf(
                    robot="W1V02X", urdf_path=self.urdf_path)
                joint_name_config = JointNameConfig.init_from_robot(
                    robot="W1V02X")
                if self.use_sdf_collision:
                    collision_sphere_model_config = CollisionSphereModelConfig.init_from_robot(robot="W1V02X")
                    torso_sdf_collision_config = TorsoSDFCollisionConfig(
                            torso_frame_name="waist",
                            sdf_path=self.sdf_map_path,
                            cost_weight=2.5,
                    )
                    inter_arm_collision_config = InterArmSphereCollisionConfig.init_from_robot(robot="W1V02X")
            elif self.robot_verison_value.startswith("v03"):
                tracking_frame_config = TrackingFrameConfig.init_from_urdf(
                    robot="W1V03X", urdf_path=self.urdf_path)
                joint_name_config = JointNameConfig.init_from_robot(
                    robot="W1V03X")
                if self.use_sdf_collision:
                    collision_sphere_model_config = CollisionSphereModelConfig.init_from_robot(robot="W1V03X")
                    torso_sdf_collision_config = TorsoSDFCollisionConfig(
                            torso_frame_name="waist_yaw_link",
                            sdf_path=self.sdf_map_path,
                            cost_weight=2.5,
                    )
                    inter_arm_collision_config = InterArmSphereCollisionConfig.init_from_robot(robot="W1V03X")
            else:
                self.logger.error(
                    f"Unsupported robot version value: {self.robot_verison_value}"
                )
                return False
            solver_options = SqpMethodOptions()
            weight_config = WeightConfig()
            posture_config = PostureConfig()
            freezed_joint_dict = None

            ik_solver = WholeBodyIK(urdf_path=self.urdf_path,
                                    tracking_frame_config=tracking_frame_config,
                                    joint_name_config=joint_name_config,
                                    solver_options=solver_options,
                                    weight_config=weight_config,
                                    posture_config=posture_config,
                                    freezed_joint_dict=freezed_joint_dict,
                                    collision_sphere_model_config=collision_sphere_model_config,
                                    torso_sdf_collision_config=torso_sdf_collision_config,
                                    inter_arm_collision_config=inter_arm_collision_config)

            self.solver_manager["standard_sqp"] = ik_solver
        elif self.robot_family == "a1":
            pass
        elif self.robot_family == "l1":
            pass

        return True

    @property
    def supported_solvers(self):
        solver_list = [key for key in self.solver_manager]
        return solver_list

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
        use_same_targets_filter: bool = False,
    ):
        sqp_solver = self.solver_manager.get("standard_sqp")
        ipopt_solver = self.solver_manager.get("standard_ipopt")
        if sqp_solver is None or ipopt_solver is None:
            self.logger.error(
                "Both standard_sqp and standard_ipopt solvers must be registered."
            )
            return False, None

        solve_kwargs = {
            "left_arm_target": left_arm_target,
            "right_arm_target": right_arm_target,
            "head_target": head_target,
            "ref_qpos": ref_qpos,
            "upper_body_leaning_threshold": upper_body_leaning_threshold,
            "eye_focus_point": eye_focus_point,
            "is_strict_solution": is_strict_solution,
            "extra_targets": extra_targets,
            "extra_parameters": extra_parameters,
            "use_same_targets_filter": use_same_targets_filter,
        }
        success, solution_q = sqp_solver.solve_ik(**solve_kwargs)
        if success:
            return True, solution_q

        self.logger.warning("SQP IK failed; retrying with IPOPT.")
        return ipopt_solver.solve_ik(**solve_kwargs)
