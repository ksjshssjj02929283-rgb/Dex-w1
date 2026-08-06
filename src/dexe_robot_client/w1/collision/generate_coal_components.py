'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

from pathlib import Path
from enum import Enum, auto
from dataclasses import dataclass, field

from ...common.config import CACHE_EXTRACTED_DIR
from dexe_rp import ArmType, RobotVersion, EndEffectorType
from dexe_rp.toolkits.urdf_assembly.type import MeshType
from dexe_rp.toolkits.urdf_assembly.ee_config import EEVersion, END_EFFECTORS_URDF_PATHS
from dexe_rp.toolkits.data import get_data_path


# yapf: disable
def get_robot_component_mesh_dir(robot_version: RobotVersion,
                                   mesh_type: MeshType = MeshType.DEFAULT):
    robot_family = robot_version.robot_family
    version_value = robot_version.version_value

    urdf_path = get_data_path(str(Path(robot_family) / "robot.urdf"), version_value)
    package_dir = Path(urdf_path).parent.parent
    # collision_dir = package_dir / "collision"
    if mesh_type == MeshType.FINE_COLLISION:
        collision_folder = "collision"
        collision_level = "fine"
    elif mesh_type == MeshType.DEFAULT:
        collision_folder = "collision"
        collision_level = "rough"
    elif mesh_type == MeshType.STL_VISUAL:
        collision_folder = "visual"
        collision_level = "stl"

    if robot_family == "w1":
        torso_mesh_dir = str(package_dir / "torso" / collision_folder / collision_level)
        head_mesh_dir = str(package_dir / "head" / collision_folder / collision_level)
        left_arm_mesh_dir = str(package_dir / "left_arm" / collision_folder / collision_level)
        right_arm_mesh_dir = str(package_dir / "right_arm" / collision_folder / collision_level)

        return torso_mesh_dir, head_mesh_dir, left_arm_mesh_dir, right_arm_mesh_dir
    elif robot_family == "l1":
        pass
    elif robot_family == "a1":
        pass
    else:
        raise ValueError(f"Unsupported robot family: {robot_family}")

def get_end_effector_component_mesh_dir(robot_version: RobotVersion,
                                          left_end_effector_type: EndEffectorType = None,
                                          right_end_effector_type: EndEffectorType = None,
                                          mesh_type: MeshType = MeshType.DEFAULT):
    # if left_end_effector_type is None and right_end_effector_type is None:
    #     raise ValueError("At least one of left_end_effector_type or right_end_effector_type must be provided.")
    version_value = robot_version.version_value
    urdf_paths = END_EFFECTORS_URDF_PATHS.get(mesh_type, {})
    if mesh_type == MeshType.FINE_COLLISION:
        collision_folder = "collision"
        collision_level = "fine"
    elif mesh_type == MeshType.DEFAULT:
        collision_folder = "collision"
        collision_level = "rough"
    elif mesh_type == MeshType.STL_VISUAL:
        collision_folder = "visual"
        collision_level = "stl"
    left_ee_mesh_dir, right_ee_mesh_dir = None, None

    if left_end_effector_type is not None:
        left_ee_urdf_path = urdf_paths.get(left_end_effector_type, {}).get("left")
        left_ee_urdf_path = get_data_path(left_ee_urdf_path, version_value)
        left_ee_mesh_dir = str(Path(left_ee_urdf_path).parent / collision_folder / collision_level)

    if right_end_effector_type is not None:
        right_ee_urdf_path = urdf_paths.get(right_end_effector_type, {}).get("right")
        right_ee_urdf_path = get_data_path(right_ee_urdf_path, version_value)
        right_ee_mesh_dir = str(Path(right_ee_urdf_path).parent / collision_folder / collision_level)

    return left_ee_mesh_dir, right_ee_mesh_dir
