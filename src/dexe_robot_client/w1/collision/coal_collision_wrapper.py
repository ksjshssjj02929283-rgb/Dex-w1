'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''
import numpy as np
from pathlib import Path
from dataclasses import dataclass
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation as R

from ...common import get_logger, StatusCode
from .generate_coal_components import get_robot_component_mesh_dir, get_end_effector_component_mesh_dir

from dexe_rp import ArmType, RobotVersion, EndEffectorType, CoalCollision
from dexe_rp.toolkits.urdf_assembly.type import MeshType
from dexe_rp.toolkits.data import get_data_path
from dexe_rp.toolkits.urdf_assembly.ee_config import END_EFFECTORS_URDF_PATHS


@dataclass(frozen=True)
class URDFLinkCollision:
    """The selected collision mesh and its fixed transform in a URDF link."""

    component_name: str
    link_name: str
    mesh_path: str
    link_T_collision: np.ndarray


def _parse_vector(value: str | None, size: int, default: float) -> np.ndarray:
    if value is None:
        return np.full(size, default, dtype=float)
    result = np.fromstring(value, sep=" ", dtype=float)
    if result.size != size:
        raise ValueError(f"Expected {size} values, got '{value}'.")
    return result


def _origin_transform(origin: ET.Element | None) -> np.ndarray:
    transform = np.eye(4)
    if origin is None:
        return transform

    xyz = _parse_vector(origin.get("xyz"), 3, 0.0)
    rpy = _parse_vector(origin.get("rpy"), 3, 0.0)

    transform[:3, :3] = R.from_euler("xyz", rpy).as_matrix()
    transform[:3, 3] = xyz
    return transform


def _mesh_key(mesh_path: str) -> str:
    """Normalize the part of a mesh reference that is stable across levels."""
    return Path(mesh_path).stem.casefold()


def _index_mesh_directory(mesh_dir: str | Path) -> dict[str, list[str]]:
    """Index files in a mesh directory by case-insensitive filename stem."""
    mesh_dir = Path(mesh_dir)
    if not mesh_dir.is_dir():
        raise FileNotFoundError(f"Mesh directory does not exist: '{mesh_dir}'.")

    mesh_files: dict[str, list[str]] = {}
    for mesh_path in mesh_dir.iterdir():
        if not mesh_path.is_file():
            continue
        key = _mesh_key(str(mesh_path))
        mesh_files.setdefault(key, []).append(str(mesh_path))
    return mesh_files


def _select_mesh_file(urdf_mesh_path: str, candidates: list[str]) -> str:
    """Prefer the URDF filename, then allow one same-stem level variant."""
    urdf_filename = Path(urdf_mesh_path).name.casefold()
    exact_matches = [
        path for path in candidates
        if Path(path).name.casefold() == urdf_filename
    ]
    if len(exact_matches) == 1:
        return exact_matches[0]
    if not exact_matches and len(candidates) == 1:
        return candidates[0]
    raise ValueError(
        f"Cannot uniquely select a file for URDF mesh '{urdf_mesh_path}' "
        f"from candidates {candidates}.")


def load_urdf_link_collisions(
        urdf_path: str | Path,
        component_mesh_dirs: dict[str, str | Path],
        link_name_prefix: str = "") -> dict[str, list[URDFLinkCollision]]:
    """Find each URDF collision mesh in a level-selected component directory.

    Results follow URDF link order.  The extension is deliberately excluded
    from matching so an OBJ referenced by the URDF can select an STL from a
    visual mesh directory.
    """
    urdf_path = Path(urdf_path)
    root = ET.parse(urdf_path).getroot()
    component_mesh_files = {
        component_name: _index_mesh_directory(mesh_dir)
        for component_name, mesh_dir in component_mesh_dirs.items()
    }
    result: dict[str, list[URDFLinkCollision]] = {
        component_name: [] for component_name in component_mesh_dirs
    }

    for link in root.findall("link"):
        collisions = link.findall("collision")
        if not collisions:
            continue
        if len(collisions) != 1:
            raise ValueError(
                f"URDF link '{link.get('name')}' in '{urdf_path}' has "
                f"{len(collisions)} collision elements; expected exactly one.")

        collision = collisions[0]
        mesh = collision.find("geometry/mesh")
        if mesh is None:
            continue
        urdf_mesh_path = mesh.get("filename")
        if not urdf_mesh_path:
            raise ValueError(
                f"Mesh collision of URDF link '{link.get('name')}' in "
                f"'{urdf_path}' has no filename.")

        key = _mesh_key(urdf_mesh_path)
        matches = [
            (
                component_name,
                _select_mesh_file(urdf_mesh_path, mesh_files[key]),
            )
            for component_name, mesh_files in component_mesh_files.items()
            if key in mesh_files
        ]
        if not matches:
            # The URDF can contain links outside the requested components,
            # such as the W1 chassis.
            continue
        if len(matches) != 1:
            matched_components = [name for name, _ in matches]
            raise ValueError(
                f"URDF mesh '{urdf_mesh_path}' of link '{link.get('name')}' "
                f"matches multiple components: {matched_components}.")

        scale = _parse_vector(mesh.get("scale"), 3, 1.0)
        if not np.allclose(scale, np.ones(3)):
            raise ValueError(
                f"URDF link '{link.get('name')}' uses mesh scale {scale}; "
                "the current CoalCollision mesh API cannot apply mesh scale.")

        component_name, selected_mesh_path = matches[0]
        urdf_link_name = link.get("name")
        result[component_name].append(
            URDFLinkCollision(
                component_name=component_name,
                link_name=f"{link_name_prefix}{urdf_link_name}",
                mesh_path=selected_mesh_path,
                link_T_collision=_origin_transform(collision.find("origin")),
            ))

    empty_components = [
        name for name, collisions in result.items() if not collisions
    ]
    if empty_components:
        raise ValueError(f"No URDF collision meshes were found for components "
                         f"{empty_components} in '{urdf_path}'.")
    return result


class CoalCollisionWrapper:

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

        self.component_link_mesh = []

        if self.robot_family == "w1":
            robot_urdf_path = get_data_path(
                str(Path(self.robot_family) / "robot.urdf"),
                self.robot_version_value,
            )
            torso_dir, head_dir, left_arm_dir, right_arm_dir = get_robot_component_mesh_dir(
                robot_version, mesh_type)

            self.component_collisions = load_urdf_link_collisions(
                robot_urdf_path,
                {
                    "torso": torso_dir,
                    "head": head_dir,
                    "left_arm": left_arm_dir,
                    "right_arm": right_arm_dir,
                },
            )

            self.torso_link_names = self._component_link_names("torso")
            self.head_link_names = self._component_link_names("head")
            self.left_arm_link_names = self._component_link_names("left_arm")
            self.right_arm_link_names = self._component_link_names("right_arm")

        elif self.robot_family == "a1":
            # TODO: add a1 robot components
            pass
        elif self.robot_family == "l1":
            # TODO: add l1 robot components
            pass

        left_ee_mesh_dir, right_ee_mesh_dir = get_end_effector_component_mesh_dir(
            robot_version,
            left_end_effector_type,
            right_end_effector_type,
            mesh_type,
        )

        ee_urdf_paths = END_EFFECTORS_URDF_PATHS.get(mesh_type, {})
        if left_end_effector_type is not None:
            left_urdf_path = get_data_path(
                ee_urdf_paths[left_end_effector_type]["left"],
                self.robot_version_value,
            )
            self.component_collisions.update(
                load_urdf_link_collisions(
                    left_urdf_path, {"left_end_effector": left_ee_mesh_dir},
                    link_name_prefix="left_"))
        if right_end_effector_type is not None:
            right_urdf_path = get_data_path(
                ee_urdf_paths[right_end_effector_type]["right"],
                self.robot_version_value,
            )
            self.component_collisions.update(
                load_urdf_link_collisions(
                    right_urdf_path, {"right_end_effector": right_ee_mesh_dir},
                    link_name_prefix="right_"))

        self.left_ee_link_names = self._component_link_names(
            "left_end_effector")
        self.right_ee_link_names = self._component_link_names(
            "right_end_effector")

        self.component_link_mesh = [
            (collision.component_name, collision.link_name, collision.mesh_path)
            for collisions in self.component_collisions.values()
            for collision in collisions
        ]

        self.coal_collision_ = CoalCollision(self.component_link_mesh, verbose)

        self.supported_obstacle_types = self.coal_collision_.get_supported_obstacle_types(
        )

        if left_end_effector_type is not None:
            self.coal_collision_.ignore_object_with_component(
                self.left_arm_link_names[-1], "left_end_effector")
        if right_end_effector_type is not None:
            self.coal_collision_.ignore_object_with_component(
                self.right_arm_link_names[-1], "right_end_effector")

    def _component_link_names(self, component_name: str) -> list[str]:
        return [
            collision.link_name
            for collision in self.component_collisions.get(component_name, [])
        ]

    @property
    def coal_collision(self):
        return self.coal_collision_

    def get_robot_link_names(self):
        if self.robot_family == "w1":
            return self.torso_link_names, self.head_link_names, self.left_arm_link_names, self.right_arm_link_names
        elif self.robot_family == "a1":
            pass
        elif self.robot_family == "l1":
            pass
        else:
            self.logger.error(f"Unsupported robot family: {self.robot_family}")
            return None, None, None, None

    def get_end_effector_link_names(self):
        return self.left_ee_link_names, self.right_ee_link_names

    def _return_status_code(self, res: bool):
        if res:
            return StatusCode.SUCCESS
        else:
            return StatusCode.FAIL

    def set_verbose(self, verbose: bool):
        self.coal_collision_.set_verbose(verbose)

    def add_obstacle(self,
                     obstacle_type: str,
                     name: str,
                     position: np.ndarray,
                     quaternion: np.ndarray,
                     obstacle_mesh_path: str = "",
                     size: np.ndarray = None,
                     frame: str = "base_link",
                     is_user_object: bool = False):
        res = self.coal_collision_.add_obstacle(obstacle_type, name, position,
                                                quaternion, obstacle_mesh_path,
                                                size, frame, is_user_object)
        return self._return_status_code(res)

    def add_octomap_obstacle(self,
                             name: str,
                             octotree,
                             world_T_octomap: np.ndarray,
                             frame: str = "base_link",
                             is_user_object: bool = False):
        res = self.coal_collision_.add_octomap_obstacle(name, octotree,
                                                        world_T_octomap, frame,
                                                        is_user_object)
        return self._return_status_code(res)

    def set_obstacle_state(self, name: str, enabled: bool):
        res = self.coal_collision_.set_obstacle_state(name, enabled)
        return self._return_status_code(res)

    def transform_obstacle(self, name: str, position: np.ndarray,
                           quaternion: np.ndarray):
        res = self.coal_collision_.transform_obstacle(name, position,
                                                      quaternion)
        return self._return_status_code(res)

    def get_obstacle_pose(self, name: str):
        return self.coal_collision_.get_obstacle_pose(name)

    def load_environment_from_json(self, json_path: str):
        res = self.coal_collision_.load_environment_from_json(json_path)
        return self._return_status_code(res)

    def load_ignored_collision_pairs_from_json(self, json_path: str):
        res = self.coal_collision_.load_ignored_collision_pairs_from_json(
            json_path)
        return self._return_status_code(res)

    def update_robot_component_pose(self, component_name: str,
                                    link_poses: list[np.ndarray]):
        collisions = self.component_collisions.get(component_name)
        if collisions is None:
            self.logger.error(f"Unknown robot component: {component_name}")
            return self._return_status_code(False)
        if len(link_poses) != len(collisions):
            self.logger.error(f"Pose count for component '{component_name}' is "
                              f"{len(link_poses)}, expected {len(collisions)}.")
            return self._return_status_code(False)

        collision_poses = [
            np.asarray(world_T_link) @ collision.link_T_collision
            for world_T_link, collision in zip(link_poses, collisions)
        ]
        res = self.coal_collision_.update_robot_component_pose(
            component_name, collision_poses)
        return self._return_status_code(res)

    def attach_object_to_component(self, component_name: str, object_name: str,
                                   component_flange_T_object: np.ndarray):
        res = self.coal_collision_.attach_object_to_component(
            component_name, object_name, component_flange_T_object)
        return self._return_status_code(res)

    def detach_object_from_component(self, component_name: str,
                                     component_flange_pose: np.ndarray):
        res = self.coal_collision_.detach_object_from_component(
            component_name, component_flange_pose)
        return self._return_status_code(res)

    def ignore_obstacle_with_component(self, obstacle_name: str,
                                       component_name: str):
        res = self.coal_collision_.ignore_obstacle_with_component(
            obstacle_name, component_name)
        return self._return_status_code(res)

    def ignore_object_with_component(self, object_name: str,
                                     component_name: str):
        res = self.coal_collision_.ignore_object_with_component(
            object_name, component_name)
        return self._return_status_code(res)

    def ignore_objects_collision(self, object_name1: str, object_name2: str):
        res = self.coal_collision_.ignore_objects_collision(
            object_name1, object_name2)
        return self._return_status_code(res)

    def check_component_collision_with_obstacles(self,
                                                 component_name: str = None):
        if component_name is None:
            res = self.coal_collision_.check_component_collision_with_obstacles(
            )
        else:
            res = self.coal_collision_.check_component_collision_with_obstacles(
                component_name)
        return self._return_status_code(res)

    def check_components_collision(self,
                                   component_name1: str = None,
                                   component_name2: str = None):
        if component_name1 is None and component_name2 is None:
            res = self.coal_collision_.check_components_collision()
        elif component_name1 is not None and component_name2 is not None:
            res = self.coal_collision_.check_components_collision(
                component_name1, component_name2)
        else:
            self.logger.error(
                "Both component_name1 and component_name2 should be provided or both should be None."
            )
            return StatusCode.INPUT_INVALID
        return self._return_status_code(res)

    def check_component_self_collision(self, component_name: str = None):
        if component_name is None:
            res = self.coal_collision_.check_component_self_collision()
        else:
            res = self.coal_collision_.check_component_self_collision(
                component_name)
        return self._return_status_code(res)

    def check_link_collision_with_obstacle(self, component_name: str,
                                           link_name: str, obstacle_name: str):
        res = self.coal_collision_.check_link_collision_with_obstacle(
            component_name, link_name, obstacle_name)
        return self._return_status_code(res)

    def check_objects_collision(self, object_name1: str, object_name2: str):
        res = self.coal_collision_.check_objects_collision(
            object_name1, object_name2)
        return self._return_status_code(res)

    def check_collision(self, robot_name: str = "whole_body"):
        # Note: return FAIL when collision is detected, return SUCCESS when no collision is detected
        # Ignore the self-collision of the robot's end effectors

        if robot_name == "whole_body":
            res1 = self.coal_collision_.check_component_collision_with_obstacles(
            )
            if res1:
                return StatusCode.FAIL
            res2 = self.coal_collision_.check_components_collision()
            if res2:
                return StatusCode.FAIL
            if self.robot_family == "w1":
                res3 = self.coal_collision_.check_component_self_collision(
                    "torso")
                res4 = self.coal_collision_.check_component_self_collision(
                    "head")
                res5 = self.coal_collision_.check_component_self_collision(
                    "left_arm")
                res6 = self.coal_collision_.check_component_self_collision(
                    "right_arm")
                if res3 or res4 or res5 or res6:
                    return StatusCode.FAIL
            elif self.robot_family == "a1":
                pass
            elif self.robot_family == "l1":
                pass
            return StatusCode.SUCCESS
        elif robot_name == "left_arm":
            if self.robot_family == "w1":
                res1 = self.coal_collision_.check_component_collision_with_obstacles(
                    "left_arm")
                if res1:
                    return StatusCode.FAIL
                res2 = self.coal_collision_.check_component_collision_with_obstacles(
                    "torso")
                if res2:
                    return StatusCode.FAIL
                res3 = self.coal_collision_.check_component_collision_with_obstacles(
                    "head")
                if res3:
                    return StatusCode.FAIL
                res4 = self.coal_collision_.check_components_collision(
                    "left_arm", "right_arm")
                if res4:
                    return StatusCode.FAIL
                res5 = self.coal_collision_.check_components_collision(
                    "left_arm", "torso")
                if res5:
                    return StatusCode.FAIL
                res6 = self.coal_collision_.check_components_collision(
                    "left_arm", "head")
                if res6:
                    return StatusCode.FAIL
                res7 = self.coal_collision_.check_components_collision(
                    "torso", "head")
                if res7:
                    return StatusCode.FAIL
                res8 = self.coal_collision_.check_component_self_collision(
                    "torso")
                if res8:
                    return StatusCode.FAIL
                res9 = self.coal_collision_.check_component_self_collision(
                    "head")
                if res9:
                    return StatusCode.FAIL
                res10 = self.coal_collision_.check_component_self_collision(
                    "left_arm")
                if res10:
                    return StatusCode.FAIL
            elif self.robot_family == "a1":
                pass
            elif self.robot_family == "l1":
                pass
            return StatusCode.SUCCESS

        elif robot_name == "right_arm":
            if self.robot_family == "w1":
                res1 = self.coal_collision_.check_component_collision_with_obstacles(
                    "right_arm")
                if res1:
                    return StatusCode.FAIL
                res2 = self.coal_collision_.check_component_collision_with_obstacles(
                    "torso")
                if res2:
                    return StatusCode.FAIL
                res3 = self.coal_collision_.check_component_collision_with_obstacles(
                    "head")
                if res3:
                    return StatusCode.FAIL
                res4 = self.coal_collision_.check_components_collision(
                    "left_arm", "right_arm")
                if res4:
                    return StatusCode.FAIL
                res5 = self.coal_collision_.check_components_collision(
                    "right_arm", "torso")
                if res5:
                    return StatusCode.FAIL
                res6 = self.coal_collision_.check_components_collision(
                    "right_arm", "head")
                if res6:
                    return StatusCode.FAIL
                res7 = self.coal_collision_.check_components_collision(
                    "torso", "head")
                if res7:
                    return StatusCode.FAIL
                res8 = self.coal_collision_.check_component_self_collision(
                    "torso")
                if res8:
                    return StatusCode.FAIL
                res9 = self.coal_collision_.check_component_self_collision(
                    "head")
                if res9:
                    return StatusCode.FAIL
                res10 = self.coal_collision_.check_component_self_collision(
                    "right_arm")
                if res10:
                    return StatusCode.FAIL
            elif self.robot_family == "a1":
                pass
            elif self.robot_family == "l1":
                pass
            return StatusCode.SUCCESS

    def export_scene_to_ply(self, file_path: str):
        res = self.coal_collision_.export_scene_to_ply(file_path)
        return self._return_status_code(res)

    def find_link_in_component(self, link_name: str):
        res, component_name, link_index = self.coal_collision_.find_link_in_component(
            link_name)
        if res:
            return StatusCode.SUCCESS, component_name, link_index
        else:
            return StatusCode.FAIL, None, None

    #对灵巧手的robot link name除了从文件名获取之外，需要根据side添加前缀防止相同名字
