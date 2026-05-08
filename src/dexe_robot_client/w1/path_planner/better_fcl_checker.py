# ----------------------------------------------------------------------------
# Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import fcl
import numpy as np
import trimesh
import time
import os
import json
from IPython import embed
from scipy.spatial.transform import Rotation as R
from copy import deepcopy
from ...common import LOG_DIR
from dataclasses import dataclass

from dexe_rp.utils.type import ArmType, W1Version, EndEffectorType
from .utils.robot_type import *
from ...common import get_logger


@dataclass
class Obstacle:
    name: str
    fcl_obj: fcl.CollisionObject
    geometry: trimesh.base.Trimesh
    position: np.ndarray
    orientation: np.ndarray
    frame: str
    is_user_object: bool
    is_enabled: bool


@dataclass
class RobotLink:
    name: str
    fcl_obj: fcl.CollisionObject
    geometry: trimesh.base.Trimesh
    position: np.ndarray
    orientation: np.ndarray


class FCLChecker:

    def __init__(self, use_ee: list[bool] = [False, False]):
        """
        Args:
            use_ee (list[bool]): Whether to use end effectors for left and right arms
        """
        self.logger = get_logger()
        self.left_arm_links, self.right_arm_links, self.non_arm_links = [], [], []
        self.left_end_effector_links, self.right_end_effector_links = [], []
        self.obstacles = []

        self.use_ee = use_ee

        self.robot_link_components = [
            "left_arm", "right_arm", "non_arm", "left_end_effector",
            "right_end_effector"
        ]
        self.supported_obstacle_types = ["box", "sphere", "mesh"]
        self.supported_obstacle_frames = [
            "base_link", "world", "left_flange", "right_flange"
        ]
        self.supported_checking_modes = ["whole_body", "left_arm", "right_arm"]

        for name in self.robot_link_components:
            setattr(self, f"{name}_fcl_objs", [])
        self.obstacles_fcl_objs = []

        for name in self.robot_link_components:
            setattr(self, f"{name}_manager",
                    fcl.DynamicAABBTreeCollisionManager())
        self.obstacles_manager = fcl.DynamicAABBTreeCollisionManager()

        self.robot_link_name_objects, self.obstacles_name_objects = {}, {}
        self.robot_link_objects_name, self.obstacles_objects_name = {}, {}

        self.left_arm_attached, self.right_arm_attached = False, False
        self.left_flange_T_object, self.right_flange_T_object = None, None
        self.left_attached_object_name, self.right_attached_object_name = None, None

        self.enable_visualization = False

    @staticmethod
    def init_from_zip(self, zip_path: str):
        pass

    @staticmethod
    def init_from_W1Manager(robot_type: RobotType,
                            end_effector_type: EndEffectorType):
        if robot_type is None or not isinstance(robot_type, RobotType):
            valid_types = ",".join(rt.name for rt in RobotType)
            raise ValueError(
                f"robot_type must be an instance of RobotType, {valid_types}")
        if end_effector_type is not None and not isinstance(
                end_effector_type, EndEffectorType):
            valid_types = ",".join(eet.name for eet in EndEffectorType)
            raise ValueError(
                f"end_effector_type must be an instance of EndEffectorType {valid_types} or None"
            )
        if isinstance(end_effector_type, EndEffectorType):
            use_ee = [True, True]
        else:
            use_ee = [False, False]
        left_arm_mesh_paths = get_arml_full_path(robot_type=robot_type)
        right_arm_mesh_paths = get_armr_full_path(robot_type=robot_type)
        non_arm_mesh_paths = get_torso_full_path(robot_type=robot_type) + \
                                  get_head_full_path(robot_type=robot_type)
        left_end_effector_mesh_paths, right_end_effector_mesh_paths = \
            get_ee_full_path(robot_type=robot_type, ee_type=end_effector_type)

        fcl_checker = FCLChecker(use_ee)
        fcl_checker.create_robot_objects(left_arm_mesh_paths,
                                         right_arm_mesh_paths,
                                         non_arm_mesh_paths,
                                         left_end_effector_mesh_paths,
                                         right_end_effector_mesh_paths)
        fcl_checker._register_AABB_managers()
        return fcl_checker

    def _mesh_2_robot_link(self, obj_path):
        """
        Transfer a mesh file(OBJ or STL) to FCL collision object
        """
        mesh = trimesh.load(obj_path, force='mesh')
        vertices = np.asarray(mesh.vertices, dtype=np.float64)
        triangles = np.asarray(mesh.faces, dtype=np.int32)
        # self.logger.debug(f"vertices length {len(vertices)}, triangles length {len(triangles)}")

        bvh = fcl.BVHModel()
        bvh.beginModel(len(vertices), len(triangles))
        bvh.addSubModel(vertices, triangles)
        bvh.endModel()

        rotation = np.eye(3, dtype=np.float64)
        translation = np.zeros(3, dtype=np.float64).reshape(3, 1)
        fcl_transform = fcl.Transform(rotation, translation)
        fcl_obj = fcl.CollisionObject(bvh, fcl_transform)

        self.robot_link_objects_name[fcl_obj] = os.path.splitext(
            os.path.basename(obj_path))[0]
        self.robot_link_name_objects[os.path.splitext(
            os.path.basename(obj_path))[0]] = fcl_obj
        robot_link = RobotLink(name=self.robot_link_objects_name[fcl_obj],
                               fcl_obj=fcl_obj,
                               geometry=mesh,
                               position=np.zeros(3, dtype=np.float64),
                               orientation=np.eye(3, dtype=np.float64))
        return robot_link

    def create_robot_objects(
            self,
            left_arm_mesh_paths: list[str] | None = None,
            right_arm_mesh_paths: list[str] | None = None,
            non_arm_mesh_paths: list[str] | None = None,
            left_end_effector_mesh_paths: list[str] | None = None,
            right_end_effector_mesh_paths: list[str] | None = None):
        mapping = {
            "left_arm": left_arm_mesh_paths,
            "right_arm": right_arm_mesh_paths,
            "non_arm": non_arm_mesh_paths,
            "left_end_effector": left_end_effector_mesh_paths,
            "right_end_effector": right_end_effector_mesh_paths
        }
        if self.use_ee[0] != (left_end_effector_mesh_paths is not None):
            raise ValueError(
                "left_end_effector_mesh_paths should be provided when use_ee[0] is True"
            )
        if self.use_ee[1] != (right_end_effector_mesh_paths is not None):
            raise ValueError(
                "right_end_effector_mesh_paths should be provided when use_ee[1] is True"
            )
        for name, mesh_paths in mapping.items():
            if mesh_paths is not None:
                links_list: list[RobotLink] = getattr(self, f"{name}_links")
                fcl_obj_list: list[fcl.CollisionObject] = getattr(
                    self, f"{name}_fcl_objs")
                for mesh_path in mesh_paths:
                    robot_link = self._mesh_2_robot_link(mesh_path)
                    links_list.append(robot_link)
                    fcl_obj_list.append(robot_link.fcl_obj)

    def _register_AABB_managers(self):
        for name in self.robot_link_components:
            manager: fcl.DynamicAABBTreeCollisionManager = getattr(
                self, f"{name}_manager")
            fcl_obj_list: list = getattr(self, f"{name}_fcl_objs")
            for fcl_obj in fcl_obj_list:
                manager.registerObject(fcl_obj)
            manager.setup()

    def add_obstacle(self,
                     obstacle_type: str,
                     name: str,
                     center: np.ndarray,
                     quaternion: np.ndarray = np.array([1, 0, 0, 0]),
                     obstacle_mesh_path: str = None,
                     size: np.ndarray = None,
                     frame: str = "base_link",
                     is_user_object: bool = False):
        if name in self.obstacles_name_objects:
            raise ValueError(f"Obstacle with name {name} already exists.")
        if obstacle_type not in self.supported_obstacle_types:
            raise ValueError(
                f"Unsupported obstacle type {obstacle_type}. Supported types are: {self.supported_obstacle_types}"
            )
        if obstacle_type == "mesh":
            if obstacle_mesh_path is None or not os.path.exists(
                    obstacle_mesh_path):
                raise ValueError(
                    f"Try to add mesh obstacle, but obstacle_mesh_path is invalid: {obstacle_mesh_path}"
                )
            mesh = trimesh.load(obstacle_mesh_path, force='mesh')
            vertices = np.asarray(mesh.vertices, dtype=np.float64)
            triangles = np.asarray(mesh.faces, dtype=np.int32)
            bvh = fcl.BVHModel()
            bvh.beginModel(len(vertices), len(triangles))
            bvh.addSubModel(vertices, triangles)
            bvh.endModel()
            geometry = bvh
            vis_geometry = mesh
        if obstacle_type == "box":
            if size is None or size.shape != (3,):
                raise ValueError(
                    f"Try to add box obstacle, expected size is a (3, ) ndarray, but got: {size}"
                )
            box = fcl.Box(size[0], size[1], size[2])
            geometry = box
            vis_geometry = trimesh.creation.box(extents=size)
        if obstacle_type == "sphere":
            if size is None or size.shape != (1,):
                raise ValueError(
                    f"Try to add sphere obstacle, expected size is a (1, ) ndarray, but got: {size}"
                )
            sphere = fcl.Sphere(size[0])
            geometry = sphere
            vis_geometry = trimesh.creation.icosphere(radius=size[0])

        q_scipy = np.asarray(
            [quaternion[1], quaternion[2], quaternion[3], quaternion[0]])
        rot_mat = R.from_quat(q_scipy).as_matrix().astype(np.float64)
        translation = center.reshape(3, 1).astype(np.float64)
        fcl_transform = fcl.Transform(rot_mat, translation)
        fcl_obj = fcl.CollisionObject(geometry, fcl_transform)

        obstacle = Obstacle(name=name,
                            fcl_obj=fcl_obj,
                            geometry=vis_geometry,
                            position=center,
                            orientation=rot_mat,
                            frame=frame,
                            is_user_object=is_user_object,
                            is_enabled=True)

        self.obstacles.append(obstacle)
        self.obstacles_fcl_objs.append(fcl_obj)
        self.obstacles_name_objects[name] = fcl_obj
        self.obstacles_objects_name[fcl_obj] = name

        self.obstacles_manager.registerObject(fcl_obj)
        self.obstacles_manager.setup()

    def enable_obstacle(self, name: str):
        if name not in self.obstacles_name_objects:
            raise ValueError(f"Obstacle with name {name} does not exist.")
        fcl_obj = self.obstacles_name_objects[name]
        obstacle = next((obs for obs in self.obstacles if obs.name == name),
                        None)
        if obstacle.is_enabled:
            self.logger.warning(f"Obstacle {name} is already enabled.")
            return False
        obstacle.is_enabled = True
        self.obstacles_fcl_objs.append(fcl_obj)
        self.obstacles_manager.registerObject(fcl_obj)
        self.obstacles_manager.setup()
        return True

    def disable_obstacle(self, name: str):
        if name not in self.obstacles_name_objects:
            raise ValueError(f"Obstacle with name {name} does not exist.")
        fcl_obj = self.obstacles_name_objects[name]
        obstacle = next((obs for obs in self.obstacles if obs.name == name),
                        None)
        if not obstacle.is_enabled:
            self.logger.warning(f"Obstacle {name} is already disabled.")
            return False
        obstacle.is_enabled = False
        self.obstacles_fcl_objs.remove(fcl_obj)
        self.obstacles_manager.unregisterObject(fcl_obj)
        self.obstacles_manager.setup()
        return True

    def transform_obstacle(self, name: str, new_center: np.ndarray,
                           new_quaternion: np.ndarray):
        if name not in self.obstacles_name_objects:
            raise ValueError(f"Obstacle with name {name} does not exist.")
        fcl_obj = self.obstacles_name_objects[name]
        obstacle = next((obs for obs in self.obstacles if obs.name == name),
                        None)
        q_scipy = np.asarray([
            new_quaternion[1], new_quaternion[2], new_quaternion[3],
            new_quaternion[0]
        ])
        rot_mat = R.from_quat(q_scipy).as_matrix().astype(np.float64)
        translation = new_center.reshape(3, 1).astype(np.float64)
        fcl_transform = fcl.Transform(rot_mat, translation)
        fcl_obj.setTransform(fcl_transform)
        obstacle.position = new_center
        obstacle.orientation = rot_mat
        self.obstacles_manager.update()

    def get_obstacle_pose(self, name: str):
        if name not in self.obstacles_name_objects:
            self.logger.error(f"Obstalce with name {name} does not exist.")
            return False, None
        obstacle = next((obs for obs in self.obstacles if obs.name == name),
                        None)
        tf = np.eye(4, dtype=np.float64)
        tf[:3, :3] = obstacle.orientation
        tf[:3, 3] = obstacle.position
        return True, tf

    def load_env_json(self, json_path: str):
        if not os.path.exists(json_path):
            self.logger.error(f"JSON file {json_path} does not exist.")
            return False
        self.logger.info(f"Loading environment from JSON file: {json_path}")

        self.obstacles_fcl_objs.clear()
        self.obstacles_manager = fcl.DynamicAABBTreeCollisionManager()

        temp_obstacles_name_objects, temp_obstacles_objects_name = {}, {}
        temp_obstacles = []
        if self.left_arm_attached:
            cur_attached_left_obj = self.obstacles_name_objects[
                self.left_attached_object_name]
            cur_attached_left_obstacle = next(
                (obs for obs in self.obstacles
                 if obs.name == self.left_attached_object_name), None)
            temp_obstacles_name_objects[
                self.left_attached_object_name] = cur_attached_left_obj
            temp_obstacles_objects_name[
                cur_attached_left_obj] = self.left_attached_object_name
            temp_obstacles.append(cur_attached_left_obstacle)
        if self.right_arm_attached:
            cur_attached_right_obj = self.obstacles_name_objects[
                self.right_attached_object_name]
            cur_attached_right_obstacle = next(
                (obs for obs in self.obstacles
                 if obs.name == self.right_attached_object_name), None)
            temp_obstacles_name_objects[
                self.right_attached_object_name] = cur_attached_right_obj
            temp_obstacles_objects_name[
                cur_attached_right_obj] = self.right_attached_object_name
            temp_obstacles.append(cur_attached_right_obstacle)

        self.obstacles.clear()
        for obs in temp_obstacles:
            self.obstacles.append(obs)
        self.obstacles_name_objects.clear()
        self.obstacles_name_objects.update(temp_obstacles_name_objects)
        self.obstacles_objects_name.clear()
        self.obstacles_objects_name.update(temp_obstacles_objects_name)

        with open(json_path, 'r') as f:
            env_data = json.load(f)
        obstacle_cfg = env_data.get("scene_objects", None)
        if obstacle_cfg is not None:
            for obj in obstacle_cfg:
                if obj["shape"]["shape_type"] == "mesh":
                    self.add_obstacle(
                        obstacle_type="mesh",
                        name=obj["name"],
                        center=np.array(obj["pose"])[:3],
                        quaternion=np.array(obj["pose"])[3:],
                        obstacle_mesh_path=obj["shape"]["raw_data"])
                else:
                    self.add_obstacle(obstacle_type=obj["shape"]["shape_type"],
                                      name=obj["name"],
                                      center=np.array(obj["pose"])[:3],
                                      quaternion=np.array(obj["pose"])[3:],
                                      size=np.array(obj["shape"]["raw_data"]))

        user_objects_cfg = env_data.get("user_workobjects", None)
        if user_objects_cfg is not None:
            for obj in user_objects_cfg:
                if obj["shape"]["shape_type"] == "mesh":
                    self.add_obstacle(
                        obstacle_type="mesh",
                        name=obj["name_key"],
                        center=np.array(obj["pose"])[:3],
                        quaternion=np.array(obj["pose"])[3:],
                        obstacle_mesh_path=obj["shape"]["raw_data"],
                        is_user_object=True)
                else:
                    self.add_obstacle(obstacle_type=obj["shape"]["shape_type"],
                                      name=obj["name_key"],
                                      center=np.array(obj["pose"])[:3],
                                      quaternion=np.array(obj["pose"])[3:],
                                      size=np.array(obj["shape"]["raw_data"]),
                                      is_user_object=True)
        self.logger.info(
            f"Loaded {len(self.obstacles)} obstacles (including user objects) from JSON file."
        )

    def update_non_arm_pose(self, non_arm_transforms: list[np.ndarray]):
        """
        Update transformation matrices of non-arm links

        Args:
            non_arm_transforms (list[np.ndarray]): List of 4x4 transformation matrices

        Notes:
            - The order of transformation matrices should match the order of non-arm links
            - For W1 robot, order is [ankle, knee, buttock, waist, neck1, neck2]
        """
        if len(non_arm_transforms) != len(self.non_arm_links):
            raise ValueError(
                f"expected {len(self.non_arm_links)} transforms, but got {len(non_arm_transforms)}"
            )
        for i, transform in enumerate(non_arm_transforms):
            rotation = transform[:3, :3].astype(np.float64)
            translation = transform[0:3, 3].astype(np.float64)
            fcl_transform = fcl.Transform(rotation, translation)
            self.non_arm_fcl_objs[i].setTransform(fcl_transform)
            self.non_arm_links[i].position = translation
            self.non_arm_links[i].orientation = rotation
        self.non_arm_manager.update()

    def update_arm_pose(self,
                        left_arm_transforms: list[np.ndarray],
                        right_arm_transforms: list[np.ndarray],
                        left_ee_transforms: list[np.ndarray] | None = None,
                        right_ee_transforms: list[np.ndarray] | None = None):
        if self.left_arm_attached:
            left_attached_object_transform = left_arm_transforms[
                -1] @ self.left_flange_T_object
            # left_arm_transforms.append(left_attached_object_transform)
        if self.right_arm_attached:
            right_attached_object_transform = right_arm_transforms[
                -1] @ self.right_flange_T_object
            # right_arm_transforms.append(right_attached_object_transform)

        if len(left_arm_transforms) != len(self.left_arm_links):
            raise ValueError(
                f"expected {len(self.left_arm_links)} transforms for left arm, but got {len(left_arm_transforms)}"
            )
        if len(right_arm_transforms) != len(self.left_arm_links):
            raise ValueError(
                f"expected {len(self.left_arm_links)} transforms for right arm, but got {len(right_arm_transforms)}"
            )

        for i, transform in enumerate(left_arm_transforms):
            rotation = transform[:3, :3].astype(np.float64)
            translation = transform[0:3, 3].astype(np.float64)
            fcl_transform = fcl.Transform(rotation, translation)
            self.left_arm_fcl_objs[i].setTransform(fcl_transform)
            self.left_arm_links[i].position = translation
            self.left_arm_links[i].orientation = rotation
        self.left_arm_manager.update()

        if self.left_arm_attached:
            rotation = left_attached_object_transform[:3, :3].astype(np.float64)
            translation = left_attached_object_transform[0:3,
                                                         3].astype(np.float64)
            fcl_transform = fcl.Transform(rotation, translation)
            self.left_arm_fcl_objs[-1].setTransform(fcl_transform)
            attached_obstacle = next(
                (obs for obs in self.obstacles
                 if obs.name == self.left_attached_object_name), None)
            attached_obstacle.position = translation
            attached_obstacle.orientation = rotation
            self.left_arm_manager.update()

        for i, transform in enumerate(right_arm_transforms):
            rotation = transform[:3, :3].astype(np.float64)
            translation = transform[0:3, 3].astype(np.float64)
            fcl_transform = fcl.Transform(rotation, translation)
            self.right_arm_fcl_objs[i].setTransform(fcl_transform)
            self.right_arm_links[i].position = translation
            self.right_arm_links[i].orientation = rotation
        self.right_arm_manager.update()

        if self.right_arm_attached:
            rotation = right_attached_object_transform[:3, :3].astype(
                np.float64)
            translation = right_attached_object_transform[0:3,
                                                          3].astype(np.float64)
            fcl_transform = fcl.Transform(rotation, translation)
            self.right_arm_fcl_objs[-1].setTransform(fcl_transform)
            attached_obstacle = next(
                (obs for obs in self.obstacles
                 if obs.name == self.right_attached_object_name), None)
            attached_obstacle.position = translation
            attached_obstacle.orientation = rotation
            self.right_arm_manager.update()

        if self.use_ee[0]:
            if left_ee_transforms is None or len(left_ee_transforms) != len(
                    self.left_end_effector_links):
                raise ValueError(
                    f"expected {len(self.left_end_effector_links)} transforms for left end effector, but got {len(left_ee_transforms) if left_ee_transforms is not None else 'None'}"
                )
            for i, transform in enumerate(left_ee_transforms):
                rotation = transform[:3, :3].astype(np.float64)
                translation = transform[0:3, 3].astype(np.float64)
                fcl_transform = fcl.Transform(rotation, translation)
                self.left_end_effector_fcl_objs[i].setTransform(fcl_transform)
                self.left_end_effector_links[i].position = translation
                self.left_end_effector_links[i].orientation = rotation
            self.left_end_effector_manager.update()

        if self.use_ee[1]:
            if right_ee_transforms is None or len(right_ee_transforms) != len(
                    self.right_end_effector_links):
                raise ValueError(
                    f"expected {len(self.right_end_effector_links)} transforms for right end effector, but got {len(right_ee_transforms) if right_ee_transforms is not None else 'None'}"
                )
            for i, transform in enumerate(right_ee_transforms):
                rotation = transform[:3, :3].astype(np.float64)
                translation = transform[0:3, 3].astype(np.float64)
                fcl_transform = fcl.Transform(rotation, translation)
                self.right_end_effector_fcl_objs[i].setTransform(fcl_transform)
                self.right_end_effector_links[i].position = translation
                self.right_end_effector_links[i].orientation = rotation
            self.right_end_effector_manager.update()

    def attach_object(self, arm_name: str, object_name: str,
                      flange_T_object: np.ndarray):
        """
        Note:
            - For W1 robot, flange_T_object is actually arml7_T_object or armr7_T_object
        """
        if object_name not in self.obstacles_name_objects:
            self.logger.error(
                f"Cannot attach object {object_name}, it does not exist.")
            return False
        if flange_T_object.shape != (4, 4):
            self.logger.error(
                f"flange_T_object should be a 4x4 transformation matrix, but got shape {flange_T_object.shape}."
            )
            return False
        obstacle = next(
            (obs for obs in self.obstacles if obs.name == object_name), None)
        if not obstacle.is_user_object:
            self.logger.error(
                f"Cannot attach object {object_name}, it is not a user-defined object."
            )
            return False
        if arm_name == "left_arm":
            if self.left_arm_attached:
                self.logger.error("Left arm already has an attached object.")
                return False
            fcl_obj = self.obstacles_name_objects[object_name]
            self.left_arm_attached = True
            self.left_flange_T_object = flange_T_object
            self.left_attached_object_name = object_name

            self.left_arm_fcl_objs.append(fcl_obj)
            self.obstacles_fcl_objs.remove(fcl_obj)
            self.left_arm_manager.registerObject(fcl_obj)
            self.obstacles_manager.unregisterObject(fcl_obj)
            self.left_arm_manager.setup()
            self.obstacles_manager.update()
            self.logger.info(f"Attached object {object_name} to left arm.")
            return True
        elif arm_name == "right_arm":
            if self.right_arm_attached:
                self.logger.error("Right arm already has an attached object.")
                return False
            fcl_obj = self.obstacles_name_objects[object_name]
            self.right_arm_attached = True
            self.right_flange_T_object = flange_T_object
            self.right_attached_object_name = object_name

            self.right_arm_fcl_objs.append(fcl_obj)
            self.obstacles_fcl_objs.remove(fcl_obj)
            self.right_arm_manager.registerObject(fcl_obj)
            self.obstacles_manager.unregisterObject(fcl_obj)
            self.right_arm_manager.setup()
            self.obstacles_manager.update()
            self.logger.info(f"Attached object {object_name} to right arm.")
            return True
        else:
            self.logger.error(
                f"Invalid arm name {arm_name}. Use 'left_arm' or 'right_arm'.")
            return False

    def detach_object(self, arm_name: str, flange_transform: np.ndarray):
        if arm_name == "left_arm":
            if not self.left_arm_attached:
                self.logger.error("Left arm has no attached object to detach.")
                return False
            fcl_obj = self.obstacles_name_objects[
                self.left_attached_object_name]
            self.left_arm_attached = False
            object_in_world = flange_transform @ self.left_flange_T_object
            rotation = object_in_world[:3, :3].astype(np.float64)
            translation = object_in_world[0:3, 3].astype(np.float64)
            fcl_transform = fcl.Transform(rotation, translation)
            fcl_obj.setTransform(fcl_transform)

            self.left_flange_T_object = None
            self.left_attached_object_name = None

            self.left_arm_fcl_objs.remove(fcl_obj)
            self.obstacles_fcl_objs.append(fcl_obj)
            self.left_arm_manager.unregisterObject(fcl_obj)
            self.obstacles_manager.registerObject(fcl_obj)
            self.obstacles_manager.setup()
            self.left_arm_manager.update()

            self.logger.info("Detached object from left arm.")
            return True
        elif arm_name == "right_arm":
            if not self.right_arm_attached:
                self.logger.error("Right arm has no attached object to detach.")
                return False
            fcl_obj = self.obstacles_name_objects[
                self.right_attached_object_name]
            self.right_arm_attached = False
            object_in_world = flange_transform @ self.right_flange_T_object
            rotation = object_in_world[:3, :3].astype(np.float64)
            translation = object_in_world[0:3, 3].astype(np.float64)
            fcl_transform = fcl.Transform(rotation, translation)
            fcl_obj.setTransform(fcl_transform)

            self.right_flange_T_object = None
            self.right_attached_object_name = None

            self.right_arm_fcl_objs.remove(fcl_obj)
            self.obstacles_fcl_objs.append(fcl_obj)
            self.right_arm_manager.unregisterObject(fcl_obj)
            self.obstacles_manager.registerObject(fcl_obj)
            self.obstacles_manager.setup()
            self.right_arm_manager.update()

            self.logger.info("Detached object from right arm.")
            return True
        else:
            self.logger.error(
                f"Invalid arm name {arm_name}. Use 'left_arm' or 'right_arm'.")
            return False

    def _is_collide(self, obj1: fcl.CollisionObject, obj2: fcl.CollisionObject,
                    cdata: fcl.CollisionData):
        if cdata.done:
            return True
        ret = fcl.collide(obj1, obj2, cdata.request, cdata.result)
        if ret > 0:
            if obj1 in self.robot_link_objects_name:
                obj1_name = self.robot_link_objects_name[obj1]
            else:
                obj1_name = self.obstacles_objects_name[obj1]
            if obj2 in self.robot_link_objects_name:
                obj2_name = self.robot_link_objects_name[obj2]
            else:
                obj2_name = self.obstacles_objects_name[obj2]
            self.logger.info(
                f"Collision detected between {obj1_name} and {obj2_name}.")
            cdata.done = True
            return True
        return cdata.done

    def check_collision_between_sets(self,
                                     object_set1: list[fcl.CollisionObject],
                                     object_set2: list[fcl.CollisionObject],
                                     cdata: fcl.CollisionData,
                                     set1_start: int = 0,
                                     set1_end: int = -1,
                                     set2_start: int = 0,
                                     set2_end: int = -1):
        if set1_end == -1:
            set1_end = len(object_set1)
        if set2_end == -1:
            set2_end = len(object_set2)

        set1_start = np.clip(set1_start, 0, len(object_set1))
        set1_end = np.clip(set1_end, 0, len(object_set1))
        set2_start = np.clip(set2_start, 0, len(object_set2))
        set2_end = np.clip(set2_end, 0, len(object_set2))

        for i in range(set1_start, set1_end):
            for j in range(set2_start, set2_end):
                if self._is_collide(object_set1[i], object_set2[j], cdata):
                    return True
        return False

    def check_collision_single_set(self,
                                   object_set: list[fcl.CollisionObject],
                                   cdata: fcl.CollisionData,
                                   interval_step: int = 1):
        for i in range(len(object_set)):
            if i + interval_step >= len(object_set):
                break
            for j in range(i + interval_step, len(object_set)):
                if self._is_collide(object_set[i], object_set[j], cdata):
                    return True
        return False

    def check_collision_AABB(self):
        pass

    def check_collision_Naive(self, robot_name: str = "whole_body"):
        if robot_name not in self.supported_checking_modes:
            raise ValueError(
                f"Unsupported robot_name {robot_name}. Supported modes are: {self.supported_checking_modes}"
            )

        req = fcl.CollisionRequest()
        res = fcl.CollisionResult()

        req.num_max_contacts = 1
        req.enable_cost = False
        req.gjk_solver_type = fcl.GJKSolverType.GST_INDEP

        cdata = fcl.CollisionData(req, res)

        if robot_name == "whole_body":
            self.check_collision_between_sets(self.left_arm_fcl_objs,
                                              self.obstacles_fcl_objs, cdata)
            if cdata.done:
                return True
            self.check_collision_between_sets(self.left_arm_fcl_objs,
                                              self.non_arm_fcl_objs,
                                              cdata,
                                              set1_start=1)
            if cdata.done:
                return True
            self.check_collision_between_sets(self.right_arm_fcl_objs,
                                              self.obstacles_fcl_objs, cdata)
            if cdata.done:
                return True
            self.check_collision_between_sets(self.right_arm_fcl_objs,
                                              self.non_arm_fcl_objs,
                                              cdata,
                                              set1_start=1)
            if cdata.done:
                return True
            self.check_collision_between_sets(self.non_arm_fcl_objs,
                                              self.obstacles_fcl_objs, cdata)
            if cdata.done:
                return True
            self.check_collision_between_sets(self.left_arm_fcl_objs,
                                              self.right_arm_fcl_objs, cdata)
            if cdata.done:
                return True
            self.check_collision_single_set(self.left_arm_fcl_objs,
                                            cdata,
                                            interval_step=2)
            if cdata.done:
                return True
            self.check_collision_single_set(self.right_arm_fcl_objs,
                                            cdata,
                                            interval_step=2)
            if cdata.done:
                return True

            if self.use_ee[0]:
                self.check_collision_between_sets(
                    self.left_end_effector_fcl_objs, self.obstacles_fcl_objs,
                    cdata)
                if cdata.done:
                    return True
                self.check_collision_between_sets(
                    self.left_end_effector_fcl_objs, self.right_arm_fcl_objs,
                    cdata)
                if cdata.done:
                    return True
                self.check_collision_between_sets(
                    self.left_end_effector_fcl_objs, self.non_arm_fcl_objs,
                    cdata)
                if cdata.done:
                    return True
                self.check_collision_between_sets(
                    self.left_end_effector_fcl_objs,
                    self.left_arm_fcl_objs,
                    cdata,
                    set2_end=len(self.left_arm_fcl_objs) - 2)
                if cdata.done:
                    return True
            if self.use_ee[1]:
                self.check_collision_between_sets(
                    self.right_end_effector_fcl_objs, self.obstacles_fcl_objs,
                    cdata)
                if cdata.done:
                    return True
                self.check_collision_between_sets(
                    self.right_end_effector_fcl_objs, self.left_arm_fcl_objs,
                    cdata)
                if cdata.done:
                    return True
                self.check_collision_between_sets(
                    self.right_end_effector_fcl_objs, self.non_arm_fcl_objs,
                    cdata)
                if cdata.done:
                    return True
                self.check_collision_between_sets(
                    self.right_end_effector_fcl_objs,
                    self.right_arm_fcl_objs,
                    cdata,
                    set2_end=len(self.right_arm_fcl_objs) - 2)
                if cdata.done:
                    return True
        return cdata.done

    def visualize_scene(self):
        scene = trimesh.Scene()

        for name in self.robot_link_components:
            links: list[RobotLink] = getattr(self, f"{name}_links")
            for link in links:
                mesh_copy = link.geometry.copy()
                tf = np.eye(4)
                tf[:3, :3] = link.orientation
                tf[0:3, 3] = link.position
                mesh_copy.apply_transform(tf)
                scene.add_geometry(mesh_copy)

        for obstacle in self.obstacles:
            if obstacle.is_enabled:
                mesh_copy = obstacle.geometry.copy()
                tf = np.eye(4)
                tf[:3, :3] = obstacle.orientation
                tf[0:3, 3] = obstacle.position
                mesh_copy.apply_transform(tf)
                scene.add_geometry(mesh_copy)

        scene.show()
