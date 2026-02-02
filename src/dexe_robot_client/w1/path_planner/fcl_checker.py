'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

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

from ...end_effector_client import EndEffectorType
from .utils.robot_type import *
from ...common import get_logger


class FCLChecker:

    def __init__(self,
                 robot_type: RobotType = None,
                 end_effector_type: EndEffectorType = None):
        """
        Initialize the FCL collision checker for specific robot(arm) type and end effector type.

        Args:
            robot_type(RobotType): Type of the robot (e.g., W1V02_1, W1V02_2).
            end_effector_type(EndEffectorType): Type of the end effector (e.g., PGC_GRIPPER, BRAINCO_HAND).
        """
        if robot_type is None or not isinstance(robot_type, RobotType):
            raise ValueError(
                f"Robot type must be specified as {RobotType.W1V02_1} or {RobotType.W1V02_2}."
            )
        self.arml_full_path = get_arml_full_path(robot_type)
        self.armr_full_path = get_armr_full_path(robot_type)
        self.head_full_path = get_head_full_path(robot_type)
        self.torso_full_path = get_torso_full_path(robot_type)

        self.logger = get_logger()
        # Containers for fcl objects
        self.arml_links = []
        self.armr_links = []
        self.head_links = []
        self.body_links = []
        self.obstacles = []
        self.grasp_objects = []

        self.armlee_links, self.armree_links = [], []

        # Dynamic AABB Managers
        self.arml_manager = fcl.DynamicAABBTreeCollisionManager()
        self.armr_manager = fcl.DynamicAABBTreeCollisionManager()
        self.head_manager = fcl.DynamicAABBTreeCollisionManager()
        self.body_manager = fcl.DynamicAABBTreeCollisionManager()
        self.armlee_manager = fcl.DynamicAABBTreeCollisionManager()
        self.armree_manager = fcl.DynamicAABBTreeCollisionManager()
        self.obstacle_manager = fcl.DynamicAABBTreeCollisionManager()
        self.grasp_objects_manager = fcl.DynamicAABBTreeCollisionManager()

        self.grasp_objects_dict = {}
        self.obstacles_dict = {}

        self.use_ee = False
        if end_effector_type is not None:
            if isinstance(end_effector_type, EndEffectorType):
                self.armlee_full_path, self.armree_full_path = get_ee_full_path(
                    robot_type, end_effector_type)
                # self.arml_full_path.append(self.ee_full_path[0])
                # self.armr_full_path.append(self.ee_full_path[1])
                self.end_effector_type = end_effector_type
                self.use_ee = True
            else:
                raise ValueError(
                    f"Unsupported End Effector Type: {end_effector_type}. Please use PGC_GRIPPER or BRAINCO_HAND or LINKER_HAND."
                )

        self.arml_attached = False
        self.armr_attached = False
        self.arml7_T_object = None
        self.armr7_T_object = None
        self.arml_grasp_obj_name = None
        self.armr_grasp_obj_name = None

        self.obj_name = {}

        self._create_robot_obj()
        self._create_robot_manager()

        self.visualize = False

        # self.temp_obstacle = []

    def _obj2fcl(self, obj_path):
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

        self.obj_name[fcl_obj] = os.path.splitext(os.path.basename(obj_path))[0]
        return fcl_obj

    def _create_robot_obj(self):
        """
        Initialize W1 robot FCL collision objects
        """
        self.arml_links = [self._obj2fcl(path) for path in self.arml_full_path]
        self.armr_links = [self._obj2fcl(path) for path in self.armr_full_path]
        self.head_links = [self._obj2fcl(path) for path in self.head_full_path]
        self.body_links = [self._obj2fcl(path) for path in self.torso_full_path]

        self.armlee_links = [
            self._obj2fcl(path) for path in self.armlee_full_path
        ] if self.use_ee else []
        self.armree_links = [
            self._obj2fcl(path) for path in self.armree_full_path
        ] if self.use_ee else []
        self.logger.debug(
            f"Created FCL Objects: {len(self.arml_links)} left arm links, {len(self.armr_links)} right arm links, {len(self.head_links)} head links, {len(self.body_links)} body links, {len(self.armlee_links)} armlee links, {len(self.armree_links)} armree links"
        )

    def _create_robot_manager(self):
        """
        Register W1 robot FCL collision objects to Dynamic AABB manager
        """
        for link in self.arml_links:
            self.arml_manager.registerObject(link)
        for link in self.armr_links:
            self.armr_manager.registerObject(link)
        for link in self.head_links:
            self.head_manager.registerObject(link)
        for link in self.body_links:
            self.body_manager.registerObject(link)

        if self.use_ee:
            for link in self.armlee_links:
                self.armlee_manager.registerObject(link)
            for link in self.armree_links:
                self.armree_manager.registerObject(link)

        self.arml_manager.setup()
        self.armr_manager.setup()
        self.head_manager.setup()
        self.body_manager.setup()
        self.armlee_manager.setup()
        self.armree_manager.setup()

    def set_head_body_transform(self, head_transforms: list[np.ndarray],
                                body_transforms: list[np.ndarray]):
        """
        Update the transformation matrices of head and body links

        Args:
            head_transforms (list of nd.array, shape=(4, 4)): The transformation matrices of head links.
            body_transforms (list of nd.array, shape=(4, 4)): The transformation matrices of body links.
        Notes:
            - The transformation mtrices are based on the robot base link frame.
            - Update head and torso transformation matrices before checking collisions.
            - Only need to update one time per static planning process.
        """
        if len(head_transforms) != len(self.head_links):
            raise ValueError(
                f"Expected {len(self.head_links)} head transforms, got {len(head_transforms)}"
            )
        if len(body_transforms) != len(self.body_links):
            raise ValueError(
                f"Expected {len(self.body_links)} body transforms, got {len(body_transforms)}"
            )
        for i, link in enumerate(self.head_links):
            rotation = head_transforms[i][:3, :3].astype(np.float64)
            translation = head_transforms[i][:3, 3].astype(np.float64)
            fcl_transform = fcl.Transform(rotation, translation)
            link.setTransform(fcl_transform)
        for i, link in enumerate(self.body_links):
            rotation = body_transforms[i][:3, :3].astype(np.float64)
            translation = body_transforms[i][:3, 3].astype(np.float64)
            fcl_transform = fcl.Transform(rotation, translation)
            link.setTransform(fcl_transform)
        self.head_manager.update()
        self.body_manager.update()

    def add_cube_obstacle(self,
                          center: np.ndarray,
                          size: np.ndarray,
                          quaternion: np.ndarray = np.array([1, 0, 0, 0]),
                          name: str = None):
        """
        Add a cube obstacle to the FCL environment

        Args:
            center (nd.array, shape=(3, )): The center position of the cube obstacle
            size (nd.array, shape=(3, )): The size of the cube obstacle in (length, width, height)
            quaternion (nd.array, shape=(4, )): The orientation of the cube obstacle in (w, x, y, z) format

        Notes:
            - The center and quaternion are based on the robot base link frame.
        """

        if name in self.obstacles_dict:
            self.logger.error(
                f"Adding Obstacle failed: obstacle with name {name} already exists"
            )
            return False

        box = fcl.Box(size[0], size[1], size[2])
        q_scipy = np.array(
            [quaternion[1], quaternion[2], quaternion[3], quaternion[0]])
        rot_mat = R.from_quat(q_scipy).as_matrix()
        transform = fcl.Transform(rot_mat, center)
        fcl_obj = fcl.CollisionObject(box, transform)

        self.obstacles.append(fcl_obj)
        self.obstacle_manager.registerObject(fcl_obj)
        self.obstacle_manager.setup()

        self.obstacles_dict[name] = fcl_obj

        # for visualize
        if self.visualize:
            vis_trans = np.eye(4)
            vis_trans[:3, 3] = center
            vis_trans[:3, :3] = rot_mat
            vis_box = trimesh.creation.box(extents=size)
            vis_box.apply_transform(vis_trans)
            self.obstacle_mesh_list.append(vis_box)

    def add_sphere_obstacle(self,
                            center: np.ndarray,
                            radius: float,
                            quaternion: np.ndarray = np.array([1, 0, 0, 0]),
                            name: str = None):
        """
        Add a sphere obstacle to the FCL environment

        Args:
            center (nd.array, shape=(3, )): The center position of the sphere obstacle
            radius (float): The radius of the sphere obstacle
            quaternion (nd.array, shape=(4, )): The orientation of the sphere obstacle in (w, x, y, z) format

        Notes:
            - The center and quaternion are based on the robot base link frame.
        """

        if name in self.obstacles_dict:
            self.logger.error(
                f"Adding Obstacle failed: obstacle with name {name} already exists"
            )
            return False

        sphere = fcl.Sphere(radius)
        q_scipy = np.array(
            [quaternion[1], quaternion[2], quaternion[3], quaternion[0]])
        rot_mat = R.from_quat(q_scipy).as_matrix()
        transform = fcl.Transform(rot_mat, center)
        fcl_obj = fcl.CollisionObject(sphere, transform)

        self.obstacles.append(fcl_obj)
        self.obstacle_manager.registerObject(fcl_obj)
        self.obstacle_manager.setup()

        self.obstacles_dict[name] = fcl_obj

        # for visualize
        if self.visualize:
            vis_trans = np.eye(4)
            vis_trans[:3, 3] = center
            vis_trans[:3, :3] = rot_mat
            vis_sphere = trimesh.creation.icosphere(subdivisions=3,
                                                    radius=radius)
            vis_sphere.apply_transform(vis_trans)
            self.obstacle_mesh_list.append(vis_sphere)

    def add_mesh_obstacle(self,
                          mesh_path: str,
                          center: np.ndarray,
                          quaternion: np.ndarray = np.array([1, 0, 0, 0]),
                          name: str = None):
        """
        Add a mesh obstacle to the FCL environment

        Args:
            mesh_path (str): The file path of the mesh obstacle (OBJ or STL)
            center (nd.array, shape=(3, )): The position of the mesh obstacle(the origin of mesh frame)
            quaternion (nd.array, shape=(4, )): The orientation of the mesh obstacle in (w, x, y, z) format

        Notes:
            - The center and quaternion are based on the robot base link frame.
        """

        if name in self.obstacles_dict:
            self.logger.error(
                f"Adding Obstacle failed: obstacle with name {name} already exists"
            )
            return False

        mesh = trimesh.load(mesh_path, force='mesh')
        vertices = np.asarray(mesh.vertices, dtype=np.float64)
        triangles = np.asarray(mesh.faces, dtype=np.int32)

        bvh = fcl.BVHModel()
        bvh.beginModel(len(vertices), len(triangles))
        bvh.addSubModel(vertices, triangles)
        bvh.endModel()

        q_scipy = np.array(
            [quaternion[1], quaternion[2], quaternion[3], quaternion[0]])
        rot_mat = R.from_quat(q_scipy).as_matrix()
        translation = center.reshape(3, 1).astype(np.float64)
        fcl_transform = fcl.Transform(rot_mat, translation)
        fcl_obj = fcl.CollisionObject(bvh, fcl_transform)

        self.obstacles.append(fcl_obj)
        self.obstacle_manager.registerObject(fcl_obj)
        self.obstacle_manager.setup()

        self.obstacles_dict[name] = fcl_obj

        # for visualize
        if self.visualize:
            vis_trans = np.eye(4)
            vis_trans[:3, 3] = center
            vis_trans[:3, :3] = rot_mat
            vis_mesh = deepcopy(mesh)
            # self.temp_obstacle.append(deepcopy(mesh))
            vis_mesh.apply_transform(vis_trans)
            self.obstacle_mesh_list.append(vis_mesh)

    def get_obstacle_pose(self,
                          name: str) -> tuple[bool, np.ndarray, np.ndarray]:
        """
        Get a specific obstacle's pose

        Args:
            name (str): The name of the obstacle or the grasp object

        Return:
            success (bool): Whether the obstacle exists
            rotation (nd.array, shape=(3, 3)): The rotation matrix of the obstacle based on the robot base link frame
            translation (nd.array, shape=(3, 3)): The translation vector of the obstacle based on the robot base link frame
        """
        if name not in self.obstacles_dict:
            self.logger.error(f"The obstacle with name {name} does not exist")
            return False, None, None
        fcl_obj = self.obstacles_dict[name]
        rotation = fcl_obj.getRotation()
        translation = fcl_obj.getTranslation()
        return True, rotation, translation

    def get_grasp_object_pose(self,
                              name: str) -> tuple[bool, np.ndarray, np.ndarray]:
        """
        Get a specific grasp object's pose

        Args:
            name (str): The name of the obstacle or the grasp object

        Return:
            success (bool): Whether the grasp object exists
            rotation (nd.array, shape=(3, 3)): The rotation matrix of the grasp object based on the robot base link frame
            translation (nd.array, shape=(3, 3)): The translation vector of the grasp object based on the robot base link frame
        """
        if name not in self.grasp_objects_dict:
            self.logger.error(
                f"The grasp object with name {name} does not exist")
            return False, None, None
        fcl_obj = self.grasp_objects_dict[name]
        rotation = fcl_obj.getRotation()
        translation = fcl_obj.getTranslation()
        return True, rotation, translation

    def transform_obstacle(self, name: str, new_center: np.ndarray,
                           new_quaternion: np.ndarray):
        """
        Transform an existing obstacle to a new position and orientation

        Args:
            name (str): The name of the obstacle to be transformed
            new_center (nd.array, shape=(3, )): The new center position of the obstacle
            new_quaternion (nd.array, shape=(4, )): The new orientation of the obstacle in (w, x, y, z) format

        Notes:
            - The new center and quaternion are based on the robot base link frame.
        """

        if name not in self.obstacles_dict:
            self.logger.error(f"The obstacle with name {name} does not exist")
            return False
        fcl_obj = self.obstacles_dict[name]
        q_scipy = np.array([
            new_quaternion[1], new_quaternion[2], new_quaternion[3],
            new_quaternion[0]
        ])
        rot_mat = R.from_quat(q_scipy).as_matrix()
        translation = new_center.reshape(3, 1).astype(np.float64)
        fcl_transform = fcl.Transform(rot_mat, translation)
        fcl_obj.setTransform(fcl_transform)

        self.obstacle_manager.update()

        # if self.visualize:
        #     vis_trans = np.eye(4)
        #     vis_trans[:3, 3] = new_center
        #     vis_trans[:3, :3] = rot_mat
        #     vis_mesh = deepcopy(self.temp_obstacle[-1])
        #     vis_mesh.apply_transform(vis_trans)
        #     self.obstacle_mesh_list.append(vis_mesh)

        return True

    def disable_obstacle(self, name: str):
        """
        Disable an exsiting obstacle from collision checking

        Args:
            name (str): The name of the obstacle to be disabled
        """

        if name not in self.obstacles_dict:
            self.logger.error(f"The obstacle with name {name} does not exist")
            return False
        fcl_obj = self.obstacles_dict[name]
        self.obstacle_manager.unregisterObject(fcl_obj)
        self.obstacle_manager.setup()
        return True

    def enable_obstacle(self, name: str):
        """
        Enable an exsiting obstacle for collision checking

        Args:
            name (str): The name of the obstacle to be enabled
        """
        if name not in self.obstacles_dict:
            self.logger.error(f"The obstacle with name {name} does not exist")
            return False
        fcl_obj = self.obstacles_dict[name]
        self.obstacle_manager.registerObject(fcl_obj)
        self.obstacle_manager.setup()
        return True

    def add_octomap_from_file(self,
                              ply_path: str,
                              extrinsics: np.ndarray,
                              base_link_T_neck2: np.ndarray,
                              octo_res: float = 0.01):
        """
        This function loads a point cloud file and transform it to base link frame, then create an octomap for collision checking

        Args:
            ply_path (str): The point cloud file path in PLY format which is in camera frame
            extrinsics (nd.array, shape=(4, 4)): The transformation matrix from the camera frame to neck2 frame.
            base_link_T_neck2 (np.ndarray, shape=(4, 4)): The transformation matrix from neck2 frame to base link frame
            octo_res (float): The resolution of the octomap.
        """

        # self.obstacles.clear()
        # self.obstacle_manager = fcl.DynamicAABBTreeCollisionManager()

        point_cloud = trimesh.load(ply_path, process=False)
        point_cloud = np.asarray(point_cloud.vertices)
        # Apply transformation to the base link frame
        base_link_T_camera = base_link_T_neck2 @ extrinsics
        point_cloud_h = np.hstack(
            [point_cloud, np.ones((point_cloud.shape[0], 1))])
        point_cloud = (base_link_T_camera @ point_cloud_h.T).T[:, :3]
        # Downsample the point cloud
        coords = np.floor(point_cloud / 0.01).astype(np.int64)
        keys = ((coords[:, 0] << 42) + (coords[:, 1] << 21) + coords[:, 2])
        _, idx = np.unique(keys, return_index=True)
        point_cloud = point_cloud[idx]

        point_cloud_array = np.asarray(point_cloud)
        octo_obj = fcl.OcTree(r=octo_res, points=point_cloud_array)
        fcl_octo_obj = fcl.CollisionObject(octo_obj)
        self.obstacles.append(fcl_octo_obj)
        self.obstacle_manager.registerObject(fcl_octo_obj)
        self.obstacle_manager.setup()

        if self.visualize:
            trimesh_cloud = trimesh.points.PointCloud(point_cloud_array)
            self.obstacle_mesh_list.append(trimesh_cloud)

    def add_octomap_from_numpy(self, point_cloud: np.ndarray,
                               extrinsics: np.ndarray,
                               base_link_T_neck2: np.ndarray, octo_res: float):
        """
        This function loads a numpy array point cloud and transform it to base link frame, then create an octomap for collision checking

        Args:
            point_cloud (nd.array, shape=(N, 3)): The point cloud data in numpy array format which is in camera frame.
            extrinsics (nd.array, shape=(4, 4)): The transformation matrix from the camera frame to neck2 frame.
            octo_res (float): The resolution of the octomap.
        """

        # self.obstacles.clear()
        # self.obstacle_manager = fcl.DynamicAABBTreeCollisionManager()

        base_link_T_camera = base_link_T_neck2 @ extrinsics
        point_cloud_h = np.hstack(
            [point_cloud, np.ones((point_cloud.shape[0], 1))])
        point_cloud = (base_link_T_camera @ point_cloud_h.T).T[:, :3]

        coords = np.floor(point_cloud / 0.01).astype(np.int64)
        keys = ((coords[:, 0] << 42) + (coords[:, 1] << 21) + coords[:, 2])
        _, idx = np.unique(keys, return_index=True)
        point_cloud = point_cloud[idx]

        point_cloud_array = np.asarray(point_cloud)
        octo_obj = fcl.OcTree(r=octo_res, points=point_cloud_array)
        fcl_octo_obj = fcl.CollisionObject(octo_obj)
        self.obstacles.append(fcl_octo_obj)
        self.obstacle_manager.registerObject(fcl_octo_obj)
        self.obstacle_manager.setup()

        if self.visualize:
            trimesh_cloud = trimesh.points.PointCloud(point_cloud_array)
            self.obstacle_mesh_list.append(trimesh_cloud)

    def load_env_json(self, json_path):
        """
        This function loads environment obstacles and grasp objects from a JSON configuration file

        Args:
            json_path (str): The file path of the JSON configuration file

        Notes:
            - It will clear the current obstacles and grasp objects before loading new ones.
            - It will keep the grasped objects if any arm is currently holding an object and 
                restore them to new grasp_objects dictionary
            - After these grasped objects are dropped, they will be add back to collision checking environment.
        """

        self.logger.info(f"Loading environment from {json_path}")

        # Clear existing obstacles and grasp objects
        self.obstacles.clear()
        self.obstacle_manager = fcl.DynamicAABBTreeCollisionManager()
        self.grasp_objects.clear()
        self.grasp_objects_manager = fcl.DynamicAABBTreeCollisionManager()

        temp_grasp_objects_dict = {}
        if self.arml_attached:
            temp_grasped_obj_l = self.grasp_objects_dict[
                self.arml_grasp_obj_name]
            self.grasp_objects.append(temp_grasped_obj_l)
            temp_grasp_objects_dict[
                self.arml_grasp_obj_name] = temp_grasped_obj_l
        if self.armr_attached:
            temp_grasped_obj_r = self.grasp_objects_dict[
                self.armr_grasp_obj_name]
            self.grasp_objects.append(temp_grasped_obj_r)
            temp_grasp_objects_dict[
                self.armr_grasp_obj_name] = temp_grasped_obj_r

        self.grasp_objects_dict.clear()
        self.grasp_objects_dict.update(temp_grasp_objects_dict)

        self.obstacles_dict.clear()

        with open(json_path, 'r') as f:
            env_data = json.load(f)
        # Load environment obstacles
        obstacle_cfg = env_data.get("scene_objects", None)
        if obstacle_cfg is not None:
            for obj in obstacle_cfg:
                if obj["shape"]["shape_type"] == "box":
                    name = obj["name"]
                    center = np.array(obj["pose"])[:3]
                    size = np.array(obj["shape"]["raw_data"])
                    quaternion = np.array(obj["pose"])[3:]
                    self.add_cube_obstacle(center, size, quaternion, name)
                    self.logger.info(
                        f"Successfully add a {obj['shape']['shape_type']} obstacle at center: {center}, with size: {size} and name: {obj['name']}"
                    )
                if obj["shape"]["shape_type"] == "sphere":
                    name = obj["name"]
                    center = np.array(obj["pose"])[:3]
                    radius = obj["shape"]["raw_data"][0]
                    quaternion = np.array(obj["pose"])[3:]
                    self.add_sphere_obstacle(center, radius, quaternion, name)
                    self.logger.info(
                        f"Successfully add a {obj['shape']['shape_type']} obstacle at center: {center}, with radius: {radius} and name: {obj['name']}"
                    )
                if obj["shape"]["shape_type"] == "mesh":
                    name = obj["name"]
                    obj_mesh_path = obj["shape"]["raw_data"]
                    if not os.path.exists(obj_mesh_path):
                        raise ValueError(
                            f"Obstacle mesh path {obj_mesh_path} does not exist."
                        )
                    center = np.array(obj["pose"])[:3]
                    quaternion = np.array(obj["pose"])[3:]
                    self.add_mesh_obstacle(obj_mesh_path, center, quaternion,
                                           name)
                    self.logger.info(
                        f"Successfully add a {obj['shape']['shape_type']} obstacle with mesh path: {obj_mesh_path} at center: {center} and name: {obj['name']}"
                    )

        # Load environment grasp objects
        grasp_cfg = env_data.get("user_workobjects", None)
        if grasp_cfg is not None:
            for grasp_obj in grasp_cfg:
                if grasp_obj["shape"]["shape_type"] == "box":
                    center = np.array(grasp_obj["pose"])[:3]
                    quaternion = np.array(grasp_obj["pose"])[3:]
                    size = np.array(grasp_obj["shape"]["raw_data"])
                    name = grasp_obj["name_key"]
                    self.add_grasp_object(obj_type="box",
                                          center=center,
                                          quaternion=quaternion,
                                          size=size,
                                          name=name)
                    self.logger.info(
                        f"Successfully add a {grasp_obj['shape']['shape_type']} grasp object"
                    )
                if grasp_obj["shape"]["shape_type"] == "sphere":
                    center = np.array(grasp_obj["pose"])[:3]
                    quaternion = np.array(grasp_obj["pose"])[3:]
                    radius = grasp_obj["shape"]["raw_data"][0]
                    name = grasp_obj["name_key"]
                    self.add_grasp_object(obj_type="sphere",
                                          center=center,
                                          quaternion=quaternion,
                                          size=np.array([radius]),
                                          name=name)
                    self.logger.info(
                        f"Successfully add a {grasp_obj['shape']['shape_type']} grasp object"
                    )
                if grasp_obj["shape"]["shape_type"] == "mesh":
                    grasp_obj_mesh_path = grasp_obj["shape"]["raw_data"]
                    if not os.path.exists(grasp_obj_mesh_path):
                        raise ValueError(
                            f"Grasp object mesh path {grasp_obj_mesh_path} does not exist."
                        )
                    center = np.array(grasp_obj["pose"])[:3]
                    quaternion = np.array(grasp_obj["pose"])[3:]
                    name = grasp_obj["name_key"]
                    self.add_grasp_object(obj_type="mesh",
                                          objMesh_path=grasp_obj_mesh_path,
                                          center=center,
                                          quaternion=quaternion,
                                          name=name)
                    self.logger.info(
                        f"Successfully add a {grasp_obj['shape']['shape_type']} grasp object with mesh path: {grasp_obj_mesh_path}"
                    )

    def add_grasp_object(self,
                         obj_type: str = None,
                         objMesh_path: str = None,
                         center: np.ndarray = None,
                         quaternion: np.ndarray = np.array([1, 0, 0, 0]),
                         size: np.ndarray = None,
                         name: str = None):
        """
        Add a grasp object to the FCL environment for collision checking

        Args:
            obj_type (str): "box", "sphere", "mesh"
            objMesh_path: Path to the mesh file if obj_type is "mesh"
            center (nd.array, shape=(3, )): The center position of the grasp object
            quaternion (nd.array, shape=(4, )): The orientation of the grasp object in (w, x, y, z) format
            size (nd.array): The size of the grasp object. 
                                For box, it should be (length, width, height). 
                                For sphere, it should be (radius, ).
            name (str): The unique name of the grasp object

        Notes:
            - The center and quaternion are based on the robot base link frame.
        """
        fcl_obj = None
        if obj_type is None or obj_type not in ["box", "sphere", "mesh"]:
            raise ValueError(
                "Object type must be one of 'box', 'sphere', or 'mesh'.")
        if obj_type == "mesh":
            if objMesh_path is None or not os.path.exists(objMesh_path):
                raise ValueError(
                    "Try to add a grasp mesh object, but the mesh path is None or does not exist."
                )
            mesh = trimesh.load(objMesh_path, force='mesh')
            vertices = np.asarray(mesh.vertices, dtype=np.float64)
            triangles = np.asarray(mesh.faces, dtype=np.int32)
            bvh = fcl.BVHModel()
            bvh.beginModel(len(vertices), len(triangles))
            bvh.addSubModel(vertices, triangles)
            bvh.endModel()
            q_scipy = np.array(
                [quaternion[1], quaternion[2], quaternion[3], quaternion[0]])
            rot_mat = R.from_quat(q_scipy).as_matrix()
            translation = center.reshape(3, 1).astype(np.float64)
            fcl_transform = fcl.Transform(rot_mat, translation)
            fcl_obj = fcl.CollisionObject(bvh, fcl_transform)
        elif obj_type == "box":
            if center is None or size is None or len(size) != 3:
                raise ValueError(
                    "Try to add a box object, but center or size is None or size is not a 3-element array."
                )
            box = fcl.Box(size[0], size[1], size[2])
            q_scipy = np.array(
                [quaternion[1], quaternion[2], quaternion[3], quaternion[0]])
            rot_mat = R.from_quat(q_scipy).as_matrix()
            transform = fcl.Transform(rot_mat, center)
            fcl_obj = fcl.CollisionObject(box, transform)
        elif obj_type == "sphere":
            if center is None or size is None or len(size) != 1:
                raise ValueError(
                    "Try to add a sphere object, but center or size is None or size is not a 1-element array(radius)"
                )
            sphere = fcl.Sphere(size[0])
            q_scipy = np.array(
                [quaternion[1], quaternion[2], quaternion[3], quaternion[0]])
            rot_mat = R.from_quat(q_scipy).as_matrix()
            transform = fcl.Transform(rot_mat, center)
            fcl_obj = fcl.CollisionObject(sphere, transform)

        if name in self.grasp_objects_dict:
            raise ValueError(
                f"Grasp object with name {name} already exists. Please use a different name."
            )
        self.grasp_objects_dict[name] = fcl_obj
        self.grasp_objects.append(fcl_obj)
        self.grasp_objects_manager.registerObject(fcl_obj)
        self.grasp_objects_manager.setup()

    def attach_object_arml(self,
                           obj_name: str = None,
                           arml7_T_object: np.ndarray = None):
        """
        Attach a grasp object to the left arm flange for collision checking

        Args:
            obj_name (str): The unique name of the grasp object to be attached
            arml7_T_object (nd.array, shape=(4, 4)): The transformation matrix from objects to left arm flange
        """
        if obj_name is None or obj_name not in self.grasp_objects_dict:
            self.logger.error(
                f"Object with name {obj_name} does not exist in grasp objects.")
            return False
        if arml7_T_object is None or arml7_T_object.shape != (4, 4):
            self.logger.error(
                "Flange to object transformation must be a 4x4 matrix.")
            return False

        fcl_obj = self.grasp_objects_dict[obj_name]
        if self.arml_attached:
            self.logger.error(
                "Left arm already attached to an object. Please detach first.")
            return False
        self.arml_attached = True
        self.arml7_T_object = arml7_T_object
        self.arml_grasp_obj_name = obj_name

        self.arml_links.append(fcl_obj)
        self.grasp_objects_manager.unregisterObject(fcl_obj)
        self.arml_manager.registerObject(fcl_obj)
        self.arml_manager.setup()
        self.grasp_objects_manager.update()

        self.logger.info(
            f"After attaching object, left arm manager has {len(self.arml_manager.getObjects())} objects, grasp objects manager has {len(self.grasp_objects_manager.getObjects())} objects."
        )
        return True

    def detach_object_arml(self,
                           obj_name: str = None,
                           arml7_transform: np.ndarray = None):
        """
        Detach the grasp object from the left arm flange

        Args:
            - obj_name (str): The unique name of the grasp object to be detached
            - arml7_transform (np.ndarray, shape=(4, 4)): The transformation matrix of left arm flange in robot base link frame
        """
        if obj_name is None or obj_name not in self.grasp_objects_dict:
            self.logger.error(
                f"Object with name {obj_name} does not exist in grasp objects.")
            return False
        if arml7_transform is not None and arml7_transform.shape != (4, 4):
            self.logger.error("Arml7 transform must be a 4x4 matrix.")
            return False

        fcl_obj = self.grasp_objects_dict[obj_name]
        if not self.arml_attached:
            self.logger.error("Left arm is not attached to any object.")
            return False

        world_transform = arml7_transform @ self.arml7_T_object
        rotation = world_transform[:3, :3].astype(np.float64)
        translation = world_transform[:3, 3].reshape(3, 1).astype(np.float64)
        fcl_transform = fcl.Transform(rotation, translation)
        fcl_obj.setTransform(fcl_transform)

        self.arml_attached = False
        self.arml7_T_object = None
        self.arml_grasp_obj_name = None

        self.arml_links.remove(fcl_obj)
        self.arml_manager.unregisterObject(fcl_obj)
        self.grasp_objects_manager.registerObject(fcl_obj)
        self.grasp_objects_manager.setup()
        self.arml_manager.update()

        self.logger.info(
            f"After detaching object, left arm manager has {len(self.arml_manager.getObjects())} objects, grasp objects manager has {len(self.grasp_objects_manager.getObjects())} objects."
        )
        return True

    def attach_object_armr(self,
                           obj_name: str = None,
                           armr7_T_object: np.ndarray = None):
        """
        Attach a grasp object to the right arm flange for collision checking

        Args:
            obj_name (str): The unique name of the grasp object to be attached
            armr7_T_object (nd.array, shape=(4, 4)): The transformation matrix from objects to right arm flange
        """
        if obj_name is None or obj_name not in self.grasp_objects_dict:
            self.logger.error(
                f"Object with name {obj_name} does not exist in grasp objects.")
            return False
        if armr7_T_object is None or armr7_T_object.shape != (4, 4):
            self.logger.error(
                "Flange to object transformation must be a 4x4 matrix.")
            return False

        fcl_obj = self.grasp_objects_dict[obj_name]
        if self.armr_attached:
            self.logger.error(
                "Right arm already attached to an object. Please detach first.")
            return False
        self.armr_attached = True
        self.armr7_T_object = armr7_T_object
        self.armr_grasp_obj_name = obj_name

        self.armr_links.append(fcl_obj)
        self.grasp_objects_manager.unregisterObject(fcl_obj)
        self.armr_manager.registerObject(fcl_obj)
        self.armr_manager.setup()
        self.grasp_objects_manager.update()

        self.logger.info(
            f"After attaching object, right arm manager has {len(self.armr_manager.getObjects())} objects, grasp objects manager has {len(self.grasp_objects_manager.getObjects())} objects."
        )
        return True

    def detach_object_armr(self,
                           obj_name: str = None,
                           armr7_transform: np.ndarray = None):
        """
        Detach the grasp object from the right arm flange

        Args:
            - obj_name (str): The unique name of the grasp object to be detached
            - arml7_transform (nd.array, shape=(4, 4)): The transformation matrix of right arm flange in robot base link frame
        """
        if obj_name is None or obj_name not in self.grasp_objects_dict:
            self.logger.error(
                f"Object with name {obj_name} does not exist in grasp objects.")
            return False
        if armr7_transform is not None and armr7_transform.shape != (4, 4):
            self.logger.error("Armr7 transform must be a 4x4 matrix.")
            return False

        fcl_obj = self.grasp_objects_dict[obj_name]
        if not self.armr_attached:
            self.logger.error("Right arm is not attached to any object.")
            return False

        world_transform = armr7_transform @ self.armr7_T_object
        rotation = world_transform[:3, :3].astype(np.float64)
        translation = world_transform[:3, 3].reshape(3, 1).astype(np.float64)
        fcl_transform = fcl.Transform(rotation, translation)
        fcl_obj.setTransform(fcl_transform)

        self.armr_attached = False
        self.armr7_T_object = None
        self.armr_grasp_obj_name = None

        self.armr_links.remove(fcl_obj)
        self.armr_manager.unregisterObject(fcl_obj)
        self.grasp_objects_manager.registerObject(fcl_obj)
        self.grasp_objects_manager.setup()
        self.armr_manager.update()

        self.logger.info(
            f"After detaching object, right arm manager has {len(self.armr_manager.getObjects())} objects, grasp objects manager has {len(self.grasp_objects_manager.getObjects())} objects."
        )
        return True

    def update_arm_transform(self,
                             arm_transforms: list[np.ndarray] = None,
                             left_ee_transform: list[np.ndarray] = None,
                             right_ee_transform: list[np.ndarray] = None):
        """
        Update the transformation matrices of dual arms' links, end effectors' links for collision checking

        Args:
            transforms (list of nd.array, shape=(4, 4)): The list of transformation matrices of dual arms' links
            left_ee_transform (list of nd.array, shape=(4, 4)): The transformation matrices of left end effector links.
            right_ee_transform (list of nd.array, shape=(4, 4)): The transformation matrices of right end effector links.
        
        Notes:
            - The transformation mtrices are based on the robot base link frame.
            - The order of arugment arm_transforms is [left_arm_tf, right_arm_tf]
                * left_arm_tf: [arml_link1, arml_link2, arml_link3, arml_link4, arml_link5, arml_link6, arml_link7]
                * right_arm_tf: [armr_link1, armr_link2, armr_link3, armr_link4, armr_link5, armr_link6, armr_link7]
            - The order of end effector transformation matrices:
                * left_ee_transform: [L_base_link, L_index_dist, L_index_prox, L_little_dist, L_little_prox,
                                      L_middle_dist, L_middle_prox, L_ring_dist, L_ring_prox, L_thumb_dist, L_thumb_prox]
                * right_ee_transform: [R_base_link, R_index_dist, R_index_prox, R_little_dist, R_little_prox,
                                       R_middle_dist, R_middle_prox, R_ring_dist, R_ring_prox, R_thumb_dist, R_thumb_prox]
        """
        if self.arml_attached or self.armr_attached:
            if self.arml_attached:
                arml_dof = len(self.arml_links) - 1
                left_transforms = arm_transforms[:arml_dof]
                grasp_object_transform = left_transforms[
                    -1] @ self.arml7_T_object
                left_transforms.append(grasp_object_transform)
            else:
                arml_dof = len(self.arml_links)
                left_transforms = arm_transforms[:arml_dof]
            if self.armr_attached:
                armr_dof = len(self.armr_links) - 1
                right_transforms = arm_transforms[arml_dof:]
                grasp_object_transform = right_transforms[
                    -1] @ self.armr7_T_object
                right_transforms.append(grasp_object_transform)
            else:
                armr_dof = len(self.armr_links)
                right_transforms = arm_transforms[arml_dof:]
            if len(left_transforms) != len(
                    self.arml_links) or len(right_transforms) != len(
                        self.armr_links):
                raise ValueError(
                    f"Expected {len(self.arml_links)} left arm transforms and {len(self.armr_links)} right arm transforms, got {len(left_transforms)} and {len(right_transforms)}"
                )
        else:
            if len(arm_transforms) != (len(self.arml_links) +
                                       len(self.armr_links)):
                raise ValueError(
                    f"Expected {len(self.arml_links) + len(self.armr_links)} transforms, got {len(arm_transforms)}"
                )
            left_transforms = arm_transforms[:len(self.arml_links)]
            right_transforms = arm_transforms[len(self.armr_links):]
        for i, link in enumerate(self.arml_links):
            rotation = left_transforms[i][:3, :3].astype(np.float64)
            translation = left_transforms[i][:3,
                                             3].reshape(3, 1).astype(np.float64)
            fcl_transform = fcl.Transform(rotation, translation)
            link.setTransform(fcl_transform)
        for i, link in enumerate(self.armr_links):
            rotation = right_transforms[i][:3, :3].astype(np.float64)
            translation = right_transforms[i][:3,
                                              3].reshape(3,
                                                         1).astype(np.float64)
            fcl_transform = fcl.Transform(rotation, translation)
            link.setTransform(fcl_transform)
        self.arml_manager.update()
        self.armr_manager.update()

        if self.use_ee:
            if left_ee_transform is None or right_ee_transform is None:
                raise ValueError(
                    "End effector transforms must be provided when using end effectors."
                )
            if len(left_ee_transform) != len(self.armlee_links):
                raise ValueError(
                    f"Expected {len(self.armlee_links)} left end effector transforms, got {len(left_ee_transform)}"
                )
            if len(right_ee_transform) != len(self.armree_links):
                raise ValueError(
                    f"Expected {len(self.armree_links)} right end effector transforms, got {len(right_ee_transform)}"
                )
            for i, link in enumerate(self.armlee_links):
                rotation = left_ee_transform[i][:3, :3].astype(np.float64)
                translation = left_ee_transform[i][:3, 3].astype(np.float64)
                fcl_transform = fcl.Transform(rotation, translation)
                link.setTransform(fcl_transform)
            for i, link in enumerate(self.armree_links):
                rotation = right_ee_transform[i][:3, :3].astype(np.float64)
                translation = right_ee_transform[i][:3, 3].astype(np.float64)
                fcl_transform = fcl.Transform(rotation, translation)
                link.setTransform(fcl_transform)

            self.armlee_manager.update()
            self.armree_manager.update()

    def _is_collide(self, obs1, obs2, cdata):
        """
        Collision callback function
        """
        if cdata.done:
            return True
        ret = fcl.collide(obs1, obs2, cdata.request, cdata.result)
        if ret > 0:
            print(
                self.obj_name.get(obs1, "unknown") + "  vs  " +
                self.obj_name.get(obs2, "unknown"))
            cdata.done = True
            return True
        return cdata.done

    def check_collision(self,
                        robot_name: str = "whole_body",
                        check_between_end_effectors: bool = True) -> bool:
        """
        The main collision checking function

        Args:
            robot_name (str): "whole_body", "left_arm", "right_arm"
        
        Notes:
            - "whole_body" check collision between:
                * dual arms vs obstacles & grasp objects
                * self arm vs self arm
                * dual arms vs body(except arm link 1) & head & other arm
                * end effectors vs obstacles & grasp objects (if use_ee is True)
                * end effectors vs body & head & other arm & self arm(if use_ee is True)
            - "left_arm" check collision between:
                * left arm vs obstacles & grasp objects
                * self arm vs self arm
                * left arm vs body(except arm link 1) & head & right arm
                * left end effector vs obstacles & grasp objects (if use_ee is True)
                * left end effector vs body & head & right arm & self arm(if use_ee is True)
            - "right_arm" check collision between:
                * right arm vs obstacles & grasp objects
                * self arm vs self arm
                * right arm vs body(except arm link 1) & head & left arm
                * right end effector vs obstacles & grasp objects (if use_ee is True)
                * right end effector vs body & head & left arm & self arm(if use_ee is True)

        
        Returns:
            - True if collision detected, False otherwise
        """
        req = fcl.CollisionRequest()
        res = fcl.CollisionResult()

        req.num_max_contacts = 1
        req.enable_cost = False
        req.gjk_solver_type = fcl.GJKSolverType.GST_INDEP

        cdata = fcl.CollisionData(req, res)

        if robot_name == "whole_body":
            # Check left arm vs obstacles
            self.arml_manager.collide(self.obstacle_manager, cdata,
                                      self._is_collide)
            if cdata.done:
                self.logger.info("Left arm collided with obstacles.")
                return True
            # Check right arm vs obstacles
            self.armr_manager.collide(self.obstacle_manager, cdata,
                                      self._is_collide)
            if cdata.done:
                self.logger.info("Right arm collided with obstacles.")
                return True

            self_arm_collide = self._check_collision_single_arm(cdata)
            if self_arm_collide:
                self.logger.info("self arm collided with itself.")
                return True

            # Check left arm vs right arm
            self.arml_manager.collide(self.armr_manager, cdata,
                                      self._is_collide)
            if cdata.done:
                self.logger.info("Left arm collided with right arm.")
                return True
            # Check left arm vs head
            self.arml_manager.collide(self.head_manager, cdata,
                                      self._is_collide)
            if cdata.done:
                self.logger.info("Left arm collided with head.")
                return True
            # Check right arm vs head
            self.armr_manager.collide(self.head_manager, cdata,
                                      self._is_collide)
            if cdata.done:
                self.logger.info("Right arm collided with head.")
                return True

            for l_link in self.arml_links[1:]:
                for b_link in self.body_links:
                    if self._is_collide(l_link, b_link, cdata):
                        self.logger.info("Left arm collided with body.")
                        return True
            for r_link in self.armr_links[1:]:
                for b_link in self.body_links:
                    if self._is_collide(r_link, b_link, cdata):
                        self.logger.info("Right arm collided with body.")
                        return True

            if self.use_ee:
                # Check left end effector vs obstacles
                self.armlee_manager.collide(self.obstacle_manager, cdata,
                                            self._is_collide)
                if cdata.done:
                    self.logger.info(
                        "Left end effector collided with obstacles.")
                    return True
                # Check right end effector vs obstacles
                self.armree_manager.collide(self.obstacle_manager, cdata,
                                            self._is_collide)
                if cdata.done:
                    self.logger.info(
                        "Right end effector collided with obstacles.")
                    return True
                # Check left end effector vs grasp objects
                self.armlee_manager.collide(self.grasp_objects_manager, cdata,
                                            self._is_collide)
                if cdata.done:
                    self.logger.info(
                        "Left end effector collided with grasp objects.")
                    return True
                # Check right end effector vs grasp objects
                self.armree_manager.collide(self.grasp_objects_manager, cdata,
                                            self._is_collide)
                if cdata.done:
                    self.logger.info(
                        "Right end effector collided with grasp objects.")
                    return True
                # Check left end effector vs body & head
                self.armlee_manager.collide(self.body_manager, cdata,
                                            self._is_collide)
                if cdata.done:
                    self.logger.info("Left end effector collided with body.")
                    return True
                self.armlee_manager.collide(self.head_manager, cdata,
                                            self._is_collide)
                if cdata.done:
                    self.logger.info("Left end effector collided with head.")
                    return True
                # Check right end effector vs body & head
                self.armree_manager.collide(self.body_manager, cdata,
                                            self._is_collide)
                if cdata.done:
                    self.logger.info("Right end effector collided with body.")
                    return True
                self.armree_manager.collide(self.head_manager, cdata,
                                            self._is_collide)
                if cdata.done:
                    self.logger.info("Right end effector collided with head.")
                    return True
                # Check left end effector vs right arm & self arm
                self.armlee_manager.collide(self.armr_manager, cdata,
                                            self._is_collide)
                if cdata.done:
                    self.logger.info(
                        "Left end effector collided with right arm.")
                    return True
                for l_link in self.arml_links[:-2]:
                    self.armlee_manager.collide(l_link, cdata, self._is_collide)
                    if cdata.done:
                        self.logger.info(
                            "Left end effector collided with self arm.")
                        return True
                # Check right end effector vs left arm & self arm
                self.armree_manager.collide(self.arml_manager, cdata,
                                            self._is_collide)
                if cdata.done:
                    self.logger.info(
                        "Right end effector collided with left arm.")
                    return True
                for r_link in self.armr_links[:-2]:
                    self.armree_manager.collide(r_link, cdata, self._is_collide)
                    if cdata.done:
                        self.logger.info(
                            "Right end effector collided with self arm.")
                        return True
                if check_between_end_effectors:
                    self.armlee_manager.collide(self.armree_manager, cdata,
                                                self._is_collide)
                    if cdata.done:
                        self.logger.info(
                            "Left end effector collided with right end effector."
                        )
                        return True

            # Check collision between arms and grasp objects manager
            self.arml_manager.collide(self.grasp_objects_manager, cdata,
                                      self._is_collide)
            if cdata.done:
                return True
            self.armr_manager.collide(self.grasp_objects_manager, cdata,
                                      self._is_collide)
            if cdata.done:
                return True

        if robot_name == "left_arm":
            self.arml_manager.collide(self.obstacle_manager, cdata,
                                      self._is_collide)
            if cdata.done:
                return True
            self.arml_manager.collide(self.grasp_objects_manager, cdata,
                                      self._is_collide)
            if cdata.done:
                return True
            self_arm_collide = self._check_collision_single_arm(
                cdata, robot_name="left_arm")
            if self_arm_collide:
                return True
            self.arml_manager.collide(self.armr_manager, cdata,
                                      self._is_collide)
            if cdata.done:
                return True
            self.arml_manager.collide(self.head_manager, cdata,
                                      self._is_collide)
            if cdata.done:
                return True
            for l_link in self.arml_links[1:]:
                for b_link in self.body_links:
                    if self._is_collide(l_link, b_link, cdata):
                        return True
            if self.use_ee:
                self.armlee_manager.collide(self.obstacle_manager, cdata,
                                            self._is_collide)
                if cdata.done:
                    return True
                self.armlee_manager.collide(self.grasp_objects_manager, cdata,
                                            self._is_collide)
                if cdata.done:
                    return True
                self.armlee_manager.collide(self.body_manager, cdata,
                                            self._is_collide)
                if cdata.done:
                    return True
                self.armlee_manager.collide(self.head_manager, cdata,
                                            self._is_collide)
                if cdata.done:
                    return True
                self.armlee_manager.collide(self.armr_manager, cdata,
                                            self._is_collide)
                if cdata.done:
                    return True
                for l_link in self.arml_links[:-2]:
                    self.armlee_manager.collide(l_link, cdata, self._is_collide)
                    if cdata.done:
                        return True

                if check_between_end_effectors:
                    self.armlee_manager.collide(self.armree_manager, cdata,
                                                self._is_collide)
                    if cdata.done:
                        self.logger.info(
                            "Left end effector collided with right end effector."
                        )
                        return True

        if robot_name == "right_arm":
            self.armr_manager.collide(self.obstacle_manager, cdata,
                                      self._is_collide)
            if cdata.done:
                return True
            self.armr_manager.collide(self.grasp_objects_manager, cdata,
                                      self._is_collide)
            if cdata.done:
                return True
            self_arm_collide = self._check_collision_single_arm(
                cdata, robot_name="right_arm")
            if self_arm_collide:
                return True
            self.armr_manager.collide(self.arml_manager, cdata,
                                      self._is_collide)
            if cdata.done:
                return True
            self.armr_manager.collide(self.head_manager, cdata,
                                      self._is_collide)
            if cdata.done:
                return True
            for r_link in self.armr_links[1:]:
                for b_link in self.body_links:
                    if self._is_collide(r_link, b_link, cdata):
                        return True
            if self.use_ee:
                self.armree_manager.collide(self.obstacle_manager, cdata,
                                            self._is_collide)
                if cdata.done:
                    return True
                self.armree_manager.collide(self.grasp_objects_manager, cdata,
                                            self._is_collide)
                if cdata.done:
                    return True
                self.armree_manager.collide(self.body_manager, cdata,
                                            self._is_collide)
                if cdata.done:
                    return True
                self.armree_manager.collide(self.head_manager, cdata,
                                            self._is_collide)
                if cdata.done:
                    return True
                self.armree_manager.collide(self.arml_manager, cdata,
                                            self._is_collide)
                if cdata.done:
                    return True
                for r_link in self.armr_links[:-2]:
                    self.armree_manager.collide(r_link, cdata, self._is_collide)
                    if cdata.done:
                        return True

                if check_between_end_effectors:
                    self.armree_manager.collide(self.armlee_manager, cdata,
                                                self._is_collide)
                    if cdata.done:
                        self.logger.info(
                            "Left end effector collided with right end effector."
                        )
                        return True

        return cdata.done

    def _check_collision_two_sets(self, objects_set1, objects_set2, cdata):
        for obj1 in objects_set1:
            for obj2 in objects_set2:
                if self._is_collide(obj1, obj2, cdata):
                    return True
        return False

    def naive_check_collision(self,
                              robot_name: str = "whole_body",
                              check_between_end_effectors: bool = True) -> bool:
        req = fcl.CollisionRequest()
        res = fcl.CollisionResult()

        req.num_max_contacts = 1
        req.enable_cost = False
        req.gjk_solver_type = fcl.GJKSolverType.GST_INDEP

        cdata = fcl.CollisionData(req, res)

        if robot_name == "whole_body":
            # Check left arm vs obstacles
            self._check_collision_two_sets(self.arml_links, self.obstacles,
                                           cdata)
            if cdata.done:
                self.logger.info("Left arm collided with obstacles.")
                return True
            # Check right arm vs obstacles
            self._check_collision_two_sets(self.armr_links, self.obstacles,
                                           cdata)
            if cdata.done:
                self.logger.info("Right arm collided with obstacles.")
                return True

            self_arm_collide = self._check_collision_single_arm(cdata)
            if self_arm_collide:
                self.logger.info("self arm collided with itself.")
                return True

            # Check left arm vs right arm
            self._check_collision_two_sets(self.arml_links, self.armr_links,
                                           cdata)
            if cdata.done:
                self.logger.info("Left arm collided with right arm.")
                return True
            # Check left arm vs head
            self._check_collision_two_sets(self.arml_links, self.head_links,
                                           cdata)
            if cdata.done:
                self.logger.info("Left arm collided with head.")
                return True
            # Check right arm vs head
            self._check_collision_two_sets(self.armr_links, self.head_links,
                                           cdata)
            if cdata.done:
                self.logger.info("Right arm collided with head.")
                return True

            for l_link in self.arml_links[1:]:
                for b_link in self.body_links:
                    if self._is_collide(l_link, b_link, cdata):
                        self.logger.info("Left arm collided with body.")
                        return True
            for r_link in self.armr_links[1:]:
                for b_link in self.body_links:
                    if self._is_collide(r_link, b_link, cdata):
                        self.logger.info("Right arm collided with body.")
                        return True

            if self.use_ee:
                # Check left end effector vs obstacles
                self._check_collision_two_sets(self.armlee_links,
                                               self.obstacles, cdata)
                if cdata.done:
                    self.logger.info(
                        "Left end effector collided with obstacles.")
                    return True
                # Check right end effector vs obstacles
                self._check_collision_two_sets(self.armree_links,
                                               self.obstacles, cdata)
                if cdata.done:
                    self.logger.info(
                        "Right end effector collided with obstacles.")
                    return True
                # Check left end effector vs grasp objects
                self._check_collision_two_sets(self.armlee_links,
                                               self.grasp_objects, cdata)
                if cdata.done:
                    self.logger.info(
                        "Left end effector collided with grasp objects.")
                    return True
                # Check right end effector vs grasp objects
                self._check_collision_two_sets(self.armree_links,
                                               self.grasp_objects, cdata)
                if cdata.done:
                    self.logger.info(
                        "Right end effector collided with grasp objects.")
                    return True
                # Check left end effector vs body & head
                self._check_collision_two_sets(self.armlee_links,
                                               self.body_links, cdata)
                if cdata.done:
                    self.logger.info("Left end effector collided with body.")
                    return True
                self._check_collision_two_sets(self.armlee_links,
                                               self.head_links, cdata)
                if cdata.done:
                    self.logger.info("Left end effector collided with head.")
                    return True
                # Check right end effector vs body & head
                self._check_collision_two_sets(self.armree_links,
                                               self.body_links, cdata)
                if cdata.done:
                    self.logger.info("Right end effector collided with body.")
                    return True
                self._check_collision_two_sets(self.armree_links,
                                               self.head_links, cdata)
                if cdata.done:
                    self.logger.info("Right end effector collided with head.")
                    return True
                # Check left end effector vs right arm & self arm
                self._check_collision_two_sets(self.armlee_links,
                                               self.armr_links, cdata)
                if cdata.done:
                    self.logger.info(
                        "Left end effector collided with right arm.")
                    return True
                for l_link in self.arml_links[:-2]:
                    self._check_collision_two_sets(self.armlee_links, [l_link],
                                                   cdata)
                    if cdata.done:
                        self.logger.info(
                            "Left end effector collided with self arm.")
                        return True
                # Check right end effector vs left arm & self arm
                self._check_collision_two_sets(self.armree_links,
                                               self.arml_links, cdata)
                if cdata.done:
                    self.logger.info(
                        "Right end effector collided with left arm.")
                    return True
                for r_link in self.armr_links[:-2]:
                    self._check_collision_two_sets(self.armree_links, [r_link],
                                                   cdata)
                    if cdata.done:
                        self.logger.info(
                            "Right end effector collided with self arm.")
                        return True

                if check_between_end_effectors:
                    self._check_collision_two_sets(self.armlee_links,
                                                   self.armree_links, cdata)
                    if cdata.done:
                        self.logger.info(
                            "Left end effector collided with right end effector."
                        )
                        return True

            # Check collision between arms and grasp objects manager
            self._check_collision_two_sets(self.arml_links, self.grasp_objects,
                                           cdata)
            if cdata.done:
                return True
            self._check_collision_two_sets(self.armr_links, self.grasp_objects,
                                           cdata)
            if cdata.done:
                return True

        if robot_name == "left_arm":
            self._check_collision_two_sets(self.arml_links, self.obstacles,
                                           cdata)
            if cdata.done:
                return True
            self._check_collision_two_sets(self.arml_links, self.grasp_objects,
                                           cdata)
            if cdata.done:
                return True
            self_arm_collide = self._check_collision_single_arm(
                cdata, robot_name="left_arm")
            if self_arm_collide:
                return True
            self._check_collision_two_sets(self.arml_links, self.armr_links,
                                           cdata)
            if cdata.done:
                return True
            self._check_collision_two_sets(self.arml_links, self.head_links,
                                           cdata)
            if cdata.done:
                return True
            for l_link in self.arml_links[1:]:
                for b_link in self.body_links:
                    if self._is_collide(l_link, b_link, cdata):
                        return True
            if self.use_ee:
                self._check_collision_two_sets(self.armlee_links,
                                               self.obstacles, cdata)
                if cdata.done:
                    return True
                self._check_collision_two_sets(self.armlee_links,
                                               self.grasp_objects, cdata)
                if cdata.done:
                    return True
                self._check_collision_two_sets(self.armlee_links,
                                               self.body_links, cdata)
                if cdata.done:
                    return True
                self._check_collision_two_sets(self.armlee_links,
                                               self.head_links, cdata)
                if cdata.done:
                    return True
                self._check_collision_two_sets(self.armlee_links,
                                               self.armr_links, cdata)
                if cdata.done:
                    return True
                for l_link in self.arml_links[:-2]:
                    self._check_collision_two_sets(self.armlee_links, [l_link],
                                                   cdata)
                    if cdata.done:
                        return True
                if check_between_end_effectors:
                    self._check_collision_two_sets(self.armlee_links,
                                                   self.armree_links, cdata)
                    if cdata.done:
                        self.logger.info(
                            "Left end effector collided with right end effector."
                        )
                        return True

        if robot_name == "right_arm":
            self._check_collision_two_sets(self.armr_links, self.obstacles,
                                           cdata)
            if cdata.done:
                return True
            self._check_collision_two_sets(self.armr_links, self.grasp_objects,
                                           cdata)
            if cdata.done:
                return True
            self_arm_collide = self._check_collision_single_arm(
                cdata, robot_name="right_arm")
            if self_arm_collide:
                return True
            self._check_collision_two_sets(self.armr_links, self.arml_links,
                                           cdata)
            if cdata.done:
                return True
            self._check_collision_two_sets(self.armr_links, self.head_links,
                                           cdata)
            if cdata.done:
                return True
            for r_link in self.armr_links[1:]:
                for b_link in self.body_links:
                    if self._is_collide(r_link, b_link, cdata):
                        return True
            if self.use_ee:
                self._check_collision_two_sets(self.armree_links,
                                               self.obstacles, cdata)
                if cdata.done:
                    return True
                self._check_collision_two_sets(self.armree_links,
                                               self.grasp_objects, cdata)
                if cdata.done:
                    return True
                self._check_collision_two_sets(self.armree_links,
                                               self.body_links, cdata)
                if cdata.done:
                    return True
                self._check_collision_two_sets(self.armree_links,
                                               self.head_links, cdata)
                if cdata.done:
                    return True
                self._check_collision_two_sets(self.armree_links,
                                               self.arml_links, cdata)
                if cdata.done:
                    return True
                for r_link in self.armr_links[:-2]:
                    self._check_collision_two_sets(self.armree_links, [r_link],
                                                   cdata)
                    if cdata.done:
                        return True
                if check_between_end_effectors:
                    self._check_collision_two_sets(self.armree_links,
                                                   self.armlee_links, cdata)
                    if cdata.done:
                        self.logger.info(
                            "Left end effector collided with right end effector."
                        )
                        return True
        return cdata.done

    def check_collision_between_arms(self):
        """
        This function is used for trajectory coordination, 
            where it will find a path in coordination space and check collision between arms meanwhile.
        
        """
        req = fcl.CollisionRequest()
        res = fcl.CollisionResult()

        req.num_max_contacts = 1
        req.enable_cost = False
        req.gjk_solver_type = fcl.GJKSolverType.GST_INDEP

        cdata = fcl.CollisionData(req, res)

        # Check left arm vs right arm
        self.arml_manager.collide(self.armr_manager, cdata, self._is_collide)
        if cdata.done:
            return True
        if self.use_ee:
            # Check left end effector vs right arm
            self.armlee_manager.collide(self.armr_manager, cdata,
                                        self._is_collide)
            if cdata.done:
                return True
            # Check right end effector vs left arm
            self.armree_manager.collide(self.arml_manager, cdata,
                                        self._is_collide)
            if cdata.done:
                return True
            # Check left end effector vs right end effector
            self.armlee_manager.collide(self.armree_manager, cdata,
                                        self._is_collide)
            if cdata.done:
                return True
        return False

    def _check_collision_single_arm(self,
                                    cdata,
                                    robot_name: str = "whole_body"):
        """
        This function is used for checking collision inside a single arm

        Args:
            robot_name (str): "robot", "left_arm", "right_arm"
        """
        if robot_name == "whole_body":
            for i in range(len(self.arml_links)):
                if i + 2 >= len(self.arml_links):
                    break
                for j in range(i + 2, len(self.arml_links)):
                    if self._is_collide(self.arml_links[i], self.arml_links[j],
                                        cdata):
                        return True
            for i in range(len(self.armr_links)):
                if i + 2 >= len(self.armr_links):
                    break
                for j in range(i + 2, len(self.armr_links)):
                    if self._is_collide(self.armr_links[i], self.armr_links[j],
                                        cdata):
                        return True
        if robot_name == "left_arm":
            for i in range(len(self.arml_links)):
                if i + 2 >= len(self.arml_links):
                    break
                for j in range(i + 2, len(self.arml_links)):
                    if self._is_collide(self.arml_links[i], self.arml_links[j],
                                        cdata):
                        return True
        if robot_name == "right_arm":
            for i in range(len(self.armr_links)):
                if i + 2 >= len(self.armr_links):
                    break
                for j in range(i + 2, len(self.armr_links)):
                    if self._is_collide(self.armr_links[i], self.armr_links[j],
                                        cdata):
                        return True
        return False

    def _compute_distance_signle_arm(self):
        """
        Current not used
        """
        for i in range(len(self.arml_links)):
            if i + 2 >= len(self.arml_links):
                break
            for j in range(i + 2, len(self.arml_links)):
                dreq = fcl.DistanceRequest(enable_nearest_points=False)
                dres = fcl.DistanceResult()
                ret = fcl.distance(self.arml_links[i], self.arml_links[j], dreq,
                                   dres)
                if ret < 0.02:
                    self.logger.info(
                        f"Left arm link {i} and link {j} are too close, distance: {ret}"
                    )
                    return True
        for i in range(len(self.armr_links)):
            if i + 2 >= len(self.armr_links):
                break
            for j in range(i + 2, len(self.armr_links)):
                dreq = fcl.DistanceRequest(enable_nearest_points=False)
                dres = fcl.DistanceResult()
                ret = fcl.distance(self.armr_links[i], self.armr_links[j], dreq,
                                   dres)
                if ret < 0.02:
                    self.logger.info(
                        f"Right arm link {i} and link {j} are too close, distance: {ret}"
                    )
                    return True
        return False

    def compute_self_distance(self):
        """
        Current not used
        """
        dreq = fcl.DistanceRequest()
        dreq.enable_nearest_points = False
        dreq.gjk_solver_type = fcl.GJKSolverType.GST_INDEP
        dreq.rel_err = 0.01
        dres = fcl.DistanceResult()

        ddata = fcl.DistanceData(dreq, dres)
        start_time = time.time()
        self.arml_manager.distance(self.armr_manager, ddata,
                                   fcl.defaultDistanceCallback)
        if ddata.result.min_distance < 0.02:
            self.logger.info(
                f"Left arm and right arm are too close, distance: {ddata.result.min_distance}"
            )
            return True
        end_time = time.time()
        self.logger.info(
            f"Distance computation between arms took {end_time - start_time:.6f} seconds."
        )

        self_arm_collide = self._compute_distance_signle_arm()
        if self_arm_collide:
            self.logger.info("Self arm collision detected.")
            return True

        ddata = fcl.DistanceData(
            request=fcl.DistanceRequest(enable_nearest_points=False),
            result=fcl.DistanceResult())
        self.arml_manager.distance(self.head_manager, ddata,
                                   fcl.defaultDistanceCallback)
        if ddata.result.min_distance < 0.02:
            self.logger.info(
                f"Left arm and head are too close, distance: {ddata.result.min_distance}"
            )
            return True

        ddata = fcl.DistanceData(
            request=fcl.DistanceRequest(enable_nearest_points=False),
            result=fcl.DistanceResult())
        self.armr_manager.distance(self.head_manager, ddata,
                                   fcl.defaultDistanceCallback)
        if ddata.result.min_distance < 0.02:
            self.logger.info(
                f"Right arm and head are too close, distance: {ddata.result.min_distance}"
            )
            return True

        for l_link in self.arml_links[1:]:
            for b_link in self.body_links:
                dreq = fcl.DistanceRequest(enable_nearest_points=False)
                dres = fcl.DistanceResult()
                ret = fcl.distance(l_link, b_link, dreq, dres)
                if ret < 0.02:
                    self.logger.info(
                        f"Left arm link and body link are too close, distance: {ret}"
                    )
                    return True
            for r_link in self.armr_links[1:]:
                dreq = fcl.DistanceRequest(enable_nearest_points=False)
                dres = fcl.DistanceResult()
                ret = fcl.distance(l_link, r_link, dreq, dres)
                if ret < 0.02:
                    self.logger.info(
                        f"Left arm link and right arm link are too close, distance: {ret}"
                    )
                    return True

        end_time = time.time()
        self.logger.info(
            f"Distance computation took {end_time - start_time:.6f} seconds.")
        return False


#------------------------------------Visulaize-----------------------------------------
#------------------------------------Only for Debug------------------------------------

    def enable_visualize(self):
        self.visualize = True
        self.robot_mesh_list = []
        self.obstacle_mesh_list = []
        self.grasp_object_mesh_list = []

        for path in self.torso_full_path:
            self.create_mesh(path)
        for path in self.head_full_path:
            self.create_mesh(path)
        for path in self.arml_full_path:
            self.create_mesh(path)
        for path in self.armr_full_path:
            self.create_mesh(path)

        if self.use_ee:
            for path in self.armlee_full_path:
                self.create_mesh(path)
            for path in self.armree_full_path:
                self.create_mesh(path)

    def create_mesh(self, obj_path):
        mesh = trimesh.load(obj_path, force='mesh')
        self.robot_mesh_list.append(mesh)

    def update_vis_transform(self,
                             transforms: list[np.ndarray] = None,
                             armlee_transforms: list[np.ndarray] = None,
                             armree_transforms: list[np.ndarray] = None):
        """
        transforms should be in order of [torso_tf, head_tf, arml_tf, armr_tf]
        """
        if not self.use_ee:
            tf = transforms
        else:
            tf = transforms + armlee_transforms + armree_transforms
        vis_mesh = deepcopy(self.robot_mesh_list)
        for i, mesh in enumerate(vis_mesh):
            mesh.apply_transform(tf[i])
        scene = trimesh.Scene()
        for i, mesh in enumerate(vis_mesh):
            scene.add_geometry(mesh)
        for mesh in self.obstacle_mesh_list:
            scene.add_geometry(mesh)

        # if self.arml_attached:
        #     grasp_vis_mesh = deepcopy(self.grasp_object_mesh_list)
        #     for i, mesh in enumerate(grasp_vis_mesh):
        #         mesh.apply_transform(transforms[12] @ self.arml7_T_object)
        #     for mesh in grasp_vis_mesh:
        #         scene.add_geometry(mesh)
        # for mesh in self.grasp_object_mesh_list:
        #     scene.add_geometry(mesh)

        saved_path = os.path.join(LOG_DIR, 'collision_check_scene.ply')
        scene.export(saved_path)

        # png = scene.save_image(resolution=(1280, 720), visible=True)
        # with open('collision_check_scene.png', 'wb') as f:
        #     f.write(png)

        # scene.show()
        # embed()
