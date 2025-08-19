import fcl
import numpy as np
import trimesh
import open3d as o3d
import time
import os
import json
from copy import deepcopy 
from dexe_robot_client.end_effector_client import EndEffectorType 
from dexe_robot_client.w1.path_planner.utils.robot_type import *


class FCLChecker:
    def __init__(self, robot_type: RobotType = None, end_effector_type: EndEffectorType = None):
        if robot_type is None:
            raise ValueError(f"Robot type must be specified as {RobotType.W1V02_1} or {RobotType.W1V02_2}.")
        self.arml_full_path = get_arml_full_path(robot_type)
        self.armr_full_path = get_armr_full_path(robot_type)
        self.head_full_path = get_head_full_path()
        self.torso_full_path = get_torso_full_path()
        

        # Containers for fcl objects
        self.arml_links = []
        self.armr_links = []
        self.head_links = []
        self.body_links = []
        self.obstacles = []
        self.grasp_objects = []


        # Dynamic AABB Managers
        self.arml_manager = fcl.DynamicAABBTreeCollisionManager()
        self.armr_manager = fcl.DynamicAABBTreeCollisionManager()
        self.head_manager = fcl.DynamicAABBTreeCollisionManager()
        self.body_manager = fcl.DynamicAABBTreeCollisionManager()
        self.obstacle_manager = fcl.DynamicAABBTreeCollisionManager()
        self.grasp_objects_manager = fcl.DynamicAABBTreeCollisionManager()

        self.grasp_objects_dict = {}


        self.use_ee = False
        if end_effector_type is not None:
            if isinstance(end_effector_type, EndEffectorType):
                self.ee_full_path = get_ee_full_path(end_effector_type)
                self.arml_full_path.append(self.ee_full_path[0])
                self.armr_full_path.append(self.ee_full_path[1])
                self.use_ee = True
            else:
                raise ValueError(f"Unsupported End Effector Type: {end_effector_type}. Please use PGC_GRIPPER or BRAINCO_HAND.")
        
        self.arml_attached = False
        self.armr_attached = False
        self.arml7_T_object = None
        self.armr7_T_object = None
        self.arml_grasp_obj_name = None
        self.armr_grasp_obj_name = None

        self._create_robot_obj()
        self._create_robot_manager()

        self.compute_distance = False

    def set_compute_distance(self, compute_distance: bool = False):
        """
        Enable the distance computation between objects.
        """
        self.compute_distance = compute_distance

    def _obj2fcl(self, obj_path):
        mesh = trimesh.load(obj_path, force='mesh')
        vertices = np.asarray(mesh.vertices, dtype=np.float64)
        triangles = np.asarray(mesh.faces, dtype=np.int32)
        print(f"vertices length {len(vertices)}, triangles length {len(triangles)}")

        bvh = fcl.BVHModel()
        bvh.beginModel(len(vertices), len(triangles))
        bvh.addSubModel(vertices, triangles)
        bvh.endModel()

        rotation = np.eye(3, dtype=np.float64)
        translation = np.zeros(3, dtype=np.float64).reshape(3, 1)
        fcl_transform = fcl.Transform(rotation, translation)
        fcl_obj = fcl.CollisionObject(bvh, fcl_transform)

        return fcl_obj
    


    def _create_robot_obj(self):
        self.arml_links = [self._obj2fcl(path) for path in self.arml_full_path]
        self.armr_links = [self._obj2fcl(path) for path in self.armr_full_path]
        self.head_links = [self._obj2fcl(path) for path in self.head_full_path]
        self.body_links = [self._obj2fcl(path) for path in self.torso_full_path]

        print(f"Created FCL Objects: {len(self.arml_links)} left arm links, {len(self.armr_links)} right arm links, {len(self.head_links)} head links, {len(self.body_links)} body links")

    def _create_robot_manager(self):
        for link in self.arml_links:
            self.arml_manager.registerObject(link)
        for link in self.armr_links:
            self.armr_manager.registerObject(link)
        for link in self.head_links:
            self.head_manager.registerObject(link)
        for link in self.body_links:
            self.body_manager.registerObject(link)


        self.arml_manager.setup()
        self.armr_manager.setup()
        self.head_manager.setup()
        self.body_manager.setup()



    def set_head_body_transform(self, head_transforms, body_transforms):
        if len(head_transforms) != len(self.head_links):
            raise ValueError(f"Expected {len(self.head_links)} head transforms, got {len(head_transforms)}")
        if len(body_transforms) != len(self.body_links):
            raise ValueError(f"Expected {len(self.body_links)} body transforms, got {len(body_transforms)}")
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

    def add_cube_obstacle(self, center, size):
        box = fcl.Box(size[0], size[1], size[2])
        transform = fcl.Transform(np.eye(3), center)
        fcl_obj = fcl.CollisionObject(box, transform)

        self.obstacles.append(fcl_obj)
        self.obstacle_manager.registerObject(fcl_obj)
        self.obstacle_manager.setup()
    def add_sphere_obstacle(self, center, radius):
        sphere = fcl.Sphere(radius)
        transform = fcl.Transform(np.eye(3), center)
        fcl_obj = fcl.CollisionObject(sphere, transform)

        self.obstacles.append(fcl_obj)
        self.obstacle_manager.registerObject(fcl_obj)
        self.obstacle_manager.setup()

    def add_mesh_obstacle(self, mesh_path, center):
        mesh = trimesh.load(mesh_path, force='mesh')
        vertices = np.asarray(mesh.vertices, dtype=np.float64)
        triangles = np.asarray(mesh.faces, dtype=np.int32)

        bvh = fcl.BVHModel()
        bvh.beginModel(len(vertices), len(triangles))
        bvh.addSubModel(vertices, triangles)
        bvh.endModel()

        rotation = np.eye(3, dtype=np.float64)
        translation = center.reshape(3, 1).astype(np.float64)
        fcl_transform = fcl.Transform(rotation, translation)
        fcl_obj = fcl.CollisionObject(bvh, fcl_transform)

        self.obstacles.append(fcl_obj)
        self.obstacle_manager.registerObject(fcl_obj)
        self.obstacle_manager.setup()
        
    def _save_point_cloud(self, point_cloud):
        current_path = os.path.dirname(os.path.abspath(__file__))
        saved_folder_path = os.path.join(current_path, "maps")
        if not os.path.exists(saved_folder_path):
            os.makedirs(saved_folder_path)
        saved_file_path = os.path.join(saved_folder_path, "transformed_point_cloud.ply")
        o3d.io.write_point_cloud(saved_file_path, point_cloud)

    def add_octomap(self, ply_path, external_trans, octo_res):
        point_cloud = o3d.io.read_point_cloud(ply_path)
        # Apply transformation to the world coordinates
        point_cloud.transform(external_trans)
        # Downsample the point cloud
        point_cloud = point_cloud.voxel_down_sample(voxel_size=0.01)
        # save the point cloud
        self._save_point_cloud(point_cloud)

        point_cloud_array = np.asarray(point_cloud.points)
        octo_obj = fcl.OcTree(r=octo_res, points=point_cloud_array)
        self.obstacles.append(octo_obj)
        self.obstacle_manager.registerObject(octo_obj)
        self.obstacle_manager.setup()
    
    def load_env_json(self, json_path):

        # Clear existing obstacles and grasp objects
        self.obstacles.clear()
        self.obstacle_manager = fcl.DynamicAABBTreeCollisionManager()
        self.grasp_objects.clear()
        self.grasp_objects_manager = fcl.DynamicAABBTreeCollisionManager()
        # self.grasp_objects_dict.clear()
        temp_grasp_objects_dict = {}
        if self.arml_attached:
            temp_grasped_obj_l = self.grasp_objects_dict[self.arml_grasp_obj_name]
            self.grasp_objects.append(temp_grasped_obj_l)
            temp_grasp_objects_dict[self.arml_grasp_obj_name] = temp_grasped_obj_l
        if self.armr_attached:
            temp_grasped_obj_r = self.grasp_objects_dict[self.armr_grasp_obj_name]
            self.grasp_objects.append(temp_grasped_obj_r)
            temp_grasp_objects_dict[self.armr_grasp_obj_name] = temp_grasped_obj_r
        
        self.grasp_objects_dict.clear()
        self.grasp_objects_dict.update(temp_grasp_objects_dict)
        
        

        with open(json_path, 'r') as f:
            env_data = json.load(f)
        # Load environment obstacles
        obstacle_cfg = env_data["scene_objects"]
        for obj in obstacle_cfg:
            if obj["shape"]["shape_type"] == "box":
                center = np.array(obj["pose"])[:3]
                size = np.array(obj["shape"]["raw_data"])
                self.add_cube_obstacle(center, size)
                print(f"Successfully add a {obj['shape']['shape_type']} obstacle at center: {center}, with size: {size} and name: {obj['name']}")
            if obj["shape"]["shape_type"] == "sphere":
                center = np.array(obj["pose"])[:3]
                radius = obj["shape"]["raw_data"][0]
                self.add_sphere_obstacle(center, radius)
                print(f"Successfully add a {obj['shape']['shape_type']} obstacle at center: {center}, with radius: {radius} and name: {obj['name']}")
            if obj["shape"]["shape_type"] == "mesh":
                obj_mesh_path = obj["shape"]["raw_data"]
                if not os.path.exists(obj_mesh_path):
                    raise ValueError(f"Obstacle mesh path {obj_mesh_path} does not exist.")
                center = np.array(obj["pose"])[:3]
                self.add_mesh_obstacle(obj_mesh_path, center)
                print(f"Successfully add a {obj['shape']['shape_type']} obstacle with mesh path: {obj_mesh_path} at center: {center} and name: {obj['name']}")
        
        # Load environment grasp objects
        grasp_cfg = env_data["user_workobjects"]
        for grasp_obj in grasp_cfg:
            if grasp_obj["shape"]["shape_type"] == "box":
                center = np.array(grasp_obj["pose"])[:3]
                size = np.array(grasp_obj["shape"]["raw_data"])
                name = grasp_obj["name_key"]
                self.add_grasp_object(obj_type="box", center=center, size=size, name=name)
                print(f"Successfully add a {grasp_obj['shape']['shape_type']} grasp object")
            if grasp_obj["shape"]["shape_type"] == "sphere":
                center = np.array(grasp_obj["pose"])[:3]
                radius = grasp_obj["shape"]["raw_data"][0]
                name = grasp_obj["name_key"]
                self.add_grasp_object(obj_type="sphere", center=center, size=np.array([radius]), name=name)
                print(f"Successfully add a {grasp_obj['shape']['shape_type']} grasp object")
            if grasp_obj["shape"]["shape_type"] == "mesh":
                grasp_obj_mesh_path = grasp_obj["shape"]["raw_data"]
                if not os.path.exists(grasp_obj_mesh_path):
                    raise ValueError(f"Grasp object mesh path {grasp_obj_mesh_path} does not exist.")
                center = np.array(grasp_obj["pose"])[:3]
                name = grasp_obj["name_key"]
                self.add_grasp_object(obj_type="mesh", objMesh_path=grasp_obj_mesh_path, center=center, name=name)
                print(f"Successfully add a {grasp_obj['shape']['shape_type']} grasp object with mesh path: {grasp_obj_mesh_path}")

    def add_grasp_object(self, obj_type: str = None, objMesh_path: str =None, center: np.ndarray = None, size: np.ndarray = None, name: str = None):
        """
        obj_type: "box", "sphere", "mesh"
        objMesh_path: Path to the mesh file if obj_type is "mesh"
        """
        fcl_obj = None
        if obj_type is None or obj_type not in ["box", "sphere", "mesh"]:
            raise ValueError("Object type must be one of 'box', 'sphere', or 'mesh'.")
        if obj_type == "mesh":
            if objMesh_path is None or not os.path.exists(objMesh_path):
                raise ValueError("Try to add a grasp mesh object, but the mesh path is None or does not exist.")
            mesh = trimesh.load(objMesh_path, force='mesh')
            vertices = np.asarray(mesh.vertices, dtype=np.float64)
            triangles = np.asarray(mesh.faces, dtype=np.int32)
            bvh = fcl.BVHModel()
            bvh.beginModel(len(vertices), len(triangles))
            bvh.addSubModel(vertices, triangles)
            bvh.endModel()
            rotation = np.eye(3, dtype=np.float64)
            translation = center.reshape(3, 1).astype(np.float64)
            fcl_transform = fcl.Transform(rotation, translation)
            fcl_obj = fcl.CollisionObject(bvh, fcl_transform)
        elif obj_type == "box":
            if center is None or size is None or len(size) != 3:
                raise ValueError("Try to add a box object, but center or size is None or size is not a 3-element array.")
            box = fcl.Box(size[0], size[1], size[2])
            transform = fcl.Transform(np.eye(3), center)
            fcl_obj = fcl.CollisionObject(box, transform)
        elif obj_type == "sphere":
            if center is None or size is None or len(size) != 1:
                raise ValueError("Try to add a sphere object, but center or size is None or size is not a 1-element array(radius)")
            sphere = fcl.Sphere(size[0])
            transform = fcl.Transform(np.eye(3), center)
            fcl_obj = fcl.CollisionObject(sphere, transform)
        
        if name in self.grasp_objects_dict:
            raise ValueError(f"Grasp object with name {name} already exists. Please use a different name.")
        self.grasp_objects_dict[name] = fcl_obj
        self.grasp_objects.append(fcl_obj)
        self.grasp_objects_manager.registerObject(fcl_obj)
        self.grasp_objects_manager.setup()

    def attach_object_arml(self, obj_name: str = None, flange_T_object: np.ndarray = None):
        if obj_name is None or obj_name not in self.grasp_objects_dict:
            raise ValueError(f"Object with name {obj_name} does not exist in grasp objects.")
        if flange_T_object is None or flange_T_object.shape != (4, 4):
            raise ValueError("Flange to object transformation must be a 4x4 matrix.")
        
        fcl_obj = self.grasp_objects_dict[obj_name]
        if self.arml_attached:
            raise ValueError("Left arm already attached to an object. Please detach first.")
        self.arml_attached = True
        self.arml7_T_object = flange_T_object
        self.arml_grasp_obj_name = obj_name

        self.arml_links.append(fcl_obj)
        self.grasp_objects_manager.unregisterObject(fcl_obj)
        self.arml_manager.registerObject(fcl_obj)
        self.arml_manager.setup()
        self.grasp_objects_manager.update()

        print(f"After attaching object, left arm manager has {len(self.arml_manager.getObjects())} objects, grasp objects manager has {len(self.grasp_objects_manager.getObjects())} objects.")

    def detach_object_arml(self, obj_name: str=None, arml7_transform: np.ndarray = None):
        if obj_name is None or obj_name not in self.grasp_objects_dict:
            raise ValueError(f"Object with name {obj_name} does not exist in grasp objects.")
        if arml7_transform is not None and arml7_transform.shape != (4, 4):
            raise ValueError("Arml7 transform must be a 4x4 matrix.")
        
        fcl_obj = self.grasp_objects_dict[obj_name]
        if not self.arml_attached:
            raise ValueError("Left arm is not attached to any object.")

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
        
    def attach_object_armr(self, obj_name: str = None, flange_T_object: np.ndarray = None):
        if obj_name is None or obj_name not in self.grasp_objects_dict:
            raise ValueError(f"Object with name {obj_name} does not exist in grasp objects.")
        if flange_T_object is None or flange_T_object.shape != (4, 4):
            raise ValueError("Flange to object transformation must be a 4x4 matrix.")
        
        fcl_obj = self.grasp_objects_dict[obj_name]
        if self.armr_attached:
            raise ValueError("Right arm already attached to an object. Please detach first.")
        self.armr_attached = True
        self.armr7_T_object = flange_T_object
        self.armr_grasp_obj_name = obj_name

        self.armr_links.append(fcl_obj)
        self.grasp_objects_manager.unregisterObject(fcl_obj)
        self.armr_manager.registerObject(fcl_obj)
        self.armr_manager.setup()
        self.grasp_objects_manager.update()

    def detach_object_armr(self, obj_name: str=None, armr7_transform: np.ndarray = None):
        if obj_name is None or obj_name not in self.grasp_objects_dict:
            raise ValueError(f"Object with name {obj_name} does not exist in grasp objects.")
        if armr7_transform is not None and armr7_transform.shape != (4, 4):
            raise ValueError("Armr7 transform must be a 4x4 matrix.")
        
        fcl_obj = self.grasp_objects_dict[obj_name]
        if not self.armr_attached:
            raise ValueError("Right arm is not attached to any object.")

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
        

            

    def update_arm_transform(self, transforms):
        # transforms: Transformation matrices for dual arms (size: 2 x single_arm_freedom + 2(end_effector))
        if self.arml_attached or self.armr_attached:
            # 传入的仍然是一个长度为16的list
            if self.arml_attached:
                left_transforms = transforms[:len(self.arml_links)-1]
                
                grasp_object_transform = left_transforms[-1] @ self.arml7_T_object
                left_transforms.append(grasp_object_transform)
            else:
                left_transforms = transforms[:len(self.arml_links)]
            if self.armr_attached:
                right_transforms = transforms[len(self.arml_links)-1:]
                grasp_object_transform = right_transforms[-1] @ self.armr7_T_object
                right_transforms.append(grasp_object_transform)
                left_transforms = transforms[:len(self.arml_links)]
            else:
                right_transforms = transforms[len(self.armr_links):]
        else:
            if len(transforms) != (len(self.arml_links) + len(self.armr_links)):
                raise ValueError(f"Expected {len(self.arml_links) + len(self.armr_links)} transforms, got {len(transforms)}, might missing end effector transforms")
            left_transforms = transforms[:len(self.arml_links)]
            right_transforms = transforms[len(self.armr_links):]
        for i, link in enumerate(self.arml_links):
            rotation = left_transforms[i][:3, :3].astype(np.float64)
            translation = left_transforms[i][:3, 3].reshape(3, 1).astype(np.float64)
            fcl_transform = fcl.Transform(rotation, translation)
            link.setTransform(fcl_transform)
        for i, link in enumerate(self.armr_links):
            rotation = right_transforms[i][:3, :3].astype(np.float64)
            translation = right_transforms[i][:3, 3].reshape(3, 1).astype(np.float64)
            fcl_transform = fcl.Transform(rotation, translation)
            link.setTransform(fcl_transform)
        self.arml_manager.update()
        self.armr_manager.update()

        

    def _is_collide(self, obs1, obs2, cdata):
        if cdata.done:
            return True
        ret = fcl.collide(obs1, obs2, cdata.request, cdata.result)
        if ret>0:
            cdata.done = True
            return True
        return cdata.done

    # Main collision checking function
    def check_collision(self):
        req = fcl.CollisionRequest()
        res = fcl.CollisionResult()

        req.num_max_contacts = 1
        req.enable_cost = False
        req.gjk_solver_type = fcl.GJKSolverType.GST_INDEP

        cdata = fcl.CollisionData(req, res)

        # Check left arm vs obstacles
        self.arml_manager.collide(self.obstacle_manager, cdata, self._is_collide)
        if cdata.done: 
            # print("Left arm collided with obstacles.")
            return True
        # Check right arm vs obstacles
        self.armr_manager.collide(self.obstacle_manager, cdata, self._is_collide)
        if cdata.done:
            # print("Right arm collided with obstacles.")
            return True
        
        self_arm_collide = self._check_collision_single_arm(cdata)
        if self_arm_collide: 
            # print("self arm collided with itself.")
            return True

        # Check left arm vs right arm
        self.arml_manager.collide(self.armr_manager, cdata, self._is_collide)
        if cdata.done:
            # print("Left arm collided with right arm.")
            return True
        # Check left arm vs head
        self.arml_manager.collide(self.head_manager, cdata, self._is_collide)
        if cdata.done:
            # print("Left arm collided with head.")
            return True
        # Check right arm vs head
        self.armr_manager.collide(self.head_manager, cdata, self._is_collide)
        if cdata.done: 
            # print("Right arm collided with head.")
            return True
        
        for l_link in self.arml_links[1:]:
            for b_link in self.body_links:
                if self._is_collide(l_link, b_link, cdata):
                    # print("Left arm collided with body.")
                    return True
        for r_link in self.armr_links[1:]:
            for b_link in self.body_links:
                if self._is_collide(r_link, b_link, cdata):
                    # print("Right arm collided with body.")
                    return True
                
        # Check collision between arms and grasp objects manager
        self.arml_manager.collide(self.grasp_objects_manager, cdata, self._is_collide)
        if cdata.done: return True
        self.armr_manager.collide(self.grasp_objects_manager, cdata, self._is_collide)
        if cdata.done: return True

        return cdata.done
    
    # This function is used for trajectory coordination, where it will find a path in coordination space and check collision between arms meanwhile.
    def check_collision_between_arms(self):
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
        return False
    
    # This function is used for checking collision inside a single arm
    def _check_collision_single_arm(self, cdata):
        for i in range(len(self.arml_links)):
            if i+2 >= len(self.arml_links):
                break
            for j in range(i+2, len(self.arml_links)):
                if self._is_collide(self.arml_links[i], self.arml_links[j], cdata):
                    return True
        for i in range(len(self.armr_links)):
            if i+2 >= len(self.armr_links):
                break
            for j in range(i+2, len(self.armr_links)):
                if self._is_collide(self.armr_links[i], self.armr_links[j], cdata):
                    return True
        return False

    def compute_distance(self):
        dreq = fcl.DistanceRequest(enable_nearest_points=True) # enable_signed_distance=True
        ddata = fcl.DistanceData(request=dreq)
        
        ddata.done = False
        self.arml_manager.distance(self.obstacle_manager, ddata, fcl.defaultDistanceCallback)
        print(f"Left arm distance to obstacles: {ddata.result.min_distance}")



        