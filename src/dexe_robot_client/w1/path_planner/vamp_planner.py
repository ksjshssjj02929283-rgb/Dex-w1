import numpy as np
import vamp
import os
import time
from scipy.spatial.transform import Rotation
from dexe_robot_client.w1.path_planner.planner_pinocchio import Planner_Pin
from dexe_robot_client.w1.path_planner.utils.mapping_index import *
from dexe_robot_client.w1.path_planner.utils.robot_type import *
from dexe_robot_client.end_effector_client import EndEffectorType
from ...common import get_logger
from IPython import embed

# from vamp import pybullet_interface as vpb

class VampPlanner:
    def __init__(self, 
                 robot_type: RobotType = RobotType.W1V02_1, 
                 end_effector_type: EndEffectorType = None,
                 is_left_arm: bool = True,
                 **kwargs):
        """
        Args:
            arm_T_w: 4x4 np array, transformation from world to arm base
        """
        self.planner_pin = Planner_Pin(robot_type=robot_type, end_effector_type=end_effector_type)
        self.arm_T_w = None # transformation matrix from world to arm base
        self.arm_T_cam = None # transformation matrix from camera to arm base(currently not used)
        self.start = None
        self.goal = None

        # TODO: modify the name of arm
        if is_left_arm:
            self.arm = "w1_left_arm_manu_with_torso4"
        else:
            self.arm = "w1_right_arm_manu_with_torso4"
        self.planner_name = "rrtc"
        self.sampler_name = "halton"
        self.skip_rng_iterations = 0

        self.vamp_module, self.planner_func, self.plan_settings, self.simp_settings = \
            vamp.configure_robot_and_planner_with_kwargs(self.arm, self.planner_name, **kwargs)
        self.sampler = getattr(self.vamp_module, self.sampler_name)()
        self.sampler.skip(self.skip_rng_iterations)

        self.env = vamp.Environment()
        self.initial_env = vamp.Environment()
        self.rmin, self.rmax = self.vamp_module.min_max_radii()

        self.logger = get_logger()

        
    def reset_env(self):
        self.env = vamp.Environment()

    def set_head_body_qpos(self, head_qpos: np.ndarray = None, body_qpos: np.ndarray = None):
        if head_qpos is None or body_qpos is None:
            raise ValueError("Head or body qpos is None")
        
        # head_qpos = np.atleast_2d(head_qpos)
        # body_qpos = np.atleast_2d(body_qpos)
        if head_qpos.shape[0] != 2 or body_qpos.shape[0] != 4:
            raise ValueError("Invalid head or body qpos shape")
        
        arml_qpos, armr_qpos = np.zeros(7), np.zeros(7)
        # arml_qpos = np.atleast_2d(arml_qpos)
        # armr_qpos = np.atleast_2d(armr_qpos)
        full_qpos = np.concatenate([body_qpos, head_qpos, arml_qpos, armr_qpos])
        torso_trans, head_trans, _, _ = self.planner_pin.get_Kinematics(real_qpos=full_qpos)

        # NOTE: Left arm and right arm base are both at the waist link since the urdf defination
        #       Remeber to change it if the urdf is changed
        self.arm_T_w = np.linalg.inv(torso_trans[-1])

        # TODO: add arm_T_cam calculation when we can get the camera extrinsics to neck2


    def add_sphere(self, center: np.ndarray = None, radius: float = None):
        if center is None or radius is None or np.atleast_2d(center).shape[1] != 3:
            raise ValueError("Invalid center or radius for sphere")
        center_homo = np.hstack([center, 1.0])
        center_arm = (self.arm_T_w @ center_homo)[:3]
        
        self.env.add_sphere(vamp.Sphere(center_arm.tolist(), radius))


    def add_capsule_len_radius(self, center: np.ndarray = None, euler_xyz: np.ndarray = None, length: float = None, radius: float = None):
        if center is None or radius is None or np.atleast_2d(center).shape[1] != 3:
            raise ValueError("Invalid center or radius for capsule")
        if euler_xyz is None or np.atleast_2d(euler_xyz).shape[1] != 3:
            raise ValueError("Invalid euler angles for capsule")
        if length is None:
            raise ValueError("Invalid length for capsule")
        center_homo = np.hstack([center, 1.0])
        center_arm = (self.arm_T_w @ center_homo)[:3]
        R_world = Rotation.from_euler("xyz", euler_xyz).as_matrix()
        R_arm = self.arm_T_w[:3, :3] @ R_world
        euler_xyz_arm = Rotation.from_matrix(R_arm).as_euler("xyz")

        self.env.add_capsule(vamp.Capsule(center_arm.tolist(), euler_xyz_arm.tolist(), length, radius))

    def add_capsule_two_ends(self, pt1: np.ndarray = None, pt2: np.ndarray = None, radius: float = None):
        if pt1 is None or pt2 is None or radius is None or np.atleast_2d(pt1).shape[1] != 3 or np.atleast_2d(pt2).shape[1] != 3:
            raise ValueError("Invalid points or radius for capsule")
        pt1_homo = np.hstack([pt1, 1.0])
        pt2_homo = np.hstack([pt2, 1.0])
        pt1_arm = (self.arm_T_w @ pt1_homo)[:3]
        pt2_arm = (self.arm_T_w @ pt2_homo)[:3]

        self.env.add_capsule(vamp.Capsule.from_endpoints(pt1_arm.tolist(), pt2_arm.tolist(), radius))

    def add_cuboid(self, center: np.ndarray = None, euler_xyz: np.ndarray = None, half_extents: np.ndarray = None):
        if center is None or half_extents is None or np.atleast_2d(center).shape[1] != 3 or np.atleast_2d(half_extents).shape[1] != 3:
            raise ValueError("Invalid center or half extents for cuboid")
        if euler_xyz is None or np.atleast_2d(euler_xyz).shape[1] != 3:
            raise ValueError("Invalid euler angles for cuboid")
        center_homo = np.hstack([center, 1.0])
        center_arm = (self.arm_T_w @ center_homo)[:3]
        R_world = Rotation.from_euler("xyz", euler_xyz).as_matrix()
        R_arm = self.arm_T_w[:3, :3] @ R_world
        quaternion_arm = Rotation.from_matrix(R_arm).as_quat()
        euler_xyz_arm = Rotation.from_matrix(R_arm).as_euler("xyz")

        self.env.add_cuboid(vamp.Cuboid(center_arm.tolist(), euler_xyz_arm.tolist(), half_extents.tolist()))

    def add_pcd(self, point_cloud, pcd_radius=0.01):
        add_pcd_res = self.env.add_pointcloud(point_cloud, self.rmin, self.rmax, pcd_radius)

    def set_start_goal(self, start, goal):
        if not self.vamp_module.validate(start):
            self.logger.warning(f"Invalid start qpos: {start}")
            print(self.vamp_module.debug(start, self.env))
            return False
        if not self.vamp_module.validate(goal):
            self.logger.warning(f"Invalid goal qpos: {goal}")
            print(self.vamp_module.debug(goal, self.env))
            return False
        self.start = start
        self.goal = goal
        return True

    def plan(self):
        self.sampler.reset()
        self.sampler.skip(self.skip_rng_iterations)
        result = self.planner_func(self.start, self.goal, self.env, self.plan_settings, self.sampler)
        if result.solved:
            simplify = self.vamp_module.simplify(result.path, self.env, self.simp_settings, self.sampler)
            # stats = vamp.results_to_dict(result, simplify)
            # print(
            #     f"""
            #         Planning Time: {stats['planning_time'].microseconds:8d}μs
            #         Simplify Time: {stats['simplification_time'].microseconds:8d}μs
            #         Total Time: {stats['total_time'].microseconds:8d}μs

            #         Planning Iters: {stats['planning_iterations']}
            #         n Graph States: {stats['planning_graph_size']}

            #         Path Length:
            #         Initial: {stats['initial_path_cost']:5.3f}
            #         Simplified: {stats['simplified_path_cost']:5.3f}"""
            #     )
            simplified_path = simplify.path
            simplified_path_numpy = simplified_path.numpy()

            return simplified_path_numpy
        else:
            return None
        


if __name__ == "__main__":
    vamp_planner = VampPlanner()
    vamp_planner.set_start_goal([0.0]*7, [0.5]*7)
    vamp_planner.plan()

        