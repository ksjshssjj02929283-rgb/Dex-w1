import numpy as np
import vamp
import os
import time
from scipy.spatial.transform import Rotation
from dexe_robot_client.w1.path_planner.planner_pinocchio import Planner_Pin
from dexe_robot_client.w1.path_planner.utils.mapping_index import *
from dexe_robot_client.w1.path_planner.utils.robot_type import *
from dexe_robot_client.end_effector_client import EndEffectorType
from IPython import embed

from vamp import pybullet_interface as vpb

class VampPlanner:
    def __init__(self, robot_type: RobotType = RobotType.W1V02_1, end_effector_type: EndEffectorType = None, **kwargs):
        """
        Args:
            arm_T_w: 4x4 np array, transformation from world to arm base
        """
        self.planner_pin = Planner_Pin(robot_type=robot_type, end_effector_type=end_effector_type)
        self.arm_T_w = None
        self.arm_T_cam = None
        self.start = None
        self.goal = None

        self.arm = "w1_left_arm"
        self.planner_name = "rrtc"
        self.sampler_name = "halton"
        self.skip_rng_iterations = 0

        self.vamp_module, self.planner_func, self.plan_settings, self.simp_settings = \
            vamp.configure_robot_and_planner_with_kwargs(self.arm, self.planner_name, **kwargs)
        self.sampler = getattr(self.vamp_module, self.sampler_name)()
        self.sampler.skip(self.skip_rng_iterations)

        self.env = vamp.Environment()
        self.rmin, self.rmax = self.vamp_module.min_max_radii()
        # For visualization
        # self.sim = vpb.PyBulletSimulator(("/home/young/external_repo/vamp/resources/w1_left_arm/w1_left_arm_spherized.urdf"), self.vamp_module.joint_names(), True)
        # self.sim.client.addUserDebugLine([0,0,0], [0.5, 0, 0], [1, 0, 0], lineWidth=3)
        # self.sim.client.addUserDebugLine([0,0,0], [0, 0.5, 0], [0, 1, 0], lineWidth=3)
        # self.sim.client.addUserDebugLine([0,0,0], [0, 0, 0.5], [0, 0, 1], lineWidth=3)
        
    def reset_env(self):
        self.env = vamp.Environment()

    def set_head_body_qpos(self, head_qpos: np.ndarray = None, body_qpos: np.ndarray = None):
        if head_qpos is None or body_qpos is None:
            raise ValueError("Head or body qpos is None")
        if head_qpos.shape[0] != 2 or body_qpos.shape[0] != 4:
            raise ValueError("Invalid head or body qpos shape")
        arml_qpos, armr_qpos = np.zeros(7), np.zeros(7)
        full_qpos = np.concatenate([body_qpos, head_qpos, arml_qpos, armr_qpos])
        torso_trans, head_trans, _, _ = self.planner_pin.get_Kinematics(real_qpos=full_qpos)
        self.arm_T_w = np.linalg.inv(torso_trans[-1])
        # TODO: calculate arm_T_cam


    def add_sphere(self, center: np.ndarray = None, radius: float = None):
        if center is None or radius is None or center.shape[1] != 3:
            raise ValueError("Invalid center or radius for sphere")
        center_homo = np.hstack([center, 1.0])
        center_arm = (self.arm_T_w @ center_homo)[:3]
        
        self.env.add_sphere(vamp.Sphere(center_arm.tolist(), radius))


    def add_capsule_len_radius(self, center: np.ndarray = None, euler_xyz: np.ndarray = None, length: float = None, radius: float = None):
        if center is None or radius is None or center.shape[0] != 3:
            raise ValueError("Invalid center or radius for capsule")
        if euler_xyz is None or euler_xyz.shape[0] != 3:
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
        if pt1 is None or pt2 is None or radius is None or pt1.shape[0] != 3 or pt2.shape[0] != 3:
            raise ValueError("Invalid points or radius for capsule")
        pt1_homo = np.hstack([pt1, 1.0])
        pt2_homo = np.hstack([pt2, 1.0])
        pt1_arm = (self.arm_T_w @ pt1_homo)[:3]
        pt2_arm = (self.arm_T_w @ pt2_homo)[:3]
        self.env.add_capsule(vamp.Capsule.from_endpoints(pt1_arm.tolist(), pt2_arm.tolist(), radius))

    def add_cuboid(self, center: np.ndarray = None, euler_xyz: np.ndarray = None, half_extents: np.ndarray = None):
        if center is None or half_extents is None or center.shape[0] != 3 or half_extents.shape[0] != 3:
            raise ValueError("Invalid center or half extents for cuboid")
        if euler_xyz is None or euler_xyz.shape[0] != 3:
            raise ValueError("Invalid euler angles for cuboid")
        center_homo = np.hstack([center, 1.0])
        center_arm = (self.arm_T_w @ center_homo)[:3]
        R_world = Rotation.from_euler("xyz", euler_xyz).as_matrix()
        R_arm = self.arm_T_w[:3, :3] @ R_world
        quaternion_arm = Rotation.from_matrix(R_arm).as_quat()
        euler_xyz_arm = Rotation.from_matrix(R_arm).as_euler("xyz")
        self.env.add_cuboid(vamp.Cuboid(center_arm.tolist(), euler_xyz_arm.tolist(), half_extents.tolist()))

        # self.sim.add_cuboid(half_extents.tolist(), center_arm.tolist(), quaternion_arm.tolist())

    def add_pcd(self, point_cloud):
        # self.sim.clear_pointcloud()
        # print(self.rmin, self.rmax)
        add_pcd_res = self.env.add_pointcloud(point_cloud, self.rmin, self.rmax, 0.0025)

        # self.sim.draw_pointcloud(point_cloud)

    def sim_add_pcd(self, point_cloud):
        self.sim.draw_pointcloud(point_cloud)
        

    def set_start_goal(self, start, goal):
        if not self.vamp_module.validate(start) or not self.vamp_module.validate(goal):
            raise ValueError("Invalid start or goal")
        self.start = start
        self.goal = goal

    def plan(self):
        solve_cnt = 1
        start_time = time.time()
        
        result = self.planner_func(self.start, self.goal, self.env, self.plan_settings, self.sampler)
        # embed()
        # print(f"First planning time: {time.time() - start_time}s")
        # embed()
        if result.solved:
            simplify = self.vamp_module.simplify(result.path, self.env, self.simp_settings, self.sampler)
            stats = vamp.results_to_dict(result, simplify)
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
            # print(f"solved after {solve_cnt} attempts")

            plan = simplify.path
            # plan.interpolate_to_resolution(self.vamp_module.resolution())

            # for i in range(100):
            #     time.sleep(1)
            #     self.sim.play_once(plan)
            trajectory = plan.numpy()
            # print(trajectory)
            return trajectory
        else:
            # solve_cnt += 1
            # self.sampler.reset()
            # self.sampler.skip(self.skip_rng_iterations)
            # trajectory = self.plan()
            return None
        


if __name__ == "__main__":#
    vamp_planner = VampPlanner()
    vamp_planner.set_start_goal([0.0]*7, [0.5]*7)
    vamp_planner.plan()

        