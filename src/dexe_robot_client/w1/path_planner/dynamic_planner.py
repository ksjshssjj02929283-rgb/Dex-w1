from dexe_robot_client.w1.path_planner.vamp_planner import VampPlanner
from dexe_robot_client.w1.path_planner.planner_pinocchio import Planner_Pin
from dexe_robot_client.w1.path_planner.utils.mapping_index import *
from dexe_robot_client.w1.path_planner.utils.robot_type import *
from dexe_robot_client.end_effector_client import EndEffectorType 
from dexe_robot_client.camera_client import OrbbecClient
from dexe_robot_client.trajectory_client import TrajectoryPublisher
from dexe_robot_client.w1 import W1Manager
from dexe_robot_client.common import get_logger
from dexe_robot_client.w1.trajectory_planner.toppra_trajectory_planner import ToppraTrajectoryPlanner

from dexe_robot_client.end_effector_client import EndEffectorType 
from dexe_robot_client.common import W1Version, W1ArmType

import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import open3d as o3d
import time
from IPython import embed


class DynamicPlanner:
    cam_arml_extrinsics, cam_armr_extrinsics = None, None
    cam_neck2_extrinsics = None
    w1_manager = None
    dexsim_articulation = None
    dexsim_env = None
    dexsim_pcd_handle = None

    @classmethod
    def set_camera_arm_extrinsics(cls, cam_arml_extrinsics, cam_armr_extrinsics):
        """
        This function is used to set the transformation matrix arm_T_cam
        Args:
            cam_arml_extrinsics: (nd.array, shape=(4, 4)) The transformation matrix from camera to left arm base
            cam_armr_extrinsics: (nd.array, shape=(4, 4)) The transformation matrix from camera to right arm base
        """

        cls.cam_arml_extrinsics = cam_arml_extrinsics
        cls.cam_armr_extrinsics = cam_armr_extrinsics

    @classmethod
    def set_camera_neck2_extrinsics(cls, cam_neck2_extrinsics):
        """
        This function is used to set the transformation matrix neck2_T_cam

        Args:
            cam_neck2_extrinsics: (nd.array, shape=(4, 4)) The transformation matrix from camera to neck2
        """

        cls.cam_neck2_extrinsics = cam_neck2_extrinsics

    @classmethod
    def set_w1_manager(cls, w1_manager):
        """
        This function is used to set the W1Manager object

        Args:
            w1_manager: (W1Manager) The W1Manager object
        """
        if not isinstance(w1_manager, W1Manager):
            raise ValueError("w1_manager should be an instance of W1Manager")
        cls.w1_manager = w1_manager

    @classmethod
    def set_dexsim(cls, dexsim_articulation, dexsim_env, pcd_handle = None):
        """
        This function is used to set the dexsim articulation object

        Args:
            dexsim_articulation: (dexsim.Articulation) The dexsim articulation object
            dexsim_env: (dexsim.Environment) The dexsim environment object
            pcd_handle: The point cloud handle in dexsim environment
        """
        cls.dexsim_articulation = dexsim_articulation
        cls.dexsim_env = dexsim_env
        cls.dexsim_pcd_handle = pcd_handle


    def __init__(self, robot_type: RobotType = None, end_effector_type: EndEffectorType = None):
        self.logger = get_logger()
        self.planner_pin = Planner_Pin(robot_type=robot_type, end_effector_type=end_effector_type)
        self.executor = MultiThreadedExecutor()

        # just for update dexsim server environment
        self._dexsim_pcd_node = Node("dexsim_pcd_node")
        self.pcd_env_publisher = self._dexsim_pcd_node.create_publisher(
            String, "/dexe_robot_client/dexsim_server/update_env", 10
        )
        self.executor.add_node(self._dexsim_pcd_node)

        self._init_camera()
        self._init_trajectory_client()
        self._init_ros_thread()
        self._check_connection()
        
        self.vamp_planner = VampPlanner(robot_type=robot_type, end_effector_type=end_effector_type)
        self.start_qpos, self.target_qpos = None, None
        if self.cam_arml_extrinsics is None or self.cam_armr_extrinsics is None:
            self.logger.warning("Camera to arm extrinsics not set, please set it using set_camera_arm_extrinsics function")

        self.last_velocity = None
        self.last_traj = None
        self.is_first_plan = True

        

    def _init_camera(self):
        if not rclpy.ok():
            rclpy.init()
        self.camera_node = OrbbecClient()
        self.executor.add_node(self.camera_node)

    def _init_trajectory_client(self):
        if not rclpy.ok():
            rclpy.init()
        self.traj_client_node = TrajectoryPublisher()
        self.executor.add_node(self.traj_client_node)

        
    def _check_connection(self):
        if not self.camera_node.is_connected():
            time.sleep(0.5)
        

    def _init_ros_thread(self):
        if not rclpy.ok():
            rclpy.init()
        
        self.ros_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.ros_thread.start()

    def set_start_target_qpos(self, start_qpos: np.ndarray, target_qpos: np.ndarray):
        if target_qpos is None or target_qpos.shape[0] != SINGLE_ARM_DIM:
            self.logger.error("Invalid target_qpos shape")
            return False
        if start_qpos is None or start_qpos.shape[0] != SINGLE_ARM_DIM:
            self.logger.error("Invalid start_qpos shape")
            return False
        self.start_qpos = start_qpos
        self.target_qpos = target_qpos
        return True

    def compute_camera_neck2_extrinsics(self, torso_qpos: np.ndarray, head_qpos: np.ndarray):
        """
        This function computes the fixed transformation matrix neck2_T_cam given the torso and head qpos and arm_base_T_cam(cam_arm_extrinsics)

        Args:
            torso_qpos: (nd.array, shape=(4, )) The torso qpos of real robot
            head_qpos: (nd.array, shape=(2, )) The head qpos of real robot
        """
        if self.cam_arml_extrinsics is None or self.cam_armr_extrinsics is None:
            raise ValueError("Please set cam_arm_extrinsics first")
        zero_arm_qpos = np.zeros(SINGLE_ARM_DIM)
        full_qpos = np.concatenate([torso_qpos, head_qpos, zero_arm_qpos, zero_arm_qpos])
        torso_tf, head_tf, arml_tf, armr_tf = self.planner_pin.get_Kinematics(real_qpos=full_qpos)
        base_T_neck1 = head_tf[0]
        base_T_arml_base, base_T_armr_base = self.planner_pin.get_arm_base_tf(real_qpos=full_qpos)
        arml_base_T_neck1 = np.linalg.inv(base_T_arml_base) @ base_T_neck1
        armr_base_T_neck1 = np.linalg.inv(base_T_armr_base) @ base_T_neck1
        neck1_T_neck2 = np.linalg.inv(head_tf[0]) @ head_tf[1]
        neck2_T_cam_from_arml = np.linalg.inv(arml_base_T_neck1 @ neck1_T_neck2) @ self.cam_arml_extrinsics
        neck2_T_cam_from_armr = np.linalg.inv(armr_base_T_neck1 @ neck1_T_neck2) @ self.cam_armr_extrinsics
        
        return neck2_T_cam_from_arml, neck2_T_cam_from_armr
    
    def _get_base_link_T_arm_base(self):
        if self.w1_manager is None:
            self.logger.warning("W1Manager is not set, cannot get base_link_T_arm_base")
            return False, None, None
        current_qpos = self.w1_manager.get_current_qpos()[1]
        cur_arml_base_tf, cur_armr_base_tf = self.planner_pin.get_arm_base_tf(real_qpos=current_qpos)
        return True, cur_arml_base_tf, cur_armr_base_tf

    def _get_waist_T_arm_base(self):
        # FIXME: Since current single arm urdf is based on waist joint, we need transfer the point cloud to waist frame
        if self.w1_manager is None:
            self.logger.warning("W1Manager is not set, cannot get base_link_T_arm_base")
            return False, None, None
        current_qpos = self.w1_manager.get_current_qpos()[1]
        torso_tf, _, _, _ = self.planner_pin.get_Kinematics(real_qpos=current_qpos)
        cur_arml_base_tf, cur_armr_base_tf = self.planner_pin.get_arm_base_tf(real_qpos=current_qpos)
        base_link_T_waist = torso_tf[-1]
        waist_T_arml_base = np.linalg.inv(base_link_T_waist) @ cur_arml_base_tf
        waist_T_armr_base = np.linalg.inv(base_link_T_waist) @ cur_armr_base_tf
        return True, waist_T_arml_base, waist_T_armr_base
    
    def _get_current_arm_qpos(self):
        if self.w1_manager is None:
            self.logger.warning("W1Manager is not set, cannot get current qpos")
            return False, None, None
        current_qpos = self.w1_manager.get_current_qpos()[1]
        cur_arml_qpos = current_qpos[self.w1_manager.real_arml_idx[0]:self.w1_manager.real_arml_idx[1]+1]
        cur_armr_qpos = current_qpos[self.w1_manager.real_armr_idx[0]:self.w1_manager.real_armr_idx[1]+1]
        return True, cur_arml_qpos, cur_armr_qpos
    
    def _get_current_arm_vel(self):
        if self.w1_manager is None:
            self.logger.warning("W1Manager is not set, cannot get current qpos")
            return False, None, None
        current_robot_info = self.w1_manager.robot_client.get_info()
        cur_vel = np.array(current_robot_info.joint_velocity)
        cur_arml_vel = cur_vel[self.w1_manager.real_arml_idx[0]:self.w1_manager.real_arml_idx[1]+1]
        cur_armr_vel = cur_vel[self.w1_manager.real_armr_idx[0]:self.w1_manager.real_armr_idx[1]+1]
        return True, cur_arml_vel, cur_armr_vel
    
    def _get_current_full_qpos(self):
        if self.w1_manager is None:
            self.logger.warning("W1Manager is not set, cannot get current qpos")
            return False, None, None, None, None
        current_qpos = self.w1_manager.get_current_qpos()[1]
        cur_torso_qpos = current_qpos[self.w1_manager.real_torso_idx[0]:self.w1_manager.real_torso_idx[1]+1]
        cur_head_qpos = current_qpos[self.w1_manager.real_head_idx[0]:self.w1_manager.real_head_idx[1]+1]
        cur_arml_qpos = current_qpos[self.w1_manager.real_arml_idx[0]:self.w1_manager.real_arml_idx[1]+1]
        cur_armr_qpos = current_qpos[self.w1_manager.real_armr_idx[0]:self.w1_manager.real_armr_idx[1]+1]
        return True, cur_torso_qpos, cur_head_qpos, cur_arml_qpos, cur_armr_qpos
    
    def _get_qpos_from_topic(self):
        current_pub_qpos, current_pub_vel = self.traj_client_node.get_streaming_cmd()
        if current_pub_qpos is None:
            return True, self._get_current_arm_qpos()[1], np.zeros(7)
        current_arml_qpos = np.array(current_pub_qpos[self.w1_manager.real_arml_idx[0]:self.w1_manager.real_arml_idx[1]+1])
        current_arml_vel = np.array(current_pub_vel[self.w1_manager.real_arml_idx[0]:self.w1_manager.real_arml_idx[1]+1])
        return True, current_arml_qpos, current_arml_vel

    def _reload_environment(self):
        current_pcd = self.camera_node.get_pcd_numpy()[1]
        if current_pcd is None or current_pcd.shape[0] == 0:
            self.logger.warning("No point cloud data from camera")
            return False, None
        # if self.dexsim_env is not None and self.dexsim_articulation is not None:
        #     if self.dexsim_pcd_handle is not None:
        #         self.dexsim_env.remove_point_cloud(self.dexsim_pcd_handle)
        #         self.dexsim_pcd_handle = None
        #     vis_pcd = o3d.geometry.PointCloud()
        #     vis_pcd.points = o3d.utility.Vector3dVector(current_pcd)
        #     o3d_point_cloud = o3d_point_cloud.voxel_down_sample(voxel_size=0.01)
        #     res, base_link_T_arml_base, base_link_T_armr_base = self._get_base_link_T_arm_base()
        #     if not res:
        #         return False, None
        #     # choose one arm to transform the point cloud to arm base frame
        #     # TODO: check if base_link_T_camera got from left arm or right arm are same
        #     base_link_T_camera = base_link_T_arml_base @ self.cam_arml_extrinsics
        #     vis_pcd.transform(base_link_T_camera)
        #     # FIXME: Modify the path
        #     saved_pcd_path = "/home/young/workspace/dexe_robot_client/examples/output/dexsim_vis_pcd.ply"
        #     o3d.io.write_point_cloud(saved_pcd_path, vis_pcd)
        #     self.dexsim_pcd_handle = self.dexsim_env.load_point_cloud(saved_pcd_path)
        #     self.pcd_env_publisher.publish(String(data="update"))

        vis_pcd = o3d.geometry.PointCloud()
        vis_pcd.points = o3d.utility.Vector3dVector(current_pcd)
        vis_pcd = vis_pcd.voxel_down_sample(voxel_size=0.01)
        res, base_link_T_arml_base, base_link_T_armr_base = self._get_base_link_T_arm_base()
        if not res:
            return False, None
        # choose one arm to transform the point cloud to arm base frame
        # TODO: check if base_link_T_camera got from left arm or right arm are same
        base_link_T_camera = base_link_T_arml_base @ self.cam_arml_extrinsics
        vis_pcd.transform(base_link_T_camera)
        # FIXME: Modify the path
        saved_pcd_path = "/home/young/workspace/dexe_robot_client/examples/output/dexsim_vis_pcd.ply"
        o3d.io.write_point_cloud(saved_pcd_path, vis_pcd)
        # self.dexsim_pcd_handle = self.dexsim_env.load_point_cloud(saved_pcd_path)
        self.pcd_env_publisher.publish(String(data="update"))


        planner_pcd = o3d.geometry.PointCloud()
        planner_pcd.points = o3d.utility.Vector3dVector(current_pcd)
        planner_pcd = planner_pcd.voxel_down_sample(voxel_size=0.01)
        res, waist_T_arml_base, waist_T_armr_base = self._get_waist_T_arm_base()
        if not res:
            return False, None
        # choose one arm to transform the point cloud to waist frame
        planner_pcd.transform(waist_T_arml_base @ self.cam_arml_extrinsics)
        return True, planner_pcd
    
    def _check_start_goal_validity(self):
        res, cur_arml_qpos, cur_armr_qpos = self._get_current_arm_qpos()
        if not res:
            self.logger.warning("Get current qpos failed during checking start and goal validity")
            return False
        if self.vamp_planner.vamp_module.validate(cur_arml_qpos.tolist()) \
            and self.vamp_planner.vamp_module.validate(self.target_qpos.tolist()):
            return True
        else:
            self.logger.warning("Start or goal qpos is invalid for some reason")
            return False
        
    def _gen_trajectory_with_toppra(self, path, max_v = 0.3, last_velocity = np.zeros(SINGLE_ARM_DIM)):
        dofs = SINGLE_ARM_DIM
        vel_limits = [max_v] * dofs
        acc_limits = [0.5] * dofs
        traj_planner = ToppraTrajectoryPlanner(dofs, vel_limits, acc_limits)
        traj_planner.set_last_joint_vel(last_velocity)
        res, pos_array, vel_array, acc_array, duration = traj_planner.plan_with_last_vel(path)
        if not res:
            res, pos_array, vel_array, acc_array, duration = traj_planner.plan(path)
        return pos_array, vel_array, acc_array, duration

    def plan(self, update_frequency: float = 5.0):
        # TODO: currently use time.sleep to simulate the update frequency, need to be improved
        # Try to figure out apscheduler method or ROS timer method or other methods
        # TODO: currently only support left arm planning
        if self.target_qpos is None:
            self.logger.error("Target qpos not set, please set it using set_target_qpos function")
            return False, None
        is_first_plan = True
        while True:
            time.sleep(1.0 / update_frequency)
        
            res, planner_pcd = self._reload_environment()
            if not res:
                self.logger.warning("Reload environment failed")
                continue
            self.vamp_planner.reset_env()
            self.vamp_planner.add_pcd(point_cloud=np.asarray(planner_pcd.points).tolist())
            

            # res, cur_arml_qpos, cur_armr_qpos = self._get_current_arm_qpos()
            res, cur_arml_qpos, cur_arml_vel = self._get_qpos_from_topic()
            self.logger.debug(f"current arml qpos: {cur_arml_qpos}, current arml vel: {cur_arml_vel}")
            self.logger.debug(f"target arml qpos: {self.target_qpos}")
            if not res:
                self.logger.warning("Get current qpos failed")
                continue
            if np.allclose(cur_arml_qpos, self.target_qpos, rtol=1e-03, atol=1e-03): # FIXME: need to tune the threshold
                self.logger.info("Current qpos is close to target qpos, no need to plan")
                break
            if not self._check_start_goal_validity():
                continue
            if is_first_plan:
                is_first_plan = False
                self.vamp_planner.set_start_goal(start=self.start_qpos, goal=self.target_qpos)
            else:
                self.vamp_planner.set_start_goal(start=cur_arml_qpos, goal=self.target_qpos)
            start_time = time.time()
            # TODO: change to dual arm planning with result is 14 dim
            result = self.vamp_planner.plan()
            
            if result is not None:
                # if self.is_first_plan:
                #     self.is_first_plan = False
                #     self.last_velocity = np.zeros(SINGLE_ARM_DIM)
                # else:
                #     stop_idx = int((1.0/update_frequency) / 0.01)
                #     self.last_velocity = (self.last_traj[stop_idx] - self.last_traj[stop_idx-1]) / 0.01
                self.last_velocity = cur_arml_vel
                traj_result = self._gen_trajectory_with_toppra(result, max_v=0.3, last_velocity=self.last_velocity)[0]
                self.last_traj = traj_result
                # publish the result trajectory
                full_traj = np.zeros((traj_result.shape[0], 20))
                res, cur_torso_qpos, cur_head_qpos, cur_arml_qpos, cur_armr_qpos = self._get_current_full_qpos()
                for i in range(traj_result.shape[0]):
                    full_qpos = np.concatenate([cur_torso_qpos, cur_head_qpos, traj_result[i], cur_armr_qpos])
                    full_traj[i] = full_qpos
                
                self.traj_client_node.publish_trajectory(trajectory=full_traj[2:, :])
                self.logger.info(f"Planning time: {time.time() - start_time}s")
                self.logger.info("Planning succeeded, publishing trajectory")
            else:
                self.logger.warning("Planning failed, no trajectory to publish")

        self.traj_client_node.publish_trajectory(trajectory=None)
        # self.is_first_plan = True
        # self.last_velocity = None



if __name__ == "__main__":
    cam_arml_extrinsics = np.array([
        [0.04763052712431963, -0.7875535718815841, -0.6144027215942477, 0.19295156040533418],
        [0.03643103300760852, -0.6133219024721747, 0.7889924104716797, 0.1114495388646769],
        [-0.9982004371466978, -0.05996345023821815, -0.0005214553391622478, -0.05491695129690629],
        [-0.0, 0.0, -0.0, 1.0]
    ])

    cam_armr_extrinsics = np.array([
        [-0.01612841486046863, 0.7942157929747202, 0.607421722054476, -0.1843642799316683],
        [0.02227139878794798, -0.6070646593743736, 0.7943402823316368, 0.10413016922371413],
        [0.9996218580193799, 0.02633958101816373, -0.007897305975507406, -0.11974728220940557],
        [-0.0, 0.0, -0.0, 1.0]
    ])

    w1_manager = W1Manager(is_sim=True,
                           arm_type=W1ArmType.ANTHROPOMORPHIC,
                           w1_version=W1Version.V2,
                           end_effector_type=None,
                           use_rlia=False)
    if not w1_manager.connect():
        raise RuntimeError("Failed to connect to W1 robot.")

    DynamicPlanner.set_camera_arm_extrinsics(cam_arml_extrinsics, cam_armr_extrinsics)
    DynamicPlanner.set_w1_manager(w1_manager)

    dynamic_planner = DynamicPlanner(robot_type=RobotType.W1V02_1, end_effector_type=None)

    w1_manager.set_tcp("left_arm", np.eye(4))
    w1_manager.set_tcp("right_arm", np.array([[-1,0,0,0],
                                              [0,-1,0,0],
                                              [0,0,1,0],
                                              [0,0,0,1]]))
    # init_torso_qpos = np.deg2rad([45, -90, 45, 2])
    # init_head_qpos = np.deg2rad([0, -25])
    # init_arml_qpos = np.array([-np.pi / 2, -np.pi / 4, np.pi / 2, -np.pi / 4, -np.pi / 2, np.pi / 5, 0.0])
    # init_armr_qpos = -np.array([-np.pi / 2, -np.pi / 4, np.pi / 2, -np.pi / 4, -np.pi / 2, np.pi / 5, 0.0])
    # init_qpos = np.concatenate([init_torso_qpos, init_head_qpos, init_arml_qpos, init_armr_qpos])
    # w1_manager.set_home_qpos(init_qpos,
    #                         save_to_file=False)
    w1_manager.set_home_qpos(np.deg2rad([45, -90, 45, 2, 
                                         0, -25,
                                         0, 0, 0, 0, 0, 0, 0,
                                         0, 0, 0, 0, 0, 0, 0]), save_to_file=False)
    w1_manager.go_home(2)

    arml_target_qpos1 = np.array([-np.pi / 2, -np.pi / 2, np.pi / 2, -np.pi / 4, -np.pi / 2, np.pi / 5, 0.0])
    arml_target_qpos2 = np.zeros(7)


    # dynamic_planner.set_start_target_qpos(arml_target_qpos2, arml_target_qpos1)
    # dynamic_planner.plan(update_frequency=2)
    cnt = 0
    while True:
        if cnt % 2 == 0:
            arml_start_qpos = arml_target_qpos2
            arml_target_qpos = arml_target_qpos1
        else:
            arml_start_qpos = arml_target_qpos1
            arml_target_qpos = arml_target_qpos2
        cnt += 1
        dynamic_planner.set_start_target_qpos(arml_start_qpos, arml_target_qpos)

        dynamic_planner.plan(update_frequency=2)
        input("Press Enter to continue...")
    
    