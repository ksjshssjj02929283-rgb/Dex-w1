from dexe_robot_client.w1.path_planner.vamp_planner import VampPlanner
from dexe_robot_client.w1.path_planner.planner_pinocchio import Planner_Pin
from dexe_robot_client.w1.path_planner.utils.mapping_index import *
from dexe_robot_client.w1.path_planner.utils.robot_type import *
from dexe_robot_client.end_effector_client import EndEffectorType 
from dexe_robot_client.common import W1Version, W1ArmType
from dexe_robot_client.w1 import W1Manager
import numpy as np
import open3d as o3d
import time
import os
from IPython import embed

import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor
from dexe_robot_client.camera_client import OrbbecClient
from dexe_robot_client.w1.trajectory_planner.toppra_trajectory_planner import ToppraTrajectoryPlanner



TABLE_CENTER = np.array([0.8, 0.0, 1.1])
TABLE_SIZE = np.array([1.0, 1.0, 0.03])
# pcd_path = "/home/young/Model/scene.ply"
pcd_path = "/home/young/Downloads/maixi_container_random_grab/data/PickLight/20250409133426269/Builder/foreground/output/20250409133426269.ply"


def visualize_traj(_articulation, arml_traj, armr_traj):
    current_qpos = _articulation.get_current_qpos()[sim2real_idx]
    cur_torso_qpos = current_qpos[real_torso_idx[0]:real_torso_idx[1]+1]
    cur_head_qpos = current_qpos[real_head_idx[0]:real_head_idx[1]+1]

    vis_length = max(arml_traj.shape[0], armr_traj.shape[0])
    for i in range(vis_length):
        if i < arml_traj.shape[0]:
            target_arml_qpos = arml_traj[i]
        else:
            target_arml_qpos = arml_traj[-1]
        if i < armr_traj.shape[0]:
            target_armr_qpos = armr_traj[i]
        else:
            target_armr_qpos = armr_traj[-1]
        target_qpos = np.concatenate([cur_torso_qpos, cur_head_qpos, target_arml_qpos, target_armr_qpos])
        _articulation.set_current_qpos(target_qpos[real2sim_idx])
        time.sleep(0.01)

def _gen_trajectory_with_toppra(path, max_v = 0.3, last_velocity = np.zeros(SINGLE_ARM_DIM)):
        dofs = SINGLE_ARM_DIM
        vel_limits = [max_v] * dofs
        acc_limits = [0.5] * dofs
        traj_planner = ToppraTrajectoryPlanner(dofs, vel_limits, acc_limits)
        # traj_planner.set_last_joint_vel(last_velocity)
        res, pos_array, vel_array, acc_array, duration = traj_planner.plan(path)
        return pos_array, vel_array, acc_array, duration

def test_vamp_planner(articulation, current_point_cloud, arml_goal):

    current_qpos = articulation.get_current_qpos()[sim2real_idx]
    
    cur_torso, cur_head, cur_arml, cur_armr = current_qpos[real_torso_idx[0]:real_torso_idx[1]+1], current_qpos[real_head_idx[0]:real_head_idx[1]+1], \
                                              current_qpos[real_arml_idx[0]:real_arml_idx[1]+1], current_qpos[real_armr_idx[0]:real_armr_idx[1]+1]    
    vamp_planner.reset_env()
    vamp_planner.add_pcd(np.asarray(current_point_cloud.points).tolist())

    arml_start = cur_arml.tolist()

    if vamp_planner.vamp_module.validate(arml_start) and vamp_planner.vamp_module.validate(arml_goal):

        vamp_planner.set_start_goal(start=arml_start, goal=arml_goal)
        print(f"start validate: {vamp_planner.vamp_module.validate(arml_start)}, goal validate: {vamp_planner.vamp_module.validate(arml_goal)}")
        start_time = time.time()
        result = vamp_planner.plan()
        result_traj = _gen_trajectory_with_toppra(result, max_v=0.3)[0]
        print(f"Planning time: {time.time() - start_time}s")
        if result is not None:
            # arml_traj = result_traj
            arml_traj = result
            armr_traj = np.atleast_2d(cur_armr)
            visualize_traj(articulation, arml_traj, armr_traj)

    # embed()

    


if __name__ == "__main__":
    import dexsim
    config = dexsim.WorldConfig()
    config.open_windows = True
    _world = dexsim.World(config)
    _world.show_coordinate_axis(True)
    _world.enable_physics(True)
    _env = _world.get_env()
    ibl_data = dexsim.data.IBL()
    iblight_fpath = ibl_data.extract_dir + "/IBL/lightroom_14b"
    _env.set_IBL(iblight_fpath)

    default_length = 1000
    repeat_uv_size = int(default_length / 2)
    plane_handle = _env.create_plane(
        0, default_length, repeat_uv_size, repeat_uv_size
    )
    plane_handle.set_name("default_plane")
    plane_collision_handle = _env.create_cube(default_length, 
                                             default_length, 
                                             default_length / 10)
    plane_collision_xpos = np.eye(4, dtype=float)
    plane_collision_xpos[2, 3] = - default_length / 20 - 0.001
    plane_collision_handle.set_local_pose(plane_collision_xpos)
    plane_collision_handle.add_rigidbody(dexsim.types.ActorType.STATIC, 
                                         dexsim.types.RigidBodyShape.CONVEX)
    
    material_data = dexsim.data.Materials()
    material_fpath = material_data.extract_dir + "/materials"
    material_handle = _env.load_material(material_fpath + "/Concrete033", "Concrete033")
    plane_handle.set_material(material_handle.get_inst())


    robot_urdf_path = get_robot_urdf(RobotType.W1V02_1)
    _articulation = _env.load_urdf(robot_urdf_path)
    robot_pose = np.eye(4, dtype=float)
    _articulation.set_world_pose(robot_pose)
    _articulation.set_articulation_flag(dexsim.types.ArticulationFlag.DISABLE_SELF_COLLISION, True)
    _articulation.set_drive(drive_type=dexsim.types.DriveType.FORCE)

    w1_manager = W1Manager(is_sim=True,
                           arm_type=W1ArmType.ANTHROPOMORPHIC,
                           w1_version=W1Version.V2,
                           end_effector_type=None,
                           use_rlia=False)
    if not w1_manager.connect():
        raise RuntimeError("Failed to connect to W1 robot.")


    sim2real_idx = get_sim2real_idx(articulation=_articulation)
    real2sim_idx = get_real2sim_idx(articulation=_articulation)
    pin2real_idx = get_pin2real_idx(robot_urdf_path)
    real2pin_idx = get_real2pin_idx(robot_urdf_path)
    pin_torso_idx, pin_head_idx, pin_arml_idx, pin_armr_idx = get_pin_idx(robot_urdf_path)
    real_torso_idx, real_head_idx, real_arml_idx, real_armr_idx = get_real_idx()


    init_head_qpos = np.deg2rad([0, -25])
    init_body_qpos = np.deg2rad([45, -90, 45, 2])
    init_arml_qpos = np.array([-np.pi / 2, -np.pi / 2, np.pi / 2, -np.pi / 4, -np.pi / 2, np.pi / 5, 0.0])
    init_armr_qpos = -np.array([-np.pi / 2, -np.pi / 4, np.pi / 2, -np.pi / 4, -np.pi / 2, np.pi / 5, 0.0])
    init_q = np.concatenate([init_body_qpos, init_head_qpos, init_arml_qpos, init_armr_qpos])
    _articulation.set_current_qpos(init_q[real2sim_idx])

    planner_pin = Planner_Pin(robot_type=RobotType.W1V02_1, end_effector_type=None)
    torso_tf, head_tf, arml_tf, armr_tf = planner_pin.get_Kinematics(real_qpos=init_q)
    arml_base_tf, armr_base_tf = planner_pin.get_arm_base_tf(real_qpos=init_q)

    base_link_T_arml_base = arml_base_tf
    base_link_T_waist = torso_tf[-1]
    waist_T_arm_base = np.linalg.inv(base_link_T_waist) @ base_link_T_arml_base

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

    orbbec_node = OrbbecClient()

    executor = MultiThreadedExecutor()
    executor.add_node(orbbec_node)

    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    if not orbbec_node.is_connected():
        time.sleep(1)


    current_qpos = _articulation.get_current_qpos()[sim2real_idx]
    
    cur_torso, cur_head, cur_arml, cur_armr = current_qpos[real_torso_idx[0]:real_torso_idx[1]+1], current_qpos[real_head_idx[0]:real_head_idx[1]+1], \
                                              current_qpos[real_arml_idx[0]:real_arml_idx[1]+1], current_qpos[real_armr_idx[0]:real_armr_idx[1]+1]    

    vamp_planner = VampPlanner(robot_type=RobotType.W1V02_1, ee_type=None)
    vamp_planner.set_head_body_qpos(head_qpos=cur_head, body_qpos=cur_torso)

    arml_goal1 = [-np.pi / 2, -np.pi / 2, np.pi / 2, -np.pi / 4, -np.pi / 2, np.pi / 5, 0.0]
    arml_goal2 = [0, 0, 0, 0, 0, 0, 0]
    ply_handle = None
    count = 0
    while True:
        if ply_handle is not None:
            _env.remove_point_cloud(ply_handle)
            time.sleep(0.1)

        current_point_cloud = orbbec_node.get_pcd_numpy()[1]
        
    
        o3d_point_cloud = o3d.geometry.PointCloud()
        o3d_point_cloud.points = o3d.utility.Vector3dVector(current_point_cloud)
        o3d_point_cloud = o3d_point_cloud.voxel_down_sample(voxel_size=0.01)
        base_link_T_camera = base_link_T_arml_base @ cam_arml_extrinsics
        o3d_point_cloud.transform(base_link_T_camera)

        saved_pcd_path = "/home/young/workspace/dexe_robot_client/examples/output/saved_pcd.ply"
        o3d.io.write_point_cloud(saved_pcd_path, o3d_point_cloud)

        ply_handle = _env.load_point_cloud(saved_pcd_path)
        
        if count % 2 == 1:
            arml_goal = arml_goal1
        else:
            arml_goal = arml_goal2
        count += 1

        planner_point_cloud = o3d.geometry.PointCloud()
        planner_point_cloud.points = o3d.utility.Vector3dVector(current_point_cloud)
        planner_point_cloud = planner_point_cloud.voxel_down_sample(voxel_size=0.01)

        planner_point_cloud.transform(waist_T_arm_base @ cam_arml_extrinsics)

        # test_vamp_planner(_articulation, planner_point_cloud, arml_goal)
        time.sleep(1)


    embed()
    # test_vamp_planner(_articulation, planner_pin, _env)

