import numpy as np
import os
import open3d as o3d
import time
from scipy.spatial.transform import Rotation
from dexe_robot_client.end_effector_client import EndEffectorType 
from dexe_robot_client.common import W1Version, W1ArmType
from dexe_robot_client.w1 import W1Manager
from dexe_robot_client.w1.path_planner.utils.robot_type import *
from dexe_robot_client.w1.path_planner.utils.mapping_index import *
from dexe_robot_client.w1.path_planner.planner_pinocchio import Planner_Pin
from dexe_robot_client.w1.path_planner.fcl_checker import FCLChecker
from IPython import embed
from copy import deepcopy 
import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor
from dexe_robot_client.camera_client import OrbbecClient

def save_point_cloud(point_cloud):
    saved_folder_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")
    if not os.path.exists(saved_folder_path):
        os.makedirs(saved_folder_path)
    saved_file_path = os.path.join(saved_folder_path, "point_cloud.ply")
    o3d.io.write_point_cloud(saved_file_path, point_cloud)

def xyzabc_to_extrinsics(x, y, z, a, b, c):
    R_mat = Rotation.from_euler('xyz', [a, b, c], degrees = True).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R_mat
    T[:3, 3] = [x, y, z]
    return T

def test_octomap():
    fcl_checker = FCLChecker(robot_type=robot_type, end_effector_type=robot_ee_type)
    fcl_checker.enable_visualize()
    fcl_checker.add_octomap_from_numpy(point_cloud=current_point_cloud,
                                       extrinsics=cam_arml_extrinsics,
                                       octo_res=0.01,
                                       base_link_T_arm_base=base_link_T_arml_base,
                                       visualize=True)
    # fcl_checker.add_octomap_from_file(ply_path="/home/young/Downloads/maixi_container_random_grab/data/PickLight/20250409133426269/Builder/foreground/output/20250409133426269.ply",
    #                                    extrinsics=cam_arml_extrinsics,
    #                                    octo_res=0.01,
    #                                    base_link_T_arm_base=base_link_T_arml_base,
    #                                    visualize=True)
    fcl_checker.update_vis_transform(torso_tf+head_tf+arml_tf+armr_tf)


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
    # robot_urdf_path = "/home/young/Downloads/W1_V2_/W1_V2_A_BRAINCO/W1_V2_A_BRAINCO.urdf"
    _articulation = _env.load_urdf(robot_urdf_path)
    robot_pose = np.eye(4, dtype=float)
    # robot_pose[2, 3] = 1.5
    _articulation.set_world_pose(robot_pose)
    _articulation.set_articulation_flag(dexsim.types.ArticulationFlag.DISABLE_SELF_COLLISION, True)
    _articulation.set_drive(drive_type=dexsim.types.DriveType.FORCE)


    robot_type = RobotType.W1V02_1
    robot_ee_type = EndEffectorType.BRAINCO_HAND
    w1_manager = W1Manager(is_sim=True,
                           arm_type=W1ArmType.ANTHROPOMORPHIC,
                           w1_version=W1Version.V2,
                           end_effector_type=robot_ee_type,
                           use_rlia=False)
    if not w1_manager.connect():
        raise RuntimeError("Failed to connect to W1 robot.")
    
    w1_manager.set_tcp("left_arm", np.eye(4))
    w1_manager.set_tcp("right_arm", np.array([[-1,0,0,0],
                                              [0,-1,0,0],
                                              [0,0,1,0],
                                              [0,0,0,1]]))
    
    planner_pin = Planner_Pin(robot_type=robot_type, end_effector_type=robot_ee_type)

    # cam_arml_extrinsics = xyzabc_to_extrinsics(0.03581, 0.54907, 0.07597, -120.42, 102.88, 16.49)
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


    torso_init = np.deg2rad([45, -90, 45, 2])
    head_init = np.deg2rad([0, -25])
    arml_init, armr_init = np.zeros(7), np.zeros(7)
    init_qpos = np.concatenate([torso_init, head_init, arml_init, armr_init])
    w1_manager.set_home_qpos(init_qpos,
                            save_to_file=False)
    w1_manager.set_ee_qpos("left_arm_ee", np.array([0.]*6))
    w1_manager.set_ee_qpos("right_arm_ee", np.array([0.]*6))
    w1_manager.go_home(1)

    real2sim_idx = get_real2sim_idx(_articulation)
    _articulation.set_current_qpos(init_qpos[real2sim_idx])

    current_qpos = w1_manager.get_current_qpos()[1]
    cur_torso_qpos = current_qpos[w1_manager.real_torso_idx[0]:w1_manager.real_torso_idx[1]+1]
    cur_head_qpos = current_qpos[w1_manager.real_head_idx[0]:w1_manager.real_head_idx[1]+1]
    cur_left_arm_qpos = current_qpos[w1_manager.real_arml_idx[0]:w1_manager.real_arml_idx[1]+1]
    cur_right_arm_qpos = current_qpos[w1_manager.real_armr_idx[0]:w1_manager.real_armr_idx[1]+1]

    real_qpos = np.concatenate([cur_torso_qpos, cur_head_qpos, cur_left_arm_qpos, cur_right_arm_qpos])
    torso_tf, head_tf, arml_tf, armr_tf = planner_pin.get_Kinematics(real_qpos=real_qpos)
    arml_base_tf, armr_base_tf = planner_pin.get_arm_base_tf(real_qpos=real_qpos)

    base_link_T_arml_base = arml_tf[0]

    orbbec_node = OrbbecClient()

    executor = MultiThreadedExecutor()
    executor.add_node(orbbec_node)

    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    if not orbbec_node.is_connected():
        time.sleep(1)

    current_point_cloud = orbbec_node.get_pcd_numpy()[1]

    # o3d_point_cloud = o3d.geometry.PointCloud()
    # o3d_point_cloud.points = o3d.utility.Vector3dVector(current_point_cloud)
    
    # test_point_cloud_path = "/home/young/Pictures/2025_09_10_14_52_18/KFC-ROS_KFC-ROS.0_2025_09_10_14_52_18.ply"
    # test_point_cloud = o3d.io.read_point_cloud(test_point_cloud_path)
    # base_link_T_camera = base_link_T_arml_base @ cam_arml_extrinsics
    # test_point_cloud.transform(base_link_T_camera)
    # save_point_cloud(test_point_cloud)

    # dexsim_point_cloud = _env.load_point_cloud("/home/young/workspace/dexe_robot_client/examples/output/point_cloud.ply")
    # embed()
    test_octomap()




    
