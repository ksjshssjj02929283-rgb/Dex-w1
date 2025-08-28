import os
import numpy as np
from dexe_robot_client.end_effector_client import EndEffectorType 
from dexe_robot_client.common import W1Version, W1ArmType
from dexe_robot_client.w1 import W1Manager
from dexe_robot_client.w1.path_planner.utils.robot_type import *
from dexe_robot_client.w1.path_planner.utils.mapping_index import *
from dexe_robot_client.w1.utils.unit_conversion import *
from IPython import embed

robot_type = RobotType.W1V02_1
ee_type = EndEffectorType.BRAINCO_HAND
urdf_path = get_robot_urdf(robot_type)


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

    metal_material_handle = _env.load_material(material_fpath + "/Metal022", "Metal022")

    table_handle = _env.load_actor("/home/young/Model/coffee_demo/table_coffee_demo_fixed.obj")
    table_xpos = np.array([[1,0,0,0.5],
                           [0,1,0,0.0],
                           [0,0,1,0.0],
                           [0,0,0,1]])
    table_handle.set_world_pose(table_xpos)

    cup_shelf_handle = _env.load_actor("/home/young/Model/coffee_demo/cup_shelf.obj")
    cup_shelf_xpos = np.array([[-0.707,0.707,0,0.55],
                                [-0.707,-0.707,0,0.4],
                                [0,0,1,0.75],
                                [0,0,0,1]])
    cup_shelf_handle.set_world_pose(cup_shelf_xpos)
    cup_shelf_handle.set_material(metal_material_handle.get_inst()) 

    coffee_machine_handle = _env.load_actor("/home/young/Model/coffee_demo/coffee_machine.obj")
    coffee_machine_xpos = np.array([[-0.707,-0.707,0,0.55],
                                    [0.707,-0.707,0,-0.4],
                                    [0,0,1,0.75],
                                    [0,0,0,1]])
    coffee_machine_handle.set_world_pose(coffee_machine_xpos)
    coffee_machine_handle.set_material(metal_material_handle.get_inst())



    robot_urdf_path = get_robot_urdf(robot_type)
    _articulation = _env.load_urdf(robot_urdf_path)
    robot_pose = np.eye(4, dtype=float)
    _articulation.set_world_pose(robot_pose)
    _articulation.set_articulation_flag(dexsim.types.ArticulationFlag.DISABLE_SELF_COLLISION, True)
    _articulation.set_drive(drive_type=dexsim.types.DriveType.FORCE)

    if ee_type == EndEffectorType.BRAINCO_HAND:
        left_hand_handle = _env.load_actor(get_ee_full_path(robot_type, EndEffectorType.BRAINCO_HAND)[0])
        right_hand_handle = _env.load_actor(get_ee_full_path(robot_type, EndEffectorType.BRAINCO_HAND)[1])
        _articulation.attach_node(left_hand_handle.node, "left_ee", np.eye(4, dtype=float))
        _articulation.attach_node(right_hand_handle.node, "right_ee", np.eye(4, dtype=float))
    elif ee_type == EndEffectorType.PGC_GRIPPER:
        left_hand_handle = _env.load_actor(get_ee_full_path(robot_type, EndEffectorType.PGC_GRIPPER)[0])
        right_hand_handle = _env.load_actor(get_ee_full_path(robot_type, EndEffectorType.PGC_GRIPPER)[1])
        _articulation.attach_node(left_hand_handle.node, "left_ee", np.eye(4, dtype=float))
        _articulation.attach_node(right_hand_handle.node, "right_ee", np.eye(4, dtype=float))

    real2sim_idx = get_real2sim_idx(_articulation)
    sim2real_idx = get_sim2real_idx(_articulation)
    torso_real_idx, head_real_idx, arml_real_idx, armr_real_idx = get_real_idx()
    torso_pin_idx, head_pin_idx, arml_pin_idx, armr_pin_idx = get_pin_idx(robot_urdf_path)

    
    init_real_qpos = np.deg2rad([45, -90, 45, 0, 
                            0, 0,
                            5, -45, -5, -60, 0, -15, 0,
                            -5, 45, 5, 60, 0, 15, 0])
    _articulation.set_current_qpos(init_real_qpos[real2sim_idx])

    # init w1 manager
    w1_manager = W1Manager(is_sim=True,
                           arm_type=W1ArmType.ANTHROPOMORPHIC,
                           w1_version=W1Version.V2,
                           end_effector_type=ee_type,
                           use_rlia=False)
    # if not w1_manager.connect():
    #     raise RuntimeError("Failed to connect to W1 robot.")
    
    w1_manager.set_tcp("left_arm", np.eye(4))
    w1_manager.set_tcp("right_arm", np.array([[-1,0,0,0],
                                              [0,-1,0,0],
                                              [0,0,1,0],
                                              [0,0,0,1]]))
    

    arml_kine = w1_manager._get_arm_kine("left_arm")
    armr_kine = w1_manager._get_arm_kine("right_arm")

    current_real_qpos = _articulation.get_current_qpos()[sim2real_idx]
    cur_torso_qpos = current_real_qpos[torso_real_idx[0]: torso_real_idx[1]+1]
    cur_head_qpos = current_real_qpos[head_real_idx[0]: head_real_idx[1]+1]
    cur_arml_qpos = current_real_qpos[arml_real_idx[0]: arml_real_idx[1]+1]
    cur_armr_qpos = current_real_qpos[armr_real_idx[0]: armr_real_idx[1]+1]
    
    # arml_target_pose = np.array([-126.29, 300.57, 326.69, 28.97, 67.64, 129.92])
    # armr_target_pose = np.array([266.044, 71.307, 254.868, -161.749, 60.184, -56.87])
    # arml_target_xpos = user_pose_to_xpos(arml_target_pose[:3], arml_target_pose[3:])
    # armr_target_xpos = user_pose_to_xpos(armr_target_pose[:3], armr_target_pose[3:])

    # arml_ik_qpos = arml_kine.get_nearst_ik(xpos=arml_target_xpos, qpos_seed=cur_arml_qpos)
    # armr_ik_qpos = armr_kine.get_nearst_ik(xpos=armr_target_xpos, qpos_seed=cur_armr_qpos)
    # print(f"arml_ik_qpos: {arml_ik_qpos}")
    # print(f"armr_ik_qpos: {armr_ik_qpos}")

    # target_real_qpos = np.concatenate((cur_torso_qpos, cur_head_qpos, arml_ik_qpos[1], armr_ik_qpos[1]))
    # _articulation.set_current_qpos(target_real_qpos[real2sim_idx])

    arml_target_pose_list = [
                [-176.73, 190.87, 72.72, -174.17, 16.13, -98.1],
                [-220.179, 335.021, 251.264, 51.501, 69.117, -175.001],
                [-270.183, 335.024, 251.261, 51.501, 69.117, -175.001],
                [-280.18, 330.021, 251.256, 51.501, 69.117, -175.001],
                [-287.18, 325.025, 251.261, 51.501, 69.117, -175.001],
                [-270.183, 335.024, 251.261, 51.501, 69.117, -175.001],
                [-220.184, 325.016, 251.265, 51.501, 69.117, -175.001],
                [-142.71, 320.29, 119.75, -160.73, 65.87, -34.3],
                [-5.141, 375.012, 60.233, -140.505, 69.868, -40.018],
                [-33.142, 375.016, 60.229, -140.505, 69.868, -40.018],
                [30.136, 345.013, 130.206, -140.512, 79.865, -40.026],
                [-236.141, 221.45, 253.569, -63.476, -3.253, -10.561],
                [-206.744, 294.044, 280.958, -59.643, -3.117, -13.263],
                [-146.718, 280.038, 273.96, -59.639, -3.122, -10.259],
                [-109.164, 281.189, 273.194, -59.639, -3.122, -10.259],
                [-206.744, 294.044, 280.958, -59.643, -3.117, -13.263],
                [-206.744, 294.044, 280.958, -59.643, -3.117, -13.263],
                [-206.744, 294.044, 280.958, -59.643, -3.117, -13.263],
                [-206.744, 294.044, 280.958, -59.643, -3.117, -13.263],
                [-206.744, 294.044, 280.958, -59.643, -3.117, -13.263],
                [-290.787, 260.284, 130.106, -72.835, -3.188, -6.71],
                [-289.127, 275.251, 62.114, -86.825, -3.475, -6.537],
                [-289.127, 275.251, 62.114, -86.825, -3.475, -6.537],
                [-289.127, 275.251, 62.114, -86.825, -3.475, -6.537],
                [-247.656, 220.622, 60.016, -72.456, -0.782, -8.705],
                [-277.674, 420.605, 121.001, -72.456, -0.782, -8.705],
                [-277.664, 260.603, 120.999, -72.456, -0.782, -8.705]

    ]
    armr_target_pose_list = []

    for i in range(len(arml_target_pose_list)):
        arml_target_pose = np.array(arml_target_pose_list[i])
        arml_target_xpos = user_pose_to_xpos(arml_target_pose[:3], arml_target_pose[3:])
        arml_ik_qpos = arml_kine.get_nearst_ik(xpos=arml_target_xpos, qpos_seed=cur_arml_qpos)
        if arml_ik_qpos[0]:
            print(f"arml_ik_qpos: {arml_ik_qpos[1]}")
            target_real_qpos = np.concatenate((cur_torso_qpos, cur_head_qpos, arml_ik_qpos[1], cur_armr_qpos))
            _articulation.set_current_qpos(target_real_qpos[real2sim_idx])
            cur_arml_qpos = arml_ik_qpos[1]
            input("Press Enter to continue...")
        else:
            print(f"Cannot find IK for target pose: {arml_target_pose}")
            input("Press Enter to continue...")


    embed()


    