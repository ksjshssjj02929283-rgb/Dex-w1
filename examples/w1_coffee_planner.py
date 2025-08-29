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
    cup_shelf_xpos = np.array([[-0.707,0.707,0,0.53],
                                [-0.707,-0.707,0,0.37],
                                [0,0,1,0.75],
                                [0,0,0,1]])
    cup_shelf_handle.set_world_pose(cup_shelf_xpos)
    cup_shelf_handle.set_material(metal_material_handle.get_inst()) 

    coffee_machine_handle = _env.load_actor("/home/young/Model/coffee_demo/coffee_machine.obj")
    coffee_machine_xpos = np.array([[-0.707,-0.707,0,0.49],
                                    [0.707,-0.707,0,-0.33],
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
        left_ee_path = "/home/young/dexe_robot_client_data/assets/DexforceW1V02_v1/Collision/Brainco/brainco_left_open_anthropomorphic.obj"
        right_ee_path = "/home/young/dexe_robot_client_data/assets/DexforceW1V02_v1/Collision/Brainco/brainco_right_open_anthropomorphic.obj"
        left_hand_handle = _env.load_actor(left_ee_path)
        right_hand_handle = _env.load_actor(right_ee_path)
        # left_hand_handle = _env.load_actor(get_ee_full_path(robot_type, EndEffectorType.BRAINCO_HAND)[0])
        # right_hand_handle = _env.load_actor(get_ee_full_path(robot_type, EndEffectorType.BRAINCO_HAND)[1])
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

    
    # init_real_qpos = np.deg2rad([45, -90, 45, 0, 
    #                         0, 0,
    #                         5, -45, -5, -60, 0, -15, 0,
    #                         -5, 45, 5, 60, 0, 15, 0])
    torso_init_qpos = np.deg2rad([45, -90, 45, 2])
    head_init_qpos = np.deg2rad([24, -32])
    arml_init_qpos = np.array([0.218, -0.718, 0.563, -2.083, 1.125, -0.501, 0.393])
    armr_iniit_qpos = np.array([-0.51, 0.851, -0.291, 1.821, 1.187, -0.104, 0.946])
    init_real_qpos = np.concatenate((torso_init_qpos, head_init_qpos, arml_init_qpos, armr_iniit_qpos))
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
    
    task1_torso_qpos = np.deg2rad([31, -70, 45, 2])
    task1_head_qpos = np.deg2rad([-5, -25])
    task1_arml_qpos = np.array([-0.113, -0.604, 0.462, -1.539, 0.646, 0.269, -0.235])
    task1_armr_qpos = np.array([-0.69, 0.69, -0.25, 1.43, 1.91, 0.02, 1.27])
    task1_real_qpos = np.concatenate((task1_torso_qpos, task1_head_qpos, task1_arml_qpos, task1_armr_qpos))

    task2_torso_qpos = np.deg2rad([31, -70, 45, 2])
    task2_head_qpos = np.deg2rad([-5, -25])
    task2_arml_qpos = np.array([-0.477, -0.703, 0.552, -1.91, -0.099, 0.249, -0.611])
    task2_armr_qpos = np.array([-0.69, 0.69, -0.25, 1.43, 1.91, 0.02, 1.27])
    task2_real_qpos = np.concatenate((task2_torso_qpos, task2_head_qpos, task2_arml_qpos, task2_armr_qpos))

    task3_torso_qpos = np.deg2rad([31, -70, 45, 2])
    task3_head_qpos = np.deg2rad([0, -15])
    task3_arml_qpos = np.array([-1.0, -1.02, 0.5, -2.04, 0.42, 0.72, -0.54])
    task3_armr_qpos = np.array([-0.66, 0.64, 0.44, 2.03, 0.92, 0.48, 0.32])
    task3_real_qpos = np.concatenate((task3_torso_qpos, task3_head_qpos, task3_arml_qpos, task3_armr_qpos))

    task4_torso_qpos = np.deg2rad([38, -80, 50, -10])
    task4_head_qpos = np.deg2rad([10, -20])
    task4_arml_qpos = np.array([0.51, -0.64, 0.09, -1.7, -0.78, -0.41, 0.69])
    task4_armr_qpos = np.array([0.66, 0.67, -0.03, 1.82, 0.16, 0.19, 0.24])
    task4_real_qpos = np.concatenate((task4_torso_qpos, task4_head_qpos, task4_arml_qpos, task4_armr_qpos))


    task5_torso_qpos = np.deg2rad([36, -80, 50, -45])
    task5_head_qpos = np.deg2rad([0, -15])
    task5_arml_qpos = np.array([0.15, -0.72, 0.58, -1.66, -0.65, -0.33, 0.58])
    task5_armr_qpos = np.array([0.64, 1.05, -0.77, 1.8, -0.78, -0.02, 0.07])
    task5_real_qpos = np.concatenate((task5_torso_qpos, task5_head_qpos, task5_arml_qpos, task5_armr_qpos))

    task6_torso_qpos = np.deg2rad([36, -80, 50, -45])
    task6_head_qpos = np.deg2rad([0, -15])
    task6_arml_qpos = np.array([0.213, -1.308, 0.244, -1.685, -0.171, -0.308, 0.579])
    task6_armr_qpos = np.array([-0.651, 0.546, 0.356, 1.899, 0.524, 0.058, 0.622])
    task6_real_qpos = np.concatenate((task6_torso_qpos, task6_head_qpos, task6_arml_qpos, task6_armr_qpos))

    for target_qpos in [task1_real_qpos, task2_real_qpos, task3_real_qpos, task4_real_qpos, task5_real_qpos, task6_real_qpos]:
        _articulation.set_current_qpos(target_qpos[real2sim_idx])
        input("Press Enter to continue...")



    # for i in range(len(arml_target_pose_list)):
    #     arml_target_pose = np.array(arml_target_pose_list[i])
    #     arml_target_xpos = user_pose_to_xpos(arml_target_pose[:3], arml_target_pose[3:])
    #     arml_ik_qpos = arml_kine.get_nearst_ik(xpos=arml_target_xpos, qpos_seed=cur_arml_qpos)
    #     if arml_ik_qpos[0]:
    #         print(f"arml_ik_qpos: {arml_ik_qpos[1]}")
    #         target_real_qpos = np.concatenate((cur_torso_qpos, cur_head_qpos, arml_ik_qpos[1], cur_armr_qpos))
    #         _articulation.set_current_qpos(target_real_qpos[real2sim_idx])
    #         cur_arml_qpos = arml_ik_qpos[1]
    #         input("Press Enter to continue...")
    #     else:
    #         print(f"Cannot find IK for target pose: {arml_target_pose}")
    #         input("Press Enter to continue...")

    # for i in range(len(armr_target_pose_list)):
    #     armr_target_pose = np.array(armr_target_pose_list[i])
    #     armr_target_xpos = user_pose_to_xpos(armr_target_pose[:3], armr_target_pose[3:])
    #     armr_ik_qpos = armr_kine.get_nearst_ik(xpos=armr_target_xpos, qpos_seed=cur_armr_qpos)
    #     if armr_ik_qpos[0]:
    #         print(f"armr_ik_qpos: {armr_ik_qpos[1]}")
    #         target_real_qpos = np.concatenate((cur_torso_qpos, cur_head_qpos, cur_arml_qpos, armr_ik_qpos[1]))
    #         _articulation.set_current_qpos(target_real_qpos[real2sim_idx])
    #         cur_armr_qpos = armr_ik_qpos[1]
    #         input("Press Enter to continue...")
    #     else:
    #         print(f"Cannot find IK for target pose: {armr_target_pose}")
    #         input("Press Enter to continue...")
    embed()


    