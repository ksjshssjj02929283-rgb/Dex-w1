import os
from dexe_robot_client.w1.path_planner.utils.robot_type import *
from dexe_robot_client.w1.path_planner.single_planner import SinglePlanner
from dexe_robot_client.w1.path_planner.fcl_checker import FCLChecker
from dexe_robot_client.w1.path_planner.utils.mapping_index import *
import numpy as np
import time
import pinocchio

from copy import deepcopy
from dexe_robot_client.end_effector_client import EndEffectorType 
from dexe_robot_client.common import download_and_unzip_file, URL_PREFIX, ASSETS_DIR
from IPython import embed

ROBOT_TYPE = RobotType.W1V02_1
ROBOT_EE_TYPE = None
TABLE_CENTER = np.array([0.8, 0.0, 1.1])
TABLE_SIZE = np.array([1.0, 1.0, 0.03])
ENV_JSON_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "model", "coffee_env.json")

def _get_Kinematics(real_q, pin_model, pin_data):
    if real_q is None:
        raise ValueError("Pinocchio Kinematics requires a valid real_q input.")
    pin_q = real_q[real2pin_idx]
    pinocchio.forwardKinematics(pin_model, pin_data, pin_q)
    pinocchio.updateFramePlacements(pin_model, pin_data)
    transform_chain = [oMi.homogeneous for oMi in pin_data.oMi][1:]
    left_transform_chain = transform_chain[pin_arml_idx[0]:pin_arml_idx[1]+1]  # left arm links
    right_transform_chain = transform_chain[pin_armr_idx[0]:pin_armr_idx[1]+1]  # right arm links
    if ROBOT_EE_TYPE is not None:
        armlee_ID = pin_model.getFrameId("LEFT_EE")  
        armree_ID = pin_model.getFrameId("RIGHT_EE")  
        lee_trans = pin_data.oMf[armlee_ID].homogeneous
        ree_trans = pin_data.oMf[armree_ID].homogeneous
        left_transform_chain.append(lee_trans)
        right_transform_chain.append(ree_trans)
    
    body_transform_chain = transform_chain[pin_torso_idx[0]:pin_torso_idx[1]+1]
    head_transform_chain = transform_chain[pin_head_idx[0]:pin_head_idx[1]+1]

    return body_transform_chain, head_transform_chain, left_transform_chain, right_transform_chain


def test_fcl_distance(articulation, pin_model, pin_data):
    fcl_checker = FCLChecker(ROBOT_TYPE, ROBOT_EE_TYPE)
    


    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # The following is used to test brainco hand visualize
    fcl_checker.enable_visualize()
    fcl_checker.load_env_json(ENV_JSON_PATH)
    cur_sim_q = articulation.get_current_qpos()
    real_q = cur_sim_q[sim2real_idx]
    body_tf, head_tf, arml_tf, armr_tf = _get_Kinematics(real_q, pin_model, pin_data)
    fcl_checker.update_vis_transform(body_tf+head_tf+arml_tf+armr_tf)

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    ## The following is used to test reload environment
    # print("First loading")
    # print(f"grasp_objects {fcl_checker.grasp_objects}  grasp_objects_manager {fcl_checker.grasp_objects_manager.getObjects()}")
    # print(f"grasp_objects_dict {fcl_checker.grasp_objects_dict}")
    # fcl_checker.set_compute_distance(True)

    # fcl_checker.attach_object_arml("box1", flange_T_object=np.eye(4, dtype=np.float64))
    # fcl_checker.load_env_json("/home/young/workspace/dexe_robot_client/src/dexe_robot_client/w1/path_planner/model/test_reload_env.json")
    # print("After reloading with left arm grasp object")
    # print(f"grasp_objects {fcl_checker.grasp_objects}  grasp_objects_manager {fcl_checker.grasp_objects_manager.getObjects()}")
    # print(f"grasp_objects_dict {fcl_checker.grasp_objects_dict}")
    
    # fcl_checker.detach_object_arml("box1", arml7_transform=np.eye(4, dtype=np.float64))
    # print("After detach left arm grasp object")
    # print(f"grasp_objects {fcl_checker.grasp_objects}  grasp_objects_manager {fcl_checker.grasp_objects_manager.getObjects()}")
    # print(f"grasp_objects_dict {fcl_checker.grasp_objects_dict}")

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


    

if __name__ == "__main__":
    import dexsim
    download_and_unzip_file("http://192.168.3.120/CoreEngine/Data/embodychain_data/DexforceW1V02_v1.zip")

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
    
    """ create obstacle """
    # table_handle = _env.create_cube()
    # table_xpos = np.array([[1,0,0,TABLE_CENTER[0]],
    #                        [0,1,0,TABLE_CENTER[1]],
    #                        [0,0,1,TABLE_CENTER[2]],
    #                        [0,0,0,1]])
    # table_handle.set_world_pose(table_xpos)
    # table_handle.set_body_scale(TABLE_SIZE[0], TABLE_SIZE[1], TABLE_SIZE[2])

    # grasp_cube_handle = _env.create_cube()
    # grasp_cube_xpos = np.array([[1,0,0,0.45],
    #                             [0,1,0,0.15], 
    #                             [0,0,1,1.165],
    #                             [0,0,0,1]])
    # grasp_cube_handle.set_world_pose(grasp_cube_xpos)
    # grasp_cube_handle.set_body_scale(0.1, 0.1, 0.08)
    # grasp_cube_handle.set_material(material_handle.get_inst())

    table_handle = _env.load_actor("/home/young/Downloads/table_coffee_demo_fixed.obj")
    table_xpos = np.array([[1,0,0,0.5],
                           [0,1,0,0.0],
                           [0,0,1,0.0],
                           [0,0,0,1]])
    table_handle.set_world_pose(table_xpos)

    shelf_handle = _env.create_cube()
    shelf_xpos = np.array([[0.707,-0.707,0,0.55],
                           [0.707,0.707,0,0.4],
                           [0,0,1,1.0],
                           [0,0,0,1]])
    shelf_handle.set_world_pose(shelf_xpos)
    shelf_handle.set_body_scale(0.2, 0.1, 0.5)
    shelf_handle.set_material(material_handle.get_inst())

    

    coffee_machine_handle = _env.create_cube()
    coffee_machine_xpos = np.array([[0.707,0.707,0,0.55],
                                    [-0.707,0.707,0,-0.4],
                                    [0,0,1,0.8675], 
                                    [0,0,0,1]])
    coffee_machine_handle.set_world_pose(coffee_machine_xpos)
    coffee_machine_handle.set_body_scale(0.37, 0.2, 0.235)
    coffee_machine_handle.set_material(material_handle.get_inst())



    robot_urdf_path = get_robot_urdf(ROBOT_TYPE)
    _articulation = _env.load_urdf(robot_urdf_path)
    robot_pose = np.eye(4, dtype=float)
    _articulation.set_world_pose(robot_pose)
    _articulation.set_articulation_flag(dexsim.types.ArticulationFlag.DISABLE_SELF_COLLISION, True)
    _articulation.set_drive(drive_type=dexsim.types.DriveType.FORCE)

    pin_model = pinocchio.buildModelFromUrdf(robot_urdf_path)
    pin_data = pin_model.createData()

    sim2real_idx = get_sim2real_idx(articulation=_articulation)
    # print(f"sim2real_idx: {sim2real_idx}")
    real2sim_dix = get_real2sim_idx(articulation=_articulation)
    # print(f"real2sim_idx: {real2sim_dix}")

    pin2real_idx = get_pin2real_idx(robot_urdf_path)
    # print(f"pin2real_idx: {pin2real_idx}")
    real2pin_idx = get_real2pin_idx(robot_urdf_path)
    # print(f"real2pin_idx: {real2pin_idx}")
    pin_torso_idx, pin_head_idx, pin_arml_idx, pin_armr_idx = get_pin_idx(robot_urdf_path)
    print(f"torso_idx: {pin_torso_idx}, head_idx: {pin_head_idx}, arml_idx: {pin_arml_idx}, armr_idx: {pin_armr_idx}")
    real_torso_idx, real_head_idx, real_arml_idx, real_armr_idx = get_real_idx()
    # print(f"real_torso_idx: {real_torso_idx}, real_head_idx: {real_head_idx}, real_arml_idx: {real_arml_idx}, real_armr_idx: {real_armr_idx}")

    init_head_qpos = np.array([0.0, 0.0])
    init_body_qpos = np.array([0.0, 0.0, 0.0, 0.0])
    init_arml_qpos = np.array([-np.pi / 2, -np.pi / 4, np.pi / 2, -np.pi / 4, -np.pi / 2, np.pi / 5, 0.0])
    init_armr_qpos = -np.array([-np.pi / 2, -np.pi / 4, np.pi / 2, -np.pi / 4, -np.pi / 2, np.pi / 5, 0.0])
    q = np.concatenate([init_body_qpos, init_head_qpos, init_arml_qpos, init_armr_qpos])
    _articulation.set_current_qpos(q[real2sim_dix])
    q = np.array([0.7854, -1.5708,  0.7853,  0.    ,  0.4999,  0.2999, -0.7516,
        -0.3681,  1.5708, -2.1179,  0.915 , -0.1155,  0.6287, -0.4864,
         0.8013,  0.116 ,  1.8058, -1.3278,  0.5119, -0.6207])
    _articulation.set_current_qpos(q[real2sim_dix])
    
    embed()
    # test_fcl_distance(_articulation, pin_model, pin_data)



