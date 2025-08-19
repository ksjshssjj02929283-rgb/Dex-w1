import os
from dexe_robot_client.w1.path_planner.utils.robot_type import *
from dexe_robot_client.w1.path_planner.single_planner import SinglePlanner
from dexe_robot_client.w1.path_planner.fcl_checker import FCLChecker
from dexe_robot_client.w1.path_planner.ompl_plan import *
from dexe_robot_client.w1.path_planner.utils.mapping_index import get_sim2real_idx, get_real2sim_idx, get_pin2real_idx, get_real2pin_idx
import numpy as np
import time

from copy import deepcopy
from dexe_robot_client.end_effector_client import EndEffectorType 
from dexe_robot_client.common import download_and_unzip_file, URL_PREFIX, ASSETS_DIR
from IPython import embed

# robot configuration
SINGLE_ARM_DIM = 7
DUAL_ARM_CEN_DIM = 14
LARM_BOUND = np.array([
        [-2.9670597283903604, -2.0943951023931953, -2.9670597283903604, -2.356194490192345, -2.9670597283903604, -1.5707963267948966, -2.9670597283903604],
        [2.9670597283903604, 1.5707963267948966, 2.9670597283903604, 1.5707963267948966, 2.96705972839036041, 1.5707963267948966, 2.9670597283903604]
    ])
RARM_BOUND = np.array([
        [-2.9670597283903604, -1.5707963267948966, -2.9670597283903604, -1.5707963267948966, -2.9670597283903604, -1.5707963267948966, -2.9670597283903604],
        [2.9670597283903604, 2.0943951023931953, 2.9670597283903604, 2.356194490192345, 2.9670597283903604, 1.5707963267948966, 2.9670597283903604]
    ])

ROBOT_TYPE = RobotType.W1V02_2
ROBOT_EE_TYPE = None

REAL2SIM_IDX = [
            0,      # Ankle    -> [0  | Trunk forward/backward tilt]
            1,      # Knee     -> [1  | Trunk forward/backward tilt]
            2,      # Buttock  -> [2  | Trunk forward/backward tilt]
            3,      # Loin     -> [3  | Trunk left/right rotation]
            6,      # Left J1  -> [4  | Left shoulder first joint - Shoulder forward/back]
            4,      # Neck     -> [5  | Neck left/right rotation]
            13,     # Right J1 -> [6  | Right shoulder first joint - Shoulder forward/back]
            7,      # Left J2  -> [7  | Left shoulder abduction/adduction]
            5,      # Head     -> [8  | Head pitch up/down]
            14,     # Right J2 -> [9  | Right shoulder abduction/adduction]
            8,      # Left J3  -> [10 | Left shoulder internal/external rotation]
            15,     # Right J3 -> [11 | Right shoulder internal/external rotation]
            9,      # Left J4  -> [12 | Left elbow flexion/extension]
            16,     # Right J4 -> [13 | Right elbow flexion/extension]
            10,     # Left J5  -> [14 | Left forearm pronation/supination]
            17,     # Right J5 -> [15 | Right forearm pronation/supination]
            11,     # Left J6  -> [16 | Left wrist flexion/extension]
            18,     # Right J6 -> [17 | Right wrist flexion/extension]
            12,     # Left J7  -> [18 | Left wrist radial/ulnar deviation]
            19,     # Right J7 -> [19 | Right wrist radial/ulnar deviation]
        ]
SIM2REAL_IDX = [
            0,  # ANKLE 
            1,  # KNEE
            2,  # BUTTOCK
            3,  # WAIST
            5,  # NECK1
            8,  # NECK2
            4,  # LEFT_J1
            7,  # LEFT_J2
            10, # LEFT_J3
            12, # LEFT_J4
            14, # LEFT_J5
            16, # LEFT_J6
            18, # LEFT_J7
            6,  # RIGHT_J1
            9,  # RIGHT_J2
            11, # RIGHT_J3
            13, # RIGHT_J4
            15, # RIGHT_J5
            17, # RIGHT_J6
            19  # RIGHT_J7
        ]
# environment configuration
TABLE_CENTER = np.array([0.8, 0.0, 1.1])
TABLE_SIZE = np.array([1.0, 1.0, 0.03])
ENV_JSON_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "model", "env.json")

def _get_Kinematics(cur_body, cur_arml, cur_armr, cur_head, pin_model, pin_data):
    if cur_body is None or cur_head is None:
            raise ValueError("body_qpos and head_qpos must be set before calling _get_Kinematics")
    q = np.concatenate((cur_body, cur_arml, cur_head, cur_armr))
    pinocchio.forwardKinematics(pin_model, pin_data, q)
    pinocchio.updateFramePlacements(pin_model, pin_data)
    transform_chain = [oMi.homogeneous for oMi in pin_data.oMi]
    left_transform_chain = transform_chain[5:12]  # left arm links
    right_transform_chain = transform_chain[14:21]  # right arm links
    if ROBOT_EE_TYPE is not None:
        armlee_ID = pin_model.getFrameId("armlee")  
        armree_ID = pin_model.getFrameId("armree")  
        lee_trans = pin_data.oMf[armlee_ID].homogeneous
        ree_trans = pin_data.oMf[armree_ID].homogeneous
        left_transform_chain.append(lee_trans)
        right_transform_chain.append(ree_trans)
    
    body_transform_chain = transform_chain[1:5]
    head_transform_chain = transform_chain[12:14]

    return body_transform_chain, left_transform_chain, right_transform_chain, head_transform_chain


def _vis_traj(arml_traj, armr_traj, articulation):
    current_qpos = articulation.get_current_qpos()[SIM2REAL_IDX]
    cur_torso, cur_head, cur_arml, cur_armr = current_qpos[:4], current_qpos[4:6], current_qpos[6:13], current_qpos[13:]
    
    # 这个测试中双臂轨迹是同步的
    if arml_traj is None and armr_traj is None:
        print("No trajectory to move")
        return
    if arml_traj is not None and armr_traj is not None:
        for i in range(len(arml_traj)):
            qpos = np.concatenate([cur_torso, cur_head, arml_traj[i], armr_traj[i]])
            articulation.set_current_qpos(qpos[REAL2SIM_IDX])
            time.sleep(0.002)
    elif arml_traj is not None and armr_traj is None:
        for i in range(len(arml_traj)):
            qpos = np.concatenate([cur_torso, cur_head, arml_traj[i], cur_armr])
            articulation.set_current_qpos(qpos[REAL2SIM_IDX])
            time.sleep(0.002)
    elif arml_traj is None and armr_traj is not None:
        for i in range(len(armr_traj)):
            qpos = np.concatenate([cur_torso, cur_head, cur_arml, armr_traj[i]])
            articulation.set_current_qpos(qpos[REAL2SIM_IDX])
            time.sleep(0.002)

def test_siso_plan(articulation, pin_model, pin_data):
    SinglePlanner.set_arms_cfg(SINGLE_ARM_DIM, SINGLE_ARM_DIM, LARM_BOUND, RARM_BOUND)
    SinglePlanner.set_robot_cfg(ROBOT_TYPE, ROBOT_EE_TYPE)
    SinglePlanner.set_env_json(ENV_JSON_PATH)
    siso_planner = SinglePlanner()

    current_qpos = articulation.get_current_qpos()[SIM2REAL_IDX]
    cur_torso, cur_head, cur_arml, cur_armr = current_qpos[:4], current_qpos[4:6], current_qpos[6:13], current_qpos[13:]
    siso_planner.set_head_torso(cur_head, cur_torso)

    arml_goal1 = np.array([-0.79258, -1.03953,  0.63524, -0.97332,  0.76595, -1.13355, 0.26492])
    armr_goal1 = -arml_goal1
    siso_planner.set_start_goal(cur_arml, cur_armr, arml_goal1, armr_goal1)
    arml_traj1, armr_traj1, planning_time = siso_planner.plan()

    _vis_traj(arml_traj1, armr_traj1, articulation)

    current_qpos = articulation.get_current_qpos()[SIM2REAL_IDX]
    cur_torso, cur_head, cur_arml, cur_armr = current_qpos[:4], current_qpos[4:6], current_qpos[6:13], current_qpos[13:]
    siso_planner.set_head_torso(cur_head, cur_torso)
    body_tf, arml_tf, armr_tf, head_tf = _get_Kinematics(cur_torso, cur_arml, cur_armr, cur_head, pin_model, pin_data)
    w_T_o = np.array([[1, 0, 0, 0.45], [0, 1, 0, 0.15], [0, 0, 1, 1.165], [0, 0, 0, 1]])
    w_T_f = arml_tf[-1]
    f_T_o = np.linalg.inv(w_T_f) @ w_T_o

    siso_planner.attach_object(obj_name="box1", which_arm="left", flange_T_object=f_T_o)
    arml_goal2 = np.array([-0.6486993, -1.2057196,  0.5688665, -0.013276 , -0.3541906, -0.9850419,  0.2607839])
    armr_goal2 = cur_armr
    siso_planner.set_start_goal(cur_arml, cur_armr, arml_goal2, armr_goal2)
    arml_traj2, armr_traj2, planning_time = siso_planner.plan()
    articulation.attach_node(grasp_cube_handle.node, "left_j7", f_T_o)
    _vis_traj(arml_traj2, armr_traj2, articulation)

    current_qpos = articulation.get_current_qpos()[SIM2REAL_IDX]
    cur_torso, cur_head, cur_arml, cur_armr = current_qpos[:4], current_qpos[4:6], current_qpos[6:13], current_qpos[13:]
    siso_planner.set_head_torso(cur_head, cur_torso)
    body_tf, arml_tf, armr_tf, head_tf = _get_Kinematics(cur_torso, cur_arml, cur_armr, cur_head, pin_model, pin_data)
    siso_planner.detach_object(obj_name="box1", which_arm="left", arm7_transform=arml_tf[-1])
    arml_goal3 = np.array([-0.79258, -1.03953,  0.63524, -0.97332,  0.76595, -1.13355, 0.2649])
    armr_goal3 = cur_armr
    siso_planner.set_start_goal(cur_arml, cur_armr, arml_goal3, armr_goal3)
    arml_traj3, armr_traj3, planning_time = siso_planner.plan()
    # vis detach
    arena_root_node = _env.get_root_node()
    grasp_cube_handle.node.attach_node(arena_root_node)
    _vis_traj(arml_traj3, armr_traj3, articulation)

    embed()
    return None


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
    table_handle = _env.create_cube()
    table_xpos = np.array([[1,0,0,TABLE_CENTER[0]],
                           [0,1,0,TABLE_CENTER[1]],
                           [0,0,1,TABLE_CENTER[2]],
                           [0,0,0,1]])
    table_handle.set_world_pose(table_xpos)
    table_handle.set_body_scale(TABLE_SIZE[0], TABLE_SIZE[1], TABLE_SIZE[2])

    grasp_cube_handle = _env.create_cube()
    grasp_cube_xpos = np.array([[1,0,0,0.45],
                                [0,1,0,0.15], 
                                [0,0,1,1.165],
                                [0,0,0,1]])
    grasp_cube_handle.set_world_pose(grasp_cube_xpos)
    grasp_cube_handle.set_body_scale(0.1, 0.1, 0.08)
    grasp_cube_handle.set_material(material_handle.get_inst())

    robot_urdf_path = get_robot_urdf(ROBOT_TYPE)
    _articulation = _env.load_urdf(robot_urdf_path)
    robot_pose = np.eye(4, dtype=float)
    _articulation.set_world_pose(robot_pose)
    _articulation.set_articulation_flag(dexsim.types.ArticulationFlag.DISABLE_SELF_COLLISION, True)
    _articulation.set_drive(drive_type=dexsim.types.DriveType.FORCE)

    init_head_qpos = np.array([0.0, 0.0])
    init_body_qpos = np.array([0.0, 0.0, 0.0, 0.0])
    init_arml_qpos = np.array([-np.pi / 2, -np.pi / 4, np.pi / 2, -np.pi / 4, -np.pi / 2, np.pi / 5, 0.0])
    init_armr_qpos = -np.array([-np.pi / 2, -np.pi / 4, np.pi / 2, -np.pi / 4, -np.pi / 2, np.pi / 5, 0.0])
    q = np.concatenate([init_body_qpos, init_head_qpos, init_arml_qpos, init_armr_qpos])
    _articulation.set_current_qpos(q[REAL2SIM_IDX])

    pin_model = pinocchio.buildModelFromUrdf(robot_urdf_path)
    pin_data = pin_model.createData()


    sim2real_idx = get_sim2real_idx(articulation=_articulation)
    print(f"sim2real_idx: {sim2real_idx}")
    real2sim_dix = get_real2sim_idx(articulation=_articulation)
    print(f"real2sim_idx: {real2sim_dix}")

    pin2real_idx = get_pin2real_idx(pin_model=pin_model)
    print(f"pin2real_idx: {pin2real_idx}")
    real2pin_idx = get_real2pin_idx(pin_model=pin_model)
    print(f"real2pin_idx: {real2pin_idx}")
    # embed()
    test_siso_plan(_articulation, pin_model, pin_data)
