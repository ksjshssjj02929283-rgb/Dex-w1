import os
from dexe_robot_client.w1.path_planner.ompl_plan import *
from dexe_robot_client.w1.path_planner.fcl_checker import *
from dexe_robot_client.w1.path_planner.robot_type import RobotType, RobotURDF, RobotCollision, get_robot_urdf, get_robot_mesh_path
import numpy as np
import time

from copy import deepcopy
from rlia.kit.drive_controllers.w1_drive_controller import W1Version, W1ArmType
from dexe_robot_client.end_effector_client import EndEffectorType 
from dexe_robot_client.w1 import W1Manager
from dexe_robot_client.sim_server import DexsimServer
import rclpy
from IPython import embed

TABLE_CENTER = np.array([0.8, 0.0, 1.1])
TABLE_SIZE = np.array([1.0, 1.0, 0.03])
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

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR = os.path.join(SCRIPT_DIR, "..", "model")

ROBOT_TYPE = RobotType.W1V02_2

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

def test_grasp_plan(robot_type: RobotType, ee_type: EndEffectorType = None, sim_robot = None):
    print(f"Testing grasp plan for robot type: {robot_type}, end effector type: {ee_type}")
    urdf_path = get_robot_urdf(robot_type)
    collision_path = get_robot_mesh_path(robot_type)

    fcl_checker = FCLChecker(collision_path, ee_type)
    fcl_checker.create_robot_obj()
    fcl_checker.create_robot_manager()

    fcl_checker.add_cube_obstacle(TABLE_CENTER, TABLE_SIZE)
    fcl_checker.add_grasp_object(obj_type="box", center=np.array([0.45, 0.15, 1.165]), size=np.array([0.1, 0.1, 0.08]), name="box1")

    init_qpos = np.array([-np.pi / 2, -np.pi / 4, np.pi / 2, -np.pi / 4, -np.pi / 2, np.pi / 5, 0.0])
    arml_start = init_qpos
    armr_start = -init_qpos
    arml_goal = np.array([-0.79258, -1.03953,  0.63524, -0.97332,  0.76595, -1.13355, 0.2649])
    armr_goal = armr_start
    head_qpos = np.array([0.0, 0.0])
    body_qpos = np.array([0.0, 0.0, 0.0, 0.0])
    ompl_planner = OMPLPlanSingle(SINGLE_ARM_DIM, LARM_BOUND, 
                                  arml_start, arml_goal, 
                                  fcl_checker, 
                                  urdf_path, 
                                  which_arm="left", 
                                  other_arm_qpos=armr_start,
                                  use_ee=False)
    ompl_planner.set_head_body(head_qpos, body_qpos)
    path1, trajectory1, plan_time1 = ompl_planner.plan(planning_time=2.0)

    for i in range(trajectory1.shape[0]):
        qpos = np.concatenate([init_body_qpos, init_head_qpos, trajectory1[i, :7], init_armr_qpos])
        _articulation.set_current_qpos(qpos[REAL2SIM_IDX])
        time.sleep(0.002)

    time.sleep(1.0)

    sim_qpos = sim_robot.get_current_qpos()
    sim_qpos = sim_qpos[SIM2REAL_IDX]
    cur_body, cur_head, cur_arml, cur_armr = sim_qpos[:4], sim_qpos[4:6], sim_qpos[6:13], sim_qpos[13:20]
    body_tf, arml_tf, armr_tf, head_tf = ompl_planner._get_Kinematics(cur_body, cur_arml, cur_armr, cur_head)
    w_T_o = np.array([[1, 0, 0, 0.45], [0, 1, 0, 0.15], [0, 0, 1, 1.165], [0, 0, 0, 1]])
    w_T_f = arml_tf[-1]
    f_T_o = np.linalg.inv(w_T_f) @ w_T_o

    fcl_checker.attach_object_arml(obj_name="box1", flange_T_object=f_T_o)

    arml_start = sim_qpos[6:13]
    armr_start = sim_qpos[13:20]
    arml_goal = np.array([-0.6486993, -1.2057196,  0.5688665, -0.013276 , -0.3541906, -0.9850419,  0.2607839])
    ompl_planner.set_start_goal(arml_start, arml_goal)
    path2, trajectory2, plan_time2 = ompl_planner.plan(planning_time=2.0)

    for i in range(trajectory2.shape[0]):

        body_tf, arml_tf, armr_tf, head_tf = ompl_planner._get_Kinematics(init_body_qpos, trajectory2[i, :7], init_armr_qpos, init_head_qpos)
        w_T_o = arml_tf[-1] @ f_T_o
        grasp_cube_handle.set_world_pose(w_T_o)

        qpos = np.concatenate([init_body_qpos, init_head_qpos, trajectory2[i, :7], init_armr_qpos])
        _articulation.set_current_qpos(qpos[REAL2SIM_IDX])
        time.sleep(0.002)

    time.sleep(1.0)

    sim_qpos = sim_robot.get_current_qpos()
    sim_qpos = sim_qpos[SIM2REAL_IDX]
    cur_body, cur_head, cur_arml, cur_armr = sim_qpos[:4], sim_qpos[4:6], sim_qpos[6:13], sim_qpos[13:20]
    body_tf, arml_tf, armr_tf, head_tf = ompl_planner._get_Kinematics(cur_body, cur_arml, cur_armr, cur_head)
    fcl_checker.detach_object_arml(obj_name="box1", arml7_transform=arml_tf[-1])

    arml_start = sim_qpos[6:13]
    armr_start = sim_qpos[13:20]
    arml_goal = np.array([-0.79258, -1.03953,  0.63524, -0.97332,  0.76595, -1.13355, 0.2649])
    ompl_planner.set_start_goal(arml_start, arml_goal)
    path3, trajectory3, plan_time3 = ompl_planner.plan(planning_time=2.0)

    for i in range(trajectory3.shape[0]):
        qpos = np.concatenate([init_body_qpos, init_head_qpos, trajectory3[i, :7], init_armr_qpos])
        _articulation.set_current_qpos(qpos[REAL2SIM_IDX])
        time.sleep(0.002)
    
    embed()
    return trajectory1, trajectory2



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





    robot_urdf_path = "./data/assets/DexforceW1V02_v1/DexforceW1_v02_2.urdf"
    # robot_urdf_path = "/home/young/Model/DexforceW1V02_withEE/PGC/output/assembly_robot.urdf"
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

    # input("Press Enter to continue")
    # embed()

    # Generate trajectory
    trajectory1, trajectory2 = test_grasp_plan(RobotType.W1V02_2, None, _articulation)
    



    
