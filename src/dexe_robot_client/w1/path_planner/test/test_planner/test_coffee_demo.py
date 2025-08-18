import os
from dexe_robot_client.w1.path_planner.ompl_plan import *
from dexe_robot_client.w1.path_planner.fcl_checker import *
import numpy as np
import time

# Executing dependence
from copy import deepcopy
from rlia.kit.drive_controllers.w1_drive_controller import W1Version, W1ArmType
from dexe_robot_client.end_effector_client import EndEffectorType 
from dexe_robot_client.w1 import W1Manager

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

first_execute = 0

w1_manager = W1Manager(is_sim = False,
                    arm_type=W1ArmType.INDUSTRIAL, 
                    w1_version=W1Version.V2, 
                    end_effector_type = None)

def test_coffee_demo(traj):
    current_path = os.path.dirname(os.path.abspath(__file__))
    current_path = os.path.join(current_path, "..")
    model_folder = os.path.join(current_path, "model")
    urdf_path = os.path.join(model_folder, "DexforceW1_v02_2.urdf")
    collision_path = os.path.join(model_folder, "W1V02_2_collision")

    fcl_checker = FCLChecker(collision_path, EndEffectorType.PGC_GRIPPER)
    fcl_checker.create_robot_obj()
    fcl_checker.create_robot_manager()

    # fcl_checker.add_cube_obstacle(TABLE_CENTER, TABLE_SIZE)

    init_qpos = np.array([-np.pi / 2, -np.pi / 4, np.pi / 2, -np.pi / 4, -np.pi / 2, np.pi / 5, 0.0])
    global w1_manager
    arml_start = w1_manager.get_current_qpos(name="left_arm")[1]
    armr_start = w1_manager.get_current_qpos(name="right_arm")[1]
    if arml_start is None or armr_start is None:
        return None, None
    arml_goal = traj[0][6:13]
    armr_goal = traj[0][13:20]

    print(f"arml_start: {arml_start}, armr_start: {armr_start}")
    input("Press Enter to continue...")

    head_qpos = np.array([0.0, 0.0])
    body_qpos = np.array([0.0, 0.0, 0.0, 0.0])

    ompl_planner = OMPLPlanDecoupled(SINGLE_ARM_DIM, SINGLE_ARM_DIM, 
                                     LARM_BOUND, RARM_BOUND, 
                                     arml_start, armr_start, 
                                     arml_goal, armr_goal, 
                                     fcl_checker, 
                                     urdf_path,
                                     use_ee=True)
    ompl_planner.set_head_body(head_qpos, body_qpos)

    coordinated_left, coordinated_right, plan_time = ompl_planner.plan()
    embed()
    return coordinated_left, coordinated_right

def execute(left_traj, right_traj):

    # global first_execute 
    # if  first_execute == 0:
    #     w1_manager.set_home_qpos(np.array([0, 0, 0, 0, 0.0, 0.0, 
    #                                   -np.pi / 2, -np.pi / 4, np.pi / 2, -np.pi / 4, -np.pi / 2, np.pi / 5, 0.0, 
    #                                   np.pi / 2, np.pi / 4, -np.pi / 2, np.pi / 4, np.pi / 2, -np.pi / 5, 0.0]),
    #                         save_to_file=False)
    #     w1_manager.go_home(0.5)
    #     first_execute = 1
    
    arml_current = w1_manager.get_current_qpos(name="left_arm")[1]
    armr_current = w1_manager.get_current_qpos(name="right_arm")[1]
    head_current = w1_manager.get_current_qpos(name="head")[1]
    body_current = w1_manager.get_current_qpos(name="torso")[1]

    execute_traj = np.zeros((left_traj.shape[0], 20))
    # print(body_current.shape, head_current.shape)
    input("Press Enter to continue...")
    for i in range(left_traj.shape[0]):
        execute_traj[i, :4] = body_current
        execute_traj[i, 4:6] = head_current
        execute_traj[i, 6:13] = left_traj[i]
        execute_traj[i, 13:20] = right_traj[i]
            
    w1_manager.execute_joint_traj(execute_traj)


if __name__ == "__main__":
    folder_path = "/home/dexforce/Downloads/coffee_log/robot_traj"
    file_name = ["1.npy", "2.npy", "3.npy", "4.npy", "5.npy", "6.npy", "7.npy", "8.npy", "9.npy", "10.npy", "11.npy", "12.npy", "13.npy", "14.npy"]
    trajectory_list = []
    for i in range(len(file_name)):
        file_path = os.path.join(folder_path, file_name[i])
        trajectory = np.load(file_path)
        trajectory_list.append(trajectory)

    print(f"trajectory list length: {len(trajectory_list)}")
    for i in range(len(trajectory_list)):
        print(f"traectory {i+1} with start {trajectory_list[i][0]}")
        print(f"trajectory {i+1} with end {trajectory_list[i][-1]}")

    input("Press Enter to start executing trajectories...")


    for i in range(len(trajectory_list)):
        traj = trajectory_list[i]
        coordinated_left, coordinated_right = test_coffee_demo(traj)
        if coordinated_left is None or coordinated_right is None:
            print(f"Failed to plan trajectory {i+1}. Skipping...")
            break
        execute(coordinated_left, coordinated_right)
    