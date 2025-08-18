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

def test_central_plan():
    current_path = os.path.dirname(os.path.abspath(__file__))
    current_path = os.path.join(current_path, "..")
    model_folder = os.path.join(current_path, "model")
    urdf_path = os.path.join(model_folder, "DexforceW1_v02_2.urdf")
    collision_path = os.path.join(model_folder, "W1V02_2_collision")

    fcl_checker = FCLChecker(collision_path, EndEffectorType.PGC_GRIPPER)
    fcl_checker.create_robot_obj()
    fcl_checker.create_robot_manager()

    fcl_checker.add_cube_obstacle(TABLE_CENTER, TABLE_SIZE)

    init_qpos = np.array([-np.pi / 2, -np.pi / 4, np.pi / 2, -np.pi / 4, -np.pi / 2, np.pi / 5, 0.0])
    arml_start = init_qpos
    armr_start = -init_qpos
    arml_goal = np.array([-0.6486993, -1.2057196,  0.5688665, -0.013276 , -0.3541906, -0.9850419,  0.2607839])
    armr_goal = -np.array([-0.6486993, -1.2057196,  0.5688665, -0.013276 , -0.3541906, -0.9850419,  0.2607839])

    head_qpos = np.array([0.0, 0.0])
    body_qpos = np.array([0.0, 0.0, 0.0, 0.0])

    ompl_planner = OMPLPlanCentralized(DUAL_ARM_CEN_DIM, 
                                       LARM_BOUND, RARM_BOUND, 
                                       arml_start, armr_start, 
                                       arml_goal, armr_goal, 
                                       fcl_checker, 
                                       urdf_path,
                                       use_ee=True)
    ompl_planner.set_head_body(head_qpos, body_qpos)
    path, trajectory, plan_time = ompl_planner.plan(planning_time=5.0)
    ompl_planner.save_result_path("dual_central_table.csv")
    def test_change_start_goal():
        ompl_planner.set_start_goal(arml_goal, armr_goal, arml_start, armr_start)
        path, trajectory, plan_time = ompl_planner.plan(planning_time=5.0)
        ompl_planner.save_result_path("dual_central_table_change.csv")

    embed()
    return trajectory


def execute(trajectory):
    w1_manager = W1Manager(is_sim = False,
                        arm_type=W1ArmType.INDUSTRIAL, 
                        w1_version=W1Version.V2, 
                        end_effector_type = None)
    w1_manager.set_home_qpos(np.array([0, 0, 0, 0, 0.0, 0.0, 
                                      -np.pi / 2, -np.pi / 4, np.pi / 2, -np.pi / 4, -np.pi / 2, np.pi / 5, 0.0, 
                                      np.pi / 2, np.pi / 4, -np.pi / 2, np.pi / 4, np.pi / 2, -np.pi / 5, 0.0]),
                            save_to_file=False)
    w1_manager.go_home(0.5)
    arml_current = w1_manager.get_current_qpos(name="left_arm")[1]
    armr_current = w1_manager.get_current_qpos(name="right_arm")[1]
    head_current = w1_manager.get_current_qpos(name="head")[1]
    body_current = w1_manager.get_current_qpos(name="torso")[1]

    execute_traj = np.zeros((trajectory.shape[0], 20))
    # print(body_current.shape, head_current.shape)
    input("Press Enter to continue...")
    for i in range(trajectory.shape[0]):
        execute_traj[i, :4] = body_current
        execute_traj[i, 4:6] = head_current
        execute_traj[i, 6:13] = trajectory[i, :7]
        execute_traj[i, 13:20] = trajectory[i, 7:]
            
    w1_manager.execute_joint_traj(execute_traj)

if __name__ == "__main__":
    dual_arm_trajectory = test_central_plan()
    # execute(dual_arm_trajectory)

