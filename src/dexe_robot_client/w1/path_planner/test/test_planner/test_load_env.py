import os
# sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../..")))
from dexe_robot_client.w1.path_planner.ompl_plan import *
from dexe_robot_client.w1.path_planner.fcl_checker import *
import rclpy
from dexe_robot_client.sim_server import DexsimServer
from dexe_robot_client.end_effector_client import EndEffectorType 
import numpy as np
import time

COFFEE_PATH = "./data/assets/DexforceW1V02_v1/coffee_collision"
COFFEE_LINKS = ["body_rescale_acd.obj", "middle_rescale.obj", "top_2_rescale.obj"]
COFFEE_XPOS = np.array([
    [1, 0, 0, 0.5],
    [0, 1, 0, 0],
    [0, 0, 1, 1],
    [0, 0, 0, 1]
])

def test_load_env(json_path):
    current_path = os.path.dirname(os.path.abspath(__file__))
    current_path = os.path.join(current_path, "..")
    model_folder = os.path.join(current_path, "model")
    urdf_path = os.path.join(model_folder, "DexforceW1_v02_2.urdf")
    collision_path = os.path.join(model_folder, "collision")

    fcl_checker = FCLChecker(collision_path, EndEffectorType.BRAINCO_HAND)
    fcl_checker.create_robot_obj()
    fcl_checker.create_robot_manager()

    fcl_checker.load_env_json(json_path)
    fcl_checker.check_collision()

def test_load_coffee_sim():
    rclpy.init()
    dexsim_server = DexsimServer()
    # cube_handle = dexsim_server._env.create_cube()
    # cube_xpos = np.array([
    #     [1, 0, 0, 1],
    #     [0, 1, 0, 0],
    #     [0, 0, 1, 1],
    #     [0, 0, 0, 1]
    # ])
    # cube_handle.set_world_pose(cube_xpos)
    # cube_handle.set_body_scale(0.5, 0.5, 0.5)

    
    body_rescale_path = os.path.join(COFFEE_PATH, COFFEE_LINKS[0])
    body_handle = dexsim_server._env.load_actor(body_rescale_path)
    body_handle.set_world_pose(COFFEE_XPOS)
    middle_rescale_path = os.path.join(COFFEE_PATH, COFFEE_LINKS[1])
    middle_handle = dexsim_server._env.load_actor(middle_rescale_path)
    middle_handle.set_world_pose(COFFEE_XPOS)
    top_2_rescale_path = os.path.join(COFFEE_PATH, COFFEE_LINKS[2])
    top_handle = dexsim_server._env.load_actor(top_2_rescale_path)
    top_handle.set_world_pose(COFFEE_XPOS)

    try:
        dexsim_server.run()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

def test_load_coffee_fcl():
    current_path = os.path.dirname(os.path.abspath(__file__))
    current_path = os.path.join(current_path, "..")
    model_folder = os.path.join(current_path, "model")
    urdf_path = os.path.join(model_folder, "DexforceW1_v02_2.urdf")
    collision_path = os.path.join(model_folder, "collision")

    fcl_checker = FCLChecker(collision_path)
    fcl_checker.create_robot_obj()
    fcl_checker.create_robot_manager()

    body_rescale_path = os.path.join(COFFEE_PATH, COFFEE_LINKS[0])
    middle_rescale_path = os.path.join(COFFEE_PATH, COFFEE_LINKS[1])
    top_2_rescale_path = os.path.join(COFFEE_PATH, COFFEE_LINKS[2])
    fcl_checker.add_mesh_obstacle(body_rescale_path, COFFEE_XPOS)
    fcl_checker.add_mesh_obstacle(middle_rescale_path, COFFEE_XPOS)
    fcl_checker.add_mesh_obstacle(top_2_rescale_path, COFFEE_XPOS)


if __name__ == "__main__":
    script_path = os.path.dirname(os.path.abspath(__file__))
    script_path = os.path.join(script_path, "..")
    model_folder = os.path.join(script_path, "model")
    json_path = os.path.join(model_folder, "env.json")
    test_load_env(json_path)
