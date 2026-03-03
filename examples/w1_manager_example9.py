import os
import numpy as np
from dexe_robot_client.end_effector_client import EndEffectorType
from dexe_robot_client.common import W1Version, W1ArmType
from dexe_robot_client.w1 import W1Manager
from dexe_robot_client.w1.path_planner.utils.robot_type import *
from dexe_robot_client.common.mapping_index import *
from scipy.spatial.transform import Rotation as R
from IPython import embed
from dexe_robot_client.common.config import DATA_DIR
from loop_rate_limiters import RateLimiter
from dexe_rp import W1Vis, pos_wxyz_to_homogeneous, Box, Material

task_dict1 = {
    "torso": [[0.78539816, -1.57079633, 0.78539816, 0.03490659]],
    "torso_speed_ratio": [1.],
    "head": [[0., -0.76794487]],
    "head_speed_ratio": [1.],
    "left_arm": [[
        476.81552887392564, 151.8864369061307, 477.51278916762175,
        153.7526570682316, -85.14610235747323, 24.605372677060252
    ],
                 [
                     476.81552887392564, 151.8864369061307, 663.5256017685385,
                     153.7526570682316, -85.14610235747323, 24.605372677060252
                 ],
                 [
                     476.81552887392564, 151.8864369061307, 955.8654830562698,
                     153.7526570682316, -85.14610235747323, 24.605372677060252
                 ],
                 [
                     476.81552887392564, 151.8864369061307, 1233.6224827317324,
                     153.7526570682316, -85.14610235747323, 24.605372677060252
                 ]],
    "left_arm_speed_ratio": [0.5, 0.5, 0.5, 0.5],
    "left_arm_ee": [],
    "right_arm": [],
    "right_arm_speed_ratio": [],
    "right_arm_ee": [],
}


def test_move_linear_dual_arm_sync(task_dict):
    res, full_body_traj = w1_manager.plan_linear_dual_arm_sync(
        task_dict=task_dict, is_local=False)
    return res, full_body_traj


if __name__ == "__main__":
    w1_manager = W1Manager(is_sim=True,
                           arm_type=W1ArmType.ANTHROPOMORPHIC,
                           w1_version=W1Version.V2,
                           end_effector_type=EndEffectorType.BRAINCO_HAND)
    if not w1_manager.connect() and not w1_manager.is_enabled():
        raise RuntimeError("Failed to connect to W1 robot.")

    w1_manager.set_tcp("left_arm", np.eye(4))
    right_tcp_xpos = np.eye(4)
    right_tcp_xpos[:3, :3] = R.from_rotvec([0.0, 0.0, 180.],
                                           degrees=True).as_matrix()
    w1_manager.set_tcp("right_arm", xpos=right_tcp_xpos)

    w1_manager.set_home_qpos(np.array([
        1.353, -2.705, 1.395, 0.009, 0.0, -0.681, -0.891, -1.655, 1.55, -0.023,
        -1.514, -0.633, 0.068, 0.894, 1.7, -1.529, 0.021, 1.324, 0.679, -0.14
    ]),
                             save_to_file=False)
    w1_manager.go_home(1.)

    # # Visualization setup in viser
    # vis = W1Vis()
    # h_torso_text = vis.server.gui.add_text(label="Torso (rad)",
    #                                        initial_value="[]")
    # h_head_text = vis.server.gui.add_text(label="Head (rad)",
    #                                       initial_value="[]")
    # h_left_arm_text = vis.server.gui.add_text(label="Left arm (rad)",
    #                                           initial_value="[]")
    # h_right_arm_text = vis.server.gui.add_text(label="Right arm (rad)",
    #                                            initial_value="[]")

    # init_qpos = np.array([1.353, -2.705, 1.395, 0.009,
    #                         0.0, -0.681,
    #                         -0.891, -1.655, 1.55, -0.023, -1.514, -0.633, 0.068,
    #                         0.894, 1.7, -1.529, 0.021, 1.324, 0.679, -0.14])
    # vis.set_qpos("robot", init_qpos)

    res, full_body_traj = test_move_linear_dual_arm_sync(task_dict=task_dict1)

    input("Press Enter to exit the script and stop the robot")

    w1_manager.close()