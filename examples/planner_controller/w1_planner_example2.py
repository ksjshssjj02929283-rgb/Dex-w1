import numpy as np
from dexe_robot_client.w1.w1_planner import W1Planner
from dexe_robot_client.end_effector_client import EndEffectorType 
from dexe_robot_client.common import W1Version, W1ArmType
from dexe_robot_client.w1.path_planner.utils.robot_type import *
from dexe_robot_client.w1.path_planner.utils.mapping_index import *
from IPython import embed
import time
from scipy.spatial.transform import Rotation as R

def test_w1_planner():
    w1_planner = W1Planner(
        is_sim=False,
        arm_type=W1ArmType.ANTHROPOMORPHIC,
        w1_version=W1Version.V2,
        end_effector_type=None,
        use_rlia=False
    )

    if not w1_planner.connect():
        return
    init_qpos = np.deg2rad([45, -90, 45, 2, 
                                         0, -35,
                                         0, 0, 0, 0, 0, 0, 0,
                                         0, 0, 0, 0, 0, 0, 0])
    init_qpos[6] = -np.pi / 2
    # init_qpos[6:13] = np.array([-np.pi / 2, -np.pi / 2, np.pi / 2, -np.pi / 4, -np.pi / 2, np.pi / 5, 0.0])
    w1_planner.set_home_qpos(init_qpos, save_to_file=False)
    w1_planner.go_home(0.5)

    cam_arml_extrinsics = np.array([
        [0.04763052712431963, -0.7875535718815841, -0.6144027215942477, 0.19295156040533418],
        [0.03643103300760852, -0.6133219024721747, 0.7889924104716797, 0.1114495388646769],
        [-0.9982004371466978, -0.05996345023821815, -0.0005214553391622478, -0.05491695129690629],
        [-0.0, 0.0, -0.0, 1.0]
    ])


    angles = [0.0, np.pi/2 - 0.1, -0.13]  
    rot = R.from_euler('xyz', angles)
    rotation_matrix = rot.as_matrix()

    cam_neck2_extrinsics = np.eye(4)
    cam_neck2_extrinsics[:3, :3] = rotation_matrix
    cam_neck2_extrinsics[:3, 3] = [0.027, -0.0975 +0.015, 0]  # 平移部分

    w1_planner.init_dynamic_planner(robot_type=RobotType.W1V02_1, end_effector_type=None, cam_arml_extrinsics=cam_arml_extrinsics)
    w1_planner.set_cam_neck2_extrinsics(cam_neck2_extrinsics)
    # while True:
    #     pcd = w1_planner._reload_environment()


    arml_target_qpos1 = np.array([-np.pi / 2, -np.pi / 2 - 0.4, np.pi / 2, -np.pi / 4, -np.pi / 2, np.pi / 5, 0.0])
    arml_target_qpos2 = np.array([-np.pi / 2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    input("press Enter to continue")
    cnt = 0
    # start_moving = w1_planner.start_moving()
    while True:
        if cnt % 2 == 0:
            arml_start_qpos = arml_target_qpos2
            arml_target_qpos = arml_target_qpos1
        else:
            arml_start_qpos = arml_target_qpos1
            arml_target_qpos = arml_target_qpos2
        cnt += 1
        w1_planner.set_start_target_qpos(arml_start_qpos, arml_target_qpos)

        w1_planner.plan(update_frequency=2)
        input("Press Enter to continue...")



if __name__ == "__main__":
    test_w1_planner()