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
    # init_qpos = np.deg2rad([45, -90, 45, 2, 
    #                                      0, -35,
    #                                      0, 0, 0, 0, 0, 0, 0,
    #                                      0, 0, 0, 0, 0, 0, 0])
    # init_qpos[6] = -np.pi / 2
    # init_qpos[13] = np.pi / 2
    # init_qpos[6:13] = np.array([-np.pi / 2, -1.38, np.pi / 2, -np.pi / 4, -np.pi / 2, np.pi / 5, 0.0])
    # init_qpos[13:20] = np.array([np.pi / 2, 1.38, -np.pi / 2, np.pi / 4, np.pi / 2, -np.pi / 5, 0.0])

    init_qpos = np.array([0.77, -1.56, 1.19, 0.0,
                          0.4, -0.77,
                          -1.37, -0.3, 1.39, -0.74, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    # init_qpos[6:13] = np.array([-1.37, -1.5, 1.39, -0.97, 0.0, 0.0, 0.0])

    w1_planner.set_home_qpos(init_qpos, save_to_file=False)
    w1_planner.go_home(1)

    # print(f"base_link_T_neck2:\n{w1_planner._get_base_link_T_neck2()}")
    # print(f"waist_T_neck2:\n{w1_planner._get_waist_T_neck2()}")
    # print(f"base_link_T_waist:\n{w1_planner._get_base_link_T_waist()}")

    # input("Press Enter to continue...")


    cam_arml_extrinsics = np.array([
        [0.04763052712431963, -0.7875535718815841, -0.6144027215942477, 0.19295156040533418],
        [0.03643103300760852, -0.6133219024721747, 0.7889924104716797, 0.1114495388646769],
        [-0.9982004371466978, -0.05996345023821815, -0.0005214553391622478, -0.05491695129690629],
        [-0.0, 0.0, -0.0, 1.0]
    ])

    cam_armr_extrinsics = np.array([
        [0.04763052712431963, -0.7875535718815841, -0.6144027215942477, 0.19295156040533418],
        [0.03643103300760852, -0.6133219024721747, 0.7889924104716797, 0.1114495388646769],
        [-0.9982004371466978, -0.05996345023821815, -0.0005214553391622478, -0.05491695129690629],
        [-0.0, 0.0, -0.0, 1.0]
    ])

    # orbbec
    # angles = [-0.06, np.pi/2-0.06, -0.15]  
    # rot = R.from_euler('zyz', angles)
    # rotation_matrix = rot.as_matrix()

    # cam_neck2_extrinsics = np.eye(4)
    # cam_neck2_extrinsics[:3, :3] = rotation_matrix
    # cam_neck2_extrinsics[:3, 3] = [0.027, -0.0975, 0]  # 平移部分

    # realsense d455
    # angles = [-0.1, np.pi/2-0.1, -0.05]  
    # rot = R.from_euler('zyz', angles)
    # rotation_matrix = rot.as_matrix()

    # cam_neck2_extrinsics = np.eye(4)
    # cam_neck2_extrinsics[:3, :3] = rotation_matrix
    # cam_neck2_extrinsics[:3, 3] = [0.01, -0.1605, 0.02]

    # realsense d405
    angles = [-0.2, np.pi/2-0.1 - 0.0, 0.05]  
    rot = R.from_euler('zyz', angles)
    rotation_matrix = rot.as_matrix()

    cam_neck2_extrinsics = np.eye(4)
    cam_neck2_extrinsics[:3, :3] = rotation_matrix
    cam_neck2_extrinsics[:3, 3] = [0.01, -0.1605 - 0.04, 0.02]

    w1_planner.init_dynamic_planner(robot_type=RobotType.W1V02_1, 
                                    end_effector_type=None, 
                                    cam_arml_extrinsics=cam_arml_extrinsics,
                                    cam_armr_extrinsics=cam_armr_extrinsics)
    w1_planner.set_cam_neck2_extrinsics(cam_neck2_extrinsics)
    # while True:
    #     pcd = w1_planner._reload_environment()


    # arml_target_qpos1 = np.array([-np.pi / 2, -1.38, np.pi / 2, -np.pi / 4, -np.pi / 2, np.pi / 5, 0.0])
    # armr_target_qpos1 = np.array([np.pi / 2, 1.38, -np.pi / 2, np.pi / 4, np.pi / 2, -np.pi / 5, 0.0])
    # arml_target_qpos2 = np.array([-np.pi / 2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    # armr_target_qpos2 = np.array([np.pi / 2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    arml_target_qpos1 = np.array([-1.37, -1.5, 1.39, -0.97, 0.0, 0.0, 0.0])
    # armr_target_qpos1 = np.array([0.75, 1.43, -1.07, 0.7, 0.0, 0.0, 0.0])
    armr_target_qpos1 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    arml_target_qpos2 = np.array([-1.37, -0.3, 1.39, -0.74, 0.0, 0.0, 0.0])
    armr_target_qpos2 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # input("press Enter to continue")
    cnt = 0
    while True:
        if cnt % 2 == 0:
            arml_start_qpos = arml_target_qpos2
            arml_target_qpos = arml_target_qpos1
            armr_start_qpos = armr_target_qpos2
            armr_target_qpos = armr_target_qpos1
        else:
            arml_start_qpos = arml_target_qpos1
            arml_target_qpos = arml_target_qpos2
            armr_start_qpos = armr_target_qpos1
            armr_target_qpos = armr_target_qpos2

        w1_planner.set_dynamic_start_target_qpos(left_start_qpos=arml_start_qpos, left_target_qpos=arml_target_qpos,
                                                 right_start_qpos=armr_start_qpos, right_target_qpos=armr_target_qpos)
        # w1_planner.plan(update_frequency=3)
        w1_planner.plan_keep_checking(update_frequency=5)
        cnt += 1
        # input("Press Enter to continue...")
        time.sleep(2.0)
        print("Next task planning...")



if __name__ == "__main__":
    test_w1_planner()