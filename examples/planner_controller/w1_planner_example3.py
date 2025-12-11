import numpy as np
from dexe_robot_client.w1.w1_planner import W1Planner
from dexe_robot_client.end_effector_client import EndEffectorType 
from dexe_robot_client.common import W1Version, W1ArmType
from dexe_robot_client.w1.path_planner.utils.robot_type import *
from dexe_robot_client.w1.path_planner.utils.mapping_index import *
from IPython import embed
import time
from scipy.spatial.transform import Rotation as R

def print_time_consuming(time_dict):
    print("┌─────────────────────────────┬──────────────┐")
    print("│ Task                        │ Time (s)     │")
    print("├─────────────────────────────┼──────────────┤")
    for k, v in time_dict.items():
        print(f"│ {k:<25} │ {v:>10.6f} │")
    print("└─────────────────────────────┴──────────────┘")
    pass

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
    
    w1_planner._pin_kine.use_ee = True
    # init_qpos = np.deg2rad([45, -90, 45, 2, 
    #                                      0, -35,
    #                                      0, 0, 0, 0, 0, 0, 0,
    #                                      0, 0, 0, 0, 0, 0, 0])
    # init_qpos[6] = -np.pi / 2
    # init_qpos[13] = np.pi / 2
    # init_qpos[6:13] = np.array([-np.pi / 2, -1.38, np.pi / 2, -np.pi / 4, -np.pi / 2, np.pi / 5, 0.0])
    # init_qpos[13:20] = np.array([np.pi / 2, 1.38, -np.pi / 2, np.pi / 4, np.pi / 2, -np.pi / 5, 0.0])

    # Previous test initial qpos
    # init_qpos = np.array([0.77, -1.56, 1.19, 0.0,
    #                       0.4, -0.77,
    #                       -1.37, -0.3, 1.39, -0.74, 0.0, 0.0, 0.0,
    #                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    # # init_qpos[6:13] = np.array([-1.37, -1.5, 1.39, -0.97, 0.0, 0.0, 0.0])
    # # init_qpos[6:13] = np.array([-1.37, -1, 1.39, -0.97, 0.0, 0.0, 0.0])

    # New test initial qpos
    init_qpos = np.array([0.785, -1.57, 0.785, 0.0,
                          0.0, -0.77,
                          -1.247, -1.573, 0.933, -0.419, -0.007, -0.436, 0.01,
                          1.533, 1.276, -2.031, 0.593, 0.882, 0.613, -0.233])
    # init_qpos[6:13] = np.array([-1.4316, -1.3 ,  1.5469, -0.0814, -1.6173, -0.4734,  0.1073])
    init_qpos[13:] = np.array([1.533, 0.643, -2.031, 0.578, 0.882, 0.603, -0.234])

    w1_planner.set_home_qpos(init_qpos, save_to_file=False)
    w1_planner.go_home(1)

    # embed()


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
    # angles = [-0.2, np.pi/2-0.1 - 0.0, 0.05]  
    # rot = R.from_euler('zyz', angles)
    # rotation_matrix = rot.as_matrix()

    # cam_neck2_extrinsics = np.eye(4)
    # cam_neck2_extrinsics[:3, :3] = rotation_matrix
    # cam_neck2_extrinsics[:3, 3] = [0.01, -0.1605 - 0.04, 0.02]

    # realsense d555
    angles = [-2.0899999999999994, -0.020000000000000025, -1.6500000000000012]  
    rot = R.from_euler('xyz', angles)
    rotation_matrix = rot.as_matrix()

    cam_base_link_extrinsics = np.eye(4)
    cam_base_link_extrinsics[:3, :3] = rotation_matrix
    cam_base_link_extrinsics[:3, 3] = [0.01200000000000007, 0.07399999999999998, 1.5259999999999996]

    w1_planner.init_dynamic_planner(robot_type=RobotType.W1V02_1, 
                                    end_effector_type=None, 
                                    cam_arml_extrinsics=cam_arml_extrinsics,
                                    cam_armr_extrinsics=cam_armr_extrinsics)
    w1_planner.set_cam_base_link_extrinsics(cam_base_link_extrinsics)

    # while True:
    #     pcd = w1_planner._reload_environment()


    # arml_target_qpos1 = np.array([-np.pi / 2, -1.38, np.pi / 2, -np.pi / 4, -np.pi / 2, np.pi / 5, 0.0])
    # armr_target_qpos1 = np.array([np.pi / 2, 1.38, -np.pi / 2, np.pi / 4, np.pi / 2, -np.pi / 5, 0.0])
    # arml_target_qpos2 = np.array([-np.pi / 2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    # armr_target_qpos2 = np.array([np.pi / 2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # arml_target_qpos1 = np.array([-1.37, -1.5, 1.39, -0.97, 0.0, 0.0, 0.0])
    arml_target_qpos1 = np.array([-1.247, -0.16, 0.933, -0.419, -0.008, -0.436, 0.01])
    armr_target_qpos1 = np.array([1.533, 0.643, -2.031, 0.578, 0.882, 0.603, -0.234])

    # arml_target_qpos2 = np.array([-1.37, -0.3, 1.39, -0.74, 0.0, 0.0, 0.0])
    arml_target_qpos2 = np.array([-1.247, -1.573, 0.933, -0.419, -0.007, -0.436, 0.01])
    armr_target_qpos2 = np.array([1.533, 1.276, -2.031, 0.593, 0.882, 0.613, -0.233])
    armr_target_qpos2 = np.array([1.533, 0.643, -2.031, 0.578, 0.882, 0.603, -0.234])

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
        # total_pcd_time = 0
        # one_pcd_check = []
        # for time_dict in w1_planner.time_dict_list:
        #     # print_time_consuming(w1_planner.time_dict_list[i])
        #     temp_pcd_time = time_dict["reading pcd msg"] + time_dict["ROI crop"] + time_dict["downsample1"] + time_dict["pose transform"] + \
        #                     time_dict["sync pcd and feedback"] + time_dict["reset env & add torso"] + time_dict["filter self"] + time_dict["remove noise"]
        #     total_pcd_time += temp_pcd_time
        #     temp_average_checking_time = time_dict.get("average checking time", None)
        #     if temp_average_checking_time is not None:
        #         temp_one_pcd_check = temp_average_checking_time / time_dict["Result left pcd"]
        #         one_pcd_check.append(temp_one_pcd_check)
        # total_one_pcd_check = 0
        # for i in range(len(one_pcd_check)):
        #     total_one_pcd_check += one_pcd_check[i]
        
        # print("-----------------------------------")
        # print(f"Average pcd processing time {total_pcd_time / len(w1_planner.time_dict_list)}")
        # print(f"Average one pcd checking time {total_one_pcd_check / len(one_pcd_check)}")
        # print("-----------------------------------")
        time.sleep(2.0)
        print("Next task planning...")



if __name__ == "__main__":
    test_w1_planner()