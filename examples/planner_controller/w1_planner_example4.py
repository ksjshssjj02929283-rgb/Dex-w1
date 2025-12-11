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
        end_effector_type=EndEffectorType.BRAINCO_HAND,
        use_rlia=False
    )

    if not w1_planner.connect():
        return
    
    w1_planner._pin_kine.use_ee = True

    w1_planner.set_tcp("left_arm", np.eye(4))
    right_tcp_xpos = np.eye(4)
    right_tcp_xpos[:3, :3] = R.from_rotvec([0.0, 0.0, 180.],
                                           degrees=True).as_matrix()
    w1_planner.set_tcp("right_arm", xpos=right_tcp_xpos)

    # New test initial qpos
    init_qpos = np.array([0.785, -1.57, 0.785, 0.0,
                          0.0, -0.77,
                          -1.247, -0.16, 0.933, -0.419, -0.008, -0.436, 0.01,
                          1.533, 0.643, -2.031, 0.578, 0.882, 0.603, -0.234])

    w1_planner.set_home_qpos(init_qpos, save_to_file=False)
    w1_planner.go_home(1)


    # realsense d455
    angles = [-2.159999999999998, -0.010000000000000024, -1.5700000000000012]  
    rot = R.from_euler('xyz', angles)
    rotation_matrix = rot.as_matrix()

    cam_base_link_extrinsics = np.eye(4)
    cam_base_link_extrinsics[:3, :3] = rotation_matrix
    cam_base_link_extrinsics[:3, 3] = [0.01800000000000007, 0.04799999999999996, 1.5439999999999996]

    cam_base_link_extrinsics = np.array([
                    [-0.0174, -0.5884,  0.8084,  0.0439],
                    [-0.9998,  0.004,  -0.0186,  0.0443],
                    [ 0.0077, -0.8086, -0.5884,  1.5376],
                    [ 0.,      0.,      0.,      1.    ]
            ])

    w1_planner.init_dynamic_planner(robot_type=RobotType.W1V02_1, 
                                    end_effector_type=None)
    w1_planner.set_cam_base_link_extrinsics(cam_base_link_extrinsics)

    # print(f"manual extrinsics set:\n{cam_base_link_extrinsics}")
    # cur_qpos = w1_planner.get_current_qpos()[1]
    # arml_base_tf, armr_base_tf = w1_planner._pin_kine.get_arm_base_tf(real_qpos=cur_qpos)
    # cali_arm_base_extrinsics = np.array([
    #     [0.007704791930453307, -0.8085544207078107, -0.5883709586095873, 0.3718428938235822],
    #     [-0.017501846691544382, -0.5884073336044382, 0.8083752192657199, 0.04386740321139616],
    #     [-0.9998171440536984, 0.004069215449165034, -0.018684751588773627, -0.05416699938222554],
    #     [0.0, 0.0, 0.0, 1.0]
    # ])
    # cali_base_link_extrinsics = arml_base_tf @ cali_arm_base_extrinsics
    # print(f"calibrated extrinsics:\n{cali_base_link_extrinsics}")

    # embed()


    # while True:
    #     pcd = w1_planner._reload_environment()

    # 动态规划 左手运动到工件上方
    arml_target_qpos1 = np.array([-1.027, -0.818, 1.051, -0.616, -1.154, -0.529, -0.126])
    armr_target_qpos1 = np.array([1.533, 0.643, -2.031, 0.578, 0.882, 0.603, -0.234])

    # Move Line 左臂运动到抓取工件的位置
    arml_target_qpos2 = np.array([-0.887, -0.967, 1.244, -0.361, -1.311, -0.531, -0.107])
    armr_target_qpos2 = np.array([1.533, 0.643, -2.031, 0.578, 0.882, 0.603, -0.234])
    task_dict2 = {
                    "torso": [[0.785, -1.57, 0.785, 0.03490658503988659]],
                    "torso_speed_ratio": [1.],
                    "head": [[0.0, -0.77]],
                    "head_speed_ratio": [1.],
                    "left_arm": [[-274.18, 446.74, 325.84, -82.81, 6.62, 2.64],],
                    "left_arm_speed_ratio": [1.],
                    "left_arm_ee": [[0.2, [60, 99, 80, 80, 80, 80], 0.3],],
                }
    

    
    # Move Line 左臂运动到离开桌面的位置
    arml_target_qpos3 = np.array([-1.09, -0.968, 1.243, -0.362, -1.312, -0.19, -0.107])
    armr_target_qpos3 = np.array([1.533, 0.643, -2.031, 0.578, 0.882, 0.603, -0.234])
    task_dict3 = {
                    "torso": [[0.785, -1.57, 0.785, 0.03490658503988659]],
                    "torso_speed_ratio": [1.],
                    "head": [[0.0, -0.77]],
                    "head_speed_ratio": [1.],
                    "left_arm": [[-200.8, 492.82, 326.89, -81.46, 1.75, 10.01],],
                    "left_arm_speed_ratio": [1.],
                    "left_arm_ee": [[0.0, [60, 99, 80, 80, 80, 80], 0.0],],
                }
    
    # 动态规划 左臂运动到待放置的位置
    arml_target_qpos4 = np.array([-1.106, -1.351, 1.472, -0.894, -1.233, -0.368, -0.201])
    armr_target_qpos4 = np.array([1.008, 0.85, -1.704, 0.848, 1.535, 0.458, 0.568])

    # 右臂运动到抓取位置
    arml_target_qpos5 = np.array([-1.106, -1.351, 1.472, -0.894, -1.233, -0.368, -0.201])
    armr_target_qpos5 = np.array([1.038, 1.025, -1.654, 0.889, 1.457, 0.52, 0.409])
    task_dict5 = {
                    "torso": [[0.785, -1.57, 0.785, 0.03490658503988659]],
                    "torso_speed_ratio": [1.],
                    "head": [[0.0, -0.77]],
                    "head_speed_ratio": [1.],
                    "right_arm": [[255.12, 442.84, 102.07, -135.63, 0.72, -2.87],],
                    "right_arm_speed_ratio": [1.],
                    "right_arm_ee": [[0.5, [0, 100, 0, 0, 0, 0], 0.5],],
                }
    
    # 左臂放开
    arml_target_qpos6 = np.array([-1.106, -1.351, 1.472, -0.894, -1.233, -0.368, -0.201])
    armr_target_qpos6 = np.array([1.038, 1.025, -1.654, 0.889, 1.457, 0.52, 0.409])

    # 左臂离开
    arml_target_qpos7 = np.array([-1.151, -0.914, 1.418, -1.207, -1.232, -0.19, -0.382])
    armr_target_qpos7 = np.array([1.038, 1.025, -1.654, 0.889, 1.457, 0.52, 0.409])
    task_dict7 = {
                    "torso": [[0.785, -1.57, 0.785, 0.03490658503988659]],
                    "torso_speed_ratio": [1.],
                    "head": [[0.0, -0.77]],
                    "head_speed_ratio": [1.],
                    "left_arm": [[-143.92, 445.68, 71.91, -144.43, 3.92, -0.68],],
                    "left_arm_speed_ratio": [1.],
                    "left_arm_ee": [[0.0, [0, 99, 0, 0, 0, 0], 0.0],],
                }
    
    # 右臂运动到待放置位置
    arml_target_qpos8 = np.array([-1.151, -0.914, 1.418, -1.207, -1.232, -0.19, -0.382])
    armr_target_qpos8 = np.array([1.098, 0.672, -1.536, 0.158, 1.582, 0.564, 0.364])




    # ------------------------ 开始执行 ------------------------ 
    w1_planner.set_ee_qpos("left_arm_ee", np.array([0, 0, 0, 0, 0, 0]))
    w1_planner.set_ee_qpos("right_arm_ee", np.array([0, 0, 0, 0, 0, 0]))
    w1_planner.set_dynamic_start_target_qpos(left_start_qpos=np.array([-1.247, -0.16, 0.933, -0.419, -0.008, -0.436, 0.01]), 
                                             left_target_qpos=arml_target_qpos1,
                                             right_start_qpos=np.array([1.533, 0.643, -2.031, 0.578, 0.882, 0.603, -0.234]), 
                                             right_target_qpos=armr_target_qpos1)
    w1_planner.plan_keep_checking(update_frequency=5)
    w1_planner.set_ee_qpos("left_arm_ee", np.array([0, 100, 0, 0, 0, 0]))
    time.sleep(1.0)

    w1_planner.move_linear_full_body(waypoints_dict=task_dict2)

    arml_attachment_tf = np.identity(4)
    arml_attachment_tf[:3, 3] = [0.0, 0.0, 0.106]
    arml_attachment_spheres = [
        ([0.05, 0.045, 0.0], 0.05),
        ([0.025, 0.045, 0.0], 0.05),
        ([0.0, 0.045, 0.0], 0.05),
        ([-0.05, 0.045, 0.0], 0.05),
        ([-0.075, 0.045, 0.0], 0.05),
        ([-0.1, 0.045, 0.0], 0.05)
    ]
    w1_planner.attach_left_arm(attachment_frame_tf=arml_attachment_tf, spheres=arml_attachment_spheres)

    w1_planner.move_linear_full_body(waypoints_dict=task_dict3)

    cur_qpos = w1_planner.get_current_qpos()[1]
    cur_arml_qpos = cur_qpos[6:13]
    cur_armr_qpos = cur_qpos[13:]
    w1_planner.set_dynamic_start_target_qpos(left_start_qpos=cur_arml_qpos, 
                                             left_target_qpos=arml_target_qpos4,
                                             right_start_qpos=cur_armr_qpos, 
                                             right_target_qpos=armr_target_qpos4)
    w1_planner.plan_keep_checking(update_frequency=5)

    # ----------test-----------
    cur_qpos = w1_planner.get_current_qpos()[1]
    cur_arml_qpos = cur_qpos[6:13]
    cur_armr_qpos = cur_qpos[13:]
    w1_planner.set_dynamic_start_target_qpos(left_start_qpos=cur_arml_qpos, 
                                             left_target_qpos=arml_target_qpos1,
                                             right_start_qpos=cur_armr_qpos, 
                                             right_target_qpos=armr_target_qpos4)
    w1_planner.plan_keep_checking(update_frequency=5)
    cur_qpos = w1_planner.get_current_qpos()[1]
    cur_arml_qpos = cur_qpos[6:13]
    cur_armr_qpos = cur_qpos[13:]
    w1_planner.set_dynamic_start_target_qpos(left_start_qpos=cur_arml_qpos, 
                                             left_target_qpos=arml_target_qpos4,
                                             right_start_qpos=cur_armr_qpos, 
                                             right_target_qpos=armr_target_qpos4)
    w1_planner.plan_keep_checking(update_frequency=5)
    cur_qpos = w1_planner.get_current_qpos()[1]
    cur_arml_qpos = cur_qpos[6:13]
    cur_armr_qpos = cur_qpos[13:]
    w1_planner.set_dynamic_start_target_qpos(left_start_qpos=cur_arml_qpos, 
                                             left_target_qpos=arml_target_qpos1,
                                             right_start_qpos=cur_armr_qpos, 
                                             right_target_qpos=armr_target_qpos4)
    w1_planner.plan_keep_checking(update_frequency=5)
    cur_qpos = w1_planner.get_current_qpos()[1]
    cur_arml_qpos = cur_qpos[6:13]
    cur_armr_qpos = cur_qpos[13:]
    w1_planner.set_dynamic_start_target_qpos(left_start_qpos=cur_arml_qpos, 
                                             left_target_qpos=arml_target_qpos4,
                                             right_start_qpos=cur_armr_qpos, 
                                             right_target_qpos=armr_target_qpos4)
    w1_planner.plan_keep_checking(update_frequency=5)
    #----------test-----------

    w1_planner.set_ee_qpos("right_arm_ee", np.array([0, 100, 0, 0, 0, 0]))

    w1_planner.move_linear_full_body(waypoints_dict=task_dict5)

    w1_planner.set_ee_qpos("right_arm_ee", np.array([80, 99, 80, 80, 80, 80]))
    
    # input("Press Enter to continue...")

    armr_attachment_tf = np.identity(4)
    armr_attachment_tf[:3, 3] = [0.0, 0.0, 0.106]
    armr_attachment_spheres = [
        ([-0.05, -0.045, 0.0], 0.05),
        ([-0.025, -0.045, 0.0], 0.05),
        ([0.0, -0.045, 0.0], 0.05),
        ([0.05, -0.045, 0.0], 0.05),
        ([0.075, -0.045, 0.0], 0.05),
        ([0.1, -0.045, 0.0], 0.05)
    ]
    w1_planner.attach_right_arm(attachment_frame_tf=armr_attachment_tf, spheres=armr_attachment_spheres)

    time.sleep(1.0)

    w1_planner.set_ee_qpos("left_arm_ee", np.array([0, 99, 0, 0, 0, 0]))

    w1_planner.detach_left_arm()

    w1_planner.move_linear_full_body(waypoints_dict=task_dict7)

    cur_qpos = w1_planner.get_current_qpos()[1]
    cur_arml_qpos = cur_qpos[6:13]
    cur_armr_qpos = cur_qpos[13:]
    w1_planner.set_dynamic_start_target_qpos(left_start_qpos=cur_arml_qpos, 
                                             left_target_qpos=arml_target_qpos8,
                                             right_start_qpos=cur_armr_qpos, 
                                             right_target_qpos=armr_target_qpos8)
    w1_planner.plan_keep_checking(update_frequency=5)





if __name__ == "__main__":
    test_w1_planner()