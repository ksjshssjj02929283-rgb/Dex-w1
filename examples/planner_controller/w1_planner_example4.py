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

    # New test initial qpos
    init_qpos = np.array([0.77, -1.56, 1.19, 0.0,
                          0.0, -0.77,
                          0, 0, 0, 0, 0, 0, 0,
                          0, 0, 0, 0, 0, 0, 0])

    w1_planner.set_home_qpos(init_qpos, save_to_file=False)
    w1_planner.go_home(1)

    # realsense d455
    angles = [-0.1, np.pi/2-0.1, -0.05]  
    rot = R.from_euler('zyz', angles)
    rotation_matrix = rot.as_matrix()

    cam_neck2_extrinsics = np.eye(4)
    cam_neck2_extrinsics[:3, :3] = rotation_matrix
    cam_neck2_extrinsics[:3, 3] = [0.01, -0.1605, 0.02]

    w1_planner.init_dynamic_planner(robot_type=RobotType.W1V02_1, 
                                    end_effector_type=None)
    w1_planner.set_cam_neck2_extrinsics(cam_neck2_extrinsics)

    # while True:
    #     pcd = w1_planner._reload_environment()

    # 动态规划 初始位置
    arml_target_qpos1 = np.array([-1.5465, -1.0153,  1.6048, -0.3294, -1.6506, -0.5815, -0.1399])
    armr_target_qpos1 = np.array([1.538 , 0.7894, -1.5817,  0.6543,  1.2392,  0.314 , -0.003])

    # Move Line 左手抓取
    arml_target_qpos2 = np.array([-1.2938, -1.0668,  1.5849, -0.2508, -1.6621, -0.552 , -0.0438])
    armr_target_qpos2 = np.array([1.538 , 0.7894, -1.5817,  0.6543,  1.2392,  0.314 , -0.003])

    # Move Line 左手抬起
    arml_target_qpos3 = np.array([-1.5465, -1.0153,  1.6048, -0.3294, -1.6506, -0.5815, -0.1399])
    armr_target_qpos3 = np.array([1.538 , 0.7894, -1.5817,  0.6543,  1.2392,  0.314 , -0.003])

    # 动态规划 双手向中间位置靠拢
    arml_target_qpos4 = np.array([-1.6444, -1.1816,  1.9073, -0.821 , -1.7971, -0.5343,  0.1551])
    armr_target_qpos4 = np.array([1.5971, 1.4713, -1.9785,  0.5634,  1.4567,  0.5402, -0.436])

    w1_planner.set_dynamic_start_target_qpos(left_start_qpos=np.zeros(7), left_target_qpos=arml_target_qpos1,
                                             right_start_qpos=np.zeros(7), right_target_qpos=armr_target_qpos1)
    w1_planner.plan_keep_checking(update_frequency=5)
    time.sleep(2.0)

    

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
        w1_planner.plan_keep_checking(update_frequency=5)
        cnt += 1
        time.sleep(2.0)
        print("Next task planning...")



if __name__ == "__main__":
    test_w1_planner()