import os
import numpy as np
from dexe_robot_client.end_effector_client import EndEffectorType 
from dexe_robot_client.common import W1Version, W1ArmType
from dexe_robot_client.w1 import W1Manager
from dexe_robot_client.w1.path_planner.utils.robot_type import *
from dexe_robot_client.common.mapping_index import *
from scipy.spatial.transform import Rotation as R
from IPython import embed
from ..src.dexe_robot_client.common.config import DATA_DIR

# 回原点
print("回原点")
task_dict1 = {
            "torso": [
                [0.7853981633974483, -1.5707963267948966, 0.7853981633974483, 0.03490658503988659]
                ],
            "torso_speed_ratio": [1.],
            "head": [
                [0.0, -0.6632251157578453]
                ],
            "head_speed_ratio": [1.],
            "left_arm": [
                [-77.08, 260.39, 328.83, 91.77, 86.44, 170.82]
                ],
            "left_arm_speed_ratio": [1.],
            "left_arm_ee": [
                [0, [0, 0, 0, 0, 0, 0], 0]
                ],
            "right_arm": [
                [101.45, 242.18, 331.16, 64.6, -89.06, -149.32]
                ],
            "right_arm_speed_ratio": [1.],
            "right_arm_ee": [
                [0, [0, 0, 0, 0, 0, 0], 0]
                ]
            }

# 变手姿态下发给机械臂信息
print("变手姿态下发给机械臂信息")
task_dict2 = {
            "torso": [
                [0.7853981633974483, -1.5707963267948966, 0.7853981633974483, 0.03490658503988659]
                ],
            "torso_speed_ratio": [1.],
            "head": [
                [0.0, -0.6632251157578453]
                ],
            "head_speed_ratio": [1.],
            "left_arm": [
                [-77.08, 260.39, 328.83, 91.77, 86.44, 170.82]
                ],
            "left_arm_speed_ratio": [1.],
            "left_arm_ee": [
                [0, [40, 100, 45, 50, 100, 100], 0]
                ],
            "right_arm": [
                [101.45, 242.18, 331.16, 64.6, -89.06, -149.32]
                ],
            "right_arm_speed_ratio": [1.],
            "right_arm_ee": [
                [0, [0, 99, 45, 48, 50, 40], 0]
                ]
            }

# 左手抓胶囊_右手抓杯子下发给机械臂信息
print("左手抓胶囊_右手抓杯子下发给机械臂信息")
task_dict3 = {
            "torso": [
                [0.5410520681182421, -1.2217304763960306, 0.7853981633974483, 0.03490658503988659]
                ],
            "torso_speed_ratio": [1.],
            "head": [
                [0.0, -0.6632251157578453]
                ],
            "head_speed_ratio": [1.],
            "left_arm": [
                [-243.963, 349.782, 245.312, 129.592, 81.46, -111.979]
                ],
            "left_arm_speed_ratio": [1.],
            "left_arm_ee": [
                [0.1, [61, 100, 50, 59, 100, 100], 0.5]
                ],
            "right_arm": [
                [170.915, 433.923, 245.082, -68.944, 3.401, 6.925]
                ],
            "right_arm_speed_ratio": [1.],
            "right_arm_ee": [
                [0.2, [20, 99, 65, 80, 65, 46], 0.7]
                ]
            }

task_dict4 = {
            "torso": [
                [0.5410520681182421, -1.2217304763960306, 0.7853981633974483, 0.03490658503988659]
                ],
            "torso_speed_ratio": [1.],
            "head": [
                [0.0, -0.6632251157578453]
                ],
            "head_speed_ratio": [1.],
            "left_arm": [
                [-88.738, 425.079, 194.573, -118.667, 19.658, -33.72]
                ],
            "left_arm_speed_ratio": [1.],
            "left_arm_ee": [
                [0.5, [45, 99, 50, 55, 100, 100], 0.6]
                ],
            "right_arm": [
                [260.919, 433.915, 245.08, -68.944, 3.401, 6.925]
                ],
            "right_arm_speed_ratio": [1.],
            "right_arm_ee": [
                [0, [0, 99, 33, 42, 44, 23], 0.4]
                ]
            }

# 放较囊_放杯子下发给机械臂信息
print("放较囊_放杯子下发给机械臂信息")
task_dict5 = {
            "torso": [
                [0.4886921905584123, -1.2217304763960306, 0.7853981633974483, 0.05235987755982989]
                ],
            "torso_speed_ratio": [1.],
            "head": [
                [-0.03490658503988659, -0.5585053606381855]
                ],
            "head_speed_ratio": [1.],
            "left_arm": [
                [-88.738, 425.079, 194.573, -118.667, 19.658, -33.72]
                ],
            "left_arm_speed_ratio": [1.],
            "left_arm_ee": [
                [0, [0, 0, 0, 0, 0, 0], 0]
                ],
            "right_arm": [
                [324.954, 351.643, -28.01, -106.74, 0.823, 7.653]
                ],
            "right_arm_speed_ratio": [1.],
            "right_arm_ee": [
                [0.2, [0, 100, 25, 35, 35, 20], 0.7]
                ]
            }
# 放胶囊_收回手下发给机械臂信息
print("放胶囊_收回手下发给机械臂信息")
task_dict6 = {
            "torso": [
                [0.4886921905584123, -1.2217304763960306, 0.7853981633974483, 0.05235987755982989]
                ],
            "torso_speed_ratio": [1.],
            "head": [
                [-0.03490658503988659, -0.5585053606381855]
                ],
            "head_speed_ratio": [1.],
            "left_arm": [
                
                ],
            "left_arm_speed_ratio": [1.],
            "left_arm_ee": [
                
                ],
            "right_arm": [
                [325.011, 341.617, -18.004, -106.74, 0.823, 7.643]
                ],
            "right_arm_speed_ratio": [1.],
            "right_arm_ee": [
                [0, [0, 100, 10, 20, 22, 10], 0]
                ]
            }

task_dict7 = {
            "torso": [
                [0.4886921905584123, -1.2217304763960306, 0.7853981633974483, 0.05235987755982989]
                ],
            "torso_speed_ratio": [1.],
            "head": [
                [-0.03490658503988659, -0.5585053606381855]
                ],
            "head_speed_ratio": [1.],
            "left_arm": [
                
                ],
            "left_arm_speed_ratio": [1.],
            "left_arm_ee": [
                
                ],
            "right_arm": [
                [193.601, 324.418, 124.107, -111.746, -30.829, 31.658]
                ],
            "right_arm_speed_ratio": [1.],
            "right_arm_ee": [
                [0, [0, 0, 0, 0, 0, 0], 0]
                ]
            }

# 同步下拉前发给机械臂信息
print("同步下拉前发给机械臂信息")
task_dict8 = {
            "torso": [
                [0.4886921905584123, -1.0995574287564276, 0.7853981633974483, 0.05235987755982989]
                ],
            "torso_speed_ratio": [1.],
            "head": [
                [-0.03490658503988659, -0.5585053606381855]
                ],
            "head_speed_ratio": [1.],
            "left_arm": [
                [-39.81, 429.003, 34.033, -119.895, 3.891, -49.582]
                ],
            "left_arm_speed_ratio": [1.],
            "left_arm_ee": [
                [0, [0, 0, 0, 0, 0, 0], 0]
                ],
            "right_arm": [
                [37.87, 419.31, 42.01, -116.302, -10.129, 46.814]
                ],
            "right_arm_speed_ratio": [1.],
            "right_arm_ee": [
                [0, [0, 0, 0, 0, 0, 0], 0]
                ]
            }

# 下拉把手1&2发给机械臂信息
print("下拉把手1&2发给机械臂信息")
task_dict9 = {
            "torso": [
                [0.4886921905584123, -1.0995574287564276, 0.7853981633974483, 0.05235987755982989]
                ],
            "torso_speed_ratio": [1.],
            "head": [
                [-0.03490658503988659, -0.5585053606381855]
                ],
            "head_speed_ratio": [1.],
            "left_arm": [
                [-132.59, 305.766, 94.588, -142.213, 24.103, -69.309]
                ],
            "left_arm_speed_ratio": [1.],
            "left_arm_ee": [
                [0, [0, 0, 0, 0, 0, 0], 0]
                ],
            "right_arm": [
                [138.724, 311.453, 98.291, -138.746, -22.175, 62.797]
                ],
            "right_arm_speed_ratio": [1.],
            "right_arm_ee": [
                [0, [0, 0, 0, 0, 0, 0], 0]
                ]
            }
task_dict10 = {
            "torso": [
                [0.4886921905584123, -1.0995574287564276, 0.7853981633974483, 0.05235987755982989]
                ],
            "torso_speed_ratio": [1.],
            "head": [
                [-0.03490658503988659, -0.5585053606381855]
                ],
            "head_speed_ratio": [1.],
            "left_arm": [
                [-102.202, 282.495, 179.176, -170.858, 40.971, -108.044]
                ],
            "left_arm_speed_ratio": [1.],
            "left_arm_ee": [
                [0, [0, 0, 0, 0, 0, 0], 0]
                ],
            "right_arm": [
                [101.891, 280.535, 142.86, -163.046, -40.809, 98.539]
                ],
            "right_arm_speed_ratio": [1.],
            "right_arm_ee": [
                [0, [0, 0, 0, 0, 0, 0], 0]
                ]
            }

# 右手按按钮下发给机械臂信息
print("右手按按钮下发给机械臂信息")
task_dict11 = {
            "torso": [
                [0.4886921905584123, -1.0995574287564276, 0.7853981633974483, 0.05235987755982989]
                ],
            "torso_speed_ratio": [1.],
            "head": [
                [-0.03490658503988659, -0.5585053606381855]
                ],
            "head_speed_ratio": [1.],
            "left_arm": [

                ],
            "left_arm_speed_ratio": [1.],
            "left_arm_ee": [

                ],
            "right_arm": [
                [101.891, 280.535, 142.86, -163.046, -40.809, 98.539]
                ],
            "right_arm_speed_ratio": [1.],
            "right_arm_ee": [
                [0.2, [50, 0, 0, 0, 0, 0], 0.5]
                ]
            }

task_dict12 = {
            "torso": [
                [0.4886921905584123, -1.0995574287564276, 0.7853981633974483, 0.05235987755982989]
                ],
            "torso_speed_ratio": [1.],
            "head": [
                [-0.03490658503988659, -0.5585053606381855]
                ],
            "head_speed_ratio": [1.],
            "left_arm": [

                ],
            "left_arm_speed_ratio": [1.],
            "left_arm_ee": [

                ],
            "right_arm": [
                [101.891, 280.535, 142.86, -163.046, -40.809, 98.539]
                ],
            "right_arm_speed_ratio": [1.],
            "right_arm_ee": [
                [0, [0, 0, 0, 0, 0, 0], 0]
                ]
            }

# 拿出咖啡下发给机械臂信息
print("拿出咖啡下发给机械臂信息")
task_dict13 = {
            "torso": [
                [0.5585053606381855, -1.2566370614359172, 0.8726646259971648, 0.05235987755982989]
                ],
            "torso_speed_ratio": [1.],
            "head": [
                [-0.03490658503988659, -0.5585053606381855]
                ],
            "head_speed_ratio": [1.],
            "left_arm": [
                [-202.122, 234.686, 187.278, -119.082, 14.84, -21.557]
                ],
            "left_arm_speed_ratio": [1.],
            "left_arm_ee": [
                [0, [100, 0, 30, 30, 30, 30], 0]
                ],
            "right_arm": [
                [262.418, 279.216, 97.505, -120.266, -5.892, 9.426]
                ],
            "right_arm_speed_ratio": [1.],
            "right_arm_ee": [
                [0.5, [0, 100, 10, 20, 22, 10], 0]
                ]
            }

task_dict14 = {
            "torso": [
                [0.5585053606381855, -1.2566370614359172, 0.8726646259971648, 0.05235987755982989]
                ],
            "torso_speed_ratio": [1.],
            "head": [
                [-0.03490658503988659, -0.5585053606381855]
                ],
            "head_speed_ratio": [1.],
            "left_arm": [
                [-279.972, 275.673, 68.968, -120.428, -1.745, -8.446]
                ],
            "left_arm_speed_ratio": [1.],
            "left_arm_ee": [
                [0, [100, 0, 30, 30, 30, 30], 0]
                ],
            "right_arm": [
                [279.217, 322.297, -10.888, -119.861, 2.546, 9.628]
                ],
            "right_arm_speed_ratio": [1.],
            "right_arm_ee": [
                [0, [0, 100, 39, 47, 47, 37], 0.5]
                ]
            }

# 送出咖啡2下发给机械臂信息
print("送出咖啡2下发给机械臂信息")
task_dict15 = {
            "torso": [
                [0.5585053606381855, -1.2566370614359172, 0.8726646259971648, 0.05235987755982989]
                ],
            "torso_speed_ratio": [1.],
            "head": [
                [-0.03490658503988659, -0.5585053606381855]
                ],
            "head_speed_ratio": [1.],
            "left_arm": [
                [-160.667, 257.624, 259.856, -119.221, 44.998, -36.133]
                ],
            "left_arm_speed_ratio": [1.],
            "left_arm_ee": [
                [0, [0, 0, 0, 0, 0, 0], 0]
                ],
            "right_arm": [
                [237.549, 346.124, 149.834, -109.123, 0.177, 15.086]
                ],
            "right_arm_speed_ratio": [1.],
            "right_arm_ee": [
                [0, [0, 99, 33, 42, 44, 23], 0]
                ]
            }

task_dict16 = {
            "torso": [
                [0.5585053606381855, -1.2566370614359172, 0.8726646259971648, 0.05235987755982989]
                ],
            "torso_speed_ratio": [1.],
            "head": [
                [-0.03490658503988659, -0.5585053606381855]
                ],
            "head_speed_ratio": [1.],
            "left_arm": [
                [-160.667, 257.624, 259.856, -119.221, 44.998, -36.133]
                ],
            "left_arm_speed_ratio": [1.],
            "left_arm_ee": [
                [0, [0, 0, 0, 0, 0, 0], 0]
                ],
            "right_arm": [
                [289.553, 346.127, 149.833, -109.123, 0.177, 15.086]
                ],
            "right_arm_speed_ratio": [1.],
            "right_arm_ee": [
                [0, [0, 100, 0, 0, 0, 0], 0.5]
                ]
            }



def test_single_task(task_dict=None, load_env = True):
    if load_env:
        w1_manager.load_fcl_environment(f"{DATA_DIR}/environment/coffee_demo_downward.json")
        w1_manager.load_planner_environment(f"{DATA_DIR}/environment/expanded_coffee_demo_downward.json")
    w1_manager.move_dual_arm_single_task(task_dict=task_dict,
                                        speed_ratio=1)


if __name__ == "__main__":
    robot_type = RobotType.W1V02_1
    robot_ee_type = EndEffectorType.BRAINCO_HAND
    # robot_ee_type = None
    w1_manager = W1Manager(is_sim=True,
                           arm_type=W1ArmType.ANTHROPOMORPHIC,
                           w1_version=W1Version.V2,
                           end_effector_type=robot_ee_type)
    if not w1_manager.connect() and not w1_manager.is_enabled():
        raise RuntimeError("Failed to connect to W1 robot.")
    
    w1_manager.set_tcp("left_arm", np.eye(4))
    right_tcp_xpos = np.eye(4)
    right_tcp_xpos[:3, :3] = R.from_rotvec([0.0, 0.0, 180.], degrees=True).as_matrix()
    w1_manager.set_tcp("right_arm", xpos=right_tcp_xpos)


    # w1_manager.set_home_qpos(np.array([0, 0, 0, 0, 0.0, 0.0, 
    #                                   0.218, -0.718, 0.563, -2.083, 1.125, -0.501, 0.393, 
    #                                   -0.51, 0.851, -0.291, 1.821, 1.187, -0.104, 0.946]),
    #                         save_to_file=False)
    
    # w1_manager.set_ee_qpos("left_arm_ee", np.array([0.]*6))
    # w1_manager.set_ee_qpos("right_arm_ee", np.array([0.]*6))
    # w1_manager.go_home(1) 


    
    
    test_single_task(task_dict=task_dict1, load_env=False)
    input("Press Enter to continue to next test...")
    test_single_task(task_dict=task_dict2, load_env=True)
    input("Press Enter to continue to next test...")
    test_single_task(task_dict=task_dict3, load_env=True)
    input("Press Enter to continue to next test...")    
    test_single_task(task_dict=task_dict4, load_env=True)
    input("Press Enter to continue to next test...")    
    test_single_task(task_dict=task_dict5, load_env=True)
    input("Press Enter to continue to next test...")   
    test_single_task(task_dict=task_dict6, load_env=True)
    input("Press Enter to continue to next test...")
    test_single_task(task_dict=task_dict7, load_env=True)
    input("Press Enter to continue to next test...")
    test_single_task(task_dict=task_dict8, load_env=True)
    input("Press Enter to continue to next test...")
    test_single_task(task_dict=task_dict9, load_env=True)
    input("Press Enter to continue to next test...")
    test_single_task(task_dict=task_dict10, load_env=True)
    input("Press Enter to continue to next test...")
    test_single_task(task_dict=task_dict11, load_env=True)
    input("Press Enter to continue to next test...")
    test_single_task(task_dict=task_dict12, load_env=True)
    input("Press Enter to continue to next test...")
    test_single_task(task_dict=task_dict13, load_env=True)
    input("Press Enter to continue to next test...")
    test_single_task(task_dict=task_dict14, load_env=True)
    input("Press Enter to continue to next test...")
    test_single_task(task_dict=task_dict15, load_env=True)
    input("Press Enter to continue to next test...")
    test_single_task(task_dict=task_dict16, load_env=True)
    input("Press Enter to continue to next test...")
       


    embed()

    


