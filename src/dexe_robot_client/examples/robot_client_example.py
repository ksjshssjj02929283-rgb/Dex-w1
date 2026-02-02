'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor

from dexe_robot_client.robot_client import RobotClientRos2 as RobotClient
from dexe_robot_client.robot_client import MotionGroup, RunMode, ReferenceFrame

joint_left0 = [-0.45, -0.65, 0.35, -1.5, -1, 0.5, 0.5]
joint_right0 = [0.45, 0.65, -0.35, 1.5, 1, -0.5, -0.5]

# pose采用四元数格式[x, y, z, qw, qx, qy, qz]
pose_left0 = [-0.331, 0.3854, 0.2922, 0.8022, -0.5669, -0.0925, 0.1626]
pose_right0 = [0.331, 0.3854, 0.2922, 0.1626, 0.0925, 0.5669, 0.8022]

# yapf: disable
target_traj = [
    -0.50, -0.65, 0.35, -1.5, -1, 0.5, 0.5, 0.50, 0.65, -0.35, 1.5, 1, -0.5, -0.5,
    -0.55, -0.65, 0.35, -1.5, -1, 0.5, 0.5, 0.55, 0.65, -0.35, 1.5, 1, -0.5, -0.5,
    -0.60, -0.65, 0.35, -1.5, -1, 0.5, 0.5, 0.60, 0.65, -0.35, 1.5, 1, -0.5, -0.5,
    -0.65, -0.65, 0.35, -1.5, -1, 0.5, 0.5, 0.65, 0.65, -0.35, 1.5, 1, -0.5, -0.5,
    -0.70, -0.65, 0.35, -1.5, -1, 0.5, 0.5, 0.70, 0.65, -0.35, 1.5, 1, -0.5, -0.5,
    -0.75, -0.65, 0.35, -1.5, -1, 0.5, 0.5, 0.75, 0.65, -0.35, 1.5, 1, -0.5, -0.5,
    -0.80, -0.65, 0.35, -1.5, -1, 0.5, 0.5, 0.80, 0.65, -0.35, 1.5, 1, -0.5, -0.5,
    -0.85, -0.65, 0.35, -1.5, -1, 0.5, 0.5, 0.85, 0.65, -0.35, 1.5, 1, -0.5, -0.5,
    -0.90, -0.65, 0.35, -1.5, -1, 0.5, 0.5, 0.90, 0.65, -0.35, 1.5, 1, -0.5, -0.5,
]
# yapf: enable


def main(is_sim=True):
    "示例: 仿真client, 如需控制真实机器人则将is_sim设置为False"
    rclpy.init()
    executor = MultiThreadedExecutor()
    client_sim = RobotClient(is_sim=True)
    if not client_sim.connect_robot():
        print(f"client_sim not connected!")
        if rclpy.ok():
            rclpy.shutdown()
        executor.shutdown()
        client_sim.close_client()
        client_sim.destroy_node()
        return
    executor.add_node(client_sim)
    client = client_sim

    client_real = None
    if not is_sim:
        client_real = RobotClient(is_sim=False)
        if not client_real.connect_robot():
            print(f"client_real not connected!")
            if rclpy.ok():
                rclpy.shutdown()
            executor.shutdown()
            client_sim.close_client()
            client_sim.destroy_node()
            client_real.close_client()
            client_real.destroy_node()
            return
        executor.add_node(client_real)
        client = client_real

    # 在后台启动 executor
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    print(f"[INFO] Robot type: {client.get_robot_type()}")
    print(f"[INFO] Robot arm type: {client.get_arm_type()}")
    print(f"[INFO] Robot version: {client.get_robot_version()}")
    print(f"[INFO] Current robot status: {client.get_info().status}")
    print(f"[INFO] Current joint position: {client.get_info().joint_position}")
    print(f"[INFO] Current ee_pose: {client.get_info().ee_pose}")

    # """设置全局速度"""
    client.set_global_speed(0.5)
    """move joint接口示例"""
    client.move_joint_sync(joint_left0 + joint_right0, MotionGroup.BothArms)
    """move traj接口示例"""
    endpoint_index = [2, 5, 7, 9]
    end_id = client.move_traj_async(target_traj,
                                    MotionGroup.BothArms,
                                    interval=0.5,
                                    endpoint_index=endpoint_index)
    for latest_finished_id in range(end_id - endpoint_index.__len__() + 1,
                                    end_id + 1):
        client.wait_request_finish(request_id=latest_finished_id)
        print(f"Finished traj segment id: {latest_finished_id}")
    """move_line接口示例"""
    ee_pose = client.get_info().ee_pose
    ee_pose[0] -= 0.15
    ee_pose[7] += 0.15
    client.move_line_sync(ee_pose, MotionGroup.BothArms)
    """joint streaming接口示例"""
    joint_position = client.get_info().joint_position
    client.start_joint_streaming_request(MotionGroup.WholeBody)
    for _ in range(100):
        joint_position[7] -= 0.003
        joint_position[8] += 0.003
        client.send_streaming_command(joint_position, streaming_finished=False)
        client.wait_second(0.01)
    client.send_streaming_command(joint_position, streaming_finished=True)
    """ee pose streaming示例"""
    ee_pose = client.get_info().ee_pose
    client.start_ee_pose_streaming_request(MotionGroup.BothArms)
    for _ in range(100):
        ee_pose[0] += 0.001
        ee_pose[7] -= 0.001
        client.send_streaming_command(ee_pose, streaming_finished=False)
        client.wait_second(0.01)
    client.send_streaming_command(ee_pose, streaming_finished=True)
    client.clear_error()  # 清除错误
    """ ee velocity streaming示例"""
    client.start_ee_velocity_streaming_request(
        MotionGroup.BothArms, reference_frame=ReferenceFrame.WorldBase)
    for _ in range(100):
        ee_velocity = [0, 0, 0.05, 0, 0, 0, 0.05, 0, 0, 0, 0, 0]
        client.send_streaming_command(ee_velocity, streaming_finished=False)
        client.wait_second(0.01)
    client.send_streaming_command(ee_velocity, streaming_finished=True)
    """其他接口示例"""
    client.stop_move()  # 停止运动且清空当前队列中的运动命令
    client.set_tcp([0, 0, 0.0, 1, 0, 0, 0])  # 设置工具坐标系
    client.clear_error()  # 清除错误
    # client.set_safe_level(0.2) # 设置安全级别 范围[0.1, 1], 值越大机械臂刚度越大, 开机默认为1.0, 仅真实机器人生效
    # client.set_drag_mode(True) # 设置拖拽模式, 开启后可以拖拽关节到任意位置并保持姿态, 仅真实机器人生效
    # client.switch_run_mode(RunMode.Torque) # 切换运行模式为力控模式 RunMode.Position: 位置模式, RunMode.Torque: 力矩模式,
    # RunMode.Drag: 拖动模式, RunMode.CartesianImpedance: 笛卡尔阻抗模式
    # client.set_joint_stiffness(200) # 设置全部关节刚度
    # client.set_joint_stiffness([50, 50, 10, 10, 10, 10, 10, 50, 50, 10, 10, 10, 10, 10]) # 分别设置关节刚度
    # client.set_joint_damping(10.0) # 设置全部关节阻尼
    # client.set_joint_damping([5.0, 5.0, 1.0, 1.0, 1.0, 1.0, 1.0, 5.0, 5.0, 1.0, 1.0, 1.0, 1.0, 1.0]) # 分别设置关节阻尼
    # client.set_cartesian_stiffness(200) # 设置双臂笛卡尔刚度(x, y, z, rx, ry, rz), 长度为1
    # client.set_cartesian_stiffness([600, 600, 600, 200, 200, 200, 100, 100, 100, 200, 200, 200]) # 分别设置左右臂笛卡尔刚度 长度为12
    # client.set_cartesian_damping_ratio(1.2) # 设置双臂笛卡尔阻尼比(x, y, z, rx, ry, rz)
    # client.set_cartesian_damping_ratio([1.2, 1.2, 1.2, 0.9, 0.9, 0.9, 1.2, 1.2, 1.2, 0.9, 0.9, 0.9]) # 分别设置左右臂笛卡尔阻尼比, 范围0.7~1.3, 长度为12
    client.wait_request_finish()  # 等待已发送的运动请求完成

    # 安全退出：先关闭 executor，再释放资源
    if rclpy.ok():
        rclpy.shutdown()
    ros_thread.join()
    executor.shutdown()
    client_sim.close_client()
    client_sim.destroy_node()
    if client_real is not None:
        client_real.close_client()
        client_real.destroy_node()


if __name__ == "__main__":
    main(is_sim=True)
