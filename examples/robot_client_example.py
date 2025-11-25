# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor

from dexe_robot_client.robot_client import RobotClientRos2 as RobotClient
from dexe_robot_client.robot_client import MotionGroup

joint_industrial_left0 = [-0.45, -0.65, 0.35, -1.5, -1, 0.5, 0.5]
joint_industrial_right0 = [0.45, 0.65, -0.35, 1.5, 1, -0.5, -0.5]

# pose采用四元数格式[x, y, z, qw, qx, qy, qz]
pose_industrial_left0 = [
    -0.331, 0.3854, 0.2922, 0.8022, -0.5669, -0.0925, 0.1626
]
pose_industrial_right0 = [0.331, 0.3854, 0.2922, 0.1626, 0.0925, 0.5669, 0.8022]


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

    print(f"Current robot status: {client.get_info().status}")
    print(f"Current joint position: {client.get_info().joint_position}")
    print(f"Current ee_pose: {client.get_info().ee_pose}")
    """设置全局速度"""
    client.set_global_speed(0.5)
    """move joint/line接口示例"""
    client.move_joint_sync(joint_industrial_left0 + joint_industrial_right0,
                           MotionGroup.BothArms)
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
    """其他接口示例"""
    client.set_tcp([0, 0, 0.0, 1, 0, 0, 0])  # 设置工具坐标系
    client.clear_error()  # 清除错误
    client.wait_request_finish()  # 等待已发送的运动请求完成
    # client.set_safe_level(0.2) # 设置安全级别 范围[0.1, 1], 值越大机械臂刚度越大, 开机默认为1.0, 仅真实机器人生效
    # client.set_drag_mode(True) # 设置拖拽模式, 开启后可以拖拽关节到任意位置并保持姿态, 仅真实机器人生效

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
