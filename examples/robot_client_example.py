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

joint_industrial_left0 = [-0.45, -0.65, 0.35, -1.5, -1, 1.1, -0.4]
joint_industrial_right0 = [0.45, 0.65, -0.35, 1.5, 1, -1.1, 0.4]

# pose采用四元数格式[x, y, z, qw, qx, qy, qz]
pose_industrial_left0 = [-0.3, 0.363727, 0.262795, -0.229399, -0.440151, 0.599072, 0.628296]
pose_industrial_right0 = [0.3, 0.363727, 0.262795, 0.628296, -0.599072, 0.440151, -0.229399]

def main(is_sim = True):
    "示例: 仿真client, 如需控制真实机器人则将is_sim设置为False"
    rclpy.init()
    executor = MultiThreadedExecutor()
    client_sim = RobotClient(is_sim=True)
    if not is_sim:
        client_real = RobotClient(is_sim=False)
        client = client_real
    else:
        client = client_sim
    executor.add_node(client_sim)
    executor.add_node(client_real)
    # 在后台启动 executor
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    print(f"\nclient_sim is connnected: {client_sim.is_connected()}")
    print(f"\nclient_real is connnected: {client_real.is_connected()}")
    print(f"\nCurrent robot status: {client.get_info().status}")
    print(f"\nCurrent joint position: {client.get_info().joint_position}")
    print(f"\nCurrent ee_pose: {client.get_info().ee_pose}\n")

    client.set_global_speed(0.5)
    """move joint/line接口示例"""
    client.move_joint_sync(
        joint_industrial_left0 + joint_industrial_right0, MotionGroup.BothArms
    )
    client.move_line_sync(
        pose_industrial_left0 + pose_industrial_right0, MotionGroup.BothArms
    )

    """joint streaming接口示例"""
    joint_position = client.get_info().joint_position
    client.start_joint_streaming_request(MotionGroup.WholeBody)
    for _ in range(50):
        joint_position[8] -= 0.01
        client.send_streaming_command(joint_position, streaming_finished=False)
        client.wait_second(0.01)
    client.send_streaming_command(joint_position, streaming_finished=True)

    """ee pose streaming示例"""
    ee_pose = client.get_info().ee_pose
    client.start_ee_pose_streaming_request(MotionGroup.BothArms)
    for _ in range(50):
        ee_pose[0] -= 0.001
        ee_pose[7] += 0.001
        client.send_streaming_command(ee_pose, streaming_finished=False)
        client.wait_second(0.01)
    client.send_streaming_command(ee_pose, streaming_finished=True)

    """其他接口示例"""
    client.set_tcp([0, 0, 0.0, 1, 0, 0, 0])
    client.clear_error()
    client.wait_request_finish()

    print(f"\nclient_sim is connnected: {client_sim.is_connected()}")
    print(f"\nclient_real is connnected: {client_real.is_connected()}")

    # 安全退出：先关闭 executor，再释放资源
    executor.shutdown()
    client.close_client()
    client.destroy_node()
    ros_thread.join()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main(is_sim=True)
