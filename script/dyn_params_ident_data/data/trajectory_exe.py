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
import numpy as np


def exe_in_real(is_left, traj_path, rec_path, hz):
    rclpy.init()
    client = RobotClient(is_sim=False)

    with open(traj_path, 'r' ) as file:
          q_ref = np.loadtxt(file)
    executor = MultiThreadedExecutor()
    executor.add_node(client)
    # 在后台启动 executor
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    rec = np.zeros((q_ref.shape[0], 1 + 7 + 7))
    rate = client.create_rate(hz)

    client.set_safe_level(1.0)
    if is_left:
        client.move_joint_sync(
            [0.5, -1.0, 0.5, 0.0, 0.0, 0.0, *q_ref[0, :], -0.5, 1.2, 0.0, 1.0, 0.0, 0.5, 0.5],
            MotionGroup.WholeBody,
            speed=0.25
        )
        client.start_joint_streaming_request(MotionGroup.Primary)
        time0 = client.timestamp
        for i in range(q_ref.shape[0] + 1):
            if i != q_ref.shape[0]:
                client.send_streaming_command(q_ref[i, :], streaming_finished=False)

            if i != 0:
                rec[i - 1, :] = np.concatenate([np.array([float(client.timestamp - time0)*1e-9]),
                                        np.array(client.joint_position[6:13]),
                                        np.array(client.joint_torque[6:13])])
            rate.sleep()
        client.send_streaming_command(q_ref[-1, :], streaming_finished=True)
    else:
        q_ref = -q_ref #右臂角度相反
        client.move_joint_sync(
            [0.5, -1.0, 0.5, 0.0, 0.0, 0.0, 0.5, -1.2, 0.0, -1.0, 0.0, -0.5, -0.5, *q_ref[0, :]],
            MotionGroup.WholeBody,
            speed=0.25
        )
        client.start_joint_streaming_request(MotionGroup.Secondary)
        time0 = client.timestamp
        for i in range(q_ref.shape[0] + 1):
            if i != q_ref.shape[0]:
                client.send_streaming_command(q_ref[i, :], streaming_finished=False)

            if i != 0:
                rec[i - 1, :] = np.concatenate([np.array([float(client.timestamp - time0)*1e-9]),
                                        np.array(client.joint_position[13:20]),
                                        np.array(client.joint_torque[13:20])])
            rate.sleep()
        client.send_streaming_command(q_ref[-1, :], streaming_finished=True)

    np.savetxt(rec_path, rec, fmt="%.6f")
    print(f"Saved {rec_path}")
    client.wait_request_finish()
    client.move_joint_sync(
        [0.5, -1.0, 0.5, 0.0, 0.0, 0.0, 0.5, -1.2, 0.0, -1.0, 0.0, -0.5, -0.5, -0.5, 1.2, 0.0, 1.0, 0.0, 0.5, 0.5],
        MotionGroup.WholeBody,
        speed=0.25
    )
    rclpy.shutdown()


def exe_in_sim(is_left, traj_path, hz):
    rclpy.init()
    client = RobotClient(is_sim=True)
    executor = MultiThreadedExecutor()
    executor.add_node(client)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    rate = client.create_rate(hz)

    with open(traj_path, 'r' ) as file:
          q_ref = np.loadtxt(file)

    if is_left:
        client.move_joint_sync(
            [0.5, -1.0, 0.5, 0.0, 0.0, 0.0, *q_ref[0, :], -0.5, 1.2, 0.0, 1.0, 0.0, 0.5, 0.5],
            MotionGroup.WholeBody
        )
        client.start_joint_streaming_request(MotionGroup.Primary)
        for i in range(q_ref.shape[0]):
            client.send_streaming_command(q_ref[i, :], streaming_finished=False)
            rate.sleep()
        client.send_streaming_command(q_ref[-1, :], streaming_finished=True)
    else:
        q_ref = -q_ref #右臂角度相反
        client.move_joint_sync(
            [0.5, -1.0, 0.5, 0.0, 0.0, 0.0, 0.5, -1.2, 0.0, -1.0, 0.0, -0.5, -0.5, *q_ref[0, :]],
            MotionGroup.WholeBody
        )
        client.start_joint_streaming_request(MotionGroup.Secondary)
        for i in range(q_ref.shape[0]):
            client.send_streaming_command(q_ref[i, :], streaming_finished=False)
            rate.sleep()
        client.send_streaming_command(q_ref[-1, :], streaming_finished=True)
    client.wait_request_finish()
    client.move_joint_sync(
        [0.5, -1.0, 0.5, 0.0, 0.0, 0.0, 0.5, -1.2, 0.0, -1.0, 0.0, -0.5, -0.5, -0.5, 1.2, 0.0, 1.0, 0.0, 0.5, 0.5],
        MotionGroup.WholeBody,
    )
    rclpy.shutdown()