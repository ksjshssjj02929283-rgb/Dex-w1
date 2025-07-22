# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import time
import numpy as np
from dexe_robot_client.common import get_logger, timer
from dexe_robot_client.robot_client import RobotClientRos2 as RobotClient
from dexe_robot_client.robot_client import MotionGroup
from IPython import embed

joint_industrial_left0 = [-0.45, -0.65, 0.35, -1.5, -1, 1.1, -0.4]
joint_industrial_right0 = [0.45, 0.65, -0.35, 1.5, 1, -1.1, 0.4]

# pose采用四元数格式[x, y, z, qw, qx, qy, qz]
pose_industrial_left0 = [
    -0.3,
    0.363727,
    0.262795,
    0.628296,
    -0.599072,
    -0.440151,
    0.229399,
]
pose_industrial_right0 = [
    0.3,
    0.363727,
    0.262795,
    0.229399,
    0.440151,
    0.599072,
    0.628296,
]


def main(args=None):
    client = RobotClient()
    status = client.get_info().status
    joint_position = client.get_info().joint_position
    ee_pose = client.get_info().ee_pose

    print(f"\nCurrent robot status: {status}")
    print(f"\nCurrent joint position: {joint_position}")
    print(f"\nCurrent ee_pose: {ee_pose}\n")

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
    for _ in range(5):
        joint_position[5] -= 0.1
        joint_position[8] += 0.1
        client.send_streaming_command(joint_position, streaming_finished=False)
        client.wait_second(0.1)
    client.send_streaming_command(joint_position, streaming_finished=True)

    """ee pose streaming示例"""
    ee_pose = client.get_info().ee_pose
    client.start_ee_pose_streaming_request(MotionGroup.BothArms)
    for _ in range(20):
        ee_pose[0] += 0.01
        ee_pose[7] -= 0.01
        client.send_streaming_command(ee_pose, streaming_finished=False)
        client.wait_second(0.1)
    client.send_streaming_command(ee_pose, streaming_finished=True)

    """其他接口示例"""
    client.set_tcp([0, 0, 0.0, 1, 0, 0, 0])
    client.clear_error()
    client.wait_request_finish()
    client.close_client()


if __name__ == "__main__":
    main()
