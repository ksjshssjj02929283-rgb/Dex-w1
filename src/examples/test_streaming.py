import time

from dexe_robot_client.common import StatusCode
from dexe_robot_client.robot_client import MotionGroup
from dexe_robot_client.w1 import W1Manager
from dexe_rp import RobotVersion, EndEffectorType
import numpy as np


if __name__ == "__main__":
    w1_manager = W1Manager(is_sim=True,
                           robot_version=RobotVersion.W1V024,
                           left_ee_type=EndEffectorType.Linker_L6,
                           right_ee_type=EndEffectorType.Linker_L6)

    home_qpos = np.array([
        0.785, -1.57, 0.9, 0.0,
        0.0, -0.4,
        -1.57, -0.785, 1.57, -0.785, 0.0, 0.0, 0.0,
        1.57, 0.785, -1.57, 0.785, 0.0, 0.0, 0.0
    ])
    w1_manager.set_home_qpos(home_qpos)
    if w1_manager.go_home(0.5) != StatusCode.SUCCESS:
        raise RuntimeError("Go home failed")

    # res, left_ee_qpos = w1_manager.get_current_qpos("left_arm_ee")
    # if res != StatusCode.SUCCESS:
    #     raise RuntimeError(f"Get left end-effector qpos failed: {res}")
    # res, right_ee_qpos = w1_manager.get_current_qpos("right_arm_ee")
    # if res != StatusCode.SUCCESS:
    #     raise RuntimeError(f"Get right end-effector qpos failed: {res}")
    left_ee_qpos = np.array([[0.0]*6])
    right_ee_qpos = np.array([[0.0]*6])

    task_dict = {
        "left_arm": [
            [-1.57, -0.785, 1.57, 0.0, 0.0, 0.0, 0.0]
        ],
        "left_arm_ee": [],
        "left_arm_speed_ratio": [0.1],
    }

    res, traj, _, _ = w1_manager.plan_joint_full_body(
        start_qpos=home_qpos,
        start_left_arm_ee_qpos=left_ee_qpos,
        start_right_arm_ee_qpos=right_ee_qpos,
        waypoints_dict=task_dict,
        use_collision_validator=False
    )

    if res != StatusCode.SUCCESS or traj is None or len(traj) == 0:
        raise RuntimeError(f"Plan joint full-body trajectory failed: {res}")

    client = w1_manager.robot_client
    # traj 已按 TIME_STEP=0.01s 采样，仅下发全身关节位置。
    interval = 0.01
    request_id = client.start_joint_streaming_request(
        MotionGroup.WholeBody, use_velocity=False, use_acceleration=False
    )
    if request_id < 0:
        raise RuntimeError("Start joint streaming failed")

    last_command = traj[0].tolist()
    next_send_time = time.monotonic()
    try:
        for qpos in traj:
            time.sleep(max(0.0, next_send_time - time.monotonic()))
            command = qpos.tolist()
            if not client.send_streaming_command(command, streaming_finished=False):
                raise RuntimeError("Send streaming point failed")
            last_command = command
            next_send_time += interval

        # 最后一个轨迹点保持一个周期，再发送结束指令。
        time.sleep(max(0.0, next_send_time - time.monotonic()))
    finally:
        # 正常结束、异常或 Ctrl+C 均尝试关闭 streaming，保持最后下发的位置。
        if not client.send_streaming_command(last_command, streaming_finished=True):
            raise RuntimeError("Finish joint streaming failed")

    if not client.wait_request_finish(request_id=request_id):
        raise RuntimeError("Wait for joint streaming to finish failed")
