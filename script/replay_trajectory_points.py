#!/usr/bin/env python3
"""仅对选出的采样点逐点 MoveJoint，到位静止后保存实测 DAT。

python3 script/replay_trajectory_points.py script/pos_err_test/targets.dat --arm left
python3 script/replay_trajectory_points.py script/pos_err_test/targets.dat --arm left --real --execute

默认只检查文件/限位/FK并打印起点，不连接 ROS。--execute 默认仿真，--real 为真机。
不重新计算碰撞；只向所选手臂发送七关节 MoveJoint，左臂 Primary、右臂 Secondary。
不向躯干或另一臂下发位置目标，不要求它们移动到预设待机姿态。
只使用 MoveRobot action；不发布 joint_position，不执行原轨迹中间行。
采样点之间直接 MoveJoint 插值，不修改服务器滤波、Ruckig、保护或使能设置。
"""

import argparse
from datetime import datetime
import json
from pathlib import Path
import sys
import time

import numpy as np

from sample_trajectory_points import OUTPUT_DIR, NAMES, load_targets, vector, write_dat


RESULT_MAGIC = "# trajectory_fk_measurements_v1 "


def check_start(current, expected, tolerance=0.0025):
    if np.max(np.abs(current - expected)) > tolerance:
        raise RuntimeError("MoveJoint 后所选手臂未到达采样目标，拒绝记录")


class Client:
    def __init__(self, real, robot, side):
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.signals import SignalHandlerOptions
        from robot_server_interfaces.action import MoveRobot
        from std_msgs.msg import String
        self.ros, self.MoveRobot = rclpy, MoveRobot
        self.robot = robot
        self.active = slice(6, 13) if side == "left" else slice(13, 20)
        self.motion_group = {"left": "Primary", "right": "Secondary"}[side]
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node("replay_trajectory_points")
        suffix = "" if real else "_sim"
        self.stop_client = ActionClient(self.node, MoveRobot, "/control/move_robot" + suffix)
        self.subscription = self.node.create_subscription(String, "/feedback/robot_server_state" + suffix, self.on_state, 1)
        self.state, self.received, self.seq, self.error = None, 0., 0, None
        self.sent = False

    def on_state(self, message):
        try:
            state = json.loads(message.data)
            stamp = state["timestamp"]
            if type(stamp) not in (int, float) or not np.isfinite(stamp):
                raise ValueError("反馈 timestamp 无效")
            if self.state is not None:
                if stamp == self.state["timestamp"]:
                    return
                if stamp < self.state["timestamp"]:
                    raise ValueError("反馈 timestamp 倒退")
            self.state, self.received = state, time.monotonic()
            self.seq += 1
        except (ValueError, KeyError, TypeError) as exc:
            self.error = str(exc)

    def tick(self, timeout=0.005):
        if not self.ros.ok():
            raise RuntimeError("ROS 已关闭")
        self.ros.spin_once(self.node, timeout_sec=timeout)
        if self.error:
            raise RuntimeError(self.error)

    def fresh(self):
        return (self.state is not None and time.monotonic() - self.received <= 0.5 and
                abs(self.node.get_clock().now().nanoseconds - self.state["timestamp"]) <= 500_000_000)

    def ordered(self, field):
        names = self.state["joint_name"]
        values = vector(self.state[field], len(names))
        if len(names) != len(set(names)) or not set(NAMES).issubset(names):
            raise RuntimeError("反馈缺少完整 W1 关节或关节名重复")
        mapping = dict(zip(names, values))
        return np.array([mapping[n] for n in NAMES])

    def healthy(self):
        if not self.fresh():
            raise RuntimeError("反馈超时或服务器时钟不同步")
        state = self.state
        if state.get("robot_type") != "w1" or not str(state.get("robot_version", "")).startswith("v02"):
            raise RuntimeError("服务器必须为 w1 v02 系列")
        if state.get("run_mode") != "Position" or state.get("status") not in ("Idle", "Running"):
            raise RuntimeError("需要 Position 模式且状态正常")
        for key in ("server_error_code", "motor_error_code"):
            codes = state.get(key, [])
            if not isinstance(codes, list):
                raise RuntimeError(f"错误码格式无效: {key}")
            for code in codes:
                clear = (type(code) in (int, float) and code == 0) or (isinstance(code, str) and
                        (code.strip() == "0" or (key == "server_error_code" and code.strip() == "None")))
                if not clear:
                    raise RuntimeError(f"服务器错误: {key}={code!r}")
        vector(state["ee_pose"], 14)
        return self.robot.validate(self.ordered("joint_position"))

    def wait_idle(self, expected=None, timeout=10.):
        deadline, since, last_seq = time.monotonic() + timeout, None, self.seq
        while time.monotonic() < deadline:
            self.tick()
            if self.state is None:
                continue
            q = self.healthy()
            if self.state["status"] != "Idle" or self.state.get("latest_queued_id", -1) > max(
                    self.state.get("latest_finished_id", -1), self.state.get("latest_errordrop_id", -1)):
                if expected is None:
                    raise RuntimeError("服务器不空闲，拒绝与其他运动指令同时执行")
                since = None
                continue
            if self.seq == last_seq:
                continue
            last_seq = self.seq
            good = np.max(np.abs(self.ordered("joint_velocity"))) <= 0.005
            if expected is not None:
                good = good and np.max(np.abs(q[self.active] - expected)) <= 0.0025
                if "joint_position_cmd" in self.state:
                    good = good and np.max(np.abs(self.ordered("joint_position_cmd")[self.active] - expected)) <= 0.0025
            since = (since or time.monotonic()) if good else None
            if since is not None and time.monotonic() - since >= 0.5:
                return q.copy()
        raise RuntimeError("等待到位/静止超时")

    def wait_action(self, future, deadline):
        while not future.done():
            self.tick()
            self.healthy()
            if time.monotonic() >= deadline:
                raise RuntimeError("采样点 MoveJoint 超时")
        self.healthy()
        result = future.result()
        if result is None:
            raise RuntimeError("采样点 MoveJoint 返回空结果")
        return result

    def move_to(self, expected, speed=0.05, acceleration=0.05):
        expected = vector(expected, 7)
        current = self.wait_idle()
        target = current.copy()
        target[self.active] = expected
        self.robot.validate(target)
        command = self.ordered("joint_position_cmd") if "joint_position_cmd" in self.state else current
        if max(np.max(np.abs(current[self.active] - expected)), np.max(np.abs(command[self.active] - expected))) <= 0.0025:
            print("已在采样目标且静止，无需重复运动。", flush=True)
            return
        deadline = time.monotonic() + 5.
        while not self.stop_client.wait_for_server(timeout_sec=0.1):
            self.tick()
            self.healthy()
            if time.monotonic() >= deadline:
                raise RuntimeError("MoveRobot action 不可用")
        # Recheck idle after discovery; do not queue behind another controller.
        self.wait_idle()
        goal = self.MoveRobot.Goal()
        goal.type, goal.motion_group = "MoveJoint", self.motion_group
        # Primary/Secondary targets are J1..J7, independent of feedback array order.
        goal.target = expected.tolist()
        goal.config = json.dumps({"speed": speed, "acceleration": acceleration})
        print(f"{self.motion_group} MoveJoint 到采样点（speed={speed}, acceleration={acceleration}，未检查碰撞）…", flush=True)
        self.sent = True  # Also stop on timeout while goal acceptance is pending.
        deadline = time.monotonic() + 120.
        handle = self.wait_action(self.stop_client.send_goal_async(goal), deadline)
        if not handle.accepted:
            raise RuntimeError("采样点 MoveJoint 被拒绝")
        result = self.wait_action(handle.get_result_async(), deadline)
        if result.status != 4 or not result.result.success:
            raise RuntimeError("采样点 MoveJoint 执行失败")
        arrived = self.wait_idle(expected, timeout=30.)
        check_start(arrived[self.active], expected)
        print("MoveJoint 已到位并静止。", flush=True)

    def stop(self):
        if not self.sent:
            return
        goal = self.MoveRobot.Goal()
        goal.type, goal.motion_group, goal.config = "StopMove", self.motion_group, ""
        future = self.stop_client.send_goal_async(goal)
        self.ros.spin_until_future_complete(self.node, future, timeout_sec=2.)
        if not future.done() or not future.result().accepted:
            raise RuntimeError("StopMove 未确认接收，请现场检查")
        result = future.result().get_result_async()
        self.ros.spin_until_future_complete(self.node, result, timeout_sec=2.)
        if not result.done() or not result.result().result.success:
            raise RuntimeError("StopMove 未确认完成，请现场检查")

    def close(self):
        self.stop_client.destroy()
        self.node.destroy_node()
        if self.ros.ok():
            self.ros.shutdown()


def execute(args, meta, trajectories, robot):
    side, rows = args.arm, trajectories[args.arm]
    active = slice(6, 13) if side == "left" else slice(13, 20)
    group = {"left": "Primary", "right": "Secondary"}[side]
    print(f"{side} 首个采样目标（rad，仅该臂）: " + json.dumps(dict(zip(NAMES[active], rows[0].tolist())), ensure_ascii=False))
    if not args.execute:
        print(f"文件/限位/FK 检查通过：仅执行 {meta['count_per_arm']} 个采样点，逐点 MoveJoint；不执行其余轨迹行。未连接 ROS，未检查碰撞。")
        return 0
    output = args.output or OUTPUT_DIR / ("fk_measured_" + side + "_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".dat")
    info = {"status": "initializing", "source_dat": str(args.dat.resolve()), "arm": side,
            "real": args.real, "collected_points": 0, "joint_unit": "rad",
            "position_unit": "m", "pose_order": "x,y,z,qw,qx,qy,qz", "fk_frame": side + "_arm_base",
            "tcp": meta["tcp"][side], "collision_checked": False,
            "motion": {"type": "MoveJoint", "motion_group": group, "joint_names": NAMES[active], "speed": args.speed,
                       "acceleration": args.acceleration, "sampled_points_only": True}}
    columns = ["index", "source_row", "timestamp_ns"]
    for field in ("q_command", "q_reached", "pose_desired", "pose_reached", "pose_reached_fk"):
        columns += [f"{field}_{i}" for i in range(7)]
    records = []
    write_dat(output, RESULT_MAGIC, info, columns, records)
    client = None
    try:
        client = Client(args.real, robot, side)
        info["status"] = "collecting"
        selected = meta["sample_indices"][side]
        for i in selected:
            arm_target = rows[i]
            client.move_to(arm_target, args.speed, args.acceleration)
            measured = client.healthy()
            check_start(measured[active], arm_target)
            # Non-commanded joints come from feedback, never preset rest angles.
            target = measured.copy()
            target[active] = arm_target
            ee_offset = 0 if side == "left" else 7
            records.append([len(records) + 1, i + 1, client.state["timestamp"]] +
                target[active].tolist() + measured[active].tolist() +
                robot.fk(target, side, meta["tcp"][side]) +
                client.state["ee_pose"][ee_offset:ee_offset + 7] +
                robot.fk(measured, side, meta["tcp"][side]))
            info["collected_points"] = len(records)
            write_dat(output, RESULT_MAGIC, info, columns, records, replace=True)
            print(f"{side} 已停稳采集 {len(records)}/{len(selected)}，原轨迹行 {i + 1}", flush=True)
        client.stop()
        info["status"] = "complete"
        print(f"采集完成: {output}")
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        info.update(status="aborted", error=str(exc) or type(exc).__name__)
        print(f"中止: {info['error']}；保留 {len(records)} 点: {output}", file=sys.stderr)
        if client is not None:
            try:
                client.stop()
            except Exception as stop_error:
                info["stop_error"] = str(stop_error)
                print(f"停止未确认: {stop_error}，请检查现场急停", file=sys.stderr)
        return 1
    finally:
        try:
            write_dat(output, RESULT_MAGIC, info, columns, records, replace=True)
        finally:
            if client is not None:
                client.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("dat", type=Path)
    parser.add_argument("--arm", choices=("left", "right"), required=True)
    parser.add_argument("--real", action="store_true")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--speed", type=float, default=0.05, help="MoveJoint speed，默认 0.05，最大 0.2")
    parser.add_argument("--acceleration", type=float, default=0.05, help="MoveJoint acceleration，默认 0.05，最大 0.2")
    parser.add_argument("--output", type=Path, help="本次单臂实测 DAT，不覆盖已有文件")
    args = parser.parse_args(argv)
    try:
        for name in ("speed", "acceleration"):
            value = getattr(args, name)
            if not np.isfinite(value) or not 0 < value <= 0.2:
                raise ValueError(f"{name} 必须大于 0 且不超过 0.2")
        return execute(args, *load_targets(args.dat, expected_arm=args.arm))
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(f"错误: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
