#!/usr/bin/env python3
"""逐行发送单臂七关节轨迹，固定 w1_v02，默认 traj2.dat、1x、/control/joint_position。

示例（先 source ROS 2 和项目工作空间的 setup.bash）：
  python3 script/stream_arm_trajectory.py --arm left --dry-run
  python3 script/stream_arm_trajectory.py --arm left --output-dir /tmp/capture
  python3 script/stream_arm_trajectory.py --arm right --speed 0.5
  python3 script/stream_arm_trajectory.py --arm left --trajectory /path/to/traj2.dat

先从反馈位置插值移动到第一帧，等待 1 秒后完整播放轨迹，不裁剪、不自动回位。
只记录正式轨迹，输出 <output-dir>/<arm>/rbtlog_<轨迹名>.dat，不覆盖已有文件。
每行 22 列：时间(s)、位置(rad)×7、电流估算力矩(Nm)×7、传感器力矩(Nm)×7。
时间使用反馈 timestamp 相对正式轨迹开始时的差值；与 Qt 一样，每发一点记录最新反馈。
与 Qt 一样，ROS 回调在独立线程运行，采集数组预分配，发送结束后统一写盘。
发送按单调时钟累加目标时刻；落后时跳过等待追赶，不重置时间基准。
Ctrl+C 停止后续发布；服务端通过自己的 streaming 超时机制结束流式任务。
正常结束或 Ctrl+C 时保存已采集行；强制终止进程会丢失内存中尚未保存的数据。
"""

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import threading
import time

import numpy as np


def load_trajectory(path):
    q = np.loadtxt(path, ndmin=2)
    if q.shape[0] == 0 or q.shape[1] != 7:
        raise ValueError(f"轨迹必须为非空的 N×7 位置数据，实际形状为 {q.shape}")
    if not np.isfinite(q).all():
        raise ValueError("轨迹包含 NaN 或 Inf")
    return q


def start_points(current, target, speed=0.5, period=0.01):
    intervals = max(1, math.ceil(float(np.max(np.abs(target - current))) / speed / period))
    return np.linspace(current, target, intervals + 1)


class ArmFeedback:
    def __init__(self, names):
        self.names = names
        self.state = None
        self.received_at = None
        self.timestamp = None
        self.torque = np.zeros(7)
        self.sensor_torque = np.zeros(7)
        self._lock = threading.Lock()

    def update(self, message):
        try:
            state = json.loads(message.data)
            mapping = dict(zip(state["joint_name"], state["joint_position"]))
            q = np.asarray([mapping[name] for name in self.names], dtype=float)
            status = state["status"]
            stamp = state["timestamp"]
            torques = []
            for key in ("joint_torque", "joint_torque_sensor"):
                values = dict(zip(state["joint_name"], state.get(key, [])))
                torques.append(np.asarray([values.get(name, 0.0) for name in self.names], dtype=float))
            if (q.shape != (7,) or not np.isfinite(q).all()
                    or not np.isfinite(stamp) or not np.isfinite(torques).all()):
                return
        except (ValueError, KeyError, TypeError):
            return
        with self._lock:
            self.state = (status, q)
            self.timestamp = stamp
            self.torque, self.sensor_torque = torques
            self.received_at = time.monotonic()

    def snapshot(self):
        # 回调仅替换数组，快照引用在解锁后仍属于同一条反馈。
        with self._lock:
            if self.state is None or time.monotonic() - self.received_at > 0.5:
                raise RuntimeError("所选手臂反馈缺失或超过 0.5 秒未更新")
            status, q = self.state
            sample = (status, self.timestamp, q, self.torque, self.sensor_torque)
        if status not in ("Idle", "Running"):
            raise RuntimeError(f"机器人状态为 {status}，停止发送")
        return sample

    def position(self):
        return self.snapshot()[2].copy()


class QtRecorder:
    def __init__(self, file_handle, feedback, capacity):
        self.file_handle = file_handle
        self.feedback = feedback
        self.start_timestamp = feedback.snapshot()[1]
        self.data = np.zeros((capacity, 22))
        self.count = 0

    def record(self):
        _, timestamp, q, torque, sensor_torque = self.feedback.snapshot()
        row = self.data[self.count]
        row[0] = (timestamp - self.start_timestamp) * 1e-9
        row[1:8] = q
        row[8:15] = torque
        row[15:22] = sensor_torque
        self.count += 1

    def save(self):
        np.savetxt(self.file_handle, self.data[:self.count], fmt="%.6f")


def sleep_until(target_time, ros):
    """沿用 Qt 的单调时钟等待，每次 sleep 最多 1 ms。"""
    while ros.ok():
        remaining = target_time - time.monotonic()
        if remaining <= 0:
            break
        time.sleep(min(remaining, 0.001))


def publish_trajectory(node, publisher, message_type, q, names, period, ros, check=None, on_point=None):
    """沿用 Qt 的固定目标时刻；回调和磁盘写入不在发送循环内执行。"""
    next_time = time.monotonic()
    for row in q:
        if not ros.ok():
            return False
        if check is not None:
            check()
        msg = message_type()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = list(names)
        msg.position = row.tolist()
        publisher.publish(msg)
        if on_point is not None:
            on_point()

        next_time += period
        sleep_until(next_time, ros)
    return ros.ok()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--arm", required=True, choices=["left", "right"], help="发送目标的手臂")
    parser.add_argument("--trajectory", type=Path, help="自定义 N×7 DAT 文件，覆盖默认轨迹路径")
    parser.add_argument("--speed", type=float, choices=[1.0, 0.5], default=1.0, help="倍速（默认 1）")
    parser.add_argument("--topic", default="/control/joint_position", help="位置命令话题")
    parser.add_argument("--feedback-topic", default="/feedback/robot_server_state", help="机器人反馈话题")
    parser.add_argument("--start-speed", type=float, default=0.5, help="进入起点的插值速度，rad/s（默认 0.5）")
    parser.add_argument("--output-dir", type=Path, help="采集输出根目录（默认 recdata/stream_<时间>）")
    parser.add_argument("--dry-run", action="store_true", help="仅检查文件并打印信息，不初始化 ROS、不发送")
    args = parser.parse_args()
    if not math.isfinite(args.start_speed) or args.start_speed <= 0:
        parser.error("--start-speed 必须是有限正数")
    path = args.trajectory or (
        Path(__file__).resolve().parent / "dyn_params_ident" / "trajectories"
        / "w1_v02" / args.arm / "traj2.dat"
    )
    path = path.expanduser().resolve()
    output_dir = args.output_dir or (
        Path(__file__).resolve().parent / "dyn_params_ident" / "recdata"
        / ("stream_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    )
    output_path = output_dir.expanduser().resolve() / args.arm / f"rbtlog_{path.stem}.dat"
    try:
        q = load_trajectory(path)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))

    names = [f"{args.arm.upper()}_J{i}" for i in range(1, 8)]
    period = 0.01 / args.speed
    print(f"轨迹：{path}\n关节：{', '.join(names)}\n话题：{args.topic}", flush=True)
    print(f"点数：{len(q)}，间隔：{period:g} s，预计发送耗时：{len(q) * period:g} s", flush=True)
    print(f"第一帧（rad）：{q[0].tolist()}", flush=True)
    print(f"播放前自动进入起点，插值速度 {args.start_speed:g} rad/s；起点过渡耗时另计。", flush=True)
    print(f"采集文件：{output_path}（Qt 兼容 22 列）", flush=True)
    if args.dry_run:
        return 0

    # 延迟导入，使 --help / --dry-run 不依赖 ROS 环境。
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from joint_interfaces.msg import JointPositionControl
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("stream_arm_trajectory")
    record_file = None
    recorder = None
    executor = MultiThreadedExecutor()
    ros_thread = None
    try:
        publisher = node.create_publisher(JointPositionControl, args.topic, 10)
        feedback = ArmFeedback(names)
        subscription = node.create_subscription(String, args.feedback_topic, feedback.update, 1)
        executor.add_node(node)
        ros_thread = threading.Thread(target=executor.spin, daemon=True)
        ros_thread.start()
        deadline = time.monotonic() + 5.0
        while publisher.get_subscription_count() == 0 or feedback.state is None:
            if not rclpy.ok() or time.monotonic() >= deadline:
                raise RuntimeError("等待位置话题订阅者/手臂反馈超时，请检查 server 和 ROS 环境")
            time.sleep(0.01)
        status, _, current, _, _ = feedback.snapshot()
        if status != "Idle":
            raise RuntimeError("机器人正在运动，请在空闲状态启动脚本")
        # 运动前检查输出路径可写；发送期间只写内存，退出时保存已采集行。
        output_path.parent.mkdir(parents=True, exist_ok=True)
        record_file = output_path.open("x", encoding="utf-8")
        approach = start_points(current, q[0], args.start_speed)
        print(f"自动进入起点，约 {len(approach) * 0.01:.2f} 秒；Ctrl+C 停止发布。", flush=True)
        if not publish_trajectory(node, publisher, JointPositionControl, approach, names, 0.01, rclpy, feedback.position):
            raise RuntimeError("ROS 已关闭，进入起点中断")
        # 与 Qt 的 move_joint 一样，过渡结束后等待 1 秒，期间继续接收反馈。
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if not rclpy.ok():
                raise RuntimeError("ROS 已关闭")
            time.sleep(0.01)
            feedback.position()
        print("起点过渡结束，开始播放文件轨迹。", flush=True)
        recorder = QtRecorder(record_file, feedback, len(q))
        if not publish_trajectory(node, publisher, JointPositionControl, q, names, period,
                                  rclpy, feedback.position, recorder.record):
            raise RuntimeError("ROS 已关闭，轨迹发送中断")
        print("全部轨迹点已发布（不代表机器人已到位）。", flush=True)
        return 0
    except KeyboardInterrupt:
        print("\n已停止发布，服务端按 streaming 超时机制处理。", flush=True)
        return 130
    except (RuntimeError, OSError) as exc:
        print(f"发送失败：{exc}", flush=True)
        return 1
    finally:
        try:
            if record_file is not None:
                try:
                    if recorder is not None:
                        recorder.save()
                finally:
                    record_file.close()
                count = recorder.count if recorder is not None else 0
                print(f"已保存 {count}/{len(q)} 行采集数据：{output_path}", flush=True)
        finally:
            executor.shutdown()
            if ros_thread is not None:
                ros_thread.join()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
