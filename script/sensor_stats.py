#!/usr/bin/env python3
"""统计一段时间内的传感器数值并取平均值。

用法:
    python3 sensor_stats.py                        # 默认采集 5 秒, primary 手臂
    python3 sensor_stats.py --duration 10           # 采集 10 秒
    python3 sensor_stats.py --arm secondary         # 采集 secondary 手臂
"""

import argparse
import json
import threading
import time
from collections import deque
from typing import Deque

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_STATE_TOPIC = "/feedback/robot_server_state"


class SensorStatsNode(Node):
    def __init__(self, state_topic: str, arm: str):
        super().__init__("sensor_stats")
        self.arm = arm
        self._lock = threading.Lock()
        self._samples: Deque[np.ndarray] = deque(maxlen=50000)
        self._received = 0
        self._parsed = 0

        self.create_subscription(String, state_topic, self._callback, 10)

    def _callback(self, msg: String) -> None:
        self._received += 1
        try:
            data = json.loads(msg.data)
            force_sensor = np.asarray(data["force_sensor"], dtype=float)
            if force_sensor.size < 12:
                return

            if self.arm == "primary":
                raw = force_sensor[:6]
            else:
                raw = force_sensor[6:12]

            with self._lock:
                self._samples.append(raw.copy())
            self._parsed += 1
        except Exception:
            pass

    def get_samples(self):
        with self._lock:
            return list(self._samples)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="统计一段时间内的传感器数值并取平均值。")
    parser.add_argument("--duration", type=float, default=5.0, help="采集时长 (秒), 默认 5s")
    parser.add_argument("--arm", choices=["primary", "secondary"], default="primary")
    parser.add_argument("--state-topic", default=DEFAULT_STATE_TOPIC)
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    rclpy.init()
    node = SensorStatsNode(args.state_topic, args.arm)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    print(f"等待 {args.state_topic} 数据...")
    t0 = time.monotonic()
    while node._parsed == 0 and time.monotonic() - t0 < 10.0:
        time.sleep(0.05)
    if node._parsed == 0:
        print(f"超时: 未收到数据 (received={node._received})")
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        return

    print(f"已连接, 开始采集 {args.duration:.1f}s ...")
    with node._lock:
        node._samples.clear()

    time.sleep(args.duration)

    sample_list = node.get_samples()
    executor.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    if not sample_list:
        print("采集期间无数据!")
        return

    arr = np.array(sample_list)
    n = len(sample_list)

    print(f"\n========== 传感器统计 ({args.arm} arm) ==========")
    print(f"采集时长:  {args.duration:.1f}s")
    print(f"样本数:    {n}")

    print(f"\n{'':>4s}  {'mean':>10s}  {'std':>10s}  {'min':>10s}  {'max':>10s}")
    for i in range(6):
        print(f"[{i}]   {arr[:, i].mean():>10.4f}  {arr[:, i].std():>10.4f}  "
              f"{arr[:, i].min():>10.4f}  {arr[:, i].max():>10.4f}")

    mean_val = arr.mean(axis=0)
    vals = ", ".join(f"{v:.4f}" for v in mean_val)
    print(f"\n平均值: [{vals}]")
    print("=" * 50)


if __name__ == "__main__":
    main()
