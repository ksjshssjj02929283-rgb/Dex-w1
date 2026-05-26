#!/usr/bin/env python3
"""实时绘制 /feedback/robot_server_state 中的 force_sensor 与 raw_force_sensor 曲线。

X 轴随时间增长, 整段历史都会保留 (不做滚动窗口裁剪).

用法:
    python3 plot_force_sensor.py                     # primary 手臂
    python3 plot_force_sensor.py --arm secondary
    python3 plot_force_sensor.py --arm both          # 同时绘制 primary + secondary

force_sensor / raw_force_sensor 均为长度 12 的数组,
primary 占 [0:6], secondary 占 [6:12], 每个手臂 6 维: Fx Fy Fz Tx Ty Tz.
"""

import argparse
import json
import threading
import time
from typing import Tuple

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from matplotlib.animation import FuncAnimation
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_STATE_TOPIC = "/feedback/robot_server_state"
AXIS_LABELS = ["Fx [N]", "Fy [N]", "Fz [N]", "Tx [Nm]", "Ty [Nm]", "Tz [Nm]"]
ARM_SLICE = {"primary": slice(0, 6), "secondary": slice(6, 12)}


class ForceSensorSubscriber(Node):
    """订阅 robot_server_state 并缓存 force_sensor / raw_force_sensor 数据."""

    def __init__(self, state_topic: str):
        super().__init__("plot_force_sensor")
        self._lock = threading.Lock()
        self.t_buf: list = []
        self.force_buf: list = []
        self.raw_buf: list = []
        self.t0 = time.monotonic()
        self.received = 0
        self.parsed = 0

        self.create_subscription(String, state_topic, self._callback, 50)

    def _callback(self, msg: String) -> None:
        self.received += 1
        try:
            data = json.loads(msg.data)
            force = np.asarray(data.get("force_sensor", []), dtype=float)
            raw = np.asarray(data.get("raw_force_sensor", []), dtype=float)
            if force.size < 12 or raw.size < 12:
                return
            t = time.monotonic() - self.t0
            with self._lock:
                self.t_buf.append(t)
                self.force_buf.append(force[:12].copy())
                self.raw_buf.append(raw[:12].copy())
            self.parsed += 1
        except Exception:
            pass

    def snapshot(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        with self._lock:
            if not self.t_buf:
                return (np.empty(0), np.empty((0, 12)), np.empty((0, 12)))
            t = np.asarray(self.t_buf, dtype=float)
            force = np.asarray(self.force_buf, dtype=float)
            raw = np.asarray(self.raw_buf, dtype=float)
        return t, force, raw


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="实时绘制 force_sensor / raw_force_sensor 曲线")
    parser.add_argument("--arm", choices=["primary", "secondary", "both"], default="primary")
    parser.add_argument("--interval", type=float, default=50.0, help="刷新间隔 (毫秒), 默认 50ms")
    parser.add_argument("--state-topic", default=DEFAULT_STATE_TOPIC)
    return parser.parse_args()


def build_figure(arms):
    """构建 2 行 x 3 列子图, 每个子图为一个力/力矩通道."""
    fig, axes = plt.subplots(2, 3, figsize=(13, 7), sharex=True)
    axes = axes.flatten()
    lines = {}  # (arm, kind, axis_idx) -> Line2D
    arm_color = {"primary": "tab:blue", "secondary": "tab:orange"}
    for ax_idx, ax in enumerate(axes):
        for arm in arms:
            color = arm_color[arm]
            label_prefix = arm if len(arms) > 1 else ""
            sep = " " if label_prefix else ""
            l_filt, = ax.plot([], [], color=color, linewidth=1.4,
                              label=f"{label_prefix}{sep}force_sensor".strip())
            l_raw, = ax.plot([], [], color=color, linewidth=1.0, linestyle="--", alpha=0.7,
                             label=f"{label_prefix}{sep}raw_force_sensor".strip())
            lines[(arm, "force", ax_idx)] = l_filt
            lines[(arm, "raw", ax_idx)] = l_raw
        ax.set_title(AXIS_LABELS[ax_idx])
        ax.grid(True, alpha=0.3)
        ax.axhline(0.0, color="gray", linewidth=0.6, alpha=0.5)
        if ax_idx >= 3:
            ax.set_xlabel("time [s]")
    axes[0].legend(loc="upper left", fontsize=8)
    fig.suptitle(f"Force sensor real-time ({'+'.join(arms)})")
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    return fig, axes, lines


def main() -> None:
    args = parse_args()
    arms = ["primary", "secondary"] if args.arm == "both" else [args.arm]

    rclpy.init()
    node = ForceSensorSubscriber(args.state_topic)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    print(f"等待 {args.state_topic} 数据 ...")
    t0 = time.monotonic()
    while node.parsed == 0 and time.monotonic() - t0 < 10.0:
        time.sleep(0.05)
    if node.parsed == 0:
        print(f"超时: 未收到数据 (received={node.received})")
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        return

    print(f"已连接, received={node.received} parsed={node.parsed}, 开始绘图 ...")

    fig, axes, lines = build_figure(arms)

    def update(_frame):
        t, force, raw = node.snapshot()
        if t.size == 0:
            return list(lines.values())

        for arm in arms:
            sl = ARM_SLICE[arm]
            f6 = force[:, sl]
            r6 = raw[:, sl]
            for ax_idx in range(6):
                lines[(arm, "force", ax_idx)].set_data(t, f6[:, ax_idx])
                lines[(arm, "raw", ax_idx)].set_data(t, r6[:, ax_idx])

        t_min = float(t[0])
        t_max = float(t[-1])
        if t_max - t_min < 1e-6:
            t_max = t_min + 1.0
        for ax_idx, ax in enumerate(axes):
            ax.set_xlim(t_min, t_max)
            ymin = np.inf
            ymax = -np.inf
            for arm in arms:
                sl = ARM_SLICE[arm]
                ymin = min(ymin, force[:, sl][:, ax_idx].min(initial=np.inf),
                           raw[:, sl][:, ax_idx].min(initial=np.inf))
                ymax = max(ymax, force[:, sl][:, ax_idx].max(initial=-np.inf),
                           raw[:, sl][:, ax_idx].max(initial=-np.inf))
            if not np.isfinite(ymin) or not np.isfinite(ymax) or ymax - ymin < 1e-6:
                ymin, ymax = -1.0, 1.0
            pad = 0.1 * (ymax - ymin) + 1e-3
            ax.set_ylim(ymin - pad, ymax + pad)

        return list(lines.values())

    ani = FuncAnimation(fig, update, interval=args.interval, blit=False, cache_frame_data=False)

    try:
        plt.show()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
