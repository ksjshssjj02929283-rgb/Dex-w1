#!/usr/bin/env python3
"""实时绘制 /feedback/robot_server_state 中力相关字段的曲线.

X 轴随时间增长, 整段历史都会保留 (不做滚动窗口裁剪).

可选曲线 (开关式, 用一个或多个):
    --force_sensor              滤波后六维力 (12 维, primary/secondary 各 6 维)
    --raw_force_sensor          原始六维力
    --force_sensor_inertia      工具惯量补偿 wrench (primary, EE 系, 6 维: Fx Fy Fz Tx Ty Tz)

不指定任何开关时默认绘制 force_sensor + raw_force_sensor.

用法:
    python3 plot_force_sensor.py --force_sensor --raw_force_sensor
    python3 plot_force_sensor.py --force_sensor_inertia
    python3 plot_force_sensor.py --force_sensor --force_sensor_inertia --arm primary
"""

import argparse
import json
import threading
import time
from typing import Dict, List, Tuple

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from matplotlib.animation import FuncAnimation
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_STATE_TOPIC = "/feedback/robot_server_state"
SIX_AXIS_LABELS = ["Fx [N]", "Fy [N]", "Fz [N]", "Tx [Nm]", "Ty [Nm]", "Tz [Nm]"]
ARM_SLICE = {"primary": slice(0, 6), "secondary": slice(6, 12)}
ARM_COLOR = {"primary": "tab:blue", "secondary": "tab:orange"}


class StateSubscriber(Node):
    """订阅 robot_server_state 并按字段缓存历史样本."""

    def __init__(self, state_topic: str, want_six_axis: bool, want_inertia: bool):
        super().__init__("plot_force_sensor")
        self._lock = threading.Lock()
        self._want_six_axis = want_six_axis
        self._want_inertia = want_inertia
        self.t_buf: List[float] = []
        self.force_buf: List[np.ndarray] = []
        self.raw_buf: List[np.ndarray] = []
        self.inertia_wrench_buf: List[np.ndarray] = []  # primary 工具惯量 wrench, 6 维
        self.t0 = time.monotonic()
        self.received = 0
        self.parsed = 0

        self.create_subscription(String, state_topic, self._callback, 50)

    def _callback(self, msg: String) -> None:
        self.received += 1
        try:
            data = json.loads(msg.data)
            if self._want_six_axis:
                force = np.asarray(data.get("force_sensor", []), dtype=float)
                raw = np.asarray(data.get("raw_force_sensor", []), dtype=float)
                if force.size < 12 or raw.size < 12:
                    return
            else:
                force = np.zeros(12)
                raw = np.zeros(12)
            inertia = np.asarray(data.get("force_sensor_inertia", []), dtype=float)
            if inertia.size < 6:
                inertia = np.full(6, np.nan)
            else:
                inertia = inertia[:6]
            t = time.monotonic() - self.t0
            with self._lock:
                self.t_buf.append(t)
                self.force_buf.append(force[:12].copy())
                self.raw_buf.append(raw[:12].copy())
                self.inertia_wrench_buf.append(inertia.copy())
            self.parsed += 1
        except Exception:
            pass

    def snapshot(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        with self._lock:
            if not self.t_buf:
                empty12 = np.empty((0, 12))
                empty6 = np.empty((0, 6))
                empty1 = np.empty(0)
                return (empty1, empty12, empty12, empty6)
            t = np.asarray(self.t_buf, dtype=float)
            force = np.asarray(self.force_buf, dtype=float)
            raw = np.asarray(self.raw_buf, dtype=float)
            inertia = np.asarray(self.inertia_wrench_buf, dtype=float)
        return t, force, raw, inertia


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="实时绘制 robot_server_state 中力相关字段")
    parser.add_argument("--force_sensor", action="store_true", help="绘制 force_sensor (滤波后)")
    parser.add_argument("--raw_force_sensor", action="store_true", help="绘制 raw_force_sensor (原始)")
    parser.add_argument("--force_sensor_inertia", action="store_true",
                        help="绘制工具惯量补偿 wrench (primary, EE 系, 6 维)")
    parser.add_argument("--arm", choices=["primary", "secondary", "both"], default="primary",
                        help="选择手臂 (仅对六维力字段生效; force_sensor_inertia 只有 primary)")
    parser.add_argument("--interval", type=float, default=50.0, help="刷新间隔 (毫秒), 默认 50ms")
    parser.add_argument("--state-topic", default=DEFAULT_STATE_TOPIC)
    args = parser.parse_args()

    if not (args.force_sensor or args.raw_force_sensor or args.force_sensor_inertia):
        # 默认: force_sensor + raw_force_sensor
        args.force_sensor = True
        args.raw_force_sensor = True
    return args


def build_panels(args, arms: List[str]) -> List[Tuple[str, str]]:
    """返回 panel 列表, 每项 (kind, label).
    kind ∈ {"six_axis_0".."six_axis_5"} — 共用一组 6 个面板 (Fx Fy Fz Tx Ty Tz),
    force_sensor / raw_force_sensor / force_sensor_inertia 都画在对应分量上.
    """
    panels: List[Tuple[str, str]] = []
    if args.force_sensor or args.raw_force_sensor or args.force_sensor_inertia:
        for i, lbl in enumerate(SIX_AXIS_LABELS):
            panels.append((f"six_axis_{i}", lbl))
    return panels


def build_figure(args, arms: List[str], panels: List[Tuple[str, str]]):
    n = len(panels)
    if n <= 3:
        nrows, ncols = 1, n
    elif n <= 6:
        nrows, ncols = 2, 3
    elif n == 7:
        nrows, ncols = 2, 4
    else:
        nrows, ncols = 2, 4
    fig, ax_grid = plt.subplots(nrows, ncols, figsize=(3.4 * ncols, 3.6 * nrows), squeeze=False)
    axes = ax_grid.flatten()

    lines: Dict[Tuple, plt.Line2D] = {}
    for idx, (kind, label) in enumerate(panels):
        ax = axes[idx]
        if kind.startswith("six_axis_"):
            axis_idx = int(kind.split("_")[-1])
            for arm in arms:
                color = ARM_COLOR[arm]
                prefix = f"{arm} " if len(arms) > 1 else ""
                if args.force_sensor:
                    l, = ax.plot([], [], color=color, linewidth=1.4,
                                 label=f"{prefix}force_sensor".strip())
                    lines[(arm, "force", axis_idx)] = l
                if args.raw_force_sensor:
                    l, = ax.plot([], [], color=color, linewidth=1.0, linestyle="--", alpha=0.7,
                                 label=f"{prefix}raw_force_sensor".strip())
                    lines[(arm, "raw", axis_idx)] = l
            # 工具惯量补偿 wrench 仅 primary 一组, 不分 arm
            if args.force_sensor_inertia and "primary" in arms:
                l, = ax.plot([], [], color="tab:green", linewidth=1.4, linestyle=":",
                             label="force_sensor_inertia")
                lines[("inertia", axis_idx)] = l
        ax.set_title(label)
        ax.grid(True, alpha=0.3)
        ax.axhline(0.0, color="gray", linewidth=0.6, alpha=0.5)
        ax.set_xlabel("time [s]")
        ax.legend(loc="upper left", fontsize=8)

    # 关闭多余子图
    for idx in range(len(panels), len(axes)):
        axes[idx].set_visible(False)

    fig.suptitle(f"robot_server_state real-time ({'+'.join(arms)})")
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    return fig, axes, lines


def update_axis_ylim(ax, series_list):
    ymin = np.inf
    ymax = -np.inf
    for s in series_list:
        if s.size == 0:
            continue
        finite = s[np.isfinite(s)]
        if finite.size == 0:
            continue
        ymin = min(ymin, float(finite.min()))
        ymax = max(ymax, float(finite.max()))
    if not np.isfinite(ymin) or not np.isfinite(ymax) or ymax - ymin < 1e-6:
        ymin, ymax = -1.0, 1.0
    pad = 0.1 * (ymax - ymin) + 1e-3
    ax.set_ylim(ymin - pad, ymax + pad)


def main() -> None:
    args = parse_args()
    arms = ["primary", "secondary"] if args.arm == "both" else [args.arm]
    want_six_axis = args.force_sensor or args.raw_force_sensor
    want_inertia = args.force_sensor_inertia

    rclpy.init()
    node = StateSubscriber(args.state_topic, want_six_axis, want_inertia)
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

    panels = build_panels(args, arms)
    fig, axes, lines = build_figure(args, arms, panels)

    def update(_frame):
        t, force, raw, inertia = node.snapshot()
        if t.size == 0:
            return list(lines.values())

        t_min = float(t[0])
        t_max = float(t[-1])
        if t_max - t_min < 1e-6:
            t_max = t_min + 1.0

        for idx, (kind, _label) in enumerate(panels):
            ax = axes[idx]
            if kind.startswith("six_axis_"):
                axis_idx = int(kind.split("_")[-1])
                series_list = []
                for arm in arms:
                    sl = ARM_SLICE[arm]
                    if args.force_sensor:
                        y = force[:, sl][:, axis_idx]
                        lines[(arm, "force", axis_idx)].set_data(t, y)
                        series_list.append(y)
                    if args.raw_force_sensor:
                        y = raw[:, sl][:, axis_idx]
                        lines[(arm, "raw", axis_idx)].set_data(t, y)
                        series_list.append(y)
                if args.force_sensor_inertia and "primary" in arms and inertia.shape[0] == t.size:
                    y = inertia[:, axis_idx]
                    lines[("inertia", axis_idx)].set_data(t, y)
                    series_list.append(y)
                update_axis_ylim(ax, series_list)
            ax.set_xlim(t_min, t_max)

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
