#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Plot arm-only position error, torque, torque_ext, admittance/impedance fields, and force_sensor.

Data sources:
  - Default: read recent samples from ~/log/robot_record.jsonl.
  - Topic mode: capture std_msgs/String JSON from /feedback/robot_server_state.
    Captured topic samples are saved as plot_log_force_MMDDHHMM.jsonl
    in script/force_sensor/force_data/.

Examples:
  python3 plot_log_force.py 5
  python3 plot_log_force.py --log ~/log/robot_record.jsonl --seconds 5 --arm left
  python3 plot_log_force.py --topic --seconds 5
  python3 plot_log_force.py --topic /feedback/robot_server_state --joint 6
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from collections import deque
from datetime import datetime, timedelta
from pathlib import Path
from typing import Deque, Iterable, List, Sequence, Tuple

import numpy as np


LOG_PATH = Path("~/log/robot_record.jsonl").expanduser()
DEFAULT_STATE_TOPIC = "/feedback/robot_server_state"
ARRAY_KEYS = [
    "joint_position", "joint_position_cmd", "joint_torque", "torque_ext",
    "joint_admittance_q_offset", "joint_admittance_dq_offset",
    "joint_position_ref_admittance",
]
FORCE_SENSOR_KEY = "force_sensor"
FORCE_SENSOR_LABELS = ["Fx [N]", "Fy [N]", "Fz [N]", "Tx [Nm]", "Ty [Nm]", "Tz [Nm]"]


def parse_timestamp(value):
    ts_value = float(value)
    if ts_value > 1e12:
        ts_value /= 1e9
    return datetime.fromtimestamp(ts_value)


def parse_line(line: str, debug: bool = False):
    line = line.strip()
    if not line:
        return None

    try:
        data = json.loads(line)
    except Exception as exc:
        if debug:
            print(f"[DEBUG] Failed to parse JSON: {exc}")
        return None

    if "timestamp" not in data:
        return None

    try:
        ts = parse_timestamp(data["timestamp"])
    except Exception as exc:
        if debug:
            print(f"[DEBUG] Failed to parse timestamp '{data['timestamp']}': {exc}")
        return None

    for key in ARRAY_KEYS:
        if key not in data:
            continue
        value = data[key]
        if not isinstance(value, list) or len(value) not in (14, 20):
            if debug:
                size = len(value) if isinstance(value, list) else type(value)
                print(f"[DEBUG] Invalid array for {key}: {size}")
            return None

    return ts, data


def load_recent_samples(log_path: Path, max_samples: int, seconds: float, debug: bool = False):
    if not log_path.exists():
        raise FileNotFoundError(f"Log file not found: {log_path}")

    samples: Deque[Tuple[datetime, dict]] = deque(maxlen=max_samples)
    with log_path.open("r", encoding="utf-8", errors="ignore") as handle:
        for line in handle:
            parsed = parse_line(line, debug=debug)
            if parsed is not None:
                samples.append(parsed)

    if not samples:
        raise ValueError("No valid samples found.")

    cutoff = samples[-1][0] - timedelta(seconds=abs(seconds))
    filtered = deque((ts, data) for ts, data in samples if ts >= cutoff)
    if not filtered:
        raise ValueError(f"No samples found within the last {seconds} seconds.")
    return filtered


def make_topic_log_path() -> Path:
    timestamp = datetime.now().strftime("%m%d%H%M")
    output_dir = Path(__file__).resolve().parent / "force_data"
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir / f"plot_log_force_{timestamp}.jsonl"


def load_topic_samples(topics: Sequence[str], max_samples: int, seconds: float, debug: bool = False):
    try:
        import rclpy
        from std_msgs.msg import String
    except Exception as exc:
        raise RuntimeError("ROS topic mode requires rclpy and std_msgs in the current environment.") from exc

    if not topics:
        topics = [DEFAULT_STATE_TOPIC]

    samples: Deque[Tuple[datetime, dict]] = deque(maxlen=max_samples)
    rclpy.init(args=None)
    node = rclpy.create_node("plot_log_force_topic_capture")
    topic_log_path = make_topic_log_path()
    topic_log_handle = topic_log_path.open("w", encoding="utf-8")

    def make_callback(topic: str):
        def callback(msg: String):
            parsed = parse_line(msg.data, debug=debug)
            if parsed is None:
                return
            ts, data = parsed
            topic_log_handle.write(json.dumps(data, ensure_ascii=False, separators=(",", ":")) + "\n")
            topic_log_handle.flush()
            samples.append((ts, data))

        return callback

    subscriptions = [node.create_subscription(String, topic, make_callback(topic), 10) for topic in topics]

    if debug:
        print(f"[DEBUG] Subscribing topics: {', '.join(topics)}")
        print(f"[DEBUG] Capturing for {seconds:g}s, max samples={max_samples}")
        print(f"[DEBUG] Saving captured topic log to: {topic_log_path}")

    deadline = datetime.now() + timedelta(seconds=abs(seconds))
    try:
        while rclpy.ok() and datetime.now() < deadline and len(samples) < max_samples:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        topic_log_handle.close()
        subscriptions.clear()
        node.destroy_node()
        rclpy.shutdown()

    if not samples:
        raise ValueError(f"No valid samples captured from topics: {', '.join(topics)}")

    ordered = sorted(samples, key=lambda item: item[0])
    cutoff = ordered[-1][0] - timedelta(seconds=abs(seconds))
    filtered = [(ts, data) for ts, data in ordered if ts >= cutoff]
    if not filtered:
        raise ValueError(f"No samples captured within the last {seconds} seconds.")
    return filtered, topic_log_path


def detect_joint_count(samples: Iterable[Tuple[datetime, dict]]) -> int:
    for _, data in samples:
        value = data.get("joint_position")
        if isinstance(value, list) and len(value) in (14, 20):
            return len(value)
    raise ValueError("Unable to detect joint count from joint_position.")


def arm_joint_groups(joint_count: int) -> Tuple[List[int], List[int]]:
    if joint_count == 14:
        return list(range(0, 7)), list(range(7, 14))
    if joint_count == 20:
        return list(range(6, 13)), list(range(13, 20))
    raise ValueError(f"Unsupported joint count: {joint_count}")


def build_array(samples: Sequence[Tuple[datetime, dict]], key: str, joint_count: int) -> np.ndarray:
    arr = np.full((len(samples), joint_count), np.nan, dtype=float)
    for i, (_, data) in enumerate(samples):
        value = data.get(key)
        if isinstance(value, list) and len(value) == joint_count:
            arr[i, :] = np.asarray(value, dtype=float)
    return arr


def build_force_sensor_array(samples: Sequence[Tuple[datetime, dict]]) -> np.ndarray:
    arr = np.full((len(samples), 12), np.nan, dtype=float)
    for i, (_, data) in enumerate(samples):
        value = data.get(FORCE_SENSOR_KEY)
        if isinstance(value, list) and len(value) >= 12:
            arr[i, :] = np.asarray(value[:12], dtype=float)
    return arr


def has_force_sensor_data(force_sensor: np.ndarray) -> bool:
    return bool(force_sensor.size) and not np.isnan(force_sensor).all()


def compute_position_error_deg(joint_position: np.ndarray, joint_position_cmd: np.ndarray) -> np.ndarray:
    return np.degrees(joint_position_cmd - joint_position)


def make_time_axis(samples: Sequence[Tuple[datetime, dict]]) -> np.ndarray:
    start = samples[0][0]
    return np.asarray([(ts - start).total_seconds() for ts, _ in samples], dtype=float)


def configure_axes_grid(axes):
    import matplotlib.ticker as mticker

    for ax in axes.flatten():
        ax.grid(True, alpha=0.3, linestyle=":")
        ax.xaxis.set_major_formatter(mticker.FormatStrFormatter("%.3f"))
        ax.yaxis.set_major_formatter(mticker.FormatStrFormatter("%.3f"))
        ax.tick_params(labelsize=8)


def plot_arm_figure(
    arm_name: str,
    joint_indices: Sequence[int],
    times: np.ndarray,
    show_admittance: bool,
    pos_error_deg: np.ndarray,
    torque: np.ndarray,
    torque_ext: np.ndarray,
    admittance_q_offset: np.ndarray,
    admittance_dq_offset: np.ndarray,
    position_ref_admittance: np.ndarray,
    impedance_error_after_admittance: np.ndarray,
    force_sensor: np.ndarray,
    sensor_indices: Sequence[int],
    start_timestamp: datetime,
):
    import matplotlib.pyplot as plt

    cols = len(joint_indices)

    if show_admittance:
        plot_rows = 5
        rows = plot_rows
        figsize = (19.2, rows * 1.8)
        fig = plt.figure(figsize=figsize, dpi=100)
        grid = fig.add_gridspec(rows, cols)
        axes = np.empty((plot_rows, cols), dtype=object)
        for row in range(plot_rows):
            for col in range(cols):
                axes[row, col] = fig.add_subplot(grid[row, col])
        fig.suptitle(
            f"{arm_name} — Admittance/Impedance "
            f"(Log Start: {start_timestamp.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]})",
            fontsize=11,
        )
        configure_axes_grid(axes)

        for col, joint_idx in enumerate(joint_indices):
            joint_label = f"{arm_name.split()[0]} J{col + 1}"

            ax = axes[0, col]
            ax.plot(times, torque_ext[:, joint_idx], color="tab:purple", linewidth=1.2)
            ax.set_title(joint_label, fontsize=10)
            if col == 0:
                ax.set_ylabel("Torque Ext [Nm]", fontsize=9)

            ax = axes[1, col]
            ax.plot(times, admittance_q_offset[:, joint_idx], color="tab:orange", linewidth=1.2)
            if col == 0:
                ax.set_ylabel("Admittance Q Offset [rad]", fontsize=9)

            ax = axes[2, col]
            ax.plot(times, admittance_dq_offset[:, joint_idx], color="tab:cyan", linewidth=1.2)
            if col == 0:
                ax.set_ylabel("Admittance DQ Offset [rad/s]", fontsize=9)

            ax = axes[3, col]
            ax.plot(times, position_ref_admittance[:, joint_idx], color="tab:green", linewidth=1.2)
            if col == 0:
                ax.set_ylabel("Pos Ref Admittance [rad]", fontsize=9)

            ax = axes[4, col]
            ax.plot(times, impedance_error_after_admittance[:, joint_idx], color="tab:brown", linewidth=1.2)
            if col == 0:
                ax.set_ylabel("Impedance Error [rad]", fontsize=9)
            ax.set_xlabel("Time [s]", fontsize=9)
    else:
        has_force = has_force_sensor_data(force_sensor)
        plot_rows = 3
        rows = plot_rows + (6 if has_force else 0)
        figsize = (19.2, rows * 1.8)
        fig = plt.figure(figsize=figsize, dpi=100)
        grid = fig.add_gridspec(rows, cols)
        axes = np.empty((plot_rows, cols), dtype=object)
        for row in range(plot_rows):
            for col in range(cols):
                axes[row, col] = fig.add_subplot(grid[row, col])
        fig.suptitle(f"{arm_name} (Log Start: {start_timestamp.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]})", fontsize=11)
        configure_axes_grid(axes)

        for col, joint_idx in enumerate(joint_indices):
            joint_label = f"{arm_name.split()[0]} J{col + 1}"

            ax = axes[0, col]
            ax.plot(times, pos_error_deg[:, joint_idx], color="tab:blue", linewidth=1.2)
            ax.set_title(joint_label, fontsize=10)
            if col == 0:
                ax.set_ylabel("Pos Error [deg]", fontsize=9)

            ax = axes[1, col]
            ax.plot(times, torque[:, joint_idx], color="tab:red", linewidth=1.2)
            if col == 0:
                ax.set_ylabel("Torque [Nm]", fontsize=9)

            ax = axes[2, col]
            ax.plot(times, torque_ext[:, joint_idx], color="tab:purple", linewidth=1.2)
            if col == 0:
                ax.set_ylabel("Torque Ext [Nm]", fontsize=9)
            ax.set_xlabel("Time [s]", fontsize=9)

        if has_force:
            for row_offset, sensor_idx in enumerate(sensor_indices):
                ax = fig.add_subplot(grid[plot_rows + row_offset, :])
                configure_axes_grid(np.asarray([[ax]], dtype=object))
                ax.plot(times, force_sensor[:, sensor_idx], color="tab:green", linewidth=1.2)
                ax.axhline(0.0, color="gray", linewidth=0.6, alpha=0.5)
                ax.set_title(FORCE_SENSOR_LABELS[row_offset], fontsize=10)
                ax.set_ylabel(FORCE_SENSOR_LABELS[row_offset], fontsize=9)
                ax.set_xlabel("Time [s]", fontsize=9)

    fig.tight_layout(rect=[0.02, 0.02, 1.0, 0.97], h_pad=1.5, w_pad=1.2)
    return fig


def plot_single_joint(
    joint_idx: int,
    joint_count: int,
    times: np.ndarray,
    show_admittance: bool,
    pos_error_deg: np.ndarray,
    torque: np.ndarray,
    torque_ext: np.ndarray,
    admittance_q_offset: np.ndarray,
    admittance_dq_offset: np.ndarray,
    position_ref_admittance: np.ndarray,
    impedance_error_after_admittance: np.ndarray,
    start_timestamp: datetime,
):
    import matplotlib.pyplot as plt

    if joint_count == 14:
        title = f"{'Left' if joint_idx < 7 else 'Right'} J{joint_idx % 7 + 1}"
    else:
        if joint_idx < 6:
            raise ValueError("Requested joint is not an arm joint.")
        title = f"{'Left' if joint_idx < 13 else 'Right'} J{(joint_idx - 6) % 7 + 1}"

    if show_admittance:
        n_plots = 5
        fig, axes = plt.subplots(n_plots, 1, figsize=(12.8, n_plots * 2.0), dpi=100, sharex=True)
        configure_axes_grid(np.asarray(axes))
        fig.suptitle(
            f"{title} — Admittance/Impedance "
            f"(Log Start: {start_timestamp.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]})",
            fontsize=11,
        )

        axes[0].plot(times, torque_ext[:, joint_idx], color="tab:purple", linewidth=1.2)
        axes[0].set_ylabel("Torque Ext [Nm]", fontsize=9)

        axes[1].plot(times, admittance_q_offset[:, joint_idx], color="tab:orange", linewidth=1.2)
        axes[1].set_ylabel("Admittance Q Offset [rad]", fontsize=9)

        axes[2].plot(times, admittance_dq_offset[:, joint_idx], color="tab:cyan", linewidth=1.2)
        axes[2].set_ylabel("Admittance DQ Offset [rad/s]", fontsize=9)

        axes[3].plot(times, position_ref_admittance[:, joint_idx], color="tab:green", linewidth=1.2)
        axes[3].set_ylabel("Pos Ref Admittance [rad]", fontsize=9)

        axes[4].plot(times, impedance_error_after_admittance[:, joint_idx], color="tab:brown", linewidth=1.2)
        axes[4].set_ylabel("Impedance Error [rad]", fontsize=9)
        axes[4].set_xlabel("Time [s]", fontsize=9)
    else:
        fig, axes = plt.subplots(3, 1, figsize=(12.8, 8.0), dpi=100, sharex=True)
        configure_axes_grid(np.asarray(axes))
        fig.suptitle(f"{title} (Log Start: {start_timestamp.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]})", fontsize=11)

        axes[0].plot(times, pos_error_deg[:, joint_idx], color="tab:blue", linewidth=1.2)
        axes[0].set_ylabel("Pos Error [deg]", fontsize=9)

        axes[1].plot(times, torque[:, joint_idx], color="tab:red", linewidth=1.2)
        axes[1].set_ylabel("Torque [Nm]", fontsize=9)

        axes[2].plot(times, torque_ext[:, joint_idx], color="tab:purple", linewidth=1.2)
        axes[2].set_ylabel("Torque Ext [Nm]", fontsize=9)
        axes[2].set_xlabel("Time [s]", fontsize=9)

    fig.tight_layout(rect=[0.02, 0.02, 1.0, 0.95], h_pad=1.5)
    return fig


def parse_args() -> argparse.Namespace:
    if len(sys.argv) >= 2:
        match = re.fullmatch(r"-?\d+(\.\d+)?", sys.argv[1])
        if match:
            seconds = abs(float(sys.argv[1]))
            sys.argv[1] = f"--seconds={seconds}"

    parser = argparse.ArgumentParser(
        description="Plot arm position error, torque, torque_ext, admittance/impedance fields, and force_sensor from robot_record or robot_server_state topics."
    )
    parser.add_argument("--seconds", type=float, default=5.0, help="Time window in seconds to plot from the end.")
    parser.add_argument("--n", type=int, default=5000, help="Maximum valid samples kept while reading.")
    parser.add_argument("--save", action="store_true", help="Save figures to this script directory instead of showing.")
    parser.add_argument("--debug", action="store_true", help="Enable debug logging.")
    parser.add_argument("--admittance", action="store_true", help="Plot admittance/impedance fields only.")
    parser.add_argument("--joint", type=int, default=None, help="Plot one arm joint by raw joint index.")
    parser.add_argument("--arm", choices=["both", "left", "right"], default="both", help="Select which arm to plot.")
    parser.add_argument("--log", type=Path, default=LOG_PATH, help=f"Path to robot_record.jsonl (default: {LOG_PATH})")
    parser.add_argument(
        "--topic",
        "--topics",
        nargs="*",
        dest="topics",
        default=None,
        help=f"Capture std_msgs/String robot_server_state JSON from ROS topics (default: {DEFAULT_STATE_TOPIC}).",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.topics is None:
        samples = list(load_recent_samples(args.log.expanduser(), args.n, args.seconds, debug=args.debug))
        source_label = str(args.log.expanduser())
    else:
        topics = args.topics if args.topics else [DEFAULT_STATE_TOPIC]
        samples, topic_log_path = load_topic_samples(topics, args.n, args.seconds, debug=args.debug)
        samples = list(samples)
        source_label = ", ".join(topics)
        print(f"Saved topic log: {topic_log_path}")

    joint_count = detect_joint_count(samples)
    left_arm, right_arm = arm_joint_groups(joint_count)

    times = make_time_axis(samples)
    joint_position = build_array(samples, "joint_position", joint_count)
    joint_position_cmd = build_array(samples, "joint_position_cmd", joint_count)
    joint_torque = build_array(samples, "joint_torque", joint_count)
    torque_ext = build_array(samples, "torque_ext", joint_count)
    force_sensor = build_force_sensor_array(samples)
    pos_error_deg = compute_position_error_deg(joint_position, joint_position_cmd)
    start_timestamp = samples[0][0]

    show_admittance = args.admittance
    admittance_q_offset = build_array(samples, "joint_admittance_q_offset", joint_count) if show_admittance else None
    admittance_dq_offset = build_array(samples, "joint_admittance_dq_offset", joint_count) if show_admittance else None
    position_ref_admittance = build_array(samples, "joint_position_ref_admittance", joint_count) if show_admittance else None
    impedance_error_after_admittance = (position_ref_admittance - joint_position) if show_admittance else None

    if args.debug:
        print(f"[DEBUG] Loaded {len(samples)} samples from {source_label}")
        print(f"[DEBUG] Joint count: {joint_count}")
        print(f"[DEBUG] Arm selection: {args.arm}")
        print(f"[DEBUG] Admittance mode: {show_admittance}")

    import matplotlib.pyplot as plt

    figures = []
    if args.joint is not None:
        if args.joint not in left_arm and args.joint not in right_arm:
            raise ValueError(f"Joint {args.joint} is not an arm joint for joint_count={joint_count}.")
        figures.append(plot_single_joint(args.joint, joint_count, times, show_admittance,
                                         pos_error_deg, joint_torque, torque_ext,
                                         admittance_q_offset, admittance_dq_offset,
                                         position_ref_admittance, impedance_error_after_admittance,
                                         start_timestamp))
    else:
        if args.arm in ("both", "left"):
            figures.append(plot_arm_figure("Left Arm", left_arm, times, show_admittance,
                                           pos_error_deg, joint_torque, torque_ext,
                                           admittance_q_offset, admittance_dq_offset,
                                           position_ref_admittance, impedance_error_after_admittance,
                                           force_sensor, list(range(0, 6)), start_timestamp))
        if args.arm in ("both", "right"):
            figures.append(plot_arm_figure("Right Arm", right_arm, times, show_admittance,
                                           pos_error_deg, joint_torque, torque_ext,
                                           admittance_q_offset, admittance_dq_offset,
                                           position_ref_admittance, impedance_error_after_admittance,
                                           force_sensor, list(range(6, 12)), start_timestamp))

    if not has_force_sensor_data(force_sensor) and args.debug:
        print("[DEBUG] No valid force_sensor samples found; skipping force_sensor figure.")

    if args.save:
        output_dir = Path(__file__).resolve().parent / "force_data"
        output_dir.mkdir(parents=True, exist_ok=True)
        for idx, fig in enumerate(figures, start=1):
            suffix = f"joint_{args.joint}" if args.joint is not None else args.arm
            out_path = output_dir / f"plot_log_force_{suffix}_{idx}.png"
            fig.savefig(out_path, dpi=150)
            print(f"Saved: {out_path}")
        return

    plt.show()


if __name__ == "__main__":
    main()
