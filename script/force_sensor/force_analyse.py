#!/usr/bin/env python3
"""Record, plot, action-replay-record, and analyse robot force-sensor state fields.

Records are written to script/force_sensor/force_analyse_data/<timestamp>.json
by default. Use --record to capture data from /feedback/robot_server_state,
--plot to visualize an existing record, --replay to play an action JSON while
recording force data, and --analyse to compute RMS from the latest recording.
Plotting without a path uses the latest record in the default output directory.
"""

from __future__ import annotations

import argparse
import json
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Mapping, Sequence

DEFAULT_STATE_TOPIC = "/feedback/robot_server_state"
DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parent / "force_analyse_data"
JOINT_POSITION_TOPIC = "/control/joint_position"
LEFT_HAND_TOPIC = "/set_brainco_left_hand_qpos"
RIGHT_HAND_TOPIC = "/set_brainco_right_hand_qpos"
FORCE_FIELDS = [
    "force_sensor",
    "force_sensor_gravity",
    "raw_force_sensor",
    "force_sensor_bias",
]
EE_FIELDS = [
    "ee_omega_ee_frame",
    "ee_alpha_ee_frame",
]
JOINT_STATE_FIELDS = [
    "joint_velocity",
]
AXIS_LABELS = ["Fx [N]", "Fy [N]", "Fz [N]", "Tx [Nm]", "Ty [Nm]", "Tz [Nm]"]
EE_AXIS_LABELS = ["ωx [rad/s]", "ωy [rad/s]", "ωz [rad/s]"]
FIELD_COLORS = {
    "force_sensor": "tab:blue",
    "force_sensor_gravity": "tab:green",
    "raw_force_sensor": "tab:gray",
    "force_sensor_bias": "tab:red",
    "ee_omega_ee_frame": "tab:orange",
    "ee_alpha_ee_frame": "tab:purple",
}
FIELD_LABELS = {
    "force_sensor": "force_sensor",
    "force_sensor_gravity": "force_sensor_gravity",
    "raw_force_sensor": "raw_force_sensor",
    "force_sensor_bias": "force_sensor_bias",
    "ee_omega_ee_frame": "ee_omega_ee_frame",
    "ee_alpha_ee_frame": "ee_alpha_ee_frame",
}

JOINT_KEYS = [
    "ANKLE", "KNEE", "BUTTOCK", "WAIST", "NECK1", "NECK2",
    "LEFT_J1", "LEFT_J2", "LEFT_J3", "LEFT_J4", "LEFT_J5", "LEFT_J6", "LEFT_J7",
    "RIGHT_J1", "RIGHT_J2", "RIGHT_J3", "RIGHT_J4", "RIGHT_J5", "RIGHT_J6", "RIGHT_J7",
]
FINGER_KEYS = [
    "LEFT_HAND_THUMB1", "LEFT_HAND_THUMB2", "LEFT_HAND_INDEX",
    "LEFT_HAND_MIDDLE", "LEFT_HAND_RING", "LEFT_HAND_PINKY",
    "RIGHT_HAND_THUMB1", "RIGHT_HAND_THUMB2", "RIGHT_HAND_INDEX",
    "RIGHT_HAND_MIDDLE", "RIGHT_HAND_RING", "RIGHT_HAND_PINKY",
]


def timestamp_filename(now: datetime | None = None) -> str:
    stamp = (now or datetime.now()).strftime("%Y%m%d_%H%M%S")
    return f"{stamp}.json"


def latest_recording(output_dir: Path) -> Path:
    candidates = sorted(output_dir.glob("*.json"), key=lambda p: p.stat().st_mtime)
    if not candidates:
        raise FileNotFoundError(f"no JSON recordings found in {output_dir}")
    return candidates[-1]


def float_list(value: object, name: str, min_size: int = 1) -> List[float]:
    if value is None:
        raise ValueError(f"missing {name}")
    try:
        values = [float(item) for item in value if item is not None]  # type: ignore[union-attr]
    except TypeError as exc:
        raise ValueError(f"{name} must be a numeric list") from exc
    if len(values) < min_size:
        raise ValueError(f"{name} needs at least {min_size} values, got {len(values)}")
    return values


def extract_force_sample(data: Mapping[str, object], t: float = 0.0) -> Dict[str, object]:
    sample: Dict[str, object] = {
        "t": float(t),
        "timestamp": data.get("timestamp"),
    }
    for field in FORCE_FIELDS:
        sample[field] = float_list(data.get(field), field, 6)
    for field in EE_FIELDS:
        try:
            sample[field] = float_list(data.get(field), field, 3)
        except (ValueError, TypeError):
            pass
    for field in JOINT_STATE_FIELDS:
        try:
            sample[field] = float_list(data.get(field), field, 1)
        except (ValueError, TypeError):
            pass
    return sample


def make_payload(
    samples: Sequence[Mapping[str, object]],
    *,
    state_topic: str,
    created_at: datetime | None = None,
) -> Dict[str, object]:
    created = created_at or datetime.now()
    return {
        "metadata": {
            "created_at": created.isoformat(timespec="seconds"),
            "state_topic": state_topic,
            "sample_count": len(samples),
            "fields": FORCE_FIELDS + EE_FIELDS + JOINT_STATE_FIELDS,
            "axis_labels": AXIS_LABELS,
            "notes": "Each field is stored exactly as published by robot_server_state.",
        },
        "samples": list(samples),
    }


def arm_slice(arm: str) -> slice:
    if arm == "primary":
        return slice(0, 6)
    if arm == "secondary":
        return slice(6, 12)
    raise ValueError(f"unsupported arm: {arm}")


def ee_arm_slice(arm: str) -> slice:
    if arm == "primary":
        return slice(0, 3)
    if arm == "secondary":
        return slice(3, 6)
    raise ValueError(f"unsupported arm: {arm}")


def ee_field_matrix(samples: Sequence[Mapping[str, object]], field: str, arm: str) -> List[List[float]]:
    idx = ee_arm_slice(arm)
    matrix: List[List[float]] = []
    for sample in samples:
        try:
            values = float_list(sample.get(field), field, 3)
        except (ValueError, TypeError):
            continue
        selected = values[idx]
        if len(selected) < 3:
            continue
        matrix.append(selected[:3])
    return matrix


def field_matrix(samples: Sequence[Mapping[str, object]], field: str, arm: str) -> List[List[float]]:
    idx = arm_slice(arm)
    matrix: List[List[float]] = []
    for sample in samples:
        values = float_list(sample.get(field), field, 6)
        selected = values[idx]
        if len(selected) < 6:
            raise ValueError(f"{field} for {arm} arm needs 6 values, got {len(selected)}")
        matrix.append(selected[:6])
    return matrix


def load_payload(path: Path) -> Mapping[str, object]:
    with path.open("r", encoding="utf-8") as f:
        payload = json.load(f)
    samples = payload.get("samples")
    if not isinstance(samples, list) or not samples:
        raise ValueError(f"{path} has no samples")
    return payload


def make_recorder_node_class(node_base, string_msg):
    class ForceAnalyseRecorder(node_base):
        def __init__(self, state_topic: str):
            super().__init__("force_analyse_recorder")
            self._lock = threading.Lock()
            self._t0: float | None = None
            self.samples: List[Mapping[str, object]] = []
            self.received = 0
            self.parsed = 0
            self.last_error = ""
            self.create_subscription(string_msg, state_topic, self._callback, 50)

        def _callback(self, msg) -> None:
            self.received += 1
            now = time.monotonic()
            if self._t0 is None:
                self._t0 = now
            try:
                data = json.loads(msg.data)
                sample = extract_force_sample(data, now - self._t0)
            except Exception as exc:
                self.last_error = str(exc)
                return
            with self._lock:
                self.samples.append(sample)
            self.parsed += 1

        def snapshot(self) -> List[Mapping[str, object]]:
            with self._lock:
                return list(self.samples)

    return ForceAnalyseRecorder


def robot_status_callback(node):
    def _callback(msg) -> None:
        try:
            data = json.loads(msg.data)
            node.server_status = data.get("status")
        except Exception as exc:
            node.get_logger().error(f"Failed to parse robot_server_state: {exc}")

    return _callback


def read_action_file(path: Path) -> Mapping[str, object]:
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    frames = data.get("frames")
    if not isinstance(frames, list) or not frames:
        raise ValueError(f"{path} has no frames")
    return data


def frame_timestamp(frame: Mapping[str, object]) -> float:
    return float(frame.get("timestamp", 0.0))


def action_joint_names_and_positions(joint_data: Mapping[str, object]) -> tuple[List[str], List[float]]:
    names: List[str] = []
    positions: List[float] = []
    for key in JOINT_KEYS:
        if key in joint_data:
            names.append(key)
            positions.append(float(joint_data[key]))
    return names, positions


def hand_qpos(joint_data: Mapping[str, object], side: str) -> List[float]:
    if side == "left":
        keys = FINGER_KEYS[:6]
    elif side == "right":
        keys = FINGER_KEYS[6:]
    else:
        raise ValueError(f"unsupported hand side: {side}")
    return [float(joint_data.get(key, 0.0)) for key in keys]


def save_samples(samples: Sequence[Mapping[str, object]], output_path: Path, state_topic: str) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    payload = make_payload(samples, state_topic=state_topic)
    with output_path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
        f.write("\n")


def record(args: argparse.Namespace) -> None:
    output_path = args.output or (args.output_dir / timestamp_filename())
    output_path.parent.mkdir(parents=True, exist_ok=True)

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    recorder_node_class = make_recorder_node_class(Node, String)

    rclpy.init()
    node = recorder_node_class(args.state_topic)

    print(f"Recording force data from {args.state_topic}")
    print(f"Output: {output_path}")
    if args.duration > 0:
        print(f"Duration: {args.duration:.3f}s")
    else:
        print("Press Ctrl-C to stop.")

    start = time.monotonic()
    try:
        while rclpy.ok():
            if args.duration > 0 and time.monotonic() - start >= args.duration:
                break
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.received and node.parsed == 0 and node.last_error:
                print(f"Waiting for valid state: {node.last_error}", end="\r")
    except KeyboardInterrupt:
        pass
    finally:
        samples = node.snapshot()
        save_samples(samples, output_path, args.state_topic)
        print()
        print(f"Saved {len(samples)} samples to {output_path}")
        if len(samples) == 0 and node.last_error:
            print(f"Last parse error: {node.last_error}")
        node.destroy_node()
        rclpy.shutdown()


def update_ylim(ax, line_list: Sequence[object]) -> None:
    ymin = None
    ymax = None
    for line in line_list:
        if not line.get_visible():
            continue
        ydata = line.get_ydata()
        finite = [float(y) for y in ydata if y == y and y not in (float("inf"), float("-inf"))]
        if not finite:
            continue
        low = min(finite)
        high = max(finite)
        ymin = low if ymin is None else min(ymin, low)
        ymax = high if ymax is None else max(ymax, high)
    if ymin is None or ymax is None or abs(ymax - ymin) < 1e-9:
        ymin, ymax = -1.0, 1.0
    pad = 0.1 * (ymax - ymin) + 1e-3
    ax.set_ylim(ymin - pad, ymax + pad)


def plot(args: argparse.Namespace) -> None:
    import matplotlib.pyplot as plt
    from matplotlib.widgets import CheckButtons

    path = args.path or latest_recording(args.output_dir)
    payload = load_payload(path)
    samples = payload["samples"]

    t = [float(sample.get("t", idx)) for idx, sample in enumerate(samples)]
    matrices = {
        field: field_matrix(samples, field, args.arm)
        for field in FORCE_FIELDS
    }

    fig, axes_grid = plt.subplots(2, 3, figsize=(14, 7), sharex=True)
    axes = list(axes_grid.flatten())
    lines_by_field: Dict[str, List[plt.Line2D]] = {field: [] for field in FORCE_FIELDS}
    lines_by_axis: List[List[plt.Line2D]] = [[] for _ in range(6)]
    initially_visible = set(args.show)

    for axis_idx, ax in enumerate(axes):
        for field in FORCE_FIELDS:
            y = [row[axis_idx] for row in matrices[field]]
            (line,) = ax.plot(
                t,
                y,
                label=FIELD_LABELS[field],
                color=FIELD_COLORS[field],
                linewidth=1.2,
                linestyle="--" if field == "raw_force_sensor" else "-",
                alpha=0.85,
                visible=field in initially_visible,
            )
            lines_by_field[field].append(line)
            lines_by_axis[axis_idx].append(line)
        ax.set_title(AXIS_LABELS[axis_idx])
        ax.set_xlabel("time [s]")
        ax.grid(True, alpha=0.3)
        ax.axhline(0.0, color="gray", linewidth=0.6, alpha=0.5)
        ax.legend(loc="upper left", fontsize=8)
        update_ylim(ax, lines_by_axis[axis_idx])

    check_ax = fig.add_axes([0.78, 0.02, 0.20, 0.14])
    check = CheckButtons(
        check_ax,
        [FIELD_LABELS[field] for field in FORCE_FIELDS],
        [field in initially_visible for field in FORCE_FIELDS],
    )

    def on_check(label: str) -> None:
        field = next(key for key in FORCE_FIELDS if FIELD_LABELS[key] == label)
        for line in lines_by_field[field]:
            line.set_visible(not line.get_visible())
        for axis_idx, ax in enumerate(axes):
            update_ylim(ax, lines_by_axis[axis_idx])
        fig.canvas.draw_idle()

    check.on_clicked(on_check)
    fig.suptitle(f"{path.name} — force ({args.arm})")
    fig.tight_layout(rect=[0.0, 0.08, 1.0, 0.95])

    # EE fields figure (3 axes × 2 fields) — only if EE data is present
    ee_matrices = {
        field: ee_field_matrix(samples, field, args.arm)
        for field in EE_FIELDS
    }
    if any(len(m) > 0 for m in ee_matrices.values()):
        fig_ee, axes_ee_grid = plt.subplots(1, 3, figsize=(14, 4), sharex=True)
        axes_ee = list(axes_ee_grid.flatten())
        lines_by_ee_axis: List[List[plt.Line2D]] = [[] for _ in range(3)]

        for axis_idx, ax in enumerate(axes_ee):
            for field in EE_FIELDS:
                matrix = ee_matrices[field]
                if not matrix:
                    continue
                y = [row[axis_idx] for row in matrix]
                (line,) = ax.plot(
                    t[:len(matrix)],
                    y,
                    label=FIELD_LABELS[field],
                    color=FIELD_COLORS[field],
                    linewidth=1.2,
                    alpha=0.85,
                )
                lines_by_ee_axis[axis_idx].append(line)
            ax.set_title(EE_AXIS_LABELS[axis_idx])
            ax.set_xlabel("time [s]")
            ax.grid(True, alpha=0.3)
            ax.axhline(0.0, color="gray", linewidth=0.6, alpha=0.5)
            ax.legend(loc="upper left", fontsize=8)
            update_ylim(ax, lines_by_ee_axis[axis_idx])

        fig_ee.suptitle(f"{path.name} — EE kinematics ({args.arm})")
        fig_ee.tight_layout(rect=[0.0, 0.0, 1.0, 0.95])

    plt.show()


ANALYSE_FIELDS = [
    "raw_force_sensor",
    "force_sensor",
    "force_sensor_gravity",
]


def analyse(args: argparse.Namespace) -> None:
    path = args.path or latest_recording(args.output_dir)
    payload = load_payload(path)
    samples = payload["samples"]
    print(f"RMS vs zero — {path.name}  ({len(samples)} samples, {args.arm} arm)\n")

    for field in ANALYSE_FIELDS:
        matrix = field_matrix(samples, field, args.arm)
        print(f"{field}:")
        for axis_idx, label in enumerate(AXIS_LABELS):
            values = [row[axis_idx] for row in matrix]
            rms = (sum(v * v for v in values) / len(values)) ** 0.5
            peak = max(abs(v) for v in values)
            print(f"  {label:8s}  RMS={rms:.6f}  |peak|={peak:.6f}")
        print()


def replay_action_and_record(args: argparse.Namespace) -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from joint_interfaces.msg import JointPositionControl
    from std_msgs.msg import Float64MultiArray, String

    action_data = read_action_file(args.replay)
    frames = action_data["frames"]
    output_path = args.output or (args.output_dir / timestamp_filename())

    rclpy.init()
    recorder_node_class = make_recorder_node_class(Node, String)
    node = recorder_node_class(args.state_topic)
    node.joint_publisher = node.create_publisher(JointPositionControl, JOINT_POSITION_TOPIC, 10)
    node.left_hand_publisher = node.create_publisher(Float64MultiArray, LEFT_HAND_TOPIC, 10)
    node.right_hand_publisher = node.create_publisher(Float64MultiArray, RIGHT_HAND_TOPIC, 10)
    node.server_status = None
    node.create_subscription(String, args.state_topic, robot_status_callback(node), 10)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    print(f"Replaying action from {args.replay}")
    print(f"Recording force data from {args.state_topic}")
    print(f"Output: {output_path}")
    print("Press Ctrl-C to stop.")

    try:
        replay_start = time.time()
        record_start_time = None
        for frame in frames:
            if getattr(node, "server_status", None) == "Init":
                node.get_logger().warn("Robot status is Init; stop replay.")
                break

            timestamp = frame_timestamp(frame)
            if record_start_time is None:
                record_start_time = timestamp
            sleep_time = (timestamp - record_start_time) - (time.time() - replay_start)
            if sleep_time > 0.0:
                time.sleep(sleep_time)

            joint_data = frame.get("data", {})
            if not isinstance(joint_data, Mapping):
                continue

            joint_msg = JointPositionControl()
            joint_msg.header.stamp = node.get_clock().now().to_msg()
            joint_msg.name, joint_msg.position = action_joint_names_and_positions(joint_data)
            node.joint_publisher.publish(joint_msg)

            left_hand_msg = Float64MultiArray()
            left_hand_msg.data = hand_qpos(joint_data, "left")
            node.left_hand_publisher.publish(left_hand_msg)

            right_hand_msg = Float64MultiArray()
            right_hand_msg.data = hand_qpos(joint_data, "right")
            node.right_hand_publisher.publish(right_hand_msg)
    except KeyboardInterrupt:
        pass
    finally:
        samples = node.snapshot()
        save_samples(samples, output_path, args.state_topic)
        print()
        print(f"Saved {len(samples)} samples to {output_path}")
        if len(samples) == 0 and node.last_error:
            print(f"Last parse error: {node.last_error}")
        executor.shutdown()
        ros_thread.join(timeout=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record, plot, replay action with force recording, or analyse robot force sensor fields.")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--record", action="store_true", help="record force fields to an offline JSON file")
    mode.add_argument("--plot", action="store_true", help="plot an offline JSON record")
    mode.add_argument("--replay", type=Path, metavar="PATH", help="play an action JSON and record force data during the action")
    mode.add_argument("--analyse", action="store_true", help="compute RMS vs zero for the latest recording")
    parser.add_argument("path", nargs="?", type=Path, help="recording JSON for --plot or --analyse; default is latest recording")
    parser.add_argument("--state-topic", default=DEFAULT_STATE_TOPIC, help=f"default: {DEFAULT_STATE_TOPIC}")
    parser.add_argument("--duration", type=float, default=0.0, help="record duration in seconds; 0 means until Ctrl-C")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="directory for records; default is script/force_sensor/force_analyse_data",
    )
    parser.add_argument("--output", type=Path, default=None, help="explicit output JSON path for --record or --replay")
    parser.add_argument("--arm", choices=["primary", "secondary"], default="primary", help="arm slice to plot")
    parser.add_argument(
        "--show",
        nargs="+",
        choices=FORCE_FIELDS + EE_FIELDS,
        default=FORCE_FIELDS,
        help="fields visible initially in --plot; checkboxes can toggle them after the window opens",
    )
    args = parser.parse_args()
    if args.duration < 0.0:
        parser.error("--duration cannot be negative")
    if args.output is not None and not (args.record or args.replay is not None):
        parser.error("--output is only valid with --record or --replay")
    if args.path is not None and args.replay is not None:
        parser.error("use --replay PATH without positional path")
    return args


def main() -> None:
    args = parse_args()
    if args.record:
        record(args)
    elif args.analyse:
        analyse(args)
    elif args.replay is not None:
        replay_action_and_record(args)
    else:
        plot(args)


if __name__ == "__main__":
    main()
