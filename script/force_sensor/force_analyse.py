#!/usr/bin/env python3
"""Record and plot robot force-sensor state fields.

Records are written to script/force_sensor/force_analyse_data/<timestamp>.json
by default. Use --record to capture data from /feedback/robot_server_state and
--plot to visualize an existing record. Plotting without a path uses the latest
record in the default output directory.
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
FORCE_FIELDS = [
    "force_sensor",
    "force_sensor_gravity",
    "raw_force_sensor",
    "force_sensor_bias",
]
AXIS_LABELS = ["Fx [N]", "Fy [N]", "Fz [N]", "Tx [Nm]", "Ty [Nm]", "Tz [Nm]"]
FIELD_COLORS = {
    "force_sensor": "tab:blue",
    "force_sensor_gravity": "tab:green",
    "raw_force_sensor": "tab:gray",
    "force_sensor_bias": "tab:red",
}
FIELD_LABELS = {
    "force_sensor": "force_sensor",
    "force_sensor_gravity": "force_sensor_gravity",
    "raw_force_sensor": "raw_force_sensor",
    "force_sensor_bias": "force_sensor_bias",
}


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
        values = [float(item) for item in value]  # type: ignore[union-attr]
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
            "fields": FORCE_FIELDS,
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
        payload = make_payload(samples, state_topic=args.state_topic)
        with output_path.open("w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2, ensure_ascii=False)
            f.write("\n")
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
    fig.suptitle(f"{path.name} ({args.arm})")
    fig.tight_layout(rect=[0.0, 0.08, 1.0, 0.95])
    plt.show()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record or plot robot force sensor fields.")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--record", action="store_true", help="record force fields to an offline JSON file")
    mode.add_argument("--plot", action="store_true", help="plot an offline JSON record")
    parser.add_argument("path", nargs="?", type=Path, help="recording JSON for --plot; default is latest recording")
    parser.add_argument("--state-topic", default=DEFAULT_STATE_TOPIC, help=f"default: {DEFAULT_STATE_TOPIC}")
    parser.add_argument("--duration", type=float, default=0.0, help="record duration in seconds; 0 means until Ctrl-C")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="directory for records; default is script/force_sensor/force_analyse_data",
    )
    parser.add_argument("--output", type=Path, default=None, help="explicit output JSON path for --record")
    parser.add_argument("--arm", choices=["primary", "secondary"], default="primary", help="arm slice to plot")
    parser.add_argument(
        "--show",
        nargs="+",
        choices=FORCE_FIELDS,
        default=FORCE_FIELDS,
        help="fields visible initially in --plot; checkboxes can toggle them after the window opens",
    )
    args = parser.parse_args()
    if args.duration < 0.0:
        parser.error("--duration cannot be negative")
    if args.output is not None and args.plot:
        parser.error("--output is only valid with --record")
    return args


def main() -> None:
    args = parse_args()
    if args.record:
        record(args)
    else:
        plot(args)


if __name__ == "__main__":
    main()
