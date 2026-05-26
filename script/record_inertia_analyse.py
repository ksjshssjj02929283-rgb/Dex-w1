#!/usr/bin/env python3
"""Record primary force sensor values around tool inertia compensation.

The output is written to script/inertia_analyse/<timestamp>.json by default.
Press Ctrl-C to stop, or pass --duration to stop automatically.
"""

from __future__ import annotations

import argparse
import json
import threading
import time
from pathlib import Path
from typing import List, Mapping

from inertia_analyse_utils import (
    default_output_dir,
    extract_force_sample,
    make_payload,
    timestamp_filename,
)

DEFAULT_STATE_TOPIC = "/feedback/robot_server_state"


def make_recorder_node_class(node_base, string_msg):
    class InertiaAnalyseRecorder(node_base):
        def __init__(self, state_topic: str):
            super().__init__("record_inertia_analyse")
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
                sample = extract_force_sample(data, "primary", now - self._t0)
            except Exception as exc:
                self.last_error = str(exc)
                return
            with self._lock:
                self.samples.append(sample)
            self.parsed += 1

        def snapshot(self) -> List[Mapping[str, object]]:
            with self._lock:
                return list(self.samples)

    return InertiaAnalyseRecorder


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record primary force data before/after tool inertia compensation.")
    parser.add_argument("--state-topic", default=DEFAULT_STATE_TOPIC, help=f"default: {DEFAULT_STATE_TOPIC}")
    parser.add_argument("--duration", type=float, default=0.0, help="record duration in seconds; 0 means until Ctrl-C")
    parser.add_argument("--output-dir", type=Path, default=default_output_dir(), help="directory for timestamp.json")
    parser.add_argument("--output", type=Path, default=None, help="explicit output JSON path")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output_path = args.output or (args.output_dir / timestamp_filename())
    output_path.parent.mkdir(parents=True, exist_ok=True)

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    recorder_node_class = make_recorder_node_class(Node, String)

    rclpy.init()
    node = recorder_node_class(args.state_topic)

    print(f"Recording primary force data from {args.state_topic}")
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


if __name__ == "__main__":
    main()
