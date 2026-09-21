#!/usr/bin/env python3
"""Measure teleoperation command and chassis-motor feedback latency.

The script starts three `ros2 topic echo` subprocesses and timestamps their
local stdout reception. Source the ROS 2 workspace before running it.
"""

import argparse
import ast
import queue
import signal
import subprocess
import sys
import threading
import time
import re


def start_echo(topic, field):
    command = ["ros2", "topic", "echo", topic, "--field", field]
    return subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                            text=True, bufsize=1)


def read_lines(name, process, output_queue):
    try:
        joy_lines = []
        feedback_lines = []
        for line in process.stdout:
            stripped = line.strip()
            if name == "joy":
                if stripped.startswith("array(") or ("[" in stripped and "]" in stripped):
                    output_queue.put((name, time.monotonic(), stripped))
                    joy_lines.clear()
                    continue
                if stripped.startswith("---"):
                    if joy_lines:
                        output_queue.put((name, time.monotonic(), "[" + ",".join(joy_lines) + "]"))
                        joy_lines.clear()
                    continue
                match = re.match(r"^-?\s*([-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)$", stripped)
                if match:
                    joy_lines.append(match.group(1))
                elif joy_lines and not stripped:
                    output_queue.put((name, time.monotonic(), "[" + ",".join(joy_lines) + "]"))
                    joy_lines.clear()
            elif name == "feedback":
                # ros2 topic echo prints array fields such as speed_mps as:
                # - 0.1\n- 0.2\n---. Collect one complete array first.
                if stripped.startswith("---"):
                    if feedback_lines:
                        output_queue.put(
                            (name, time.monotonic(), "[" + ",".join(feedback_lines) + "]"))
                        feedback_lines.clear()
                    continue
                if stripped.startswith("array(") or ("[" in stripped and "]" in stripped):
                    output_queue.put((name, time.monotonic(), stripped))
                    feedback_lines.clear()
                    continue
                match = re.match(r"^-\s*([-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)$", stripped)
                if match:
                    feedback_lines.append(match.group(1))
                elif feedback_lines and not stripped:
                    output_queue.put(
                        (name, time.monotonic(), "[" + ",".join(feedback_lines) + "]"))
                    feedback_lines.clear()
            else:
                output_queue.put((name, time.monotonic(), stripped))
        if joy_lines:
            output_queue.put((name, time.monotonic(), "[" + ",".join(joy_lines) + "]"))
        if feedback_lines:
            output_queue.put((name, time.monotonic(), "[" + ",".join(feedback_lines) + "]"))
    finally:
        output_queue.put((name, None, None))


def parse_scalar(value):
    try:
        return float(value)
    except ValueError:
        return None


def parse_numeric_values(value):
    """Parse a scalar or a ROS 2 array('d', [...])/[...] into numeric values."""
    try:
        array_match = re.search(r"\[(.*?)\]", value)
        if array_match:
            return [float(item.strip()) for item in array_match.group(1).split(",") if item.strip()]
        return [float(value)]
    except (ValueError, TypeError):
        return None


def parse_joy_axis(value, axis):
    try:
        array_match = re.search(r"\[(.*?)\]", value)
        if array_match:
            values = [float(item.strip()) for item in array_match.group(1).split(",") if item.strip()]
            return values[axis]
        axes = ast.literal_eval(value)
        return float(axes[axis])
    except (ValueError, SyntaxError, TypeError, IndexError):
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--joy-axis", type=int, default=1,
                        help="/joy axes index used for the test (default: 1)")
    parser.add_argument("--cmd-field", default="linear.x",
                        help="Twist field to measure (default: linear.x; use angular.z for yaw)")
    parser.add_argument("--feedback-topic", default="/feedback/chassis_motor",
                        help="Chassis motor feedback topic (default: /feedback/chassis_motor)")
    parser.add_argument("--feedback-field", default="speed_mps",
                        help="Motor feedback field (default: speed_mps; use speed_rpm if needed)")
    parser.add_argument("--joy-threshold", type=float, default=0.1,
                        help="absolute joystick threshold (default: 0.1)")
    parser.add_argument("--cmd-threshold", type=float, default=0.02,
                        help="absolute cmd_vel threshold in m/s or rad/s (default: 0.02)")
    parser.add_argument("--feedback-threshold", type=float, default=0.02,
                        help="absolute motor-feedback threshold; arrays use max absolute element (default: 0.02)")
    parser.add_argument("--settle-timeout", type=float, default=3.0,
                        help="seconds before an incomplete event is discarded (default: 3)")
    args = parser.parse_args()

    processes = {
        "joy": start_echo("/joy", "axes"),
        "cmd": start_echo("/control/cmd_vel", args.cmd_field),
        "feedback": start_echo(args.feedback_topic, args.feedback_field),
    }
    events = queue.Queue()
    threads = [threading.Thread(target=read_lines, args=(name, proc, events), daemon=True)
               for name, proc in processes.items()]
    for thread in threads:
        thread.start()

    print("Listening. Move the selected joystick axis once, hold it briefly, then release.", flush=True)
    print("Press Ctrl-C to stop. Local receipt times are used; this is a relative latency test.", flush=True)
    samples = []
    state = {"joy": False, "cmd": False, "feedback": False}
    current = {}
    last_activity = time.monotonic()

    try:
        while True:
            try:
                name, timestamp, raw = events.get(timeout=0.2)
            except queue.Empty:
                if current and time.monotonic() - last_activity > args.settle_timeout:
                    print("Incomplete event discarded (timeout).", flush=True)
                    current.clear()
                    state = {key: False for key in state}
                continue
            if timestamp is None:
                print(f"{name} topic echo exited; check that the topic exists and has a publisher.", flush=True)
                continue

            if name == "joy":
                value = parse_joy_axis(raw, args.joy_axis)
            elif name == "feedback":
                values = parse_numeric_values(raw)
                value = max((abs(item) for item in values), default=0.0) if values is not None else None
            else:
                value = parse_scalar(raw)
            if value is None:
                continue
            last_activity = timestamp
            threshold = {"joy": args.joy_threshold, "cmd": args.cmd_threshold,
                         "feedback": args.feedback_threshold}[name]
            active = abs(value) >= threshold

            # Joy is the sole event trigger. cmd/feedback noise before a user
            # action must never create or complete an event.
            if name == "joy":
                if active and not state["joy"]:
                    state["joy"] = True
                    current = {"joy": timestamp}
                    print(f"joy  active: value={value:.4f}", flush=True)
                elif not active and state["joy"]:
                    state["joy"] = False
                
            elif state["joy"] and active and name not in current:
                current[name] = timestamp
                print(f"{name:4s} active: value={value:.4f}", flush=True)

            if not state["joy"] and all(key in current for key in ("joy", "cmd", "feedback")):
                if current["joy"] <= current["cmd"] <= current["feedback"]:
                    result = {
                        "joy_to_cmd": current["cmd"] - current["joy"],
                        "cmd_to_feedback": current["feedback"] - current["cmd"],
                    }
                    samples.append(result)
                    print("SUCCESS #{count}: joy->cmd={joy_to_cmd:.3f} ms, "
                          "cmd->feedback={cmd_to_feedback:.3f} ms".format(
                              count=len(samples),
                              **{key: value * 1000.0 for key, value in result.items()}), flush=True)
                else:
                    print("INVALID event discarded: timestamps are out of order "
                          f"(joy={current['joy']:.6f}, cmd={current['cmd']:.6f}, "
                          f"feedback={current['feedback']:.6f})", flush=True)
                current.clear()
                state = {key: False for key in state}
    except KeyboardInterrupt:
        pass
    finally:
        for process in processes.values():
            process.send_signal(signal.SIGTERM)
        for process in processes.values():
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()

    if samples:
        print(f"\nCompleted events: {len(samples)}")
        for key in ("joy_to_cmd", "cmd_to_feedback"):
            values = [sample[key] * 1000.0 for sample in samples]
            values_sorted = sorted(values)
            p95 = values_sorted[min(len(values_sorted) - 1,
                                    int(0.95 * len(values_sorted)))]
            print(f"{key:12s}: avg={sum(values) / len(values):.1f} ms, "
                  f"min={min(values):.1f} ms, max={max(values):.1f} ms, p95={p95:.1f} ms")
    else:
        print("No complete event was measured.")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
