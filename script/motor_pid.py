#!/usr/bin/env python3
"""Read or write motor PID parameters via ethercat SDO commands."""

import argparse
import re
import subprocess
import sys
import time
from typing import Dict, List, Tuple

# Logical parameter name -> (index, subindex)
PID_MAP: Dict[str, Tuple[str, str]] = {
    "pos_kp": ("0x2382", "1"),
    "vel_kp": ("0x2381", "1"),
    "vel_ki": ("0x2381", "2"),
}


def run_cmd(cmd: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, check=False, capture_output=True, text=True)


def get_slave_positions() -> List[int]:
    result = run_cmd(["ethercat", "slaves"])
    if result.returncode != 0:
        print(result.stderr.strip() or "Failed to run 'ethercat slaves'", file=sys.stderr)
        return []

    positions: List[int] = []
    for line in result.stdout.splitlines():
        match = re.match(r"^\s*(\d+)\s", line)
        if match:
            positions.append(int(match.group(1)))
    return positions


def read_one_pid(position: int, param: str) -> Tuple[bool, str]:
    index, subindex = PID_MAP[param]
    cmd = ["ethercat", "upload", "-p", str(position), index, subindex]
    result = run_cmd(cmd)
    if result.returncode != 0:
        err = result.stderr.strip() or "ethercat upload failed"
        return False, err

    return True, result.stdout.strip()


def read_all_pid_for_position(position: int) -> int:
    print(f"[p={position}]")
    has_error = False
    for param in PID_MAP:
        ok, output = read_one_pid(position, param)
        if ok:
            print(f"{param}: {output}")
        else:
            has_error = True
            print(f"{param}: ERROR: {output}", file=sys.stderr)
    return 1 if has_error else 0


def read_pid(position: int | None, all_motors: bool) -> int:
    if all_motors:
        positions = get_slave_positions()
        if not positions:
            return 1

        has_error = False
        for idx, p in enumerate(positions):
            if idx > 0:
                print()
            if read_all_pid_for_position(p) != 0:
                has_error = True
        return 1 if has_error else 0

    if position is None:
        print("read mode requires --position or --all-motors", file=sys.stderr)
        return 1

    return read_all_pid_for_position(position)


def write_param(position: int, param: str, value: str) -> int:
    index, subindex = PID_MAP[param]
    cmd = ["ethercat", "download", "-p", str(position), index, subindex, value]
    result = run_cmd(cmd)
    if result.returncode != 0:
        print(result.stderr.strip() or "ethercat download failed", file=sys.stderr)
        return 1

    print(result.stdout.strip() or "OK")
    return 0


def parse_first_int(text: str) -> int | None:
    tokens = re.findall(r"0x[0-9A-Fa-f]+|\b\d+\b", text)
    for token in tokens:
        try:
            return int(token, 16) if token.lower().startswith("0x") else int(token)
        except ValueError:
            continue
    return None


def save_and_wait(position: int, timeout_s: float = 5.0, interval_s: float = 0.1) -> int:
    save_cmd = ["ethercat", "download", "-p", str(position), "0x1010", "1", "0x65766173"]
    save_result = run_cmd(save_cmd)
    if save_result.returncode != 0:
        print(save_result.stderr.strip() or "save command failed", file=sys.stderr)
        return 1

    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        check_cmd = ["ethercat", "upload", "-p", str(position), "0x1010", "1"]
        check_result = run_cmd(check_cmd)
        if check_result.returncode == 0:
            value = parse_first_int(check_result.stdout)
            if value == 2:
                print("save done: 0x1010:1 == 2")
                return 0
        time.sleep(interval_s)

    print("save timeout: 0x1010:1 did not become 2", file=sys.stderr)
    return 1


def write_all_pid_for_position(
    position: int,
    pos_kp: str,
    vel_kp: str,
    vel_ki: str,
) -> int:
    has_error = False
    for param, value in (("pos_kp", pos_kp), ("vel_kp", vel_kp), ("vel_ki", vel_ki)):
        ret = write_param(position, param, value)
        if ret != 0:
            has_error = True
    if has_error:
        return 1
    return save_and_wait(position)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read or write motor PID parameters using ethercat upload/download",
    )
    parser.add_argument(
        "action",
        choices=["read", "write", "r", "w"],
        help="read/r: read parameter(s), write/w: modify parameter",
    )
    parser.add_argument(
        "-p",
        "--position",
        type=int,
        help="joint position/slave position, e.g. 1",
    )
    parser.add_argument(
        "--all-motors",
        action="store_true",
        help="read PID of all motors (all slave positions)",
    )
    parser.add_argument(
        "write_args",
        nargs="*",
        help="write args: '<pos_kp> <vel_kp> <vel_ki>'",
    )
    return parser


def main() -> int:
    parser = build_parser()
    # Allow optional args (e.g. -p) to appear between positional write values.
    if hasattr(parser, "parse_intermixed_args"):
        args = parser.parse_intermixed_args()
    else:
        args = parser.parse_args()

    action = {"r": "read", "w": "write"}.get(args.action, args.action)

    if action == "read":
        if args.all_motors and args.position is not None:
            parser.error("--position and --all-motors cannot be used together")
        return read_pid(args.position, args.all_motors)

    if args.position is None:
        parser.error("--position is required when action=write")

    # Batch write only: write -p 1 <pos_kp> <vel_kp> <vel_ki>
    if len(args.write_args) == 3:
        return write_all_pid_for_position(
            args.position,
            args.write_args[0],
            args.write_args[1],
            args.write_args[2],
        )

    parser.error(
        "write usage: '-p N <pos_kp> <vel_kp> <vel_ki>'"
    )


if __name__ == "__main__":
    sys.exit(main())
