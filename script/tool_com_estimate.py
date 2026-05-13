#!/usr/bin/env python3
"""Online tool COM identification with known mass and known wrench bias."""

import argparse
import json
import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Deque, Dict, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_STATE_TOPIC = "/feedback/robot_server_state"
DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parent / "tool_gravity_compensation_data"
DEFAULT_GRAVITY = 9.79  # m/s^2
DEFAULT_KNOWN_MASS = 1.061  # kg
DEFAULT_W_BIAS_RAW_SRI = np.array([13.2234, -21.4832, -24.0719, 0.1601, -0.4670, -0.6611], dtype=float)


@dataclass
class StateSample:
    arrival_time: float
    timestamp: float
    ee_pose: np.ndarray
    raw_wrench: np.ndarray
    wrench_ee: np.ndarray
    rotation_world_from_ee: np.ndarray


@dataclass
class CapturedSample:
    index: int
    timestamp: float
    count: int
    ee_pose_mean: np.ndarray
    raw_wrench_mean: np.ndarray
    raw_wrench_std: np.ndarray
    wrench_ee_mean: np.ndarray
    wrench_ee_std: np.ndarray
    rotation_world_from_ee: np.ndarray


def skew(v: np.ndarray) -> np.ndarray:
    return np.array(
        [
            [0.0, -v[2], v[1]],
            [v[2], 0.0, -v[0]],
            [-v[1], v[0], 0.0],
        ],
        dtype=float,
    )


def quat_wxyz_to_rot(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if norm <= 0.0:
        raise ValueError("Quaternion norm is zero.")
    w, x, y, z = q / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def euler_xyz_to_rot(euler: np.ndarray) -> np.ndarray:
    """Convert Euler angles (roll, pitch, yaw in radians, XYZ order) to rotation matrix."""
    euler = np.asarray(euler, dtype=float)
    if euler.shape != (3,):
        raise ValueError(f"Expected 3D Euler angles, got {euler.shape}")
    
    rx, ry, rz = euler
    
    # Rotation around X axis (roll)
    Rx = np.array([
        [1.0, 0.0, 0.0],
        [0.0, np.cos(rx), -np.sin(rx)],
        [0.0, np.sin(rx), np.cos(rx)],
    ], dtype=float)
    
    # Rotation around Y axis (pitch)
    Ry = np.array([
        [np.cos(ry), 0.0, np.sin(ry)],
        [0.0, 1.0, 0.0],
        [-np.sin(ry), 0.0, np.cos(ry)],
    ], dtype=float)
    
    # Rotation around Z axis (yaw)
    Rz = np.array([
        [np.cos(rz), -np.sin(rz), 0.0],
        [np.sin(rz), np.cos(rz), 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=float)
    
    # Combined rotation: Z * Y * X (intrinsic) or X * Y * Z (extrinsic)
    return Rz @ Ry @ Rx


def average_quaternions_wxyz(quaternions: np.ndarray) -> np.ndarray:
    if quaternions.size == 0:
        raise ValueError("No quaternion samples.")
    qs = np.asarray(quaternions, dtype=float).copy()
    first = qs[0]
    for i in range(qs.shape[0]):
        if np.dot(first, qs[i]) < 0.0:
            qs[i] *= -1.0
    q = qs.mean(axis=0)
    norm = np.linalg.norm(q)
    if norm <= 0.0:
        raise ValueError("Averaged quaternion norm is zero.")
    return q / norm


def map_sri_raw_wrench_to_ee(raw: np.ndarray) -> np.ndarray:
    """Map raw SRI order [-Fx, -Fy, Fz, -Mx, -My, Mz] to EE [Fx, Fy, Fz, Mx, My, Mz]."""
    raw = np.asarray(raw, dtype=float)
    if raw.shape != (6,):
        raise ValueError(f"Expected 6D wrench, got shape {raw.shape}.")
    force = np.array([-raw[0], -raw[1], raw[2]], dtype=float)
    torque = np.array([-raw[3], -raw[4], raw[5]], dtype=float)
    return np.concatenate([force, torque])





def array_to_list(value: np.ndarray) -> List[float]:
    return [float(x) for x in np.asarray(value).reshape(-1)]


class ToolGravityNode(Node):
    """只订阅 robot_server_state，不再订阅 IMU。"""

    def __init__(self, state_topic: str, arm: str, wrench_format: str):
        super().__init__("tool_gravity_gompensation")
        self.arm = arm
        self.wrench_format = wrench_format
        self._lock = threading.Lock()
        self._state_samples: Deque[StateSample] = deque(maxlen=5000)
        self._received_count = 0
        self._parsed_count = 0
        self._error_counts = {}

        print(f"[DEBUG] 订阅话题: {state_topic}")
        self.create_subscription(String, state_topic, self._state_callback, 10)

    def _state_callback(self, msg: String) -> None:
        self._received_count += 1
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self._record_error("json_decode", str(e))
            return
        
        try:
            key = "raw_force_sensor" if self.wrench_format == "raw_sri" and "raw_force_sensor" in data else "force_sensor"
            force_sensor = np.asarray(data[key], dtype=float)
        except (KeyError, ValueError, TypeError) as e:
            self._record_error("force_sensor_parse", str(e))
            return
        
        try:
            ee_pose = np.asarray(data["ee_pose"], dtype=float)
        except (KeyError, ValueError, TypeError) as e:
            self._record_error("ee_pose_parse", str(e))
            return

        try:
            timestamp = float(data.get("timestamp", 0.0))
            
            if force_sensor.size < 12:
                raise ValueError(f"force_sensor has {force_sensor.size} values, expected 12.")
            
            # Extract wrench for correct arm
            if self.arm == "primary":
                raw_wrench = force_sensor[:6]
            else:
                raw_wrench = force_sensor[6:12]
            
            # Support both 14-value (7+7 quat) and 12-value (6+6 euler/RPY) formats
            if ee_pose.size >= 14:
                # Original format: 7 values per arm (x,y,z,qw,qx,qy,qz)
                if self.arm == "primary":
                    pose = ee_pose[:7]
                else:
                    pose = ee_pose[7:14]
            elif ee_pose.size == 12:
                # New format: 6 values per arm (x,y,z,rx,ry,rz)
                if self.arm == "primary":
                    pose = ee_pose[:6]
                else:
                    pose = ee_pose[6:12]
            else:
                raise ValueError(f"ee_pose has {ee_pose.size} values, expected 12 or 14.")

            if self.wrench_format == "state_mapped":
                wrench_ee = raw_wrench.copy()
            elif self.wrench_format == "raw_sri":
                wrench_ee = map_sri_raw_wrench_to_ee(raw_wrench)
            else:
                raise ValueError(f"Unsupported wrench format: {self.wrench_format}")
            
            # Handle both quaternion (7-value) and Euler angle (6-value) formats
            if len(pose) == 7:
                # Quaternion format: [x, y, z, qw, qx, qy, qz]
                rotation = quat_wxyz_to_rot(pose[3:7])
            elif len(pose) == 6:
                # Euler angle format: [x, y, z, rx, ry, rz]
                rotation = euler_xyz_to_rot(pose[3:6])
            else:
                raise ValueError(f"Unexpected pose format: {len(pose)} values")
            
            sample = StateSample(
                arrival_time=time.monotonic(),
                timestamp=timestamp,
                ee_pose=pose.copy(),
                raw_wrench=raw_wrench.copy(),
                wrench_ee=wrench_ee,
                rotation_world_from_ee=rotation,
            )
            with self._lock:
                self._state_samples.append(sample)
            self._parsed_count += 1
            
            if self._parsed_count == 1:
                print(f"[DEBUG] 成功解析第一条消息! force_sensor: {force_sensor.shape}, ee_pose: {ee_pose.shape}, pose_format={'quat' if len(pose)==7 else 'euler'}")
        except Exception as e:
            self._record_error("processing", str(e))
    
    def _record_error(self, error_type: str, detail: str) -> None:
        if error_type not in self._error_counts:
            self._error_counts[error_type] = {"count": 0, "last_msg": ""}
        self._error_counts[error_type]["count"] += 1
        self._error_counts[error_type]["last_msg"] = detail

    def latest_state_age(self) -> Optional[float]:
        with self._lock:
            if not self._state_samples:
                return None
            return time.monotonic() - self._state_samples[-1].arrival_time
    
    def get_message_count(self) -> int:
        """Return total number of messages received."""
        return self._received_count
    
    def get_diagnostics(self) -> str:
        """Return diagnostic info about message processing."""
        diag = f"raw_received={self._received_count}, parsed={self._parsed_count}"
        if self._error_counts:
            errors_str = "; ".join(
                f"{k}={v['count']}({v['last_msg'][:40]})"
                for k, v in self._error_counts.items()
            )
            diag += f" | errors: {errors_str}"
        return diag


def wait_for_robot_state(node: ToolGravityNode, timeout_sec: float = 10.0) -> None:
    print("等待 /feedback/robot_server_state 数据...")
    print("  (如果持续超时，运行此诊断命令:)")
    print("    ros2 topic list | grep robot_server")
    print("    ros2 topic echo /feedback/robot_server_state --once")
    last_report = 0.0
    start = time.monotonic()
    while rclpy.ok():
        age = node.latest_state_age()
        diag = node.get_diagnostics()
        now = time.monotonic()
        
        if age is not None and age < 1.0:
            print(f"✓ robot_server_state 已连接 ({diag})")
            return
        
        if now - last_report > 1.0:
            elapsed = now - start
            print(f"  [{elapsed:.1f}s] {diag}")
            last_report = now
        
        if now - start > timeout_sec:
            print(f"✗ 等待超时 ({timeout_sec}s)")
            print(f"诊断: {diag}")
            if node._received_count == 0:
                print("  → 消息未到达，请检查:")
                print("     1. robot server 是否运行: ps aux | grep robot_server")
                print("     2. 话题是否存在: ros2 topic list | grep robot_server")
            elif node._parsed_count == 0:
                print("  → 消息到达但全部解析失败，问题可能是:")
                if node._error_counts:
                    for error_type, info in node._error_counts.items():
                        print(f"     - {error_type}: {info['last_msg']}")
                print("  建议: ros2 topic echo /feedback/robot_server_state --once")
                print("        检查消息 JSON 结构和字段名是否正确")
            raise RuntimeError("Timeout waiting for robot_server_state.")
        
        time.sleep(0.05)
    raise RuntimeError("ROS shutdown while waiting for robot_server_state.")


def capture_sample(node: ToolGravityNode, index: int, duration_sec: float) -> CapturedSample:
    start_time = time.monotonic()
    deadline = start_time + duration_sec
    while time.monotonic() < deadline and rclpy.ok():
        time.sleep(min(0.02, max(0.0, deadline - time.monotonic())))

    with node._lock:
        samples = [item for item in node._state_samples if item.arrival_time >= start_time]

    if not samples:
        raise RuntimeError("No robot_server_state samples received.")

    raw = np.vstack([sample.raw_wrench for sample in samples])
    wrench = np.vstack([sample.wrench_ee for sample in samples])
    poses = np.vstack([sample.ee_pose for sample in samples])
    
    pose_len = poses.shape[1]
    if pose_len == 7:
        # Quaternion format: average using quaternion algebra
        quat_mean = average_quaternions_wxyz(poses[:, 3:7])
        pose_mean = np.concatenate([poses[:, :3].mean(axis=0), quat_mean])
        rotation = quat_wxyz_to_rot(quat_mean)
    elif pose_len == 6:
        # Euler angle format: simple averaging (valid for small sample windows)
        pose_mean = poses.mean(axis=0)
        rotation = euler_xyz_to_rot(pose_mean[3:6])
    else:
        raise ValueError(f"Unexpected pose format in capture: {pose_len} values")

    return CapturedSample(
        index=index,
        timestamp=float(time.time()),
        count=len(samples),
        ee_pose_mean=pose_mean,
        raw_wrench_mean=raw.mean(axis=0),
        raw_wrench_std=raw.std(axis=0),
        wrench_ee_mean=wrench.mean(axis=0),
        wrench_ee_std=wrench.std(axis=0),
        rotation_world_from_ee=rotation,
    )


def estimate_tool_parameters(
    samples: List[CapturedSample],
    gravity: float,
    model_sign: int,
    known_mass: float,
    w_bias_raw_sri: np.ndarray,
) -> Dict[str, object]:
    """Estimate only COM with known mass and known wrench bias."""
    known_mass = float(known_mass)
    w_bias_ee = map_sri_raw_wrench_to_ee(np.asarray(w_bias_raw_sri, dtype=float))
    force_bias = w_bias_ee[:3].copy()
    torque_bias = w_bias_ee[3:6].copy()

    g_world = np.array([0.0, 0.0, -gravity], dtype=float)
    forces_expected_arr = np.vstack(
        [float(model_sign) * sample.rotation_world_from_ee.T @ (known_mass * g_world) for sample in samples]
    )
    forces_measured_arr = np.vstack([s.wrench_ee_mean[:3] for s in samples])
    torques_measured_arr = np.vstack([s.wrench_ee_mean[3:6] for s in samples])

    force_residual = forces_measured_arr - (forces_expected_arr + force_bias)

    blocks = []
    targets = []
    for expected_force, measured_torque in zip(forces_expected_arr, torques_measured_arr):
        blocks.append(-skew(expected_force))
        targets.append(measured_torque - torque_bias)
    A_torque = np.vstack(blocks)
    b_torque = np.concatenate(targets)
    com, _, rank_torque, sv_torque = np.linalg.lstsq(A_torque, b_torque, rcond=None)

    torque_pred_arr = np.vstack([np.cross(com, f) + torque_bias for f in forces_expected_arr])
    torque_residual_arr = torques_measured_arr - torque_pred_arr

    compensated_force_norm = np.linalg.norm(forces_measured_arr - force_bias, axis=1)
    mass_estimates = compensated_force_norm / gravity

    condition_torque = float(np.linalg.cond(A_torque)) if A_torque.size else float("inf")

    return {
        "gravity_magnitude": float(gravity),
        "known_mass": known_mass,
        "model_sign": int(model_sign),
        "com_m": array_to_list(com),
        "com_mm": array_to_list(com * 1000.0),
        "force_bias_known": array_to_list(force_bias),
        "torque_bias_known": array_to_list(torque_bias),
        "w_bias_raw_sri": array_to_list(w_bias_raw_sri),
        "rank_torque": int(rank_torque),
        "condition_torque": condition_torque,
        "singular_values_torque": array_to_list(sv_torque),
        "force_rms": float(np.sqrt(np.mean(force_residual**2))),
        "torque_rms": float(np.sqrt(np.mean(torque_residual_arr**2))),
        "mass_consistency": {
            "mean_kg": float(np.mean(mass_estimates)),
            "std_kg": float(np.std(mass_estimates)),
            "per_sample_kg": array_to_list(mass_estimates),
        },
        "per_sample": [
            {
                "index": sample.index,
                "expected_force": array_to_list(forces_expected_arr[i]),
                "measured_force": array_to_list(forces_measured_arr[i]),
                "force_residual": array_to_list(force_residual[i]),
                "measured_torque": array_to_list(torques_measured_arr[i]),
                "predicted_torque": array_to_list(torque_pred_arr[i]),
                "torque_residual": array_to_list(torque_residual_arr[i]),
            }
            for i, sample in enumerate(samples)
        ],
    }


def sample_to_json(sample: CapturedSample) -> Dict[str, object]:
    return {
        "index": sample.index,
        "timestamp": sample.timestamp,
        "count": sample.count,
        "ee_pose_mean": array_to_list(sample.ee_pose_mean),
        "raw_wrench_mean_sensor_order": array_to_list(sample.raw_wrench_mean),
        "raw_wrench_std_sensor_order": array_to_list(sample.raw_wrench_std),
        "wrench_ee_mean": array_to_list(sample.wrench_ee_mean),
        "wrench_ee_std": array_to_list(sample.wrench_ee_std),
        "rotation_world_from_ee": sample.rotation_world_from_ee.tolist(),
    }


def print_result(result: Dict[str, object]) -> None:
    print("\n========== Tool Gravity Compensation Result ==========")
    print(f"gravity_magnitude: {result['gravity_magnitude']:.6f} m/s^2")
    print(f"known_mass:        {result['known_mass']:.6f} kg")
    print(f"model_sign:        {result['model_sign']}")
    print(f"com_m (estimated): {np.array(result['com_m'])}")
    print(f"com_mm (estimated):{np.array(result['com_mm'])}")
    print(f"force_bias [N]  (known, EE):  {np.array(result['force_bias_known'])}")
    print(f"torque_bias [Nm](known, EE):  {np.array(result['torque_bias_known'])}")
    print(f"w_bias_raw_sri (known):       {np.array(result['w_bias_raw_sri'])}")
    print(f"force_rms:         {result['force_rms']:.6f} N")
    print(f"torque_rms:        {result['torque_rms']:.6f} Nm")
    print(f"rank_torque:       {result['rank_torque']}")
    print(f"condition_torque:  {result['condition_torque']:.6g}")
    mass_diag = result["mass_consistency"]
    print(
        "mass consistency: "
        f"mean={mass_diag['mean_kg']:.6f} kg, std={mass_diag['std_kg']:.6f} kg"
    )
    if result["rank_torque"] < 3:
        print("WARNING: rank < 3, 姿态激励不足，质心可能不可辨识。")
    if result["condition_torque"] > 1e4:
        print("WARNING: condition number is high, 建议增加更多不同方向的姿态。")
    print("")
    print("===== robot_config.yaml 配置 (直接复制) =====")
    com_mm = np.array(result['com_mm'])
    w_bias_raw = np.array(result['w_bias_raw_sri'])
    print(f"tool_gravity_compensation:")
    print(f"  com_mm: [{com_mm[0]:.5f}, {com_mm[1]:.5f}, {com_mm[2]:.5f}]")
    print(f"  w_bias: [{w_bias_raw[0]:.5f}, {w_bias_raw[1]:.5f}, {w_bias_raw[2]:.5f}, {w_bias_raw[3]:.5f}, {w_bias_raw[4]:.5f}, {w_bias_raw[5]:.5f}]")
    print(f"  mass_kg: {result['known_mass']:.6f}")
    print(f"  gravity_magnitude: {result['gravity_magnitude']}")
    print(f"  model_sign: {result['model_sign']}")
    print("======================================================\n")


def run_self_test() -> None:
    rng = np.random.default_rng(7)
    gravity = 9.79
    model_sign = 1
    true_mass = 1.061
    true_com = np.array([0.03, -0.015, 0.08])
    true_force_bias = np.array([-13.2234, 21.4832, -24.0719], dtype=float)
    true_torque_bias = np.array([-0.1601, 0.4670, -0.6611], dtype=float)

    samples = []
    for i in range(12):
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        angle = rng.uniform(-math.pi, math.pi)
        K = skew(axis)
        R = np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)
        F = model_sign * R.T @ (true_mass * np.array([0.0, 0.0, -gravity]))
        wrench = np.concatenate([F + true_force_bias, np.cross(true_com, F) + true_torque_bias])
        pose = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
        samples.append(
            CapturedSample(
                index=i + 1,
                timestamp=time.time(),
                count=1,
                ee_pose_mean=pose,
                raw_wrench_mean=np.zeros(6),
                raw_wrench_std=np.zeros(6),
                wrench_ee_mean=wrench,
                wrench_ee_std=np.zeros(6),
                rotation_world_from_ee=R,
            )
        )

    result = estimate_tool_parameters(
        samples,
        gravity,
        model_sign,
        known_mass=true_mass,
        w_bias_raw_sri=np.array([
            -true_force_bias[0], -true_force_bias[1], true_force_bias[2],
            -true_torque_bias[0], -true_torque_bias[1], true_torque_bias[2],
        ]),
    )
    com_err = np.linalg.norm(np.array(result["com_m"]) - true_com)
    print_result(result)
    print(f"self-test com_err={com_err:.6e}")
    if com_err > 1e-8:
        raise RuntimeError("Synthetic self-test failed.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Identify tool COM online with known mass and known wrench bias.")
    parser.add_argument("--arm", choices=["primary", "secondary"], default="primary")
    parser.add_argument("--state-topic", default=DEFAULT_STATE_TOPIC)
    parser.add_argument(
        "--wrench-format",
        choices=["state_mapped", "raw_sri"],
        default="raw_sri",
        help=(
            "Interpretation of force_sensor data. "
            "Default raw_sri means force_sensor is interpreted as "
            "[-Fx, -Fy, Fz, -Mx, -My, Mz]. "
            "Use state_mapped only when the input is already EE-frame wrench [Fx, Fy, Fz, Mx, My, Mz]."
        ),
    )
    parser.add_argument("--gravity", type=float, default=DEFAULT_GRAVITY,
                        help=f"Fixed gravity magnitude in m/s^2 (default: {DEFAULT_GRAVITY}).")
    parser.add_argument("--known-mass", type=float, default=DEFAULT_KNOWN_MASS,
                        help=f"Known tool mass in kg (default: {DEFAULT_KNOWN_MASS}).")
    parser.add_argument("--w-bias", type=float, nargs=6, default=DEFAULT_W_BIAS_RAW_SRI.tolist(),
                        metavar=("FX", "FY", "FZ", "MX", "MY", "MZ"),
                        help="Known wrench bias in raw SRI sensor order. Default matches robot_config.yaml.")
    parser.add_argument("--avg-duration", type=float, default=1.0)
    parser.add_argument("--min-samples", type=int, default=6)
    parser.add_argument("--model-sign", type=int, choices=[-1, 1], default=1)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--self-test", action="store_true", help="Run synthetic fitting test and exit.")
    args = parser.parse_args()

    if not args.self_test:
        if args.avg_duration <= 0.0:
            parser.error("--avg-duration must be positive.")
        if args.min_samples < 2:
            parser.error("--min-samples must be at least 2.")
        if args.gravity <= 0.0:
            parser.error("--gravity must be positive.")
        if args.known_mass <= 0.0:
            parser.error("--known-mass must be positive.")
    return args


def main() -> None:
    args = parse_args()
    if args.self_test:
        run_self_test()
        return

    rclpy.init()
    node = ToolGravityNode(args.state_topic, args.arm, args.wrench_format)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    samples: List[CapturedSample] = []
    try:
        wait_for_robot_state(node)

        # 使用固定重力加速度
        gravity = args.gravity
        imu_diag = {"source": "fixed_command_line", "value": gravity}
        print(f"\n使用固定重力加速度: {gravity:.6f} m/s^2")

        print("\n开始工具重力补偿采集。")
        print("操作: 摆好一个静止姿态后按 Enter 采样；输入 q 后回车结束并拟合。")
        print(f"当前设置: arm={args.arm}, known_mass={args.known_mass:.6f} kg, avg_duration={args.avg_duration:.3f}s")
        print(f"已知 wrench 零偏(raw SRI): {np.array(args.w_bias, dtype=float)}")
        print("将只估计工具质心偏移 COM。建议采集 >= 6 个姿态差异较大的位姿。")
        if args.wrench_format == "raw_sri":
            print("当前 force_sensor 按原始 SRI 数组处理: 将执行 [-Fx,-Fy,Fz,-Mx,-My,Mz] -> [Fx,Fy,Fz,Mx,My,Mz] 映射。")
        else:
            print("当前 force_sensor 作为 EE 坐标系 wrench 直接使用，不再做二次映射。")
            print("辨识时请确保运行时工具重力补偿关闭，否则发布出来的是已补偿数据。")

        while rclpy.ok():
            cmd = input(f"[{len(samples)} samples] Enter=采样, q=拟合退出 > ").strip().lower()
            if cmd == "q":
                if len(samples) < args.min_samples:
                    print(f"样本不足: {len(samples)}/{args.min_samples}，请继续采集更多不同姿态。")
                    continue
                break
            if cmd not in ("", "c", "capture"):
                print("未知输入；按 Enter 采样，或输入 q 结束。")
                continue

            print(f"采集中，请保持静止 {args.avg_duration:.2f}s ...")
            try:
                sample = capture_sample(node, len(samples) + 1, args.avg_duration)
            except RuntimeError as exc:
                print(f"采样失败: {exc}")
                continue
            samples.append(sample)
            print(
                f"已采集 #{sample.index}: count={sample.count}, "
                f"raw_wrench_mean={np.array2string(sample.raw_wrench_mean, precision=4)}, "
                f"wrench_ee_mean={np.array2string(sample.wrench_ee_mean, precision=4)}, "
                f"ee_pose_mean={np.array2string(sample.ee_pose_mean, precision=5)}"
            )

        result = estimate_tool_parameters(
            samples,
            gravity,
            args.model_sign,
            known_mass=args.known_mass,
            w_bias_raw_sri=np.array(args.w_bias, dtype=float),
        )
        print_result(result)

        args.output_dir.mkdir(parents=True, exist_ok=True)
        out_path = args.output_dir / f"tool_gravity_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        payload = {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "gravity_source": "fixed_command_line",
            "gravity_magnitude": gravity,
            "imu_diagnostics": imu_diag,
            "config": {
                "arm": args.arm,
                "state_topic": args.state_topic,
                "wrench_format": args.wrench_format,
                "gravity": args.gravity,
                "known_mass": args.known_mass,
                "w_bias_raw_sri": array_to_list(np.array(args.w_bias, dtype=float)),
                "avg_duration": args.avg_duration,
                "min_samples": args.min_samples,
                "model_sign": args.model_sign,
                "wrench_mapping": (
                    "state_mapped: use force_sensor directly as EE wrench"
                    if args.wrench_format == "state_mapped"
                    else "raw_sri: [-Fx,-Fy,Fz,-Mx,-My,Mz] -> [Fx,Fy,Fz,Mx,My,Mz]"
                ),
            },
            "result": result,
            "samples": [sample_to_json(sample) for sample in samples],
        }
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2, ensure_ascii=False)
        print(f"结果已保存: {out_path}")
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
