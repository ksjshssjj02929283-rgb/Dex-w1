#!/usr/bin/env python3
"""Online tool gravity compensation identification.

The file name keeps the requested spelling: ``gompensation``.
"""

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
from sensor_msgs.msg import Imu
from std_msgs.msg import String


DEFAULT_STATE_TOPIC = "/feedback/robot_server_state"
DEFAULT_IMU_TOPIC = "/imu/chassis/refined"
DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parent / "tool_gravity_compensation_data"
DEFAULT_IMU_ACC_STD_THRESHOLD = 0.04  # m/s^2
DEFAULT_IMU_GYRO_MEAN_THRESHOLD = 0.004  # rad/s


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


def map_sensor_wrench_to_ee(raw: np.ndarray) -> np.ndarray:
    """Map raw sensor order [-fx, -fy, fz, Mx, My, Mz] to EE [Fx,Fy,Fz,Mx,My,Mz]."""
    raw = np.asarray(raw, dtype=float)
    if raw.shape != (6,):
        raise ValueError(f"Expected 6D wrench, got shape {raw.shape}.")
    force = np.array([-raw[0], -raw[1], raw[2]], dtype=float)
    torque = np.array([raw[3], raw[4], raw[5]], dtype=float)
    return np.concatenate([force, torque])


def array_to_list(value: np.ndarray) -> List[float]:
    return [float(x) for x in np.asarray(value).reshape(-1)]


class ToolGravityNode(Node):
    def __init__(self, state_topic: str, imu_topic: str, arm: str):
        super().__init__("tool_gravity_gompensation")
        self.arm = arm
        self._lock = threading.Lock()
        self._imu_samples: Deque[Tuple[float, np.ndarray, np.ndarray]] = deque(maxlen=5000)
        self._state_samples: Deque[StateSample] = deque(maxlen=5000)
        self._state_parse_error_logged = False

        self.create_subscription(String, state_topic, self._state_callback, 10)
        self.create_subscription(Imu, imu_topic, self._imu_callback, 100)

    def _imu_callback(self, msg: Imu) -> None:
        acc = np.array(
            [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ],
            dtype=float,
        )
        gyro = np.array(
            [
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            ],
            dtype=float,
        )
        with self._lock:
            self._imu_samples.append((time.monotonic(), acc, gyro))

    def _state_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            force_sensor = np.asarray(data["force_sensor"], dtype=float)
            ee_pose = np.asarray(data["ee_pose"], dtype=float)
            timestamp = float(data.get("timestamp", 0.0))

            if force_sensor.size < 12:
                raise ValueError(f"force_sensor has {force_sensor.size} values, expected 12.")
            if ee_pose.size < 14:
                raise ValueError(f"ee_pose has {ee_pose.size} values, expected 14.")

            if self.arm == "primary":
                raw_wrench = force_sensor[:6]
                pose = ee_pose[:7]
            else:
                raw_wrench = force_sensor[6:12]
                pose = ee_pose[7:14]

            wrench_ee = map_sensor_wrench_to_ee(raw_wrench)
            rotation = quat_wxyz_to_rot(pose[3:7])
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
            self._state_parse_error_logged = False
        except Exception as exc:
            if not self._state_parse_error_logged:
                self.get_logger().warn(f"Failed to parse robot_server_state: {exc}")
                self._state_parse_error_logged = True

    def latest_state_age(self) -> Optional[float]:
        with self._lock:
            if not self._state_samples:
                return None
            return time.monotonic() - self._state_samples[-1].arrival_time

    def get_imu_window(self, window_sec: float) -> List[Tuple[float, np.ndarray, np.ndarray]]:
        cutoff = time.monotonic() - window_sec
        with self._lock:
            return [item for item in self._imu_samples if item[0] >= cutoff]


def wait_for_gravity(
    node: ToolGravityNode,
    imu_window: float,
    acc_std_threshold: float,
    gyro_mean_threshold: float,
) -> Tuple[float, Dict[str, float]]:
    print("等待 /imu/chassis/refined 稳定数据，请保持机器人静止...")
    last_report = 0.0
    while rclpy.ok():
        window = node.get_imu_window(imu_window)
        now = time.monotonic()
        if window:
            duration = window[-1][0] - window[0][0]
        else:
            duration = 0.0

        if duration >= imu_window * 0.8 and len(window) >= 10:
            acc = np.vstack([item[1] for item in window])
            gyro = np.vstack([item[2] for item in window])
            acc_norm = np.linalg.norm(acc, axis=1)
            gyro_norm = np.linalg.norm(gyro, axis=1)
            acc_std = float(np.std(acc_norm))
            gyro_mean = float(np.mean(gyro_norm))
            gravity = float(np.mean(acc_norm))

            if acc_std <= acc_std_threshold and gyro_mean <= gyro_mean_threshold:
                diag = {
                    "sample_count": float(len(window)),
                    "window_duration_s": float(duration),
                    "acc_norm_mean": gravity,
                    "acc_norm_std": acc_std,
                    "gyro_norm_mean": gyro_mean,
                    "acc_std_threshold": float(acc_std_threshold),
                    "gyro_mean_threshold": float(gyro_mean_threshold),
                }
                print(
                    f"IMU重力读取成功: g={gravity:.6f} m/s^2, "
                    f"acc_std={acc_std:.6f}, gyro_mean={gyro_mean:.6f}, samples={len(window)}"
                )
                return gravity, diag

            if now - last_report > 1.0:
                print(
                    f"IMU尚未静止: acc_std={acc_std:.6f}/{acc_std_threshold:.6f}, "
                    f"gyro_mean={gyro_mean:.6f}/{gyro_mean_threshold:.6f}, samples={len(window)}"
                )
                last_report = now
        elif now - last_report > 1.0:
            print(f"等待IMU窗口: {len(window)} samples, duration={duration:.2f}/{imu_window:.2f}s")
            last_report = now

        time.sleep(0.05)

    raise RuntimeError("ROS shutdown while waiting for IMU gravity.")


def wait_for_robot_state(node: ToolGravityNode) -> None:
    print("等待 /feedback/robot_server_state 数据...")
    last_report = 0.0
    while rclpy.ok():
        age = node.latest_state_age()
        if age is not None and age < 1.0:
            print("robot_server_state 已连接。")
            return
        now = time.monotonic()
        if now - last_report > 1.0:
            print("尚未收到有效 robot_server_state，请确认 robot server 正在运行。")
            last_report = now
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
    quat_mean = average_quaternions_wxyz(poses[:, 3:7])
    pose_mean = np.concatenate([poses[:, :3].mean(axis=0), quat_mean])
    rotation = quat_wxyz_to_rot(quat_mean)

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
    mass: float,
    gravity: float,
    model_sign: int,
) -> Dict[str, object]:
    g_world = np.array([0.0, 0.0, -gravity], dtype=float)
    forces_expected = []
    forces_measured = []
    torques_measured = []

    for sample in samples:
        expected_force = float(model_sign) * sample.rotation_world_from_ee.T @ (mass * g_world)
        forces_expected.append(expected_force)
        forces_measured.append(sample.wrench_ee_mean[:3])
        torques_measured.append(sample.wrench_ee_mean[3:6])

    forces_expected_arr = np.vstack(forces_expected)
    forces_measured_arr = np.vstack(forces_measured)
    torques_measured_arr = np.vstack(torques_measured)

    force_bias = np.mean(forces_measured_arr - forces_expected_arr, axis=0)
    force_residual = forces_measured_arr - (forces_expected_arr + force_bias)

    blocks = []
    y = []
    for expected_force, measured_torque in zip(forces_expected_arr, torques_measured_arr):
        # tau = com x F + tau_bias = -skew(F) * com + tau_bias
        blocks.append(np.hstack([-skew(expected_force), np.eye(3)]))
        y.append(measured_torque)
    A = np.vstack(blocks)
    b = np.concatenate(y)
    solution, _, rank, singular_values = np.linalg.lstsq(A, b, rcond=None)
    com = solution[:3]
    torque_bias = solution[3:6]

    torque_pred = []
    torque_residual = []
    for expected_force, measured_torque in zip(forces_expected_arr, torques_measured_arr):
        pred = np.cross(com, expected_force) + torque_bias
        torque_pred.append(pred)
        torque_residual.append(measured_torque - pred)
    torque_pred_arr = np.vstack(torque_pred)
    torque_residual_arr = np.vstack(torque_residual)

    compensated_force_norm = np.linalg.norm(forces_measured_arr - force_bias, axis=1)
    mass_estimates = compensated_force_norm / gravity
    condition = float(np.linalg.cond(A)) if A.size else float("inf")

    return {
        "gravity_magnitude": float(gravity),
        "mass_input": float(mass),
        "model_sign": int(model_sign),
        "com_m": array_to_list(com),
        "com_mm": array_to_list(com * 1000.0),
        "force_bias": array_to_list(force_bias),
        "torque_bias": array_to_list(torque_bias),
        "w_bias": array_to_list(np.concatenate([force_bias, torque_bias])),
        "rank": int(rank),
        "condition": condition,
        "singular_values": array_to_list(singular_values),
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
    print(f"mass_input:        {result['mass_input']:.6f} kg")
    print(f"model_sign:        {result['model_sign']}")
    print(f"com_m:             {np.array(result['com_m'])}")
    print(f"com_mm:            {np.array(result['com_mm'])}")
    print(f"force_bias [N]:    {np.array(result['force_bias'])}")
    print(f"torque_bias [Nm]:  {np.array(result['torque_bias'])}")
    print(f"w_bias:            {np.array(result['w_bias'])}")
    print(f"force_rms:         {result['force_rms']:.6f} N")
    print(f"torque_rms:        {result['torque_rms']:.6f} Nm")
    print(f"rank/condition:    {result['rank']} / {result['condition']:.6g}")
    mass_diag = result["mass_consistency"]
    print(
        "mass consistency: "
        f"mean={mass_diag['mean_kg']:.6f} kg, std={mass_diag['std_kg']:.6f} kg"
    )
    if result["rank"] < 6:
        print("WARNING: rank < 6, 姿态激励不足，质心/力矩零偏可能不可辨识。")
    if result["condition"] > 1e4:
        print("WARNING: condition number is high, 建议增加更多不同方向的姿态。")
    print("======================================================\n")


def run_self_test() -> None:
    rng = np.random.default_rng(7)
    mass = 1.25
    gravity = 9.79
    model_sign = 1
    true_com = np.array([0.03, -0.015, 0.08])
    true_bias = np.array([0.4, -0.2, 0.1, 0.03, -0.02, 0.01])

    samples = []
    for i in range(12):
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        angle = rng.uniform(-math.pi, math.pi)
        K = skew(axis)
        R = np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)
        F = model_sign * R.T @ (mass * np.array([0.0, 0.0, -gravity]))
        wrench = np.concatenate([F + true_bias[:3], np.cross(true_com, F) + true_bias[3:]])
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

    result = estimate_tool_parameters(samples, mass, gravity, model_sign)
    com_err = np.linalg.norm(np.array(result["com_m"]) - true_com)
    bias_err = np.linalg.norm(np.array(result["w_bias"]) - true_bias)
    print_result(result)
    print(f"self-test com_err={com_err:.6e}, bias_err={bias_err:.6e}")
    if com_err > 1e-8 or bias_err > 1e-8:
        raise RuntimeError("Synthetic self-test failed.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Identify tool COM and force sensor wrench bias online.")
    parser.add_argument("--mass", type=float, required=False, help="Known tool mass in kg.")
    parser.add_argument("--arm", choices=["primary", "secondary"], default="primary")
    parser.add_argument("--state-topic", default=DEFAULT_STATE_TOPIC)
    parser.add_argument("--imu-topic", default=DEFAULT_IMU_TOPIC)
    parser.add_argument("--avg-duration", type=float, default=1.0)
    parser.add_argument("--imu-window", type=float, default=2.0)
    parser.add_argument("--min-samples", type=int, default=6)
    parser.add_argument("--model-sign", type=int, choices=[-1, 1], default=1)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--imu-acc-std-threshold", type=float, default=DEFAULT_IMU_ACC_STD_THRESHOLD)
    parser.add_argument("--imu-gyro-mean-threshold", type=float, default=DEFAULT_IMU_GYRO_MEAN_THRESHOLD)
    parser.add_argument("--self-test", action="store_true", help="Run synthetic fitting test and exit.")
    args = parser.parse_args()

    if not args.self_test:
        if args.mass is None or args.mass <= 0.0:
            parser.error("--mass is required and must be positive.")
        if args.avg_duration <= 0.0:
            parser.error("--avg-duration must be positive.")
        if args.imu_window <= 0.0:
            parser.error("--imu-window must be positive.")
        if args.min_samples < 2:
            parser.error("--min-samples must be at least 2.")
    return args


def main() -> None:
    args = parse_args()
    if args.self_test:
        run_self_test()
        return

    rclpy.init()
    node = ToolGravityNode(args.state_topic, args.imu_topic, args.arm)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    samples: List[CapturedSample] = []
    try:
        gravity, imu_diag = wait_for_gravity(
            node,
            args.imu_window,
            args.imu_acc_std_threshold,
            args.imu_gyro_mean_threshold,
        )
        wait_for_robot_state(node)

        print("\n开始工具重力补偿采集。")
        print("操作: 摆好一个静止姿态后按 Enter 采样；输入 q 后回车结束并拟合。")
        print(f"当前设置: arm={args.arm}, mass={args.mass:.6f} kg, avg_duration={args.avg_duration:.3f}s")

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
                f"wrench_ee_mean={np.array2string(sample.wrench_ee_mean, precision=4)}"
            )

        result = estimate_tool_parameters(samples, args.mass, gravity, args.model_sign)
        print_result(result)

        args.output_dir.mkdir(parents=True, exist_ok=True)
        out_path = args.output_dir / f"tool_gravity_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        payload = {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "gravity_source": args.imu_topic,
            "gravity_magnitude": gravity,
            "imu_diagnostics": imu_diag,
            "config": {
                "mass": args.mass,
                "arm": args.arm,
                "state_topic": args.state_topic,
                "imu_topic": args.imu_topic,
                "avg_duration": args.avg_duration,
                "imu_window": args.imu_window,
                "min_samples": args.min_samples,
                "model_sign": args.model_sign,
                "wrench_mapping": "raw [-fx,-fy,fz,Mx,My,Mz] -> ee [Fx,Fy,Fz,Mx,My,Mz]",
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
