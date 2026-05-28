#!/usr/bin/env python3
"""通过末端动态激励辨识工具绕质心的转动惯量。

数据来源全部来自 /feedback/robot_server_state, F = ee_link 系:
  - force_sensor_gravity[0:6] : 已扣零偏 + 已扣工具静态重力的六维力 (primary)
  - ee_omega_ee_frame[0:3]    : 末端角速度 (server 1 kHz Butterworth 滤波后)
  - ee_alpha_ee_frame[0:3]    : 末端角加速度 (同上)

物理方程 (与 server 端 ApplyToolInertiaCompensationPrimary 注释一致):
    F_g = -m a_C
    T_g = -(I_C α_F + ω_F × I_C ω_F) + com × F_g
=>  I_C α_F + ω_F × I_C ω_F = com × F_g - T_g

Ixz = Iyz = 0 硬约束 (工具沿 z 轴展开) → 4-param LS.

输出: ^F I_C, kg·m^2.
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
from typing import Deque, Dict, Iterable, List, Optional, Tuple

import numpy as np

try:
    import scipy.signal
except Exception:  # pragma: no cover
    scipy = None


DEFAULT_STATE_TOPIC = "/feedback/robot_server_state"
DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parent / "tool_inertia_data"
DEFAULT_DT = 0.01
DEFAULT_TOOL_MASS_KG = 1.061
DEFAULT_TOOL_COM_MM = "-0.15804154,0.70721872,50.33428542"
DEFAULT_EXCITATION_DURATION = 120.0
DEFAULT_EXCITATION_AMPLITUDES = "0.15,0.30,0.20,0.15"
DEFAULT_EXCITATION_FREQUENCIES = "0.30,0.40,0.65,0.90"
DEFAULT_EXCITATION_PHASES = "0.0,1.57079632679,1.0471975512,0.52359877559"
DEFAULT_EXCITATION_RAMP = 2.0


@dataclass
class DynamicSample:
    arrival_time: float
    timestamp: float
    wrench_gravity_F: np.ndarray  # 6 维, F 系, 零偏+重力补偿后
    omega_F: np.ndarray           # 3 维, F 系
    alpha_F: np.ndarray           # 3 维, F 系


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def parse_float_list(text: str, expected: int, name: str) -> np.ndarray:
    values = [float(item.strip()) for item in text.replace(";", ",").split(",") if item.strip()]
    if expected > 0 and len(values) != expected:
        raise ValueError(f"{name} 需要 {expected} 个逗号分隔的数值, 实际得到 {len(values)} 个。")
    if expected < 0 and not values:
        raise ValueError(f"{name} 不能为空。")
    return np.asarray(values, dtype=float)


def array_to_list(value: np.ndarray) -> List[float]:
    return [float(x) for x in np.asarray(value).reshape(-1)]


def smooth_excitation_envelope(t: float, duration: float, ramp_sec: float) -> float:
    if duration <= 0.0:
        return 1.0
    if t < 0.0 or t > duration:
        return 0.0
    if ramp_sec <= 0.0:
        return 1.0
    edge = min(t, duration - t, ramp_sec)
    if edge >= ramp_sec:
        return 1.0
    ratio = max(0.0, min(1.0, edge / ramp_sec))
    return math.sin(0.5 * math.pi * ratio) ** 2


def arm_wrist_indices(arm: str, count: int = 3) -> List[int]:
    j7 = 12 if arm == "primary" else 19
    return list(range(j7 - count + 1, j7 + 1))


def wrist_indices_for_position(position: np.ndarray, arm: str, count: int = 3) -> List[int]:
    if np.asarray(position).size == 7:
        return list(range(7 - count, 7))
    return arm_wrist_indices(arm, count)


def generate_wrist_excitation_target(
    base_position: np.ndarray, arm: str, t: float, duration: float,
    amplitudes: np.ndarray, frequencies: np.ndarray, phases: np.ndarray, ramp_sec: float,
) -> np.ndarray:
    target = np.asarray(base_position, dtype=float).copy()
    count = len(amplitudes)
    envelope = smooth_excitation_envelope(t, duration, ramp_sec)
    offsets = envelope * amplitudes * np.sin(2.0 * np.pi * frequencies * t + phases)
    for idx, offset in zip(wrist_indices_for_position(target, arm, count), offsets):
        target[idx] += offset
    return target


def butter_filtfilt_2d(signal: np.ndarray, fc_hz: float, dt: float, order: int = 3) -> np.ndarray:
    if scipy is None or fc_hz <= 0:
        return signal.copy()
    wc = fc_hz * 2.0 * math.pi * dt
    if not (0.0 < wc < 1.0):
        return signal.copy()
    b, a = scipy.signal.butter(order, wc)
    out = np.empty_like(signal)
    for k in range(signal.shape[1]):
        out[:, k] = scipy.signal.filtfilt(b, a, signal[:, k], method="pad")
    return out


# ---------------------------------------------------------------------------
# regression
# ---------------------------------------------------------------------------

def theta_to_inertia_matrix(theta: np.ndarray) -> np.ndarray:
    t = np.asarray(theta, dtype=float).ravel()
    return np.array(
        [[t[0], t[3], 0.0],
         [t[3], t[1], 0.0],
         [0.0, 0.0, t[2]]],
        dtype=float,
    )


def _inertia_regressor_cols(omega: np.ndarray, alpha: np.ndarray) -> np.ndarray:
    """4 cols: Ixx, Iyy, Izz, Ixy (Ixz=Iyz=0 hard-constrained)."""
    cols = []
    basis = np.zeros((3, 3))
    # Ixx
    basis[0, 0] = 1.0
    cols.append(basis @ alpha + np.cross(omega, basis @ omega))
    basis[0, 0] = 0.0
    # Iyy
    basis[1, 1] = 1.0
    cols.append(basis @ alpha + np.cross(omega, basis @ omega))
    basis[1, 1] = 0.0
    # Izz
    basis[2, 2] = 1.0
    cols.append(basis @ alpha + np.cross(omega, basis @ omega))
    basis[2, 2] = 0.0
    # Ixy
    basis[0, 1] = basis[1, 0] = 1.0
    cols.append(basis @ alpha + np.cross(omega, basis @ omega))
    return np.column_stack(cols)


def fit_inertia(
    omega: np.ndarray, alpha: np.ndarray, torque_rot: np.ndarray,
) -> Dict[str, object]:
    """4-param LS: I·α + ω×I·ω = torque_rot  (Ixz=Iyz=0)."""
    omega = np.asarray(omega, dtype=float)
    alpha = np.asarray(alpha, dtype=float)
    torque_rot = np.asarray(torque_rot, dtype=float)
    n = omega.shape[0]
    if not (omega.shape == alpha.shape == torque_rot.shape == (n, 3)):
        raise ValueError("omega, alpha, torque_rot 都必须是 shape=(N, 3)。")

    rows = [_inertia_regressor_cols(omega[i], alpha[i]) for i in range(n)]
    y = np.vstack(rows)                        # (3N, 4)
    b = torque_rot.reshape(-1)                 # (3N,)

    theta, _, rank, sv = np.linalg.lstsq(y, b, rcond=None)
    pred = (y @ theta).reshape((-1, 3))
    err = torque_rot - pred

    I_est = theta_to_inertia_matrix(theta)
    eigvals = np.linalg.eigvalsh(I_est)
    return {
        "theta": theta,                        # [Ixx, Iyy, Izz, Ixy]
        "inertia_com": I_est,
        "rank": int(rank),
        "condition_number": float(np.linalg.cond(y)) if y.size else float("inf"),
        "singular_values": sv,
        "torque_rms": float(np.sqrt(np.mean(err ** 2))),
        "physical": {
            "positive_definite": bool(np.all(eigvals > 0.0)),
            "triangle_inequality": bool(
                I_est[0, 0] + I_est[1, 1] >= I_est[2, 2]
                and I_est[0, 0] + I_est[2, 2] >= I_est[1, 1]
                and I_est[1, 1] + I_est[2, 2] >= I_est[0, 0]
            ),
            "eigenvalues": array_to_list(eigvals),
        },
    }


def torque_rotational_component(
    torque_g: np.ndarray, force_g: np.ndarray, com_m: np.ndarray,
) -> np.ndarray:
    """com × F_g − T_g."""
    com_m = np.asarray(com_m, dtype=float).ravel()
    return np.cross(com_m, force_g) - torque_g


def prepare_fit_arrays(
    samples: List[DynamicSample], com_m: np.ndarray,
    trim_start_sec: float, trim_end_sec: float, dt: float,
    extra_filter_fc_hz: Optional[float] = None,
) -> Dict[str, np.ndarray]:
    if len(samples) < 20:
        raise ValueError("动态采样点至少需要 20 个。")
    wrench_g = np.vstack([s.wrench_gravity_F for s in samples])
    omega = np.vstack([s.omega_F for s in samples])
    alpha = np.vstack([s.alpha_F for s in samples])

    if not np.all(np.isfinite(omega)) or not np.all(np.isfinite(alpha)):
        n_bad = int(np.sum(~np.isfinite(omega).all(axis=1) | ~np.isfinite(alpha).all(axis=1)))
        raise ValueError(
            f"omega/alpha 中存在 {n_bad} 帧 NaN, 通常是运动学预处理无效, "
            "请确认 server 处于 Idle/Running。"
        )

    if extra_filter_fc_hz is not None and extra_filter_fc_hz > 0:
        wrench_g = np.hstack([
            butter_filtfilt_2d(wrench_g[:, :3], extra_filter_fc_hz, dt),
            butter_filtfilt_2d(wrench_g[:, 3:6], extra_filter_fc_hz, dt),
        ])

    F_g = wrench_g[:, :3]
    T_g = wrench_g[:, 3:6]
    torque_rot = torque_rotational_component(T_g, F_g, com_m)

    si = max(0, int(round(trim_start_sec / dt)))
    ei = len(samples) - max(0, int(round(trim_end_sec / dt)))
    if ei <= si + 10:
        raise ValueError("首尾裁剪后剩余样本太少。")
    return {
        "omega": omega[si:ei],
        "alpha": alpha[si:ei],
        "torque_rot": torque_rot[si:ei],
    }


# ---------------------------------------------------------------------------
# data pipeline
# ---------------------------------------------------------------------------

def _slice_arm(arr: np.ndarray, arm: str) -> np.ndarray:
    arr = np.asarray(arr, dtype=float)
    if arr.size in (6, 12):
        per = arr.size // 2
        return arr[:per].copy() if arm == "primary" else arr[per:per * 2].copy()
    raise ValueError(f"无法从长度 {arr.size} 的数组按 arm 切片。")


def sample_from_state_dict(data: Dict[str, object], arm: str) -> DynamicSample:
    timestamp_raw = float(data.get("timestamp", 0.0))
    timestamp = timestamp_raw * 1e-9 if timestamp_raw > 1e12 else timestamp_raw

    for field in ("force_sensor_gravity", "ee_omega_ee_frame", "ee_alpha_ee_frame"):
        if field not in data:
            raise ValueError(f"{field} 字段缺失, 请升级 server。")
    w = _slice_arm(data["force_sensor_gravity"], arm)
    om = _slice_arm(data["ee_omega_ee_frame"], arm)
    al = _slice_arm(data["ee_alpha_ee_frame"], arm)
    if w.size != 6 or om.size != 3 or al.size != 3:
        raise ValueError(f"wrench/ω/α 维度异常: {w.size}/{om.size}/{al.size}")
    return DynamicSample(time.monotonic(), timestamp, w, om, al)


def sample_to_json(sample: DynamicSample) -> Dict[str, object]:
    return {
        "timestamp": sample.timestamp,
        "wrench_gravity_F": array_to_list(sample.wrench_gravity_F),
        "omega_F": array_to_list(sample.omega_F),
        "alpha_F": array_to_list(sample.alpha_F),
    }


def samples_from_jsonl(path: Path) -> List[DynamicSample]:
    samples: List[DynamicSample] = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            if not line.strip():
                continue
            data = json.loads(line)
            for key in ("wrench_gravity_F", "omega_F", "alpha_F"):
                if key not in data:
                    raise ValueError(f"jsonl 缺少字段 {key}, 请用新版重新采集。")
            samples.append(DynamicSample(
                arrival_time=time.monotonic(),
                timestamp=float(data.get("timestamp", 0.0)),
                wrench_gravity_F=np.asarray(data["wrench_gravity_F"], dtype=float),
                omega_F=np.asarray(data["omega_F"], dtype=float),
                alpha_F=np.asarray(data["alpha_F"], dtype=float),
            ))
    return samples


def write_jsonl(path: Path, samples: Iterable[DynamicSample]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        for sample in samples:
            f.write(json.dumps(sample_to_json(sample), ensure_ascii=False) + "\n")


# ---------------------------------------------------------------------------
# online capture node
# ---------------------------------------------------------------------------

class ToolInertiaNode:

    def __init__(self, state_topic: str, arm: str):
        import rclpy
        from joint_interfaces.msg import JointPositionControl
        from rclpy.node import Node
        from std_msgs.msg import Header, String

        class _Node(Node):
            def __init__(self, outer):
                super().__init__("tool_inertia_estimate")
                self.outer = outer
                self.create_subscription(String, state_topic, self._state_callback, 10)
                self.publisher = self.create_publisher(JointPositionControl, "/control/joint_position", 10)

            def _state_callback(self, msg):
                try:
                    data = json.loads(msg.data)
                    sample = sample_from_state_dict(data, arm)
                except Exception as exc:
                    self.outer.errors.append(str(exc))
                    return
                try:
                    jp = np.asarray(data["joint_position"], dtype=float)
                    jn = list(data.get("joint_name", []))
                except Exception as exc:
                    self.outer.errors.append(f"解析 joint_position/joint_name 失败: {exc}")
                    return
                with self.outer.lock:
                    self.outer.samples.append(sample)
                    self.outer.latest_joint_position = jp.copy()
                    self.outer.latest_joint_name = jn

            def publish_joint_position(self, target_position: np.ndarray):
                msg = JointPositionControl()
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.position = [float(x) for x in np.asarray(target_position).reshape(-1)]
                msg.name = self.outer.get_joint_name()
                self.publisher.publish(msg)

        self.rclpy = rclpy
        self.lock = threading.Lock()
        self.samples: Deque[DynamicSample] = deque(maxlen=200000)
        self.errors: Deque[str] = deque(maxlen=20)
        self.latest_joint_position: Optional[np.ndarray] = None
        self.latest_joint_name: List[str] = []
        self.node = _Node(self)

    def latest_age(self) -> Optional[float]:
        with self.lock:
            if not self.samples:
                return None
            return time.monotonic() - self.samples[-1].arrival_time

    def snapshot_since(self, start_time: float) -> List[DynamicSample]:
        with self.lock:
            return [s for s in self.samples if s.arrival_time >= start_time]

    def get_joint_position(self) -> Optional[np.ndarray]:
        with self.lock:
            return None if self.latest_joint_position is None else self.latest_joint_position.copy()

    def get_joint_name(self) -> List[str]:
        with self.lock:
            return list(self.latest_joint_name)

    def publish_joint_position(self, target_position: np.ndarray) -> None:
        self.node.publish_joint_position(target_position)


def capture_online(args: argparse.Namespace) -> List[DynamicSample]:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init()
    wrapper = ToolInertiaNode(args.state_topic, args.arm)
    executor = MultiThreadedExecutor()
    executor.add_node(wrapper.node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        print(f"等待 {args.state_topic} 数据...")
        start_wait = time.monotonic()
        while wrapper.latest_age() is None or wrapper.latest_age() > 1.0:
            if time.monotonic() - start_wait > 10.0:
                detail = wrapper.errors[-1] if wrapper.errors else "没有解析到消息"
                raise RuntimeError(f"等待 robot_server_state 超时: {detail}")
            time.sleep(0.05)

        if args.no_excitation:
            input("数据已连接。按 Enter 开始记录 > ")
            start = time.monotonic()
            if args.duration > 0.0:
                print(f"记录中: {args.duration:.2f}s")
                time.sleep(args.duration)
            else:
                input("记录中。完成动态激励后按 Enter 停止 > ")
            samples = wrapper.snapshot_since(start)
            if not samples:
                raise RuntimeError("没有采集到样本。")
            return samples

        base_position = wrapper.get_joint_position()
        if base_position is None:
            raise RuntimeError("没有获取到当前关节位置。")
        if not wrapper.get_joint_name():
            raise RuntimeError("没有获取到 joint_name。")

        amplitudes = parse_float_list(args.excitation_amplitudes, -1, "--excitation-amplitudes")
        frequencies = parse_float_list(args.excitation_frequencies, -1, "--excitation-frequencies")
        phases = parse_float_list(args.excitation_phases, -1, "--excitation-phases")
        if not (len(amplitudes) == len(frequencies) == len(phases) and len(amplitudes) in (3, 4)):
            raise RuntimeError("excitation 数组长度必须相等且为 3 或 4。")
        count = len(amplitudes)
        wrist = wrist_indices_for_position(base_position, args.arm, count)
        if not wrist or max(wrist) >= base_position.size:
            raise RuntimeError(f"关节数组长度 {base_position.size} 不支持 {count} 关节激励。")

        print(f"激励关节: {wrist}  幅值: {amplitudes}  频率: {frequencies}  相位: {phases}")
        print(f"时长: {args.duration:.2f}s")
        input("确认安全后按 Enter 开始 > ")

        start = time.monotonic()
        next_t = time.monotonic()
        print("自动激励中...")
        while True:
            t = time.monotonic() - start
            if t > args.duration:
                break
            target = generate_wrist_excitation_target(
                base_position, args.arm, t, args.duration,
                amplitudes, frequencies, phases, args.excitation_ramp,
            )
            wrapper.publish_joint_position(target)
            next_t += args.dt
            time.sleep(max(0.0, next_t - time.monotonic()))

        print("保持中心姿态 1 秒。")
        hold_end = time.monotonic() + 1.0
        while time.monotonic() < hold_end:
            wrapper.publish_joint_position(base_position)
            time.sleep(args.dt)

        samples = wrapper.snapshot_since(start)
        if not samples:
            raise RuntimeError("没有采集到样本。")
        return samples
    finally:
        executor.shutdown()
        wrapper.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# ---------------------------------------------------------------------------
# output
# ---------------------------------------------------------------------------

def result_to_json(
    result: Dict[str, object], args: argparse.Namespace, sample_count: int,
) -> Dict[str, object]:
    inertia = np.asarray(result["inertia_com"], dtype=float)
    theta = np.asarray(result["theta"], dtype=float)
    com_mm = parse_float_list(args.com_mm, 3, "--com-mm")
    return {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "sample_count": int(sample_count),
        "arm": args.arm,
        "dt": args.dt,
        "wrench_source": "force_sensor_gravity (零偏+重力补偿后, F 系)",
        "kinematics_source": "ee_omega/alpha_ee_frame (server 1 kHz Butterworth, F 系)",
        "mass_kg": args.mass,
        "com_mm": array_to_list(com_mm),
        "theta_order": ["Ixx", "Iyy", "Izz", "Ixy"],
        "theta": array_to_list(theta),
        "inertia_com_kg_m2": inertia.tolist(),
        "rank": f"{result['rank']} / 4",
        "condition_number": result["condition_number"],
        "torque_rms": result["torque_rms"],
        "physical": result["physical"],
    }


def print_result(payload: Dict[str, object]) -> None:
    print("\n========== 工具质心转动惯量辨识结果 ==========")
    print(f"有效样本数:        {payload['sample_count']}")
    print(f"辨识机械臂:        {payload['arm']}")
    print(f"六维力来源:        {payload['wrench_source']}")
    print(f"运动学来源:        {payload['kinematics_source']}")
    print(f"工具质量 kg:       {payload['mass_kg']}")
    print(f"质心偏移 mm:       {np.array(payload['com_mm'])}")
    print(f"参数顺序:          {payload['theta_order']}")
    print(f"参数向量:          {np.array(payload['theta'])}")
    print(f"^F I_C kg*m^2:\n{np.array(payload['inertia_com_kg_m2'])}")
    print(f"力矩残差 RMS:      {float(payload['torque_rms']):.6g} Nm")
    print(f"回归矩阵秩:        {payload['rank']}")
    print(f"条件数:            {float(payload['condition_number']):.6g}")
    p = payload["physical"]
    print(f"正定检查:          {p['positive_definite']}")
    print(f"三角不等式检查:    {p['triangle_inequality']}")
    print("\n===== robot_config.yaml 建议字段 =====")
    I = np.array(payload["inertia_com_kg_m2"])
    print(f"  inertia_com_kg_m2: [[{I[0,0]:.8g}, {I[0,1]:.8g}, {I[0,2]:.8g}],")
    print(f"                     [{I[1,0]:.8g}, {I[1,1]:.8g}, {I[1,2]:.8g}],")
    print(f"                     [{I[2,0]:.8g}, {I[2,1]:.8g}, {I[2,2]:.8g}]]")
    print("============================================\n")


# ---------------------------------------------------------------------------
# self-test
# ---------------------------------------------------------------------------

def run_self_test() -> None:
    rng = np.random.default_rng(42)
    I_true = np.array([[0.018, 0.002, 0.0],
                       [0.002, 0.026, 0.0],
                       [0.0,   0.0,   0.031]], dtype=float)
    com = np.array([0.001, -0.002, 0.05])
    mass = 1.0

    n, dt = 4000, 0.005
    t = np.arange(n) * dt
    omega = np.column_stack([
        0.30 * np.sin(2.0 * np.pi * 0.40 * t + 0.0),
        0.25 * np.sin(2.0 * np.pi * 0.65 * t + 1.0),
        0.20 * np.sin(2.0 * np.pi * 0.90 * t + 2.0),
    ])
    alpha = np.column_stack([
        0.30 * (2.0 * np.pi * 0.40) * np.cos(2.0 * np.pi * 0.40 * t + 0.0),
        0.25 * (2.0 * np.pi * 0.65) * np.cos(2.0 * np.pi * 0.65 * t + 1.0),
        0.20 * (2.0 * np.pi * 0.90) * np.cos(2.0 * np.pi * 0.90 * t + 2.0),
    ])

    a_C = rng.normal(scale=0.5, size=(n, 3))
    F_g = -mass * a_C
    T_g = -np.array([
        I_true @ alpha[i] + np.cross(omega[i], I_true @ omega[i]) for i in range(n)
    ]) + np.cross(com, F_g)

    torque_rot = torque_rotational_component(T_g, F_g, com)
    result = fit_inertia(omega, alpha, torque_rot)
    theta = np.asarray(result["theta"])
    err = np.linalg.norm(theta - np.array([0.018, 0.026, 0.031, 0.002]))
    print(f"self-test: theta_err={err:.6e}  torque_rms={result['torque_rms']:.6e}")
    if err > 1e-10:
        raise RuntimeError(f"self-test 失败: theta={theta}")


# ---------------------------------------------------------------------------
# CLI & main
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="辨识工具绕质心的转动惯量 ^F I_C (4-param LS, F 系, Ixz=Iyz=0)。",
    )
    parser.add_argument("--arm", choices=["primary", "secondary"], default="primary")
    parser.add_argument("--state-topic", default=DEFAULT_STATE_TOPIC)
    parser.add_argument("--input", type=Path, help="已有 jsonl 文件; 不填则在线采集。")
    parser.add_argument("--duration", type=float, default=DEFAULT_EXCITATION_DURATION)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--dt", type=float, default=DEFAULT_DT)
    parser.add_argument("--mass", type=float, default=DEFAULT_TOOL_MASS_KG)
    parser.add_argument("--com-mm", default=DEFAULT_TOOL_COM_MM,
                        help="质心偏移 mm (x,y,z, F 系, 来自 tool_com_estimate.py)。")
    parser.add_argument("--no-excitation", action="store_true",
                        help="不自动下发激励, 只采集。")
    parser.add_argument("--excitation-amplitudes", default=DEFAULT_EXCITATION_AMPLITUDES)
    parser.add_argument("--excitation-frequencies", default=DEFAULT_EXCITATION_FREQUENCIES)
    parser.add_argument("--excitation-phases", default=DEFAULT_EXCITATION_PHASES)
    parser.add_argument("--excitation-ramp", type=float, default=DEFAULT_EXCITATION_RAMP)
    parser.add_argument("--trim-start", type=float, default=1.0)
    parser.add_argument("--trim-end", type=float, default=1.0)
    parser.add_argument("--filter-fc", type=float, default=0.0,
                        help="可选: 仅对 wrench 再做零相位 Butterworth (Hz)。默认 0。")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        return args
    if args.dt <= 0.0:
        parser.error("--dt 必须为正数。")
    if args.mass <= 0.0:
        parser.error("--mass 必须为正数。")
    if args.duration <= 0.0 and not args.no_excitation:
        parser.error("自动激励模式下 --duration 必须为正数。")
    if args.excitation_ramp < 0.0:
        parser.error("--excitation-ramp 不能为负数。")
    parse_float_list(args.com_mm, 3, "--com-mm")
    amps = parse_float_list(args.excitation_amplitudes, -1, "--excitation-amplitudes")
    freqs = parse_float_list(args.excitation_frequencies, -1, "--excitation-frequencies")
    phs = parse_float_list(args.excitation_phases, -1, "--excitation-phases")
    if not (len(amps) == len(freqs) == len(phs) and len(amps) in (3, 4)):
        parser.error("excitation 数组长度必须相等且为 3 或 4。")
    if np.any(amps < 0.0):
        parser.error("--excitation-amplitudes 不能包含负数。")
    if np.any(freqs <= 0.0):
        parser.error("--excitation-frequencies 必须全部为正数。")
    return args


def main() -> None:
    args = parse_args()
    if args.self_test:
        run_self_test()
        return

    print("\n========== 工具惯量辨识 ==========")
    print(f"机械臂: {args.arm}  dt: {args.dt}s  mass: {args.mass}kg  com: {args.com_mm}mm")
    if args.input:
        print("数据: 离线文件")
    elif args.no_excitation:
        print("数据: 手动采集")
    else:
        print(f"激励: 幅值={args.excitation_amplitudes}  频率={args.excitation_frequencies}")
        print(f"  相位={args.excitation_phases}  时长={args.duration}s")
    print("====================================\n")

    if args.input:
        samples = samples_from_jsonl(args.input.expanduser())
        capture_path = args.input.expanduser()
    else:
        samples = capture_online(args)
        capture_path = args.output_dir / f"tool_inertia_capture_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl"
        write_jsonl(capture_path, samples)
        print(f"已保存: {capture_path}")

    com_m = parse_float_list(args.com_mm, 3, "--com-mm") / 1000.0
    extra_fc = args.filter_fc if args.filter_fc and args.filter_fc > 0 else None
    arrays = prepare_fit_arrays(
        samples, com_m=com_m,
        trim_start_sec=args.trim_start, trim_end_sec=args.trim_end, dt=args.dt,
        extra_filter_fc_hz=extra_fc,
    )
    print(f"拟合中, 有效样本: {len(arrays['omega'])}")
    result = fit_inertia(arrays["omega"], arrays["alpha"], arrays["torque_rot"])
    payload = result_to_json(result, args, len(arrays["omega"]))
    payload["capture_path"] = str(capture_path)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    result_path = args.output_dir / f"tool_inertia_result_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    with result_path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
    print_result(payload)
    print(f"结果已保存: {result_path}")


if __name__ == "__main__":
    main()
