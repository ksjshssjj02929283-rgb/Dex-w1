#!/usr/bin/env python3
"""通过末端动态激励联合辨识工具绕质心的转动惯量与质心偏移。

数据来源全部来自 /feedback/robot_server_state, F = ee_link 系:
  - force_sensor[0:6]      : 零偏后、重力补偿前的六维力 (primary)
  - ee_omega_ee_frame[0:3] : 末端角速度 (server 1 kHz Butterworth 滤波后)
  - ee_alpha_ee_frame[0:3] : 末端角加速度 (同上)

物理方程 (使用未扣重力的 unbiased wrench):
    F_unb = m·R^T·g_W  −  m·a_C           (含重力 + 动态力)
    T_unb = com × F_unb  −  (I·α + ω×I·ω)
=>  I·α + ω×I·ω  +  skew(F_unb)·com  =  −T_unb

Ixz = Iyz = 0 硬约束 → 联合估计 4 个惯量参数 + 3 个 COM 参数 = 7 params.

com 的可观测性来自 F_unb 中的静态重力分量 (随姿态变化), 与 tool_com_estimate.py 物理意义一致.

输出: ^F I_C (kg·m^2) 与 com (mm, F 系).
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

_PARAM_NAMES = ["Ixx", "Iyy", "Izz", "Ixy", "cx", "cy", "cz"]
_COM_WARN_DIFF_MM = 3.0  # 拟合 COM 与输入 COM 偏差超过此值告警


@dataclass
class DynamicSample:
    arrival_time: float
    timestamp: float
    wrench_unbiased_F: np.ndarray  # 6 维, F 系, 零偏后重力补偿前
    omega_F: np.ndarray            # 3 维, F 系
    alpha_F: np.ndarray            # 3 维, F 系


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
    """theta[:6] → 3×3 symmetric inertia matrix. theta may have >6 elements (com appended)."""
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
    for k in [0, 1, 2, 3]:
        basis = np.zeros((3, 3))
        if k == 0:
            basis[0, 0] = 1.0
        elif k == 1:
            basis[1, 1] = 1.0
        elif k == 2:
            basis[2, 2] = 1.0
        else:
            basis[0, 1] = basis[1, 0] = 1.0
        cols.append(basis @ alpha + np.cross(omega, basis @ omega))
    return np.column_stack(cols)


def _com_regressor_cols(F_g: np.ndarray) -> np.ndarray:
    """3 cols for com: skew(F_g) = [[0,-Fz,Fy],[Fz,0,-Fx],[-Fy,Fx,0]]."""
    F = np.asarray(F_g, dtype=float).ravel()
    return np.array([
        [0.0, -F[2], F[1]],
        [F[2], 0.0, -F[0]],
        [-F[1], F[0], 0.0],
    ], dtype=float)


def fit_inertia_and_com(
    omega: np.ndarray,
    alpha: np.ndarray,
    F_unb: np.ndarray,
    T_unb: np.ndarray,
) -> Dict[str, object]:
    """Joint LS: I·α + ω×I·ω + skew(F_unb)·com = -T_unb  (7 params)."""
    omega = np.asarray(omega, dtype=float)
    alpha = np.asarray(alpha, dtype=float)
    F_unb = np.asarray(F_unb, dtype=float)
    T_unb = np.asarray(T_unb, dtype=float)
    n = omega.shape[0]
    if not (omega.shape == alpha.shape == F_unb.shape == T_unb.shape == (n, 3)):
        raise ValueError("omega, alpha, F_unb, T_unb 都必须是 shape=(N, 3)。")

    rows = []
    for i in range(n):
        y_i = _inertia_regressor_cols(omega[i], alpha[i])
        y_com = _com_regressor_cols(F_unb[i])
        rows.append(np.hstack([y_i, y_com]))
    y = np.vstack(rows)                       # (3N, 7)
    b = -T_unb.reshape(-1)                    # (3N,)

    theta, _, rank, sv = np.linalg.lstsq(y, b, rcond=None)
    pred = (y @ theta).reshape((-1, 3))
    err = (-T_unb) - pred

    I_est = theta_to_inertia_matrix(theta)
    com_est = theta[4:7].copy()
    eigvals = np.linalg.eigvalsh(I_est)
    return {
        "theta": theta,
        "inertia_com": I_est,
        "com_fitted_m": com_est,
        "rank": int(rank),
        "condition_number": float(np.linalg.cond(y)) if y.size else float("inf"),
        "singular_values": sv,
        "torque_rms": float(np.sqrt(np.mean(err ** 2))),
        "physical": {
            "symmetric": True,
            "positive_definite": bool(np.all(eigvals > 0.0)),
            "triangle_inequality": bool(
                I_est[0, 0] + I_est[1, 1] >= I_est[2, 2]
                and I_est[0, 0] + I_est[2, 2] >= I_est[1, 1]
                and I_est[1, 1] + I_est[2, 2] >= I_est[0, 0]
            ),
            "eigenvalues": array_to_list(eigvals),
        },
    }


# ---------------------------------------------------------------------------
# data pipeline
# ---------------------------------------------------------------------------

def prepare_fit_arrays(
    samples: List[DynamicSample],
    trim_start_sec: float, trim_end_sec: float, dt: float,
    extra_filter_fc_hz: Optional[float] = None,
) -> Dict[str, np.ndarray]:
    if len(samples) < 20:
        raise ValueError("动态采样点至少需要 20 个。")
    wrench_g = np.vstack([s.wrench_unbiased_F for s in samples])
    omega = np.vstack([s.omega_F for s in samples])
    alpha = np.vstack([s.alpha_F for s in samples])

    if not np.all(np.isfinite(omega)) or not np.all(np.isfinite(alpha)):
        n_bad = int(np.sum(~np.isfinite(omega).all(axis=1) | ~np.isfinite(alpha).all(axis=1)))
        raise ValueError(
            f"omega/alpha 中存在 {n_bad} 帧 NaN, 通常是 server 端运动学预处理判定无效 "
            "(manager 不在 Idle/Running, 或反馈失效, 或首拍). 请确认 server 处于 Idle/Running。"
        )

    if extra_filter_fc_hz is not None and extra_filter_fc_hz > 0:
        wrench_g = np.hstack([
            butter_filtfilt_2d(wrench_g[:, :3], extra_filter_fc_hz, dt),
            butter_filtfilt_2d(wrench_g[:, 3:6], extra_filter_fc_hz, dt),
        ])

    si = max(0, int(round(trim_start_sec / dt)))
    ei = len(samples) - max(0, int(round(trim_end_sec / dt)))
    if ei <= si + 10:
        raise ValueError("首尾裁剪后剩余样本太少, 请缩短 --trim-start/--trim-end 或增加采集时长。")
    return {
        "omega": omega[si:ei],
        "alpha": alpha[si:ei],
        "F_unb": wrench_g[si:ei, :3],
        "T_unb": wrench_g[si:ei, 3:6],
    }


def _slice_arm(arr: np.ndarray, arm: str) -> np.ndarray:
    """12 维 (per=6) 或 6 维 (per=3): primary=左半, secondary=右半."""
    arr = np.asarray(arr, dtype=float)
    if arr.size in (6, 12):
        per = arr.size // 2
        return arr[:per].copy() if arm == "primary" else arr[per:per * 2].copy()
    raise ValueError(f"无法从长度 {arr.size} 的数组按 arm 切片。")


def sample_from_state_dict(data: Dict[str, object], arm: str) -> DynamicSample:
    timestamp_raw = float(data.get("timestamp", 0.0))
    timestamp = timestamp_raw * 1e-9 if timestamp_raw > 1e12 else timestamp_raw

    for field in ("force_sensor", "ee_omega_ee_frame", "ee_alpha_ee_frame"):
        if field not in data:
            raise ValueError(f"{field} 字段缺失, 请升级 server。")
    wrench_unb = _slice_arm(data["force_sensor"], arm)
    omega_F = _slice_arm(data["ee_omega_ee_frame"], arm)
    alpha_F = _slice_arm(data["ee_alpha_ee_frame"], arm)
    if wrench_unb.size != 6 or omega_F.size != 3 or alpha_F.size != 3:
        raise ValueError(f"wrench/ω/α 切片维度异常: {wrench_unb.size}/{omega_F.size}/{alpha_F.size}")
    return DynamicSample(time.monotonic(), timestamp, wrench_unb, omega_F, alpha_F)


def sample_to_json(sample: DynamicSample) -> Dict[str, object]:
    return {
        "timestamp": sample.timestamp,
        "wrench_unbiased_F": array_to_list(sample.wrench_unbiased_F),
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
            for key in ("wrench_unbiased_F", "omega_F", "alpha_F"):
                if key not in data:
                    raise ValueError(f"jsonl 缺少字段 {key}: 请用新版重新采集。")
            samples.append(DynamicSample(
                arrival_time=time.monotonic(),
                timestamp=float(data.get("timestamp", 0.0)),
                wrench_unbiased_F=np.asarray(data["wrench_unbiased_F"], dtype=float),
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
                    joint_position = np.asarray(data["joint_position"], dtype=float)
                    joint_name = list(data.get("joint_name", []))
                except Exception as exc:
                    self.outer.errors.append(f"解析 joint_position/joint_name 失败: {exc}")
                    return
                with self.outer.lock:
                    self.outer.samples.append(sample)
                    self.outer.latest_joint_position = joint_position.copy()
                    self.outer.latest_joint_name = joint_name

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
            input("数据已连接。请手动执行动态激励, 按 Enter 开始记录 > ")
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
        excitation_count = len(amplitudes)
        wrist_indices = wrist_indices_for_position(base_position, args.arm, excitation_count)
        if not wrist_indices or max(wrist_indices) >= base_position.size:
            raise RuntimeError(f"关节数组长度 {base_position.size} 不支持 {excitation_count} 关节激励。")

        print(f"数据已连接; 激励关节索引: {wrist_indices}")
        print(f"幅值 rad: {amplitudes}")
        print(f"频率 Hz:  {frequencies}")
        print(f"相位 rad: {phases}")
        print(f"时长:     {args.duration:.2f}s, 平滑: {args.excitation_ramp:.2f}s")
        input("确认周围安全后按 Enter 开始 > ")

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

        print("激励结束, 保持中心姿态 1 秒。")
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
    com_in = parse_float_list(args.com_mm, 3, "--com-mm") / 1000.0
    com_fit = np.asarray(result["com_fitted_m"], dtype=float)
    return {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "sample_count": int(sample_count),
        "arm": args.arm,
        "dt": args.dt,
        "wrench_source": "force_sensor (零偏后重力补偿前, F 系)",
        "kinematics_source": "ee_omega/alpha_ee_frame (server 1 kHz Butterworth, F 系)",
        "mass_kg": args.mass,
        "com_input_mm": array_to_list(com_in * 1000.0),
        "com_fitted_mm": array_to_list(com_fit * 1000.0),
        "com_diff_mm": float(np.linalg.norm(com_fit - com_in) * 1000.0),
        "theta_order": _PARAM_NAMES,
        "theta": array_to_list(theta),
        "inertia_com_kg_m2": inertia.tolist(),
        "rank": f"{result['rank']} / 7",
        "condition_number": result["condition_number"],
        "singular_values": array_to_list(np.asarray(result["singular_values"], dtype=float)),
        "torque_rms": result["torque_rms"],
        "physical": result["physical"],
    }


def print_result(payload: Dict[str, object]) -> None:
    print("\n========== 工具质心转动惯量联合辨识结果 ==========")
    print(f"有效样本数:        {payload['sample_count']}")
    print(f"辨识机械臂:        {payload['arm']}")
    print(f"工具质量 kg:       {payload['mass_kg']}")
    print(f"输入 COM mm:       {np.array(payload['com_input_mm'])}")
    print(f"拟合 COM mm:       {np.array(payload['com_fitted_mm'])}")
    diff = float(payload["com_diff_mm"])
    print(f"COM 偏差 mm:       {diff:.4f}" + (" *** 偏差较大, 请检查 ***" if diff > _COM_WARN_DIFF_MM else ""))
    print(f"参数顺序:          {payload['theta_order']}")
    print(f"参数向量:          {np.array(payload['theta'])}")
    print(f"^F I_C kg*m^2:\n{np.array(payload['inertia_com_kg_m2'])}")
    print(f"力矩残差 RMS:      {float(payload['torque_rms']):.6g} Nm")
    print(f"回归矩阵秩:        {payload['rank']}")
    print(f"条件数:            {float(payload['condition_number']):.6g}")
    physical = payload["physical"]
    print(f"正定检查:          {physical['positive_definite']}")
    print(f"三角不等式检查:    {physical['triangle_inequality']}")
    if float(payload["condition_number"]) > 1e4:
        print("警告: 条件数较大, 建议增加激励方向多样性。")
    print("\n===== robot_config.yaml 建议字段 =====")
    inertia = np.array(payload["inertia_com_kg_m2"])
    com_fitted = np.array(payload["com_fitted_mm"])
    print("tool_inertia:")
    print(f"  inertia_com_kg_m2: [[{inertia[0,0]:.8g}, {inertia[0,1]:.8g}, {inertia[0,2]:.8g}],")
    print(f"                     [{inertia[1,0]:.8g}, {inertia[1,1]:.8g}, {inertia[1,2]:.8g}],")
    print(f"                     [{inertia[2,0]:.8g}, {inertia[2,1]:.8g}, {inertia[2,2]:.8g}]]")
    print("tool_gravity_compensation:")
    print(f"  com_mm: [{com_fitted[0]:.5f}, {com_fitted[1]:.5f}, {com_fitted[2]:.5f}]")
    print("============================================\n")


# ---------------------------------------------------------------------------
# self-test
# ---------------------------------------------------------------------------

def run_self_test() -> None:
    rng = np.random.default_rng(42)
    I_true = np.array([[0.018, 0.002, 0.0],
                       [0.002, 0.026, 0.0],
                       [0.0,   0.0,   0.031]], dtype=float)
    com_true = np.array([0.001, -0.002, 0.05])
    g_world = np.array([0.0, 0.0, -9.81])

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

    # 模拟末端姿态变化 (低频摆动, 让 gravity 投影在 EE 系下有一定变化)
    R_ee = np.array([rot_y(0.15 * math.sin(2.0 * math.pi * 0.1 * ti)) for ti in t])
    F_grav_ee = np.array([r.T @ g_world for r in R_ee])  # m=1 → 数值上等于加速度

    a_C = rng.normal(scale=0.5, size=(n, 3))
    F_unb = F_grav_ee - a_C   # 重力 + 动态力 (mass=1)
    T_unb = np.cross(com_true, F_unb) - np.array([
        I_true @ alpha[i] + np.cross(omega[i], I_true @ omega[i]) for i in range(n)
    ])

    result = fit_inertia_and_com(omega, alpha, F_unb, T_unb)
    theta = np.asarray(result["theta"])
    I_err = np.linalg.norm(theta[:4] - np.array([0.018, 0.026, 0.031, 0.002]))
    com_err = np.linalg.norm(result["com_fitted_m"] - com_true)
    print(f"self-test: I_err={I_err:.6e}  com_err={com_err:.6e}  torque_rms={result['torque_rms']:.6e}")
    if I_err > 1e-10 or com_err > 1e-10:
        raise RuntimeError(f"self-test 失败: theta={theta}")


def rot_y(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=float)


# ---------------------------------------------------------------------------
# CLI & main
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="联合辨识工具绕质心的转动惯量 ^F I_C 与 COM 偏移 (7-param LS, F 系)。",
    )
    parser.add_argument("--arm", choices=["primary", "secondary"], default="primary")
    parser.add_argument("--state-topic", default=DEFAULT_STATE_TOPIC)
    parser.add_argument("--input", type=Path, help="已有 jsonl 文件; 不填则在线采集。")
    parser.add_argument("--duration", type=float, default=DEFAULT_EXCITATION_DURATION)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--dt", type=float, default=DEFAULT_DT)
    parser.add_argument("--mass", type=float, default=DEFAULT_TOOL_MASS_KG,
                        help="工具质量 kg (仅用于结果元数据)。")
    parser.add_argument("--com-mm", default=DEFAULT_TOOL_COM_MM,
                        help="COM 初始值 mm (x,y,z, F 系)。联合估计将输出拟合值与之对比。")
    parser.add_argument("--no-excitation", action="store_true",
                        help="不自动下发激励, 只采集。")
    parser.add_argument("--excitation-amplitudes", default=DEFAULT_EXCITATION_AMPLITUDES)
    parser.add_argument("--excitation-frequencies", default=DEFAULT_EXCITATION_FREQUENCIES)
    parser.add_argument("--excitation-phases", default=DEFAULT_EXCITATION_PHASES)
    parser.add_argument("--excitation-ramp", type=float, default=DEFAULT_EXCITATION_RAMP)
    parser.add_argument("--trim-start", type=float, default=1.0)
    parser.add_argument("--trim-end", type=float, default=1.0)
    parser.add_argument("--filter-fc", type=float, default=0.0,
                        help="可选: 仅对 wrench 再做零相位 Butterworth (Hz)。默认 0 = 不滤。")
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
    amplitudes = parse_float_list(args.excitation_amplitudes, -1, "--excitation-amplitudes")
    frequencies = parse_float_list(args.excitation_frequencies, -1, "--excitation-frequencies")
    phases = parse_float_list(args.excitation_phases, -1, "--excitation-phases")
    if not (len(amplitudes) == len(frequencies) == len(phases) and len(amplitudes) in (3, 4)):
        parser.error("excitation 数组长度必须相等且为 3 或 4。")
    if np.any(amplitudes < 0.0):
        parser.error("--excitation-amplitudes 不能包含负数。")
    if np.any(frequencies <= 0.0):
        parser.error("--excitation-frequencies 必须全部为正数。")
    return args


def main() -> None:
    args = parse_args()
    if args.self_test:
        run_self_test()
        return

    print("\n========== 工具惯量 + COM 联合辨识 ==========")
    print(f"机械臂:            {args.arm}")
    print(f"dt:                {args.dt:.6f} s")
    print(f"质量:              {args.mass:.6f} kg")
    print(f"输入 COM mm:       {args.com_mm}")
    if args.filter_fc and args.filter_fc > 0:
        print(f"额外滤波 (wrench): {args.filter_fc:.2f} Hz")
    if args.input:
        print("模式:              离线拟合")
    elif args.no_excitation:
        print("模式:              手动采集")
    else:
        print("模式:              自动激励 (J4-J7)")
        print(f"  幅值: {args.excitation_amplitudes}")
        print(f"  频率: {args.excitation_frequencies}")
        print(f"  相位: {args.excitation_phases}")
        print(f"  时长: {args.duration:.2f}s")
    print("============================================\n")

    if args.input:
        print(f"读取: {args.input.expanduser()}")
        samples = samples_from_jsonl(args.input.expanduser())
        capture_path = args.input.expanduser()
    else:
        samples = capture_online(args)
        capture_path = args.output_dir / f"tool_inertia_capture_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl"
        write_jsonl(capture_path, samples)
        print(f"已保存: {capture_path}")

    extra_fc = args.filter_fc if args.filter_fc and args.filter_fc > 0 else None
    arrays = prepare_fit_arrays(
        samples,
        trim_start_sec=args.trim_start, trim_end_sec=args.trim_end, dt=args.dt,
        extra_filter_fc_hz=extra_fc,
    )
    print(f"拟合中, 有效样本: {len(arrays['omega'])}")
    result = fit_inertia_and_com(arrays["omega"], arrays["alpha"], arrays["F_unb"], arrays["T_unb"])
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
