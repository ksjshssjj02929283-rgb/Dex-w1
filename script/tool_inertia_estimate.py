#!/usr/bin/env python3
"""通过末端动态激励辨识工具绕质心的转动惯量。

数据处理约定:
- q/dq/ddq 的来源与 qt_dyn_params_ident.py 保持一致: 记录关节位置 q,
  离线滤波并通过中心差分得到 dq/ddq。
- 默认使用补偿前 raw_force_sensor, 避免运行时工具重力补偿消掉待辨识信号。
- 输出惯量为 ^F I_C: 绕工具质心 C, 方向表达在末端法兰/EE 坐标系 F。
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
except Exception:  # pragma: no cover - deployment env normally has scipy
    scipy = None


DEFAULT_STATE_TOPIC = "/feedback/robot_server_state"
DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parent / "tool_inertia_data"
DEFAULT_ROBOT_CONFIG = Path(__file__).resolve().parent.parent / "w1_server" / "robot_config.yaml"
DEFAULT_DT = 0.01
DEFAULT_WF = 0.12 * np.pi
DEFAULT_L = 3
DEFAULT_TOOL_MASS_KG = 1.061
DEFAULT_TOOL_COM_MM = "56.72691,17.72389,-0.47806"
DEFAULT_TOOL_W_BIAS_RAW_SRI = "13.22340,-21.48320,-24.07190,0.16010,-0.46700,-0.66110"
DEFAULT_EXCITATION_DURATION = 30.0
DEFAULT_EXCITATION_AMPLITUDES = "0.20,0.18,0.25"
DEFAULT_EXCITATION_FREQUENCIES = "0.35,0.50,0.70"
DEFAULT_EXCITATION_PHASES = "0.0,1.57079632679,1.0471975512"
DEFAULT_EXCITATION_RAMP = 2.0


@dataclass
class DynamicSample:
    """单个动态采样点: 7 轴关节位置和映射到 EE/F 坐标系的六维力。"""

    arrival_time: float
    timestamp: float
    q_raw: np.ndarray
    wrench_ee: np.ndarray


def skew(v: np.ndarray) -> np.ndarray:
    return np.array(
        [[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]],
        dtype=float,
    )


def parse_float_list(text: str, expected: int, name: str) -> np.ndarray:
    values = [float(item.strip()) for item in text.replace(";", ",").split(",") if item.strip()]
    if len(values) != expected:
        raise ValueError(f"{name} 需要 {expected} 个逗号分隔的数值，实际得到 {len(values)} 个。")
    return np.asarray(values, dtype=float)


def array_to_list(value: np.ndarray) -> List[float]:
    return [float(x) for x in np.asarray(value).reshape(-1)]


def arm_wrist_indices(arm: str) -> List[int]:
    """返回全身关节数组中所选手臂 J5/J6/J7 的索引。"""
    if arm == "primary":
        return [10, 11, 12]
    return [17, 18, 19]


def wrist_indices_for_position(position: np.ndarray, arm: str) -> List[int]:
    """兼容全身关节数组和单臂 7 轴数组。"""
    if np.asarray(position).size == 7:
        return [4, 5, 6]
    return arm_wrist_indices(arm)


def smooth_excitation_envelope(t: float, duration: float, ramp_sec: float) -> float:
    """激励包络，启动和结束都平滑回到中心姿态。"""
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


def generate_wrist_excitation_target(
    base_position: np.ndarray,
    arm: str,
    t: float,
    duration: float,
    amplitudes: np.ndarray,
    frequencies: np.ndarray,
    phases: np.ndarray,
    ramp_sec: float,
) -> np.ndarray:
    """在当前姿态附近生成 J5/J6/J7 多频正弦激励目标。

    base_position 是全身关节位置。函数只修改所选手臂的 J5/J6/J7。
    """
    target = np.asarray(base_position, dtype=float).copy()
    envelope = smooth_excitation_envelope(t, duration, ramp_sec)
    offsets = envelope * amplitudes * np.sin(2.0 * np.pi * frequencies * t + phases)
    for idx, offset in zip(wrist_indices_for_position(target, arm), offsets):
        target[idx] += offset
    return target


def map_sri_raw_wrench_to_ee(raw: np.ndarray) -> np.ndarray:
    """把 SRI 原始顺序/符号映射为 EE/F 坐标系 wrench。

    约定与 tool_com_estimate.py 保持一致:
        raw_sri = [-Fx, -Fy, Fz, -Mx, -My, Mz]
        ee      = [ Fx,  Fy, Fz,  Mx,  My, Mz]
    """
    raw = np.asarray(raw, dtype=float)
    if raw.shape != (6,):
        raise ValueError(f"六维力数据应为 6 维，实际 shape={raw.shape}。")
    return np.array([-raw[0], -raw[1], raw[2], -raw[3], -raw[4], raw[5]], dtype=float)


def map_raw_sri_bias_to_ee(w_bias_raw_sri: np.ndarray) -> np.ndarray:
    return map_sri_raw_wrench_to_ee(w_bias_raw_sri)


def get_actual_q(q_raw: np.ndarray, arm: str, robot_type: str) -> np.ndarray:
    """把反馈关节角转换为 DH 模型使用的关节角。

    这里的 offset 与 qt_dyn_params_ident.py 的 get_actual_q() 保持一致。
    """
    is_left = arm == "primary"
    is_anth = robot_type in ("w1_anth", "W1Anthropomorphic", "anth")
    if is_left:
        offset = (
            np.array([0.0, 0.0, -np.pi / 2.0, 0.0, 0.0, -np.pi / 2.0, 0.0])
            if is_anth
            else np.array([0.0, 0.0, -np.pi / 2.0, 0.0, 0.0, 0.0, 0.0])
        )
    else:
        offset = (
            np.array([0.0, 0.0, np.pi / 2.0, 0.0, 0.0, np.pi / 2.0, 0.0])
            if is_anth
            else np.array([0.0, 0.0, np.pi / 2.0, 0.0, 0.0, 0.0, 0.0])
        )
    return np.asarray(q_raw, dtype=float) + offset


def central_diff(array: np.ndarray, dt: float, order: int = 2) -> np.ndarray:
    """中心差分。order=2 时使用与动力学参数辨识脚本一致的四阶中心格式。"""
    array = np.asarray(array, dtype=float)
    diff = np.zeros_like(array)
    n = len(array)
    if n < 5 or order == 1:
        diff[0] = (array[1] - array[0]) / dt
        for i in range(1, n - 1):
            diff[i] = (array[i + 1] - array[i - 1]) / (2.0 * dt)
        diff[-1] = (array[-1] - array[-2]) / dt
        return diff
    diff[0] = (array[1] - array[0]) / dt
    diff[1] = (array[2] - array[0]) / (2.0 * dt)
    for i in range(2, n - 2):
        diff[i] = (-array[i + 2] + 8.0 * array[i + 1] - 8.0 * array[i - 1] + array[i - 2]) / (12.0 * dt)
    diff[-2] = (array[-1] - array[-3]) / (2.0 * dt)
    diff[-1] = (array[-1] - array[-2]) / dt
    return diff


def butter_filtfilt(order: int, wn: float, signal: np.ndarray) -> np.ndarray:
    """零相位 Butterworth 滤波；scipy 不可用或截止频率非法时直接返回原信号。"""
    if scipy is None:
        return signal.copy()
    try:
        wn_val = float(wn)
        if not np.isfinite(wn_val) or not (0.0 < wn_val < 1.0):
            return signal.copy()
    except Exception:
        return signal.copy()
    b, a = scipy.signal.butter(order, wn_val)
    return scipy.signal.filtfilt(b, a, signal, method="pad")


def diff_and_filter_q(q_raw: np.ndarray, dt: float, wf: float = DEFAULT_WF, harmonics: int = DEFAULT_L) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """由位置序列生成 q/dq/ddq。

    DEFAULT_WF 和 DEFAULT_L 来自 qt_dyn_params_ident.py, 用于估算滤波截止频率。
    如果手腕激励比原动力学辨识轨迹更快, 后续应把滤波截止频率做成可调参数。
    """
    fc = 10.0 * (wf / (2.0 * np.pi) * harmonics)
    wc = fc * 2.0 * math.pi * dt
    q = np.zeros_like(q_raw)
    dq = np.zeros_like(q_raw)
    ddq = np.zeros_like(q_raw)
    for i in range(q_raw.shape[1]):
        q_i = q_raw[:, i]
        q[:, i] = butter_filtfilt(3, wc, q_i)
        dq_raw_i = central_diff(q_i, dt, 2)
        dq[:, i] = butter_filtfilt(3, wc, dq_raw_i)
        ddq_raw_i = central_diff(dq_raw_i, dt, 2)
        ddq[:, i] = butter_filtfilt(3, wc, ddq_raw_i)
    return q, dq, ddq


def dh_transform(alpha: float, d: float, theta: float) -> np.ndarray:
    """标准 DH 单节变换，a 固定为 0，与 qt_dyn_params_ident.py 的 7 轴模型一致。"""
    ca, sa = math.cos(alpha), math.sin(alpha)
    ct, st = math.cos(theta), math.sin(theta)
    return np.array(
        [
            [ct, -st * ca, st * sa, 0.0],
            [st, ct * ca, -ct * sa, 0.0],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def ee_rotation_and_joint_axes(q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """计算末端法兰姿态和每个关节轴在世界系下的方向。

    返回:
        rotation: ^W R_F, 法兰坐标系 F 到世界系 W 的旋转
        axes:     7x3, 每个关节旋转轴在世界系下的单位向量
    """
    alphas = [-math.pi / 2.0, math.pi / 2.0, -math.pi / 2.0, math.pi / 2.0, -math.pi / 2.0, math.pi / 2.0, 0.0]
    ds = [0.1025, 0.0, 0.26, 0.0, 0.264, 0.0, 0.0]
    transform = np.eye(4)
    axes = []
    for i in range(7):
        axes.append(transform[:3, 2].copy())
        transform = transform @ dh_transform(alphas[i], ds[i], float(q[i]))
    return transform[:3, :3].copy(), np.vstack(axes)


def compute_body_omega_alpha(q: np.ndarray, dq: np.ndarray, dt: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """由 q/dq 计算末端角速度和角加速度，并转换到法兰系 F。

    这里不重点使用末端线速度/线加速度，因为脚本直接使用六维力传感器的
    force 通道扣除 com x force, 只拟合旋转惯量项。
    """
    rotations = []
    omega_world = []
    for i in range(q.shape[0]):
        rotation, axes_world = ee_rotation_and_joint_axes(q[i])
        rotations.append(rotation)
        omega_world.append(axes_world.T @ dq[i])
    rotations_arr = np.asarray(rotations)
    omega_world_arr = np.asarray(omega_world)
    alpha_world_arr = np.vstack([central_diff(omega_world_arr[:, j], dt, 2) for j in range(3)]).T
    omega_body = np.einsum("nij,nj->ni", np.transpose(rotations_arr, (0, 2, 1)), omega_world_arr)
    alpha_body = np.einsum("nij,nj->ni", np.transpose(rotations_arr, (0, 2, 1)), alpha_world_arr)
    return rotations_arr, omega_body, alpha_body


def theta_to_inertia_matrix(theta: np.ndarray) -> np.ndarray:
    """把 6 个独立惯量参数还原为对称 3x3 惯量矩阵。"""
    theta = np.asarray(theta, dtype=float).reshape(6)
    return np.array(
        [
            [theta[0], theta[3], theta[4]],
            [theta[3], theta[1], theta[5]],
            [theta[4], theta[5], theta[2]],
        ],
        dtype=float,
    )


def inertia_matrix_to_theta(inertia: np.ndarray) -> np.ndarray:
    inertia = np.asarray(inertia, dtype=float).reshape(3, 3)
    return np.array([inertia[0, 0], inertia[1, 1], inertia[2, 2], inertia[0, 1], inertia[0, 2], inertia[1, 2]], dtype=float)


def inertia_regressor_row(omega: np.ndarray, alpha: np.ndarray) -> np.ndarray:
    """生成单个采样点的线性回归矩阵 Y_I。

    模型:
        tau_rot = I * alpha + omega x (I * omega)
                = Y_I(omega, alpha) * [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
    """
    cols = []
    for basis_theta in np.eye(6):
        basis_i = theta_to_inertia_matrix(basis_theta)
        cols.append(basis_i @ alpha + np.cross(omega, basis_i @ omega))
    return np.column_stack(cols)


def fit_inertia_from_rotational_terms(omega: np.ndarray, alpha: np.ndarray, torque_residual: np.ndarray) -> Dict[str, object]:
    """最小二乘拟合 ^F I_C。"""
    omega = np.asarray(omega, dtype=float)
    alpha = np.asarray(alpha, dtype=float)
    torque_residual = np.asarray(torque_residual, dtype=float)
    if omega.shape != alpha.shape or omega.shape != torque_residual.shape or omega.shape[1] != 3:
        raise ValueError("omega、alpha、torque_residual 都必须是 shape=(N, 3)。")
    blocks = [inertia_regressor_row(omega[i], alpha[i]) for i in range(omega.shape[0])]
    y = np.vstack(blocks)
    b = torque_residual.reshape(-1)
    theta, residuals, rank, singular_values = np.linalg.lstsq(y, b, rcond=None)
    inertia = theta_to_inertia_matrix(theta)
    pred = (y @ theta).reshape((-1, 3))
    err = torque_residual - pred
    eigvals = np.linalg.eigvalsh(inertia)
    physical = {
        "symmetric": bool(np.allclose(inertia, inertia.T)),
        "positive_definite": bool(np.all(eigvals > 0.0)),
        "triangle_inequality": bool(
            inertia[0, 0] + inertia[1, 1] >= inertia[2, 2]
            and inertia[0, 0] + inertia[2, 2] >= inertia[1, 1]
            and inertia[1, 1] + inertia[2, 2] >= inertia[0, 0]
        ),
        "eigenvalues": array_to_list(eigvals),
    }
    return {
        "theta": theta,
        "inertia_com": inertia,
        "rank": int(rank),
        "condition_number": float(np.linalg.cond(y)) if y.size else float("inf"),
        "singular_values": singular_values,
        "torque_rms": float(np.sqrt(np.mean(err ** 2))),
        "torque_residual_pred": pred,
        "torque_error": err,
        "physical": physical,
    }


def prepare_fit_arrays(
    samples: List[DynamicSample],
    arm: str,
    robot_type: str,
    dt: float,
    com_m: np.ndarray,
    w_bias_ee: np.ndarray,
    trim_start_sec: float,
    trim_end_sec: float,
) -> Dict[str, np.ndarray]:
    """把原始采样数据整理为惯量拟合需要的数组。

    处理步骤:
    1. 从 q_raw 生成 q/dq/ddq。
    2. 由 q/dq 计算法兰系角速度 omega 和角加速度 alpha。
    3. 对六维力扣除零偏。
    4. 从法兰力矩中扣除 com x force, 剩余部分主要用于拟合旋转惯量。
    """
    if len(samples) < 20:
        raise ValueError("动态采样点至少需要 20 个。")
    q_raw = np.vstack([sample.q_raw for sample in samples])
    wrench = np.vstack([sample.wrench_ee for sample in samples])
    q_actual = get_actual_q(q_raw, arm, robot_type)
    q, dq, ddq = diff_and_filter_q(q_actual, dt)
    _, omega, alpha = compute_body_omega_alpha(q, dq, dt)

    force_unbiased = wrench[:, :3] - w_bias_ee[:3]
    torque_unbiased = wrench[:, 3:6] - w_bias_ee[3:6]
    torque_rot = torque_unbiased - np.cross(np.asarray(com_m, dtype=float), force_unbiased)

    si = max(0, int(round(trim_start_sec / dt)))
    ei = len(samples) - max(0, int(round(trim_end_sec / dt)))
    if ei <= si + 10:
        raise ValueError("首尾裁剪后剩余样本太少，请缩短 --trim-start/--trim-end 或增加采集时长。")
    return {
        "q": q[si:ei],
        "dq": dq[si:ei],
        "ddq": ddq[si:ei],
        "wrench_ee": wrench[si:ei],
        "omega": omega[si:ei],
        "alpha": alpha[si:ei],
        "torque_rot": torque_rot[si:ei],
    }


def samples_from_jsonl(path: Path, arm: str, wrench_format: str) -> List[DynamicSample]:
    """读取离线 JSONL 采样文件，便于反复调整滤波和拟合参数。"""
    samples = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            if not line.strip():
                continue
            data = json.loads(line)
            samples.append(sample_from_state_dict(data, arm, wrench_format))
    return samples


def sample_to_json(sample: DynamicSample) -> Dict[str, object]:
    return {
        "timestamp": sample.timestamp,
        "q_raw": array_to_list(sample.q_raw),
        "wrench_ee": array_to_list(sample.wrench_ee),
    }


def sample_from_state_dict(data: Dict[str, object], arm: str, wrench_format: str) -> DynamicSample:
    """从 robot_server_state JSON 字段中提取单臂 q 和六维力。

    q 的切片规则与 qt_dyn_params_ident.py 一致:
        primary   -> joint_position[6:13]
        secondary -> joint_position[13:20]
    """
    timestamp_raw = float(data.get("timestamp", 0.0))
    timestamp = timestamp_raw * 1e-9 if timestamp_raw > 1e12 else timestamp_raw
    q_all = np.asarray(data.get("joint_position", data.get("q_raw")), dtype=float)
    if q_all.size == 7:
        q_raw = q_all
    elif arm == "primary":
        q_raw = q_all[6:13]
    else:
        q_raw = q_all[13:20]
    if q_raw.size != 7:
        raise ValueError(f"无法从 joint_position(size={q_all.size}) 中提取 7 个单臂关节。")

    if "wrench_ee" in data:
        wrench_ee = np.asarray(data["wrench_ee"], dtype=float)
    else:
        key = "raw_force_sensor" if wrench_format == "raw_sri" and "raw_force_sensor" in data else "force_sensor"
        force_sensor = np.asarray(data[key], dtype=float)
        raw = force_sensor[:6] if arm == "primary" else force_sensor[6:12]
        wrench_ee = map_sri_raw_wrench_to_ee(raw) if wrench_format == "raw_sri" else raw.copy()
    if wrench_ee.size != 6:
        raise ValueError(f"无法提取 6 维力数据，实际 size={wrench_ee.size}。")
    return DynamicSample(time.monotonic(), timestamp, q_raw.copy(), wrench_ee.copy())


class ToolInertiaNode:
    """在线采集节点: 订阅 robot_server_state，并可发布关节位置激励。"""

    def __init__(self, state_topic: str, arm: str, wrench_format: str):
        import rclpy
        from joint_interfaces.msg import JointPositionControl
        from rclpy.node import Node
        from std_msgs.msg import Header
        from std_msgs.msg import String

        class _Node(Node):
            def __init__(self, outer):
                super().__init__("tool_inertia_estimate")
                self.outer = outer
                self.create_subscription(String, state_topic, self._state_callback, 10)
                self.publisher = self.create_publisher(JointPositionControl, "/control/joint_position", 10)

            def _state_callback(self, msg):
                try:
                    data = json.loads(msg.data)
                    sample = sample_from_state_dict(data, arm, wrench_format)
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
            return [sample for sample in self.samples if sample.arrival_time >= start_time]

    def get_joint_position(self) -> Optional[np.ndarray]:
        with self.lock:
            if self.latest_joint_position is None:
                return None
            return self.latest_joint_position.copy()

    def get_joint_name(self) -> List[str]:
        with self.lock:
            return list(self.latest_joint_name)

    def publish_joint_position(self, target_position: np.ndarray) -> None:
        self.node.publish_joint_position(target_position)


def capture_online(args: argparse.Namespace) -> List[DynamicSample]:
    """在线采集动态激励数据。"""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init()
    wrapper = ToolInertiaNode(args.state_topic, args.arm, args.wrench_format)
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
            input("数据已连接。请手动执行动态激励，按 Enter 开始记录 > ")
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
            raise RuntimeError("没有获取到当前关节位置，无法启动自动激励。")
        if not wrapper.get_joint_name():
            raise RuntimeError("没有获取到 joint_name，无法发布 JointPositionControl 自动激励。")

        amplitudes = parse_float_list(args.excitation_amplitudes, 3, "--excitation-amplitudes")
        frequencies = parse_float_list(args.excitation_frequencies, 3, "--excitation-frequencies")
        phases = parse_float_list(args.excitation_phases, 3, "--excitation-phases")
        wrist_indices = wrist_indices_for_position(base_position, args.arm)
        if max(wrist_indices) >= base_position.size:
            raise RuntimeError(f"当前关节数组长度 {base_position.size} 不支持 {args.arm} 的 J5/J6/J7 激励。")

        print("数据已连接，将启动推荐腕部激励。")
        print(f"中心姿态取当前反馈位置；激励关节索引: {wrist_indices}")
        print(f"幅值 rad: {amplitudes}")
        print(f"频率 Hz:  {frequencies}")
        print(f"相位 rad: {phases}")
        print(f"时长:     {args.duration:.2f}s，首尾平滑时间: {args.excitation_ramp:.2f}s")
        input("确认周围安全、无碰撞风险后按 Enter 开始自动激励和采集 > ")

        start = time.monotonic()
        next_t = time.monotonic()
        print("自动激励中，请保持人员和物体远离机械臂工作空间。")
        while True:
            t = time.monotonic() - start
            if t > args.duration:
                break
            target = generate_wrist_excitation_target(
                base_position,
                args.arm,
                t,
                args.duration,
                amplitudes,
                frequencies,
                phases,
                args.excitation_ramp,
            )
            wrapper.publish_joint_position(target)
            next_t += args.dt
            time.sleep(max(0.0, next_t - time.monotonic()))

        print("激励结束，发送中心姿态保持 1 秒。")
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


def write_jsonl(path: Path, samples: Iterable[DynamicSample]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        for sample in samples:
            f.write(json.dumps(sample_to_json(sample), ensure_ascii=False) + "\n")


def result_to_json(result: Dict[str, object], args: argparse.Namespace, sample_count: int) -> Dict[str, object]:
    """组织结果 JSON。字段名保留英文，便于后续程序读取。"""
    inertia = np.asarray(result["inertia_com"], dtype=float)
    theta = np.asarray(result["theta"], dtype=float)
    return {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "sample_count": int(sample_count),
        "arm": args.arm,
        "robot_type": args.robot_type,
        "dt": args.dt,
        "wrench_format": args.wrench_format,
        "wrench_source": "补偿前 raw_force_sensor" if args.wrench_format == "raw_sri" else "force_sensor/state_mapped",
        "mass_kg": args.mass,
        "com_m": array_to_list(parse_float_list(args.com_mm, 3, "--com-mm") / 1000.0),
        "com_mm": array_to_list(parse_float_list(args.com_mm, 3, "--com-mm")),
        "w_bias_raw_sri": array_to_list(parse_float_list(args.w_bias, 6, "--w-bias")),
        "theta_order": ["Ixx", "Iyy", "Izz", "Ixy", "Ixz", "Iyz"],
        "theta": array_to_list(theta),
        "inertia_com_kg_m2": inertia.tolist(),
        "rank": result["rank"],
        "condition_number": result["condition_number"],
        "singular_values": array_to_list(np.asarray(result["singular_values"], dtype=float)),
        "torque_rms": result["torque_rms"],
        "physical": result["physical"],
    }


def print_result(payload: Dict[str, object]) -> None:
    print("\n========== 工具质心转动惯量辨识结果 ==========")
    print(f"有效样本数:        {payload['sample_count']}")
    print(f"辨识机械臂:        {payload['arm']}")
    print(f"六维力来源:        {payload['wrench_source']}")
    print(f"工具质量 kg:       {payload['mass_kg']}")
    print(f"质心偏移 mm:       {np.array(payload['com_mm'])}")
    print(f"参数顺序:          {payload['theta_order']}")
    print(f"参数向量:          {np.array(payload['theta'])}")
    print(f"^F I_C kg*m^2:\n{np.array(payload['inertia_com_kg_m2'])}")
    print(f"力矩残差 RMS:      {float(payload['torque_rms']):.6g} Nm")
    print(f"回归矩阵秩:        {payload['rank']} / 6")
    print(f"条件数:            {float(payload['condition_number']):.6g}")
    physical = payload["physical"]
    print(f"正定检查:          {physical['positive_definite']}")
    print(f"三角不等式检查:    {physical['triangle_inequality']}")
    if payload["rank"] < 6:
        print("警告: 回归矩阵秩小于 6，末端旋转激励不足，部分惯量参数不可观。")
    if float(payload["condition_number"]) > 1e4:
        print("警告: 条件数较大，建议增加绕多个方向的旋转激励。")
    print("\n===== robot_config.yaml 建议字段 =====")
    inertia = np.array(payload["inertia_com_kg_m2"])
    print("tool_inertia:")
    print(f"  inertia_com_kg_m2: [[{inertia[0,0]:.8g}, {inertia[0,1]:.8g}, {inertia[0,2]:.8g}],")
    print(f"                     [{inertia[1,0]:.8g}, {inertia[1,1]:.8g}, {inertia[1,2]:.8g}],")
    print(f"                     [{inertia[2,0]:.8g}, {inertia[2,1]:.8g}, {inertia[2,2]:.8g}]]")
    print("============================================\n")


def run_self_test() -> None:
    """合成数据自测试: 已知惯量 -> 生成力矩 -> 反解惯量。"""
    rng = np.random.default_rng(42)
    theta_true = np.array([0.018, 0.026, 0.031, 0.002, -0.003, 0.004])
    inertia_true = theta_to_inertia_matrix(theta_true)
    omega = rng.normal(size=(240, 3))
    alpha = rng.normal(size=(240, 3))
    torque = np.vstack([inertia_true @ alpha[i] + np.cross(omega[i], inertia_true @ omega[i]) for i in range(omega.shape[0])])
    result = fit_inertia_from_rotational_terms(omega, alpha, torque)
    err = np.linalg.norm(np.asarray(result["theta"]) - theta_true)
    print(f"自测试: theta_err={err:.6e}, torque_rms={result['torque_rms']:.6e}")
    if err > 1e-10:
        raise RuntimeError("合成数据惯量自测试失败。")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="通过动态六维力数据辨识工具绕质心的转动惯量 ^F I_C。")
    parser.add_argument("--arm", choices=["primary", "secondary"], default="primary", help="选择左臂 primary 或右臂 secondary。")
    parser.add_argument("--state-topic", default=DEFAULT_STATE_TOPIC, help="robot_server_state 话题名。")
    parser.add_argument("--input", type=Path, help="已有 JSONL 采集文件；不填则在线采集。")
    parser.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_EXCITATION_DURATION,
        help=f"在线采集/自动激励时长，单位秒；默认 {DEFAULT_EXCITATION_DURATION}。",
    )
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR, help="采集数据和辨识结果输出目录。")
    parser.add_argument("--dt", type=float, default=DEFAULT_DT, help="数据处理采样周期，单位秒，默认 0.01。")
    parser.add_argument("--robot-type", choices=["w1_anth", "w1_indst"], default="w1_anth", help="机器人 DH offset 类型。")
    parser.add_argument(
        "--mass",
        type=float,
        default=DEFAULT_TOOL_MASS_KG,
        help=f"工具质量，单位 kg，默认硬编码: {DEFAULT_TOOL_MASS_KG}。",
    )
    parser.add_argument(
        "--com-mm",
        default=DEFAULT_TOOL_COM_MM,
        help=f"工具质心相对法兰/EE 的偏移，单位 mm，格式 x,y,z，默认硬编码: {DEFAULT_TOOL_COM_MM}。",
    )
    parser.add_argument(
        "--w-bias",
        default=DEFAULT_TOOL_W_BIAS_RAW_SRI,
        help=(
            "六维力零偏，raw_sri 顺序，格式同 tool_com_estimate.py 的 w_bias，"
            f"默认硬编码: {DEFAULT_TOOL_W_BIAS_RAW_SRI}。"
        ),
    )
    parser.add_argument(
        "--wrench-format",
        choices=["raw_sri", "state_mapped"],
        default="raw_sri",
        help="六维力格式。默认 raw_sri，使用补偿前 raw_force_sensor，避免使用已补偿数据。",
    )
    parser.add_argument(
        "--no-excitation",
        action="store_true",
        help="不自动下发推荐激励，只采集数据；此时需要人工或其他脚本执行运动。",
    )
    parser.add_argument(
        "--excitation-amplitudes",
        default=DEFAULT_EXCITATION_AMPLITUDES,
        help=f"J5/J6/J7 正弦激励幅值，单位 rad，默认: {DEFAULT_EXCITATION_AMPLITUDES}。",
    )
    parser.add_argument(
        "--excitation-frequencies",
        default=DEFAULT_EXCITATION_FREQUENCIES,
        help=f"J5/J6/J7 正弦激励频率，单位 Hz，默认: {DEFAULT_EXCITATION_FREQUENCIES}。",
    )
    parser.add_argument(
        "--excitation-phases",
        default=DEFAULT_EXCITATION_PHASES,
        help=f"J5/J6/J7 正弦激励相位，单位 rad，默认: {DEFAULT_EXCITATION_PHASES}。",
    )
    parser.add_argument(
        "--excitation-ramp",
        type=float,
        default=DEFAULT_EXCITATION_RAMP,
        help=f"自动激励首尾平滑时间，单位秒，默认: {DEFAULT_EXCITATION_RAMP}。",
    )
    parser.add_argument("--trim-start", type=float, default=1.0, help="拟合前裁掉开头秒数，避开启动瞬态。")
    parser.add_argument("--trim-end", type=float, default=1.0, help="拟合前裁掉结尾秒数，避开停止瞬态。")
    parser.add_argument("--self-test", action="store_true", help="运行合成数据自测试后退出。")
    args = parser.parse_args()
    if args.self_test:
        return args
    if args.dt <= 0.0:
        parser.error("--dt 必须为正数。")
    if args.mass <= 0.0:
        parser.error("--mass 必须为正数。")
    if args.duration <= 0.0 and not args.no_excitation:
        parser.error("自动激励模式下 --duration 必须为正数；如需手动停止，请使用 --no-excitation。")
    if args.excitation_ramp < 0.0:
        parser.error("--excitation-ramp 不能为负数。")
    parse_float_list(args.com_mm, 3, "--com-mm")
    parse_float_list(args.w_bias, 6, "--w-bias")
    amplitudes = parse_float_list(args.excitation_amplitudes, 3, "--excitation-amplitudes")
    frequencies = parse_float_list(args.excitation_frequencies, 3, "--excitation-frequencies")
    parse_float_list(args.excitation_phases, 3, "--excitation-phases")
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

    print("\n========== 工具惯量辨识脚本 ==========")
    print(f"机械臂:            {args.arm}")
    print(f"机器人类型:        {args.robot_type}")
    print(f"采样周期 dt:       {args.dt:.6f} s")
    print(f"六维力格式:        {args.wrench_format}")
    print(f"工具质量:          {args.mass:.6f} kg")
    print(f"质心偏移 com_mm:   {args.com_mm}")
    print(f"六维力零偏 w_bias: {args.w_bias}")
    if args.input:
        print("运动模式:          离线拟合，不下发运动")
    elif args.no_excitation:
        print("运动模式:          只采集，不自动下发激励")
    else:
        print("运动模式:          自动下发推荐 J5/J6/J7 多频正弦激励")
        print(f"激励幅值 rad:      {args.excitation_amplitudes}")
        print(f"激励频率 Hz:       {args.excitation_frequencies}")
        print(f"激励相位 rad:      {args.excitation_phases}")
        print(f"激励时长:          {args.duration:.2f} s")
    print("说明: 默认使用补偿前 raw_force_sensor；请在动态激励过程中避免碰撞和传感器饱和。")
    print("======================================\n")

    if args.input:
        print(f"读取离线采集文件: {args.input.expanduser()}")
        samples = samples_from_jsonl(args.input.expanduser(), args.arm, args.wrench_format)
        capture_path = args.input.expanduser()
    else:
        samples = capture_online(args)
        capture_path = args.output_dir / f"tool_inertia_capture_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl"
        write_jsonl(capture_path, samples)
        print(f"采集数据已保存: {capture_path}")

    com_m = parse_float_list(args.com_mm, 3, "--com-mm") / 1000.0
    w_bias_raw = parse_float_list(args.w_bias, 6, "--w-bias")
    w_bias_ee = map_raw_sri_bias_to_ee(w_bias_raw) if args.wrench_format == "raw_sri" else w_bias_raw
    print("开始处理 q/dq/ddq、末端角速度/角加速度和六维力残差...")
    arrays = prepare_fit_arrays(samples, args.arm, args.robot_type, args.dt, com_m, w_bias_ee, args.trim_start, args.trim_end)
    print(f"开始拟合 ^F I_C，有效样本数: {len(arrays['omega'])}")
    result = fit_inertia_from_rotational_terms(arrays["omega"], arrays["alpha"], arrays["torque_rot"])
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
