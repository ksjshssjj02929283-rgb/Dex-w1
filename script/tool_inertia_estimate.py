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

输出: ^F I_C, kg·m^2.

不再依赖 raw_force_sensor / force_sensor_bias / ee_pose_world / 手写 DH /
joint_position+joint_velocity. 这些字段如果出现在状态里也会被忽略.
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
# 激励关节数: 取 4 时同时激励 J4/J5/J6/J7 (绕 EE x/y/z 更均衡, 改善 Ixx 可观性).
# 幅值从 J4 往 J7 递减, 避免末端摆动幅度过大.
EXCITATION_JOINT_COUNT = 4
DEFAULT_EXCITATION_AMPLITUDES = "0.15,0.30,0.20,0.15"
DEFAULT_EXCITATION_FREQUENCIES = "0.30,0.40,0.65,0.90"
DEFAULT_EXCITATION_PHASES = "0.0,1.57079632679,1.0471975512,0.52359877559"
DEFAULT_EXCITATION_RAMP = 2.0


@dataclass
class DynamicSample:
    arrival_time: float
    timestamp: float
    wrench_gravity_F: np.ndarray  # 6 维, F 系, 已扣零偏与重力
    omega_F: np.ndarray  # 3 维, F 系
    alpha_F: np.ndarray  # 3 维, F 系
    # 仅诊断/兼容采集文件; 不进入回归.
    q_raw: Optional[np.ndarray] = None
    dq_raw: Optional[np.ndarray] = None


def parse_float_list(text: str, expected: int, name: str) -> np.ndarray:
    values = [float(item.strip()) for item in text.replace(";", ",").split(",") if item.strip()]
    if expected > 0 and len(values) != expected:
        raise ValueError(f"{name} 需要 {expected} 个逗号分隔的数值，实际得到 {len(values)} 个。")
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
    """全身关节数组中所选手臂倒数 count 个腕部关节索引。

    count=3 → J5/J6/J7
    count=4 → J4/J5/J6/J7
    """
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


def theta_to_inertia_matrix(theta: np.ndarray) -> np.ndarray:
    theta = np.asarray(theta, dtype=float).reshape(6)
    return np.array(
        [[theta[0], theta[3], theta[4]],
         [theta[3], theta[1], theta[5]],
         [theta[4], theta[5], theta[2]]],
        dtype=float,
    )


def inertia_regressor_row(omega: np.ndarray, alpha: np.ndarray,
                         param_mask: Optional[np.ndarray] = None) -> np.ndarray:
    """tau_rot = I α + ω × (I ω) = Y_I(ω,α) · θ.

    θ default: [Ixx, Iyy, Izz, Ixy, Ixz, Iyz] (6 params).
    param_mask: bool array of length 6, True = include column.
    """
    cols = []
    for k, basis_theta in enumerate(np.eye(6)):
        if param_mask is not None and not param_mask[k]:
            continue
        basis_i = theta_to_inertia_matrix(basis_theta)
        cols.append(basis_i @ alpha + np.cross(omega, basis_i @ omega))
    return np.column_stack(cols)


# 完整 6-param 顺序
_PARAM_NAMES = ["Ixx", "Iyy", "Izz", "Ixy", "Ixz", "Iyz"]
# 约束 Ixz=Iyz=0 → 仅拟合 Ixx, Iyy, Izz, Ixy
_PARAM_MASK_XZ_YZ_ZERO = np.array([True, True, True, True, False, False], dtype=bool)
_TRIANGLE_IZZ_LE_IXX_IYY = np.array([1.0, 1.0, -1.0, 0.0], dtype=float)
_TRIANGLE_CHECK_TOL = 1e-10


def solve_lstsq_with_lower_halfspace(
    y: np.ndarray,
    b: np.ndarray,
    lower_bound_normal: np.ndarray,
    tol: float = 1e-12,
) -> Tuple[np.ndarray, bool, int, np.ndarray]:
    """Solve min ||Yθ-b|| with one constraint: normal·θ >= 0.

    The unconstrained LS solution is used unless it violates the halfspace. When
    it violates, solve the active equality normal·θ = 0 via a small KKT system.
    """
    theta, _, rank, singular_values = np.linalg.lstsq(y, b, rcond=None)
    normal = np.asarray(lower_bound_normal, dtype=float).reshape(-1)
    if normal.size != theta.size:
        raise ValueError("lower_bound_normal size must match the fitted parameter count.")
    if float(normal @ theta) >= -tol:
        return theta, False, int(rank), singular_values

    h = y.T @ y
    rhs = y.T @ b
    kkt = np.block([
        [h, normal[:, None]],
        [normal[None, :], np.zeros((1, 1))],
    ])
    kkt_rhs = np.concatenate([rhs, np.zeros(1)])
    active_solution, _, _, _ = np.linalg.lstsq(kkt, kkt_rhs, rcond=None)
    return active_solution[:-1], True, int(rank), singular_values


def inertia_triangle_inequality_ok(inertia: np.ndarray, tol: float = _TRIANGLE_CHECK_TOL) -> bool:
    ixx, iyy, izz = float(inertia[0, 0]), float(inertia[1, 1]), float(inertia[2, 2])
    return bool(
        ixx + iyy + tol >= izz
        and ixx + izz + tol >= iyy
        and iyy + izz + tol >= ixx
    )


def fit_inertia_from_rotational_terms(
    omega: np.ndarray, alpha: np.ndarray, torque_residual: np.ndarray,
    constrain_xz_yz_zero: bool = False,
) -> Dict[str, object]:
    omega = np.asarray(omega, dtype=float)
    alpha = np.asarray(alpha, dtype=float)
    torque_residual = np.asarray(torque_residual, dtype=float)
    if omega.shape != alpha.shape or omega.shape != torque_residual.shape or omega.shape[1] != 3:
        raise ValueError("omega、alpha、torque_residual 都必须是 shape=(N, 3)。")
    param_mask = _PARAM_MASK_XZ_YZ_ZERO if constrain_xz_yz_zero else None
    blocks = [inertia_regressor_row(omega[i], alpha[i], param_mask) for i in range(omega.shape[0])]
    y = np.vstack(blocks)
    b = torque_residual.reshape(-1)
    triangle_constraint_active = False
    if constrain_xz_yz_zero:
        theta_partial, triangle_constraint_active, rank, singular_values = (
            solve_lstsq_with_lower_halfspace(y, b, _TRIANGLE_IZZ_LE_IXX_IYY)
        )
    else:
        theta_partial, _, rank, singular_values = np.linalg.lstsq(y, b, rcond=None)
    pred = (y @ theta_partial).reshape((-1, 3))
    err = torque_residual - pred
    # 还原 6 维 theta (约束模式下缺失的列用 0 补齐)
    if constrain_xz_yz_zero:
        theta_full = np.zeros(6)
        theta_full[param_mask] = theta_partial
    else:
        theta_full = theta_partial
    inertia = theta_to_inertia_matrix(theta_full)
    eigvals = np.linalg.eigvalsh(inertia)
    physical = {
        "symmetric": bool(np.allclose(inertia, inertia.T)),
        "positive_definite": bool(np.all(eigvals > 0.0)),
        "triangle_inequality": inertia_triangle_inequality_ok(inertia),
        "eigenvalues": array_to_list(eigvals),
    }
    n_cols = int(param_mask.sum()) if param_mask is not None else 6
    return {
        "theta": theta_full,
        "inertia_com": inertia,
        "constrained": constrain_xz_yz_zero,
        "triangle_constraint_active": triangle_constraint_active,
        "rank": int(rank),
        "condition_number": float(np.linalg.cond(y)) if y.size else float("inf"),
        "singular_values": singular_values,
        "torque_rms": float(np.sqrt(np.mean(err ** 2))),
        "physical": physical,
    }


def torque_rotational_component(
    torque_g: np.ndarray, force_g: np.ndarray, com_m: np.ndarray,
) -> np.ndarray:
    """从 force_sensor_gravity 中分离出旋转动力学项.

    F_g = -m a_C,  T_g = -(I α + ω × I ω) + com × F_g
    => I α + ω × I ω = com × F_g - T_g
    """
    com_m = np.asarray(com_m, dtype=float).reshape(3)
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

    # ω/α 来自 server pinocchio FK → URDF ee_link 系.
    # force_sensor_gravity 与 com 来自 MapSriRawToEE → SRI-EE 系.
    # 两者差一个 R_z(180°) (传感器相对于法兰绕 z 旋转 180° 安装):
    #   v_SRI = R_z(180°) · v_URDF = [-v_x, -v_y, v_z]
    # 把 ω/α 旋到 SRI-EE 系, 与 wrench/com 统一坐标系.
    omega[:, 0] *= -1.0
    omega[:, 1] *= -1.0
    alpha[:, 0] *= -1.0
    alpha[:, 1] *= -1.0

    if not np.all(np.isfinite(omega)) or not np.all(np.isfinite(alpha)):
        n_bad = int(np.sum(~np.isfinite(omega).all(axis=1) | ~np.isfinite(alpha).all(axis=1)))
        raise ValueError(
            f"omega/alpha 中存在 {n_bad} 帧 NaN, 通常是 server 端运动学预处理判定无效 "
            "(manager 不在 Idle/Running, 或反馈失效, 或首拍). 请确认 server 处于 Idle/Running, "
            "并丢弃首拍/无效帧后再采集。"
        )

    if extra_filter_fc_hz is not None and extra_filter_fc_hz > 0:
        # server 已 1 kHz Butterworth, 这里仅在用户显式要求时再过一次. 仅滤 wrench, 不滤 ω/α
        # (避免与 server 滤波链相位错配).
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
        raise ValueError("首尾裁剪后剩余样本太少, 请缩短 --trim-start/--trim-end 或增加采集时长。")
    return {
        "wrench_gravity_F": wrench_g[si:ei],
        "omega": omega[si:ei],
        "alpha": alpha[si:ei],
        "torque_rot": torque_rot[si:ei],
    }


def _slice_arm_6(arr: np.ndarray, arm: str) -> np.ndarray:
    """从 12 维 (primary[0:6] + secondary[6:12]) 或 6 维 (primary[0:3] + secondary[3:6]) 切出.

    本文件中 force_sensor_gravity 是 12 维 wrench, ω/α 是 6 维 (primary 各 3, secondary 各 3 留 NaN).
    根据数组长度判断每段宽度.
    """
    arr = np.asarray(arr, dtype=float)
    if arr.size in (6, 12):
        per = arr.size // 2
        return arr[:per].copy() if arm == "primary" else arr[per:per * 2].copy()
    raise ValueError(f"无法从长度 {arr.size} 的数组按 arm 切片。")


def sample_from_state_dict(data: Dict[str, object], arm: str) -> DynamicSample:
    """从 robot_server_state JSON 字段中解析 F 系 wrench/ω/α.

    必须字段: force_sensor_gravity, ee_omega_ee_frame, ee_alpha_ee_frame.
    可选字段 (仅诊断): joint_position, joint_velocity (按 arm 切单臂 7 维).
    """
    timestamp_raw = float(data.get("timestamp", 0.0))
    timestamp = timestamp_raw * 1e-9 if timestamp_raw > 1e12 else timestamp_raw

    if "force_sensor_gravity" not in data:
        raise ValueError("force_sensor_gravity 字段缺失, 请升级 server 到包含该字段的版本。")
    if "ee_omega_ee_frame" not in data or "ee_alpha_ee_frame" not in data:
        raise ValueError(
            "ee_omega_ee_frame / ee_alpha_ee_frame 字段缺失, 请升级 server 到发布 F 系运动学的版本。"
        )

    wrench_g = _slice_arm_6(data["force_sensor_gravity"], arm)
    if wrench_g.size != 6:
        raise ValueError(f"force_sensor_gravity 切片得到 {wrench_g.size} 维, 期望 6 维。")
    omega_F = _slice_arm_6(data["ee_omega_ee_frame"], arm)
    alpha_F = _slice_arm_6(data["ee_alpha_ee_frame"], arm)
    if omega_F.size != 3 or alpha_F.size != 3:
        raise ValueError(
            f"ee_omega/alpha 切片得到 {omega_F.size}/{alpha_F.size} 维, 期望 3 维 "
            "(ω/α 应为 6 维, primary[0:3] + secondary[3:6])。"
        )

    q_raw: Optional[np.ndarray] = None
    dq_raw: Optional[np.ndarray] = None
    q_all = data.get("joint_position")
    dq_all = data.get("joint_velocity")
    if q_all is not None and dq_all is not None:
        q_arr = np.asarray(q_all, dtype=float)
        dq_arr = np.asarray(dq_all, dtype=float)
        if q_arr.size == 7:
            q_raw, dq_raw = q_arr.copy(), dq_arr.copy()
        elif q_arr.size >= 13:
            if arm == "primary":
                q_raw = q_arr[6:13].copy()
                dq_raw = dq_arr[6:13].copy()
            elif q_arr.size >= 20:
                q_raw = q_arr[13:20].copy()
                dq_raw = dq_arr[13:20].copy()

    return DynamicSample(time.monotonic(), timestamp, wrench_g, omega_F, alpha_F, q_raw, dq_raw)


def sample_to_json(sample: DynamicSample) -> Dict[str, object]:
    out: Dict[str, object] = {
        "timestamp": sample.timestamp,
        "wrench_gravity_F": array_to_list(sample.wrench_gravity_F),
        "omega_F": array_to_list(sample.omega_F),
        "alpha_F": array_to_list(sample.alpha_F),
    }
    if sample.q_raw is not None:
        out["q_raw"] = array_to_list(sample.q_raw)
    if sample.dq_raw is not None:
        out["dq_raw"] = array_to_list(sample.dq_raw)
    return out


def samples_from_jsonl(path: Path) -> List[DynamicSample]:
    """读取已采集的 jsonl 离线复跑. 仅识别新格式 (含 wrench_gravity_F/omega_F/alpha_F)."""
    samples: List[DynamicSample] = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            if not line.strip():
                continue
            data = json.loads(line)
            for key in ("wrench_gravity_F", "omega_F", "alpha_F"):
                if key not in data:
                    raise ValueError(
                        f"jsonl 缺少字段 {key}: 该文件可能由旧版本脚本生成, 请用新版重新采集。"
                    )
            samples.append(DynamicSample(
                arrival_time=time.monotonic(),
                timestamp=float(data.get("timestamp", 0.0)),
                wrench_gravity_F=np.asarray(data["wrench_gravity_F"], dtype=float),
                omega_F=np.asarray(data["omega_F"], dtype=float),
                alpha_F=np.asarray(data["alpha_F"], dtype=float),
                q_raw=np.asarray(data["q_raw"], dtype=float) if "q_raw" in data else None,
                dq_raw=np.asarray(data["dq_raw"], dtype=float) if "dq_raw" in data else None,
            ))
    return samples


def write_jsonl(path: Path, samples: Iterable[DynamicSample]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        for sample in samples:
            f.write(json.dumps(sample_to_json(sample), ensure_ascii=False) + "\n")


class ToolInertiaNode:
    """在线采集节点: 订阅 robot_server_state, 可发布关节位置激励。"""

    def __init__(self, state_topic: str, arm: str):
        import rclpy
        from joint_interfaces.msg import JointPositionControl
        from rclpy.node import Node
        from std_msgs.msg import Header, String

        self.arm = arm

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
            return [sample for sample in self.samples if sample.arrival_time >= start_time]

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
            raise RuntimeError("没有获取到当前关节位置, 无法启动自动激励。")
        if not wrapper.get_joint_name():
            raise RuntimeError("没有获取到 joint_name, 无法发布 JointPositionControl 自动激励。")

        amplitudes = parse_float_list(args.excitation_amplitudes, -1, "--excitation-amplitudes")
        frequencies = parse_float_list(args.excitation_frequencies, -1, "--excitation-frequencies")
        phases = parse_float_list(args.excitation_phases, -1, "--excitation-phases")
        if not (len(amplitudes) == len(frequencies) == len(phases) and len(amplitudes) in (3, 4)):
            raise RuntimeError(
                "excitation 幅值/频率/相位数组长度必须相等且为 3 (J5/J6/J7) 或 4 (J4/J5/J6/J7)。")
        excitation_count = len(amplitudes)
        wrist_indices = wrist_indices_for_position(base_position, args.arm, excitation_count)
        if not wrist_indices or max(wrist_indices) >= base_position.size:
            raise RuntimeError(f"关节数组长度 {base_position.size} 不支持 {excitation_count} 关节激励。")

        print("数据已连接, 将启动推荐腕部激励。")
        print(f"中心姿态取当前反馈位置; 激励关节索引: {wrist_indices}")
        print(f"幅值 rad: {amplitudes}")
        print(f"频率 Hz:  {frequencies}")
        print(f"相位 rad: {phases}")
        print(f"时长:     {args.duration:.2f}s, 首尾平滑时间: {args.excitation_ramp:.2f}s")
        input("确认周围安全、无碰撞风险后按 Enter 开始自动激励和采集 > ")

        start = time.monotonic()
        next_t = time.monotonic()
        print("自动激励中, 请保持人员和物体远离机械臂工作空间。")
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

        print("激励结束, 发送中心姿态保持 1 秒。")
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


def result_to_json(
    result: Dict[str, object], args: argparse.Namespace, sample_count: int,
) -> Dict[str, object]:
    inertia = np.asarray(result["inertia_com"], dtype=float)
    theta = np.asarray(result["theta"], dtype=float)
    n_params = 4 if result.get("constrained") else 6
    return {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "sample_count": int(sample_count),
        "arm": args.arm,
        "dt": args.dt,
        "constrained": bool(result.get("constrained", False)),
        "triangle_constraint_active": bool(result.get("triangle_constraint_active", False)),
        "wrench_source": "force_sensor_gravity (server 端已扣零偏 + 已扣工具静态重力, F 系)",
        "kinematics_source": "ee_omega_ee_frame / ee_alpha_ee_frame (server 端 1 kHz Butterworth 滤波后, F 系)",
        "mass_kg": args.mass,
        "com_m": array_to_list(parse_float_list(args.com_mm, 3, "--com-mm") / 1000.0),
        "com_mm": array_to_list(parse_float_list(args.com_mm, 3, "--com-mm")),
        "theta_order": _PARAM_NAMES,
        "theta": array_to_list(theta),
        "inertia_com_kg_m2": inertia.tolist(),
        "rank": f"{result['rank']} / {n_params}",
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
    print(f"运动学来源:        {payload['kinematics_source']}")
    print(f"工具质量 kg:       {payload['mass_kg']}")
    print(f"质心偏移 mm:       {np.array(payload['com_mm'])}")
    print(f"参数顺序:          {payload['theta_order']}")
    print(f"参数向量:          {np.array(payload['theta'])}")
    print(f"^F I_C kg*m^2:\n{np.array(payload['inertia_com_kg_m2'])}")
    print(f"力矩残差 RMS:      {float(payload['torque_rms']):.6g} Nm")
    if payload.get("constrained"):
        constraint_text = "Ixz=Iyz=0, Izz<=Ixx+Iyy (4 params)"
        if payload.get("triangle_constraint_active"):
            constraint_text += " [triangle active]"
    else:
        constraint_text = "无约束 (6 params)"
    print(f"约束模式:          {constraint_text}")
    print(f"回归矩阵秩:        {payload['rank']}")
    print(f"条件数:            {float(payload['condition_number']):.6g}")
    physical = payload["physical"]
    print(f"正定检查:          {physical['positive_definite']}")
    print(f"三角不等式检查:    {physical['triangle_inequality']}")
    if float(payload["condition_number"]) > 1e4:
        print("警告: 条件数较大, 建议增加绕多个方向的旋转激励。")
    print("\n===== robot_config.yaml 建议字段 =====")
    inertia = np.array(payload["inertia_com_kg_m2"])
    print("tool_inertia:")
    print(f"  inertia_com_kg_m2: [[{inertia[0,0]:.8g}, {inertia[0,1]:.8g}, {inertia[0,2]:.8g}],")
    print(f"                     [{inertia[1,0]:.8g}, {inertia[1,1]:.8g}, {inertia[1,2]:.8g}],")
    print(f"                     [{inertia[2,0]:.8g}, {inertia[2,1]:.8g}, {inertia[2,2]:.8g}]]")
    print("============================================\n")


def run_self_test() -> None:
    """合成 ω_F / α_F / F_g / T_g, 验证 LS 链路自洽."""
    rng = np.random.default_rng(42)
    theta_true = np.array([0.018, 0.026, 0.031, 0.002, -0.003, 0.004])
    I_true = theta_to_inertia_matrix(theta_true)
    com = np.array([0.001, -0.002, 0.05])
    mass = 1.0

    n, dt = 4000, 0.005
    t = np.arange(n) * dt
    omega_F = np.column_stack([
        0.30 * np.sin(2.0 * np.pi * 0.40 * t + 0.0),
        0.25 * np.sin(2.0 * np.pi * 0.65 * t + 1.0),
        0.20 * np.sin(2.0 * np.pi * 0.90 * t + 2.0),
    ])
    # α 用解析导数 (避免数值差分误差进入 self-test)
    alpha_F = np.column_stack([
        0.30 * (2.0 * np.pi * 0.40) * np.cos(2.0 * np.pi * 0.40 * t + 0.0),
        0.25 * (2.0 * np.pi * 0.65) * np.cos(2.0 * np.pi * 0.65 * t + 1.0),
        0.20 * (2.0 * np.pi * 0.90) * np.cos(2.0 * np.pi * 0.90 * t + 2.0),
    ])

    a_C_F = rng.normal(scale=0.5, size=(n, 3))
    F_g = -mass * a_C_F
    T_g = -np.array([
        I_true @ alpha_F[i] + np.cross(omega_F[i], I_true @ omega_F[i]) for i in range(n)
    ]) + np.cross(com, F_g)

    samples = [
        DynamicSample(0.0, 0.0,
                      wrench_gravity_F=np.concatenate([F_g[i], T_g[i]]),
                      omega_F=omega_F[i], alpha_F=alpha_F[i])
        for i in range(n)
    ]
    arrays = prepare_fit_arrays(samples, com_m=com,
                                trim_start_sec=0.5, trim_end_sec=0.5, dt=dt)
    result = fit_inertia_from_rotational_terms(
        arrays["omega"], arrays["alpha"], arrays["torque_rot"],
        constrain_xz_yz_zero=False,  # self-test 使用全 6-param 验证
    )
    err = np.linalg.norm(np.asarray(result["theta"]) - theta_true)
    print(f"self-test: theta_err={err:.6e}, torque_rms={result['torque_rms']:.6e}")
    if err > 1e-10:
        raise RuntimeError(f"self-test 失败: theta={result['theta']}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="通过 force_sensor_gravity + ee_omega/alpha_ee_frame 辨识工具绕质心的转动惯量 ^F I_C。",
    )
    parser.add_argument("--arm", choices=["primary", "secondary"], default="primary",
                        help="选择左臂 primary 或右臂 secondary (secondary 当前未支持, ω/α 为 NaN)。")
    parser.add_argument("--state-topic", default=DEFAULT_STATE_TOPIC)
    parser.add_argument("--input", type=Path, help="已有 jsonl 文件; 不填则在线采集。")
    parser.add_argument("--duration", type=float, default=DEFAULT_EXCITATION_DURATION)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--dt", type=float, default=DEFAULT_DT)
    parser.add_argument("--mass", type=float, default=DEFAULT_TOOL_MASS_KG,
                        help="工具质量 kg, 仅用于结果元数据 (不参与回归)。")
    parser.add_argument("--com-mm", default=DEFAULT_TOOL_COM_MM,
                        help="质心相对 EE 原点偏移, 单位 mm, 格式 x,y,z (F 系)。")
    parser.add_argument("--no-excitation", action="store_true",
                        help="不自动下发激励, 只采集; 需人工执行运动。")
    parser.add_argument("--excitation-amplitudes", default=DEFAULT_EXCITATION_AMPLITUDES)
    parser.add_argument("--excitation-frequencies", default=DEFAULT_EXCITATION_FREQUENCIES)
    parser.add_argument("--excitation-phases", default=DEFAULT_EXCITATION_PHASES)
    parser.add_argument("--excitation-ramp", type=float, default=DEFAULT_EXCITATION_RAMP)
    parser.add_argument("--trim-start", type=float, default=1.0)
    parser.add_argument("--trim-end", type=float, default=1.0)
    parser.add_argument("--filter-fc", type=float, default=0.0,
                        help=("可选: 仅对 wrench 再做一次零相位 Butterworth (Hz)。"
                              "默认 0 = 不再滤; server 已 1 kHz Butterworth, "
                              "再滤会引入相位偏移, 谨慎使用。"))
    parser.add_argument(
        "--no-constrain-xz-yz", dest="constrain_xz_yz_zero",
        action="store_false", default=True,
        help="关闭 Ixz = Iyz = 0 物理约束 (恢复 6-param LS)。",
    )
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        return args
    if args.dt <= 0.0:
        parser.error("--dt 必须为正数。")
    if args.mass <= 0.0:
        parser.error("--mass 必须为正数。")
    if args.duration <= 0.0 and not args.no_excitation:
        parser.error("自动激励模式下 --duration 必须为正数; 如需手动停止, 使用 --no-excitation。")
    if args.excitation_ramp < 0.0:
        parser.error("--excitation-ramp 不能为负数。")
    parse_float_list(args.com_mm, 3, "--com-mm")
    amplitudes = parse_float_list(args.excitation_amplitudes, -1, "--excitation-amplitudes")
    frequencies = parse_float_list(args.excitation_frequencies, -1, "--excitation-frequencies")
    phases = parse_float_list(args.excitation_phases, -1, "--excitation-phases")
    if not (len(amplitudes) == len(frequencies) == len(phases) and len(amplitudes) in (3, 4)):
        parser.error(
            "excitation 幅值/频率/相位数组长度必须相等且为 3 (J5/J6/J7) 或 4 (J4/J5/J6/J7)。")
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
    print(f"采样周期 dt:       {args.dt:.6f} s")
    print(f"工具质量:          {args.mass:.6f} kg")
    print(f"质心偏移 com_mm:   {args.com_mm}")
    print("六维力来源:        force_sensor_gravity (零偏 + 重力补偿后, F 系)")
    print("运动学来源:        ee_omega/alpha_ee_frame (server 1 kHz Butterworth 后, F 系)")
    if args.filter_fc and args.filter_fc > 0:
        print(f"额外滤波 (仅 wrench): {args.filter_fc:.2f} Hz")
    if args.input:
        print("运动模式:          离线拟合, 不下发运动")
    elif args.no_excitation:
        print("运动模式:          只采集, 不自动下发激励")
    else:
        print("运动模式:          自动下发推荐 J5/J6/J7 多频正弦激励")
        print(f"激励幅值 rad:      {args.excitation_amplitudes}")
        print(f"激励频率 Hz:       {args.excitation_frequencies}")
        print(f"激励相位 rad:      {args.excitation_phases}")
        print(f"激励时长:          {args.duration:.2f} s")
    print("======================================\n")

    if args.input:
        print(f"读取离线采集文件: {args.input.expanduser()}")
        samples = samples_from_jsonl(args.input.expanduser())
        capture_path = args.input.expanduser()
    else:
        samples = capture_online(args)
        capture_path = args.output_dir / f"tool_inertia_capture_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl"
        write_jsonl(capture_path, samples)
        print(f"采集数据已保存: {capture_path}")

    com_m = parse_float_list(args.com_mm, 3, "--com-mm") / 1000.0
    extra_fc = args.filter_fc if args.filter_fc and args.filter_fc > 0 else None

    print("开始处理 wrench_gravity_F / ω_F / α_F 并构建回归方程...")
    arrays = prepare_fit_arrays(
        samples, com_m=com_m,
        trim_start_sec=args.trim_start, trim_end_sec=args.trim_end, dt=args.dt,
        extra_filter_fc_hz=extra_fc,
    )
    print(f"开始拟合 ^F I_C, 有效样本数: {len(arrays['omega'])}")
    constrain = getattr(args, "constrain_xz_yz_zero", False)
    result = fit_inertia_from_rotational_terms(
        arrays["omega"], arrays["alpha"], arrays["torque_rot"],
        constrain_xz_yz_zero=constrain,
    )
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
