#!/usr/bin/env python3
"""通过末端动态激励辨识工具绕质心的转动惯量。

数据处理约定:
- q 使用 /feedback/robot_server_state 里的 joint_position。
- dq 使用同一条 robot_server_state 里的 joint_velocity (电机 feedback 关节角速度)。
- 末端角速度由 J_omega(q) * dq_feedback 得到，角加速度由这条末端角速度序列差分得到。
- 默认使用同一条 robot_server_state 里的补偿前 raw_force_sensor, 避免运行时工具重力补偿消掉待辨识信号。
- 输出惯量为 ^F I_C: 绕工具质心 C, 方向表达在末端法兰/EE 坐标系 F。
- 力/力矩约定与 tool_com_estimate.py 一致 (tool-on-sensor):
      F_meas ≈  m R^T g_W + m a_C^F        (静止时 F_meas = m R^T g_W)
      M_meas ≈  com x F_unbiased + (I alpha + omega x I omega)
  因此拟合用的旋转动力学残差是
      I alpha + omega x I omega = T_unbiased - com x F_unbiased
  (符号易错: tool_com_estimate.py 在静止情形得到 T_meas = com x F_meas,
   动态旋转项的符号必须与 server 惯量补偿扣除的 wrench 一致, 否则惯量出现负对角元。)
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
DEFAULT_TOOL_COM_MM = "-0.15804154,0.70721872,50.33428542"
DEFAULT_TOOL_W_BIAS_RAW_SRI = None  # None 表示从 server 状态自动读取
DEFAULT_EXCITATION_DURATION = 120.0
DEFAULT_EXCITATION_AMPLITUDES = "0.30,0.20,0.15"
DEFAULT_EXCITATION_FREQUENCIES = "0.40,0.65,0.90"
DEFAULT_EXCITATION_PHASES = "0.0,1.57079632679,1.0471975512"
DEFAULT_EXCITATION_RAMP = 2.0
DEFAULT_FILTER_FC_HZ = 5.0


@dataclass
class DynamicSample:
    """单个动态采样点: 7 轴关节位置/速度和映射到 EE/F 坐标系的六维力。

    rotation_world_from_ee 来自 server 的 ee_pose_world, 与 tool_com_estimate.py
    保持一致: 这是 URDF ee_link 系到 world 系的旋转。当前惯量拟合不再用姿态差分
    计算 omega/alpha, 保留该字段只用于离线诊断/兼容采集文件。
    """

    arrival_time: float
    timestamp: float
    q_raw: np.ndarray
    dq_raw: np.ndarray
    wrench_ee: np.ndarray
    rotation_world_from_ee: Optional[np.ndarray] = None


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


def butter_filtfilt_2d(signal: np.ndarray, fc_hz: float, dt: float, order: int = 3) -> np.ndarray:
    """对 (N, K) 数组按列做零相位 Butterworth 滤波。"""
    if fc_hz <= 0:
        return signal.copy()
    wc = fc_hz * 2.0 * math.pi * dt
    out = np.empty_like(signal)
    for k in range(signal.shape[1]):
        out[:, k] = butter_filtfilt(order, wc, signal[:, k])
    return out


def diff_and_filter_q(
    q_raw: np.ndarray,
    dt: float,
    wf: float = DEFAULT_WF,
    harmonics: int = DEFAULT_L,
    fc_hz: Optional[float] = None,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """由位置序列生成 q/dq/ddq。

    fc_hz 显式传入时, 直接使用该截止频率; 否则按 DEFAULT_WF/DEFAULT_L 估算
    (用于和 qt_dyn_params_ident.py 默认行为兼容)。
    """
    if fc_hz is not None and fc_hz > 0:
        fc = float(fc_hz)
    else:
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


def rot_y(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=float)


def dh_frame_to_ee_rotation() -> np.ndarray:
    """URDF jointEE fixed rotation: parent arm7/DH frame -> child ee_link."""
    return rot_y(-math.pi / 2.0)


def ee_rotation_and_joint_axes(q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """计算 DH frame7 姿态和每个关节轴在 DH base 系下的方向。

    返回:
        rotation: ^B R_D, DH frame7 到 DH base 系的旋转
        axes:     7x3, 每个关节旋转轴在 DH base 系下的单位向量
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


def compute_body_omega_alpha_from_joint_feedback(
    q_raw: np.ndarray,
    dq_raw: np.ndarray,
    arm: str,
    robot_type: str,
    dt: float,
    fc_hz: Optional[float] = None,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """由 robot_server_state 的 joint_velocity 计算 EE/F 系 omega 和 alpha。

    joint_velocity 是电机反馈关节角速度。这里先由 J_omega(q) * dq_feedback
    得到末端角速度, 再对这条末端角速度序列做中心差分得到角加速度。
    这样保留了 J_omega(q) 随姿态变化带来的项, 与 server 端惯量补偿链路一致。
    最后应用 URDF jointEE 的固定旋转, 显式旋到传感器 wrench 使用的 ee_link/F 系。
    """
    q_raw = np.asarray(q_raw, dtype=float)
    dq_raw = np.asarray(dq_raw, dtype=float)
    if q_raw.shape != dq_raw.shape or q_raw.ndim != 2 or q_raw.shape[1] != 7:
        raise ValueError("q_raw 和 dq_raw 必须都是 shape=(N, 7)。")
    q_model = np.vstack([get_actual_q(q_raw[i], arm, robot_type) for i in range(q_raw.shape[0])])

    rotations = []
    omega_base = []
    for i in range(q_model.shape[0]):
        rotation_dh, axes_base = ee_rotation_and_joint_axes(q_model[i])
        rotation_ee = rotation_dh @ dh_frame_to_ee_rotation()
        rotations.append(rotation_ee)
        omega_base.append(axes_base.T @ dq_raw[i])

    rotations_arr = np.asarray(rotations)
    omega_base_arr = np.asarray(omega_base)
    alpha_base_arr = np.vstack([central_diff(omega_base_arr[:, j], dt, 2) for j in range(3)]).T

    omega_body = np.einsum("nij,nj->ni", np.transpose(rotations_arr, (0, 2, 1)), omega_base_arr)
    alpha_body = np.einsum("nij,nj->ni", np.transpose(rotations_arr, (0, 2, 1)), alpha_base_arr)
    if fc_hz is not None and fc_hz > 0:
        omega_body = butter_filtfilt_2d(omega_body, fc_hz, dt)
        alpha_body = butter_filtfilt_2d(alpha_body, fc_hz, dt)
    return rotations_arr, omega_body, alpha_body


def quat_wxyz_to_rot(q: np.ndarray) -> np.ndarray:
    """四元数 (w, x, y, z) -> 3x3 旋转矩阵, 与 tool_com_estimate.py 一致。"""
    q = np.asarray(q, dtype=float).reshape(4)
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n <= 0.0:
        raise ValueError("四元数模为 0。")
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def body_omega_alpha_from_rotations(
    rotations_world_from_ee: np.ndarray,
    dt: float,
    fc_hz: Optional[float] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """直接由 ^W R_E(t) 计算 ee_link 系的角速度 omega 和角加速度 alpha.

    这与 tool_com_estimate.py / server 重力补偿使用同一个 ee_link 系, 避免 DH 与
    URDF 之间的常值旋转差异。

    步骤:
      1. omega_world(t) ≈ vee( dR/dt · R^T ) 用中心差分实现;
      2. omega_body = R^T omega_world;
      3. alpha_body 由 omega_body 中心差分得到;
      4. 可选 Butterworth 零相位滤波。
    """
    rotations = np.asarray(rotations_world_from_ee, dtype=float)
    n = rotations.shape[0]
    if n < 5:
        raise ValueError("旋转序列至少需要 5 帧。")
    omega_world = np.zeros((n, 3))
    for i in range(n):
        if i == 0:
            r_minus = rotations[0]
            r_plus = rotations[1]
            denom = dt
        elif i == n - 1:
            r_minus = rotations[-2]
            r_plus = rotations[-1]
            denom = dt
        else:
            r_minus = rotations[i - 1]
            r_plus = rotations[i + 1]
            denom = 2.0 * dt
        d_r = (r_plus - r_minus) / denom
        skew_w = d_r @ rotations[i].T
        omega_world[i] = np.array(
            [skew_w[2, 1] - skew_w[1, 2], skew_w[0, 2] - skew_w[2, 0], skew_w[1, 0] - skew_w[0, 1]]
        ) * 0.5
    if fc_hz is not None and fc_hz > 0:
        omega_world = butter_filtfilt_2d(omega_world, fc_hz, dt)
    omega_body = np.einsum("nij,nj->ni", np.transpose(rotations, (0, 2, 1)), omega_world)
    alpha_body = np.vstack([central_diff(omega_body[:, j], dt, 2) for j in range(3)]).T
    if fc_hz is not None and fc_hz > 0:
        alpha_body = butter_filtfilt_2d(alpha_body, fc_hz, dt)
    return omega_body, alpha_body


def torque_rotational_component(
    torque_unbiased: np.ndarray,
    force_unbiased: np.ndarray,
    com_m: np.ndarray,
) -> np.ndarray:
    """从扣零偏后的法兰系六维力中分离出旋转动力学项。

    采用 tool-on-sensor 约定（与 tool_com_estimate.py 一致）:
        F_meas = m * R^T g_W + m a_C^F + bias_F
        M_meas = [I alpha + omega x I omega] + com x F_unbiased + bias_T

    扣零偏后整理得到:
        I alpha + omega x I omega = T_unbiased - com x F_unbiased

    这里 com_m = r_FC, 即从法兰原点指向工具质心的向量, 表达在法兰系。
    """
    com_m = np.asarray(com_m, dtype=float).reshape(3)
    return torque_unbiased - np.cross(com_m, force_unbiased)


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
    filter_fc_hz: Optional[float] = None,
) -> Dict[str, np.ndarray]:
    """把原始采样数据整理为惯量拟合需要的数组。

    处理步骤:
    1. 从 server 的 joint_velocity 得到末端角速度, 并由末端角速度序列差分得到角加速度。
       二者再旋到 EE/F 系。
    2. 对六维力扣除零偏, 并用同一截止频率做零相位滤波, 与运动学保持频谱对齐。
    3. 从 ee_link 力矩中扣除 com x force, 剩余部分主要用于拟合旋转惯量。

    关键: joint_velocity 是电机 feedback 的关节角速度, 不是由 joint_position 离线
    差分得到的速度。角加速度从末端角速度差分得到, 因而包含 J_omega(q) 的姿态变化项。
    DH frame7 还会经过 URDF jointEE 固定旋转后再进入回归, 与 wrench_ee、com_m
    和输出的 ^F I_C 保持同一 EE/F 坐标系。
    """
    if len(samples) < 20:
        raise ValueError("动态采样点至少需要 20 个。")
    q_raw = np.vstack([sample.q_raw for sample in samples])
    dq_raw = np.vstack([sample.dq_raw for sample in samples])
    wrench = np.vstack([sample.wrench_ee for sample in samples])
    _, omega, alpha = compute_body_omega_alpha_from_joint_feedback(
        q_raw, dq_raw, arm, robot_type, dt, fc_hz=filter_fc_hz
    )

    if filter_fc_hz is not None and filter_fc_hz > 0:
        wrench = np.hstack([
            butter_filtfilt_2d(wrench[:, :3], filter_fc_hz, dt),
            butter_filtfilt_2d(wrench[:, 3:6], filter_fc_hz, dt),
        ])

    force_unbiased = wrench[:, :3] - w_bias_ee[:3]
    torque_unbiased = wrench[:, 3:6] - w_bias_ee[3:6]
    torque_rot = torque_rotational_component(torque_unbiased, force_unbiased, com_m)

    si = max(0, int(round(trim_start_sec / dt)))
    ei = len(samples) - max(0, int(round(trim_end_sec / dt)))
    if ei <= si + 10:
        raise ValueError("首尾裁剪后剩余样本太少，请缩短 --trim-start/--trim-end 或增加采集时长。")
    return {
        "wrench_ee": wrench[si:ei],
        "q_raw": q_raw[si:ei],
        "dq_raw": dq_raw[si:ei],
        "omega": omega[si:ei],
        "alpha": alpha[si:ei],
        "torque_rot": torque_rot[si:ei],
    }


def _parse_sensor_bias_ee(data: Dict[str, object], arm: str) -> Optional[np.ndarray]:
    """从 robot_server_state JSON 中提取单臂的 force_sensor_bias (EE系)。

    server 发布的 force_sensor_bias 已经是 EE 系，直接取对应臂的 6 个值即可。
    primary -> [0:6], secondary -> [6:12]
    """
    raw = data.get("force_sensor_bias")
    if raw is None:
        return None
    try:
        arr = np.asarray(raw, dtype=float)
    except (ValueError, TypeError):
        return None
    if arr.size < 6:
        return None
    if arr.size >= 12:
        return arr[:6].copy() if arm == "primary" else arr[6:12].copy()
    return arr[:6].copy()


def sensor_bias_ee_to_raw_sri(bias_ee: np.ndarray) -> np.ndarray:
    """把 EE 系零偏反映射回 raw_sri 顺序。

    map_sri_raw_wrench_to_ee: ee = [-raw[0], -raw[1], raw[2], -raw[3], -raw[4], raw[5]]
    逆变换:                   raw = [-ee[0],  -ee[1], ee[2],  -ee[3],  -ee[4], ee[5]]
    """
    b = np.asarray(bias_ee, dtype=float)
    return np.array([-b[0], -b[1], b[2], -b[3], -b[4], b[5]], dtype=float)


def samples_from_jsonl(path: Path, arm: str, wrench_format: str) -> Tuple[List[DynamicSample], Optional[np.ndarray]]:
    """读取离线 JSONL 采样文件，便于反复调整滤波和拟合参数。

    返回 (samples, bias_ee)，bias_ee 取自文件中第一条含 force_sensor_bias 的记录；
    若文件中没有该字段则返回 None。
    """
    samples = []
    bias_ee: Optional[np.ndarray] = None
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            if not line.strip():
                continue
            data = json.loads(line)
            samples.append(sample_from_state_dict(data, arm, wrench_format))
            if bias_ee is None:
                bias_ee = _parse_sensor_bias_ee(data, arm)
    return samples, bias_ee


def sample_to_json(sample: DynamicSample) -> Dict[str, object]:
    out: Dict[str, object] = {
        "timestamp": sample.timestamp,
        "q_raw": array_to_list(sample.q_raw),
        "dq_raw": array_to_list(sample.dq_raw),
        "wrench_ee": array_to_list(sample.wrench_ee),
    }
    if sample.rotation_world_from_ee is not None:
        out["rotation_world_from_ee"] = sample.rotation_world_from_ee.tolist()
    return out


def sample_from_state_dict(data: Dict[str, object], arm: str, wrench_format: str) -> DynamicSample:
    """从 robot_server_state JSON 字段中提取单臂 q/dq、六维力, 以及 world->ee 旋转。

    q/dq 的切片规则与 qt_dyn_params_ident.py 和 server primary/secondary 配置一致:
        primary   -> joint_position[6:13]
        secondary -> joint_position[13:20]
    若输入已经是单臂 7 轴采集文件, 则直接使用 7 维 q_raw/dq_raw。
    旋转优先取 ee_pose_world (与 tool_com_estimate.py 保持一致, 即 ee_link 系到 world)。
    """
    timestamp_raw = float(data.get("timestamp", 0.0))
    timestamp = timestamp_raw * 1e-9 if timestamp_raw > 1e12 else timestamp_raw
    q_all = np.asarray(data.get("joint_position", data.get("q_raw")), dtype=float)
    dq_all = np.asarray(data.get("joint_velocity", data.get("dq_raw")), dtype=float)
    if q_all.size == 7:
        q_raw = q_all
    elif arm == "primary":
        q_raw = q_all[6:13]
    else:
        q_raw = q_all[13:20]
    if q_raw.size != 7:
        raise ValueError(f"无法从 joint_position(size={q_all.size}) 中提取 7 个单臂关节。")
    if dq_all.size == 7:
        dq_raw = dq_all
    elif arm == "primary":
        dq_raw = dq_all[6:13]
    else:
        dq_raw = dq_all[13:20]
    if dq_raw.size != 7:
        raise ValueError(f"无法从 joint_velocity(size={dq_all.size}) 中提取 7 个单臂关节速度。")

    if "wrench_ee" in data:
        wrench_ee = np.asarray(data["wrench_ee"], dtype=float)
    else:
        key = "raw_force_sensor" if wrench_format == "raw_sri" and "raw_force_sensor" in data else "force_sensor"
        force_sensor = np.asarray(data[key], dtype=float)
        if force_sensor.size == 6:
            raw = force_sensor[:6]
        else:
            raw = force_sensor[:6] if arm == "primary" else force_sensor[6:12]
        wrench_ee = map_sri_raw_wrench_to_ee(raw) if wrench_format == "raw_sri" else raw.copy()
    if wrench_ee.size != 6:
        raise ValueError(f"无法提取 6 维力数据，实际 size={wrench_ee.size}。")

    rotation = None
    if "rotation_world_from_ee" in data:
        rotation = np.asarray(data["rotation_world_from_ee"], dtype=float).reshape(3, 3)
    else:
        ee_pose_world_raw = data.get("ee_pose_world")
        if ee_pose_world_raw is not None:
            try:
                ee_pose_world = np.asarray(ee_pose_world_raw, dtype=float)
            except (ValueError, TypeError):
                ee_pose_world = None
            if ee_pose_world is not None and ee_pose_world.size >= 14:
                pose_world = ee_pose_world[:7] if arm == "primary" else ee_pose_world[7:14]
                if np.linalg.norm(pose_world[3:7]) > 1e-6:
                    rotation = quat_wxyz_to_rot(pose_world[3:7])
    return DynamicSample(time.monotonic(), timestamp, q_raw.copy(), dq_raw.copy(), wrench_ee.copy(), rotation)


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
                sensor_bias_ee = _parse_sensor_bias_ee(data, arm)
                with self.outer.lock:
                    self.outer.samples.append(sample)
                    self.outer.latest_joint_position = joint_position.copy()
                    self.outer.latest_joint_name = joint_name
                    if sensor_bias_ee is not None:
                        self.outer.latest_sensor_bias_ee = sensor_bias_ee

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
        self.latest_sensor_bias_ee: Optional[np.ndarray] = None
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

    def get_sensor_bias_ee(self) -> Optional[np.ndarray]:
        with self.lock:
            return self.latest_sensor_bias_ee.copy() if self.latest_sensor_bias_ee is not None else None

    def publish_joint_position(self, target_position: np.ndarray) -> None:
        self.node.publish_joint_position(target_position)


def capture_online(args: argparse.Namespace) -> Tuple[List[DynamicSample], Optional[np.ndarray]]:
    """在线采集动态激励数据。返回 (samples, bias_ee)，bias_ee 来自 server 的 force_sensor_bias 字段。"""
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
            return samples, wrapper.get_sensor_bias_ee()

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
        return samples, wrapper.get_sensor_bias_ee()
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


def result_to_json(result: Dict[str, object], args: argparse.Namespace, sample_count: int, w_bias_ee: np.ndarray) -> Dict[str, object]:
    """组织结果 JSON。字段名保留英文，便于后续程序读取。"""
    inertia = np.asarray(result["inertia_com"], dtype=float)
    theta = np.asarray(result["theta"], dtype=float)
    w_bias_raw_sri = (
        sensor_bias_ee_to_raw_sri(w_bias_ee) if args.wrench_format == "raw_sri" else w_bias_ee
    )
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
        "w_bias_raw_sri": array_to_list(w_bias_raw_sri),
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
    """合成数据自测试: 已知惯量 -> 生成力矩 -> 反解惯量。

    包含两部分:
      A. 直接 LS: 给定 omega/alpha, 验证回归求解正确;
      B. joint_velocity -> J_omega -> omega/alpha -> prepare_fit_arrays 端到端测试,
         验证脚本使用 feedback dq 链路而不是从 joint_position 再差分 dq。
    """
    rng = np.random.default_rng(42)
    theta_true = np.array([0.018, 0.026, 0.031, 0.002, -0.003, 0.004])
    inertia_true = theta_to_inertia_matrix(theta_true)
    omega = rng.normal(size=(240, 3))
    alpha = rng.normal(size=(240, 3))
    torque = np.vstack([inertia_true @ alpha[i] + np.cross(omega[i], inertia_true @ omega[i]) for i in range(omega.shape[0])])
    result = fit_inertia_from_rotational_terms(omega, alpha, torque)
    err = np.linalg.norm(np.asarray(result["theta"]) - theta_true)
    print(f"自测试 A (LS): theta_err={err:.6e}, torque_rms={result['torque_rms']:.6e}")
    if err > 1e-10:
        raise RuntimeError("合成数据惯量自测试 A 失败。")

    dt = 0.005
    n = 4000
    t = np.arange(n) * dt
    q_raw = np.zeros((n, 7))
    dq_raw = np.zeros((n, 7))
    for j in range(7):
        amp = 0.08 + 0.015 * j
        freq = 0.25 + 0.07 * j
        phase = 0.3 * j
        q_raw[:, j] = amp * np.sin(2.0 * np.pi * freq * t + phase)
        dq_raw[:, j] = amp * 2.0 * np.pi * freq * np.cos(2.0 * np.pi * freq * t + phase)

    com = np.array([0.001, -0.002, 0.05])
    mass = 1.0
    g_world = np.array([0.0, 0.0, -9.81])

    rotations, omega_b, alpha_b = compute_body_omega_alpha_from_joint_feedback(
        q_raw, dq_raw, "primary", "w1_anth", dt, fc_hz=None
    )
    wrench = np.zeros((n, 6))
    for i in range(n):
        force = mass * (rotations[i].T @ g_world) - mass * np.zeros(3)
        torque_rot = inertia_true @ alpha_b[i] + np.cross(omega_b[i], inertia_true @ omega_b[i])
        torque = np.cross(com, force) + torque_rot
        wrench[i, :3] = force
        wrench[i, 3:6] = torque

    samples = [
        DynamicSample(0.0, 0.0, q_raw[i].copy(), dq_raw[i].copy(), wrench[i].copy(), rotations[i].copy())
        for i in range(n)
    ]
    arrays = prepare_fit_arrays(
        samples,
        arm="primary",
        robot_type="w1_anth",
        dt=dt,
        com_m=com,
        w_bias_ee=np.zeros(6),
        trim_start_sec=0.5,
        trim_end_sec=0.5,
        filter_fc_hz=None,
    )
    result_b = fit_inertia_from_rotational_terms(arrays["omega"], arrays["alpha"], arrays["torque_rot"])
    theta_b = np.asarray(result_b["theta"])
    err_b = np.linalg.norm(theta_b - theta_true)
    print(f"自测试 B (joint_velocity->prepare_fit): theta_err={err_b:.6e}, torque_rms={result_b['torque_rms']:.6e}")
    if err_b > 1e-3:
        raise RuntimeError(f"端到端自测试 B 失败, theta={theta_b}")


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
        default=None,
        help=(
            "六维力零偏，raw_sri 顺序，格式同 tool_com_estimate.py 的 w_bias，"
            "例如 '13.22,-21.48,-24.07,0.16,-0.47,-0.66'。"
            "不填时自动从 server 状态的 force_sensor_bias 字段读取。"
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
    parser.add_argument(
        "--filter-fc",
        type=float,
        default=DEFAULT_FILTER_FC_HZ,
        help=(
            "运动学和力传感器共用的零相位 Butterworth 截止频率, 单位 Hz, "
            f"默认 {DEFAULT_FILTER_FC_HZ}。应高于激励主频且高于其谐波, 否则角加速度信号会被衰减; "
            "传 0 或负数则用 qt_dyn_params_ident 默认值 (1.8 Hz, 仅与该脚本兼容用)。"
        ),
    )
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
    if args.w_bias is not None:
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
    w_bias_source = "命令行" if args.w_bias else "自动从 server 读取"
    print(f"六维力零偏来源:    {w_bias_source}")
    if args.w_bias:
        print(f"六维力零偏 w_bias: {args.w_bias}")
    if args.filter_fc and args.filter_fc > 0:
        print(f"零相位滤波截止:    {args.filter_fc:.2f} Hz (运动学+力传感器共用)")
    else:
        print(f"零相位滤波截止:    qt_dyn_params_ident 默认 (~1.8 Hz)")
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
        samples, bias_ee_from_data = samples_from_jsonl(args.input.expanduser(), args.arm, args.wrench_format)
        capture_path = args.input.expanduser()
    else:
        samples, bias_ee_from_data = capture_online(args)
        capture_path = args.output_dir / f"tool_inertia_capture_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl"
        write_jsonl(capture_path, samples)
        print(f"采集数据已保存: {capture_path}")

    com_m = parse_float_list(args.com_mm, 3, "--com-mm") / 1000.0
    if args.w_bias:
        w_bias_raw = parse_float_list(args.w_bias, 6, "--w-bias")
        w_bias_ee = map_raw_sri_bias_to_ee(w_bias_raw) if args.wrench_format == "raw_sri" else w_bias_raw
        print(f"使用命令行零偏 (raw_sri): {array_to_list(w_bias_raw)}")
    elif bias_ee_from_data is not None:
        w_bias_ee = bias_ee_from_data
        if args.wrench_format == "raw_sri":
            w_bias_raw = sensor_bias_ee_to_raw_sri(w_bias_ee)
            print(f"自动读取零偏 (EE系):     {array_to_list(w_bias_ee)}")
            print(f"自动读取零偏 (raw_sri):  {array_to_list(w_bias_raw)}")
        else:
            print(f"自动读取零偏 (EE系):     {array_to_list(w_bias_ee)}")
    else:
        raise RuntimeError(
            "未能从 server 状态中读取 force_sensor_bias，请通过 --w-bias 手动指定零偏。"
        )
    print("开始处理 joint_position/joint_velocity、末端角速度/角加速度和六维力残差...")
    filter_fc = args.filter_fc if args.filter_fc and args.filter_fc > 0 else None
    arrays = prepare_fit_arrays(
        samples,
        args.arm,
        args.robot_type,
        args.dt,
        com_m,
        w_bias_ee,
        args.trim_start,
        args.trim_end,
        filter_fc_hz=filter_fc,
    )
    print(f"开始拟合 ^F I_C，有效样本数: {len(arrays['omega'])}")
    result = fit_inertia_from_rotational_terms(arrays["omega"], arrays["alpha"], arrays["torque_rot"])
    payload = result_to_json(result, args, len(arrays["omega"]), w_bias_ee)
    payload["capture_path"] = str(capture_path)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    result_path = args.output_dir / f"tool_inertia_result_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    with result_path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
    print_result(payload)
    print(f"结果已保存: {result_path}")


if __name__ == "__main__":
    main()
