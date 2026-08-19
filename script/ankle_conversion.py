#!/usr/bin/env python3
"""
L1 并联踝关节 —— 电机空间 ↔ 关节空间（pitch/roll）双向转换

基于 C++ parallel_ankle_calc.cpp 移植，几何参数与 l1_config.yaml 的 l1_v02 完全一致。

使用方式：
    from ankle_conversion import PMAnkle

    # 构造（左脚/右脚）
    ankle = PMAnkle(is_left=True)

    # ---------- 关节空间 → 电机空间（逆运动学 IK）----------
    theta1, theta2, ok1, ok2 = ankle.get_motor_angles(q_pitch, q_roll)

    # 关节力矩 → 电机力矩
    tau_m1, tau_m2 = ankle.motor_torques_from_task(q_pitch, q_roll, tau_pitch, tau_roll)

    # 关节速度 → 电机速度  (ω = J · q̇)
    vel_m1, vel_m2 = ankle.motor_velocities_from_task(q_pitch, q_roll, qd_pitch, qd_roll)

    # 任务刚度 → 电机刚度矩阵（2x2，含耦合）
    K_motor = ankle.motor_stiffness_from_task(q_pitch, q_roll, k_pitch, k_roll)

    # 任务阻尼 → 电机阻尼矩阵（2x2，含耦合）
    D_motor = ankle.motor_damping_from_task(q_pitch, q_roll, d_pitch, d_roll)

    # ---------- 电机空间 → 关节空间（正运动学 FK）----------
    q_pitch, q_roll = ankle.get_pose_from_motor_angles(theta1, theta2)

    # 电机速度 → 关节速度
    qd_pitch, qd_roll = ankle.task_velocities_from_motor(q_pitch, q_roll, vel_m1, vel_m2)

    # 电机力矩 → 关节力矩
    tau_pitch, tau_roll = ankle.task_torques_from_motor(q_pitch, q_roll, tau_m1, tau_m2)

    # 雅可比矩阵 J = dθ/dq (2x2)
    J = ankle.jacobian(q_pitch, q_roll)
"""

from dataclasses import dataclass
from typing import Tuple

import numpy as np


# ─── 几何参数（v02，与 l1_config.yaml 完全一致）───────────────────────────────


@dataclass
class ActuatorParams:
    """单个执行器的几何参数"""

    r_a: np.ndarray          # 电机轴 A 点在基坐标系的位置 (3,)
    r_c0: np.ndarray         # 足端铰点 C 点的初始位置 (3,)
    l_bar: float             # 法兰杆（曲柄）长度
    l_rod: float             # 连杆长度
    alpha_offset_rad: float  # 曲柄零位偏置角 (rad)
    is_left_motor: bool      # 是否为左电机（用于角度符号修正）

    @property
    def l_bar_sq(self) -> float:
        return self.l_bar * self.l_bar

    @property
    def l_rod_sq(self) -> float:
        return self.l_rod * self.l_rod


def _v02_left_params() -> Tuple[ActuatorParams, ActuatorParams, np.ndarray, np.ndarray]:
    """v02 左脚参数（与 l1_config.yaml l1_v02.ankle_params.left 一致）"""
    p1 = ActuatorParams(
        r_a=np.array([-29.61, 0.0, 261.0]),
        r_c0=np.array([-29.61, 29.544, 0.0]),
        l_bar=30.0,
        l_rod=266.209,
        alpha_offset_rad=-0.1745,
        is_left_motor=True,
    )
    p2 = ActuatorParams(
        r_a=np.array([-29.61, 0.0, 181.0]),
        r_c0=np.array([-29.61, -29.544, 0.0]),
        l_bar=30.0,
        l_rod=186.209,
        alpha_offset_rad=0.1745,
        is_left_motor=False,
    )
    roll_center = np.array([0.0, 0.0, -15.0])
    pitch_center = np.array([0.0, 0.0, -5.0])
    return p1, p2, roll_center, pitch_center


def _v02_right_params() -> Tuple[ActuatorParams, ActuatorParams, np.ndarray, np.ndarray]:
    """v02 右脚参数（与 l1_config.yaml l1_v02.ankle_params.right 一致）"""
    p1 = ActuatorParams(
        r_a=np.array([-29.61, 0.0, 261.0]),
        r_c0=np.array([-29.61, -29.544, 0.0]),
        l_bar=30.0,
        l_rod=266.209,
        alpha_offset_rad=0.1745,
        is_left_motor=False,
    )
    p2 = ActuatorParams(
        r_a=np.array([-29.61, 0.0, 181.0]),
        r_c0=np.array([-29.61, 29.544, 0.0]),
        l_bar=30.0,
        l_rod=186.209,
        alpha_offset_rad=-0.1745,
        is_left_motor=True,
    )
    roll_center = np.array([0.0, 0.0, -15.0])
    pitch_center = np.array([0.0, 0.0, -5.0])
    return p1, p2, roll_center, pitch_center


# ─── 旋转矩阵 ────────────────────────────────────────────────────────────────


def _Rx(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _Ry(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


# ─── PMAnkle 类 ──────────────────────────────────────────────────────────────


class PMAnkle:
    """
    并联踝关节计算类。

    坐标系说明（与 C++ 实现一致）：
      - 基坐标系原点在 roll 旋转中心
      - pitch 旋转中心沿 +Z 方向相对 roll 中心偏移（v02: pitch_center.z - roll_center.z = 10 mm）
      - 运动链：基座 → pitch（绕 pitch_center）→ roll（绕 roll_center）→ 脚
    """

    def __init__(self, is_left: bool):
        if is_left:
            p1, p2, roll_c, pitch_c = _v02_left_params()
        else:
            p1, p2, roll_c, pitch_c = _v02_right_params()
        self.params_1 = p1
        self.params_2 = p2
        self.is_left = is_left
        self.roll_center = roll_c
        self.pitch_center = pitch_c

    # ── 内部：单执行器逆运动学 ──────────────────────────────────────────────

    def _ik_single(self, q_pitch: float, q_roll: float, params: ActuatorParams) -> Tuple[float, bool]:
        """
        解析法计算单个执行器的电机角度。
        返回 (theta_i_rad, success)
        """
        R_pitch = _Ry(q_pitch)
        R_roll = _Rx(q_roll)
        I = np.eye(3)

        # 两级串联旋转（基座 → pitch → roll → 脚）
        # r_ci = R_pitch·R_roll·r_c0
        #      + R_pitch·(I - R_roll)·roll_center
        #      + (I - R_pitch)·pitch_center
        r_ci = (
            R_pitch @ R_roll @ params.r_c0
            + R_pitch @ (I - R_roll) @ self.roll_center
            + (I - R_pitch) @ self.pitch_center
        )

        r_diff = r_ci - params.r_a
        a_val = r_diff[1]                      # y 分量
        b_val = (params.r_a - r_ci)[2]         # (r_a - r_ci).z
        norm_sq = float(r_diff @ r_diff)

        if params.l_bar < 1e-9:
            return 0.0, False

        c_val = (params.l_rod_sq - params.l_bar_sq - norm_sq) / (2.0 * params.l_bar)

        inner = a_val * a_val + b_val * b_val - c_val * c_val
        if inner < -1e-9:
            return 0.0, False
        if inner < 0.0:
            inner = 0.0

        den = a_val * a_val + b_val * b_val
        if abs(den) < 1e-12:
            return 0.0, False

        num = b_val * c_val + abs(a_val) * np.sqrt(inner)
        arg = num / den
        arg = np.clip(arg, -1.0, 1.0)
        theta = float(np.arcsin(arg))

        # 电机方向修正（右电机取反）
        if not params.is_left_motor:
            theta = -theta

        theta += params.alpha_offset_rad
        return theta, True

    # ── 内部：双执行器逆运动学 ──────────────────────────────────────────────

    def _ik_both(self, q_pitch: float, q_roll: float) -> Tuple[float, float, bool, bool]:
        t1, s1 = self._ik_single(q_pitch, q_roll, self.params_1)
        t2, s2 = self._ik_single(q_pitch, q_roll, self.params_2)
        return t1, t2, s1, s2

    # ── 内部：数值雅可比（单执行器一行）────────────────────────────────────

    def _num_jacobian_row(self, q_pitch: float, q_roll: float, params: ActuatorParams,
                          eps: float = 1e-6) -> np.ndarray:
        theta0, _ = self._ik_single(q_pitch, q_roll, params)
        dtheta_dp = (self._ik_single(q_pitch + eps, q_roll, params)[0] - theta0) / eps
        dtheta_dr = (self._ik_single(q_pitch, q_roll + eps, params)[0] - theta0) / eps
        return np.array([dtheta_dp, dtheta_dr])

    # =============================================================
    #  公开接口
    # =============================================================

    # ── 关节 → 电机（逆运动学）──────────────────────────────────────────────

    def get_motor_angles(self, q_pitch: float, q_roll: float) -> Tuple[float, float, bool, bool]:
        """
        逆运动学：给定 (pitch, roll) 角度，返回两个电机的角度。

        参数:
            q_pitch: 俯仰角 (rad)
            q_roll:  横滚角 (rad)

        返回:
            (theta_1, theta_2, success_1, success_2)
        """
        t1, t2, s1, s2 = self._ik_both(q_pitch, q_roll)
        return t1, t2, s1, s2

    # ── 电机 → 关节（正运动学，Newton-Raphson）─────────────────────────────

    def get_pose_from_motor_angles(self, theta1: float, theta2: float,
                                   max_iter: int = 20, eps: float = 1e-6) -> Tuple[float, float]:
        """
        正运动学：给定两个电机角度，返回 (pitch, roll)。

        采用牛顿迭代法求解残差方程 f(q) = IK(q) - θ_desired = 0。
        """
        q = np.zeros(2)  # [q_pitch, q_roll]

        for _ in range(max_iter):
            t1, t2, s1, s2 = self._ik_both(q[0], q[1])
            f = np.array([t1 - theta1, t2 - theta2])
            if np.linalg.norm(f) < eps:
                break

            J = self._jacobian_from_q(q)
            det = np.linalg.det(J)
            if abs(det) < 1e-12:
                break
            q = q - np.linalg.inv(J) @ f

        return float(q[0]), float(q[1])

    # ── 雅可比矩阵 ─────────────────────────────────────────────────────────

    def jacobian(self, q_pitch: float, q_roll: float, eps: float = 1e-6) -> np.ndarray:
        """
        数值雅可比矩阵 J = dθ/dq (2x2)。

        J[0,:] = [dθ1/dq_pitch, dθ1/dq_roll]
        J[1,:] = [dθ2/dq_pitch, dθ2/dq_roll]
        """
        J = np.zeros((2, 2))
        J[0, :] = self._num_jacobian_row(q_pitch, q_roll, self.params_1, eps)
        J[1, :] = self._num_jacobian_row(q_pitch, q_roll, self.params_2, eps)
        return J

    def _jacobian_from_q(self, q: np.ndarray) -> np.ndarray:
        return self.jacobian(q[0], q[1])

    # ── 速度转换 ───────────────────────────────────────────────────────────

    def motor_velocities_from_task(self, q_pitch: float, q_roll: float,
                                   qd_pitch: float, qd_roll: float) -> Tuple[float, float]:
        """
        关节速度 → 电机速度：θ̇ = J · q̇
        """
        J = self.jacobian(q_pitch, q_roll)
        qd = np.array([qd_pitch, qd_roll])
        theta_dot = J @ qd
        return float(theta_dot[0]), float(theta_dot[1])

    def task_velocities_from_motor(self, q_pitch: float, q_roll: float,
                                   vel_m1: float, vel_m2: float) -> Tuple[float, float]:
        """
        电机速度 → 关节速度：q̇ = J⁻¹ · θ̇
        """
        J = self.jacobian(q_pitch, q_roll)
        vel_motor = np.array([vel_m1, vel_m2])
        qd = np.linalg.inv(J) @ vel_motor
        return float(qd[0]), float(qd[1])

    # ── 力矩转换 ───────────────────────────────────────────────────────────

    def motor_torques_from_task(self, q_pitch: float, q_roll: float,
                                tau_pitch: float, tau_roll: float) -> Tuple[float, float]:
        """
        关节力矩 → 电机力矩：τ_motor = (Jᵀ)⁻¹ · τ_task
        """
        J = self.jacobian(q_pitch, q_roll)
        tau_task = np.array([tau_pitch, tau_roll])
        tau_motor = np.linalg.inv(J.T) @ tau_task
        return float(tau_motor[0]), float(tau_motor[1])

    def task_torques_from_motor(self, q_pitch: float, q_roll: float,
                                tau_m1: float, tau_m2: float) -> Tuple[float, float]:
        """
        电机力矩 → 关节力矩：τ_task = Jᵀ · τ_motor
        """
        J = self.jacobian(q_pitch, q_roll)
        tau_motor = np.array([tau_m1, tau_m2])
        tau_task = J.T @ tau_motor
        return float(tau_task[0]), float(tau_task[1])

    # ── 刚度 / 阻尼矩阵转换 ────────────────────────────────────────────────

    def motor_stiffness_from_task(self, q_pitch: float, q_roll: float,
                                  k_pitch: float, k_roll: float) -> np.ndarray:
        """
        任务空间刚度（对角）→ 电机空间刚度矩阵（满阵，含耦合）。

        K_motor = J⁻ᵀ · K_task · J⁻¹
        """
        J = self.jacobian(q_pitch, q_roll)
        J_inv = np.linalg.inv(J)
        K_task = np.diag([k_pitch, k_roll])
        return J_inv.T @ K_task @ J_inv

    def motor_damping_from_task(self, q_pitch: float, q_roll: float,
                                d_pitch: float, d_roll: float) -> np.ndarray:
        """
        任务空间阻尼（对角）→ 电机空间阻尼矩阵（满阵，含耦合）。

        D_motor = J⁻ᵀ · D_task · J⁻¹
        """
        J = self.jacobian(q_pitch, q_roll)
        J_inv = np.linalg.inv(J)
        D_task = np.diag([d_pitch, d_roll])
        return J_inv.T @ D_task @ J_inv

    # ── 一键封装：关节空间 (pitch,roll) 整体 → 电机空间整体 ─────────────────

    def task_to_motor(self, q_pitch: float, q_roll: float,
                      qd_pitch: float, qd_roll: float,
                      tau_pitch: float, tau_roll: float
                      ) -> Tuple[float, float, float, float, float, float]:
        """
        一键转换：关节空间（pitch/roll）→ 电机空间（motor1/motor2）。

        输入:
            q_pitch, q_roll     位置 (rad)
            qd_pitch, qd_roll   速度 (rad/s)
            tau_pitch, tau_roll 力矩 (Nm)

        返回:
            (theta1, theta2, vel1, vel2, tau1, tau2)
        """
        t1, t2, s1, s2 = self.get_motor_angles(q_pitch, q_roll)
        if not (s1 and s2):
            raise ValueError(f"逆运动学求解失败: pitch={q_pitch}, roll={q_roll}")
        v1, v2 = self.motor_velocities_from_task(q_pitch, q_roll, qd_pitch, qd_roll)
        tau1, tau2 = self.motor_torques_from_task(q_pitch, q_roll, tau_pitch, tau_roll)
        return t1, t2, v1, v2, tau1, tau2

    # ── 一键封装：电机空间整体 → 关节空间 (pitch,roll) 整体 ─────────────────

    def motor_to_task(self, theta1: float, theta2: float,
                      vel1: float, vel2: float,
                      tau1: float, tau2: float
                      ) -> Tuple[float, float, float, float, float, float]:
        """
        一键转换：电机空间（motor1/motor2）→ 关节空间（pitch/roll）。

        输入:
            theta1, theta2   电机位置 (rad)
            vel1, vel2       电机速度 (rad/s)
            tau1, tau2       电机力矩 (Nm)

        返回:
            (q_pitch, q_roll, qd_pitch, qd_roll, tau_pitch, tau_roll)
        """
        qp, qr = self.get_pose_from_motor_angles(theta1, theta2)
        qdp, qdr = self.task_velocities_from_motor(qp, qr, vel1, vel2)
        taup, taur = self.task_torques_from_motor(qp, qr, tau1, tau2)
        return qp, qr, qdp, qdr, taup, taur


# ─── 自检测试 ────────────────────────────────────────────────────────────────


def _self_test():
    """快速自检：验证位置/速度/力矩双向转换的互逆一致性。"""
    for is_left in (True, False):
        ankle = PMAnkle(is_left=is_left)

        pitch_position = np.deg2rad(10.0)   # pitch 位置 (rad)
        roll_position  = np.deg2rad(5.0)    # roll  位置 (rad)
        pitch_velocity = 1.0                # pitch 速度 (rad/s)
        roll_velocity  = 0.5                # roll  速度 (rad/s)
        pitch_torque   = 20.0               # pitch 力矩 (Nm)
        roll_torque    = 10.0               # roll  力矩 (Nm)

        # 关节空间 → 电机空间
        motor1_position, motor2_position, motor1_velocity, motor2_velocity, motor1_torque, motor2_torque = ankle.task_to_motor(
            pitch_position, roll_position,
            pitch_velocity, roll_velocity,
            pitch_torque,   roll_torque,
        )

        # 电机空间 → 关节空间
        p_pos, r_pos, p_vel, r_vel, p_tau, r_tau = ankle.motor_to_task(
            motor1_position, motor2_position,
            motor1_velocity, motor2_velocity,
            motor1_torque,   motor2_torque)

        assert abs(p_pos - pitch_position) < 1e-8,  "位置互逆失败 (pitch)"
        assert abs(r_pos - roll_position)  < 1e-8,  "位置互逆失败 (roll)"
        assert abs(p_vel - pitch_velocity) < 1e-6,  "速度互逆失败 (pitch)"
        assert abs(r_vel - roll_velocity)  < 1e-6,  "速度互逆失败 (roll)"
        assert abs(p_tau - pitch_torque)   < 1e-5,  "力矩互逆失败 (pitch)"
        assert abs(r_tau - roll_torque)    < 1e-5,  "力矩互逆失败 (roll)"

        # 刚度矩阵对称（物理上必须满足）
        K = ankle.motor_stiffness_from_task(pitch_position, roll_position, 1000.0, 500.0)
        assert np.allclose(K, K.T), "刚度矩阵不对称"

    print("PMAnkle 自检通过 (v02)")


if __name__ == "__main__":
    _self_test()
