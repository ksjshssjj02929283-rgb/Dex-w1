#!/usr/bin/env python3
"""
计算并联踝关节任务空间的额定/最大速度与力矩。

输入：电机规格（额定/最大速度、额定/最大力矩）
输出：任务空间（pitch, roll）对应的额定/最大速度与力矩，
      以及最差构型的位置。

原理：
  - 电机 → 任务 速度: q̇ = J⁻¹ · ω
  - 电机 → 任务 力矩: τ_task = Jᵀ · τ_motor
  遍历工作空间网格，取保守（最差）值。

用法：
  python calc_ankle_task_limits.py
  python calc_ankle_task_limits.py --rated-vel 14.66 --max-vel 26.18 --rated-tau 16 --max-tau 96
"""

import argparse
import sys
from dataclasses import dataclass

import numpy as np


# ─── 几何参数（与 C++ parallel_ankle_calc.cpp 完全一致） ──────────────────────


@dataclass
class ActuatorParams:
    is_first: bool
    r_a: np.ndarray  # 电机轴位置 (3,)
    r_c0: np.ndarray  # 平台铰点初始位置 (3,)
    l_bar: float  # 曲柄长度
    l_rod: float  # 连杆长度
    alpha: float  # 曲柄零位偏置角 (rad)

    @property
    def l_bar_sq(self) -> float:
        return self.l_bar * self.l_bar

    @property
    def l_rod_sq(self) -> float:
        return self.l_rod * self.l_rod


def make_params(is_left: bool):
    """构造左右脚的执行器参数"""
    if is_left:
        p1 = ActuatorParams(
            is_first=True,
            r_a=np.array([-46.0, 0.0, 257.94]),
            r_c0=np.array([-46.0, -59.0, 0.0]),
            l_bar=60.0,
            l_rod=268.354,
            alpha=0.1745,
        )
        p2 = ActuatorParams(
            is_first=False,
            r_a=np.array([-46.0, 0.0, 177.94]),
            r_c0=np.array([-46.0, 59.0, 0.0]),
            l_bar=60.0,
            l_rod=188.355,
            alpha=-0.1745,
        )
    else:
        p1 = ActuatorParams(
            is_first=True,
            r_a=np.array([-46.0, 0.0, 257.94]),
            r_c0=np.array([-46.0, 59.0, 0.0]),
            l_bar=60.0,
            l_rod=268.354,
            alpha=-0.1745,
        )
        p2 = ActuatorParams(
            is_first=False,
            r_a=np.array([-46.0, 0.0, 177.94]),
            r_c0=np.array([-46.0, -59.0, 0.0]),
            l_bar=60.0,
            l_rod=188.355,
            alpha=0.1745,
        )
    return p1, p2


# ─── 旋转矩阵 ────────────────────────────────────────────────────────────────


def Rx(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Ry(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


# ─── IK：单执行器逆运动学（解析解） ────────────────────────────────────────────


def ik_single(q_pitch: float, q_roll: float, params: ActuatorParams, is_left: bool) -> tuple[float, bool]:
    """返回 (theta_i_rad, success)"""
    x_rot = Ry(q_pitch) @ Rx(q_roll)
    r_ci = x_rot @ params.r_c0
    r_diff = r_ci - params.r_a

    a_val = r_diff[1]  # y
    b_val = -r_diff[2]  # -z  (对应 C++ 中 (r_ai - r_ci).z())
    norm_sq = float(r_diff @ r_diff)

    if params.l_bar < 1e-9:
        return 0.0, False

    c_val = (params.l_rod_sq - params.l_bar_sq - norm_sq) / (2.0 * params.l_bar)

    inner = a_val * a_val + b_val * b_val - c_val * c_val
    if inner < -1e-9:
        return 0.0, False
    if inner < 0:
        inner = 0.0

    den = a_val * a_val + b_val * b_val
    if abs(den) < 1e-12:
        return 0.0, False

    num = b_val * c_val + abs(a_val) * np.sqrt(inner)
    arg = num / den
    arg = np.clip(arg, -1.0, 1.0)
    theta = float(np.arcsin(arg))

    # 符号翻转（与 C++ 一致）
    if is_left == params.is_first:
        theta = -theta

    theta += params.alpha
    return theta, True


# ─── IK：双执行器 ────────────────────────────────────────────────────────────


def ik_both(
    q_pitch: float, q_roll: float, p1: ActuatorParams, p2: ActuatorParams, is_left: bool
) -> tuple[float, float, bool, bool]:
    t1, s1 = ik_single(q_pitch, q_roll, p1, is_left)
    t2, s2 = ik_single(q_pitch, q_roll, p2, is_left)
    return t1, t2, s1, s2


# ─── Jacobian（数值微分） ─────────────────────────────────────────────────────


def compute_jacobian(
    q_pitch: float, q_roll: float, p1: ActuatorParams, p2: ActuatorParams, is_left: bool, eps: float = 1e-6
) -> np.ndarray:
    t10, t20, _, _ = ik_both(q_pitch, q_roll, p1, p2, is_left)

    t1p, _, s1p, _ = ik_both(q_pitch + eps, q_roll, p1, p2, is_left)
    t1r, _, s1r, _ = ik_both(q_pitch, q_roll + eps, p1, p2, is_left)

    _, t2p, _, s2p = ik_both(q_pitch + eps, q_roll, p1, p2, is_left)
    _, t2r, _, s2r = ik_both(q_pitch, q_roll + eps, p1, p2, is_left)

    if not all([s1p, s1r, s2p, s2r]):
        return np.full((2, 2), np.nan)

    J = np.array(
        [
            [(t1p - t10) / eps, (t1r - t10) / eps],
            [(t2p - t20) / eps, (t2r - t20) / eps],
        ]
    )
    return J


# ─── 任务空间极限计算 ─────────────────────────────────────────────────────────


def task_velocity_bounds(J: np.ndarray, motor_vel_max: float) -> tuple[float, float]:
    """给定 Jacobian 和电机最大速度，返回 |q̇_pitch| 和 |q̇_roll| 的上界"""
    J_inv = np.linalg.inv(J)
    # q̇ = J⁻¹ · ω,  |ω_i| ≤ motor_vel_max
    # max |q̇_pitch| = (|J⁻¹(0,0)| + |J⁻¹(0,1)|) * motor_vel_max
    pitch_bound = (abs(J_inv[0, 0]) + abs(J_inv[0, 1])) * motor_vel_max
    roll_bound = (abs(J_inv[1, 0]) + abs(J_inv[1, 1])) * motor_vel_max
    return pitch_bound, roll_bound


def task_torque_bounds(J: np.ndarray, motor_tau_max: float) -> tuple[float, float]:
    """给定 Jacobian 和电机最大力矩，返回 |τ_pitch| 和 |τ_roll| 的上界"""
    # τ_task = Jᵀ · τ_motor,  |τ_motor_i| ≤ motor_tau_max
    # max |τ_pitch| = (|J(0,0)| + |J(1,0)|) * motor_tau_max
    pitch_bound = (abs(J[0, 0]) + abs(J[1, 0])) * motor_tau_max
    roll_bound = (abs(J[0, 1]) + abs(J[1, 1])) * motor_tau_max
    return pitch_bound, roll_bound


def task_inertia(J: np.ndarray, motor_inertia: float) -> np.ndarray:
    """将电机转子惯量映射为任务空间等效惯量矩阵.

    I_task = Jᵀ · diag(I_m, I_m) · J = I_m · Jᵀ · J
    返回 2×2 矩阵: [[I_pp, I_pr], [I_rp, I_rr]]
    """
    return motor_inertia * (J.T @ J)


# ─── 工作空间扫描 ─────────────────────────────────────────────────────────────


def sweep_workspace(
    p1,
    p2,
    is_left: bool,
    pitch_range=(-1.05, 0.61),
    roll_range=(-0.26, 0.26),
    n_pitch=41,
    n_roll=25,
    motor_vel_max=1.0,
    motor_tau_max=1.0,
    motor_inertia: float | None = None,
):
    """扫描工作空间，返回每个自由度的速度/力矩/惯量上界（保守值）以及最差构型"""
    pitches = np.linspace(*pitch_range, n_pitch)
    rolls = np.linspace(*roll_range, n_roll)

    vel_pitch_min = float("inf")
    vel_roll_min = float("inf")
    tau_pitch_min = float("inf")
    tau_roll_min = float("inf")
    inert_pitch_min = float("inf")
    inert_roll_min = float("inf")
    inert_pitch_max = float("-inf")
    inert_roll_max = float("-inf")
    inert_nominal = None  # 中性位姿处的惯量矩阵

    worst = {}

    for qp in pitches:
        for qr in rolls:
            J = compute_jacobian(qp, qr, p1, p2, is_left)
            if np.any(np.isnan(J)):
                continue

            vp, vr = task_velocity_bounds(J, motor_vel_max)
            tp, tr = task_torque_bounds(J, motor_tau_max)

            if vp < vel_pitch_min:
                vel_pitch_min = vp
                worst["vel_pitch"] = (qp, qr)
            if vr < vel_roll_min:
                vel_roll_min = vr
                worst["vel_roll"] = (qp, qr)
            if tp < tau_pitch_min:
                tau_pitch_min = tp
                worst["tau_pitch"] = (qp, qr)
            if tr < tau_roll_min:
                tau_roll_min = tr
                worst["tau_roll"] = (qp, qr)

            if motor_inertia is not None:
                I_task = task_inertia(J, motor_inertia)
                ipp, irr = float(I_task[0, 0]), float(I_task[1, 1])
                if ipp < inert_pitch_min:
                    inert_pitch_min = ipp
                    worst["inert_pitch_min"] = (qp, qr)
                if ipp > inert_pitch_max:
                    inert_pitch_max = ipp
                    worst["inert_pitch_max"] = (qp, qr)
                if irr < inert_roll_min:
                    inert_roll_min = irr
                    worst["inert_roll_min"] = (qp, qr)
                if irr > inert_roll_max:
                    inert_roll_max = irr
                    worst["inert_roll_max"] = (qp, qr)
                if abs(qp) < 1e-9 and abs(qr) < 1e-9:
                    inert_nominal = I_task

    result = {
        "vel_pitch": vel_pitch_min,
        "vel_roll": vel_roll_min,
        "tau_pitch": tau_pitch_min,
        "tau_roll": tau_roll_min,
        "worst_config": worst,
    }

    if motor_inertia is not None:
        result["inertia"] = {
            "pitch_min": inert_pitch_min,
            "pitch_max": inert_pitch_max,
            "roll_min": inert_roll_min,
            "roll_max": inert_roll_max,
            "nominal": inert_nominal,
        }

    return result


# ─── 格式化输出 ───────────────────────────────────────────────────────────────


def fmt_rad_per_s(v: float) -> str:
    return f"{v:.3f} rad/s  ({np.rad2deg(v):.2f} °/s)"


def fmt_nm(v: float) -> str:
    return f"{v:.2f} Nm"


def print_results(results_left, results_right, motor_specs, pitch_range, roll_range):
    print("=" * 72)
    print("  并联踝关节 任务空间极限参数计算")
    print("=" * 72)
    print()
    print("  电机参数:")
    print(f"    额定速度:  {motor_specs['rated_vel']:.2f} rad/s  ({np.rad2deg(motor_specs['rated_vel']):.1f} °/s)")
    print(f"    最大速度:  {motor_specs['max_vel']:.2f} rad/s  ({np.rad2deg(motor_specs['max_vel']):.1f} °/s)")
    print(f"    额定力矩:  {motor_specs['rated_tau']:.2f} Nm")
    print(f"    最大力矩:  {motor_specs['max_tau']:.2f} Nm")
    print()
    print(
        f"  工作空间: pitch ∈ [{pitch_range[0]}, {pitch_range[1]}] rad"
        f" ({np.rad2deg(pitch_range[0]):.0f}° ~ {np.rad2deg(pitch_range[1]):.0f}°)"
    )
    print(
        f"            roll  ∈ [{roll_range[0]}, {roll_range[1]}] rad"
        f" ({np.rad2deg(roll_range[0]):.0f}° ~ {np.rad2deg(roll_range[1]):.0f}°)"
    )
    print()
    print("-" * 72)
    print("  任务空间额定值（取工作空间内保守值）")
    print("-" * 72)
    print(f"  {'':<20} {'左脚':>25}")
    print(f"  {'pitch 额定速度:':<20} {fmt_rad_per_s(results_left['rated']['vel_pitch']):>25}")
    print(f"  {'roll  额定速度:':<20} {fmt_rad_per_s(results_left['rated']['vel_roll']):>25}")
    print(f"  {'pitch 额定力矩:':<20} {fmt_nm(results_left['rated']['tau_pitch']):>25}")
    print(f"  {'roll  额定力矩:':<20} {fmt_nm(results_left['rated']['tau_roll']):>25}")
    print()
    print("-" * 72)
    print("  任务空间最大值（取工作空间内保守值）")
    print("-" * 72)
    print(f"  {'':<20} {'左脚':>25} {'右脚':>25}")
    print(
        f"  {'pitch 最大速度:':<20} {fmt_rad_per_s(results_left['max']['vel_pitch']):>25} {fmt_rad_per_s(results_right['max']['vel_pitch']):>25}"
    )
    print(
        f"  {'roll  最大速度:':<20} {fmt_rad_per_s(results_left['max']['vel_roll']):>25} {fmt_rad_per_s(results_right['max']['vel_roll']):>25}"
    )
    print(
        f"  {'pitch 最大力矩:':<20} {fmt_nm(results_left['max']['tau_pitch']):>25} {fmt_nm(results_right['max']['tau_pitch']):>25}"
    )
    print(
        f"  {'roll  最大力矩:':<20} {fmt_nm(results_left['max']['tau_roll']):>25} {fmt_nm(results_right['max']['tau_roll']):>25}"
    )
    print()
    print("-" * 72)
    print("  最差构型（保守值出现的位置）")
    print("-" * 72)
    for key, label in [
        ("vel_pitch", "pitch速度"),
        ("vel_roll", "roll速度"),
        ("tau_pitch", "pitch力矩"),
        ("tau_roll", "roll力矩"),
    ]:
        qp, qr = results_left["rated"]["worst_config"][key]
        print(f"  {label}最差点: pitch={qp:.3f} rad ({np.rad2deg(qp):.1f}°), roll={qr:.3f} rad ({np.rad2deg(qr):.1f}°)")
    print()
    print("-" * 72)
    print("  参考：电机能力与任务空间映射关系")
    print("-" * 72)
    vl, vr = results_left["rated"]["vel_pitch"], results_left["rated"]["vel_roll"]
    tl, tr = results_left["rated"]["tau_pitch"], results_left["rated"]["tau_roll"]
    print(f"  速度: 额定 {motor_specs['rated_vel']:.2f} → pitch {vl:.2f} / roll {vr:.2f} rad/s")
    print(
        f"        最大 {motor_specs['max_vel']:.2f} → pitch {results_left['max']['vel_pitch']:.2f} / roll {results_left['max']['vel_roll']:.2f} rad/s"
    )
    print(f"  力矩: 额定 {motor_specs['rated_tau']:.2f} → pitch {tl:.2f} / roll {tr:.2f} Nm")
    print(
        f"        最大 {motor_specs['max_tau']:.2f} → pitch {results_left['max']['tau_pitch']:.2f} / roll {results_left['max']['tau_roll']:.2f} Nm"
    )

    if "inertia" in results_left["rated"]:
        print()
        print("-" * 72)
        print("  任务空间等效转子惯量")
        print("-" * 72)
        I_m = motor_specs["motor_inertia"]
        i_rated = results_left["rated"]["inertia"]

        # 显式计算中性位姿的惯量矩阵（不依赖网格采样）
        p1_left, p2_left = make_params(is_left=True)
        J0 = compute_jacobian(0.0, 0.0, p1_left, p2_left, is_left=True)
        I0 = task_inertia(J0, I_m)

        print("  电机转子惯量:  {:.6f}".format(I_m))
        print()
        print("  中性位姿 (qp=0, qr=0):")
        print("    I_pitch = {:.6f}".format(float(I0[0, 0])))
        print("    I_roll  = {:.6f}".format(float(I0[1, 1])))
        print("    耦合项   = {:.6f}".format(float(I0[0, 1])))
        print()
        print("  工作空间内变化范围:")
        print("    I_pitch: {:.6f} ~ {:.6f}".format(i_rated["pitch_min"], i_rated["pitch_max"]))
        print("    I_roll:  {:.6f} ~ {:.6f}".format(i_rated["roll_min"], i_rated["roll_max"]))
        print("  (惯量单位与电机惯量单位一致)")
    print("=" * 72)


# ─── main ─────────────────────────────────────────────────────────────────────


def main():
    parser = argparse.ArgumentParser(
        description="计算并联踝关节任务空间速度/力矩极限",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--rated-vel", type=float, default=14.66, help="电机额定速度 (rad/s), 默认 14.66")
    parser.add_argument("--max-vel", type=float, default=26.18, help="电机最大速度 (rad/s), 默认 26.18")
    parser.add_argument("--rated-tau", type=float, default=16.0, help="电机额定力矩 (Nm), 默认 16.0")
    parser.add_argument("--max-tau", type=float, default=96.0, help="电机最大力矩 (Nm), 默认 96.0")
    parser.add_argument("--pitch-min", type=float, default=-1.05, help="pitch 下限 (rad), 默认 -1.05")
    parser.add_argument("--pitch-max", type=float, default=0.61, help="pitch 上限 (rad), 默认 0.61")
    parser.add_argument("--roll-min", type=float, default=-0.26, help="roll 下限 (rad), 默认 -0.26")
    parser.add_argument("--roll-max", type=float, default=0.26, help="roll 上限 (rad), 默认 0.26")
    parser.add_argument("--motor-inertia", type=float, default=0.0132, help="电机转子惯量, 默认 0.0132")
    parser.add_argument("--only-left", action="store_true", help="只计算左脚（左右对称，结果相同）")
    args = parser.parse_args()

    motor_specs = {
        "rated_vel": args.rated_vel,
        "max_vel": args.max_vel,
        "rated_tau": args.rated_tau,
        "max_tau": args.max_tau,
        "motor_inertia": args.motor_inertia,
    }

    pitch_range = (args.pitch_min, args.pitch_max)
    roll_range = (args.roll_min, args.roll_max)

    results = {}
    for side, is_left in [("left", True), ("right", False)]:
        if args.only_left and side == "right":
            continue
        p1, p2 = make_params(is_left)

        res_rated = sweep_workspace(
            p1,
            p2,
            is_left,
            pitch_range=pitch_range,
            roll_range=roll_range,
            motor_vel_max=motor_specs["rated_vel"],
            motor_tau_max=motor_specs["rated_tau"],
            motor_inertia=motor_specs["motor_inertia"],
        )
        res_max = sweep_workspace(
            p1,
            p2,
            is_left,
            pitch_range=pitch_range,
            roll_range=roll_range,
            motor_vel_max=motor_specs["max_vel"],
            motor_tau_max=motor_specs["max_tau"],
            motor_inertia=motor_specs["motor_inertia"],
        )
        results[side] = {"rated": res_rated, "max": res_max}

    # 如果只算了左脚，复制到右脚（对称）
    if args.only_left:
        results["right"] = results["left"]

    print_results(results["left"], results["right"], motor_specs, pitch_range, roll_range)


if __name__ == "__main__":
    main()
