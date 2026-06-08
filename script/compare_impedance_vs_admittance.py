#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
实时录制 + 自动对比: 左臂(传感器+导纳) vs 右臂(纯阻抗)

原理:
  - 左臂 joints 6-12:  有力传感器 → tau_ext 非零 → 导纳环工作 → 阻抗+导纳
  - 右臂 joints 13-19: 无传感器 → tau_ext = 0 → 纯阻抗
  - 同一时刻、同一外力、一次录制

使用方法:
    source /opt/ros/humble/setup.bash
    python3 compare_impedance_vs_admittance.py

运行后:
    1. 脚本开始录制 /feedback/robot_server_state
    2. 手动推左右臂，保持几秒，松手，等待 3-5 秒
    3. 输入 s 并回车 → 停止录制
    4. 自动输出统计结果 + 保存可视化图片

可选参数:
    --left  J1,J2...   左臂关节索引 (默认 6-12)
    --right J1,J2...   右臂关节索引 (默认 13-19)
    --settle SEC       稳态评估等待秒数 (默认 2.0s)
    --threshold RAD    恢复阈值 rad (默认 0.003 ≈ 0.17°)
    --topic TOPIC      ROS topic (默认 /feedback/robot_server_state)
    --save-dir DIR     图片保存目录 (默认脚本所在目录)
"""

import argparse
import json
import sys
import threading
import time
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")  # 不弹窗，直接保存
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ──────────────────────── 录制节点 ────────────────────────

# 分析所需的字段
ANALYSIS_KEYS = [
    "joint_position",
    "joint_position_cmd",
    "torque_ext",
    "joint_admittance_q_offset",
    "joint_position_ref_admittance",
    "joint_impedance_error_after_admittance",
    "force_sensor",
]

# JSONL 要保存的字段 (与 robot_record.jsonl 一致)
LOG_KEYS = [
    "timestamp",
    "joint_position",
    "joint_velocity",
    "joint_torque",
    "joint_position_input",
    "joint_position_after_ruckig",
    "joint_position_after_filter",
    "joint_position_cmd",
    "joint_torque_feedforward",
    "torque_ext",
    "torque_ext_raw",
    "joint_admittance_q_offset",
    "joint_admittance_dq_offset",
    "joint_position_ref_admittance",
    "joint_impedance_error_after_admittance",
    "force_sensor",
]


class Recorder(Node):
    def __init__(self, topic: str):
        super().__init__("impedance_vs_admittance_recorder")
        self._lock = threading.Lock()
        self._t0 = time.monotonic()
        self._timestamps = []  # 每个采样到达时的相对时间
        self._raw_samples = []  # 完整 JSON dict 列表
        self.count = 0

        self.create_subscription(String, topic, self._callback, 50)
        self.get_logger().info(f"Subscribed to {topic}")

    def _callback(self, msg: String):
        try:
            d = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return

        if not all(k in d for k in ANALYSIS_KEYS):
            return

        t = time.monotonic() - self._t0
        with self._lock:
            self._timestamps.append(t)
            self._raw_samples.append(d)
            self.count += 1

    def snapshot_raw(self):
        """返回完整 JSON dict 列表 (用于保存 JSONL)"""
        with self._lock:
            return list(self._raw_samples)

    def snapshot_analysis(self):
        """返回 (timestamps_ndarray, data_dict_of_ndarrays) 用于分析"""
        with self._lock:
            ts = np.array(self._timestamps)
            samples = self._raw_samples
        data = {}
        for k in ANALYSIS_KEYS:
            data[k] = np.array([s[k] for s in samples])
        return ts, data

    def save_jsonl(self, filepath):
        """保存为与 robot_record.jsonl 同格式的文件"""
        with self._lock:
            samples = self._raw_samples
        with open(filepath, "w") as f:
            for s in samples:
                row = {}
                for k in LOG_KEYS:
                    if k in s:
                        row[k] = s[k]
                f.write(json.dumps(row) + "\n")


# ──────────────────────── 分析逻辑 ────────────────────────

def detect_force_events(torque_ext, joint_indices, threshold_ratio=0.12):
    """用指定关节 |tau_ext| 之和检测外力事件"""
    mag = np.sum(np.abs(torque_ext[:, joint_indices]), axis=1)
    threshold = np.max(mag) * threshold_ratio
    if threshold < 0.3:
        threshold = 0.3

    in_force = mag > threshold
    events = []
    started, start = False, 0
    for i in range(len(in_force)):
        if in_force[i] and not started:
            started, start = True, i
        elif not in_force[i] and started:
            events.append((start, i))
            started = False
    if started:
        events.append((start, len(in_force) - 1))

    return events, mag, threshold


def compute_metrics_per_joint(timestamps, joint_position, joint_position_cmd,
                              force_event, settle_sec=2.0, threshold=0.003):
    """
    joint_position:     单个关节的原始位置数组 (rad)
    joint_position_cmd: 单个关节的指令位置数组 (rad)
    所有误差均对 cmd 计算, 不用实测初值作参考.
    """
    dt = np.median(np.diff(timestamps)) if len(timestamps) > 1 else 0.001
    fs, fe = force_event
    n = len(joint_position)

    error = joint_position - joint_position_cmd

    half_sec = max(1, int(0.5 / dt))

    baseline_error = np.mean(error[:half_sec])
    peak_error = np.max(np.abs(error[fs:fe + 1]))

    final_error = np.mean(error[-half_sec:])
    final_error_abs = np.mean(np.abs(error[-half_sec:]))
    baseline_error_abs = np.mean(np.abs(error[:half_sec:]))

    net_recovery_error = abs(final_error - baseline_error)

    post_error = error[fe + 1:]
    recovered = np.where(np.abs(post_error - baseline_error) < threshold)[0]
    recovery_time = recovered[0] * dt if len(recovered) > 0 else float("inf")

    return {
        "peak_error": peak_error,
        "baseline_error": baseline_error,
        "baseline_error_abs": baseline_error_abs,
        "final_error": final_error,
        "final_error_abs": final_error_abs,
        "net_recovery_error": net_recovery_error,
        "recovery_time": recovery_time,
    }


# ──────────────────────── 绘图 ────────────────────────

def save_plots(timestamps, data, left_joints, right_joints,
               force_events, force_mag, threshold, args, save_path):
    t_rel = timestamps
    n_pairs = min(len(left_joints), len(right_joints))

    fig, axes = plt.subplots(n_pairs + 1, 3, figsize=(18, 3.5 * (n_pairs + 1)))

    for row in range(n_pairs):
        jl, jr = left_joints[row], right_joints[row]

        # Col 0: 位置误差
        ax = axes[row, 0]
        err_l = np.degrees(data["joint_position"][:, jl] - data["joint_position_cmd"][:, jl])
        err_r = np.degrees(data["joint_position"][:, jr] - data["joint_position_cmd"][:, jr])
        ax.plot(t_rel, err_l, color="#2ecc71", linewidth=0.8, label=f"Left J{jl} (adm)")
        ax.plot(t_rel, err_r, color="#e74c3c", linewidth=0.8, label=f"Right J{jr} (imp)")
        for fs, fe in force_events:
            ax.axvspan(t_rel[fs], t_rel[min(fe, len(t_rel) - 1)], alpha=0.1, color="orange")
        ax.set_ylabel("Error (deg)", fontsize=9)
        ax.legend(fontsize=7, loc="upper right")
        ax.grid(True, alpha=0.3)
        ax.set_title(f"J{jl} vs J{jr} — Position Error", fontsize=10)

        # Col 1: tau_ext
        ax = axes[row, 1]
        ax.plot(t_rel, data["torque_ext"][:, jl], color="#2ecc71", linewidth=0.8, label=f"Left J{jl}")
        ax.plot(t_rel, data["torque_ext"][:, jr], color="#e74c3c", linewidth=0.8, label=f"Right J{jr}")
        for fs, fe in force_events:
            ax.axvspan(t_rel[fs], t_rel[min(fe, len(t_rel) - 1)], alpha=0.1, color="orange")
        ax.set_ylabel("tau_ext (Nm)", fontsize=9)
        ax.legend(fontsize=7, loc="upper right")
        ax.grid(True, alpha=0.3)
        ax.set_title(f"J{jl} vs J{jr} — External Torque", fontsize=10)

        # Col 2: 导纳偏移
        ax = axes[row, 2]
        off_l = np.degrees(data["joint_admittance_q_offset"][:, jl])
        off_r = np.degrees(data["joint_admittance_q_offset"][:, jr])
        ax.plot(t_rel, off_l, color="#2ecc71", linewidth=0.8, label=f"Left J{jl}")
        ax.plot(t_rel, off_r, color="#e74c3c", linewidth=0.8, label=f"Right J{jr}")
        for fs, fe in force_events:
            ax.axvspan(t_rel[fs], t_rel[min(fe, len(t_rel) - 1)], alpha=0.1, color="orange")
        ax.set_ylabel("Offset (deg)", fontsize=9)
        ax.legend(fontsize=7, loc="upper right")
        ax.grid(True, alpha=0.3)
        ax.set_title(f"J{jl} vs J{jr} — Admittance q_offset", fontsize=10)

    # 底部: 力强度 + 汇总
    ax_force = axes[n_pairs, 0]
    mag_l = np.sum(np.abs(data["torque_ext"][:, left_joints]), axis=1)
    mag_r = np.sum(np.abs(data["torque_ext"][:, right_joints]), axis=1)
    ax_force.plot(t_rel, mag_l, color="#2ecc71", linewidth=0.8, label="Left arm |tau_ext|")
    ax_force.plot(t_rel, mag_r, color="#e74c3c", linewidth=0.8, label="Right arm |tau_ext|")
    ax_force.axhline(threshold, color="orange", linestyle="--", linewidth=0.8, alpha=0.6)
    for fs, fe in force_events:
        ax_force.axvspan(t_rel[fs], t_rel[min(fe, len(t_rel) - 1)], alpha=0.15, color="orange")
    ax_force.set_xlabel("Time (s)")
    ax_force.set_ylabel("|tau_ext| sum (Nm)")
    ax_force.set_title("Force Magnitude", fontsize=10)
    ax_force.legend(fontsize=7)
    ax_force.grid(True, alpha=0.3)

    # 汇总表
    ax_table = axes[n_pairs, 1]
    ax_table.axis("off")
    ax_table.set_title("Peak Error (deg)", fontsize=10)

    header = f"{'Pair':<12}{'Left(adm)':<14}{'Right(imp)':<14}{'Improve':<10}\n" + "─" * 50 + "\n"
    rows_text = header
    for row in range(n_pairs):
        jl, jr = left_joints[row], right_joints[row]
        peak_l = np.degrees(np.max(np.abs(data["joint_position"][:, jl] - data["joint_position_cmd"][:, jl])))
        peak_r = np.degrees(np.max(np.abs(data["joint_position"][:, jr] - data["joint_position_cmd"][:, jr])))
        imp = f"{(1 - peak_l / peak_r) * 100:+.1f}%" if peak_r > 1e-6 else "N/A"
        rows_text += f"J{jl}vsJ{jr:<6}{peak_l:>8.4f}°    {peak_r:>8.4f}°    {imp}\n"

    ax_table.text(0.05, 0.95, rows_text, transform=ax_table.transAxes,
                  fontsize=9, va="top", fontfamily="monospace",
                  bbox=dict(boxstyle="round", facecolor="lightyellow", alpha=0.8))

    # 恢复指标表
    ax_det = axes[n_pairs, 2]
    ax_det.axis("off")
    ax_det.set_title("Recovery Metrics", fontsize=10)

    det = f"{'Joint':<8}{'Base(°)':<10}{'Final(°)':<10}{'NetRec(°)':<10}{'RecT(s)':<10}{'Mode':<6}\n" + "─" * 54 + "\n"
    if force_events:
        fe = force_events[0]
        for row in range(n_pairs):
            for side, joints, label in [("L", left_joints, "adm"), ("R", right_joints, "imp")]:
                j = joints[row]
                pos = data["joint_position"][:, j]
                cmd = data["joint_position_cmd"][:, j]
                m = compute_metrics_per_joint(timestamps, pos, cmd, fe, args.settle, args.threshold)
                bl_str = f"{np.degrees(m['baseline_error']):+.3f}"
                fi_str = f"{np.degrees(m['final_error']):+.3f}"
                nr_str = f"{np.degrees(m['net_recovery_error']):.4f}"
                rt_str = f"{m['recovery_time']:.3f}" if m["recovery_time"] < 100 else ">100"
                det += f"J{j:<6}{bl_str:>9} {fi_str:>9} {nr_str:>9} {rt_str:>8}s  {label}\n"
            det += "─" * 54 + "\n"

    ax_det.text(0.05, 0.95, det, transform=ax_det.transAxes,
                fontsize=8, va="top", fontfamily="monospace",
                bbox=dict(boxstyle="round", facecolor="lightyellow", alpha=0.8))

    fig.suptitle(
        "Left Arm (Impedance+Admittance) vs Right Arm (Impedance Only)\n"
        "Orange = detected external force",
        fontsize=12, fontweight="bold",
    )
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    fig.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


# ──────────────────────── 终端输出 ────────────────────────

def print_statistics(timestamps, data, left_joints, right_joints,
                     force_events, threshold, args):
    n_pairs = min(len(left_joints), len(right_joints))
    duration = timestamps[-1] - timestamps[0]
    dt_ms = (timestamps[-1] - timestamps[0]) / len(timestamps) * 1000

    print(f"\n{'═' * 72}")
    print(f"  录制: {len(timestamps)} samples, {duration:.2f}s, dt≈{dt_ms:.1f}ms ({1000/dt_ms:.0f} Hz)")
    print(f"  外力阈值: {threshold:.2f} Nm (left arm |tau_ext| sum)")
    print(f"  检测到 {len(force_events)} 次外力事件")
    for i, (fs, fe) in enumerate(force_events):
        print(f"    Event {i+1}: {timestamps[fs]:.2f}s ~ {timestamps[fe]:.2f}s "
              f"(持续 {timestamps[fe]-timestamps[fs]:.2f}s)")
    print(f"{'═' * 72}")

    if not force_events:
        print("[WARN] 未检测到外力事件，以下指标可能无意义")

    fe = force_events[0] if force_events else (0, len(timestamps) - 1)

    print(f"\n{'Joint':<7}{'Side':<5}{'Baseline':>10}{'Final':>10}{'NetRecov':>10}{'Peak':>10}{'RecovT':>10}{'tau_ext':>9}")
    print(f"{'':12}{'(deg)':>10}{'(deg)':>10}{'(deg)':>10}{'(deg)':>10}{'(s)':>10}{'(Nm)':>9}")
    print("─" * 78)

    improvements = {"peak": [], "final": [], "net_recovery": [], "rt": []}

    for row in range(n_pairs):
        jl, jr = left_joints[row], right_joints[row]

        for j, label in [(jl, "adm"), (jr, "imp")]:
            pos = data["joint_position"][:, j]
            cmd = data["joint_position_cmd"][:, j]
            m = compute_metrics_per_joint(timestamps, pos, cmd, fe, args.settle, args.threshold)
            bl_deg = np.degrees(m["baseline_error"])
            fi_deg = np.degrees(m["final_error"])
            nr_deg = np.degrees(m["net_recovery_error"])
            pk_deg = np.degrees(m["peak_error"])
            rt = m["recovery_time"]
            tau = np.mean(np.abs(data["torque_ext"][fe[0]:fe[1]+1, j]))

            rt_str = f"{rt:.3f}" if rt < 100 else ">100"
            print(f"J{j:<5}{label:<5}{bl_deg:>+10.4f}{fi_deg:>+10.4f}{nr_deg:>10.4f}{pk_deg:>10.4f}{rt_str:>10s}{tau:>9.2f}")

        pos_l = data["joint_position"][:, jl]
        pos_r = data["joint_position"][:, jr]
        cmd_l = data["joint_position_cmd"][:, jl]
        cmd_r = data["joint_position_cmd"][:, jr]
        ml = compute_metrics_per_joint(timestamps, pos_l, cmd_l, fe, args.settle, args.threshold)
        mr = compute_metrics_per_joint(timestamps, pos_r, cmd_r, fe, args.settle, args.threshold)

        if mr["peak_error"] > 1e-6:
            improvements["peak"].append((1 - ml["peak_error"] / mr["peak_error"]) * 100)
        if mr["final_error_abs"] > 1e-6:
            improvements["final"].append((1 - ml["final_error_abs"] / mr["final_error_abs"]) * 100)
        if mr["net_recovery_error"] > 1e-6:
            improvements["net_recovery"].append((1 - ml["net_recovery_error"] / mr["net_recovery_error"]) * 100)
        if mr["recovery_time"] > 1e-6 and ml["recovery_time"] < 100 and mr["recovery_time"] < 100:
            improvements["rt"].append((1 - ml["recovery_time"] / mr["recovery_time"]) * 100)

        print("─" * 78)

    print(f"\n{'═' * 78}")
    print("  总体改善 (左臂 admittance 相对右臂 impedance-only, 正=左臂更优)")
    print(f"{'═' * 78}")
    for key, label in [("peak", "峰值偏差"), ("final", "末段|actual-cmd|"), ("net_recovery", "净恢复误差"), ("rt", "恢复时间")]:
        if improvements[key]:
            avg = np.mean(improvements[key])
            print(f"  {label+'改善:':<20}{avg:+.1f}%")
    print(f"{'═' * 78}")


# ──────────────────────── 主函数 ────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="实时录制 + 自动对比: 左臂(导纳) vs 右臂(纯阻抗)"
    )
    parser.add_argument("--left", type=str, default="6,7,8,9,10,11,12",
                        help="左臂关节索引 (默认 6-12)")
    parser.add_argument("--right", type=str, default="13,14,15,16,17,18,19",
                        help="右臂关节索引 (默认 13-19)")
    parser.add_argument("--settle", type=float, default=2.0,
                        help="稳态评估等待秒数 (默认 2.0)")
    parser.add_argument("--threshold", type=float, default=0.003,
                        help="恢复阈值 rad (默认 0.003)")
    parser.add_argument("--topic", type=str, default="/feedback/robot_server_state",
                        help="ROS topic")
    parser.add_argument("--save-dir", type=str, default=None,
                        help="图片保存目录 (默认脚本所在目录)")
    args = parser.parse_args()

    left_joints = [int(j) for j in args.left.split(",")]
    right_joints = [int(j) for j in args.right.split(",")]

    save_dir = Path(args.save_dir) if args.save_dir else Path(__file__).parent
    save_dir.mkdir(parents=True, exist_ok=True)
    save_path = save_dir / "comparison_impedance_vs_admittance.png"

    # JSONL 文件名带时间戳
    jsonl_path = save_dir / f"record_{time.strftime('%Y%m%d_%H%M%S')}.jsonl"

    # ── 启动 ROS ──
    rclpy.init()
    recorder = Recorder(args.topic)
    spin_thread = threading.Thread(target=rclpy.spin, args=(recorder,), daemon=True)
    spin_thread.start()

    # ── 等待数据到达 ──
    print("\n╔══════════════════════════════════════════════════════════╗")
    print("║  Impedance vs Admittance 实时对比工具                    ║")
    print("║                                                          ║")
    print("║  左臂 J6-12  = 阻抗+导纳 (有传感器)                     ║")
    print("║  右臂 J13-19 = 纯阻抗   (无传感器)                      ║")
    print("║                                                          ║")
    print("║  现在开始录制，请施加外力...                             ║")
    print("║  输入 s 并回车 → 停止录制并输出结果                     ║")
    print("╚══════════════════════════════════════════════════════════╝")
    print(f"  JSONL: {jsonl_path}")
    print()

    # 后台打印录制状态
    last_count = [0]
    stop_monitor = threading.Event()

    def monitor():
        while not stop_monitor.is_set():
            time.sleep(1.0)
            n = recorder.count
            rate = n - last_count[0]
            last_count[0] = n
            if rate > 0:
                print(f"  [录制中] {n} samples, ~{rate} Hz")

    monitor_thread = threading.Thread(target=monitor, daemon=True)
    monitor_thread.start()

    # ── 等待用户输入 s ──
    try:
        while True:
            cmd = input().strip().lower()
            if cmd == "s":
                break
    except (KeyboardInterrupt, EOFError):
        pass

    stop_monitor.set()

    # ── 停止录制 ──
    recorder.destroy_node()
    rclpy.shutdown()
    spin_thread.join(timeout=2.0)

    # ── 保存 JSONL ──
    recorder.save_jsonl(str(jsonl_path))
    print(f"[SAVED] {jsonl_path}")

    # ── 提取分析数据 ──
    timestamps, data = recorder.snapshot_analysis()

    if len(timestamps) < 20:
        print("\n[ERROR] 数据量太少 (<20 samples)，无法分析")
        sys.exit(1)

    # 相对时间 (从 0 开始)
    timestamps = timestamps - timestamps[0]

    # ── 分析 ──
    force_events, force_mag, threshold = detect_force_events(
        data["torque_ext"], left_joints
    )

    # ── 输出统计 ──
    print_statistics(timestamps, data, left_joints, right_joints,
                     force_events, threshold, args)

    # ── 保存图片 ──
    save_plots(timestamps, data, left_joints, right_joints,
               force_events, force_mag, threshold, args, save_path)
    print(f"[SAVED] {save_path}")
    print("[DONE]")


if __name__ == "__main__":
    main()
