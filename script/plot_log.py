import argparse
import json
import os
import re
import sys
from collections import deque
from datetime import datetime, timedelta
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker


def parse_line(line, debug=False):
    """
    解析单行：
    - 提取 [] 中的时间戳
    - 提取 {} 中的 JSON
    返回: (timestamp: datetime, data: dict) 或 None
    """
    line = line.strip()
    if not line:
        if debug:
            print("[DEBUG] Empty line skipped")
        return None

    # 提取时间戳
    ts_match = re.search(r"\[(.*?)\]", line)
    if not ts_match:
        if debug:
            print(f"[DEBUG] No timestamp found in line: {line[:80]}...")
        return None
    ts_str = ts_match.group(1)
    try:
        # 形如 2025-12-02 15:07:32.170155
        ts = datetime.fromisoformat(ts_str)
    except Exception as e:
        if debug:
            print(f"[DEBUG] Failed to parse timestamp '{ts_str}': {e}")
        return None

    # 提取 JSON
    brace_idx = line.find("{")
    if brace_idx == -1:
        if debug:
            print(f"[DEBUG] No JSON found in line: {line[:80]}...")
        return None
    json_str = line[brace_idx:]
    try:
        data = json.loads(json_str)
    except Exception as e:
        if debug:
            print(f"[DEBUG] Failed to parse JSON: {e}")
        return None

    # 校验存在的数组长度是否为 20（标准模式）或 14（A1模式）
    array_keys = [
        "joint_position",
        "joint_velocity",
        "joint_torque",
        "joint_position_cmd",
        "joint_torque_feedforward",
    ]
    for k in array_keys:
        if k in data:
            arr = data[k]
            if not isinstance(arr, list) or len(arr) not in (20, 14):
                print(f"[DEBUG] Invalid array length for {k}: {len(arr) if isinstance(arr, list) else type(arr)}")
                return None

    return ts, data


def main():
    if len(sys.argv) >= 2:
        m = re.fullmatch(r"-?\d+(\.\d+)?", sys.argv[1])
        if m:
            sec = abs(float(sys.argv[1]))
            sys.argv[1] = f"--seconds={sec}"

    parser = argparse.ArgumentParser(description="Plot last N points from robot_server log.")
    parser.add_argument(
        "--path",
        type=str,
        default="~/log/robot_server.txt",
        help="Path to robot_server log file.",
    )
    parser.add_argument(
        "--n",
        type=int,
        default=5000,
        help="Maximum number of valid points to keep while reading (legacy, default: 5000).",
    )
    parser.add_argument(
        "--seconds",
        type=float,
        default=2.0,
        help="Time window in seconds to plot from the end (default: 2.0).",
    )
    parser.add_argument(
        "--save",
        type=str,
        default=None,
        help="Save plots to the specified directory instead of showing.",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable debug logging.",
    )
    parser.add_argument(
        "--torso",
        action="store_true",
        help="Only plot torso data (joint 0-5).",
    )
    parser.add_argument(
        "--a1",
        action="store_true",
        help="A1 mode: only plot arms (joint 6-19, 14 joints).",
    )
    parser.add_argument(
        "--hz",
        action="store_true",
        help="Show frequency spectrum for each joint.",
    )
    args = parser.parse_args()

    log_path = Path(args.path).expanduser()
    if not log_path.exists():
        print(f"Log file not found: {log_path}")
        return

    if args.debug:
        print(f"[DEBUG] Reading log file: {log_path}")
        print(f"[DEBUG] Max samples: {args.n}, Time window: {args.seconds}s")

    # 用 deque 限制读取时的最大样本数，最终按时间窗口过滤
    samples = deque(maxlen=args.n)
    total_lines = 0
    parsed_count = 0
    failed_count = 0

    with open(log_path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            total_lines += 1
            parsed = parse_line(line, debug=args.debug)
            if parsed is None:
                failed_count += 1
                continue
            ts, data = parsed
            samples.append((ts, data))
            parsed_count += 1

    if args.debug:
        print(f"[DEBUG] Total lines read: {total_lines}")
        print(f"[DEBUG] Successfully parsed: {parsed_count}, Failed: {failed_count}")
        print(f"[DEBUG] Samples in buffer: {len(samples)}")

    if not samples:
        print("No valid samples found.")
        return

    # 过滤：只保留包含至少一个关节数组的数据行
    valid_samples = deque()
    for ts, d in samples:
        has_joint_data = any(k in d for k in ["joint_position", "joint_velocity", "joint_torque"])
        if has_joint_data:
            valid_samples.append((ts, d))

    if args.debug:
        print(f"[DEBUG] Samples with joint data: {len(valid_samples)} (filtered out {len(samples) - len(valid_samples)} non-joint entries)")

    if not valid_samples:
        print("No joint data samples found.")
        return

    samples = valid_samples

    # 按时间窗口过滤：以最后一条样本为基准，保留最近 args.seconds 秒的样本
    last_ts = samples[-1][0]
    cutoff = last_ts - timedelta(seconds=abs(args.seconds))
    if args.debug:
        print(f"[DEBUG] Last timestamp: {last_ts}")
        print(f"[DEBUG] Cutoff time: {cutoff} (last {args.seconds}s)")
    filtered = deque()
    for ts, d in samples:
        if ts >= cutoff:
            filtered.append((ts, d))

    if args.debug:
        print(f"[DEBUG] Samples after time filtering: {len(filtered)}")
        if filtered:
            # 显示第一个和最后一个样本的详细信息
            first_ts, first_data = filtered[0]
            last_ts, last_data = filtered[-1]
            print(f"[DEBUG] First sample time: {first_ts}")
            print(f"[DEBUG] Last sample time: {last_ts}")
            print(f"[DEBUG] First sample keys: {list(first_data.keys())}")
            for key in ["joint_position", "joint_velocity", "joint_torque"]:
                if key in first_data:
                    arr = first_data[key]
                    print(f"[DEBUG] {key}: len={len(arr)}, first_val={arr[0]:.4f}, last_val={arr[-1]:.4f}")

    if not filtered:
        print(f"No samples found within the last {args.seconds} seconds.")
        return

    samples = filtered
    start_timestamp = samples[0][0] # 获取记录开始的时间戳

    # 组装为时间序列（长度 = 样本条数），以保留样本的第一个时间为基点
    times = np.array([(s[0] - samples[0][0]).total_seconds() for s in samples], dtype=float)
    n = len(samples)

    # 根据第一个有效样本确定关节数量（14=A1模式，20=标准模式）
    joint_count = 14
    for _, d in samples:
        if "joint_position" in d and isinstance(d["joint_position"], list):
            joint_count = len(d["joint_position"])
            break

    if args.debug:
        print(f"[DEBUG] Joint count: {joint_count} ({'A1' if joint_count == 14 else 'Standard'} mode)")

    # 为每个键准备 Nx关节数 的数组，缺失数据用 NaN 填充
    def build_array(key):
        arr = np.full((n, joint_count), np.nan, dtype=float)
        for i, (_, d) in enumerate(samples):
            if key in d:
                v = d[key]
                if isinstance(v, list) and len(v) == joint_count:
                    arr[i, :] = np.array(v, dtype=float)
        return arr

    pos = build_array("joint_position")
    pos_cmd = build_array("joint_position_cmd")
    tor = build_array("joint_torque")
    tor_ff = build_array("joint_torque_feedforward")
    vel = build_array("joint_velocity")

    if args.debug:
        print(f"[DEBUG] Data array shape: ({n}, {joint_count})")
        print(f"[DEBUG] Position data - NaN count: {np.sum(~np.isfinite(pos))}, valid: {np.sum(np.isfinite(pos))}")
        print(f"[DEBUG] Torque data - NaN count: {np.sum(~np.isfinite(tor))}, valid: {np.sum(np.isfinite(tor))}")
        print(f"[DEBUG] Velocity data - NaN count: {np.sum(~np.isfinite(vel))}, valid: {np.sum(np.isfinite(vel))}")

    # 将位置从弧度转换为度（便于展示）
    pos_deg = np.degrees(pos)
    pos_cmd_deg = np.degrees(pos_cmd)

    # 误差（仅在双方都有数据时才会得到数值，否则为 NaN），单位为度
    pos_err_deg = pos_cmd_deg - pos_deg
    tor_err = tor_ff - tor
    # 速度已读取，转换为度/秒
    vel_deg = np.degrees(vel)

    # 设置matplotlib参数以最大化显示
    plt.rcParams['figure.max_open_warning'] = 100  # 允许更多图形
    plt.rcParams['figure.autolayout'] = False  # 手动调整布局

    # 如果需要保存，使用非交互式后端
    save_dir = args.save
    if save_dir:
        os.makedirs(save_dir, exist_ok=True)
        plt.switch_backend('Agg')

    # 每个手臂一张大图，每张图包含7个关节
    # 如果启用 --hz，则添加频谱图（第5行）
    def plot_arm(arm_name, joint_indices, arm_side, times_input, start_timestamp, show_hz=False):
        rows = 5 if show_hz else 4
        cols = len(joint_indices)
        fig, axes = plt.subplots(rows, cols, figsize=(19.2, 14 if show_hz else 11), dpi=100)
        fig.suptitle(f"{arm_name} (Log Start: {start_timestamp.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]})", fontsize=10)

        # 将所有坐标轴刻度格式化为小数点后3位
        for ax in axes.flatten():
            ax.xaxis.set_major_formatter(mticker.FormatStrFormatter('%.3f'))
            ax.yaxis.set_major_formatter(mticker.FormatStrFormatter('%.3f'))

        for ci, j in enumerate(joint_indices):
            joint_num = ci + 1
            # torso 关节最后两个改名为 head1, head2
            if arm_name == "Torso" and ci >= 4:
                col_title = f"Head{ci - 3}"
            else:
                col_title = f"{arm_side} J{joint_num}"

            # 1) position（度）
            ax = axes[0, ci]
            ax.plot(times, pos_deg[:, j], label="position", color="tab:blue", linewidth=1.2)
            if np.isfinite(pos_cmd_deg[:, j]).any():
                ax.plot(times, pos_cmd_deg[:, j], label="position_cmd",
                       color="tab:green", linewidth=1.2, linestyle="--")
            ax.grid(True, alpha=0.3, linestyle=':')
            ax.tick_params(labelsize=7)
            ax.set_title(col_title, fontsize=9)
            if ci == 0:
                ax.set_ylabel("Position [deg]", fontsize=7)
                ax.legend(loc="best", fontsize=7)
            else:
                pass  # 仅首列显示图例
            # 强制 position 纵轴范围至少为 0.1（对已有数据有效）
            try:
                p_vals = np.hstack([
                    pos_deg[:, j][np.isfinite(pos_deg[:, j])],
                    pos_cmd_deg[:, j][np.isfinite(pos_cmd_deg[:, j])]
                ])
                if p_vals.size > 0:
                    pmin = p_vals.min()
                    pmax = p_vals.max()
                    prank = pmax - pmin
                    # 最小范围以度为单位
                    min_range = 0.4
                    if prank < min_range:
                        center = 0.5 * (pmax + pmin)
                        half = 0.5 * min_range
                        ax.set_ylim(center - half, center + half)
            except Exception:
                pass

            # 2) velocity
            ax = axes[1, ci]
            if np.isfinite(vel_deg[:, j]).any():
                ax.plot(times, vel_deg[:, j], label="velocity",
                       color="tab:brown", linewidth=1.2)
                if ci == 0:
                    ax.legend(loc="best", fontsize=7)
            else:
                ax.text(0.5, 0.5, "No velocity data",
                       transform=ax.transAxes, ha="center", va="center",
                       alpha=0.6, fontsize=10)
            ax.grid(True, alpha=0.3, linestyle=':')
            ax.tick_params(labelsize=7)
            # 强制 velocity 纵轴范围至少为 0.02 rad/s（对已有数据有效）
            try:
                v_vals = vel_deg[:, j][np.isfinite(vel_deg[:, j])]
                if v_vals.size > 0:
                    vmin = v_vals.min()
                    vmax = v_vals.max()
                    vrange = vmax - vmin
                    # 保持当前数值门限不变（单位现在为 deg/s）
                    min_vrange = 0.4
                    if vrange < min_vrange:
                        center = 0.5 * (vmax + vmin)
                        half = 0.5 * min_vrange
                        ax.set_ylim(center - half, center + half)
            except Exception:
                pass
            if ci == 0:
                ax.set_ylabel("Velocity [deg/s]", fontsize=7)

            # 3) position error
            ax = axes[2, ci]
            if np.isfinite(pos_err_deg[:, j]).any():
                ax.plot(times, pos_err_deg[:, j], label="pos_error",
                       color="tab:purple", linewidth=1.2)
                if ci == 0:
                    ax.legend(loc="best", fontsize=7)
            else:
                ax.text(0.5, 0.5, "No position error data",
                       transform=ax.transAxes, ha="center", va="center",
                       alpha=0.6, fontsize=10)
            # 新增：强制 position error 纵轴范围至少为 0.1 度
            try:
                pe_vals = pos_err_deg[:, j][np.isfinite(pos_err_deg[:, j])]
                if pe_vals.size > 0:
                    pmin = pe_vals.min()
                    pmax = pe_vals.max()
                    prank = pmax - pmin
                    min_perr_range = 0.04
                    if prank < min_perr_range:
                        center = 0.5 * (pmax + pmin)
                        half = 0.5 * min_perr_range
                        ax.set_ylim(center - half, center + half)
            except Exception:
                pass
            ax.grid(True, alpha=0.3, linestyle=':')
            ax.tick_params(labelsize=7)
            if ci == 0:
                ax.set_ylabel("Pos Error [deg]", fontsize=7)
            # 4) torque
            ax = axes[3, ci]
            ax.plot(times, tor[:, j], label="torque", color="tab:red", linewidth=1.2)
            ax.grid(True, alpha=0.3, linestyle=':')
            ax.tick_params(labelsize=7)
            if ci == 0:
                ax.set_ylabel("Torque [Nm]", fontsize=7)
                ax.legend(loc="best", fontsize=7)
            # 强制 torque 纵轴范围至少为 1.0（对已有数据有效）
            try:
                t_vals = tor[:, j][np.isfinite(tor[:, j])]
                if t_vals.size > 0:
                    tmin = t_vals.min()
                    tmax = t_vals.max()
                    trange = tmax - tmin
                    min_trange = 0.4
                    if trange < min_trange:
                        center = 0.5 * (tmax + tmin)
                        half = 0.5 * min_trange
                        ax.set_ylim(center - half, center + half)
            except Exception:
                pass

            # 5) position spectrum (FFT) - only if --hz enabled
            if show_hz:
                ax = axes[4, ci]
                valid_pos_mask = np.isfinite(pos_deg[:, j])
                valid_pos = pos_deg[:, j][valid_pos_mask]
                if valid_pos.size >= 2:
                    valid_times = times[valid_pos_mask]
                    time_diffs = np.diff(valid_times)
                    time_diffs = time_diffs[time_diffs > 0]
                    if len(time_diffs) >= 1:
                        dt = np.mean(time_diffs)
                    else:
                        dt = 0.001
                    fs = 1.0 / dt if dt > 0 else 1000.0
                    n = valid_pos.size
                    pos_centered = valid_pos - np.mean(valid_pos)
                    fft_vals = np.fft.fft(pos_centered)
                    fft_freq = np.fft.fftfreq(n, d=dt)
                    pos_mask = fft_freq > 0
                    fft_freq_pos = fft_freq[pos_mask]
                    fft_mag = np.abs(fft_vals[pos_mask])
                    fft_mag_norm = 2.0 / n * fft_mag
                    freq_mask = fft_freq_pos <= 10
                    ax.plot(fft_freq_pos[freq_mask], fft_mag_norm[freq_mask],
                        color="tab:blue", linewidth=1.0)
                    ax.grid(True, alpha=0.3, linestyle=':')
                    ax.tick_params(labelsize=7)
                    if fft_mag_norm[freq_mask].size > 0:
                        peak_idx = np.argmax(fft_mag_norm[freq_mask])
                        if fft_mag_norm[freq_mask][peak_idx] > 0.01 * fft_mag_norm[freq_mask].max():
                            peak_freq = fft_freq_pos[freq_mask][peak_idx]
                            ax.axvline(x=peak_freq, color='tab:red', linestyle='--', alpha=0.7, linewidth=1)
                            ax.annotate(f'{peak_freq:.1f}Hz',
                                    xy=(peak_freq, fft_mag_norm[freq_mask][peak_idx]),
                                    xytext=(peak_freq * 1.2, fft_mag_norm[freq_mask][peak_idx] * 0.8),
                                    fontsize=8, color='tab:red')
                    ax.set_xlim(0, 10)
                    ax.set_xlabel("(Hz)", fontsize=7, x=1.02, ha='left')
                else:
                    ax.text(0.5, 0.5, "No spectrum data",
                        transform=ax.transAxes, ha="center", va="center",
                        alpha=0.6, fontsize=10)
                    ax.set_xlabel("(Hz)", fontsize=7, x=1.02, ha='left')
                if ci == 0:
                    ax.set_ylabel("Spectrum [deg]", fontsize=7)

        # 调整布局
        fig.tight_layout(rect=[0.02, 0.02, 1, 0.96], h_pad=2, w_pad=2)
        return fig

    # A1模式：只有双臂14个关节（数据数组索引0-13，对应原始关节6-19）
    if args.a1:
        left_joints = list(range(7))
        right_joints = list(range(7, 14))
    else:
        left_joints = list(range(6, 13))
        right_joints = list(range(13, 20))
    # Torso：关节索引 0-5 (6个关节)
    torso_joints = list(range(6))

    figs = []
    if args.a1:
        if args.debug:
            print("[DEBUG] Creating A1 mode plots (arms only)...")
        figs.append(plot_arm("Left Arm", left_joints, "Left", times, start_timestamp, show_hz=args.hz))
        figs.append(plot_arm("Right Arm", right_joints, "Right", times, start_timestamp, show_hz=args.hz))
    elif args.torso:
        if args.debug:
            print("[DEBUG] Creating Torso plot only...")
        figs.append(plot_arm("Torso", torso_joints, "Torso", times, start_timestamp, show_hz=args.hz))
    else:
        if args.debug:
            print("[DEBUG] Creating Torso plot...")
        figs.append(plot_arm("Torso", torso_joints, "Torso", times, start_timestamp, show_hz=args.hz))
        if args.debug:
            print("[DEBUG] Creating Left Arm plot...")
        figs.append(plot_arm("Left Arm", left_joints, "Left", times, start_timestamp, show_hz=args.hz))
        if args.debug:
            print("[DEBUG] Creating Right Arm plot...")
        figs.append(plot_arm("Right Arm", right_joints, "Right", times, start_timestamp, show_hz=args.hz))
    if args.debug:
        print(f"[DEBUG] {len(figs)} figures created successfully")

    if save_dir:
        for i, fig in enumerate(figs):
            filenames = ["torso.png", "left_arm.png", "right_arm.png"]
            fig.savefig(os.path.join(save_dir, filenames[i]), dpi=100)
            print(f"Saved: {os.path.join(save_dir, filenames[i])}")
        plt.close('all')
    else:
        plt.show()


if __name__ == "__main__":
    main()