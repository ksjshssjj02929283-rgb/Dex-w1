#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
从 robot_server 的 JSONL 日志中评估关节位置跟踪性能。

本脚本比较 joint_position_cmd 和 joint_position。对每个关节执行：
1. 从日志中读取成对的指令/反馈采样点；
2. 互相关计算前先对两路信号去均值；
3. 使用互相关估计整数采样点级别的时延；
4. 根据估计出的时延平移指令曲线，使其与实际位置曲线对齐；
5. 分别计算平移前和平移后的 RMS 误差；
6. 将原始曲线、平移后的曲线和 RMS 结果保存到 ~/log/evaluate_performance/时间戳目录。

使用示例：
    python3 script/evaluate_performance.py
    python3 script/evaluate_performance.py --seconds 10
    python3 script/evaluate_performance.py --joint 6
    python3 script/evaluate_performance.py --joint arms
    python3 script/evaluate_performance.py --log ~/log/robot_record.jsonl --max-lag 0.5
    python3 script/evaluate_performance.py --output-dir ~/log/evaluate_performance
"""

import argparse
import json
import math
import sys
from collections import deque
from datetime import datetime, timedelta
from pathlib import Path

np = None


def parse_line(line, debug=False):
    """解析一行 JSONL 日志，并提取 timestamp、joint_position_cmd、joint_position。"""
    line = line.strip()
    if not line:
        return None

    try:
        data = json.loads(line)
    except Exception as exc:
        if debug:
            print(f"[DEBUG] JSON 解析失败: {exc}")
        return None

    try:
        ts_float = float(data["timestamp"])
        ts = datetime.fromtimestamp(ts_float)
    except Exception as exc:
        if debug:
            print(f"[DEBUG] timestamp 解析失败: {exc}")
        return None

    pos = data.get("joint_position")
    cmd = data.get("joint_position_cmd")
    if not isinstance(pos, list) or not isinstance(cmd, list) or len(pos) != len(cmd):
        return None

    try:
        pos_arr = np.asarray(pos, dtype=float)
        cmd_arr = np.asarray(cmd, dtype=float)
    except Exception as exc:
        if debug:
            print(f"[DEBUG] 关节数组转换失败: {exc}")
        return None

    if pos_arr.ndim != 1 or cmd_arr.ndim != 1 or pos_arr.size == 0:
        return None

    return ts, cmd_arr, pos_arr


def read_samples(log_path, max_points, debug=False):
    """读取日志文件，保留同时包含指令位置和实际位置的有效采样点。"""
    samples = deque(maxlen=max_points if max_points > 0 else None)
    total_lines = 0
    parsed_count = 0

    with log_path.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            total_lines += 1
            parsed = parse_line(line, debug=debug)
            if parsed is None:
                continue
            samples.append(parsed)
            parsed_count += 1

    if debug:
        print(f"[DEBUG] 读取总行数: {total_lines}")
        print(f"[DEBUG] 有效成对采样点: {parsed_count}")
        print(f"[DEBUG] 当前保留采样点: {len(samples)}")

    return list(samples)


def filter_by_seconds(samples, seconds, debug=False):
    """按最后一个采样点向前截取指定秒数的数据窗口。"""
    if not samples or seconds is None or seconds <= 0:
        return samples

    last_ts = samples[-1][0]
    cutoff = last_ts - timedelta(seconds=seconds)
    filtered = [sample for sample in samples if sample[0] >= cutoff]

    if debug:
        print(f"[DEBUG] 最后一帧时间戳: {last_ts}")
        print(f"[DEBUG] 时间窗口起点: {cutoff}")
        print(f"[DEBUG] 时间过滤后采样点: {len(filtered)}")

    return filtered


def build_arrays(samples):
    """将采样点整理为时间数组和 Nx关节数 的指令/实际位置矩阵。"""
    joint_count = min(cmd.size for _, cmd, pos in samples if cmd.size == pos.size)
    usable = [(ts, cmd[:joint_count], pos[:joint_count]) for ts, cmd, pos in samples if cmd.size >= joint_count]

    times = np.asarray([(ts - usable[0][0]).total_seconds() for ts, _, _ in usable], dtype=float)
    cmd = np.vstack([sample_cmd for _, sample_cmd, _ in usable])
    pos = np.vstack([sample_pos for _, _, sample_pos in usable])
    return times, cmd, pos, usable[0][0]


def median_sample_time(times):
    """使用相邻时间差的中位数估计日志采样周期。"""
    if times.size < 2:
        return math.nan
    diffs = np.diff(times)
    diffs = diffs[np.isfinite(diffs) & (diffs > 0)]
    if diffs.size == 0:
        return math.nan
    return float(np.median(diffs))


def estimate_delay_by_xcorr(cmd, pos, dt, max_lag_seconds=None):
    """对单个关节使用归一化互相关估计指令到实际位置的时延。"""
    valid = np.isfinite(cmd) & np.isfinite(pos)
    cmd = cmd[valid]
    pos = pos[valid]
    n = cmd.size
    if n < 3:
        raise ValueError("有效采样点不足")

    # 去均值后再做互相关，避免静态偏置影响时延估计。
    cmd_demean = cmd - np.mean(cmd)
    pos_demean = pos - np.mean(pos)
    if np.allclose(cmd_demean, 0.0) or np.allclose(pos_demean, 0.0):
        raise ValueError("去均值后信号近似为常数")

    # 限制搜索时延范围，避免长窗口内无关片段产生错误匹配。
    max_possible_lag = n - 2
    if max_lag_seconds is None or not np.isfinite(dt) or dt <= 0:
        max_lag_samples = max_possible_lag
    else:
        max_lag_samples = max(0, min(max_possible_lag, int(round(max_lag_seconds / dt))))

    best_lag = 0
    best_corr = -np.inf
    for lag in range(-max_lag_samples, max_lag_samples + 1):
        # lag >= 0 表示实际位置相对指令滞后 lag 个采样点。
        if lag >= 0:
            cmd_overlap = cmd_demean[: n - lag]
            pos_overlap = pos_demean[lag:]
        else:
            cmd_overlap = cmd_demean[-lag:]
            pos_overlap = pos_demean[: n + lag]

        if cmd_overlap.size < 2:
            continue

        denom = np.linalg.norm(cmd_overlap) * np.linalg.norm(pos_overlap)
        if denom <= 0:
            continue

        corr = float(np.dot(cmd_overlap, pos_overlap) / denom)
        if corr > best_corr:
            best_corr = corr
            best_lag = lag

    delay_seconds = best_lag * dt if np.isfinite(dt) else math.nan
    return best_lag, delay_seconds, best_corr, cmd, pos


def aligned_series(times, cmd, pos, lag):
    """根据估计出的 lag 对齐指令曲线和实际位置曲线，并返回重叠区间。"""
    if lag >= 0:
        aligned_times = times[lag:]
        aligned_cmd_times = times[: times.size - lag] + (times[lag] - times[0] if lag > 0 else 0.0)
        aligned_cmd = cmd[: cmd.size - lag]
        aligned_pos = pos[lag:]
    else:
        lead = -lag
        aligned_times = times[: times.size - lead]
        aligned_cmd_times = times[lead:] - (times[lead] - times[0])
        aligned_cmd = cmd[lead:]
        aligned_pos = pos[: pos.size - lead]

    valid = np.isfinite(aligned_cmd) & np.isfinite(aligned_pos)
    return aligned_times[valid], aligned_cmd_times[valid], aligned_cmd[valid], aligned_pos[valid]


def compute_joint_metrics(times, cmd_matrix, pos_matrix, joint_indices, max_lag_seconds, debug=False):
    """逐关节估计时延，并计算平移前/平移后的 RMS 与最大绝对误差。"""
    dt = median_sample_time(times)
    results = []

    for joint in joint_indices:
        raw_cmd = cmd_matrix[:, joint]
        raw_pos = pos_matrix[:, joint]
        try:
            lag, delay_seconds, corr, cmd_valid, pos_valid = estimate_delay_by_xcorr(
                raw_cmd, raw_pos, dt, max_lag_seconds=max_lag_seconds
            )
        except ValueError as exc:
            if debug:
                print(f"[DEBUG] 跳过关节 {joint}: {exc}")
            else:
                print(f"跳过关节 {joint}: {exc}")
            continue

        valid = np.isfinite(raw_cmd) & np.isfinite(raw_pos)
        joint_times = times[valid]

        raw_err = cmd_valid - pos_valid
        raw_rms_rad = float(np.sqrt(np.mean(raw_err * raw_err)))
        raw_max_abs_rad = float(np.max(np.abs(raw_err)))

        t_aligned, cmd_time_aligned, cmd_aligned, pos_aligned = aligned_series(
            joint_times, cmd_valid, pos_valid, lag
        )
        aligned_err = cmd_aligned - pos_aligned
        aligned_rms_rad = float(np.sqrt(np.mean(aligned_err * aligned_err)))
        aligned_max_abs_rad = float(np.max(np.abs(aligned_err)))

        results.append(
            {
                "joint": joint,
                "lag_samples": lag,
                "delay_seconds": delay_seconds,
                "corr": corr,
                "raw_sample_count": int(raw_err.size),
                "aligned_sample_count": int(aligned_err.size),
                "raw_rms_rad": raw_rms_rad,
                "raw_rms_deg": float(np.degrees(raw_rms_rad)),
                "raw_max_abs_rad": raw_max_abs_rad,
                "raw_max_abs_deg": float(np.degrees(raw_max_abs_rad)),
                "aligned_rms_rad": aligned_rms_rad,
                "aligned_rms_deg": float(np.degrees(aligned_rms_rad)),
                "aligned_max_abs_rad": aligned_max_abs_rad,
                "aligned_max_abs_deg": float(np.degrees(aligned_max_abs_rad)),
                "times": joint_times,
                "cmd": cmd_valid,
                "pos": pos_valid,
                "aligned_times": t_aligned,
                "aligned_cmd_times": cmd_time_aligned,
                "aligned_cmd": cmd_aligned,
                "aligned_pos": pos_aligned,
            }
        )

    return results, dt


def print_summary(results, dt):
    if not results:
        print("没有关节生成有效评估结果。")
        return

    print(f"采样周期中位数: {dt:.6f} s" if np.isfinite(dt) else "采样周期中位数: 未知")
    print("关节 | 时延采样点 | 时延_s | 相关系数 | 原始RMS_deg | 对齐RMS_deg | 对齐样本数")
    print("-" * 86)
    for item in results:
        print(
            f"{item['joint']:>5d} | "
            f"{item['lag_samples']:>11d} | "
            f"{item['delay_seconds']:>7.4f} | "
            f"{item['corr']:>5.3f} | "
            f"{item['raw_rms_deg']:>11.4f} | "
            f"{item['aligned_rms_deg']:>11.4f} | "
            f"{item['aligned_sample_count']:>10d}"
        )

    if results:
        raw_rms_values = np.asarray([item["raw_rms_deg"] for item in results], dtype=float)
        aligned_rms_values = np.asarray([item["aligned_rms_deg"] for item in results], dtype=float)
        print("-" * 86)
        print(f"选中关节平均原始 RMS: {np.mean(raw_rms_values):.4f} deg")
        print(f"选中关节平均对齐 RMS: {np.mean(aligned_rms_values):.4f} deg")


def plot_results(results, start_timestamp, save_path, show_aligned):
    """绘制每个关节的指令位置和实际位置曲线。"""
    import matplotlib

    if "matplotlib.pyplot" not in sys.modules:
        matplotlib.use("Agg")

    import matplotlib.pyplot as plt

    joint_count = len(results)
    if joint_count == 0:
        return

    cols = 2 if joint_count > 1 else 1
    rows = int(math.ceil(joint_count / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(8.5 * cols, 3.2 * rows), dpi=110, squeeze=False)
    fig.suptitle(
        "Joint Position Tracking Performance "
        f"(Log Start: {start_timestamp.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]})",
        fontsize=12,
    )

    for ax, item in zip(axes.flatten(), results):
        if show_aligned:
            cmd_t = item["aligned_cmd_times"]
            cmd_y = np.degrees(item["aligned_cmd"])
            pos_t = item["aligned_times"]
            pos_y = np.degrees(item["aligned_pos"])
            title_suffix = (
                f"Aligned, delay={item['delay_seconds']:.4f}s, "
                f"RMS={item['aligned_rms_deg']:.4f} deg"
            )
        else:
            cmd_t = item["times"]
            cmd_y = np.degrees(item["cmd"])
            pos_t = item["times"]
            pos_y = np.degrees(item["pos"])
            title_suffix = f"Raw, RMS={item['raw_rms_deg']:.4f} deg"

        ax.plot(cmd_t, cmd_y, label="joint_position_cmd", linewidth=1.2)
        ax.plot(pos_t, pos_y, label="joint_position", linewidth=1.2)
        ax.set_title(
            f"Joint {item['joint']} ({title_suffix})",
            fontsize=9,
        )
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Position [deg]")
        ax.grid(True, linestyle=":", alpha=0.35)
        ax.legend(loc="best", fontsize=8)

    for ax in axes.flatten()[joint_count:]:
        ax.axis("off")

    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(save_path)
    plt.close(fig)
    print(f"图片已保存: {save_path}")


def create_report_dir(base_dir):
    """在输出目录下创建以当前时间命名的报告子目录。"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    report_dir = base_dir.expanduser() / timestamp
    suffix = 1
    while report_dir.exists():
        report_dir = base_dir.expanduser() / f"{timestamp}_{suffix:02d}"
        suffix += 1
    report_dir.mkdir(parents=True, exist_ok=False)
    return report_dir


def write_metrics(results, dt, report_dir, log_path, start_timestamp, args, joint_indices):
    """将平移前/平移后的 RMS 指标写入报告目录。"""
    metrics_path = report_dir / "rms_result.txt"
    raw_rms_values = np.asarray([item["raw_rms_deg"] for item in results], dtype=float)
    aligned_rms_values = np.asarray([item["aligned_rms_deg"] for item in results], dtype=float)

    lines = [
        "关节位置跟踪性能评估结果",
        "=" * 72,
        f"日志文件: {log_path}",
        f"日志数据起点: {start_timestamp.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}",
        f"采样周期中位数: {dt:.6f} s" if np.isfinite(dt) else "采样周期中位数: 未知",
        f"时间窗口: 最近 {args.seconds} s" if args.seconds > 0 else "时间窗口: 全部保留数据",
        f"关节选择参数: {args.joint}",
        f"实际评估关节索引: {joint_indices}",
        f"最大互相关搜索时延: {args.max_lag} s" if args.max_lag > 0 else "最大互相关搜索时延: 完整信号",
        "",
        "说明:",
        "figure1.png: 原始 joint_position_cmd 与 joint_position 曲线",
        "figure2.png: 根据互相关估计时延平移后的曲线",
        "RMS 单位: deg",
        "",
        "关节 | 时延采样点 | 时延_s | 相关系数 | 原始样本数 | 对齐样本数 | 原始RMS | 对齐RMS | 原始最大误差 | 对齐最大误差",
        "-" * 120,
    ]

    for item in results:
        lines.append(
            f"{item['joint']:>5d} | "
            f"{item['lag_samples']:>11d} | "
            f"{item['delay_seconds']:>7.4f} | "
            f"{item['corr']:>8.5f} | "
            f"{item['raw_sample_count']:>10d} | "
            f"{item['aligned_sample_count']:>10d} | "
            f"{item['raw_rms_deg']:>8.4f} | "
            f"{item['aligned_rms_deg']:>8.4f} | "
            f"{item['raw_max_abs_deg']:>12.4f} | "
            f"{item['aligned_max_abs_deg']:>12.4f}"
        )

    lines.extend(
        [
            "-" * 120,
            f"选中关节平均原始 RMS: {np.mean(raw_rms_values):.4f} deg",
            f"选中关节平均对齐 RMS: {np.mean(aligned_rms_values):.4f} deg",
        ]
    )

    metrics_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"RMS 结果已保存: {metrics_path}")
    return metrics_path


def arm_joint_indices(joint_count):
    """根据日志关节数量推断双臂关节索引。"""
    if joint_count == 20:
        return list(range(6, 20))
    if joint_count == 14:
        return list(range(joint_count))
    raise ValueError(f"无法从 joint_count={joint_count} 推断双臂关节索引，请使用 --joint 手动指定。")


def parse_joint_indices(joint_arg, joint_count):
    """解析 --joint 参数，支持 arms、all、单个索引、逗号列表和闭区间范围。"""
    if joint_arg is None or joint_arg.lower() == "arms":
        return arm_joint_indices(joint_count)
    if joint_arg.lower() == "all":
        return list(range(joint_count))

    indices = []
    for part in joint_arg.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            start_text, end_text = part.split("-", 1)
            start = int(start_text)
            end = int(end_text)
            indices.extend(range(start, end + 1))
        else:
            indices.append(int(part))

    indices = sorted(set(indices))
    bad = [idx for idx in indices if idx < 0 or idx >= joint_count]
    if bad:
        raise ValueError(f"关节索引超出范围，joint_count={joint_count}: {bad}")
    return indices


def main():
    global np

    parser = argparse.ArgumentParser(description="评估 joint_position_cmd 与 joint_position 的位置跟踪性能。")
    parser.add_argument(
        "--log",
        type=Path,
        default=Path("~/log/robot_record.jsonl").expanduser(),
        help="robot_record JSONL 日志路径，默认 ~/log/robot_record.jsonl。",
    )
    parser.add_argument(
        "--seconds",
        type=float,
        default=5.0,
        help="只使用保留数据中最后 N 秒；设置 <=0 表示使用全部保留数据，默认 5。",
    )
    parser.add_argument(
        "--n",
        type=int,
        default=50000,
        help="读取时最多保留的有效成对采样点数量；设置 <=0 表示不限制，默认 50000。",
    )
    parser.add_argument(
        "--joint",
        type=str,
        default="arms",
        help="关节选择：arms、all、单个索引、逗号列表，或 6-12 这种闭区间范围，默认 arms；20关节取6-19，14关节取0-13。",
    )
    parser.add_argument(
        "--max-lag",
        type=float,
        default=1.0,
        help="互相关搜索的最大绝对时延，单位秒；设置 <=0 表示搜索完整信号，默认 1.0。",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("~/log/evaluate_performance").expanduser(),
        help="评估结果根目录；脚本会在该目录下创建时间戳子目录，默认 ~/log/evaluate_performance。",
    )
    parser.add_argument("--debug", action="store_true", help="输出调试信息。")
    args = parser.parse_args()

    try:
        import numpy as numpy_module
    except ModuleNotFoundError:
        print("缺少依赖: numpy。请在运行该脚本的 Python 环境中安装 numpy。", file=sys.stderr)
        return 1
    np = numpy_module

    log_path = args.log.expanduser()
    if not log_path.exists():
        print(f"找不到日志文件: {log_path}", file=sys.stderr)
        return 1

    samples = read_samples(log_path, args.n, debug=args.debug)
    samples = filter_by_seconds(samples, args.seconds, debug=args.debug)
    if not samples:
        print("没有找到有效的 joint_position_cmd/joint_position 成对采样点。", file=sys.stderr)
        return 1

    times, cmd_matrix, pos_matrix, start_timestamp = build_arrays(samples)
    joint_count = cmd_matrix.shape[1]
    joint_indices = parse_joint_indices(args.joint, joint_count)
    max_lag_seconds = None if args.max_lag <= 0 else args.max_lag

    if args.debug:
        print(f"[DEBUG] 关节数量: {joint_count}")
        print(f"[DEBUG] 选中关节: {joint_indices}")
        print(f"[DEBUG] 时间范围: {times[0]:.6f}s 到 {times[-1]:.6f}s")

    results, dt = compute_joint_metrics(times, cmd_matrix, pos_matrix, joint_indices, max_lag_seconds, debug=args.debug)
    print_summary(results, dt)
    if not results:
        return 1

    report_dir = create_report_dir(args.output_dir)
    figure1_path = report_dir / "figure1.png"
    figure2_path = report_dir / "figure2.png"
    write_metrics(results, dt, report_dir, log_path, start_timestamp, args, joint_indices)

    try:
        plot_results(results, start_timestamp, save_path=figure1_path, show_aligned=False)
        plot_results(results, start_timestamp, save_path=figure2_path, show_aligned=True)
    except ModuleNotFoundError as exc:
        if exc.name == "matplotlib":
            print(
                "缺少依赖: matplotlib。指标已经计算完成，但绘图需要 matplotlib。",
                file=sys.stderr,
            )
            return 1
        raise

    print(f"评估结果目录: {report_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())