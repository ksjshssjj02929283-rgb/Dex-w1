#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
对多个 robot_record.jsonl 日志绘制“逐时刻 RMS”曲线。

使用方式：
1. 在 LOG_CONFIGS 中填入 5 个日志路径与图例名；
2. 可选修改 JOINT_SELECTOR、MAX_LAG_SECONDS、OUTPUT_DIR；
3. 运行：python3 script/plot_multi_log_rms.py

输出：
- figure_raw_rms.png: 全数据的对齐前 RMS 曲线（每个时刻一个 RMS）
- figure_aligned_rms.png: 全数据的对齐后 RMS 曲线（每个时刻一个 RMS）
"""

import json
import math
import sys
from datetime import datetime
from pathlib import Path

import numpy as np


# 你可以在这里填入 5 个日志路径和图例名称。
LOG_CONFIGS = [
    {"path": "", "label": ""},
    {"path": "", "label": ""},
    {"path": "", "label": ""},
    {"path": "", "label": ""},
    {"path": "", "label": ""},
]

# 关节选择："arms" / "all" / "0,1,2" / "6-12" / "6-12,15"
JOINT_SELECTOR = "arms"

# 互相关估计时延时的最大绝对搜索范围（秒）；<=0 表示不限制。
MAX_LAG_SECONDS = 0.5

# 输出目录（脚本会在该目录下创建时间戳子目录）
OUTPUT_DIR = Path("~/log/evaluate_performance/multi_log_rms").expanduser()


def parse_line(line):
    line = line.strip()
    if not line:
        return None

    try:
        data = json.loads(line)
    except Exception:
        return None

    try:
        ts = float(data["timestamp"])
    except Exception:
        return None

    cmd = data.get("joint_position_cmd")
    pos = data.get("joint_position")
    if not isinstance(cmd, list) or not isinstance(pos, list) or len(cmd) != len(pos):
        return None

    try:
        cmd_arr = np.asarray(cmd, dtype=float)
        pos_arr = np.asarray(pos, dtype=float)
    except Exception:
        return None

    if cmd_arr.ndim != 1 or pos_arr.ndim != 1 or cmd_arr.size == 0:
        return None

    return ts, cmd_arr, pos_arr


def read_samples(log_path):
    samples = []
    with log_path.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            parsed = parse_line(line)
            if parsed is not None:
                samples.append(parsed)
    return samples


def build_arrays(samples):
    joint_count = min(cmd.size for _, cmd, pos in samples if cmd.size == pos.size)
    usable = [(ts, cmd[:joint_count], pos[:joint_count]) for ts, cmd, pos in samples if cmd.size >= joint_count]

    ts0 = usable[0][0]
    times = np.asarray([ts - ts0 for ts, _, _ in usable], dtype=float)
    cmd = np.vstack([x for _, x, _ in usable])
    pos = np.vstack([x for _, _, x in usable])
    return times, cmd, pos, joint_count


def median_sample_time(times):
    if times.size < 2:
        return math.nan
    diffs = np.diff(times)
    diffs = diffs[np.isfinite(diffs) & (diffs > 0)]
    if diffs.size == 0:
        return math.nan
    return float(np.median(diffs))


def estimate_delay_by_xcorr(cmd, pos, dt, max_lag_seconds=None):
    valid = np.isfinite(cmd) & np.isfinite(pos)
    cmd = cmd[valid]
    pos = pos[valid]
    n = cmd.size
    if n < 3:
        raise ValueError("有效采样点不足")

    cmd_demean = cmd - np.mean(cmd)
    pos_demean = pos - np.mean(pos)
    if np.allclose(cmd_demean, 0.0) or np.allclose(pos_demean, 0.0):
        raise ValueError("去均值后信号近似常数")

    max_possible_lag = n - 2
    if max_lag_seconds is None or not np.isfinite(dt) or dt <= 0:
        max_lag_samples = max_possible_lag
    else:
        max_lag_samples = max(0, min(max_possible_lag, int(round(max_lag_seconds / dt))))

    best_lag = 0
    best_corr = -np.inf
    for lag in range(-max_lag_samples, max_lag_samples + 1):
        if lag >= 0:
            a = cmd_demean[: n - lag]
            b = pos_demean[lag:]
        else:
            a = cmd_demean[-lag:]
            b = pos_demean[: n + lag]

        if a.size < 2:
            continue

        denom = np.linalg.norm(a) * np.linalg.norm(b)
        if denom <= 0:
            continue

        corr = float(np.dot(a, b) / denom)
        if corr > best_corr:
            best_corr = corr
            best_lag = lag

    return best_lag


def arm_joint_indices(joint_count):
    if joint_count == 20:
        return list(range(6, 20))
    if joint_count == 14:
        return list(range(joint_count))
    raise ValueError(f"无法从 joint_count={joint_count} 推断 arms，请改用显式关节列表。")


def parse_joint_indices(joint_arg, joint_count):
    text = (joint_arg or "arms").strip().lower()
    if text == "arms":
        return arm_joint_indices(joint_count)
    if text == "all":
        return list(range(joint_count))

    indices = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            a, b = part.split("-", 1)
            start = int(a)
            end = int(b)
            if end < start:
                start, end = end, start
            indices.extend(range(start, end + 1))
        else:
            indices.append(int(part))

    indices = sorted(set(indices))
    bad = [i for i in indices if i < 0 or i >= joint_count]
    if bad:
        raise ValueError(f"关节索引超出范围，joint_count={joint_count}: {bad}")
    if not indices:
        raise ValueError("未解析出任何有效关节索引")
    return indices


def compute_raw_rms_per_time(cmd_matrix, pos_matrix, joint_indices):
    err = cmd_matrix[:, joint_indices] - pos_matrix[:, joint_indices]
    valid = np.isfinite(err)
    sq = np.where(valid, err * err, np.nan)
    # 每一帧对选中关节做 RMS
    mean_sq = np.nanmean(sq, axis=1)
    rms = np.sqrt(mean_sq)
    return rms


def compute_aligned_rms_per_time(times, cmd_matrix, pos_matrix, joint_indices, max_lag_seconds):
    n = times.size
    dt = median_sample_time(times)
    aligned_err = np.full((n, len(joint_indices)), np.nan, dtype=float)

    for col, joint in enumerate(joint_indices):
        cmd = cmd_matrix[:, joint]
        pos = pos_matrix[:, joint]
        lag = estimate_delay_by_xcorr(cmd, pos, dt, max_lag_seconds=max_lag_seconds)

        if lag >= 0:
            # pos 相对 cmd 滞后，误差落在 [lag, n)
            size = n - lag
            if size > 0:
                e = cmd[:size] - pos[lag: lag + size]
                aligned_err[lag:, col] = e
        else:
            lead = -lag
            size = n - lead
            if size > 0:
                e = cmd[lead: lead + size] - pos[:size]
                aligned_err[:size, col] = e

    valid = np.isfinite(aligned_err)
    sq = np.where(valid, aligned_err * aligned_err, np.nan)
    mean_sq = np.nanmean(sq, axis=1)
    rms = np.sqrt(mean_sq)
    return rms


def make_output_dir(base_dir):
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = base_dir / stamp
    suffix = 1
    while out.exists():
        out = base_dir / f"{stamp}_{suffix:02d}"
        suffix += 1
    out.mkdir(parents=True, exist_ok=False)
    return out


def trim_to_min_samples(series_list):
    """以最短序列的样本数为准，截取每条序列的末尾段，横轴统一重置为从 0 开始。"""
    min_n = min(item["rms_rad"].size for item in series_list)
    trimmed = []
    for item in series_list:
        rms = item["rms_rad"][-min_n:]
        times = item["times"][-min_n:]
        # 重置横轴：最早时刻归零
        times = times - times[0]
        trimmed.append({"label": item["label"], "times": times, "rms_rad": rms})
    return trimmed, min_n


def plot_curves(series_list, title, ylabel, save_path):
    import matplotlib

    if "matplotlib.pyplot" not in sys.modules:
        matplotlib.use("Agg")

    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(1, 1, figsize=(12, 5), dpi=120)
    for item in series_list:
        t = item["times"]
        y = np.degrees(item["rms_rad"])
        valid = np.isfinite(t) & np.isfinite(y)
        ax.plot(t[valid], y[valid], linewidth=1.4, label=item["label"])

    ax.set_title(title)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(ylabel)
    ax.grid(True, linestyle=":", alpha=0.35)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(save_path)
    plt.close(fig)
    print(f"图片已保存: {save_path}")


def validate_configs(configs):
    filled = []
    for idx, item in enumerate(configs, start=1):
        p = str(item.get("path", "")).strip()
        label = str(item.get("label", "")).strip()
        if not p:
            continue
        if not label:
            raise ValueError(f"第 {idx} 个配置已填写 path 但未填写 label")
        filled.append({"path": Path(p).expanduser(), "label": label})

    if len(filled) != 5:
        raise ValueError(f"请恰好填写 5 个配置，当前有效配置数量为 {len(filled)}")
    return filled


def main():
    try:
        configs = validate_configs(LOG_CONFIGS)
    except Exception as exc:
        print(f"配置错误: {exc}", file=sys.stderr)
        return 1

    raw_series = []
    aligned_series = []

    for cfg in configs:
        log_path = cfg["path"]
        label = cfg["label"]

        if not log_path.exists():
            print(f"找不到日志文件: {log_path}", file=sys.stderr)
            return 1

        samples = read_samples(log_path)
        if not samples:
            print(f"日志无有效样本: {log_path}", file=sys.stderr)
            return 1

        times, cmd_matrix, pos_matrix, joint_count = build_arrays(samples)
        try:
            joint_indices = parse_joint_indices(JOINT_SELECTOR, joint_count)
        except Exception as exc:
            print(f"关节选择错误 ({log_path}): {exc}", file=sys.stderr)
            return 1

        max_lag = None if MAX_LAG_SECONDS <= 0 else MAX_LAG_SECONDS

        try:
            raw_rms = compute_raw_rms_per_time(cmd_matrix, pos_matrix, joint_indices)
            aligned_rms = compute_aligned_rms_per_time(times, cmd_matrix, pos_matrix, joint_indices, max_lag)
        except Exception as exc:
            print(f"计算失败 ({log_path}): {exc}", file=sys.stderr)
            return 1

        raw_series.append({"label": label, "times": times, "rms_rad": raw_rms})
        aligned_series.append({"label": label, "times": times, "rms_rad": aligned_rms})

        print(
            f"已处理: {label} | 样本数={times.size} | 关节数={joint_count} | 选中关节={len(joint_indices)}"
        )

    # 按最短序列从末尾对齐，横轴从 0 开始
    raw_trimmed, min_n = trim_to_min_samples(raw_series)
    aligned_trimmed, _ = trim_to_min_samples(aligned_series)
    print(f"对齐后使用最短样本数: {min_n}")

    out_dir = make_output_dir(OUTPUT_DIR)
    fig1 = out_dir / "figure_raw_rms.png"
    fig2 = out_dir / "figure_aligned_rms.png"

    plot_curves(
        raw_trimmed,
        title=f"Raw RMS Per Time (tail-aligned, N={min_n})",
        ylabel="RMS [deg]",
        save_path=fig1,
    )
    plot_curves(
        aligned_trimmed,
        title=f"Aligned RMS Per Time (tail-aligned, N={min_n})",
        ylabel="RMS [deg]",
        save_path=fig2,
    )

    print(f"输出目录: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
