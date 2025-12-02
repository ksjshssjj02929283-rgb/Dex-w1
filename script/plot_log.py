import argparse
import json
import re
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


def parse_line(line):
    """
    解析单行：
    - 提取 [] 中的时间戳
    - 提取 {} 中的 JSON
    - 仅检查数组长度是否为 14（存在的键校验），缺失键不视为格式错误
    返回: (timestamp: datetime, data: dict) 或 None
    """
    line = line.strip()
    if not line:
        return None

    # 提取时间戳
    ts_match = re.search(r"\[(.*?)\]", line)
    if not ts_match:
        return None
    ts_str = ts_match.group(1)
    try:
        # 形如 2025-12-02 15:07:32.170155
        ts = datetime.fromisoformat(ts_str)
    except Exception:
        return None

    # 提取 JSON
    brace_idx = line.find("{")
    if brace_idx == -1:
        return None
    json_str = line[brace_idx:]
    try:
        data = json.loads(json_str)
    except Exception:
        return None

    # 校验存在的数组长度是否为 14
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
            if not isinstance(arr, list) or len(arr) != 14:
                return None

    return ts, data


def main():
    parser = argparse.ArgumentParser(description="Plot last N points from robot_server log.")
    parser.add_argument(
        "--path",
        type=str,
        default="~/dexe_robot_server_data/logs/robot_server.txt",
        help="Path to robot_server log file.",
    )
    parser.add_argument(
        "--n",
        type=int,
        default=10000,
        help="Number of valid points to keep from the end (default: 10000).",
    )
    args = parser.parse_args()

    log_path = Path(args.path).expanduser()
    if not log_path.exists():
        print(f"Log file not found: {log_path}")
        return

    # 用 deque 仅保留最后 N 条“有效”样本
    samples = deque(maxlen=args.n)

    with open(log_path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            parsed = parse_line(line)
            if parsed is None:
                continue
            ts, data = parsed
            samples.append((ts, data))

    if not samples:
        print("No valid samples found.")
        return

    # 组装为时间序列（长度 = 样本条数）
    times = np.array([(s[0] - samples[0][0]).total_seconds() for s in samples], dtype=float)
    n = len(samples)

    # 为每个键准备 Nx14 的数组，缺失数据用 NaN 填充
    def build_array(key):
        arr = np.full((n, 14), np.nan, dtype=float)
        for i, (_, d) in enumerate(samples):
            if key in d:
                v = d[key]
                if isinstance(v, list) and len(v) == 14:
                    arr[i, :] = np.array(v, dtype=float)
        return arr

    pos = build_array("joint_position")
    pos_cmd = build_array("joint_position_cmd")
    tor = build_array("joint_torque")
    tor_ff = build_array("joint_torque_feedforward")

    # 单位换算：位置相关从弧度转为角度
    pos_deg = np.rad2deg(pos)
    pos_cmd_deg = np.rad2deg(pos_cmd)

    # 误差（仅在双方都有数据时才会得到数值，否则为 NaN）
    pos_err_deg = pos_cmd_deg - pos_deg
    tor_err = tor_ff - tor

    # 移除：每两个关节在一张大图的函数与调用
    def arm_side_and_idx(j):
        if j < 7:
            return "Left Arm", j + 1
        else:
            return "Right Arm", j - 6

    # 每个手臂分两张图：前3关节与后4关节；每张图布局为“每关节一列（4行）”
    def plot_group(arm_name, joint_indices):
        rows = 4                    # position, torque, pos_error, tor_error
        cols = len(joint_indices)   # 每个关节一列
        fig, axes = plt.subplots(rows, cols, figsize=(3.4 * cols, 2.6 * rows), sharex=True)
        if cols == 1:
            axes = np.array([[axes[r]] for r in range(rows)])  # 统一二维索引
        fig.suptitle(arm_name)

        # 行标题
        row_titles = ["position", "torque", "pos error", "tor error"]

        for ci, j in enumerate(joint_indices):
            arm_side, arm_idx = arm_side_and_idx(j)
            col_title = f"{arm_side} J{arm_idx}"

            # 1) position（度）
            ax = axes[0, ci]
            ax.plot(times, pos_deg[:, j], label="position (deg)", color="tab:blue", linewidth=1.0)
            if np.isfinite(pos_cmd_deg[:, j]).any():
                ax.plot(times, pos_cmd_deg[:, j], label="position_cmd (deg)", color="tab:green", linewidth=1.0, linestyle="--")
            ax.grid(True, alpha=0.3)
            ax.set_title(col_title)
            if ci == 0:
                ax.set_ylabel("position [deg]")
                ax.legend(loc="best", fontsize=8)
            else:
                # 仅首列显示图例，其他列去掉图例以减少重复
                pass

            # 2) torque
            ax = axes[1, ci]
            ax.plot(times, tor[:, j], label="torque", color="tab:red", linewidth=1.0)
            if np.isfinite(tor_ff[:, j]).any():
                ax.plot(times, tor_ff[:, j], label="torque_cmd", color="tab:orange", linewidth=1.0, linestyle="--")
            ax.grid(True, alpha=0.3)
            if ci == 0:
                ax.set_ylabel("torque [Nm]")
                ax.legend(loc="best", fontsize=8)

            # 3) position error（度）
            ax = axes[2, ci]
            if np.isfinite(pos_err_deg[:, j]).any():
                ax.plot(times, pos_err_deg[:, j], label="pos_error (deg)", color="tab:purple", linewidth=1.0)
                if ci == 0:
                    ax.legend(loc="best", fontsize=8)
            else:
                ax.text(0.5, 0.5, "No position error data", transform=ax.transAxes, ha="center", va="center", alpha=0.6)
            ax.grid(True, alpha=0.3)
            if ci == 0:
                ax.set_ylabel("pos error [deg]")

            # 4) torque error
            ax = axes[3, ci]
            if np.isfinite(tor_err[:, j]).any():
                ax.plot(times, tor_err[:, j], label="tor_error (ff - tor)", color="tab:brown", linewidth=1.0)
                if ci == 0:
                    ax.legend(loc="best", fontsize=8)
            else:
                ax.text(0.5, 0.5, "No torque error data", transform=ax.transAxes, ha="center", va="center", alpha=0.6)
            ax.grid(True, alpha=0.3)
            ax.set_xlabel("time [s]")
            if ci == 0:
                ax.set_ylabel("tor error [Nm]")

        fig.tight_layout(rect=[0, 0.03, 1, 0.97])
        return fig

    # 左臂：关节索引 0-6 -> 前3(0,1,2)，后4(3,4,5,6)
    left_front3 = [0, 1, 2]
    left_back4 = [3, 4, 5, 6]
    # 右臂：关节索引 7-13 -> 前3(7,8,9)，后4(10,11,12,13)
    right_front3 = [7, 8, 9]
    right_back4 = [10, 11, 12, 13]

    figs = []
    figs.append(plot_group("Left Arm (J1-J3)", left_front3))
    figs.append(plot_group("Left Arm (J4-J7)", left_back4))
    figs.append(plot_group("Right Arm (J1-J3)", right_front3))
    figs.append(plot_group("Right Arm (J4-J7)", right_back4))

    # 一次性显示
    plt.show()


if __name__ == "__main__":
    main()