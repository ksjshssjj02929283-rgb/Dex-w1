import argparse
import json
import re
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker


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
        default=1000,  # 修改为1000个点
        help="Number of valid points to keep from the end (default: 1000).",
    )
    args = parser.parse_args()

    log_path = Path(args.path).expanduser()
    if not log_path.exists():
        print(f"Log file not found: {log_path}")
        return

    # 用 deque 仅保留最后 N 条"有效"样本
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

    # 将位置从弧度转换为度（便于展示）
    pos_deg = np.degrees(pos)
    pos_cmd_deg = np.degrees(pos_cmd)

    # 误差（仅在双方都有数据时才会得到数值，否则为 NaN），单位为度
    pos_err_deg = pos_cmd_deg - pos_deg
    tor_err = tor_ff - tor
    # 速度，原始为 rad/s -> 转为 deg/s 以便显示
    vel = build_array("joint_velocity")
    vel_deg = np.degrees(vel)

    # 设置matplotlib参数以最大化显示
    plt.rcParams['figure.max_open_warning'] = 100  # 允许更多图形
    plt.rcParams['figure.autolayout'] = False  # 手动调整布局
    
    # 每个手臂一张大图，每张图包含7个关节（每个关节一列，4行）
    def plot_arm(arm_name, joint_indices, arm_side):
        rows = 4  # position, torque, pos_error, tor_error
        cols = len(joint_indices)  # 7个关节
        # 将整张大图分辨率设置为 1920x1080 像素（每张大图）
        # figsize 单位为英寸，dpi*figsize = 像素。这里使用 dpi=100 -> figsize=(19.2, 10.8)
        fig, axes = plt.subplots(rows, cols, figsize=(19.2, 10.8), dpi=100, sharex=True)
        
        # 将所有坐标轴刻度格式化为小数点后3位
        for ax in axes.flatten():
            ax.xaxis.set_major_formatter(mticker.FormatStrFormatter('%.3f'))
            ax.yaxis.set_major_formatter(mticker.FormatStrFormatter('%.3f'))
        # 最大化显示
        mng = plt.get_current_fig_manager()
        try:
            mng.window.state('zoomed')  # Windows
        except:
            try:
                mng.window.showMaximized()  # Linux
            except:
                try:
                    mng.frame.Maximize(True)  # macOS
                except:
                    pass

        for ci, j in enumerate(joint_indices):
            joint_num = ci + 1
            col_title = f"{arm_side} J{joint_num}"

            # 1) position（度）
            ax = axes[0, ci]
            ax.plot(times, pos_deg[:, j], label="position (deg)", color="tab:blue", linewidth=1.2)
            if np.isfinite(pos_cmd_deg[:, j]).any():
                ax.plot(times, pos_cmd_deg[:, j], label="position_cmd (deg)", 
                       color="tab:green", linewidth=1.2, linestyle="--")
            ax.grid(True, alpha=0.3, linestyle=':')
            ax.set_title(col_title, fontsize=12)
            if ci == 0:
                ax.set_ylabel("Position [rad]", fontsize=11)
                ax.legend(loc="best", fontsize=9)
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
                    min_range = 0.2
                    if prank < min_range:
                        center = 0.5 * (pmax + pmin)
                        half = 0.5 * min_range
                        ax.set_ylim(center - half, center + half)
            except Exception:
                pass

            # 2) torque
            ax = axes[1, ci]
            ax.plot(times, tor[:, j], label="torque", color="tab:red", linewidth=1.2)
            if np.isfinite(tor_ff[:, j]).any():
                ax.plot(times, tor_ff[:, j], label="torque_cmd", 
                       color="tab:orange", linewidth=1.2, linestyle="--")
            ax.grid(True, alpha=0.3, linestyle=':')
            if ci == 0:
                ax.set_ylabel("Torque [Nm]", fontsize=11)
                ax.legend(loc="best", fontsize=9)
            # 强制 torque 纵轴范围至少为 1.0（对已有数据有效）
            try:
                t_vals = np.hstack([
                    tor[:, j][np.isfinite(tor[:, j])],
                    tor_ff[:, j][np.isfinite(tor_ff[:, j])]
                ])
                if t_vals.size > 0:
                    tmin = t_vals.min()
                    tmax = t_vals.max()
                    trange = tmax - tmin
                    min_trange = 0.2
                    if trange < min_trange:
                        center = 0.5 * (tmax + tmin)
                        half = 0.5 * min_trange
                        ax.set_ylim(center - half, center + half)
            except Exception:
                pass

            # 3) position error（度）
            ax = axes[2, ci]
            if np.isfinite(pos_err_deg[:, j]).any():
                ax.plot(times, pos_err_deg[:, j], label="pos_error (deg)", 
                       color="tab:purple", linewidth=1.2)
                if ci == 0:
                    ax.legend(loc="best", fontsize=9)
            else:
                ax.text(0.5, 0.5, "No position error data", 
                       transform=ax.transAxes, ha="center", va="center", 
                       alpha=0.6, fontsize=10)
            ax.grid(True, alpha=0.3, linestyle=':')
            if ci == 0:
                ax.set_ylabel("Pos Error [deg]", fontsize=11)

            # 4) velocity（deg/s）  — 用速度替换力矩偏差图，单位改为度/秒
            ax = axes[3, ci]
            if np.isfinite(vel_deg[:, j]).any():
                ax.plot(times, vel_deg[:, j], label="velocity (deg/s)", 
                       color="tab:brown", linewidth=1.2)
                if ci == 0:
                    ax.legend(loc="best", fontsize=9)
            else:
                ax.text(0.5, 0.5, "No velocity data", 
                       transform=ax.transAxes, ha="center", va="center", 
                       alpha=0.6, fontsize=10)
            ax.grid(True, alpha=0.3, linestyle=':')
            ax.set_xlabel("Time [s]", fontsize=11)
            # 强制 velocity 纵轴范围至少为 0.02 rad/s（对已有数据有效）
            try:
                v_vals = vel_deg[:, j][np.isfinite(vel_deg[:, j])]
                if v_vals.size > 0:
                    vmin = v_vals.min()
                    vmax = v_vals.max()
                    vrange = vmax - vmin
                    # 保持当前数值门限不变（单位现在为 deg/s）
                    min_vrange = 0.2
                    if vrange < min_vrange:
                        center = 0.5 * (vmax + vmin)
                        half = 0.5 * min_vrange
                        ax.set_ylim(center - half, center + half)
            except Exception:
                pass
            if ci == 0:
                ax.set_ylabel("Velocity [deg/s]", fontsize=11)

        # 调整布局，为标题留出更多空间，并进一步增大子图间的水平/垂直间距
        fig.subplots_adjust(top=0.96, bottom=0.03, left=0.05, right=0.96,
                     hspace=0.1, wspace=0.4)
        return fig

    # 左臂：关节索引 0-6
    left_joints = list(range(7))
    # 右臂：关节索引 7-13
    right_joints = list(range(7, 14))

    figs = []
    figs.append(plot_arm("Left Arm", left_joints, "Left"))
    figs.append(plot_arm("Right Arm", right_joints, "Right"))

    # 一次性显示
    plt.show()


if __name__ == "__main__":
    main()