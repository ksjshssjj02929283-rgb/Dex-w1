#!/usr/bin/env python3
"""分析实测 FK DAT，保存图表、逐点误差 DAT 和统计 JSON；不连接 ROS。

python3 script/analyze_trajectory_points.py script/pos_err_test/fk_measured_left_*.dat
python3 script/analyze_trajectory_points.py left.dat right.dat --show

默认输出 script/pos_err_test/analysis/<原文件名>_<分析时间>/，不覆盖历史结果。
图中关节/姿态单位 deg，位置误差 mm；位姿四元数按 wxyz 归一化，q 与 -q 等价。
三种对照：server-desired（总到位误差）、FK-desired（关节到位对 FK 的影响）、
server-FK（服务器与保存的 Pinocchio FK 一致性）。FK 使用采集时保存值，不重新运行。
服务器末端反馈不是独立外部测量，不能据此认定真机几何精度。
"""

import argparse
from datetime import datetime
import json
from pathlib import Path
import sys

import numpy as np


MAGIC = "# trajectory_fk_measurements_v1 "
OUTPUT_DIR = Path(__file__).resolve().parent / "pos_err_test/analysis"
FIELDS = ("q_command", "q_reached", "pose_desired", "pose_reached", "pose_reached_fk")
PAIRS = (
    ("server_minus_desired", "Server - desired", "pose_reached", "pose_desired"),
    ("fk_minus_desired", "FK - desired", "pose_reached_fk", "pose_desired"),
    ("server_minus_fk", "Server - FK", "pose_reached", "pose_reached_fk"),
)


def load_measurements(path):
    path = Path(path)
    with path.open(encoding="utf-8") as stream:
        first, second = stream.readline(), stream.readline()
        if not first.startswith(MAGIC) or not second.startswith("# "):
            raise ValueError("需要 fk_measured 实测 DAT，不是采样目标 DAT")
        meta = json.loads(first[len(MAGIC):])
        columns = second[2:].split()
        body = [line for line in stream if line.strip() and not line.lstrip().startswith("#")]
    if not body:
        raise ValueError("没有已采集的数据点")
    if len(columns) != len(set(columns)):
        raise ValueError("列名重复")
    required = ["index", "source_row", "timestamp_ns"] + [f"{field}_{i}" for field in FIELDS for i in range(7)]
    if not set(required).issubset(columns):
        raise ValueError("缺少关节角或末端位姿字段")
    table = np.loadtxt(body, ndmin=2)
    if table.shape[1] != len(columns) or not np.isfinite(table).all():
        raise ValueError("数据列数不匹配或存在非有限数值")
    if (meta.get("joint_unit"), meta.get("position_unit"), meta.get("pose_order")) != ("rad", "m", "x,y,z,qw,qx,qy,qz"):
        raise ValueError("单位或四元数顺序不匹配")
    if meta.get("arm") not in ("left", "right") or type(meta.get("real")) is not bool:
        raise ValueError("需要有效的 arm 和 real 元信息")
    if meta.get("collected_points") != len(table):
        raise ValueError("collected_points 与实际行数不符")
    index = table[:, columns.index("index")]
    if np.any(index < 1) or np.any(index != np.floor(index)) or np.any(np.diff(index) <= 0):
        raise ValueError("采样点编号必须是递增正整数")
    values = {field: table[:, [columns.index(f"{field}_{i}") for i in range(7)]] for field in FIELDS}
    for field in FIELDS[2:]:
        norms = np.linalg.norm(values[field][:, 3:], axis=1)
        if np.any(np.abs(norms - 1) > 0.01):
            raise ValueError(f"{field} 含无效四元数（范数偏离 1 超过 0.01）")
    return {"path": path, "meta": meta, "index": index.astype(int), "values": values}


def orientation_error_deg(actual, reference):
    """Shortest SO(3) angle, stable near zero and invariant to quaternion sign."""
    a, b = np.asarray(actual, dtype=float), np.asarray(reference, dtype=float)
    an, bn = np.linalg.norm(a, axis=1, keepdims=True), np.linalg.norm(b, axis=1, keepdims=True)
    if np.any(an < 1e-10) or np.any(bn < 1e-10):
        raise ValueError("零四元数")
    a, b = a / an, b / bn
    rel_w = np.sum(a * b, axis=1)
    rel_v = b[:, :1] * a[:, 1:] - a[:, :1] * b[:, 1:] - np.cross(b[:, 1:], a[:, 1:])
    return np.rad2deg(2 * np.arctan2(np.linalg.norm(rel_v, axis=1), np.abs(rel_w)))


def statistics(values):
    values = np.asarray(values)
    return {"mean": np.mean(values, axis=0).tolist(),
            "mean_abs": np.mean(np.abs(values), axis=0).tolist(),
            "rmse": np.sqrt(np.mean(values ** 2, axis=0)).tolist(),
            "max_abs": np.max(np.abs(values), axis=0).tolist(),
            "p95_abs": np.percentile(np.abs(values), 95, axis=0).tolist()}


def analyze(data):
    values = data["values"]
    joint = np.rad2deg(values["q_reached"] - values["q_command"])
    residuals = {"joint_error_deg": joint}
    report = {"source": str(data["path"].resolve()), "arm": data["meta"]["arm"],
              "real": data["meta"]["real"], "status": data["meta"].get("status"),
              "count": len(joint), "fk_frame": data["meta"].get("fk_frame"), "tcp": data["meta"].get("tcp"),
              "joint_names": [f"{data['meta']['arm'].upper()}_J{i}" for i in range(1, 8)],
              "joint_error_deg": statistics(joint), "pose_errors": {},
              "note": "FK values were recorded during acquisition. Server pose is not external metrology; simulation results do not measure physical accuracy."}
    for key, label, actual, reference in PAIRS:
        position = (values[actual][:, :3] - values[reference][:, :3]) * 1000
        norm = np.linalg.norm(position, axis=1)
        angle = orientation_error_deg(values[actual][:, 3:], values[reference][:, 3:])
        residuals[key] = {"position_mm": position, "position_norm_mm": norm, "orientation_deg": angle}
        worst = int(np.argmax(norm))
        report["pose_errors"][key] = {"label": label, "xyz_mm": statistics(position),
            "position_norm_mm": statistics(norm), "orientation_deg": statistics(angle),
            "worst_position_point": int(data["index"][worst]),
            "worst_orientation_point": int(data["index"][np.argmax(angle)])}
    return report, residuals


def plot(data, residuals, folder, show=False):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    x, values = data["index"], data["values"]
    mode = "REAL" if data["meta"]["real"] else "SIM"
    title = f"{data['meta']['arm'].upper()} | {mode} | {len(x)} points | {data['meta'].get('status', 'unknown')}"
    figures = []
    with plt.rc_context({"axes.grid": True, "grid.alpha": 0.25, "font.size": 10}):
        fig, axes = plt.subplots(7, 2, figsize=(14, 16), sharex=True, layout="constrained")
        fig.suptitle("Joint targets, feedback and error — " + title)
        for j in range(7):
            axes[j, 0].plot(x, np.rad2deg(values["q_command"][:, j]), label="Command", lw=1.6)
            axes[j, 0].plot(x, np.rad2deg(values["q_reached"][:, j]), label="Reached", ls="--", marker=".", ms=3)
            axes[j, 0].set_ylabel(f"J{j + 1} (deg)")
            axes[j, 1].plot(x, residuals["joint_error_deg"][:, j], color="tab:red", marker=".", ms=3)
            axes[j, 1].axhline(0, color="gray", lw=0.7)
            axes[j, 1].set_ylabel("Reached - command (deg)")
            axes[j, 1].ticklabel_format(axis="y", style="sci", scilimits=(-3, 3), useOffset=False)
        axes[0, 0].legend()
        axes[-1, 0].set_xlabel("Sample index")
        axes[-1, 1].set_xlabel("Sample index")
        fig.savefig(folder / "joint_errors.png", dpi=160)
        figures.append(fig)

        fig, axes = plt.subplots(3, 2, figsize=(14, 11), sharex=True, layout="constrained")
        fig.suptitle("End-effector differences — " + title + "\nServer pose is not external metrology")
        for i, (key, label, _, _) in enumerate(PAIRS):
            delta = residuals[key]
            for j, component in enumerate(("dx", "dy", "dz")):
                axes[i, 0].plot(x, delta["position_mm"][:, j], label=component, lw=1)
            axes[i, 0].plot(x, delta["position_norm_mm"], label="3D norm", color="black", lw=1.5)
            axes[i, 0].set_title(label)
            axes[i, 0].set_ylabel("Position difference (mm)")
            axes[i, 0].legend(ncol=4, fontsize=8)
            axes[i, 1].plot(x, delta["orientation_deg"], color="tab:purple", marker=".", ms=3)
            axes[i, 1].set_title(label + " | shortest rotation angle")
            axes[i, 1].set_ylabel("Orientation difference (deg)")
            for ax in axes[i]:
                ax.ticklabel_format(axis="y", style="sci", scilimits=(-3, 3), useOffset=False)
        for ax in axes[-1]:
            ax.set_xlabel("Sample index")
        fig.savefig(folder / "pose_errors.png", dpi=160)
        figures.append(fig)

        fig = plt.figure(figsize=(10, 8), layout="constrained")
        ax = fig.add_subplot(111, projection="3d")
        fig.suptitle("Sampled end-effector positions — " + title)
        positions = []
        for field, label, marker in (("pose_desired", "Desired", "o"), ("pose_reached", "Server reached", "x"), ("pose_reached_fk", "FK reached", "+")):
            xyz = values[field][:, :3] * 1000
            positions.append(xyz)
            ax.scatter(*xyz.T, label=label, marker=marker, s=25, alpha=0.75)
        bounds = np.concatenate(positions)
        low, high = bounds.min(axis=0), bounds.max(axis=0)
        center, radius = (low + high) / 2, max(float(np.max(high - low)) * 0.55, 1.)
        ax.set_xlim(center[0] - radius, center[0] + radius)
        ax.set_ylim(center[1] - radius, center[1] + radius)
        ax.set_zlim(center[2] - radius, center[2] + radius)
        ax.set_box_aspect((1, 1, 1))
        ax.set(xlabel="X (mm)", ylabel="Y (mm)", zlabel="Z (mm)")
        ax.legend()
        ax.set_title("Frame: " + str(data["meta"].get("fk_frame", "unknown")) + " | samples only, not a motion path")
        fig.savefig(folder / "positions_3d.png", dpi=160)
        figures.append(fig)
    if show:
        plt.show()
    for fig in figures:
        plt.close(fig)


def save_analysis(data, root, show=False):
    report, residuals = analyze(data)
    folder = Path(root) / (data["path"].stem + "_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    folder.mkdir(parents=True, exist_ok=False)
    (folder / "summary.json").write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
    columns = ["index"] + [f"J{i}_error_deg" for i in range(1, 8)]
    parts = [data["index"][:, None], residuals["joint_error_deg"]]
    for key, _, _, _ in PAIRS:
        delta = residuals[key]
        columns += [key + "_" + component for component in ("dx_mm", "dy_mm", "dz_mm", "norm_mm", "angle_deg")]
        parts += [delta["position_mm"], delta["position_norm_mm"][:, None], delta["orientation_deg"][:, None]]
    np.savetxt(folder / "errors.dat", np.column_stack(parts), fmt="%.12g", header=" ".join(columns))
    plot(data, residuals, folder, show=show)
    print(f"\n{data['path'].name}: {report['count']} 点，{report['arm']}，{'真机' if report['real'] else '仿真'}，状态 {report['status']}")
    print(f"关节绝对误差最大值: {max(report['joint_error_deg']['max_abs']):.6g} deg")
    for key, _, _, _ in PAIRS:
        error = report["pose_errors"][key]
        print(f"{error['label']}: 位置 RMSE={error['position_norm_mm']['rmse']:.6g} mm，"
              f"最大={error['position_norm_mm']['max_abs']:.6g} mm；姿态最大={error['orientation_deg']['max_abs']:.6g} deg")
    print(f"图表及统计: {folder}")
    return folder


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("files", type=Path, nargs="+", help="实测 DAT，支持同时分析多个文件")
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR)
    parser.add_argument("--show", action="store_true", help="保存图片后打开交互绘图窗口")
    args = parser.parse_args(argv)
    try:
        datasets = [load_measurements(path) for path in args.files]
        for data in datasets:
            save_analysis(data, args.output_dir, args.show)
        return 0
    except (OSError, ValueError, KeyError, TypeError, ImportError) as exc:
        print(f"分析失败: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
