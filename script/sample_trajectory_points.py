#!/usr/bin/env python3
"""从左右原始轨迹各选 10 点，左右分别保存关节目标及 Pinocchio FK 到 DAT。

python3 script/sample_trajectory_points.py
python3 script/sample_trajectory_points.py --traj traj2 --count 10
python3 script/sample_trajectory_points.py --traj traj2 --arm right --count 50

不连接 ROS、不运动、不计算碰撞。DAT 引用原始文件并记录哈希；回放仅执行选中的采样点。
每个文件只包含一臂，数值列：index、source_row、q1..q7、x y z qw qx qy qz。
关节 rad，位置 m；末端坐标系 arm_base -> ee * TCP，默认 TCP 单位变换。
"""

import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile

import numpy as np
import pinocchio as pin


ROOT = Path(__file__).resolve().parents[1]
OUTPUT_DIR = ROOT / "script/pos_err_test"
MAGIC = "# trajectory_fk_points_v1 "
DEFAULT_SOURCE = ROOT / "script/dyn_params_ident/trajectories/w1_v02"
# 与原 qt_dyn_params_ident 的逐臂执行姿态相同，不把两条单臂轨迹组合成双臂运动。
REST = {"left": [0, -1.2, 0, -1, 0, -0.5, -0.5], "right": [0, 1.2, 0, 1, 0, 0.5, 0.5]}
NAMES = ["ANKLE", "KNEE", "BUTTOCK", "WAIST", "NECK1", "NECK2"] + [f"{s}_J{i}" for s in ("LEFT", "RIGHT") for i in range(1, 8)]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def vector(values, length):
    result = np.asarray(values, dtype=float)
    if result.shape != (length,) or not np.isfinite(result).all():
        raise ValueError(f"需要 {length} 个有限数值")
    return result


def transform(values):
    p = vector(values, 7)
    norm = np.linalg.norm(p[3:])
    if norm < 1e-10:
        raise ValueError("四元数不能为零")
    w, x, y, z = p[3:] / norm
    return pin.SE3(pin.Quaternion(w, x, y, z).matrix(), p[:3])


class Kinematics:
    """仅加载运动学和限位，不加载碰撞网格。"""
    def __init__(self, urdf):
        self.urdf = Path(urdf).resolve(strict=True)
        self.model = pin.buildModelFromUrdf(str(self.urdf))
        self.data = self.model.createData()
        if set(self.model.names[1:]) != set(NAMES) or any(self.model.joints[i].nq != 1 for i in range(1, self.model.njoints)):
            raise ValueError("需要 W1 的 20 个单自由度关节模型")
        self.indices = {n: self.model.joints[self.model.getJointId(n)].idx_q for n in NAMES}
        self.lower = np.array([self.model.lowerPositionLimit[self.indices[n]] for n in NAMES])
        self.upper = np.array([self.model.upperPositionLimit[self.indices[n]] for n in NAMES])
        for side in ("left", "right"):
            for suffix in ("_arm_base", "_ee"):
                if not self.model.existFrame(side + suffix):
                    raise ValueError(f"缺少 FK 坐标系 {side + suffix}")

    def validate(self, q):
        q = vector(q, 20)
        if np.any(q < self.lower) or np.any(q > self.upper):
            raise ValueError("关节角超出 URDF 限位")
        return q

    def fk(self, values, side, tcp):
        values = self.validate(values)
        q = pin.neutral(self.model)
        for i, name in enumerate(NAMES):
            q[self.indices[name]] = values[i]
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        base = self.data.oMf[self.model.getFrameId(side + "_arm_base")]
        ee = self.data.oMf[self.model.getFrameId(side + "_ee")]
        T = base.inverse() * ee * transform(tcp)
        xyzw = pin.Quaternion(T.rotation).coeffs()
        return np.r_[T.translation, xyzw[3], xyzw[:3]].tolist()


def full_target(side, arm_q):
    return np.array([0.] * 6 + (list(arm_q) if side == "left" else REST["left"]) +
                    (list(arm_q) if side == "right" else REST["right"]), dtype=float)


def read_trajectory(path):
    rows = np.loadtxt(path, ndmin=2)
    if rows.ndim != 2 or rows.shape[1] != 7 or len(rows) < 2 or not np.isfinite(rows).all():
        raise ValueError(f"原轨迹必须至少两行、七列有限弧度数值: {path}")
    return rows


def write_dat(path, magic, meta, columns, rows, replace=False):
    """新目标文件拒绝覆盖；本次回放创建的结果文件可原子更新。"""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", dir=path.parent, encoding="utf-8", delete=False) as stream:
            temp = Path(stream.name)
            stream.write(magic + json.dumps(meta, ensure_ascii=False, allow_nan=False) + "\n")
            stream.write("# " + " ".join(columns) + "\n")
            for row in rows:
                stream.write(" ".join(str(int(v)) if isinstance(v, (int, np.integer)) else format(float(v), ".17g") for v in row) + "\n")
            stream.flush()
            os.fsync(stream.fileno())
        if replace:
            os.replace(temp, path)
        else:
            os.link(temp, path)
    finally:
        if temp is not None:
            temp.unlink(missing_ok=True)


def load_targets(path, expected_arm=None):
    with Path(path).open(encoding="utf-8") as stream:
        first = stream.readline()
        if not first.startswith(MAGIC):
            raise ValueError("不是 sample_trajectory_points 生成的目标 DAT")
        meta = json.loads(first[len(MAGIC):])
        table = np.loadtxt(stream, ndmin=2)
    if "arm" in meta:
        if meta["arm"] not in ("left", "right"):
            raise ValueError("目标 DAT 的 arm 无效")
        arms = [meta["arm"]]
    else:
        arms = ["left", "right"]  # Compatible with previous combined files.
    if expected_arm is not None and expected_arm not in arms:
        raise ValueError(f"文件属于 {meta['arm']} 臂，不能用 --arm {expected_arm} 回放")
    count = meta["count_per_arm"]
    if count < 2 or table.shape != (count, 1 + 15 * len(arms)) or not np.isfinite(table).all():
        raise ValueError("目标 DAT 行列数或数值无效")
    if not np.array_equal(table[:, 0], np.arange(1, count + 1)):
        raise ValueError("目标点编号无效")
    if (meta["rest"] != REST or meta["joint_unit"] != "rad" or meta["position_unit"] != "m" or
            meta["pose_order"] != "x,y,z,qw,qx,qy,qz"):
        raise ValueError("目标 DAT 单位/待机姿态不匹配")
    if digest(meta["urdf"]) != meta["urdf_sha256"]:
        raise ValueError("URDF 已改变，拒绝使用旧目标")
    robot = Kinematics(meta["urdf"])
    trajectories = {}
    for i, side in enumerate(arms):
        source = meta["sources"][side]
        if digest(source["path"]) != source["sha256"]:
            raise ValueError(f"{side} 原轨迹已改变")
        rows = read_trajectory(source["path"])
        selected = np.asarray(meta["sample_indices"][side])
        if (len(rows) != source["rows"] or selected.shape != (count,) or selected.dtype.kind not in "iu" or
                selected[0] != 0 or selected[-1] != len(rows) - 1 or np.any(np.diff(selected) <= 0)):
            raise ValueError("采样索引必须递增并覆盖原轨迹首尾")
        offset = 1 + 15 * i
        if not np.array_equal(table[:, offset], selected + 1) or not np.array_equal(table[:, offset + 1:offset + 8], rows[selected]):
            raise ValueError("目标关节角与原轨迹不一致")
        for row in rows:
            robot.validate(full_target(side, row))
        for j, idx in enumerate(selected):
            actual = transform(robot.fk(full_target(side, rows[idx]), side, meta["tcp"][side]))
            saved = transform(table[j, offset + 8:offset + 15])
            if np.linalg.norm(actual.translation - saved.translation) > 1e-8 or np.linalg.norm(pin.log3(actual.rotation.T @ saved.rotation)) > 1e-7:
                raise ValueError("DAT 末端目标与 FK 不一致")
        trajectories[side] = rows
    return meta, trajectories, robot


def sample(args):
    robot = Kinematics(args.urdf)
    trajectories, sources, indices = {}, {}, {}
    arms = ["left", "right"] if args.arm == "both" else [args.arm]
    for side in arms:
        path = (args.source / side / (args.traj + ".dat")).resolve(strict=True)
        rows = read_trajectory(path)
        if not 2 <= args.count <= len(rows):
            raise ValueError("count 必须在 2 到原轨迹行数之间")
        for row in rows:
            robot.validate(full_target(side, row))
        indices[side] = np.linspace(0, len(rows) - 1, args.count, dtype=int).tolist()
        trajectories[side] = rows
        sources[side] = {"path": str(path), "sha256": digest(path), "rows": len(rows)}
    meta = {"sources": sources, "sample_indices": indices, "count_per_arm": args.count,
            "urdf": str(robot.urdf), "urdf_sha256": digest(robot.urdf),
            "joint_unit": "rad", "position_unit": "m", "pose_order": "x,y,z,qw,qx,qy,qz",
            "tcp": {s: getattr(args, "tcp_" + s) for s in arms}, "rest": REST,
            "execution": "separate_arms_sampled_points_movejoint",
            "collision_checked": False, "measured": False}
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    outputs = {}
    for side in arms:
        if args.output is not None:
            output = args.output if len(arms) == 1 else args.output.with_name(args.output.stem + "_" + side + args.output.suffix)
        else:
            output = OUTPUT_DIR / f"w1_v02_{args.traj}_{side}_fk{args.count}_{timestamp}.dat"
        if output.exists():
            raise FileExistsError(f"输出文件已存在，保留原文件: {output}")
        outputs[side] = output
    for side in arms:
        side_meta = dict(meta, arm=side, sources={side: sources[side]}, sample_indices={side: indices[side]},
                         tcp={side: meta["tcp"][side]}, joint_names=[f"{side.upper()}_J{i}" for i in range(1, 8)])
        table = []
        columns = ["index", side + "_source_row"] + [f"{side}_q{j}" for j in range(1, 8)]
        columns += [f"{side}_{v}" for v in ("x", "y", "z", "qw", "qx", "qy", "qz")]
        for i in range(args.count):
            idx = indices[side][i]
            target = trajectories[side][idx]
            table.append([i + 1, idx + 1] + target.tolist() + robot.fk(full_target(side, target), side, meta["tcp"][side]))
        write_dat(outputs[side], MAGIC, side_meta, columns, table)
        print(f"已保存 {side} 臂 {args.count} 个目标点: {outputs[side]}")
    print("回放仅对选出的采样点逐点 MoveJoint；未连接 ROS，未计算碰撞，未产生实测数据。")
    return outputs


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--traj", choices=("traj1", "traj2"), default="traj1")
    parser.add_argument("--count", type=int, default=10)
    parser.add_argument("--arm", choices=("both", "left", "right"), default="both", help="默认分别生成左右两个文件")
    parser.add_argument("--urdf", type=Path, default=ROOT / "robot_server_lib/robot_models/w1/robot.urdf")
    parser.add_argument("--output", type=Path, help="单臂指定文件；双臂自动在文件名后追加 _left / _right")
    for side in ("left", "right"):
        parser.add_argument("--tcp-" + side, type=float, nargs=7, default=[0, 0, 0, 1, 0, 0, 0])
    try:
        sample(parser.parse_args(argv))
        return 0
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(f"采样失败: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
