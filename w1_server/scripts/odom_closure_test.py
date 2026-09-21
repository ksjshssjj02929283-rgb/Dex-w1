#!/usr/bin/env python3
"""
odom_closure_test.py —— 用激光回参考帧配准测里程计精度（检查点真值，前后双雷达合成）

用法（实机）：
  export ROS_DOMAIN_ID=20 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  python3 odom_closure_test.py --label ekf            # 默认前后两颗雷达合成 360°
  python3 odom_closure_test.py --scan /laser_scan/front --label ekf   # 只用一颗

流程：
  1. 机器人静止在起点，按 r  → 每颗雷达累积 N 帧，经各自外参转到 base_footprint 合成参考帧；记 odom 起点
  2. 遥控走任意路径（脚本持续累积 odom 里程 S 与累计转角 Θ）
  3. 回到起点附近停稳，按 c → 当前合成帧对参考帧做 ICP（base 系），得到真值位移
     ⇒ odom 闭合误差：距离 |e_xy|、角度 e_θ，及占 S / Θ 的百分比、每圈角误差
  4. 可多次按 c；按 q 退出并保存 CSV/JSON/点云

坐标系约定：
  - 每颗雷达的点先用静态 TF（base→laser_i）转到 base_footprint 再合成：点云刚性绑在车上，
    ICP 的结果就是 base 的运动，不需要再做杠杆臂换算
  - 不合成到 odom 系：那等于把待测量揉进真值构造里；odom 只作 ICP 初值
  - 前后雷达时间戳不同步，本工具只在静止时采集，故不需要去畸变；运动中连续真值需节点版
  - 双雷达对角安装各 ±134.5°，合成 360° 无盲区 ⇒ 回到起点时朝向任意都能配

真值性质：ICP 给检查点真值，不是激光里程计。odom 累计误差须在 ICP 收敛域内（max_corr）；
  每次配准记 fitness（内点平均距离）与内点比例；差的配准不要当真值。

离线自检：python3 odom_closure_test.py --selftest   （不需要 ROS）
"""
import argparse
import json
import math
import os
import select
import subprocess
import sys
import threading
import time
from datetime import datetime

import numpy as np

try:
    from scipy.spatial import cKDTree
except ImportError:  # pragma: no cover
    cKDTree = None


# ---------------------------------------------------------------- SE(2) 工具
def se2(x, y, th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])


def se2_inv(T):
    R = T[:2, :2]
    t = T[:2, 2]
    Ti = np.eye(3)
    Ti[:2, :2] = R.T
    Ti[:2, 2] = -R.T @ t
    return Ti


def se2_xyth(T):
    return float(T[0, 2]), float(T[1, 2]), math.atan2(T[1, 0], T[0, 0])


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def quat_yaw(q):
    # q: (x, y, z, w)
    return math.atan2(2.0 * (q[3] * q[2] + q[0] * q[1]),
                      1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]))


# ---------------------------------------------------------------- 点云工具
def scan_to_points(angle_min, angle_inc, ranges, rmin, rmax):
    r = np.asarray(ranges, dtype=np.float64)
    n = r.size
    ang = angle_min + angle_inc * np.arange(n)
    ok = np.isfinite(r) & (r > rmin) & (r < rmax)
    return np.stack([r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])], axis=1)


def voxel_down(pts, voxel):
    if pts.shape[0] == 0:
        return pts
    keys = np.floor(pts / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[np.sort(idx)]


def transform_pts(pts, T):
    return pts @ T[:2, :2].T + T[:2, 2]


def coverage_deg(pts_base, bin_deg=5.0):
    """点云绕 base 原点的方位角覆盖（度）：有点的 bin 数 × bin 宽"""
    if pts_base.shape[0] == 0:
        return 0.0
    ang = np.degrees(np.arctan2(pts_base[:, 1], pts_base[:, 0])) % 360.0
    bins = np.unique(np.floor(ang / bin_deg).astype(int))
    return float(bins.size * bin_deg)


def icp2d(src,
          dst,
          T_init,
          max_corr=0.3,
          max_iter=60,
          tol=1e-6,
          min_inliers=50):
    """点到点 2D ICP。返回 (T, info)。T 满足 dst ≈ T · src。"""
    if cKDTree is None:
        raise RuntimeError("scipy 不可用，无法建 KD-tree")
    tree = cKDTree(dst)
    T = T_init.copy()
    info = {
        "converged": False,
        "iters": 0,
        "fitness": float("nan"),
        "inlier_ratio": 0.0,
        "n_src": int(src.shape[0]),
        "n_dst": int(dst.shape[0])
    }
    step = float("inf")
    for it in range(1, max_iter + 1):
        P = transform_pts(src, T)
        d, idx = tree.query(P, distance_upper_bound=max_corr)
        m = np.isfinite(d)
        n_in = int(m.sum())
        info["iters"] = it
        info["inlier_ratio"] = n_in / max(1, src.shape[0])
        if n_in < min_inliers:
            info["fitness"] = float("nan")
            return T, info
        A = P[m]
        B = dst[idx[m]]
        ca, cb = A.mean(axis=0), B.mean(axis=0)
        H = (A - ca).T @ (B - cb)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[1, :] *= -1
            R = Vt.T @ U.T
        t = cb - R @ ca
        dT = np.eye(3)
        dT[:2, :2] = R
        dT[:2, 2] = t
        T = dT @ T
        info["fitness"] = float(d[m].mean())
        step = math.hypot(t[0], t[1]) + abs(math.atan2(R[1, 0], R[0, 0]))
        if step < tol:
            info["converged"] = True
            break
    else:
        info["converged"] = step < 1e-4  # 迭代耗尽但已基本停住也算收敛
    return T, info


# ---------------------------------------------------------------- 离线自检
def _lidar_view(wall_base, T_ext, fov_half_rad, noise, rng):
    """从 base 系墙面点生成某颗雷达看到的点（雷达系），受 FOV 限制；返回 base 系点"""
    pts_l = transform_pts(wall_base, se2_inv(T_ext))
    ang = np.arctan2(pts_l[:, 1], pts_l[:, 0])
    vis = np.abs(ang) <= fov_half_rad
    pts_l = pts_l[vis] + rng.normal(0, noise, (int(vis.sum()), 2))
    return transform_pts(pts_l, T_ext)


def selftest():
    rng = np.random.default_rng(0)
    wall = np.concatenate([
        np.stack([np.linspace(-4, 6, 800),
                  np.full(800, 3.0)], 1),
        np.stack([np.linspace(-4, 6, 800),
                  np.full(800, -2.5)], 1),
        np.stack(
            [np.full(500, 6.0), np.linspace(-2.5, 3.0, 500)], 1),
        np.stack([np.full(300, -4.0),
                  np.linspace(-2.5, 0.5, 300)], 1),
        np.stack([np.linspace(-4, -1, 300),
                  np.full(300, 0.5)], 1),
    ])
    # 本机实测外参：前 (0.171, 0.145, 45.24°)，后 (−0.179, −0.149, −135.55°)，各 ±134.5°
    exts = [
        se2(0.171, 0.145, math.radians(45.243)),
        se2(-0.179, -0.149, math.radians(-135.547))
    ]
    fov = math.radians(134.55)
    T_true = se2(0.03, -0.02, math.radians(1.5))  # 真实 base 运动（回到原点附近的残差）
    T_odom = se2(0.05, 0.01, math.radians(0.3))  # odom 以为的
    wall_cur = transform_pts(wall, se2_inv(T_true))  # 墙在"当前 base 系"下

    ok_all = True
    for name, use in (("双雷达", [0, 1]), ("仅前雷达", [0])):
        ref = voxel_down(
            np.concatenate(
                [_lidar_view(wall, exts[i], fov, 0.01, rng) for i in use]),
            0.02)
        cur = voxel_down(
            np.concatenate(
                [_lidar_view(wall_cur, exts[i], fov, 0.01, rng) for i in use]),
            0.02)
        T_B, info = icp2d(cur, ref, T_odom)
        x, y, th = se2_xyth(T_B)
        ex, ey, eth = x - 0.03, y + 0.02, math.degrees(
            wrap(th - math.radians(1.5)))
        ok = abs(ex) < 0.005 and abs(ey) < 0.005 and abs(eth) < 0.1
        ok_all &= ok
        print(
            f"[selftest][{name}] 覆盖 {coverage_deg(ref):.0f}°  ICP iters={info['iters']} conv={info['converged']} "
            f"fitness={info['fitness']*1000:.1f} mm inliers={info['inlier_ratio']*100:.0f}%  "
            f"残差 ({ex*1000:.1f} mm, {ey*1000:.1f} mm, {eth:.3f}°)  {'PASS' if ok else 'FAIL'}"
        )

    # 反例：把前雷达点云不做外参、直接当 base 系用，会错多少（杠杆臂）
    ref_l = voxel_down(
        transform_pts(_lidar_view(wall, exts[0], fov, 0.01, rng),
                      se2_inv(exts[0])), 0.02)
    cur_l = voxel_down(
        transform_pts(_lidar_view(wall_cur, exts[0], fov, 0.01, rng),
                      se2_inv(exts[0])), 0.02)
    T_L, _ = icp2d(cur_l, ref_l, se2_inv(exts[0]) @ T_odom @ exts[0])
    xl, yl, _ = se2_xyth(T_L)
    print(
        f"[selftest] 前雷达不做外参换算时 base 位姿 = ({xl:.4f}, {yl:.4f})，与真值差 {math.hypot(xl-0.03, yl+0.02)*1000:.1f} mm ← 杠杆臂"
    )

    # 反例：回到原点但朝向转了 150°，单前雷达视野重叠不足
    T_turn = se2(0.03, -0.02, math.radians(150.0))
    wall_turn = transform_pts(wall, se2_inv(T_turn))
    for name, use in (("双雷达", [0, 1]), ("仅前雷达", [0])):
        ref = voxel_down(
            np.concatenate(
                [_lidar_view(wall, exts[i], fov, 0.01, rng) for i in use]),
            0.02)
        cur = voxel_down(
            np.concatenate(
                [_lidar_view(wall_turn, exts[i], fov, 0.01, rng) for i in use]),
            0.02)
        T_B, info = icp2d(cur, ref, se2(0.05, 0.01, math.radians(149.0)))
        x, y, th = se2_xyth(T_B)
        e = math.hypot(x - 0.03, y + 0.02) * 1000
        eth = math.degrees(wrap(th - math.radians(150.0)))
        print(
            f"[selftest][朝向转 150° 回原点][{name}] inliers={info['inlier_ratio']*100:.0f}% 残差 {e:.1f} mm / {eth:.3f}°"
        )
    print("[selftest] PASS" if ok_all else "[selftest] FAIL")
    return 0 if ok_all else 1


# ---------------------------------------------------------------- ROS 节点
def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--selftest",
                    action="store_true",
                    help="离线自检 ICP + 双雷达合成，不需要 ROS")
    ap.add_argument("--scan",
                    default="/laser_scan/front,/laser_scan/rear",
                    help="激光话题，逗号分隔，可 1 颗或多颗")
    ap.add_argument("--frames",
                    default="",
                    help="对应的 frame_id 覆盖（逗号分隔），留空取消息 header")
    ap.add_argument("--odom", default="/feedback/odom")
    ap.add_argument("--base-frame", default="base_footprint")
    ap.add_argument("--label",
                    default="",
                    help="本轮标签（如 odom_mode 名）；留空则尝试 ros2 param get")
    ap.add_argument("--out-dir", default="./odom_closure_results")
    ap.add_argument("--ref-scans", type=int, default=10, help="每颗雷达累积帧数")
    ap.add_argument("--voxel", type=float, default=0.02)
    ap.add_argument("--max-corr", type=float, default=0.3)
    ap.add_argument("--range-min", type=float, default=0.15)
    ap.add_argument("--range-max", type=float, default=30.0)
    ap.add_argument("--still-v",
                    type=float,
                    default=0.01,
                    help="静止判据 |v| (m/s)")
    ap.add_argument("--still-w",
                    type=float,
                    default=0.01,
                    help="静止判据 |ω| (rad/s)")
    ap.add_argument("--still-sec", type=float, default=1.0, help="静止需持续秒数")
    ap.add_argument("--force", action="store_true", help="不检查静止")
    args = ap.parse_args()

    if args.selftest:
        sys.exit(selftest())

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from tf2_ros import Buffer, TransformListener

    topics = [t.strip() for t in args.scan.split(",") if t.strip()]
    frame_override = [f.strip() for f in args.frames.split(",")
                     ] if args.frames else []

    class ClosureTest(Node):

        def __init__(self):
            super().__init__("odom_closure_test")
            self.lock = threading.Lock()
            # 每颗雷达一个槽：frame / T_ext(base<-laser) / 累积缓冲 / 还需帧数 / 累计收到帧数
            self.lidars = {
                t: {
                    "frame":
                        (frame_override[i] if i < len(frame_override) else None
                        ),
                    "T_ext": None,
                    "buf": [],
                    "need": 0,
                    "seen": 0
                } for i, t in enumerate(topics)
            }
            self.odom_T = None
            self.v = 0.0
            self.w = 0.0
            self.still_since = None
            self.prev_T = None
            self.S = 0.0
            self.Theta = 0.0
            self.ref = None
            self.ref_T0 = None
            self.ref_S0 = 0.0
            self.ref_Th0 = 0.0
            self.results = []
            self.tfb = Buffer()
            self.tfl = TransformListener(self.tfb, self)
            qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST,
                             depth=5)
            for t in topics:
                self.create_subscription(LaserScan,
                                         t,
                                         lambda msg, t=t: self.on_scan(t, msg),
                                         qos)
            self.create_subscription(Odometry, args.odom, self.on_odom, 50)

        # --- 回调
        def on_scan(self, topic, msg):
            slot = self.lidars[topic]
            if slot["frame"] is None:
                slot["frame"] = msg.header.frame_id
            if slot["T_ext"] is None:
                slot["T_ext"] = self.lookup_ext(slot["frame"])
                if slot["T_ext"] is None:
                    return
            pts = scan_to_points(msg.angle_min, msg.angle_increment, msg.ranges,
                                 args.range_min, args.range_max)
            pts_base = transform_pts(pts, slot["T_ext"])  # 立即转到 base 系
            with self.lock:
                slot["seen"] += 1
                if slot["need"] > 0:
                    slot["buf"].append(pts_base)
                    slot["need"] -= 1

        def on_odom(self, msg):
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            T = se2(p.x, p.y, quat_yaw((q.x, q.y, q.z, q.w)))
            now = time.monotonic()
            with self.lock:
                self.v = abs(msg.twist.twist.linear.x)
                self.w = abs(msg.twist.twist.angular.z)
                if self.v < args.still_v and self.w < args.still_w:
                    if self.still_since is None:
                        self.still_since = now
                else:
                    self.still_since = None
                if self.prev_T is not None:
                    dx, dy, dth = se2_xyth(se2_inv(self.prev_T) @ T)
                    ds = math.hypot(dx, dy)
                    if ds > 1e-4:
                        self.S += ds
                    self.Theta += abs(dth)
                self.prev_T = T
                self.odom_T = T

        # --- 工具
        def lookup_ext(self, frame):
            try:
                tr = self.tfb.lookup_transform(args.base_frame, frame,
                                               rclpy.time.Time())
            except Exception:  # noqa —— TF 还没到，下一帧再试
                return None
            t = tr.transform.translation
            q = tr.transform.rotation
            T = se2(t.x, t.y, quat_yaw((q.x, q.y, q.z, q.w)))
            x, y, th = se2_xyth(T)
            print(
                f"\n  ✓ 外参 {args.base_frame}->{frame}: ({x:.3f}, {y:.3f}, {math.degrees(th):.2f}°)"
            )
            return T

        def is_still(self):
            if args.force:
                return True
            with self.lock:
                return self.still_since is not None and (
                    time.monotonic() - self.still_since) >= args.still_sec

        def collect_cloud(self, n):
            """每颗雷达累积 n 帧（已在 base 系），合成后体素降采样。返回 (cloud, 每颗帧数)。"""
            with self.lock:
                for slot in self.lidars.values():
                    slot["buf"] = []
                    slot["need"] = n
            t0 = time.monotonic()
            while True:
                with self.lock:
                    pending = [
                        t for t, s in self.lidars.items() if s["need"] > 0
                    ]
                if not pending:
                    break
                if time.monotonic() - t0 > 10.0:
                    print(f"  ⚠️ 10 s 内未攒够帧的雷达：{pending}（将只用已到的帧）")
                    break
                time.sleep(0.05)
            with self.lock:
                parts = {t: s["buf"] for t, s in self.lidars.items() if s["buf"]}
            if not parts:
                print("  ✗ 没有任何雷达数据")
                return None, {}
            cloud = voxel_down(
                np.concatenate([np.concatenate(b) for b in parts.values()]),
                args.voxel)
            return cloud, {t: len(b) for t, b in parts.items()}

        # --- 命令
        def cmd_register(self):
            if not self.is_still():
                print(
                    f"  ✗ 机器人未静止满 {args.still_sec} s（v={self.v:.3f} ω={self.w:.3f}），停稳再按 r；或加 --force"
                )
                return
            cloud, counts = self.collect_cloud(args.ref_scans)
            if cloud is None:
                return
            with self.lock:
                self.ref = cloud
                self.ref_T0 = self.odom_T.copy()
                self.ref_S0 = self.S
                self.ref_Th0 = self.Theta
            x, y, th = se2_xyth(self.ref_T0)
            print(
                f"  ✓ 参考帧已登记：{cloud.shape[0]} 点，覆盖 {coverage_deg(cloud):.0f}°，帧数 {counts}；odom 起点 ({x:.3f}, {y:.3f}, {math.degrees(th):.1f}°)"
            )
            print("    现在遥控走任意路径，回到起点附近停稳后按 c")

        def cmd_status(self):
            with self.lock:
                if self.ref_T0 is None:
                    print(
                        f"  （未登记参考帧）雷达帧数 { {t: s['seen'] for t, s in self.lidars.items()} }"
                    )
                    return
                S = self.S - self.ref_S0
                Th = self.Theta - self.ref_Th0
                d = se2_inv(self.ref_T0) @ self.odom_T
            x, y, th = se2_xyth(d)
            print(
                f"  里程 S={S:.3f} m  累计转角 Θ={math.degrees(Th):.1f}°  odom 相对起点 ({x:.3f}, {y:.3f}, {math.degrees(th):.2f}°)  静止={'是' if self.is_still() else '否'}"
            )

        def cmd_checkpoint(self):
            if self.ref is None:
                print("  ✗ 先按 r 登记参考帧")
                return
            if not self.is_still():
                print(
                    f"  ✗ 机器人未静止满 {args.still_sec} s（v={self.v:.3f} ω={self.w:.3f}）"
                )
                return
            cur, counts = self.collect_cloud(args.ref_scans)
            if cur is None:
                return
            with self.lock:
                T_odom = se2_inv(self.ref_T0) @ self.odom_T  # odom 说的 base 运动
                S = self.S - self.ref_S0
                Th = self.Theta - self.ref_Th0
            # 点云已在 base 系：ref ≈ T_true · cur，T_true 就是 base 的真实运动；odom 只作初值
            T_true, info = icp2d(cur, self.ref, T_odom, max_corr=args.max_corr)
            E = se2_inv(T_true) @ T_odom
            ox, oy, oth = se2_xyth(T_odom)
            lx, ly, lth = se2_xyth(T_true)
            ex, ey, eth = se2_xyth(E)
            exy = math.hypot(ex, ey)
            pct_xy = 100.0 * exy / S if S > 1e-6 else float("nan")
            pct_th = 100.0 * abs(eth) / Th if Th > 1e-6 else float("nan")
            deg_per_rev = math.degrees(
                abs(eth)) / (Th / (2 * math.pi)) if Th > 1e-6 else float("nan")
            idx = len(self.results) + 1
            quality = "好" if (info["converged"] and
                              info["inlier_ratio"] > 0.6 and
                              info["fitness"] < 0.03) else "⚠️ 差，慎用"
            print(f"\n  ── 检查点 #{idx}  [{args.label}] ──")
            print(
                f"  点云: {cur.shape[0]} 点, 覆盖 {coverage_deg(cur):.0f}°, 帧数 {counts}"
            )
            print(
                f"  ICP: {'收敛' if info['converged'] else '未收敛'} {info['iters']} 次, fitness={info['fitness']*1000:.1f} mm, 内点 {info['inlier_ratio']*100:.0f}%  → 配准质量：{quality}"
            )
            print(f"  累积里程 S = {S:.3f} m   累计转角 Θ = {math.degrees(Th):.1f}°")
            print(
                f"  odom 说相对起点: ({ox:+.4f}, {oy:+.4f}) m, {math.degrees(oth):+.3f}°"
            )
            print(
                f"  激光说相对起点: ({lx:+.4f}, {ly:+.4f}) m, {math.degrees(lth):+.3f}°"
            )
            print(
                f"  距离误差 |e_xy| = {exy*1000:.1f} mm  ({ex*1000:+.1f}, {ey*1000:+.1f})  = {pct_xy:.3f}% of S"
            )
            print(
                f"  角度误差  e_θ   = {math.degrees(eth):+.3f}°  = {pct_th:.3f}% of Θ  ≈ {deg_per_rev:.3f}°/圈\n"
            )
            row = {
                "time": datetime.now().isoformat(timespec="seconds"),
                "label": args.label,
                "checkpoint": idx,
                "S_m": S,
                "Theta_deg": math.degrees(Th),
                "odom_dx": ox,
                "odom_dy": oy,
                "odom_dth_deg": math.degrees(oth),
                "laser_dx": lx,
                "laser_dy": ly,
                "laser_dth_deg": math.degrees(lth),
                "err_x": ex,
                "err_y": ey,
                "err_xy_m": exy,
                "err_xy_pct_S": pct_xy,
                "err_th_deg": math.degrees(eth),
                "err_th_pct_Theta": pct_th,
                "err_th_deg_per_rev": deg_per_rev,
                "icp_fitness_m": info["fitness"],
                "icp_inlier_ratio": info["inlier_ratio"],
                "icp_iters": info["iters"],
                "icp_converged": info["converged"],
                "coverage_deg": coverage_deg(cur),
                "n_ref": int(self.ref.shape[0]),
                "n_cur": int(cur.shape[0]),
                "frames": json.dumps(counts),
            }
            self.results.append(row)
            os.makedirs(args.out_dir, exist_ok=True)
            exts = {
                t: s["T_ext"]
                for t, s in self.lidars.items()
                if s["T_ext"] is not None
            }
            np.savez_compressed(os.path.join(
                args.out_dir, f"cp{idx:02d}_{args.label or 'run'}.npz"),
                                ref=self.ref,
                                cur=cur,
                                T_odom=T_odom,
                                T_true=T_true,
                                lidar_topics=np.array(list(exts.keys())),
                                lidar_T_ext=np.array(list(exts.values())))

        def save(self):
            if not self.results:
                return
            os.makedirs(args.out_dir, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            csv = os.path.join(args.out_dir,
                               f"closure_{stamp}_{args.label or 'run'}.csv")
            keys = list(self.results[0].keys())
            with open(csv, "w") as f:
                f.write(",".join(keys) + "\n")
                for r in self.results:
                    f.write(",".join(
                        '"%s"' % r[k] if k == "frames" else str(r[k])
                        for k in keys) + "\n")
            with open(csv.replace(".csv", ".json"), "w") as f:
                json.dump({
                    "args": vars(args),
                    "results": self.results
                },
                          f,
                          ensure_ascii=False,
                          indent=1)
            print(f"  ✓ 已保存 {csv}（{len(self.results)} 个检查点）")

    if not args.label:
        try:
            out = subprocess.run(
                ["ros2", "param", "get", "/chassis_node", "odom_mode"],
                capture_output=True,
                text=True,
                timeout=5).stdout
            if "value is:" in out:
                args.label = out.split("value is:")[-1].strip()
        except Exception:  # noqa
            pass

    rclpy.init()
    node = ClosureTest()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    print("=" * 64)
    print(
        f" odom 闭合精度测试   scan={topics}  odom={args.odom}  label={args.label or '(无)'}"
    )
    print("   r = 登记参考帧（起点，静止）   c = 检查点（回到起点附近，静止）")
    print("   s = 当前里程/转角              q = 退出并保存")
    print("=" * 64)
    try:
        while rclpy.ok():
            # sys.stdout.write("> ")
            sys.stdout.flush()
            rl, _, _ = select.select([sys.stdin], [], [], 0.5)
            if not rl:
                continue
            cmd = sys.stdin.readline().strip().lower()
            if cmd == "r":
                node.cmd_register()
            elif cmd == "c":
                node.cmd_checkpoint()
            elif cmd == "s":
                node.cmd_status()
            elif cmd == "q":
                break
            elif cmd:
                print("  r / c / s / q")
    except KeyboardInterrupt:
        pass
    node.save()
    # 先 shutdown 让 spin() 返回，再 join 线程：否则解释器退出时 rclpy 的 C++ 线程仍在运行，
    # 会触发 "terminate called without an active exception" 并 core dump
    rclpy.shutdown()
    spinner.join(timeout=3.0)
    node.destroy_node()


if __name__ == "__main__":
    main()
