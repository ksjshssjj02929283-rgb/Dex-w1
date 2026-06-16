#!/usr/bin/python3
import os
import sys
import yaml
import numpy as np
import yourdfpy
from typing import List, Tuple, Optional

def get_robot_info() -> Tuple[Optional[str], Optional[str], Optional[str]]:
    config_path = os.path.expanduser("~/dexe_robot_server_data/robot_config.yaml")
    robot_type = None
    version = None
    urdf_p = None

    if os.path.exists(config_path):
        try:
            with open(config_path) as f:
                cfg = yaml.safe_load(f)
                robot_type = cfg.get('robot_type')
                version = cfg.get('version')
                urdf_p = cfg.get('file_path', {}).get('robot_urdf')
        except Exception:
            pass

    if not urdf_p:
        cwd = os.getcwd()
        for p in [
            os.path.join(cwd, "install/robot_server_lib/share/robot_server_lib/robot_models/w1/robot.urdf"),
            os.path.join(cwd, "robot_server_lib/robot_models/w1/robot.urdf"),
            os.path.expanduser("~/w1/dexe_robot_server/robot_server_lib/robot_models/w1/robot.urdf"),
        ]:
            if os.path.exists(p):
                urdf_p = p
                break

    return urdf_p, robot_type, version

def get_arm_urdf_path(base_urdf_path: str, arm: str) -> str:
    p = os.path.join(os.path.dirname(base_urdf_path), f"{arm}_arm.urdf")
    return p if os.path.exists(p) else base_urdf_path

def load_robot(arm: str):
    base_urdf_path, _, _ = get_robot_info()
    if not base_urdf_path or not os.path.exists(base_urdf_path):
        raise FileNotFoundError(f"URDF file not found: {base_urdf_path}")

    urdf_path = get_arm_urdf_path(base_urdf_path, arm)
    robot = yourdfpy.URDF.load(urdf_path)
    start_link = robot.base_link if urdf_path != base_urdf_path else "base_link"

    end_link = f"{arm}_ee"
    if end_link not in robot.link_map:
        for candidate in [f"{arm}_ee", "ee_link"]:
            if candidate in robot.link_map:
                end_link = candidate
                break
        else:
            for ln in robot.link_map:
                if "ee" in ln.lower():
                    end_link = ln
                    break

    joint_path = find_chain(robot, start_link, end_link)
    if not joint_path:
        raise ValueError(f"Chain not found: {start_link} -> {end_link}")

    active_joints = [jn for jn in joint_path if robot.joint_map[jn].type != 'fixed']

    end_link = robot.joint_map[active_joints[-1]].child

    print(f"{'使用手臂 URDF' if urdf_path != base_urdf_path else '使用完整机器人 URDF'}: {urdf_path}")
    print(f"start link: {start_link} -> end link: {end_link}")

    return robot, start_link, active_joints, end_link

def find_chain(robot, start_link: str, end_link: str) -> Optional[List[str]]:
    adj = {}
    for j in robot.joint_map.values():
        adj.setdefault(j.parent, []).append((j.child, j.name))

    queue = [(start_link, [])]
    visited = {start_link}
    while queue:
        curr, path = queue.pop(0)
        if curr == end_link:
            return path
        for child, jn in adj.get(curr, []):
            if child not in visited:
                visited.add(child)
                queue.append((child, path + [jn]))
    return None

def find_arm_base_link(robot, arm: str) -> str:
    side = arm[0].upper()
    for jn, j in robot.joint_map.items():
        if 'J1' in jn.upper() and side in jn.upper():
            return j.parent
    for jn, j in robot.joint_map.items():
        if 'J1' in jn.upper():
            return j.parent
    raise ValueError(f"Arm base link (parent of J1) not found for {arm}")

def normalize(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else v

def compute_axes(robot, active_joints: List[str], target_end_link: str):
    axes = []
    link_frames = []
    for jn in active_joints:
        joint = robot.joint_map[jn]
        T = robot.get_transform(joint.child)
        z = normalize(T[:3, :3] @ np.array(joint.axis))
        axes.append((z, T[:3, 3]))
        link_frames.append(T[:3, :3].copy())

    T_ee = robot.get_transform(target_end_link)
    axes.append((axes[-1][0], T_ee[:3, 3]))
    link_frames.append(T_ee[:3, :3].copy())
    return axes, link_frames

def _build_dh_base_transform(z0, x_init):
    y_dh = normalize(np.cross(z0, x_init))
    R_dh = np.column_stack([x_init, y_dh, z0])
    return R_dh

def _transform_axes_to_dh_base(axes, link_frames, R_dh, p_origin):
    local_axes = []
    local_frames = []
    for (z, p), R in zip(axes, link_frames):
        z_local = R_dh.T @ z
        p_local = R_dh.T @ (p - p_origin)
        R_local = R_dh.T @ R
        local_axes.append((z_local, p_local))
        local_frames.append(R_local)
    return local_axes, local_frames

def _transform_axes_to_dh_base(axes, link_frames, R_dh, p_origin):
    local_axes = []
    local_frames = []
    for (z, p), R in zip(axes, link_frames):
        z_local = R_dh.T @ z
        p_local = R_dh.T @ (p - p_origin)
        R_local = R_dh.T @ R
        local_axes.append((z_local, p_local))
        local_frames.append(R_local)
    return local_axes, local_frames

def get_arm_base_x_axis(robot, arm_base_link: str) -> np.ndarray:
    return normalize(robot.get_transform(arm_base_link)[:3, 0])

def _dh_pair(z_prev, p_prev, z_curr, p_curr, x_prev=None, x_ref=None):
    x_i = np.cross(z_prev, z_curr)
    if np.linalg.norm(x_i) < 1e-5:
        v = p_curr - p_prev
        x_i = v - np.dot(v, z_prev) * z_prev
        if np.linalg.norm(x_i) < 1e-5:
            if x_ref is not None:
                x_i = x_ref - np.dot(x_ref, z_prev) * z_prev
                if np.linalg.norm(x_i) < 1e-5:
                    x_i = x_ref if x_prev is None else x_prev
            elif x_prev is not None:
                x_i = x_prev
            else:
                ref = np.array([1, 0, 0]) if abs(z_prev[0]) < 0.9 else np.array([0, 1, 0])
                x_i = np.cross(z_prev, ref)
    x_i = normalize(x_i)

    if x_ref is not None and np.linalg.norm(np.cross(z_prev, z_curr)) > 1e-5:
        proj = x_ref - np.dot(x_ref, z_curr) * z_curr
        if np.linalg.norm(proj) > 1e-5 and np.dot(x_i, proj) < 0:
            x_i = -x_i

    a = -np.dot(p_prev - p_curr, x_i)
    rhs = p_curr - p_prev + a * x_i
    m = np.dot(z_prev, z_curr)
    denom = 1 - m * m
    if abs(denom) < 1e-8:
        t = np.dot(p_curr - p_prev, z_prev)
        s = 0.0
    else:
        t = (np.dot(rhs, z_prev) - m * np.dot(rhs, z_curr)) / denom
        s = (m * np.dot(rhs, z_prev) - np.dot(rhs, z_curr)) / denom

    d = t
    cos_a = np.clip(np.dot(z_prev, z_curr), -1.0, 1.0)
    sin_a = np.dot(np.cross(z_prev, z_curr), x_i)
    alpha = np.arctan2(sin_a, cos_a)

    if x_prev is not None:
        cos_t = np.clip(np.dot(x_prev, x_i), -1.0, 1.0)
        sin_t = np.dot(np.cross(x_prev, x_i), z_prev)
        theta = np.arctan2(sin_t, cos_t)
    else:
        theta = 0.0

    return a, alpha, d, theta, x_i, p_curr + s * z_curr

def compute_dh_from_axes(axes, x_init=None, link_frames=None) -> List[Tuple[float, float, float, float]]:
    x_prev = x_init
    p_prev = axes[0][1]
    params = []
    for i in range(len(axes) - 1):
        z_prev, _ = axes[i]
        z_curr, p_curr = axes[i + 1]
        x_ref = None
        if link_frames is not None and i + 1 < len(link_frames):
            x_ref = link_frames[i + 1][:, 0]
        a, alpha, d, theta, x_curr, p_new = _dh_pair(z_prev, p_prev, z_curr, p_curr, x_prev, x_ref)
        params.append((a, alpha, d, theta))
        x_prev, p_prev = x_curr, p_new
    return params

def compute_dh_params(axes, link_frames, x_init):
    z0 = axes[0][0]
    p_origin = axes[0][1]
    R_dh = _build_dh_base_transform(z0, x_init)
    local_axes, local_frames = _transform_axes_to_dh_base(axes, link_frames, R_dh, p_origin)
    x_init_local = np.array([1.0, 0.0, 0.0])
    dh_params = compute_dh_from_axes(local_axes, x_init_local, local_frames)
    return dh_params, R_dh, p_origin

def compute_tool_frame(robot, active_joints, target_end_link, dh_params, R_dh, p_origin):
    robot.update_cfg({j: 0.0 for j in active_joints})
    T_urdf = urdf_end_in_dh_base(robot, target_end_link, R_dh, p_origin)
    T_dh = dh_fk(dh_params, np.zeros(len(active_joints)))
    T_tool = np.linalg.inv(T_dh) @ T_urdf
    return T_tool

def urdf_end_in_dh_base(robot, target_end_link, R_dh, p_origin):
    T_root_to_end = robot.get_transform(target_end_link)
    T_dh_base = np.eye(4)
    T_dh_base[:3, :3] = R_dh
    T_dh_base[:3, 3] = p_origin
    T_end_in_dh_base = np.linalg.inv(T_dh_base) @ T_root_to_end
    return T_end_in_dh_base

def dh_transform(a, alpha, d, theta):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d],
        [0,   0,        0,       1],
    ])

def dh_fk(params: List[Tuple[float, float, float, float]], q: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    for i, (a, alpha, d, theta) in enumerate(params):
        T = T @ dh_transform(a, alpha, d, theta + q[i])
    return T

def format_angle(val: float) -> str:
    val = (val + np.pi) % (2 * np.pi) - np.pi
    for target, label in [(0, "0"), (np.pi / 2, "pi/2"), (-np.pi / 2, "-pi/2"), (np.pi, "pi"), (-np.pi, "pi")]:
        if abs(val - target) < 1e-4:
            return label
    return f"{val:.4f}"

def format_theta_offset(val: float) -> str:
    val = (val + np.pi) % (2 * np.pi) - np.pi
    if abs(val) < 1e-4:
        return "q"
    for target, label in [(np.pi / 2, "pi/2 + q"), (-np.pi / 2, "-pi/2 + q"), (np.pi, "pi + q"), (-np.pi, "pi + q")]:
        if abs(val - target) < 1e-4:
            return label
    sgn = "+" if val > 0 else "-"
    return f"q {sgn} {abs(val):.4f}"

def format_dist(val: float) -> float:
    if abs(val) < 1e-5:
        return 0.0
    r = round(float(val), 4)
    return 0.0 if abs(r) < 1e-5 else r

def print_dh_list(dh_params, joint_names):
    print("[")
    lines = []
    for (a, alpha, d, theta), jn in zip(dh_params, joint_names):
        alpha_s = format_angle(alpha)
        theta_s = format_theta_offset(theta)
        a_v, d_v = format_dist(a), format_dist(d)
        a_s = int(a_v) if isinstance(a_v, float) and a_v.is_integer() else a_v
        d_s = int(d_v) if isinstance(d_v, float) and d_v.is_integer() else d_v
        line = f'    ("{alpha_s}", {a_s}, {d_s}, "{theta_s}"),'
        lines.append((line, jn))
    max_len = max(len(l) for l, _ in lines)
    for line, jn in lines:
        print(f"{line}{' ' * (max_len - len(line) + 2)}# {jn}")
    print("]")

def run_verification(robot, arm, active_joints, target_end_link, dh_params, q, label, R_dh, p_origin, T_tool):
    print(f"\n正在验证 {arm.upper()} 手臂 ({label}):")
    robot.update_cfg({jn: q[i] for i, jn in enumerate(active_joints)})

    T_urdf_in_dh_base = urdf_end_in_dh_base(robot, target_end_link, R_dh, p_origin)
    T_dh = dh_fk(dh_params, q)
    T_dh_corrected = T_dh @ T_tool

    print(f"测试关节角 (rad): {np.round(q, 4).tolist()}")
    print(f"URDF 末端位姿 (DH基座坐标系):\n{np.round(T_urdf_in_dh_base, 6)}")
    print(f"DH  末端位姿 (含T_tool):\n{np.round(T_dh_corrected, 6)}")
    pos_err = np.linalg.norm(T_urdf_in_dh_base[:3, 3] - T_dh_corrected[:3, 3])
    R_err = T_urdf_in_dh_base[:3, :3].T @ T_dh_corrected[:3, :3]
    ang_err = np.degrees(np.arccos(np.clip((np.trace(R_err) - 1) / 2, -1, 1)))
    print(f"位置误差: {pos_err:.6f} m,  姿态误差: {ang_err:.4f} deg")
    print("✅ DH 模型与 URDF 完全等价。" if (pos_err < 1e-3 and ang_err < 1) else "❌ 存在偏差。")

def get_gravity_in_arm_base(arm: str) -> Tuple[float, float, float]:
    base_urdf_path, _, _ = get_robot_info()
    if not base_urdf_path or not os.path.exists(base_urdf_path):
        raise FileNotFoundError(f"URDF file not found: {base_urdf_path}")

    full_robot = yourdfpy.URDF.load(base_urdf_path)
    arm_base_link = find_arm_base_link(full_robot, arm)
    R = full_robot.get_transform(arm_base_link)[:3, :3]
    g = R.T @ np.array([0.0, 0.0, -9.81])
    return (float(g[0]), float(g[1]), float(g[2]))

def get_gravity_in_dh_base(arm: str) -> Tuple[float, float, float]:
    robot, start_link, active_joints, target_end_link = load_robot(arm)
    robot.update_cfg({j: 0.0 for j in active_joints})
    axes, link_frames = compute_axes(robot, active_joints, target_end_link)
    arm_base_link = find_arm_base_link(robot, arm)
    x_init = get_arm_base_x_axis(robot, arm_base_link)
    _, R_dh, _ = compute_dh_params(axes, link_frames, x_init)
    g_world = np.array([0.0, 0.0, -9.81])
    g_dh = R_dh.T @ g_world
    return (float(g_dh[0]), float(g_dh[1]), float(g_dh[2]))

def get_dh_base_rotation(arm: str) -> np.ndarray:
    """返回 arm base → DH base 的旋转矩阵 (3x3)。
    gravity_dh = R @ gravity_arm_base。
    find_arm_base_link 返回的 arm base 不一定是 URDF root，
    需用 R_arm_base 消除 arm base 相对 root 的固定变换。
    """
    robot, start_link, active_joints, target_end_link = load_robot(arm)
    robot.update_cfg({j: 0.0 for j in active_joints})
    axes, link_frames = compute_axes(robot, active_joints, target_end_link)
    arm_base_link = find_arm_base_link(robot, arm)
    R_arm_base = robot.get_transform(arm_base_link)[:3, :3]
    x_init = get_arm_base_x_axis(robot, arm_base_link)
    _, R_dh, _ = compute_dh_params(axes, link_frames, x_init)
    return R_dh.T @ R_arm_base

def get_dh_params_for_arm(arm: str) -> List[Tuple[str, float, float, str]]:
    robot, start_link, active_joints, target_end_link = load_robot(arm)
    robot.update_cfg({j: 0.0 for j in active_joints})
    axes, link_frames = compute_axes(robot, active_joints, target_end_link)
    arm_base_link = find_arm_base_link(robot, arm)
    x_init = get_arm_base_x_axis(robot, arm_base_link)
    dh_params, R_dh, p_origin = compute_dh_params(axes, link_frames, x_init)

    result = []
    for a, alpha, d, theta in dh_params:
        result.append((format_angle(alpha), format_dist(a), format_dist(d), format_theta_offset(theta)))
    return result

def get_gravity_in_arm_base(arm: str) -> Tuple[float, float, float]:
    base_urdf_path, _, _ = get_robot_info()
    if not base_urdf_path or not os.path.exists(base_urdf_path):
        raise FileNotFoundError(f"URDF file not found: {base_urdf_path}")

    full_robot = yourdfpy.URDF.load(base_urdf_path)
    arm_base_link = find_arm_base_link(full_robot, arm)
    R = full_robot.get_transform(arm_base_link)[:3, :3]
    g = R.T @ np.array([0.0, 0.0, -9.81])
    return (float(g[0]), float(g[1]), float(g[2]))

def get_dh_params_for_arm(arm: str) -> List[Tuple[str, float, float, str]]:
    robot, start_link, active_joints, target_end_link = load_robot(arm)
    robot.update_cfg({j: 0.0 for j in active_joints})
    axes, link_frames = compute_axes(robot, active_joints, target_end_link)
    arm_base_link = find_arm_base_link(robot, arm)
    x_init = get_arm_base_x_axis(robot, arm_base_link)
    dh_params, R_dh, p_origin = compute_dh_params(axes, link_frames, x_init)

    result = []
    for a, alpha, d, theta in dh_params:
        result.append((format_angle(alpha), format_dist(a), format_dist(d), format_theta_offset(theta)))
    return result

def main():
    target_arm = None
    if len(sys.argv) > 1 and sys.argv[1].lower() in ("left", "right"):
        target_arm = sys.argv[1].lower()

    base_urdf_path, robot_type, version = get_robot_info()
    print(f"机器人构型: {robot_type or '未知'}")
    print(f"机器人版本: {version or '未知'}")

    if not base_urdf_path or not os.path.exists(base_urdf_path):
        print(f"错误: 未找到 URDF 文件: {base_urdf_path}")
        return

    for arm in [target_arm] if target_arm else ["left", "right"]:
        print(f"\n=== {arm.upper()} 手臂 ===")
        robot, start_link, active_joints, target_end_link = load_robot(arm)

        robot.update_cfg({j: 0.0 for j in active_joints})
        axes, link_frames = compute_axes(robot, active_joints, target_end_link)
        arm_base_link = find_arm_base_link(robot, arm)
        print(f"手臂基座 link: {arm_base_link}")
        x_init = get_arm_base_x_axis(robot, arm_base_link)
        dh_params, R_dh, p_origin = compute_dh_params(axes, link_frames, x_init)

        print("\nDH 参数:")
        print_dh_list(dh_params, active_joints)

        T_tool = compute_tool_frame(robot, active_joints, target_end_link, dh_params, R_dh, p_origin)
        is_identity = np.max(np.abs(T_tool - np.eye(4))) < 1e-6
        if is_identity:
            print("工具坐标系 T_tool: 单位矩阵 (DH 帧与 URDF 帧一致)")
        else:
            print(f"工具坐标系 T_tool (DH FK 末端 → URDF 末端):\n{np.round(T_tool, 6)}")
            ang_tool = np.degrees(np.arccos(np.clip((np.trace(T_tool[:3,:3]) - 1) / 2, -1, 1)))
            print(f"  旋转角度: {ang_tool:.1f}°,  平移: {np.linalg.norm(T_tool[:3,3]):.6f} m")

        gravity = get_gravity_in_arm_base(arm)
        print(f"手臂基座重力方向 (gx, gy, gz): ({gravity[0]:.4f}, {gravity[1]:.4f}, {gravity[2]:.4f})")

        q_zero = np.array([0.0, 0.1, 0.0, 0.1, 0.0, 0.0, 0.0])
        run_verification(robot, arm, active_joints, target_end_link, dh_params, q_zero, "关节构型: [0.0, 0.1, 0.0, 0.1, 0.0, 0.0, 0.0]", R_dh, p_origin, T_tool)

        q_rand = np.random.uniform(-1.0, 1.0, len(active_joints))
        run_verification(robot, arm, active_joints, target_end_link, dh_params, q_rand, "随机关节构型", R_dh, p_origin, T_tool)

if __name__ == "__main__":
    main()
