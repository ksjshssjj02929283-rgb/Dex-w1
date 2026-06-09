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

    # end_link 改为最后一个活动关节的子 link，忽略后续固定的工具变换
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
    for jn in active_joints:
        joint = robot.joint_map[jn]
        T = robot.get_transform(joint.child)
        z = normalize(T[:3, :3] @ np.array(joint.axis))
        axes.append((z, T[:3, 3]))

    T_ee = robot.get_transform(target_end_link)
    axes.append((axes[-1][0], T_ee[:3, 3]))
    return axes

def compute_dh_from_axes(axes, x_init=None) -> List[Tuple[float, float, float, float]]:
    x_prev = x_init
    p_prev = axes[0][1]
    params = []
    for i in range(len(axes) - 1):
        z_prev, _ = axes[i]
        z_curr, p_curr = axes[i + 1]
        a, alpha, d, theta, x_curr, p_new = _dh_pair(z_prev, p_prev, z_curr, p_curr, x_prev)
        params.append((a, alpha, d, theta))
        x_prev, p_prev = x_curr, p_new
    return params

def correct_dh_orientation(robot, active_joints, target_end_link, arm_base_link,
                           dh_params) -> List[Tuple[float, float, float, float]]:
    robot.update_cfg({j: 0.0 for j in active_joints})
    T_urdf = robot.get_transform(target_end_link, arm_base_link)
    T_dh = dh_fk(dh_params, np.zeros(len(active_joints)))

    pos_err = np.linalg.norm(T_urdf[:3, 3] - T_dh[:3, 3])
    R_err = T_urdf[:3, :3].T @ T_dh[:3, :3]
    ang_err = np.degrees(np.arccos(np.clip((np.trace(R_err) - 1) / 2, -1, 1)))

    if pos_err < 1e-3 and abs(ang_err - 180) < 1.0:
        params = list(dh_params)
        a, alpha, d, theta = params[-1]
        params[-1] = (a, alpha, d, theta + np.pi)
        return params
    return dh_params

def get_arm_base_x_axis(robot, arm_base_link: str) -> np.ndarray:
    return normalize(robot.get_transform(arm_base_link)[:3, 0])

def _dh_pair(z_prev, p_prev, z_curr, p_curr, x_prev=None):
    x_i = np.cross(z_prev, z_curr)
    if np.linalg.norm(x_i) < 1e-5:
        v = p_curr - p_prev
        x_i = v - np.dot(v, z_prev) * z_prev
        if np.linalg.norm(x_i) < 1e-5:
            if x_prev is not None:
                x_i = x_prev
            else:
                ref = np.array([1, 0, 0]) if abs(z_prev[0]) < 0.9 else np.array([0, 1, 0])
                x_i = np.cross(z_prev, ref)
    x_i = normalize(x_i)

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

def run_verification(robot, arm, active_joints, target_end_link, arm_base_link, dh_params, q, label):
    print(f"\n正在验证 {arm.upper()} 手臂 ({label}):")
    robot.update_cfg({jn: q[i] for i, jn in enumerate(active_joints)})

    T_urdf = robot.get_transform(target_end_link, arm_base_link)
    T_dh = dh_fk(dh_params, q)

    print(f"测试关节角 (rad): {np.round(q, 4).tolist()}")
    print(f"URDF 末端位姿:\n{np.round(T_urdf, 6)}")
    print(f"DH  末端位姿:\n{np.round(T_dh, 6)}")
    pos_err = np.linalg.norm(T_urdf[:3, 3] - T_dh[:3, 3])
    R_err = T_urdf[:3, :3].T @ T_dh[:3, :3]
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

def get_dh_params_for_arm(arm: str) -> List[Tuple[str, float, float, str]]:
    robot, start_link, active_joints, target_end_link = load_robot(arm)
    robot.update_cfg({j: 0.0 for j in active_joints})
    axes = compute_axes(robot, active_joints, target_end_link)
    arm_base_link = find_arm_base_link(robot, arm)
    x_init = get_arm_base_x_axis(robot, arm_base_link)
    dh_params = compute_dh_from_axes(axes, x_init)
    dh_params = correct_dh_orientation(robot, active_joints, target_end_link,
                                       arm_base_link, dh_params)

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
        axes = compute_axes(robot, active_joints, target_end_link)
        arm_base_link = find_arm_base_link(robot, arm)
        print(f"手臂基座 link: {arm_base_link}")
        x_init = get_arm_base_x_axis(robot, arm_base_link)
        dh_params = compute_dh_from_axes(axes, x_init)
        dh_params = correct_dh_orientation(robot, active_joints, target_end_link,
                                           arm_base_link, dh_params)

        print("\nDH 参数:")
        print_dh_list(dh_params, active_joints)

        gravity = get_gravity_in_arm_base(arm)
        print(f"手臂基座重力方向 (gx, gy, gz): ({gravity[0]:.4f}, {gravity[1]:.4f}, {gravity[2]:.4f})")

        q_zero = np.array([0.0, 0.1, 0.0, 0.1, 0.0, 0.0, 0.0])
        run_verification(robot, arm, active_joints, target_end_link, arm_base_link, dh_params, q_zero, "关节构型: [0.0, 0.1, 0.0, 0.1, 0.0, 0.0, 0.0]")

        q_rand = np.random.uniform(-1.0, 1.0, len(active_joints))
        run_verification(robot, arm, active_joints, target_end_link, arm_base_link, dh_params, q_rand, "随机关节构型")

if __name__ == "__main__":
    main()
