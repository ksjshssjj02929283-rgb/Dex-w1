#!/usr/bin/python3
# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import os
import sys
import yaml
import numpy as np
import yourdfpy
from typing import List, Tuple, Optional
from IPython import embed

def get_robot_info() -> Tuple[Optional[str], Optional[str], Optional[str]]:
    """
    从 robot_config.yaml 中查找 URDF 路径和机器人信息。
    """
    config_path = os.path.expanduser("~/dexe_robot_server_data/robot_config.yaml")
    robot_type = None
    version = None
    urdf_p = None

    if os.path.exists(config_path):
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                robot_type = config.get('robot_type')
                version = config.get('version')
                if 'file_path' in config and 'robot_urdf' in config['file_path']:
                    urdf_p = config['file_path']['robot_urdf']
        except Exception:
            pass
    
    if not urdf_p:
        # 工作区备选路径
        cwd = os.getcwd()
        fallbacks = [
            os.path.join(cwd, "install/robot_server_lib/share/robot_server_lib/robot_models/w1/robot.urdf"),
            os.path.join(cwd, "robot_server_lib/robot_models/w1/robot.urdf"),
            os.path.expanduser("~/w1/dexe_robot_server/robot_server_lib/robot_models/w1/robot.urdf")
        ]
        for p in fallbacks:
            if os.path.exists(p):
                urdf_p = p
                break
    
    return urdf_p, robot_type, version

def find_chain(robot, start_link: str, end_link: str) -> Optional[List[str]]:
    """
    查找连接 start_link 和 end_link 的关节名称序列。
    """
    adj = {}
    for j in robot.joint_map.values():
        if j.parent not in adj: adj[j.parent] = []
        adj[j.parent].append((j.child, j.name))
    
    queue = [(start_link, [])]
    visited = {start_link}
    while queue:
        curr, path = queue.pop(0)
        if curr == end_link:
            return path
        for child, joint_name in adj.get(curr, []):
            if child not in visited:
                visited.add(child)
                queue.append((child, path + [joint_name]))
    return None

def normalize(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else v

def compute_dh_params(z_prev: np.ndarray, p_intersect_prev: np.ndarray, 
                      z_curr: np.ndarray, p_curr_origin: np.ndarray, 
                      x_prev: Optional[np.ndarray] = None) -> Tuple[float, float, float, float, np.ndarray, np.ndarray]:
    """
    计算两个轴之间的标准 DH 参数。
    z_prev: 上一个关节的轴向量
    p_intersect_prev: 上一个公垂线与 z_prev 的交点 (即 Frame i-1 的原点)
    z_curr: 当前关节的轴向量
    p_curr_origin: 当前关节在 URDF 中的原点位置 (用于定义 z_curr 轴线)
    x_prev: 上一个公垂线向量
    
    返回: a, alpha, d, theta, x_curr, p_intersect_curr
    """
    # 1. 计算当前公垂线 X_i
    x_i = np.cross(z_prev, z_curr)
    if np.linalg.norm(x_i) < 1e-5:
        # 轴平行或重合
        v = p_curr_origin - p_intersect_prev
        x_i = v - np.dot(v, z_prev) * z_prev
        if np.linalg.norm(x_i) < 1e-5:
            # 轴重合
            if x_prev is not None:
                x_i = x_prev
            else:
                ref = np.array([1, 0, 0]) if abs(z_prev[0]) < 0.9 else np.array([0, 1, 0])
                x_i = np.cross(z_prev, ref)
    x_i = normalize(x_i)
    
    # 2. 计算交点
    # 寻求 t 和 s 使得 p_intersect_prev + t*z_prev + a*x_i = p_curr_origin + s*z_curr
    # 这是一个射线间最短距离问题
    # 令 w = p_intersect_prev - p_curr_origin
    # t*z_prev - s*z_curr + a*x_i + w = 0
    
    # 由于 x_i 与 z_prev, z_curr 都垂直，点积得：
    a = -np.dot(p_intersect_prev - p_curr_origin, x_i)
    
    # 求解 t (offset along z_prev)
    # t*z_prev - s*z_curr = -w - a*x_i
    rhs = p_curr_origin - p_intersect_prev + a * x_i
    m = np.dot(z_prev, z_curr)
    denom = 1 - m*m
    if abs(denom) < 1e-8:
        t = np.dot(p_curr_origin - p_intersect_prev, z_prev)
        s = 0.0
    else:
        t = (np.dot(rhs, z_prev) - m * np.dot(rhs, z_curr)) / denom
        s = (m * np.dot(rhs, z_prev) - np.dot(rhs, z_curr)) / denom
    
    # p_intersect_curr (交点在 z_curr 上)
    p_h = p_intersect_prev + t * z_prev  # z_prev 上的公垂线足
    p_k = p_curr_origin + s * z_curr      # z_curr 上的公垂线足 (Frame i 的原点)
    
    d = t
    
    # 3. 计算 alpha 和 theta
    # alpha 是 z_prev 到 z_curr 绕 x_i 的转角
    cos_alpha = np.clip(np.dot(z_prev, z_curr), -1.0, 1.0)
    sin_alpha = np.dot(np.cross(z_prev, z_curr), x_i)
    alpha = np.arctan2(sin_alpha, cos_alpha)
    
    if x_prev is not None:
        cos_theta = np.clip(np.dot(x_prev, x_i), -1.0, 1.0)
        sin_theta = np.dot(np.cross(x_prev, x_i), z_prev)
        theta = np.arctan2(sin_theta, cos_theta)
    else:
        theta = 0.0
        
    return a, alpha, d, theta, x_i, p_k

def dh_transform_matrix(a, alpha, d, theta):
    """从坐标系 i-1 到 i 的标准 DH 变换矩阵"""
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d],
        [0,   0,        0,       1]
    ])

def main():
    # 仅支持选择手臂：left, right 或省略（两者）
    target_arm = None
    if len(sys.argv) > 1:
        if sys.argv[1].lower() in ["left", "right"]:
            target_arm = sys.argv[1].lower()

    arms_to_process = [target_arm] if target_arm else ["left", "right"]

    base_urdf_path, robot_type, version = get_robot_info()
    
    print("\n" + "*"*50)
    print(f"机器人构型: {robot_type if robot_type else '未知'}")
    print(f"机器人版本: {version if version else '未知'}")
    print("*"*50)

    if not base_urdf_path or not os.path.exists(base_urdf_path):
        print(f"错误: 未找到 URDF 文件。尝试路径为: {base_urdf_path}")
        return

    for arm in arms_to_process:
        print(f"\n" + "="*50)
        print(f"正在处理 {arm.upper()} 手臂")
        print("="*50)
        
        urdf_path = base_urdf_path
        start_link = "base_link" # 全机器人的默认起点
        end_link = f"{arm}_ee"

        arm_filename = f"{arm}_arm.urdf"
        arm_urdf_path = os.path.join(os.path.dirname(base_urdf_path), arm_filename)
        
        if os.path.exists(arm_urdf_path):
            print(f"使用手臂专用 URDF: {arm_urdf_path}")
            urdf_path = arm_urdf_path
            robot = yourdfpy.URDF.load(urdf_path)
            start_link = robot.base_link
        else:
            print(f"使用完整机器人 URDF: {urdf_path}")
            robot = yourdfpy.URDF.load(urdf_path)

        joint_path = find_chain(robot, start_link, end_link)
        if not joint_path:
            print(f"错误: 未找到从 {start_link} 到 {end_link} 的链路")
            continue
        
        active_joints = [jn for jn in joint_path if robot.joint_map[jn].type != 'fixed']
        
        # ========== 取消末端 eelink，只计算到最后一个旋转关节 ==========
        end_link = robot.joint_map[active_joints[-1]].child
        idx = joint_path.index(active_joints[-1])
        joint_path = joint_path[:idx+1]
        
        print(f"链路: {start_link} -> {' -> '.join(joint_path)} -> {end_link} (已忽略固定末端)")
        # ==========================================================

        robot.update_cfg({j: 0.0 for j in active_joints})
        
        axes = [] 
        for jn in active_joints:
            joint = robot.joint_map[jn]
            T_base_parent = robot.get_transform(joint.parent)
            z_base = normalize(T_base_parent[:3, :3] @ np.array(joint.axis))
            p_base = (T_base_parent @ np.append(np.array(joint.origin[:3, 3]), 1.0))[:3]
            axes.append((z_base, p_base))
        
        # 复制最后一个旋转关节的轴和位置作为虚拟解析终点
        # 这确保最后一组参数只有在存在沿轴向平移时产生 d 偏置，且 a=0, alpha=0
        axes.append((axes[-1][0], axes[-1][1]))

        print(f"\nDH 参数 (Standard 标准约定):")
        print(f"{'Joint':<25} | {'a':>8} | {'alpha':>8} | {'d':>8} | {'theta':>8}")
        print("-" * 75)

        x_prev = None
        p_intersect_prev = axes[0][1] # 一开始位于 J1 原点
        dh_params_list = []
        for i in range(len(active_joints)):
            z_i_minus_1, _ = axes[i]
            z_i, p_i_origin = axes[i+1]
            a, alpha, d, theta, x_curr, p_intersect_curr = compute_dh_params(z_i_minus_1, p_intersect_prev, z_i, p_i_origin, x_prev)
            print(f"{active_joints[i]:<25} | {a:8.4f} | {alpha:8.4f} | {d:8.4f} | {theta:8.4f}")
            # DEBUG
            # if active_joints[i] == "WRIST_ROLL_L_J7":
            #    print(f"DEBUG J7: p_intersect_prev={p_intersect_prev}, p_i_origin={p_i_origin}, z_prev={z_i_minus_1}, z_curr={z_i}")
            dh_params_list.append((a, alpha, d, theta))
            x_prev = x_curr
            p_intersect_prev = p_intersect_curr

        print(f"\nPython 列表格式 (用于 SymPyBotics):")
        print("[")
        for i, (a, alpha, d, theta) in enumerate(dh_params_list):
            print(f"    ({alpha:8.4f}, {a:8.4f}, {d:8.4f}, {theta:8.4f}),  # {active_joints[i]}")
        print("]")

        # 验证环节
        print(f"\n正在验证 {arm.upper()} 手臂 (全 0 关节构型):")
        q_test = np.zeros(len(active_joints))
        
        # 1. URDF 正向运动学
        robot.update_cfg({active_joints[i]: q_test[i] for i in range(len(active_joints))})
        
        # URDF 在世界坐标系中的变换
        T_world_to_end_urdf = robot.get_transform(end_link)
        pos_urdf_world = T_world_to_end_urdf[:3, 3]

        # 2. DH 正向运动学
        T_dh_chain = np.eye(4)
        for i in range(len(active_joints)):
            a, alpha, d, theta = dh_params_list[i]
            T_i = dh_transform_matrix(a, alpha, d, theta + q_test[i])
            T_dh_chain = T_dh_chain @ T_i
            
        # 3. 构造 DH Frame 0 到 URDF 世界坐标系的转换矩阵 R_0
        # DH Frame 0: 原点 p_0, Z轴 = z_0, X轴 = x_1 (因为起始 theta 偏差假设为0)
        z_0 = axes[0][0]
        p_0 = axes[0][1]
        z_1 = axes[1][0]
        p_1_orig = axes[1][1]
        _, _, _, _, x_1, _ = compute_dh_params(z_0, p_0, z_1, p_1_orig, None)
        
        X_0 = x_1
        Z_0 = z_0
        Y_0 = np.cross(Z_0, X_0)
        
        T_world_0 = np.eye(4)
        T_world_0[:3, 0] = X_0
        T_world_0[:3, 1] = Y_0
        T_world_0[:3, 2] = Z_0
        T_world_0[:3, 3] = p_0
        
        # 将 DH 结果转换到 URDF 统一坐标系下
        T_world_end_dh = T_world_0 @ T_dh_chain
        pos_dh_world = T_world_end_dh[:3, 3]
        
        dist_error = np.linalg.norm(pos_urdf_world - pos_dh_world)
        
        print(f"测试关节角 (rad): {[f'{v:.3f}' for v in q_test]}")
        print(f"URDF 世界坐标位置: {pos_urdf_world}")
        print(f"DH转换 世界坐标位置: {pos_dh_world}")
        print(f"全局空间坐标对齐误差: {dist_error:.6f} m")
        
        if dist_error < 1e-3:
            print("✅ 连杆几何验证通过：坐标完全对齐，DH 模型在物理空间中与 URDF 完全等价。")
        else:
            print("❌ 连杆几何验证失败：虽然长度可能一致，但在三维空间中的指向存在偏差。")
            print("  可能原因：起始 Frame 0 与 URDF 的起始姿态未正确对齐。")

if __name__ == "__main__":
    main()
