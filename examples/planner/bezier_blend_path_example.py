# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R

from dexe_robot_client.w1 import PathType, PathRnBezier, PathR3xSO3Bezier


def plot_SE3(ax, SE3_, axis_len=0.05):
    # 位置 p
    p = SE3_[:3, 3]
    # 旋转 R
    R = SE3_[:3, :3]
    # 坐标轴方向（x,y,z）
    x_axis = R[:, 0] * axis_len
    y_axis = R[:, 1] * axis_len
    z_axis = R[:, 2] * axis_len
    # 绘制三条轴
    ax.quiver(*p, *x_axis, color='r', linewidth=1)
    ax.quiver(*p, *y_axis, color='g', linewidth=1)
    ax.quiver(*p, *z_axis, color='b', linewidth=1)


def R7_blend_path():
    waypoints = np.array([[0.09, -0.79, -0.09, -1.57, -0., -0.26, 0.],
                          [-0.49, -1.07, 0.11, -2.16, 0.37, 0.74, -0.73],
                          [-0.95, -1.3, 0.8, -1.73, 0.56, 0.01, -0.62],
                          [-0.45, -1.3, 0.94, -1.23, 0.05, -0.02, -0.05],
                          [0.09, -1.44, 0.27, -1.11, 0.05, -0.39, 0.06]])

    path = PathRnBezier(waypoints)
    if not path.calc_path(PathType.Bezier5thJoint, 0.1):
        exit(-1)

    from dexe_rp import DexforceW1Kinematics
    w1_kine = DexforceW1Kinematics(is_anthropmorphic=True)
    left_arm_kine = w1_kine.left_arm_kine
    SE3_waypoints = np.array([left_arm_kine.get_fk(q)[1] for q in waypoints])

    s_vals = np.linspace(0, path.get_length(), 500)
    q_vals = np.array([path.get_config(s) for s in s_vals])
    dq_vals = np.array([path.get_tangent(s) for s in s_vals])
    ddq_vals = np.array([path.get_curvature(s) for s in s_vals])
    dddq_vals = np.array([path.get_torsion(s) for s in s_vals])
    SE3_vals = np.array([left_arm_kine.get_fk(q)[1] for q in q_vals])

    fig = plt.figure(figsize=(18, 6))
    ax3d = fig.add_subplot(1, 3, 1, projection='3d')
    for SE3 in SE3_waypoints:
        plot_SE3(ax3d, SE3, axis_len=0.05)
    for SE3 in SE3_vals[::5]:
        plot_SE3(ax3d, SE3, axis_len=0.01)
    ax3d.set_box_aspect([1, 1, 1])
    ax3d.set_title("3D Path")
    ax3d.set_xlim(-0.45, 0.05)
    ax3d.set_ylim(0.0, 0.5)
    ax3d.set_zlim(-0.1, 0.4)
    ax3d.set_xlabel("x")
    ax3d.set_ylabel("y")
    ax3d.set_zlabel("z")

    gs = fig.add_gridspec(1, 3)
    gs_right = gs[0, 1:].subgridspec(2, 2)
    axs = np.empty((2, 2), dtype=object)
    axs[0, 0] = fig.add_subplot(gs_right[0, 0])
    axs[0, 1] = fig.add_subplot(gs_right[0, 1])
    axs[1, 0] = fig.add_subplot(gs_right[1, 0])
    axs[1, 1] = fig.add_subplot(gs_right[1, 1])
    axs[0, 0].plot(s_vals, q_vals)
    axs[0, 0].set_ylabel("Position p(s)")
    axs[0, 1].plot(s_vals, dq_vals)
    axs[0, 1].set_ylabel("Velocity v(s)")
    axs[1, 0].plot(s_vals, ddq_vals)
    axs[1, 0].set_ylabel("Acceleration a(s)")
    axs[1, 1].plot(s_vals, dddq_vals)
    axs[1, 1].set_ylabel("Jerk j(s)")
    for ax in axs.flatten():
        ax.set_xlabel("s")
        ax.set_xlim(0.0, s_vals[-1])
        ax.grid()

    plt.tight_layout()
    plt.show()


def R3xSO3_blend_path():
    waypoint1 = np.eye(4)
    waypoint1[:3, 3] = np.array([0, 0, 0])

    waypoint2 = np.eye(4)
    waypoint2[:3, :3] = R.from_euler(seq="xyz", angles=[30, 0, 0],
                                     degrees=True).as_matrix()
    waypoint2[:3, 3] = np.array([1, 0, 1])

    waypoint3 = np.eye(4)
    waypoint3[:3, :3] = R.from_euler(seq="xyz",
                                     angles=[30, 0, -45],
                                     degrees=True).as_matrix()
    waypoint3[:3, 3] = np.array([0.3, 1, 1])

    waypoint4 = np.eye(4)
    waypoint4[:3, :3] = R.from_euler(seq="xyz",
                                     angles=[30, 20, -45],
                                     degrees=True).as_matrix()
    waypoint4[:3, 3] = np.array([0, 1, 1])

    waypoint5 = np.eye(4)
    waypoint5[:3, 3] = np.array([0.1, 0.5, 0])

    waypoints = np.array(
        [waypoint1, waypoint2, waypoint3, waypoint4, waypoint5])
    path = PathR3xSO3Bezier(waypoints)
    if not path.calc_path(PathType.Bezier5thCartesian, 0.05):
        exit(-1)

    s_vals = np.linspace(0, path.get_length(), 200)
    p_vals = np.array([path.get_config(s) for s in s_vals])
    v_vals = np.array([path.get_tangent(s) for s in s_vals])
    a_vals = np.array([path.get_curvature(s) for s in s_vals])
    j_vals = np.array([path.get_torsion(s) for s in s_vals])

    fig = plt.figure(figsize=(18, 6))
    ax3d = fig.add_subplot(1, 3, 1, projection='3d')
    for SE3_ in path.get_waypoints():
        plot_SE3(ax3d, SE3_, axis_len=0.2)
    for SE3_ in p_vals[::2]:
        plot_SE3(ax3d, SE3_, axis_len=0.05)
    ax3d.set_xlim([-0.2, 1.2])
    ax3d.set_ylim([-0.2, 1.2])
    ax3d.set_zlim([-0.2, 1.2])
    ax3d.set_title("3D Path")
    ax3d.set_xlabel("x")
    ax3d.set_ylabel("y")
    ax3d.set_zlabel("z")
    ax3d.set_box_aspect([1, 1, 1])

    gs = fig.add_gridspec(1, 3)
    gs_right = gs[0, 1:].subgridspec(2, 2)
    axs = np.empty((2, 2), dtype=object)
    axs[0, 0] = fig.add_subplot(gs_right[0, 0])
    axs[0, 1] = fig.add_subplot(gs_right[0, 1])
    axs[1, 0] = fig.add_subplot(gs_right[1, 0])
    axs[1, 1] = fig.add_subplot(gs_right[1, 1])
    axs[0, 0].plot(s_vals, np.array([SE3_[:3, 3] for SE3_ in p_vals]))
    axs[0, 0].set_ylabel("Config p(s)")
    axs[0, 1].plot(s_vals, np.array([SE3Tangent_ for SE3Tangent_ in v_vals]))
    axs[0, 1].set_ylabel("Tangent v(s)")
    axs[1, 0].plot(s_vals, np.array([SE3Tangent_ for SE3Tangent_ in a_vals]))
    axs[1, 0].set_ylabel("Curvature a(s)")
    axs[1, 1].plot(s_vals, np.array([SE3Tangent_ for SE3Tangent_ in j_vals]))
    axs[1, 1].set_ylabel("Torsion j(s)")
    for ax in axs.flatten():
        ax.set_xlabel("s")
        ax.set_xlim(0.0, s_vals[-1])
        ax.grid()

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    R7_blend_path()
    # R3xSO3_blend_path()
