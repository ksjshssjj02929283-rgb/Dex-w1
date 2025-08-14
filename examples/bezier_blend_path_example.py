# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R

from dexe_robot_client.w1 import interpolate_cartesian_path


def plot_SE3(ax, SE3_matrix, len=0.05):
    origin = SE3_matrix[:3,3]
    x_axis = SE3_matrix[:3,0]
    y_axis = SE3_matrix[:3,1]
    z_axis = SE3_matrix[:3,2]
    ax.quiver(*origin, *x_axis, length=len, color='r')
    ax.quiver(*origin, *y_axis, length=len, color='g')
    ax.quiver(*origin, *z_axis, length=len, color='b')


def main():
    waypoints1 = np.eye(4)
    waypoints1[:3,3] = np.array([0,0,0])

    waypoints2 = np.eye(4)
    waypoints2[:3,:3] = R.from_euler(seq="xyz", angles=[30,0,0], degrees=True).as_matrix()
    waypoints2[:3,3] = np.array([1,0,1])

    waypoints3 = np.eye(4)
    waypoints3[:3,:3] = R.from_euler(seq="xyz", angles=[30,0,-45], degrees=True).as_matrix()
    waypoints3[:3,3] = np.array([0.3,1,1])

    waypoints4 = np.eye(4)
    waypoints4[:3,:3] = R.from_euler(seq="xyz", angles=[30,20,-45], degrees=True).as_matrix()
    waypoints4[:3,3] = np.array([0,1,1])

    waypoints5 = np.eye(4)
    waypoints5[:3,3] = np.array([0.1,0.5,0])
    waypoints_list = [waypoints1, waypoints2, waypoints3, waypoints4, waypoints5]

    num_samples = 200
    res, SE3_vec = interpolate_cartesian_path(waypoints_list, 0.1, num_samples)
    if res:
        R3_vec = np.array([temp[:3,3] for temp in SE3_vec])
        SO3_vec = [temp[:3,:3] for temp in SE3_vec]

        # 绘制图像
        fig = plt.figure()
        ax1 = fig.add_subplot(121, projection='3d')
        # 绘制R3曲线
        x_list = [waypoint[0,3] for waypoint in waypoints_list]
        y_list = [waypoint[1,3] for waypoint in waypoints_list]
        z_list = [waypoint[2,3] for waypoint in waypoints_list]
        ax1.plot(x_list, y_list, z_list, 'r-', linewidth=2, label='waypoints')
        ax1.plot(R3_vec[:,0], R3_vec[:,1], R3_vec[:,2], 'b--', linewidth=2, label='blended path')
        # 绘制SO3
        for SE3 in SE3_vec[::2]:
            plot_SE3(ax1, SE3)
        ax1.legend()
        ax1.grid()
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        ax1.set_zlabel('Z')

        ax2 = fig.add_subplot(122)
        i_list = [i for i in range(num_samples)]
        ax2.plot(i_list, R3_vec[:,0], 'r-', linewidth=2, label='X')
        ax2.plot(i_list, R3_vec[:,1], 'g-', linewidth=2, label='Y')
        ax2.plot(i_list, R3_vec[:,2], 'b-', linewidth=2, label='Z')
        ax2.legend()
        ax2.grid()
        ax2.set_xlabel('i')
        ax2.set_ylabel('XYZ')

        # 显示图形
        plt.show()


if __name__ == "__main__":
    main()