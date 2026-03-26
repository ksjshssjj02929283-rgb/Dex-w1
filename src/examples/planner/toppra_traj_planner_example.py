'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

import numpy as np
import matplotlib.pyplot as plt
import toppra as ta

from dexe_robot_client.w1 import ToppraTrajectoryPlanner


def plot_trajectory(pos_array: np.ndarray,
                    vel_array: np.ndarray,
                    acc_array: np.ndarray,
                    vel_limits: np.ndarray = None,
                    acc_limits: np.ndarray = None):
    num_step, dofs = pos_array.shape
    i_steps = np.arange(num_step)

    fig, axs = plt.subplots(3, 1, figsize=(10, 8))
    for i in range(dofs):
        axs[0].plot(i_steps, pos_array[:, i], label=f"Joint {i+1}")
        axs[1].plot(i_steps, vel_array[:, i], label=f"Joint {i+1}")
        axs[2].plot(i_steps, acc_array[:, i], label=f"Joint {i+1}")

    if vel_limits is not None:
        axs[1].plot(i_steps, [vel_limits[0][0]] * len(i_steps), "k--")
        axs[1].plot(i_steps, [vel_limits[0][1]] * len(i_steps), "k--")

    if acc_limits is not None:
        axs[2].plot(i_steps, [acc_limits[0][0]] * len(i_steps), "k--")
        axs[2].plot(i_steps, [acc_limits[0][1]] * len(i_steps), "k--")

    axs[0].set_title("Pos")
    axs[1].set_title("Vel")
    axs[2].set_title("Acc")

    for ax in axs:
        ax.set_xlabel("Time [s]")
        ax.legend()
        ax.grid()

    plt.tight_layout()
    plt.show()


def main():
    ta.setup_logging("INFO")

    dofs = 3
    pos_limits = np.array([[-3.14, -3.14, -3.14], [3.14, 3.14, 3.14]])
    vel_limits = [1.] * dofs
    acc_limits = [5.] * dofs
    planner = ToppraTrajectoryPlanner(dofs, pos_limits, vel_limits, acc_limits)

    qpos_array = np.array([[0., 0., 0.], [1., 2., 3.], [3., 2., 1.],
                           [0., 0., 0.]])
    qvel_array = np.array([[100., 100., 100.], [1., 1., 1.], [2., 2., 2.],
                           [1., 1., 1.]])
    res, pos_array, vel_array, acc_array, duration = planner.plan(
        qpos_array, qvel_array)
    # res, pos_array, vel_array, acc_array, duration = planner.plan(qpos_array)

    plot_trajectory(pos_array, vel_array, acc_array)

    qpos_array2 = np.array([[0., 0., 0.], [1., 2., 3.], [1.5, 2.5, 3.135],
                            [1.5, 2.5, 3.138], [1.5, 2.5, 3.139], [3., 2., 1.],
                            [0., 0., 0.]])
    qvel_array2 = np.array([[100., 100., 100.], [1., 1., 1.], [1., 1., 1.],
                            [1., 1., 1.], [1., 1., 1.], [2., 2., 2.],
                            [1., 1., 1.]])
    res, pos_array2, vel_array2, acc_array2, duration2 = planner.plan(
        qpos_array2, qvel_array2)

    plot_trajectory(pos_array2, vel_array2, acc_array2)


if __name__ == "__main__":
    main()
