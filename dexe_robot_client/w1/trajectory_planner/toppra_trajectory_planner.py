# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import numpy as np
import bisect
import toppra as ta
import toppra.constraint as constraint
from dexe_robot_client.common import get_logger

class ToppraTrajectoryPlanner:
    def __init__(self, dofs: int, vel_limits: np.ndarray, acc_limits: np.ndarray):
        self.logger = get_logger()
        self.dofs = dofs
        # default constraints
        self.vel_limits = np.array([[-v, v] for v in vel_limits])
        self.acc_limits = np.array([[-a, a] for a in acc_limits])
        if self.vel_limits.shape[0] != self.dofs:
            raise ValueError(f"Vel limits dimension mismatch, {self.vel_limits.shape[0]} != {self.dofs}")
        if self.acc_limits.shape[0] != self.dofs:
            raise ValueError(f"Acc limits dimension mismatch, {self.acc_limits.shape[0]} != {self.dofs}")

    def plan(self, 
             qpos_array: np.ndarray, 
             qvel_limits_array: np.ndarray = None, 
             time_step: float = 0.01) -> tuple[bool, np.ndarray, np.ndarray, np.ndarray, float]:
        if qpos_array.shape[0] == 2 and \
            np.allclose(qpos_array[0], qpos_array[1], atol=np.deg2rad(0.1)):
            self.logger.info(f"Only two same waypoints, do not plan")
            return (True, 
                    qpos_array, 
                    np.array([[0.]*self.dofs, [0.]*self.dofs]), 
                    np.array([[0.]*self.dofs, [0.]*self.dofs]), 
                    2*time_step)
        
        # Create spline interpolation
        ss = np.linspace(0, 1, qpos_array.shape[0])
        path = ta.SplineInterpolator(ss, qpos_array)

        # Set constraints
        if qvel_limits_array is not None:
            if qvel_limits_array.shape != qpos_array.shape:
                self.logger.error(f"qpos and qvel_limits dimension mismatch, {qvel_limits_array.shape} != {qpos_array.shape}")
                return False, None, None, None, None
            
            qvel_bound_list = []
            for qvel_limits in qvel_limits_array:
                qvel_bound = np.array([[-v, v] for v in qvel_limits])
                qvel_bound_list.append(qvel_bound)
            
            def vlim_func(s):
                idx = bisect.bisect_right(ss, s)
                if idx >= len(ss):
                    idx = len(ss) - 1
                return qvel_bound_list[idx]
            
            pc_vel = constraint.JointVelocityConstraintVarying(vlim_func)
        else:
            pc_vel = constraint.JointVelocityConstraint(self.vel_limits)
       
        pc_acc = constraint.JointAccelerationConstraint(self.acc_limits)
        
        # Create TOPPRA instance
        instance = ta.algorithm.TOPPRA(
            [pc_vel, pc_acc],
            path,
            parametrizer="ParametrizeConstAccel", # 会导致加速度锯齿状，但强约束保证
            gridpt_min_nb_points=max(100, 15 * qpos_array.shape[0]),
        )

        # Compute parameterized trajectory
        jnt_traj = instance.compute_trajectory()
        if jnt_traj is None:
            self.logger.error(f"Unable to find feasible trajectory")
            return False, None, None, None, None

        # Sample trajectory points
        duration = jnt_traj.duration
        ts = np.arange(0, duration, time_step)
        pos_array = jnt_traj.eval(ts)
        vel_array = jnt_traj.evald(ts)
        acc_array = jnt_traj.evaldd(ts)

        # Check acc
        if self._is_acc_valid(acc_array):
            return True, pos_array, vel_array, acc_array, duration
        return False, None, None, None, None

    def _is_acc_valid(self, acc_array) -> bool:
        _acc_limits = self.acc_limits * (1 + 0.25)

        acc_valid = np.all((acc_array >= _acc_limits[:, 0]) & \
                           (acc_array <= _acc_limits[:, 1]))
        
        if not acc_valid:
            self.logger.error(f"Failed to plan trajectory that satisfied acceleration constraint")
            acc_exceed_info = []
            min_acc = np.min(acc_array, axis=0)
            max_acc = np.max(acc_array, axis=0)
            for i in range(self.dofs):
                exceed_ratio = 0.
                if min_acc[i] < self.acc_limits[i, 0]:
                    exceed_ratio = (min_acc[i] - self.acc_limits[i, 0]) / self.acc_limits[i, 0]
                if max_acc[i] > self.acc_limits[i, 1]:
                    temp = (max_acc[i] - self.acc_limits[i, 1]) / self.acc_limits[i, 1]
                    if temp > exceed_ratio:
                        exceed_ratio = temp
                acc_exceed_info.append(exceed_ratio * 100)
            self.logger.info(f"Acc exceeding limit result: {acc_exceed_info} %")

        return acc_valid



if __name__ == "__main__":
    import matplotlib.pyplot as plt

    def plot_trajectory(pos_array: np.ndarray, 
                        vel_array: np.ndarray, 
                        acc_array: np.ndarray, 
                        vel_limits: np.ndarray = None, 
                        acc_limits: np.ndarray = None):
        num_step, dofs = pos_array.shape
        i_steps = np.arange(num_step)

        fig, axs = plt.subplots(3, 1, figsize=(10, 8))
        for i in range(dofs):
            axs[0].plot(i_steps, pos_array[:,i], label=f"Joint {i+1}")
            axs[1].plot(i_steps, vel_array[:,i], label=f"Joint {i+1}")
            axs[2].plot(i_steps, acc_array[:,i], label=f"Joint {i+1}")

        if vel_limits is not None:
            axs[1].plot(i_steps,[vel_limits[0][0]] * len(i_steps), "k--")
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


    ta.setup_logging("INFO")

    dofs = 3
    vel_limits = [1.] * dofs
    acc_limits = [5.] * dofs
    planner = ToppraTrajectoryPlanner(dofs, vel_limits, acc_limits)
    
    qpos_array = np.array([[0.,0.,0.], [1.,2.,3.], [3.,2.,1.], [0.,0.,0.]])
    qvel_array = np.array([[100.,100.,100.], [1.,1.,1.], [2.,2.,2.], [1.,1.,1.]])
    res, pos_array, vel_array, acc_array, duration = planner.plan(qpos_array, qvel_array)
    # res, pos_array, vel_array, acc_array, duration = planner.plan(qpos_array)

    plot_trajectory(pos_array, vel_array, acc_array)