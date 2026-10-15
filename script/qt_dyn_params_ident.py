import os
import sys
import math
import re
from dataclasses import replace
import time
from datetime import datetime
import json
import yaml
import threading
import warnings
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from joint_interfaces.msg import JointPositionControl
from robot_server_interfaces.srv import SetProperty
from std_msgs.msg import Header, String
import numpy as np
import matplotlib

warnings.filterwarnings("ignore", category=SyntaxWarning)
matplotlib.use("Qt5Agg")
from PyQt5 import QtWidgets, QtCore
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

DEFAULT_DT = 0.01
DEFAULT_NDOF = 7
DEFAULT_WF = 0.12 * np.pi  # 轨迹基频
DEFAULT_L = 3  # 轨迹谐波数
RUNTIME_INVERSE_DYNAMICS_SOURCE = (
    Path(__file__).resolve().parents[1]
    / "robot_server_lib"
    / "dynamics"
    / "inverse_dynamics_symbol.cpp"
)


def read_data(dof, h, rbtlogfile, trajreffile):
    rbtlog = np.loadtxt(rbtlogfile)
    t = np.array(rbtlog[:, 0])
    q = np.array(rbtlog[:, 1 : dof + 1])
    tau = np.array(rbtlog[:, dof + 1 : dof * 2 + 1])
    if rbtlog.shape[1] >= dof * 2 + 1 + dof:
        tau_sensor = np.array(rbtlog[:, dof * 2 + 1 : dof * 2 + 1 + dof])
    else:
        tau_sensor = np.zeros_like(tau)
    trajref = np.loadtxt(trajreffile) if trajreffile is not None else q.copy()
    reft = np.array([h * i for i in range(trajref.shape[0])])
    return t, q, tau, tau_sensor, reft, trajref


def skew(v):
    return sympy.Matrix([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


class DynParamIdentifier(Node):
    # Default rest positions
    q_torso_rest = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    q_left_rest = [0.0, -1.2, 0.0, -1.0, 0.0, -0.5, -0.5]
    q_right_rest = [0.0, 1.2, 0.0, 1.0, 0.0, 0.5, 0.5]

    def get_full_target_position(self, arm_q, is_left=True):
        if is_left:
            return [*self.q_torso_rest, *arm_q, *self.q_right_rest]
        else:
            return [*self.q_torso_rest, *self.q_left_rest, *arm_q]

    def get_rest_position(self):
        return [*self.q_torso_rest, *self.q_left_rest, *self.q_right_rest]

    def __init__(self, traj="traj1", logger_callable=None):
        super().__init__("qt_dyn_params_ident")
        self.log_msg = logger_callable
        self.joint_publisher = self.create_publisher(
            JointPositionControl, "/control/joint_position", 10
        )
        self.robot_state_subscription = self.create_subscription(
            String, "/feedback/robot_server_state", self._robot_state_callback, 10
        )
        self.set_property_srv = self.create_client(SetProperty, "/control/set_property")
        self.joint_publisher_sim = self.create_publisher(
            JointPositionControl, "/control/joint_position_sim", 10
        )
        self.robot_state_subscription_sim = self.create_subscription(
            String,
            "/feedback/robot_server_state_sim",
            self._robot_state_callback_sim,
            10,
        )
        self.traj = traj
        self.robot_cfg_path = (
            Path.home() / "dexe_robot_server_data/robot_config.yaml"
        )
        # 默认关节列表
        self.joint_full_names = ["ANKLE", "KNEE", "BUTTOCK", "WAIST", "NECK1", "NECK2", "LEFT_J1", "LEFT_J2", "LEFT_J3", "LEFT_J4", "LEFT_J5", "LEFT_J6", "LEFT_J7", "RIGHT_J1", "RIGHT_J2", "RIGHT_J3", "RIGHT_J4", "RIGHT_J5", "RIGHT_J6", "RIGHT_J7"]

        # 初始化状态变量
        self.joint_name = None
        self.timestamp = None
        self.status = None
        self.joint_position = None
        self.joint_torque = None
        self.joint_torque_sensor = None
        self.status_sim = None
        self.joint_position_sim = None
        self.joint_torque_sim = None
        self.joint_torque_sensor_sim = None
        self.dt = DEFAULT_DT
        self.ndof = DEFAULT_NDOF
        self.movej_started = False
        self._stop_flag = threading.Event()
        with open(self.robot_cfg_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
        
        self.robot_name = cfg.get("robot_type", "")
        self.version = cfg.get("version", "")
        version_prefix = self.version[:3]
        robot_version_key = f"{self.robot_name}_{version_prefix}"
        project_root = Path(__file__).resolve().parent
        self.traj_dir = project_root / "dyn_params_ident" / "trajectories" / robot_version_key
        self.rec_dir = project_root / "dyn_params_ident" / "recdata"
        self.traj_path = os.path.join(self.traj_dir, "left", f"{self.traj}.dat")
        # 创建定时器检查连接状态
        self._last_msg_time = None
        self._last_msg_time_sim = None
        rclpy.spin_once(self, timeout_sec=0.1)  # 让订阅生效
        self.create_timer(0.1, self._check_connection_status)
        # 创建calculator
        self.calculator_left = DynParamCalculator(
            robot_type=robot_version_key, is_left=True, logger=self.log_msg
        )
        self.calculator_right = DynParamCalculator(
            robot_type=robot_version_key, is_left=False, logger=self.log_msg
        )
        for calculator in (self.calculator_left, self.calculator_right):
            calculator.set_friction_options(cfg.get(robot_version_key, {}))

    def _sleep_until(self, target_time):
        """精确睡眠到指定单调时钟时间，避免累计漂移。"""
        while True:
            now = time.monotonic()
            remaining = target_time - now
            if remaining <= 0:
                break
            time.sleep(min(remaining, 0.001))

    def _fixed_rate_next(self, last_time, dt):
        """基于无漂移的固定频率调度，返回下一目标时间。"""
        return last_time + dt

    def _check_connection_status(self, timeout_sec: float = 0.5):
        """定时检查消息是否超时"""
        cur_time = self.get_clock().now().nanoseconds / 1e9
        if self._last_msg_time_sim is not None:
            if (cur_time - self._last_msg_time_sim) > timeout_sec:
                self.status_sim = None
                self.joint_position_sim = None
                self.joint_torque_sim = None
                self._last_msg_time_sim = None
        if self._last_msg_time is not None:
            if (cur_time - self._last_msg_time) > timeout_sec:
                self.timestamp = None
                self.status = None
                self.joint_position = None
                self.joint_torque = None
                self._last_msg_time = None

    def is_sim_connected(self, timeout_sec: float = 0.5):
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if self._last_msg_time_sim is not None:
                return True
            time.sleep(0.01)
        return False

    def is_real_connected(self, timeout_sec: float = 0.5):
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if self._last_msg_time is not None:
                return True
            time.sleep(0.01)
        return False

    def set_safe_level(self, value: float):
        """
        设置 SafeLevel 属性
        :param node: rclpy 的 Node 实例
        :param value: 安全等级值（float 或 str）
        """
        req = SetProperty.Request()
        req.property_type = "SafeLevel"
        req.value = str(value)
        self.set_property_srv.call_async(req)

    def clear_error(self):
        """
        清除机器人错误
        """
        req = SetProperty.Request()
        req.property_type = "ClearError"
        req.value = ""
        self.set_property_srv.call_async(req)

    def run_traj_sim(self, traj_path):
        """
        在仿真中测试轨迹的可行性。
        """
        self._stop_flag.clear()
        with open(traj_path, "r") as file:
            q_ref = np.loadtxt(file)
        # 使用GUI设置的仿真步长覆盖（仅调度用）
        local_dt = getattr(self, "_sim_dt_override", self.dt)
        if self.is_left:
            # 运动到第一个点
            target_position = self.get_full_target_position(q_ref[0, :], is_left=True)
            if not self.move_joint(is_sim=True, target_position=target_position):
                return False
            self.log_msg(
                f"开始执行左臂仿真轨迹: {self.traj} 当前qpos: {target_position}"
            )
            next_t = time.monotonic()
            for i in range(q_ref.shape[0]):
                if self._stop_flag.is_set():
                    self.log_msg("仿真停止指令收到，提前结束。")
                    return False
                target_position = self.get_full_target_position(q_ref[i, :], is_left=True)
                if not self._joint_publisher(
                    is_sim=True, target_position=target_position
                ):
                    return False
                next_t = self._fixed_rate_next(next_t, local_dt)
                self._sleep_until(next_t)
        else:
            # 运动到第一个点
            target_position = self.get_full_target_position(q_ref[0, :], is_left=False)
            if not self.move_joint(is_sim=True, target_position=target_position):
                return False
            self.log_msg(
                f"开始执行右臂仿真轨迹: {self.traj}, 当前qpos: {target_position}"
            )
            next_t = time.monotonic()
            for i in range(q_ref.shape[0]):
                if self._stop_flag.is_set():
                    self.log_msg("仿真停止指令收到，提前结束。")
                    return False
                target_position = self.get_full_target_position(q_ref[i, :], is_left=False)
                if not self._joint_publisher(
                    is_sim=True, target_position=target_position
                ):
                    return False
                next_t = self._fixed_rate_next(next_t, local_dt)
                self._sleep_until(next_t)
        if not self._stop_flag.is_set():
            time.sleep(1)  # 发送结束后等待1s
            self.log_msg("仿真轨迹执行完成, 返回初始姿态。")
            # 返回初始姿态
            target_position = self.get_rest_position()
            self.move_joint(is_sim=True, target_position=target_position)
        return True

    def run_real_capture(self):
        """
        在真机上依次执行左臂两段轨迹 + 右臂两段轨迹，并记录轨迹数据。
        """
        # 先设置安全等级为最高
        self.set_safe_level(1.0)
        self._stop_flag.clear()
        orig_is_left = self.is_left
        orig_traj = self.traj

        traj_list = ["traj1", "traj2"]
        left_dir = os.path.join(self.rec_dir, "left")
        right_dir = os.path.join(self.rec_dir, "right")
        os.makedirs(left_dir, exist_ok=True)
        os.makedirs(right_dir, exist_ok=True)

        # 先左臂
        self.is_left = True
        for traj_name in traj_list:
            if self._stop_flag.is_set():
                self.log_msg("收到停止指令，终止剩余真机采集。")
                return False
            self.traj = traj_name
            self.traj_path = os.path.join(self.traj_dir, "left", f"{self.traj}.dat")
            rec_path = os.path.join(left_dir, f"rbtlog_{self.traj}.dat")
            if not self._run_real_capture_once(self.traj_path, rec_path):
                return False
        # 再右臂
        if not self._stop_flag.is_set():
            self.is_left = False
            for traj_name in traj_list:
                if self._stop_flag.is_set():
                    self.log_msg("收到停止指令，终止剩余真机采集。")
                    return False
                self.traj = traj_name
                self.traj_path = os.path.join(self.traj_dir, "right", f"{self.traj}.dat")
                rec_path = os.path.join(right_dir, f"rbtlog_{self.traj}.dat")
                if not self._run_real_capture_once(self.traj_path, rec_path):
                    return False
        # 恢复初始状态
        self.is_left = orig_is_left
        self.traj = orig_traj
        side_dir = "left" if self.is_left else "right"
        self.traj_path = os.path.join(self.traj_dir, side_dir, f"{self.traj}.dat")
        if not self._stop_flag.is_set():
            time.sleep(1)  # 发送结束后等待1s
            self.log_msg("所有轨迹采集完成, 返回初始姿态。")
            target_position = self.get_rest_position()
            ret = self.move_joint(is_sim=False, target_position=target_position)
            if not ret:
                return False
            self.log_msg("已返回初始姿态")
        return True

    def run_real_capture_arm(self, is_left):
        self.set_safe_level(1.0)
        self._stop_flag.clear()
        orig_is_left = self.is_left
        orig_traj = self.traj

        traj_list = ["traj1", "traj2"]
        side = "left" if is_left else "right"
        side_dir = os.path.join(self.rec_dir, side)
        os.makedirs(side_dir, exist_ok=True)

        self.is_left = is_left
        for traj_name in traj_list:
            if self._stop_flag.is_set():
                self.log_msg("收到停止指令，终止剩余真机采集。")
                return False
            self.traj = traj_name
            self.traj_path = os.path.join(self.traj_dir, side, f"{self.traj}.dat")
            rec_path = os.path.join(side_dir, f"rbtlog_{self.traj}.dat")
            if not self._run_real_capture_once(self.traj_path, rec_path):
                return False

        self.is_left = orig_is_left
        self.traj = orig_traj
        side_dir_orig = "left" if self.is_left else "right"
        self.traj_path = os.path.join(self.traj_dir, side_dir_orig, f"{self.traj}.dat")
        if not self._stop_flag.is_set():
            time.sleep(1)
            arm_label = "左" if is_left else "右"
            self.log_msg(f"{arm_label}臂轨迹采集完成, 返回初始姿态。")
            target_position = self.get_rest_position()
            ret = self.move_joint(is_sim=False, target_position=target_position)
            if not ret:
                return False
            self.log_msg("已返回初始姿态")
        return True

    def _run_real_capture_once(self, traj_path, rec_path):
        with open(traj_path, "r") as file:
            q_ref = np.loadtxt(file)
        # 使用GUI设置的真机步长覆盖
        local_dt = getattr(self, "_real_dt_override", self.dt)
        rec = np.zeros((q_ref.shape[0], 1 + 7 + 7 + 7))
        if self.is_left:
            # 运动到第一个点
            self.log_msg("正在运动到左臂轨迹起始点...")
            target_position = self.get_full_target_position(q_ref[0, :], is_left=True)
            ret = self.move_joint(is_sim=False, target_position=target_position)
            if not ret:
                return False
            self.log_msg(f"开始执行左臂真机采集: {self.traj}")
            time0 = self.timestamp
            next_t = time.monotonic()
            for i in range(q_ref.shape[0]):
                if self._stop_flag.is_set():
                    self.log_msg("收到停止指令，终止当前段采集。")
                    return False
                target_position = self.get_full_target_position(q_ref[i, :], is_left=True)
                if not self._joint_publisher(
                    is_sim=False, target_position=target_position
                ):
                    return False
                _tau_current = self.joint_torque[6:13] if self.joint_torque else [0.0] * 7
                _tau_sensor = self.joint_torque_sensor[6:13] if self.joint_torque_sensor else [0.0] * 7
                rec[i, :] = np.concatenate(
                    [
                        np.array([float(self.timestamp - time0) * 1e-9]),
                        np.array(self.joint_position[6:13]),
                        np.array(_tau_current),
                        np.array(_tau_sensor),
                    ]
                )
                next_t = self._fixed_rate_next(next_t, local_dt)
                self._sleep_until(next_t)
        else:
            # 运动到第一个点
            self.log_msg("正在运动到右臂轨迹起始点...")
            target_position = self.get_full_target_position(q_ref[0, :], is_left=False)
            ret = self.move_joint(is_sim=False, target_position=target_position)
            if not ret:
                return False
            self.log_msg(f"开始执行右臂真机采集: {self.traj}")
            time0 = self.timestamp
            next_t = time.monotonic()
            for i in range(q_ref.shape[0]):
                if self._stop_flag.is_set():
                    self.log_msg("收到停止指令，终止当前段采集。")
                    return False
                target_position = self.get_full_target_position(q_ref[i, :], is_left=False)
                if not self._joint_publisher(
                    is_sim=False, target_position=target_position
                ):
                    return False
                _tau_current = self.joint_torque[13:20] if self.joint_torque else [0.0] * 7
                _tau_sensor = self.joint_torque_sensor[13:20] if self.joint_torque_sensor else [0.0] * 7
                rec[i, :] = np.concatenate(
                    [
                        np.array([float(self.timestamp - time0) * 1e-9]),
                        np.array(self.joint_position[13:20]),
                        np.array(_tau_current),
                        np.array(_tau_sensor),
                    ]
                )
                next_t = self._fixed_rate_next(next_t, local_dt)
                self._sleep_until(next_t)
        # 仅在未停止时保存
        if not self._stop_flag.is_set():
            np.savetxt(rec_path, rec, fmt="%.6f")
            time.sleep(1)  # 发送结束后等待1s
            self.log_msg(f"本段轨迹采集完成, 数据已保存到 {rec_path}")
            return True

    def move_joint(self, is_sim: bool, target_position: list, max_speed=0.5):
        if self.movej_started:
            self.log_msg("已有插值movej移动在进行中，忽略新的移动请求")
            return False
        self._stop_flag.clear()
        if is_sim:
            if not self.is_sim_connected():
                self.log_msg("仿真server连接失败，无法发送指令。")
                return False
            joint_position = self.joint_position_sim
        else:
            if not self.is_real_connected():
                self.log_msg("真机server连接失败，无法发送指令。")
                return False
            joint_position = self.joint_position

        if joint_position is None:
            self.log_msg("当前关节位置未知，无法进行插值movej移动。")
            return False

        max_distance = max(
            [abs(tp - cp) for tp, cp in zip(target_position, joint_position)]
        )
        total_points = max(2, math.ceil(max_distance / max_speed / self.dt))
        interp_positions = np.linspace(
            np.array(joint_position), np.array(target_position), total_points
        )
        self.movej_started = True
        self.log_msg(
            f"执行插值movej, 当前是否仿真: {is_sim}，轨迹时长 {total_points * self.dt} 秒"
        )

        next_t = time.monotonic()
        for step in range(total_points):
            if self._stop_flag.is_set():
                self.log_msg("停止指令收到，终止插值movej。")
                self.movej_started = False
                return False
            ret = self._joint_publisher(is_sim, interp_positions[step].tolist())
            if not ret:
                self.log_msg("插值movej移动过程中被中止。")
                self.movej_started = False
                return False
            next_t = self._fixed_rate_next(next_t, self.dt)
            self._sleep_until(next_t)
        self.movej_started = False
        time.sleep(1)  # 发送结束后等待1s
        self.log_msg("插值movej移动完成。")
        return True

    def _joint_publisher(self, is_sim: bool, target_position: list):
        if (is_sim and (self.status_sim in ("Init", "Error"))) or (
            (not is_sim) and (self.status in ("Init", "Error"))
        ):
            self.log_msg("检测到机器人未处于Enable状态, 停止发送指令")
            return False
        if self.joint_name is None:
            self.log_msg("当前关节名称未知，无法移动。")
            return False
        if is_sim:
            if not self.is_sim_connected():
                self.log_msg("仿真server连接失败，无法发送指令。")
                return False
            publisher = self.joint_publisher_sim
            joint_name = self.joint_name
        else:
            if not self.is_real_connected():
                self.log_msg("真机server连接失败，无法发送指令。")
                return False
            publisher = self.joint_publisher
            joint_name = self.joint_name

        msg = JointPositionControl()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = target_position
        msg.name = joint_name
        publisher.publish(msg)
        return True

    def _stop_move(self):
        """
        停止当前所有轨迹下发。
        """
        self._stop_flag.set()

    def plot_records_to_axes(self, ax_handles):
        """
        读取记录并绘制到传入的 matplotlib axes 列表。
        如果 ax_handles 为 None，则使用 plt.show() 弹窗（兼容单独调用）。
        """
        rec_filename = f"rbtlog_{self.traj}.dat"
        side_dir = "left" if self.is_left else "right"
        calculator = self.calculator_left if self.is_left else self.calculator_right
        rec_path = calculator.recording_path(self.traj)
        traj_path = os.path.join(self.traj_dir, side_dir, f"{self.traj}.dat")

        t_raw, q_raw, tau_raw, tau_sensor_raw, t_ref, q_ref = read_data(
            self.ndof, self.dt, rec_path, None if self.traj in calculator.data_files else traj_path
        )
        n_joints = q_raw.shape[1]

        # 统一取样长度
        len_tref = len(t_ref)
        len_traw = len(t_raw)
        len_ttau = tau_raw.shape[0]
        min_len = int(min(len_tref, len_traw, len_ttau))

        t_ref_cut = np.asarray(t_ref[:min_len])
        t_raw_cut = np.asarray(t_raw[:min_len])
        q_ref_cut = np.asarray(q_ref[:min_len, :])
        q_raw_cut = np.asarray(q_raw[:min_len, :])
        tau_cut = np.asarray(tau_raw[:min_len, :])
        tau_sensor_cut = np.asarray(tau_sensor_raw[:min_len, :]) if tau_sensor_raw.size > 0 else None

        # 滤波后的力矩: 与辨识中 q/dq/ddq 用同一套 Butterworth 参数, 与原始力矩画在一起对比
        tau_filt_cut = None
        tau_filt_label = "Filtered τ"
        try:
            calc = self.calculator_left if self.is_left else self.calculator_right
            wn = calc.get_filter_cutoff() * 2 * np.pi * calc.dt  # 与 butter_filtfilt 的传参一致
            fc_eff = wn * 0.5 / calc.dt                          # 换算回实际 -3dB 频率(Hz)
            tau_filt_cut = np.column_stack(
                [calc.butter_filtfilt(3, wn, tau_cut[:, j]) for j in range(n_joints)]
            )
            tau_filt_label = f"Filtered τ ({fc_eff:.1f} Hz)"
        except Exception as e:
            self.log_msg("力矩滤波绘图失败(仅影响该曲线): " + str(e))

        for j in range(min(len(ax_handles), n_joints)):
            ax1 = ax_handles[j]
            ax1.clear()
            ref_series = q_ref_cut[:, j]
            if self.traj not in calculator.data_files:
                ax1.plot(
                    t_ref_cut,
                    ref_series,
                    label="Reference q",
                    color="tab:blue",
                    linewidth=1,
                )
            ax1.plot(
                t_raw_cut,
                q_raw_cut[:, j],
                label="Measured q",
                color="tab:green",
                alpha=0.8,
                linewidth=1,
            )
            ax1.set_title(f"Joint {j+1}")
            ax1.grid(True, linestyle="--", alpha=0.5)
            ax1.set_xlabel("Time (s)")
            ax1.set_ylabel("Angle (rad)", color="tab:blue")
            ax2 = ax1.twinx()
            ax2.plot(
                t_raw_cut,
                tau_cut[:, j],
                label="Measured τ (current)",
                color="tab:red",
                linewidth=1,
            )
            if tau_filt_cut is not None:
                ax2.plot(
                    t_raw_cut,
                    tau_filt_cut[:, j],
                    label=tau_filt_label,
                    color="tab:purple",
                    linewidth=1,
                )
            if tau_sensor_cut is not None and np.any(np.abs(tau_sensor_cut[:, j]) > 1e-9):
                ax2.plot(
                    t_raw_cut,
                    tau_sensor_cut[:, j],
                    label="Measured τ (sensor)",
                    color="tab:orange",
                    linewidth=1,
                    linestyle="--",
                )
            ax2.set_ylabel("Torque (Nm)", color="tab:red")
            ax1.tick_params(axis="y", labelcolor="tab:blue")
            ax2.tick_params(axis="y", labelcolor="tab:red")
            h1, l1 = ax1.get_legend_handles_labels()
            h2, l2 = ax2.get_legend_handles_labels()
            ax1.legend(h1 + h2, l1 + l2, loc="upper right", fontsize="small")

    def _robot_state_callback(self, msg):
        self._last_msg_time = self.get_clock().now().nanoseconds / 1e9
        state = json.loads(msg.data)
        if "timestamp" in state:
            self.timestamp = state["timestamp"]
        if "status" in state:
            self.status = state["status"]
        
        if "joint_name" in state and "joint_position" in state:
            msg_joint_names = state["joint_name"]
            msg_joint_positions = [float(x) for x in state["joint_position"]]
            msg_joint_torques = [float(x) for x in state.get("joint_torque", [])]
            msg_joint_torques_sensor = [float(x) for x in state.get("joint_torque_sensor", [])]
            
            full_pos = []
            full_tau = []
            full_tau_sensor = []
            for name in self.joint_full_names:
                if name in msg_joint_names:
                    idx = msg_joint_names.index(name)
                    full_pos.append(msg_joint_positions[idx])
                    if idx < len(msg_joint_torques):
                        full_tau.append(msg_joint_torques[idx])
                    else:
                        full_tau.append(0.0)
                    if idx < len(msg_joint_torques_sensor):
                        full_tau_sensor.append(msg_joint_torques_sensor[idx])
                    else:
                        full_tau_sensor.append(0.0)
                else:
                    full_pos.append(0.0)
                    full_tau.append(0.0)
                    full_tau_sensor.append(0.0)
            self.joint_name = self.joint_full_names
            self.joint_position = full_pos
            self.joint_torque = full_tau
            self.joint_torque_sensor = full_tau_sensor

    def _robot_state_callback_sim(self, msg):
        self._last_msg_time_sim = self.get_clock().now().nanoseconds / 1e9
        state = json.loads(msg.data)
        if "status" in state:
            self.status_sim = state["status"]
        
        if "joint_name" in state and "joint_position" in state:
            msg_joint_names = state["joint_name"]
            msg_joint_positions = [float(x) for x in state["joint_position"]]
            msg_joint_torques = [float(x) for x in state.get("joint_torque", [])]
            msg_joint_torques_sensor = [float(x) for x in state.get("joint_torque_sensor", [])]
            full_pos = []
            full_tau = []
            full_tau_sensor = []
            for name in self.joint_full_names:
                if name in msg_joint_names:
                    idx = msg_joint_names.index(name)
                    full_pos.append(msg_joint_positions[idx])
                    if idx < len(msg_joint_torques):
                        full_tau.append(msg_joint_torques[idx])
                    else:
                        full_tau.append(0.0)
                    if idx < len(msg_joint_torques_sensor):
                        full_tau_sensor.append(msg_joint_torques_sensor[idx])
                    else:
                        full_tau_sensor.append(0.0)
                else:
                    full_pos.append(0.0)
                    full_tau.append(0.0)
            
            self.joint_position_sim = full_pos
            self.joint_torque_sim = full_tau
            self.joint_torque_sensor_sim = full_tau_sensor


import sympy, sympybotics, scipy.signal, lmi_sdp, subprocess
from sympy import BlockMatrix, Matrix, eye, Identity
from lmi_sdp import LMI
from sympybotics.dynident.regression import mrepl
from dyn_params_ident.urdf2dh import get_dh_params_for_arm, get_gravity_in_dh_base
from dyn_params_ident.acceleration_free_regressor import (
    build_filtered_regressor,
    lowpass_filter,
)
from dyn_params_ident.runtime_friction import read_runtime_friction, predict_runtime_friction
from dyn_params_ident.friction_model import (
    smooth_sign, friction_bases, friction_settings,
    FRICTION_MODEL_CLASSIC, FRICTION_MODEL_SMOOTH, FRICTION_MODEL_STRIBECK,
)


def update_robot_config_section(content, version_key, updates, remove_keys=()):
    config = yaml.safe_load(content)
    if not isinstance(config, dict) or not isinstance(config.get(version_key), dict):
        raise ValueError(f"配置中未找到版本段: {version_key}")
    section = re.search(
        rf"^{re.escape(version_key)}:[ \t]*(?:#[^\n]*)?\n(?:(?!^[^\s#][^\n]*:).*(?:\n|$))*",
        content, re.MULTILINE,
    )
    if section is None:
        raise ValueError(f"不支持的版本段格式: {version_key}")
    block = section.group(0)
    for key in remove_keys:
        block = re.sub(rf"^  {re.escape(key)}:[^\n]*(?:\n|$)", "", block, flags=re.MULTILINE)
    for key, value in updates.items():
        encoded = json.dumps(value, ensure_ascii=False, allow_nan=False)
        if isinstance(value, str):
            encoded = value
        pattern = rf"^(  {re.escape(key)}:[ \t]*)(?:\[[\s\S]*?\]|[^\n#]*)([ \t]*(?:#[^\n]*)?)$"
        if re.search(pattern, block, re.MULTILINE):
            block = re.sub(pattern, lambda match: match[1] + encoded + match[2], block, flags=re.MULTILINE)
        else:
            block = block.rstrip("\n") + f"\n  {key}: {encoded}\n"
    result = content[:section.start()] + block + content[section.end():]
    yaml.safe_load(result)
    return result

IDENTIFICATION_STANDARD = "standard"
IDENTIFICATION_AF_ARF = "af_arf"

class DynParamCalculator:
    """
    动力学参数辨识计算类, 通过读取采集的轨迹计算最小辨识集动力学参数。
    """

    def __init__(self, robot_type, is_left, logger):
        self.log_msg = logger
        project_root = Path(__file__).resolve().parent
        self.traj_dir = project_root / "dyn_params_ident" / "trajectories" / robot_type
        self.rec_dir = project_root / "dyn_params_ident" / "recdata"
        self.is_v02 = robot_type == "w1_v02"
        self.is_left = is_left
        self.robot_type = robot_type
        self.ndof = DEFAULT_NDOF
        self.dt = DEFAULT_DT
        self.wf = DEFAULT_WF
        self.L = DEFAULT_L
        self.proc_data = {}
        self.data_files = {}
        self._feedforward_functions = None
        self.solutions_beta_starextra = None
        self.rbt = None
        self.identification_algorithm = IDENTIFICATION_STANDARD
        self.af_alpha = 15.0
        self.af_beta = 10.0
        self.filter_identification_torque = False
        self.friction_options = friction_settings({})

    def set_friction_options(self, config):
        options = friction_settings(config)
        if options == self.friction_options:
            return False
        self.friction_options = options
        self.reset_identification()
        return True

    def reset_identification(self):
        self.proc_data.clear()
        self.solutions_beta_starextra = None

    def recording_path(self, traj):
        side = "left" if self.is_left else "right"
        return self.data_files.get(traj, Path(self.rec_dir) / side / f"rbtlog_{traj}.dat")

    def set_recording_file(self, traj, filename):
        self.data_files[traj] = Path(filename)
        if traj == "traj1":
            self.reset_identification()
        else:
            self.proc_data.pop("traj2", None)

    def cache_validation_data(self):
        self._cache_trajectory("traj2")

    def _cache_zero_acceleration_validation_data(self):
        """Prepare q/dq and measured torque for ddq=0 plotting without W."""
        traj = "traj2"
        rec_path = self.recording_path(traj)
        if not rec_path.is_file():
            raise RuntimeError("未找到验证数据，请选择验证文件。")
        fc = self.get_filter_cutoff()
        si = int(3 / self.dt)
        sf = -int(1 / self.dt)
        t_raw, q_raw, tau_current_raw, tau_sensor_raw, _, _ = read_data(
            self.ndof, self.dt, rec_path, None
        )
        if len(t_raw) <= si - sf:
            raise ValueError("采集数据不足，裁剪前 3 秒和后 1 秒后没有可用样本。")
        q_filtered, dq_filtered, _, tau_current = self.diff_and_filt_data(
            dof=self.ndof, h=self.dt, q_raw=q_raw, tau_raw=tau_current_raw,
            fc_q=fc, fc_dq=fc, fc_ddq=fc, fc_tau=None,
        )
        keep = slice(si, sf)
        t = np.arange(len(q_filtered[keep])) * self.dt
        self.proc_data[traj] = {
            "t": t,
            "q": q_filtered[keep],
            "dq": dq_filtered[keep],
            "tau_current": tau_current_raw[keep],
            "tau_sensor": tau_sensor_raw[keep],
            "tau": tau_current[keep],
            "tau_sensor_filtered": None,
            "friction_context": None,
            "algorithm": self.identification_algorithm,
            "evaluation_domain": "instantaneous",
            "zero_acceleration_only": True,
        }

    def set_identification_options(self, algorithm, alpha, beta, filter_torque=False):
        if algorithm not in (IDENTIFICATION_STANDARD, IDENTIFICATION_AF_ARF):
            raise ValueError(f"未知辨识算法: {algorithm}")
        alpha, beta = float(alpha), float(beta)
        if not np.isfinite(alpha) or alpha <= 0 or not np.isfinite(beta) or beta < 0:
            raise ValueError("AF-ARF 滤波参数要求 alpha > 0, beta >= 0")
        options = (algorithm, alpha, beta, bool(filter_torque))
        if options == (self.identification_algorithm, self.af_alpha, self.af_beta,
                       self.filter_identification_torque):
            return False
        (self.identification_algorithm, self.af_alpha, self.af_beta,
         self.filter_identification_torque) = options
        self.reset_identification()
        return True

    # todo: 补充v03模型
    def create_robot_model(self, robot_type):
        # 构建机器人模型, 并计算基础参数集
        arm_side = "left" if self.is_left else "right"
        dh_params = get_dh_params_for_arm(arm_side)
        self.log_msg(f"DH parameters for {robot_type} {arm_side} arm:\n{dh_params}")
        gravity = get_gravity_in_dh_base(arm_side)
        self.log_msg(f"Gravity direction in {arm_side} arm base: ({gravity[0]:.4f}, {gravity[1]:.4f}, {gravity[2]:.4f})")
        self.rbtdef_num = sympybotics.RobotDef(
            f"{robot_type}_{arm_side}",
            dh_params,
            dh_convention="standard",
        )
        self.rbtdef_num.frictionmodel = {"Coulomb", "viscous", "offset"}
        self.rbtdef_num.driveinertiamodel = "simplified"
        self.rbtdef_num.gravityacc = sympy.Matrix(gravity)
        rbt = sympybotics.RobotDynCode(self.rbtdef_num, verbose=True)
        rbt.calc_base_parms()
        return rbt

    def calc_dyn_params(self):
        self.reset_identification()
        # 创建机器人模型并计算基础参数集
        self.log_msg("正在进行机器人建模...")
        self.rbt = self.create_robot_model(self.robot_type)
        self._feedforward_functions = None
        self.log_msg(f"机器人建模完毕, 最小参数辨识集: {self.rbt.dyn.baseparms}")
        # 输出包含fo_的参数的序号
        fc_indices = [i for i, param in enumerate(self.rbt.dyn.baseparms) if "fc_" in str(param)]
        fv_indices = [i for i, param in enumerate(self.rbt.dyn.baseparms) if "fv_" in str(param)]
        fo_indices = [i for i, param in enumerate(self.rbt.dyn.baseparms) if "fo_" in str(param)]
        self.log_msg(f"f_coulumb index: {fc_indices}, f_viscous index: {fv_indices}, f_offset index: {fo_indices}")
        self.log_msg("开始处理轨迹数据...")
        self.process_and_cache_data()
        self.log_msg("轨迹数据处理完毕, 开始计算参数...")
        delta = self.rbt.dyn.dynparms
        beta = self.rbt.dyn.baseparms
        n_beta = self.rbt.dyn.n_base
        beta_symbs = sympy.Matrix(
            [sympy.Symbol("beta" + str(i + 1), real=True) for i in range(n_beta)]
        )
        delta_d = self.rbt.dyn.Pd.T * delta
        Pb = self.rbt.dyn.Pb
        varchange_dict = dict(zip(Pb.T * delta, beta_symbs - (beta - Pb.T * delta)))
        I = Identity
        S = skew
        D_inertia_blocks = []
        for i in range(self.rbt.dof):
            L = self.rbt.rbtdef.L[i]
            l = self.rbt.rbtdef.l[i]
            m = self.rbt.rbtdef.m[i]
            Di = BlockMatrix([[L, S(l).T], [S(l), I(3) * m]])
            D_inertia_blocks.append(Di.as_explicit())
        D_other_blocks = []
        for i in range(self.rbt.dof):
            if self.rbt.rbtdef.driveinertiamodel == "simplified":
                D_other_blocks.append(Matrix([self.rbt.rbtdef.Ia[i]]))
            if "viscous" in self.rbt.rbtdef.frictionmodel:
                D_other_blocks.append(Matrix([self.rbt.rbtdef.fv[i]]))
            if "Coulomb" in self.rbt.rbtdef.frictionmodel:
                D_other_blocks.append(Matrix([self.rbt.rbtdef.fc[i]]))
            if self.friction_options["friction_model"] == FRICTION_MODEL_STRIBECK:
                D_other_blocks.append(Matrix([self.rbt.rbtdef.fo[i]]))
        D_blocks = D_inertia_blocks + D_other_blocks
        varchange_dict = dict(zip(Pb.T * delta, beta_symbs - (beta - Pb.T * delta)))
        DB_blocks = [mrepl(Di, varchange_dict) for Di in D_blocks]
        epsilon_safemargin = 1e-6
        DB_LMIs_marg = list(
            map(lambda lm: LMI(lm, epsilon_safemargin * eye(lm.shape[0])), DB_blocks)
        )

        ident_traj = "traj1"
        regr_mats = self.proc_data[ident_traj]
        W = np.matrix(regr_mats["W"])
        omega = np.matrix(regr_mats["omega"])
        R1 = np.matrix(regr_mats["R1"])
        rho1 = np.matrix(regr_mats["rho1"])
        rho2_norm_sqr = np.linalg.norm(omega - W * ((R1.T * R1).I * R1.T * rho1)) ** 2

        # 质心位置约束
        link_cuboid_hulls = [
            [[-0.1, +0.06], [-0.08, +0.08], [-0.0, +0.16]],
            [[-0.0, +0.16], [-0.08, +0.08], [-0.08, +0.08]],
            [[-0.11, +0.05], [-0.06, +0.1], [0.02, +0.18]],
            [[-0.0, +0.16], [-0.08, +0.08], [-0.08, +0.08]],
            [[-0.08, +0.08], [-0.1, +0.06], [-0.04, +0.12]],
            [[-0.08, +0.08], [-0.08, +0.08], [-0.08, +0.08]],
            [[-0.04, +0.12], [-0.04, +0.04], [-0.0, +0.16]],
        ]

        robotmaxmass = 27.0
        D_add_blocks = []
        for i in range(self.rbt.dof):
            l = self.rbt.rbtdef.l[i]
            m = self.rbt.rbtdef.m[i]
            link_cuboid_hull = link_cuboid_hulls[i]
            for j in range(3):
                D_add_blocks.append(Matrix([[l[j] - m * link_cuboid_hull[j][0]]]))
                D_add_blocks.append(Matrix([[-l[j] + m * link_cuboid_hull[j][1]]]))
        # Variable Change and LMI Definition
        D_add_blocks.append(Matrix([robotmaxmass - sum(self.rbt.rbtdef.m)]))
        DB_add_blocks = [mrepl(Di, varchange_dict) for Di in D_add_blocks]
        DB_add_LMIs_marg = list(
            map(
                lambda lm: LMI(lm, epsilon_safemargin * eye(lm.shape[0])), DB_add_blocks
            )
        )

        u = sympy.Symbol("u")
        U_rho = BlockMatrix(
            [
                [Matrix([u - rho2_norm_sqr]), (rho1 - R1 * beta_symbs).T],
                [rho1 - R1 * beta_symbs, I(n_beta)],
            ]
        )
        U_rho = U_rho.as_explicit()
        lmis_fbpe_ols_add = [LMI(U_rho)] + DB_LMIs_marg + DB_add_LMIs_marg
        variables_fbpe_ols_add = [u] + list(beta_symbs) + list(delta_d)
        objf_fbpe_ols_add = u
        sol_fbpe_ols_add = self.dsdp5(
            objf_fbpe_ols_add, lmis_fbpe_ols_add, variables_fbpe_ols_add
        )
        beta_starextra = np.matrix(sol_fbpe_ols_add[1 : 1 + n_beta])
        delta_d_starextra = np.matrix(sol_fbpe_ols_add[1 + n_beta :])
        dict_subs = dict(zip(beta_symbs, beta_starextra.flatten().tolist()[0]))
        dict_subs.update(dict(zip(delta_d, delta_d_starextra.flatten().tolist()[0])))
        for DBi in DB_blocks + DB_add_blocks:
            m = np.matrix(mrepl(DBi, dict_subs)).astype(float)
            for v in np.linalg.eigvals(m):
                if v <= 0.0:
                    return False
        candidate = np.matrix(beta_starextra)
        if not self.test_mass_matrix_psd(candidate):
            return False
        self.solutions_beta_starextra = candidate
        return True

    def process_and_cache_data(self):
        for traj in ["traj1", "traj2"]:
            if traj == "traj2" and not self.recording_path(traj).is_file():
                self.log_msg("未找到 traj2 采集数据，跳过独立轨迹验证。")
                continue
            self._cache_trajectory(traj)
        self.log_msg("所有轨迹数据处理完成并保存。")

    def _cache_trajectory(self, traj):
        (
            t_raw, q_raw, tau_current_raw, tau_sensor_raw,
            t, q, dq, ddq, tau, W, omega, Q1, R1, rho1, tau_sensor_filtered,
            friction_context,
        ) = self.process_regression_data(traj)
        self.proc_data[traj] = {
            "t_raw": t_raw,
            "q_raw": q_raw,
            "tau_current": tau_current_raw,
            "tau_sensor": tau_sensor_raw,
            "t": t,
            "q": q,
            "dq": dq,
            "ddq": ddq,
            "tau": tau,
            "W": W,
            "omega": omega,
            "Q1": Q1,
            "R1": R1,
            "rho1": rho1,
            "tau_sensor_filtered": tau_sensor_filtered,
            "algorithm": self.identification_algorithm,
            "evaluation_domain": "instantaneous" if traj == "traj2" else None,
            "af_alpha": self.af_alpha,
            "af_beta": self.af_beta,
            "filter_identification_torque": self.filter_identification_torque if traj == "traj1" else False,
            "friction_context": friction_context,
        }

    def get_filter_cutoff(self):
        """辨识数据滤波的截止频率(Hz)。

        取轨迹最高谐波频率的 10 倍, 与 process_regression_data 中 q/dq/ddq 用的取值一致。
        注意传入 scipy 的归一化频率是 fc * 2 * pi * dt, 换算回实际 -3dB 频率约为 pi * fc。
        """
        return 10 * (self.wf / (2 * np.pi) * self.L)

    def process_regression_data(self, traj):
        # 数据滤波, 差分和回归数据生成
        fc = self.get_filter_cutoff()
        si = int(3 / self.dt)
        sf = -int(1 / self.dt)
        # 使用 Path / 拼接，并在 read_data 时转成 str
        rec_path = self.recording_path(traj)
        self.log_msg(f"[{traj}] {'辨识' if traj == 'traj1' else '验证'}数据: {rec_path}")
        t_raw, q_raw, tau_current_raw, tau_sensor_raw, t_ref, q_ref = read_data(
            self.ndof, self.dt, rec_path, None
        )
        # 辨识使用电流估算力矩 (joint_torque), sensor torque 仅用于显示对比
        tau_raw = tau_current_raw
        self.log_msg(f"[{traj}] 使用电流估算力矩 (current torque) 进行辨识")
        if traj == "traj1" and self.identification_algorithm == IDENTIFICATION_AF_ARF:
            return self.process_acceleration_free_data(
                traj, t_raw, q_raw, tau_current_raw, tau_sensor_raw, fc, si, sf
            )
        if len(t_raw) <= si - sf:
            raise ValueError("采集数据不足，裁剪前 3 秒和后 1 秒后没有可用样本。")
        purpose = "统一验证" if traj == "traj2" else "传统回归辨识"
        filter_torque = traj == "traj1" and self.filter_identification_torque
        torque_description = "电流力矩使用与 q 相同的三阶 Butterworth 零相位滤波" if filter_torque else "电流力矩保持原始值（仅裁剪）"
        self.log_msg(f"[{traj}] {purpose}：q/dq/ddq 滤波，{torque_description}。")
        q, dq, ddq, tau = self.diff_and_filt_data(
            dof=self.ndof,
            h=self.dt,
            q_raw=q_raw,
            tau_raw=tau_raw,
            fc_q=fc,
            fc_dq=fc,
            fc_ddq=fc,
            fc_tau=fc if filter_torque else None,
        )
        t_raw = t_raw[si:sf]
        q_raw = q_raw[si:sf]
        tau_current_raw = tau_current_raw[si:sf]
        tau_sensor_raw = tau_sensor_raw[si:sf]
        q = q[si:sf]
        dq = dq[si:sf]
        ddq = ddq[si:sf]
        tau = tau[si:sf]
        t_raw = t_raw - t_raw[0]
        t = np.array(range(q.shape[0])) * self.dt
        W, omega, Q1, R1, rho1 = self.gen_regr_matrices(self.rbt, q, dq, ddq, tau)
        return t_raw, q_raw, tau_current_raw, tau_sensor_raw, t, q, dq, ddq, tau, W, omega, Q1, R1, rho1, None, None

    def process_acceleration_free_data(self, traj, t_raw, q_raw, tau_current, tau_sensor, fc, si, sf):
        if len(t_raw) <= si - sf:
            raise ValueError("采集数据不足，裁剪前 3 秒和后 1 秒后没有可用样本。")
        self.log_msg(
            f"[{traj}] AF-ARF：不重采样，按固定 dt={self.dt:g}s 处理；"
            "q 原始位置滤波，dq 按传统回归先差分后滤波。"
        )
        t = np.arange(len(q_raw), dtype=float) * self.dt
        q = np.column_stack([
            self.butter_filtfilt(3, fc * 2 * np.pi * self.dt, q_raw[:, j])
            for j in range(self.ndof)
        ])
        dq = np.column_stack([
            self.butter_filtfilt(
                3, fc * 2 * np.pi * self.dt,
                self.central_diff(q_raw[:, j], self.dt, 2),
            ) for j in range(self.ndof)
        ])
        friction_symbols = set(self.rbt.rbtdef.fc + self.rbt.rbtdef.fv + self.rbt.rbtdef.fo)
        friction_indices = [
            j for j, parameter in enumerate(self.rbt.dyn.dynparms)
            if parameter in friction_symbols
        ]
        self.log_msg(
            f"[{traj}] AF-ARF: alpha={self.af_alpha:g}, beta={self.af_beta:g} s^-1，"
            "构造无加速度回归矩阵（不计算 ddq）..."
        )
        tau_input = tau_current
        if self.filter_identification_torque:
            tau_input = np.column_stack([
                self.butter_filtfilt(3, fc * 2 * np.pi * self.dt, tau_current[:, joint])
                for joint in range(self.ndof)
            ])
            self.log_msg(f"[{traj}] 电流力矩先使用与 q 相同的三阶 Butterworth 零相位滤波，再进行 AF-ARF 方程滤波。")
        else:
            self.log_msg(f"[{traj}] 原始电流力矩直接进入 AF-ARF 方程滤波，不额外进行 Butterworth 滤波。")
        H, tau = build_filtered_regressor(
            self.create_regressor_func(self.rbt, self.friction_options), t, q, dq, tau_input,
            self.rbt.dyn.n_dynparms, friction_indices,
            alpha=self.af_alpha, beta=self.af_beta,
        )
        tau_sensor_filtered = lowpass_filter(t, tau_sensor, self.af_alpha)
        if self.af_beta > 0:
            tau_sensor_filtered = lowpass_filter(t, tau_sensor_filtered, self.af_beta)
        keep = slice(si, sf)
        friction_context = {
            "t": t, "q": q, "dq": dq, "keep": keep,
            "alpha": self.af_alpha, "beta": self.af_beta,
        }
        t = t[keep] - t[si]
        q, dq, tau = q[keep], dq[keep], tau[keep]
        W, omega, Q1, R1, rho1 = self.decompose_regression(
            self.rbt, H[keep].reshape(-1, self.rbt.dyn.n_dynparms), tau
        )
        return (
            t_raw[keep] - t_raw[si], q_raw[keep], tau_current[keep], tau_sensor[keep],
            t, q, dq, None, tau, W, omega, Q1, R1, rho1,
            tau_sensor_filtered[keep],
            friction_context,
        )

    # 惯性矩阵M(q)正定性测试: M(q)理论上应为对称正定矩阵，否则对应的动能不满足正定性，模型物理不可行, 因此需要验证辨识结果对应的质量矩阵是否为正定矩阵）
    def test_mass_matrix_psd(self, beta_solution):
        self.log_msg("开始进行惯性矩阵正定性测试...")
        K = self.rbt.dyn.Pb.T + self.rbt.dyn.Kd * self.rbt.dyn.Pd.T
        A = np.matrix(K).astype(float)
        b = np.matrix(beta_solution).astype(float)
        x = np.linalg.pinv(A) * b
        d_test = x
        M_func_def = sympybotics.robotcodegen.robot_code_to_func(
            "python", self.rbt.M_code, "M", "M_func", self.rbt.rbtdef
        )
        exec(M_func_def, globals())
        global sin, cos
        from math import sin, cos

        ntests = 10000
        ok = 0
        nok = 0
        for i in range(ntests):
            q_test = np.random.rand(self.rbt.dof)
            M_out = M_func(np.array(d_test).flatten().tolist(), q_test)
            M = np.matrix(M_out).reshape((self.rbt.dof, self.rbt.dof))
            try:
                c = np.linalg.cholesky(M)
            except:
                nok += 1
                continue
            ok += 1
        print("ok     %8i  %3i%%" % (ok, 100.0 * ok / ntests))
        print("not ok %8i  %3i%%" % (nok, 100.0 * nok / ntests))
        if nok > 0:
            self.log_msg(
                "惯性矩阵正定性测试未通过，存在不正定情况！请检查辨识数据及结果。"
            )
            return False
        else:
            self.log_msg("惯性矩阵正定性测试通过。")
        return True

    def dsdp5(self, objf, lmis, variables):
        # 生成 SDPA 稀疏格式输入
        sdpadat = lmi_sdp.to_sdpa_sparse(objf, lmis, variables)
        import tempfile

        epsilon_sdptol = 1e-7
        # 使用临时目录，避免在项目目录中持久化文件
        with tempfile.TemporaryDirectory() as tmpdir:
            sdpa_in = os.path.join(tmpdir, "sdp.dat-s")
            dsdp_out = os.path.join(tmpdir, "dsdp5.out")
            with open(sdpa_in, "w") as f:
                f.write(sdpadat)
            cmd = [
                "dsdp5",
                os.path.basename(sdpa_in),
                "-save",
                os.path.basename(dsdp_out),
                "-gaptol",
                str(epsilon_sdptol),
            ]
            proc = subprocess.run(
                cmd,
                cwd=tmpdir,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                check=True,
            )
            with open(dsdp_out, "r") as f:
                first_line = f.readline().strip()
        sol = [float(v) for v in first_line.split()]
        return np.matrix(sol).T

    @staticmethod
    def create_regressor_func(rbt, friction_options=None):
        namespace = {"math": math, "sin": np.sin, "cos": np.cos, "sign": smooth_sign}
        code = sympybotics.robot_code_to_func(
            "python", rbt.H_code, "H", "regressor_func", rbt.rbtdef
        )
        code = re.sub(
            r"\(0\.0 if (dq\[\d+\]) == 0 else math\.copysign\(1, \1\)\)",
            r"sign(\1)", code,
        )
        if "copysign" in code:
            raise ValueError("Unsupported Coulomb expression in generated Python regressor")
        exec(code, namespace)
        generated = namespace["regressor_func"]
        options = friction_settings(friction_options or {})
        names = list(map(str, rbt.dyn.dynparms))
        fc_indices = [names.index(str(symbol)) for symbol in rbt.rbtdef.fc]
        fo_indices = [names.index(str(symbol)) for symbol in rbt.rbtdef.fo]

        def regressor(q, dq, ddq):
            matrix = np.asarray(generated(q, dq, ddq), dtype=float).reshape(rbt.dof, rbt.dyn.n_dynparms)
            fc_basis, fo_basis = friction_bases(
                dq, options["friction_model"], options["friction_velocity_scale"],
                options["stribeck_velocity"], options["stribeck_alpha"],
            )
            for joint, (fc_index, fo_index) in enumerate(zip(fc_indices, fo_indices)):
                matrix[joint, fc_index] = fc_basis[joint]
                matrix[joint, fo_index] = fo_basis[joint]
            return matrix.reshape(-1).tolist()

        return regressor

    def gen_regr_matrices(self, rbt, q, dq, ddq, tau):
        regressor_func = self.create_regressor_func(rbt, self.friction_options)
        H_S, omega = self.regr_matrices(
            rbt.dof, rbt.dyn.n_dynparms, q, dq, ddq, tau, regressor_func
        )
        return self.decompose_regression(rbt, H_S, tau)

    @staticmethod
    def decompose_regression(rbt, H_S, tau):
        W = np.matrix(H_S[:, rbt.dyn.base_idxs])
        omega = np.matrix(np.asarray(tau).reshape(-1, 1))
        Q1, R1 = np.linalg.qr(W)
        rho1 = Q1.T * omega
        return W, omega, Q1, R1, rho1

    def regr_matrices(self, dof, parm_num, q, dq, ddq, tau, regr_func):
        sn = q.shape[0]
        # 向量化构造 H_S，每次调用返回 (dof, parm_num)，再堆叠
        blocks = [
            np.array(regr_func(q[i], dq[i], ddq[i])).reshape(dof, parm_num)
            for i in range(sn)
        ]
        H_S = np.matrix(np.vstack(blocks))
        tau_S = np.matrix(tau.reshape(dof * sn, 1))
        return H_S, tau_S

    def diff_and_filt_data(self, dof, h, q_raw, tau_raw, fc_q, fc_dq, fc_ddq, fc_tau):
        q = np.zeros_like(q_raw)
        dq = np.zeros_like(q_raw)
        ddq = np.zeros_like(q_raw)
        tau = np.zeros_like(tau_raw)
        wc_q = fc_q * 2 * math.pi * h
        wc_dq = fc_dq * 2 * math.pi * h
        wc_ddq = fc_ddq * 2 * math.pi * h
        wc_tau = None if fc_tau is None else fc_tau * 2 * math.pi * h

        for i in range(dof):
            q_i = q_raw[:, i]
            q[:, i] = self.butter_filtfilt(3, wc_q, q_i)
            dq_raw_i = self.central_diff(q_i, h, 2)
            dq[:, i] = self.butter_filtfilt(3, wc_dq, dq_raw_i)
            ddq_raw_i = self.central_diff(dq_raw_i, h, 2)
            ddq[:, i] = self.butter_filtfilt(3, wc_ddq, ddq_raw_i)
            tau[:, i] = self.butter_filtfilt(3, wc_tau, tau_raw[:, i])
        return q, dq, ddq, tau

    def butter_filtfilt(self, N, Wn, signal):
        # 安全处理非有限/不可转换的截止频率：直接不滤波
        try:
            if Wn is None:
                return signal[:]
            Wn_val = float(Wn)
            if not np.isfinite(Wn_val):
                return signal[:]
            if not (0.0 < Wn_val < 1.0):
                return signal[:]
        except Exception:
            return signal[:]

        b, a = scipy.signal.butter(N, Wn_val)
        return scipy.signal.filtfilt(b, a, signal, method="pad")

    def filtfilt(self, b, a, x):
        ntaps = max(len(a), len(b))
        edge = ntaps * 3
        if x.ndim != 1:
            raise ValueError("Filiflit is only accepting 1 dimension arrays.")
        if x.size < edge:
            raise ValueError(
                "Input vector needs to be bigger than 3 * max(len(a),len(b)."
            )
        if len(a) < ntaps:
            a = np.r_[a, np.zeros(len(b) - len(a))]
        if len(b) < ntaps:
            b = np.r_[b, np.zeros(len(a) - len(b))]
        zi = scipy.signal.lfilter_zi(b, a)
        s = np.r_[2 * x[0] - x[edge:1:-1], x, 2 * x[-1] - x[-1:-edge:-1]]
        (y, zf) = scipy.signal.lfilter(b, a, s, -1, zi * s[0])
        (y, zf) = scipy.signal.lfilter(b, a, np.flipud(y), -1, zi * y[-1])
        return np.flipud(y[edge - 1 : -edge + 1])

    def central_diff(self, array, div, n=2):
        size = len(array)
        diff = np.zeros_like(array)
        if n == 1:
            diff[0] = (array[1] - array[0]) / div
            for i in range(1, size - 1):
                diff[i] = (array[i + 1] - array[i - 1]) / (2 * div)
            diff[size - 1] = (array[size - 1] - array[size - 2]) / div
        elif n == 2:
            diff[0] = (array[1] - array[0]) / div
            diff[1] = (array[2] - array[0]) / (2 * div)
            for i in range(2, size - 2):
                diff[i] = (
                    -array[i + 2] + 8 * array[i + 1] - 8 * array[i - 1] + array[i - 2]
                ) / (12 * div)
            diff[size - 2] = (array[size - 1] - array[size - 3]) / (2 * div)
            diff[size - 1] = (array[size - 1] - array[size - 2]) / div
        else:
            raise Exception("use n = 1 or 2")
        return diff

    def get_regr_validatation(self):
        """Evaluate full instantaneous dynamics on filtered states against raw torque."""
        # 使用内存缓存的数据与解进行验证（默认 traj2 验证，参数来自 traj1）
        if ("traj2" not in self.proc_data or self.proc_data["traj2"].get("zero_acceleration_only")) and self.rbt is not None and self.solutions_beta_starextra is not None:
            if self.recording_path("traj2").is_file():
                self.cache_validation_data()
        traj_proc = self.proc_data.get("traj2")
        if traj_proc is None:
            raise RuntimeError("未找到 traj2 验证数据，请采集或导入独立验证轨迹后重新计算。")
        W = np.matrix(traj_proc["W"])
        t = traj_proc["t"]
        tau_current = np.matrix(traj_proc["tau_current"])
        tau_sensor = np.matrix(traj_proc["tau_sensor"])
        tau_used = np.matrix(traj_proc["tau"])
        if self.solutions_beta_starextra is None:
            raise RuntimeError("未找到辨识结果，请先计算动力学参数。")
        omega_pred = W * np.matrix(self.solutions_beta_starextra)
        tau_pred = omega_pred.reshape(
            int(omega_pred.shape[0] / tau_used.shape[1]), tau_used.shape[1]
        )
        total = tau_used.shape[0]
        return (
            t[:total],
            tau_current[:total, :],
            tau_sensor[:total, :],
            tau_pred[:total, :],
            tau_used[:total, :],
        )

    def get_validation_friction_rules(self, scales=None):
        rules = read_runtime_friction(
            RUNTIME_INVERSE_DYNAMICS_SOURCE, self.robot_type, self.rbt.dyn.baseparms,
            config_path=Path.home() / "dexe_robot_server_data/robot_config.yaml", is_left=self.is_left,
        )
        if rules.model is not None:
            options = self.friction_options
            rules = replace(
                rules, model=options["friction_model"], velocity_scale=options["friction_velocity_scale"],
                stribeck_velocity=options["stribeck_velocity"], stribeck_alpha=options["stribeck_alpha"],
            )
        if scales is None:
            return rules
        scales = np.asarray(scales, dtype=float)
        if scales.shape != (3,) or not np.all(np.isfinite(scales)) or np.any(scales < 0):
            raise ValueError("摩擦比例要求三个有限非负数：J1–J5 fv、J6–J7 fv、fc。")
        names = list(map(str, self.rbt.dyn.baseparms))
        viscous = {
            index: float(scales[0 if int(names[index].split('_')[-1]) <= 5 else 1])
            for index in rules.viscous
        }
        return replace(rules, viscous=viscous, coulomb_scale=float(scales[2]))

    def get_friction_ratio_validation(self, rules=None):
        """Compare normal and runtime-friction-scaled predictions on traj2."""
        t, tau_current, tau_sensor, tau_pred, tau_used = self.get_regr_validatation()
        if rules is None:
            rules = self.get_validation_friction_rules()
        proc = self.proc_data["traj2"]
        scaled_pred = predict_runtime_friction(
            proc["W"], self.solutions_beta_starextra, proc["dq"], rules,
            proc["friction_context"],
        )
        return t, tau_current, tau_sensor, tau_pred, scaled_pred, tau_used, rules

    def get_zero_acceleration_validation(self, rules=None):
        """Evaluate instantaneous g + c + friction against the same raw validation torque."""
        if self.solutions_beta_starextra is None or self.rbt is None:
            raise RuntimeError("未找到辨识结果，请先计算动力学参数。")
        if "traj2" not in self.proc_data:
            self._cache_zero_acceleration_validation_data()
        proc = self.proc_data["traj2"]
        if rules is None:
            rules = self.get_validation_friction_rules()
        # Pb embeds a representative full parameter vector with the same dynamics.
        full_parameters = np.asarray(self.rbt.dyn.Pb, dtype=float) @ np.asarray(
            self.solutions_beta_starextra, dtype=float
        ).reshape(-1)
        if self._feedforward_functions is None:
            functions = []
            for name, code in (("gravity", self.rbt.g_code), ("coriolis", self.rbt.c_code)):
                namespace = {"math": math, "sin": np.sin, "cos": np.cos}
                exec(sympybotics.robot_code_to_func(
                    "python", code, "torque", name, self.rbt.rbtdef
                ), namespace)
                functions.append(namespace[name])
            self._feedforward_functions = functions
        gravity, coriolis = self._feedforward_functions
        q, dq = np.asarray(proc["q"]), np.asarray(proc["dq"])
        rigid = np.array([
            np.asarray(gravity(full_parameters, position)) +
            np.asarray(coriolis(full_parameters, position, velocity))
            for position, velocity in zip(q, dq)
        ])
        full_names = list(self.rbt.dyn.dynparms)
        fv, fc, fo = [
            full_parameters[[full_names.index(symbol) for symbol in symbols]]
            for symbols in (self.rbt.rbtdef.fv, self.rbt.rbtdef.fc, self.rbt.rbtdef.fo)
        ]
        ratios = np.array([
            rules.viscous[list(map(str, self.rbt.dyn.baseparms)).index(str(symbol))]
            for symbol in self.rbt.rbtdef.fv
        ])
        options = self.friction_options
        fc_basis, fo_basis = friction_bases(
            dq, options["friction_model"], options["friction_velocity_scale"],
            options["stribeck_velocity"], options["stribeck_alpha"],
        )
        baseline = rigid + dq * fv + fc_basis * fc + fo_basis * fo
        runtime_sign, runtime_offset = rules.bases(dq)
        offset_scale = rules.coulomb_scale if rules.model == FRICTION_MODEL_STRIBECK else 1.0
        runtime = rigid + dq * fv * ratios + runtime_sign * fc * rules.coulomb_scale + runtime_offset * fo * offset_scale
        return proc["t"], proc["tau_current"], proc["tau_sensor"], baseline, runtime, proc["tau"], rules

    def get_all_validation_predictions(self, friction_scales=None):
        """Return the four torque conditions used by the Qt comparison button."""
        if self.rbt is None or self.solutions_beta_starextra is None:
            raise RuntimeError("请先完成动力学参数辨识。")
        rules = self.get_validation_friction_rules(friction_scales)
        full = self.get_friction_ratio_validation(rules)
        zero = self.get_zero_acceleration_validation(rules)
        return {
            "t_full": np.asarray(full[0]),
            "measured_full": np.asarray(full[5]),
            "full_unscaled": np.asarray(full[3]),
            "full_scaled": np.asarray(full[4]),
            "t_zero": np.asarray(zero[0]),
            "measured_zero": np.asarray(zero[5]),
            "zero_unscaled": np.asarray(zero[3]),
            "zero_scaled": np.asarray(zero[4]),
            "rules": full[6],
        }


class DynParamGUI(QtWidgets.QMainWindow):
    log_signal = QtCore.pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("动力学参数辨识")
        self.resize(1920, 1080)

        # 中央 widget 和布局
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        main_layout = QtWidgets.QVBoxLayout(central)
        top_container = QtWidgets.QWidget()
        top_layout = QtWidgets.QHBoxLayout(top_container)
        top_layout.setContentsMargins(0, 0, 0, 0)
        top_layout.setSpacing(8)

        # 左侧控制面板
        ctrl_widget = QtWidgets.QWidget()
        ctrl_layout = QtWidgets.QVBoxLayout(ctrl_widget)
        ctrl_layout.setContentsMargins(0, 0, 0, 0)
        ctrl_layout.setSpacing(10)

        self.traj_combo = QtWidgets.QComboBox()
        self.traj_combo.addItems(["轨迹1", "轨迹2"])
        self.traj_combo.setCurrentText("轨迹1")
        self.arm_combo = QtWidgets.QComboBox()
        self.arm_combo.addItems(["左臂", "右臂"])
        self.arm_combo.setCurrentText("左臂")

        app_pt = QtWidgets.QApplication.font().pointSize()
        cf = self.arm_combo.font()
        cf.setPointSize(app_pt)
        self.arm_combo.setFont(cf)
        tf = self.traj_combo.font()
        tf.setPointSize(app_pt)
        self.traj_combo.setFont(tf)
        arm_view = QtWidgets.QListView()
        arm_view.setFont(cf)
        self.arm_combo.setView(arm_view)
        traj_view = QtWidgets.QListView()
        traj_view.setFont(tf)
        self.traj_combo.setView(traj_view)

        # 第一部分：轨迹仿真检查
        check_group = QtWidgets.QGroupBox("轨迹仿真检查")
        check_vbox = QtWidgets.QVBoxLayout(check_group)
        check_vbox.setContentsMargins(8, 8, 8, 8)
        check_vbox.setSpacing(8)
        combo_row = QtWidgets.QWidget()
        combo_row_layout = QtWidgets.QHBoxLayout(combo_row)
        combo_row_layout.setContentsMargins(0, 0, 0, 0)
        combo_row_layout.setSpacing(8)
        combo_row_layout.addWidget(self.arm_combo)
        combo_row_layout.addWidget(self.traj_combo)
        check_vbox.addWidget(combo_row)
        # 操作按钮（可视化与仿真）
        self.btn_open_meshcat = QtWidgets.QPushButton("打开可视化")
        self.btn_run_sim = QtWidgets.QPushButton("仿真运行待采集轨迹")
        self.btn_show_reference = QtWidgets.QPushButton("图示待采集轨迹")
        # 仿真倍速选择（与运行按钮同一行）
        self.sim_speed_combo = QtWidgets.QComboBox()
        self.sim_speed_combo.addItems(["x1倍速", "x2倍速"])
        self.sim_speed_combo.setCurrentText("x1倍速")
        self.sim_speed_combo.setSizeAdjustPolicy(QtWidgets.QComboBox.AdjustToContents)
        fm = self.sim_speed_combo.fontMetrics()
        max_text = max(["x1倍速", "x2倍速"], key=lambda s: fm.width(s))
        width = fm.width(max_text) + 36  # 预留左右内边距与下拉箭头
        self.sim_speed_combo.setFixedWidth(width)

        self.btn_show_plot = QtWidgets.QPushButton("图示已采集轨迹")
        self.btn_show_regression = QtWidgets.QPushButton("图示参数验证结果")
        btn_h = max(
            self.btn_run_sim.sizeHint().height(), self.btn_show_plot.sizeHint().height()
        )
        self.arm_combo.setFixedHeight(btn_h)
        self.traj_combo.setFixedHeight(btn_h)
        self.arm_combo.setStyleSheet(
            f"QComboBox QAbstractItemView::item{{height:{btn_h - 6}px;}}"
        )
        self.traj_combo.setStyleSheet(
            f"QComboBox QAbstractItemView::item{{height:{btn_h - 6}px;}}"
        )
        # 仿真倍速下拉框与按钮高度一致
        self.sim_speed_combo.setFixedHeight(btn_h)
        self.sim_speed_combo.setStyleSheet(
            f"QComboBox QAbstractItemView::item{{height:{btn_h - 6}px;}}"
        )
        check_vbox.addWidget(self.btn_open_meshcat)
        check_vbox.addWidget(self.btn_show_reference)
        # 将“运行仿真”与倍速下拉框放在同一行
        sim_row = QtWidgets.QWidget()
        sim_row_layout = QtWidgets.QHBoxLayout(sim_row)
        sim_row_layout.setContentsMargins(0, 0, 0, 0)
        sim_row_layout.setSpacing(8)
        sim_row_layout.addWidget(self.btn_run_sim)
        sim_row_layout.addWidget(self.sim_speed_combo)
        check_vbox.addWidget(sim_row)
        ctrl_layout.addWidget(check_group)
        self.traj_combo.currentTextChanged.connect(self.update_params_from_ui)
        self.arm_combo.currentTextChanged.connect(self.update_params_from_ui)
        # 停止按钮与清错按钮
        self.btn_stop = QtWidgets.QPushButton("停止运行")
        self.btn_clear = QtWidgets.QPushButton("清除错误")
        ctrl_layout.addStretch(1)
        # 第二部分：真机采集与控制
        bottom_group = QtWidgets.QGroupBox("轨迹真机采集")
        bottom_vbox = QtWidgets.QVBoxLayout(bottom_group)
        bottom_vbox.setContentsMargins(8, 8, 8, 8)
        bottom_vbox.setSpacing(8)
        self.btn_run_real = QtWidgets.QPushButton("真机采集双臂轨迹")
        
        # 真机采集倍速选择
        self.real_speed_combo = QtWidgets.QComboBox()
        self.real_speed_combo.addItems(["x1倍速", "x0.5倍速"])
        self.real_speed_combo.setCurrentText("x1倍速")
        self.real_speed_combo.setSizeAdjustPolicy(QtWidgets.QComboBox.AdjustToContents)
        fm_real = self.real_speed_combo.fontMetrics()
        max_text_real = max(["x1倍速", "x0.5倍速"], key=lambda s: fm_real.width(s))
        width_real = fm_real.width(max_text_real) + 36
        self.real_speed_combo.setFixedWidth(width_real)
        self.real_speed_combo.setFixedHeight(btn_h)
        self.real_speed_combo.setStyleSheet(
            f"QComboBox QAbstractItemView::item{{height:{btn_h - 6}px;}}"
        )

        real_row = QtWidgets.QWidget()
        real_row_layout = QtWidgets.QHBoxLayout(real_row)
        real_row_layout.setContentsMargins(0, 0, 0, 0)
        real_row_layout.setSpacing(8)
        real_row_layout.addWidget(self.btn_run_real)
        real_row_layout.addWidget(self.real_speed_combo)
        bottom_vbox.addWidget(real_row)

        self.btn_run_real_left = QtWidgets.QPushButton("真机采集左臂")
        self.btn_run_real_right = QtWidgets.QPushButton("真机采集右臂")
        self.real_arm_speed_combo = QtWidgets.QComboBox()
        self.real_arm_speed_combo.addItems(["x1倍速", "x0.5倍速"])
        self.real_arm_speed_combo.setCurrentText("x1倍速")
        self.real_arm_speed_combo.setSizeAdjustPolicy(QtWidgets.QComboBox.AdjustToContents)
        fm_arm = self.real_arm_speed_combo.fontMetrics()
        max_text_arm = max(["x1倍速", "x0.5倍速"], key=lambda s: fm_arm.width(s))
        width_arm = fm_arm.width(max_text_arm) + 36
        self.real_arm_speed_combo.setFixedWidth(width_arm)
        self.real_arm_speed_combo.setFixedHeight(btn_h)
        self.real_arm_speed_combo.setStyleSheet(
            f"QComboBox QAbstractItemView::item{{height:{btn_h - 6}px;}}"
        )
        real_arm_row = QtWidgets.QWidget()
        real_arm_row_layout = QtWidgets.QHBoxLayout(real_arm_row)
        real_arm_row_layout.setContentsMargins(0, 0, 0, 0)
        real_arm_row_layout.setSpacing(8)
        real_arm_row_layout.addWidget(self.btn_run_real_left)
        real_arm_row_layout.addWidget(self.btn_run_real_right)
        real_arm_row_layout.addWidget(self.real_arm_speed_combo)
        bottom_vbox.addWidget(real_arm_row)

        self.btn_calc_params = QtWidgets.QPushButton("计算双臂动力学参数")
        self.btn_calc_params_left = QtWidgets.QPushButton("计算左臂动力学参数")
        self.btn_calc_params_right = QtWidgets.QPushButton("计算右臂动力学参数")
        self.btn_write_params = QtWidgets.QPushButton("写入双臂辨识参数")
        self.btn_write_params_left = QtWidgets.QPushButton("写入左臂辨识参数")
        self.btn_write_params_right = QtWidgets.QPushButton("写入右臂辨识参数")
        # 图示已采集轨迹放在真机采集所有轨迹下方
        bottom_vbox.addWidget(self.btn_show_plot)
        # 离线/异地辨识：手动指定采集数据目录（默认使用 recdata）
        self.btn_select_data_dir = QtWidgets.QPushButton("选择本地数据目录")
        bottom_vbox.addWidget(self.btn_select_data_dir)
        self.data_dir_label = QtWidgets.QLabel()
        self.data_dir_label.setWordWrap(True)
        bottom_vbox.addWidget(self.data_dir_label)
        file_grid = QtWidgets.QGridLayout()
        self.recording_file_buttons = []
        self.recording_file_labels = {}
        file_grid.addWidget(QtWidgets.QLabel("离线数据"), 0, 0)
        file_grid.addWidget(QtWidgets.QLabel("辨识"), 0, 1)
        file_grid.addWidget(QtWidgets.QLabel("验证"), 0, 2)
        for row, (side, title) in enumerate((("left", "左臂"), ("right", "右臂")), 1):
            file_grid.addWidget(QtWidgets.QLabel(title), row, 0)
            for column, traj in enumerate(("traj1", "traj2"), 1):
                button = QtWidgets.QPushButton("选择文件")
                button.clicked.connect(
                    lambda _checked=False, arm=side, role=traj: self.on_select_recording_file(arm, role)
                )
                file_grid.addWidget(button, row, column)
                self.recording_file_buttons.append(button)
        bottom_vbox.addLayout(file_grid)
        for side, title in (("left", "左臂"), ("right", "右臂")):
            for traj, role in (("traj1", "辨识"), ("traj2", "验证")):
                label = QtWidgets.QLabel()
                label.setWordWrap(True)
                label.setMinimumWidth(0)
                self.recording_file_labels[side, traj] = label
                bottom_vbox.addWidget(label)
        ctrl_layout.addWidget(bottom_group)
        ctrl_layout.addStretch(1)
        # 第三部分：轨迹采集及计算结果
        result_group = QtWidgets.QGroupBox("计算参数与写入")
        result_vbox = QtWidgets.QVBoxLayout(result_group)
        result_vbox.setContentsMargins(8, 8, 8, 8)
        result_vbox.setSpacing(8)
        identification_form = QtWidgets.QFormLayout()
        self.friction_model_combo = QtWidgets.QComboBox()
        self.friction_model_combo.addItem("经典库仑 + 黏性 + 静摩擦（偏置）", FRICTION_MODEL_CLASSIC)
        self.friction_model_combo.addItem("平滑库仑 + 黏性 + 静摩擦（偏置）", FRICTION_MODEL_SMOOTH)
        self.friction_model_combo.addItem("Stribeck + 黏性", FRICTION_MODEL_STRIBECK)
        options = friction_settings({})
        self.friction_model_combo.setCurrentIndex(self.friction_model_combo.findData(options["friction_model"]))
        self.friction_model_combo.setToolTip(
            "经典/平滑模型: Fc·sign(v) 或 Fc·tanh(v/v0) + Bv·v + 偏置。\n"
            "Stribeck: [Fc+(Fs-Fc)·exp(-(|v|/vs)^alpha)]·tanh(v/v0)+Bv·v。\n"
            "特征速度 vs 控制指数衰减，tanh 平滑速度 v0 控制方向过渡，可分别在 Qt 或 YAML 修改。\n"
            "模型参数改变后须重新辨识。"
        )
        identification_form.addRow("摩擦模型", self.friction_model_combo)
        self.stribeck_alpha_spin = QtWidgets.QDoubleSpinBox()
        self.stribeck_alpha_spin.setRange(0.1, 10.0)
        self.stribeck_alpha_spin.setDecimals(2)
        self.stribeck_alpha_spin.setValue(options["stribeck_alpha"])
        self.stribeck_alpha_spin.setEnabled(options["friction_model"] == FRICTION_MODEL_STRIBECK)
        identification_form.addRow("Stribeck 指数 α", self.stribeck_alpha_spin)
        self.friction_velocity_spins = {}
        for key, label, tooltip in (
            ("friction_velocity_scale", "tanh 平滑速度", "平滑库仑及 Stribeck 方向项 tanh(v/v0) 的 v0，与 Stribeck 特征速度独立，必须大于 0。"),
            ("stribeck_velocity", "Stribeck 特征速度", "指数衰减 exp(-(|v|/vs)^alpha) 的 vs，与 tanh 平滑速度独立，必须大于 0。"),
        ):
            spin = QtWidgets.QDoubleSpinBox()
            spin.setDecimals(6)
            spin.setRange(0.000001, 1000.0)
            spin.setSingleStep(0.01)
            spin.setSuffix(" rad/s")
            spin.setValue(options[key])
            spin.setToolTip(tooltip + " 修改后须重新辨识双臂；写入参数时同时保存到 YAML。")
            spin.setEnabled(options["friction_model"] in (
                (FRICTION_MODEL_SMOOTH, FRICTION_MODEL_STRIBECK) if key == "friction_velocity_scale"
                else (FRICTION_MODEL_STRIBECK,)
            ))
            self.friction_velocity_spins[key] = spin
            identification_form.addRow(label, spin)
        self.identification_combo = QtWidgets.QComboBox()
        self.identification_combo.addItem("传统回归（使用 ddq）", IDENTIFICATION_STANDARD)
        self.identification_combo.addItem("AF-ARF（无需 ddq）", IDENTIFICATION_AF_ARF)
        identification_form.addRow("辨识算法", self.identification_combo)
        self.filter_torque_checkbox = QtWidgets.QCheckBox("辨识力矩滤波（与 q 相同）")
        self.filter_torque_checkbox.setChecked(False)
        self.filter_torque_checkbox.setToolTip(
            "仅对辨识集电流估算力矩使用与 q 相同的三阶 Butterworth 零相位滤波及截止频率。"
            "默认关闭，验证集始终使用原始力矩。AF-ARF 自身的 alpha/beta 方程滤波不受此开关影响。"
        )
        identification_form.addRow(self.filter_torque_checkbox)
        self.af_alpha_spin = QtWidgets.QDoubleSpinBox()
        self.af_alpha_spin.setRange(0.1, 500.0)
        self.af_alpha_spin.setDecimals(1)
        self.af_alpha_spin.setValue(15.0)
        self.af_alpha_spin.setSuffix(" s^-1")
        self.af_alpha_spin.setToolTip("AF-ARF 第一级低通速率 alpha，截止频率为 alpha / (2*pi) Hz")
        self.af_beta_spin = QtWidgets.QDoubleSpinBox()
        self.af_beta_spin.setRange(0.0, 500.0)
        self.af_beta_spin.setDecimals(1)
        self.af_beta_spin.setValue(10.0)
        self.af_beta_spin.setSuffix(" s^-1")
        self.af_beta_spin.setSpecialValueText("关闭")
        self.af_beta_spin.setToolTip("第二级低通速率 beta；设为 0 关闭第二级滤波")
        identification_form.addRow("滤波 alpha", self.af_alpha_spin)
        identification_form.addRow("滤波 beta", self.af_beta_spin)
        result_vbox.addLayout(identification_form)
        result_vbox.addWidget(self.btn_calc_params)
        calc_arm_row = QtWidgets.QWidget()
        calc_arm_row_layout = QtWidgets.QHBoxLayout(calc_arm_row)
        calc_arm_row_layout.setContentsMargins(0, 0, 0, 0)
        calc_arm_row_layout.setSpacing(8)
        calc_arm_row_layout.addWidget(self.btn_calc_params_left)
        calc_arm_row_layout.addWidget(self.btn_calc_params_right)
        result_vbox.addWidget(calc_arm_row)
        result_vbox.addWidget(self.btn_show_regression)
        self.use_source_friction = QtWidgets.QCheckBox("使用 YAML 摩擦比例（取消勾选可离线调节）")
        self.use_source_friction.setChecked(True)
        result_vbox.addWidget(self.use_source_friction)
        friction_form = QtWidgets.QFormLayout()
        self.friction_scale_spins = []
        for label, default in (("J1–J5 黏性 fv 比例", 0.6),
                               ("J6–J7 黏性 fv 比例", 0.5),
                               ("全部关节库仑 fc 比例", 0.2)):
            spin = QtWidgets.QDoubleSpinBox()
            spin.setRange(0.0, 10.0)
            spin.setDecimals(3)
            spin.setSingleStep(0.05)
            spin.setValue(default)
            spin.setEnabled(False)
            spin.setToolTip("左右臂共用；相对于原辨识摩擦系数的倍率。调整后点击对比，无需重新辨识。")
            self.friction_scale_spins.append(spin)
            friction_form.addRow(label, spin)
        result_vbox.addLayout(friction_form)
        self.use_source_friction.toggled.connect(
            lambda checked: [spin.setEnabled(not checked) for spin in self.friction_scale_spins]
        )
        self.btn_feedforward_validation = QtWidgets.QPushButton(
            "四种动力学工况 RMSE 对比"
        )
        self.btn_feedforward_validation.setToolTip(
            "一次输出完整动力学、ddq=0 前馈及两种摩擦缩放工况的逐关节 RMSE"
        )
        result_vbox.addWidget(self.btn_feedforward_validation)
        # 保留旧属性名作为兼容别名，但界面只有一个按钮。
        self.btn_friction_ratio_validation = self.btn_feedforward_validation
        self.btn_zero_acceleration_validation = self.btn_feedforward_validation
        self.btn_zero_acceleration_runtime_validation = self.btn_feedforward_validation
        result_vbox.addWidget(self.btn_write_params)
        write_arm_row = QtWidgets.QWidget()
        write_arm_row_layout = QtWidgets.QHBoxLayout(write_arm_row)
        write_arm_row_layout.setContentsMargins(0, 0, 0, 0)
        write_arm_row_layout.setSpacing(8)
        write_arm_row_layout.addWidget(self.btn_write_params_left)
        write_arm_row_layout.addWidget(self.btn_write_params_right)
        result_vbox.addWidget(write_arm_row)
        ctrl_layout.addWidget(result_group)
        ctrl_layout.addStretch(1)
        # 第四部分：停止与清错控制
        stop_group = QtWidgets.QGroupBox("停止与清错控制")
        stop_vbox = QtWidgets.QVBoxLayout(stop_group)
        stop_vbox.setContentsMargins(8, 8, 8, 8)
        stop_vbox.setSpacing(8)
        stop_vbox.addWidget(self.btn_clear)
        stop_vbox.addWidget(self.btn_stop)
        ctrl_layout.addWidget(stop_group)
        ctrl_layout.addStretch(1)
        ctrl_layout.addStretch(1)

        # 底部整行日志显示窗口（可拖动调整高度）
        self.log = QtWidgets.QTextEdit()
        self.log.setReadOnly(True)
        self.log.setMinimumHeight(120)
        # 使用垂直分割器，允许用户拖动调整上部与日志的高度
        splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        splitter.addWidget(top_container)
        splitter.addWidget(self.log)
        # 设置初始尺寸分配：上部占较多空间，日志占较少空间
        splitter.setSizes([500, 180])
        main_layout.addWidget(splitter)

        plot_widget = QtWidgets.QWidget()
        plot_layout = QtWidgets.QVBoxLayout(plot_widget)
        # 使用水平分割器容纳左侧控制与右侧绘图
        hsplitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        ctrl_layout.removeWidget(stop_group)
        ctrl_scroll = QtWidgets.QScrollArea()
        ctrl_scroll.setWidgetResizable(True)
        ctrl_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        ctrl_scroll.setWidget(ctrl_widget)
        ctrl_scroll.setMinimumWidth(
            ctrl_widget.minimumSizeHint().width()
            + self.style().pixelMetric(QtWidgets.QStyle.PM_ScrollBarExtent)
        )
        ctrl_panel = QtWidgets.QWidget()
        ctrl_panel_layout = QtWidgets.QVBoxLayout(ctrl_panel)
        ctrl_panel_layout.setContentsMargins(0, 0, 0, 0)
        ctrl_panel_layout.addWidget(ctrl_scroll, 1)
        ctrl_panel_layout.addWidget(stop_group)
        hsplitter.addWidget(ctrl_panel)
        hsplitter.addWidget(plot_widget)
        # 默认让左侧控制区域更宽一些，同时支持左右拖拽
        hsplitter.setSizes([350, 650])
        hsplitter.setStretchFactor(0, 0)
        hsplitter.setStretchFactor(1, 1)
        top_layout.addWidget(hsplitter)

        # 绘图区域分页：左臂/右臂（移除可视化页）
        self.tabs = QtWidgets.QTabWidget()
        self.tab_left = QtWidgets.QWidget()
        self.tab_right = QtWidgets.QWidget()
        self.tabs.addTab(self.tab_left, "左臂")
        self.tabs.addTab(self.tab_right, "右臂")

        self.left_fig = Figure(figsize=(8, 6), constrained_layout=True)
        self.left_canvas = FigureCanvas(self.left_fig)
        self.right_fig = Figure(figsize=(8, 6), constrained_layout=True)
        self.right_canvas = FigureCanvas(self.right_fig)

        left_layout = QtWidgets.QVBoxLayout(self.tab_left)
        right_layout = QtWidgets.QVBoxLayout(self.tab_right)
        left_layout.addWidget(self.left_canvas)
        right_layout.addWidget(self.right_canvas)
        # 初始化可视化地址（不嵌入，仅外部浏览器）
        self.meshcat_url = None
        plot_layout.addWidget(self.tabs)

        self.left_canvas.mpl_connect("resize_event", self._on_resize)
        self.right_canvas.mpl_connect("resize_event", self._on_resize)
        self.dyn = DynParamIdentifier(logger_callable=self.log_msg)
        self.filter_torque_checkbox.setChecked(self.dyn.calculator_left.filter_identification_torque)
        options = self.dyn.calculator_left.friction_options
        self.friction_model_combo.setCurrentIndex(self.friction_model_combo.findData(options["friction_model"]))
        self.stribeck_alpha_spin.setValue(options["stribeck_alpha"])
        self.stribeck_alpha_spin.setEnabled(options["friction_model"] == FRICTION_MODEL_STRIBECK)
        self._load_friction_velocity_controls(options)
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.dyn)
        self.ros_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.ros_thread.start()

        self.log_signal.connect(self._append_log)

        # 连接信号
        self.btn_open_meshcat.clicked.connect(self._start_visual_node)
        self.btn_run_sim.clicked.connect(self.on_run_sim)
        self.btn_run_real.clicked.connect(self.on_run_real)
        self.btn_run_real_left.clicked.connect(self.on_run_real_left)
        self.btn_run_real_right.clicked.connect(self.on_run_real_right)
        self.btn_show_reference.clicked.connect(self.on_show_reference)
        self.btn_show_plot.clicked.connect(self.on_show_plot)
        self.btn_select_data_dir.clicked.connect(self.on_select_data_dir)
        self.btn_calc_params.clicked.connect(self.on_calc_params)
        self.btn_calc_params_left.clicked.connect(self.on_calc_params_left)
        self.btn_calc_params_right.clicked.connect(self.on_calc_params_right)
        self.btn_write_params.clicked.connect(self.on_write_params)
        self.btn_write_params_left.clicked.connect(self.on_write_params_left)
        self.btn_write_params_right.clicked.connect(self.on_write_params_right)
        # 倍速选择变化
        self.sim_speed_combo.currentTextChanged.connect(self._on_sim_speed_changed)
        self.real_speed_combo.currentTextChanged.connect(self._on_real_speed_changed)
        self.real_arm_speed_combo.currentTextChanged.connect(self._on_real_arm_speed_changed)
        # 新增：停止按钮连接
        self.btn_stop.clicked.connect(self.on_stop_clicked)
        self.btn_clear.clicked.connect(self.on_clear_clicked)
        # 新增：图示辨识结果按钮
        self.btn_show_regression.clicked.connect(self.on_show_regression)
        self.btn_feedforward_validation.clicked.connect(self.on_feedforward_validation)
        # 可视化子进程句柄
        self._visual_proc = None
        # 外部浏览器打开防抖标志（避免多次打开同一地址）
        self._external_opened = False
        # 标志：由“切换到可视化页”触发的加载不自动外部打开
        self._suppress_external_fallback = False
        # 标志：已触发过WebGL失败回退，避免重复日志
        self._fallback_triggered = False
        # 记录上一次检测到的Meshcat地址，避免重复日志
        self._last_detected_url = None

        try:
            self.robot_label = QtWidgets.QLabel(
                f"机器人：{self.dyn.robot_name}_{self.dyn.version[:3]}"
            )
            lf = self.robot_label.font()
            lf.setBold(True)
            self.robot_label.setFont(lf)
            ctrl_layout.insertWidget(0, self.robot_label)
        except Exception:
            pass

        # 初始状态与禁用策略
        self._params_calculated = False
        self._calculation_running = False
        self._left_captured = False
        self._right_captured = False
        self._cancel_requested = False
        self._left_calculated = False
        self._right_calculated = False
        self.btn_show_plot.setDisabled(True)
        self.btn_show_regression.setDisabled(True)
        self.btn_feedforward_validation.setDisabled(True)
        self.btn_write_params.setDisabled(True)
        self.btn_write_params_left.setDisabled(True)
        self.btn_write_params_right.setDisabled(True)
        # 控制可见性：仅当脚本以 --dev 参数运行时才显示该按钮
        self._show_dex_features = "--dev" in sys.argv
        if not getattr(self, "_show_dex_features", False):
            self.btn_calc_params.setDisabled(True)
            self.btn_calc_params_left.setDisabled(True)
            self.btn_calc_params_right.setDisabled(True)
        else:
            self.btn_show_plot.setDisabled(False)

        self._update_data_dir_label()
        self._did_initial_ref_plot = False
        self.identification_combo.currentIndexChanged.connect(self.on_identification_options_changed)
        self.filter_torque_checkbox.toggled.connect(self.on_identification_options_changed)
        self.friction_model_combo.currentIndexChanged.connect(self.on_friction_model_changed)
        self.stribeck_alpha_spin.valueChanged.connect(self.on_identification_options_changed)
        for key, spin in self.friction_velocity_spins.items():
            spin.valueChanged.connect(self.on_identification_options_changed)
        self.af_alpha_spin.valueChanged.connect(self.on_identification_options_changed)
        self.af_beta_spin.valueChanged.connect(self.on_identification_options_changed)
        self.af_alpha_spin.setDisabled(True)
        self.af_beta_spin.setDisabled(True)

    def log_msg(self, msg):
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        formatted = f"[{ts}] {msg}"
        self.log_signal.emit(formatted)
        print(formatted)

    @QtCore.pyqtSlot(str)
    def _append_log(self, msg):
        self.log.append(msg)

    def update_params_from_ui(self):
        # 从下拉框读取轨迹名与臂侧（显示名 -> 内部值映射）
        traj_text = self.traj_combo.currentText().strip()
        traj_map = {"轨迹1": "traj1", "轨迹2": "traj2"}
        self.dyn.traj = traj_map.get(traj_text, "traj1")
        self.dyn.is_left = self.arm_combo.currentText() == "左臂"
        side = "left" if self.dyn.is_left else "right"
        self.dyn.traj_path = os.path.join(self.dyn.traj_dir, side, f"{self.dyn.traj}.dat")
        for calculator in (self.dyn.calculator_left, self.dyn.calculator_right):
            options = dict(calculator.friction_options)
            options["friction_model"] = self.friction_model_combo.currentData()
            options["stribeck_alpha"] = self.stribeck_alpha_spin.value()
            options.update({key: spin.value() for key, spin in self.friction_velocity_spins.items()})
            calculator.set_friction_options(options)
            calculator.set_identification_options(
                self.identification_combo.currentData(),
                self.af_alpha_spin.value(), self.af_beta_spin.value(),
                filter_torque=self.filter_torque_checkbox.isChecked(),
            )

    def _load_friction_velocity_controls(self, options):
        model = options["friction_model"]
        for key, spin in self.friction_velocity_spins.items():
            value = options[key]
            was_blocked = spin.blockSignals(True)
            try:
                spin.setRange(min(spin.minimum(), value), max(spin.maximum(), value))
                spin.setValue(value)
            finally:
                spin.blockSignals(was_blocked)
            spin.setEnabled(model != FRICTION_MODEL_CLASSIC if key == "friction_velocity_scale"
                            else model == FRICTION_MODEL_STRIBECK)

    def on_friction_model_changed(self, *_args):
        options = dict(self.dyn.calculator_left.friction_options)
        options["friction_model"] = self.friction_model_combo.currentData()
        self._load_friction_velocity_controls(options)
        self.on_identification_options_changed()

    def on_identification_options_changed(self, *_args):
        self.update_params_from_ui()
        self._left_calculated = False
        self._right_calculated = False
        self._params_calculated = False
        self.enable_buttons()
        torque_description = "力矩滤波（与 q 相同）" if self.filter_torque_checkbox.isChecked() else "原始力矩"
        self.log_msg(f"辨识设置已更改: {self.identification_combo.currentText()} / {self.friction_model_combo.currentText()} / {torque_description}，请重新计算参数。")

    def disable_buttons(self, disabled=True):
        # 除停止按钮外均禁用/启用；部分按钮受状态标志控制
        disabled = disabled or self._calculation_running
        try:
            self.btn_run_sim.setDisabled(disabled)
            self.btn_run_real.setDisabled(disabled)
            self.btn_run_real_left.setDisabled(disabled)
            self.btn_run_real_right.setDisabled(disabled)
            # 采集/计算过程中不允许切换数据源
            self.btn_select_data_dir.setDisabled(disabled)
            for button in self.recording_file_buttons:
                button.setDisabled(disabled)
            self.identification_combo.setDisabled(disabled)
            self.filter_torque_checkbox.setDisabled(disabled)
            self.friction_model_combo.setDisabled(disabled)
            self.stribeck_alpha_spin.setDisabled(
                disabled or self.friction_model_combo.currentData() != FRICTION_MODEL_STRIBECK
            )
            for key, spin in self.friction_velocity_spins.items():
                model = self.friction_model_combo.currentData()
                enabled = model != FRICTION_MODEL_CLASSIC if key == "friction_velocity_scale" else model == FRICTION_MODEL_STRIBECK
                spin.setDisabled(disabled or not enabled)
            af_enabled = self.identification_combo.currentData() == IDENTIFICATION_AF_ARF
            self.af_alpha_spin.setDisabled(disabled or not af_enabled)
            self.af_beta_spin.setDisabled(disabled or not af_enabled)
            # 可视化始终可用
            self.btn_open_meshcat.setDisabled(False)
            # 待采集参考轨迹：仅随操作禁用，不受采集/计算状态限制
            self.btn_show_reference.setDisabled(disabled)
            # 仿真倍速选择在运行/计算时也禁用
            self.sim_speed_combo.setDisabled(disabled)
            self.real_speed_combo.setDisabled(disabled)
            self.real_arm_speed_combo.setDisabled(disabled)
            # 根据采集完成状态决定是否可用
            self.btn_show_plot.setDisabled(
                True if disabled else not (self._left_captured or self._right_captured)
            )
            if getattr(self, "_show_dex_features", False):
                self.btn_calc_params.setDisabled(disabled)
                self.btn_calc_params_left.setDisabled(disabled)
                self.btn_calc_params_right.setDisabled(disabled)
            else:
                self.btn_calc_params.setDisabled(
                    True if disabled else not (self._left_captured and self._right_captured)
                )
                self.btn_calc_params_left.setDisabled(
                    True if disabled else not self._left_captured
                )
                self.btn_calc_params_right.setDisabled(
                    True if disabled else not self._right_captured
                )
            # 根据计算完成状态决定是否可用
            self.btn_show_regression.setDisabled(
                True if disabled else (not self._params_calculated)
            )
            self.btn_feedforward_validation.setDisabled(
                disabled or not (self._left_calculated or self._right_calculated)
            )
            self.use_source_friction.setDisabled(disabled)
            for spin in self.friction_scale_spins:
                spin.setDisabled(disabled or self.use_source_friction.isChecked())
            self.btn_write_params.setDisabled(
                True if disabled else (not self._params_calculated)
            )
            self.btn_write_params_left.setDisabled(
                True if disabled else (not self._left_calculated)
            )
            self.btn_write_params_right.setDisabled(
                True if disabled else (not self._right_calculated)
            )
            self.traj_combo.setDisabled(disabled)
            self.arm_combo.setDisabled(disabled)
            # 停止按钮与清错按钮始终可用
            self.btn_stop.setDisabled(False)
            self.btn_clear.setDisabled(False)
        except Exception:
            pass

    @QtCore.pyqtSlot()
    def enable_buttons(self):
        self.disable_buttons(False)

    def on_run_sim(self):
        self.update_params_from_ui()
        self.log_msg("开始运行仿真...")
        # 根据倍速系数调整仿真步长（仅影响调度，不改数据本身）
        try:
            self.dyn._sim_dt_override = float(self.dyn.dt) / max(
                1.0, float(getattr(self, "_sim_speed_mult", 1.0))
            )
        except Exception:
            self.dyn._sim_dt_override = self.dyn.dt
        self.disable_buttons(True)
        t = threading.Thread(target=self._thread_run_sim, daemon=True)
        t.start()

    def _thread_run_sim(self):
        try:
            if not self.dyn.run_traj_sim(self.dyn.traj_path):
                return
            self.log_msg("仿真运行结束。")
        except Exception as e:
            self.log_msg("仿真出错: " + str(e))
        finally:
            try:
                QtCore.QMetaObject.invokeMethod(
                    self, "enable_buttons", QtCore.Qt.QueuedConnection
                )
            except Exception:
                pass

    def _on_sim_speed_changed(self, text: str):
        try:
            mapping = {"x1倍速": 1.0, "x2倍速": 2.0}
            self._sim_speed_mult = mapping.get(text, 1.0)
            self.log_msg(f"仿真速度系数设置为 {self._sim_speed_mult}x")
        except Exception:
            self._sim_speed_mult = 1.0

    def _on_real_speed_changed(self, text: str):
        try:
            mapping = {"x1倍速": 1.0, "x0.5倍速": 0.5}
            self._real_speed_mult = mapping.get(text, 1.0)
            self.log_msg(f"真机采集速度系数设置为 {self._real_speed_mult}x")
        except Exception:
            self._real_speed_mult = 1.0

    def _on_real_arm_speed_changed(self, text: str):
        try:
            mapping = {"x1倍速": 1.0, "x0.5倍速": 0.5}
            self._real_arm_speed_mult = mapping.get(text, 1.0)
            self.log_msg(f"单臂真机采集速度系数设置为 {self._real_arm_speed_mult}x")
        except Exception:
            self._real_arm_speed_mult = 1.0

    def _update_data_dir_label(self):
        """刷新左侧面板显示的采集数据目录。"""
        try:
            path = str(getattr(self.dyn, "rec_dir", ""))
            self.data_dir_label.setText(f"当前数据目录: {path}")
            self.data_dir_label.setToolTip(path)
            for side, title in (("left", "左臂"), ("right", "右臂")):
                calculator = self.dyn.calculator_left if side == "left" else self.dyn.calculator_right
                for traj, role in (("traj1", "辨识"), ("traj2", "验证")):
                    filename = calculator.recording_path(traj)
                    label = self.recording_file_labels[side, traj]
                    label.setText(f"{title}{role}: {filename.name}")
                    label.setToolTip(str(filename))
        except Exception:
            pass

    def on_select_recording_file(self, side, traj):
        calculator = self.dyn.calculator_left if side == "left" else self.dyn.calculator_right
        title = ("左臂" if side == "left" else "右臂") + ("辨识" if traj == "traj1" else "验证")
        filename, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, f"选择{title}数据", str(calculator.recording_path(traj).parent),
            "数据文件 (*.dat *.txt);;所有文件 (*)",
        )
        if not filename:
            return
        try:
            data = np.loadtxt(filename)
            if data.ndim != 2 or data.shape[1] not in (15, 22) or not np.all(np.isfinite(data)):
                raise ValueError("需要 15 或 22 列数据：时间、7 关节位置、7 电流力矩、可选的 7 传感器力矩。")
            if len(data) <= int(4 / calculator.dt):
                raise ValueError("数据不足，裁剪前 3 秒和后 1 秒后无可用样本。")
            calculator.set_recording_file(traj, filename)
            if traj == "traj1":
                if side == "left":
                    self._left_calculated, self._left_captured = False, True
                else:
                    self._right_calculated, self._right_captured = False, True
                self._params_calculated = False
            self._update_data_dir_label()
            self.enable_buttons()
            self.log_msg(f"{title}文件: {filename}" +
                         ("；请重新辨识该臂。" if traj == "traj1" else "；保留当前辨识参数。"))
        except Exception as exc:
            self.log_msg(f"选择{title}数据失败: {exc}")

    @staticmethod
    def _probe_rec_file(path):
        """快速探测记录文件: 返回 (行数, 列数)，不存在或不可读返回 None。"""
        try:
            rows = 0
            cols = 0
            with open(path, "r") as f:
                for line in f:
                    if line.strip():
                        if rows == 0:
                            cols = len(line.split())
                        rows += 1
            return (rows, cols) if rows > 0 else None
        except Exception:
            return None

    @staticmethod
    def _format_probe(path, item):
        """把探测结果格式化成一行日志。"""
        if item is None:
            return f"{path} 缺失"
        rows, cols = item
        warn = "" if cols == 22 else f", 列数 {cols} (应为 22, 可能不是本工具采集的数据)"
        return f"{path} {rows} 行{warn}"

    def on_select_data_dir(self):
        """选择本地采集数据目录，用于图示已采集轨迹与离线辨识计算。"""
        try:
            base = str(getattr(self.dyn, "rec_dir", ""))
            if not os.path.isdir(base):
                base = str(Path(__file__).resolve().parent / "dyn_params_ident")
            selected = QtWidgets.QFileDialog.getExistingDirectory(
                self, "选择采集数据目录（含 left / right 子目录）", base
            )
            if not selected:
                return
            self._apply_data_dir(selected)
        except Exception as e:
            self.log_msg("选择数据目录出错: " + str(e))

    def _apply_data_dir(self, selected):
        """校验选中的目录并切换为当前数据源；若直接选中 left/right 子目录则自动上溯一级。"""
        base = Path(selected)
        if not ((base / "left").is_dir() or (base / "right").is_dir()):
            if base.name in ("left", "right"):
                base = base.parent
        # 逐臂探测: traj1 辨识必需, traj2 验证用
        probes = {}
        for side in ("left", "right"):
            probes[side] = {
                "traj1": self._probe_rec_file(base / side / "rbtlog_traj1.dat"),
                "traj2": self._probe_rec_file(base / side / "rbtlog_traj2.dat"),
            }
        if not any(probes[s]["traj1"] for s in probes):
            self.log_msg(f"目录 {base} 下未找到 left/right/rbtlog_traj1.dat, 未切换数据源。")
            QtWidgets.QMessageBox.warning(
                self,
                "数据目录无效",
                "未找到 rbtlog_traj1.dat。\n\n"
                "请选择包含 left/ 与 right/ 子目录的采集数据目录"
                "（子目录内应有 rbtlog_traj1.dat / rbtlog_traj2.dat）。",
            )
            return
        # 切换数据源（Node 用于绘图, 两个 calculator 用于辨识）
        self.dyn.rec_dir = base
        self.dyn.calculator_left.rec_dir = base
        self.dyn.calculator_right.rec_dir = base
        self.dyn.calculator_left.data_files.clear()
        self.dyn.calculator_right.data_files.clear()
        self._left_captured = probes["left"]["traj1"] is not None
        self._right_captured = probes["right"]["traj1"] is not None
        # 数据源已更换, 之前算出的参数不再对应当前数据
        if self._left_calculated or self._right_calculated:
            self.log_msg("数据源已更换, 之前计算的辨识参数已失效, 请重新计算。")
        self._left_calculated = False
        self._right_calculated = False
        self._params_calculated = False
        self.dyn.calculator_left.reset_identification()
        self.dyn.calculator_right.reset_identification()
        self._update_data_dir_label()
        self.enable_buttons()
        self.log_msg(f"数据源已切换为: {base}")
        for side, label in (("left", "左臂"), ("right", "右臂")):
            for traj in ("traj1", "traj2"):
                self.log_msg(
                    f"  {label} {self._format_probe(f'{side}/rbtlog_{traj}.dat', probes[side][traj])}"
                )
        if not (probes["left"]["traj2"] and probes["right"]["traj2"]):
            self.log_msg("提示: traj2 用于参数验证, 缺失时仍可辨识但无法看图示参数验证结果。")

    def _open_meshcat_url(self):
        try:
            url = (self.meshcat_url or "").strip()
            if not url:
                return
            # 仅使用外部浏览器打开
            self._open_external_browser()
        except Exception as e:
            self.log_msg("打开Meshcat地址失败: " + str(e))

    def _start_visual_node(self):
        try:
            # 每次点击按钮都允许重新在外部浏览器打开
            self._external_opened = False
            # 若已在运行则不重复启动
            if self._visual_proc and (self._visual_proc.poll() is None):
                self.log_msg("可视化节点已在运行。")
                # 节点已运行时也尝试打开可视化视图
                self._open_meshcat_url()
                return
            import subprocess, sys

            script_path = os.path.join(os.path.dirname(__file__), "visual_ros2_node.py")
            self._visual_proc = subprocess.Popen(
                [sys.executable, "-u", script_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            self.log_msg("启动可视化节点成功, 正在打开可视化显示...")
            # 异步读取子进程输出，捕捉Meshcat地址
            t = threading.Thread(target=self._read_visual_stdout, daemon=True)
            t.start()
        except Exception as e:
            self.log_msg("启动可视化节点失败: " + str(e))

    def _read_visual_stdout(self):
        # 读取可视化子进程的标准输出，捕捉Meshcat地址
        try:
            import re

            if not self._visual_proc or not self._visual_proc.stdout:
                return
            while True:
                line = self._visual_proc.stdout.readline()
                self.log_msg(line.strip())
                if not line:
                    break
                l = line.strip()
                # 通用meshcat提示行
                if ("http://" in l or "https://" in l) and "/static/" in l:
                    m = re.search(r"(https?://[^\s]+/static/)", l)
                    if m:
                        url = m.group(1)
                        if url != self._last_detected_url:
                            self.meshcat_url = url
                            self._last_detected_url = url
                        try:
                            QtCore.QTimer.singleShot(0, self._open_meshcat_url)
                        except Exception:
                            pass
        except Exception as e:
            self.log_msg("读取可视化节点输出失败: " + str(e))

    def _open_external_browser(self):
        # 使用系统默认浏览器打开Meshcat地址
        try:
            # 防抖：若已打开过则不再重复打开
            if getattr(self, "_external_opened", False):
                # 避免重复日志刷屏
                return
            import webbrowser

            url = (self.meshcat_url or "").strip()
            if not url:
                return
            webbrowser.open(url)
            self._external_opened = True
            self.log_msg(f"已在外部浏览器打开: {url}")
        except Exception as e:
            self.log_msg("外部浏览器打开失败: " + str(e))

    def on_run_real(self):
        self.update_params_from_ui()
        # 根据倍速系数调整真机采集步长
        try:
            self.dyn._real_dt_override = float(self.dyn.dt) / max(
                0.1, float(getattr(self, "_real_speed_mult", 1.0))
            )
        except Exception:
            self.dyn._real_dt_override = self.dyn.dt
        # 弹窗确认
        msg = (
            "将执行真机采集：\n\n"
            "- 左右臂各采集2段轨迹（辨识+验证）\n"
            "- 轨迹范围较大，请注意安全并离开机器人手臂运动范围!\n"
            "- 预计用时约5分钟\n\n"
            "是否确认开始？"
        )
        # 使用自定义中文文字按钮，纯文字无图标
        mbox = QtWidgets.QMessageBox(self)
        # 纯文字按钮：不显示图标
        mbox.setIcon(QtWidgets.QMessageBox.NoIcon)
        mbox.setWindowTitle("确认开始真机采集")
        mbox.setText(msg)
        # 置顶并应用程序级模态，避免被遮挡并阻塞交互
        mbox.setWindowModality(QtCore.Qt.ApplicationModal)
        mbox.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint, True)
        btn_confirm = mbox.addButton("确认", QtWidgets.QMessageBox.YesRole)
        btn_cancel = mbox.addButton("取消", QtWidgets.QMessageBox.NoRole)
        mbox.setDefaultButton(btn_cancel)
        mbox.exec_()
        if mbox.clickedButton() is not btn_confirm:
            self.log_msg("已取消真机采集。")
            return
        self.log_msg("开始进行真机采集, 请注意安全!")
        self._left_captured = False
        self._right_captured = False
        self.disable_buttons(True)
        t = threading.Thread(target=self._thread_run_real, daemon=True)
        t.start()

    def _thread_run_real(self):
        try:
            if self.dyn.run_real_capture():
                self.log_msg(f"真机采集结束")
                self._left_captured = True
                self._right_captured = True
                self._plot_records_on_canvas()
        except Exception as e:
            self.log_msg("真机采集出错: " + str(e))
        finally:
            try:
                QtCore.QMetaObject.invokeMethod(
                    self, "enable_buttons", QtCore.Qt.QueuedConnection
                )
            except Exception:
                pass

    def on_run_real_left(self):
        self.update_params_from_ui()
        try:
            self.dyn._real_dt_override = float(self.dyn.dt) / max(
                0.1, float(getattr(self, "_real_arm_speed_mult", 1.0))
            )
        except Exception:
            self.dyn._real_dt_override = self.dyn.dt
        msg = (
            "将执行左臂真机采集：\n\n"
            "- 左臂采集2段轨迹（辨识+验证）\n"
            "- 轨迹范围较大，请注意安全并离开机器人手臂运动范围!\n\n"
            "是否确认开始？"
        )
        mbox = QtWidgets.QMessageBox(self)
        mbox.setIcon(QtWidgets.QMessageBox.NoIcon)
        mbox.setWindowTitle("确认开始左臂真机采集")
        mbox.setText(msg)
        mbox.setWindowModality(QtCore.Qt.ApplicationModal)
        mbox.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint, True)
        btn_confirm = mbox.addButton("确认", QtWidgets.QMessageBox.YesRole)
        btn_cancel = mbox.addButton("取消", QtWidgets.QMessageBox.NoRole)
        mbox.setDefaultButton(btn_cancel)
        mbox.exec_()
        if mbox.clickedButton() is not btn_confirm:
            self.log_msg("已取消左臂真机采集。")
            return
        self.log_msg("开始进行左臂真机采集, 请注意安全!")
        self._left_captured = False
        self.disable_buttons(True)
        t = threading.Thread(target=self._thread_run_real_left, daemon=True)
        t.start()

    def _thread_run_real_left(self):
        try:
            if self.dyn.run_real_capture_arm(is_left=True):
                self.log_msg("左臂真机采集结束")
                self._left_captured = True
                self._plot_records_on_canvas()
        except Exception as e:
            self.log_msg("左臂真机采集出错: " + str(e))
        finally:
            try:
                QtCore.QMetaObject.invokeMethod(
                    self, "enable_buttons", QtCore.Qt.QueuedConnection
                )
            except Exception:
                pass

    def on_run_real_right(self):
        self.update_params_from_ui()
        try:
            self.dyn._real_dt_override = float(self.dyn.dt) / max(
                0.1, float(getattr(self, "_real_arm_speed_mult", 1.0))
            )
        except Exception:
            self.dyn._real_dt_override = self.dyn.dt
        msg = (
            "将执行右臂真机采集：\n\n"
            "- 右臂采集2段轨迹（辨识+验证）\n"
            "- 轨迹范围较大，请注意安全并离开机器人手臂运动范围!\n\n"
            "是否确认开始？"
        )
        mbox = QtWidgets.QMessageBox(self)
        mbox.setIcon(QtWidgets.QMessageBox.NoIcon)
        mbox.setWindowTitle("确认开始右臂真机采集")
        mbox.setText(msg)
        mbox.setWindowModality(QtCore.Qt.ApplicationModal)
        mbox.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint, True)
        btn_confirm = mbox.addButton("确认", QtWidgets.QMessageBox.YesRole)
        btn_cancel = mbox.addButton("取消", QtWidgets.QMessageBox.NoRole)
        mbox.setDefaultButton(btn_cancel)
        mbox.exec_()
        if mbox.clickedButton() is not btn_confirm:
            self.log_msg("已取消右臂真机采集。")
            return
        self.log_msg("开始进行右臂真机采集, 请注意安全!")
        self._right_captured = False
        self.disable_buttons(True)
        t = threading.Thread(target=self._thread_run_real_right, daemon=True)
        t.start()

    def _thread_run_real_right(self):
        try:
            if self.dyn.run_real_capture_arm(is_left=False):
                self.log_msg("右臂真机采集结束")
                self._right_captured = True
                self._plot_records_on_canvas()
        except Exception as e:
            self.log_msg("右臂真机采集出错: " + str(e))
        finally:
            try:
                QtCore.QMetaObject.invokeMethod(
                    self, "enable_buttons", QtCore.Qt.QueuedConnection
                )
            except Exception:
                pass

    def on_show_plot(self):
        self.update_params_from_ui()
        self.log_msg("准备图示已采集轨迹...")
        self.disable_buttons(True)
        t = threading.Thread(target=self._thread_show_plot, daemon=True)
        t.start()

    def _thread_show_plot(self):
        try:
            QtCore.QMetaObject.invokeMethod(
                self, "_plot_records_on_canvas", QtCore.Qt.QueuedConnection
            )
        except Exception as e:
            self.log_msg("绘图出错: " + str(e))

    def on_show_regression(self):
        self.update_params_from_ui()
        self.log_msg("准备图示辨识结果")
        self.disable_buttons(True)
        t = threading.Thread(target=self._thread_show_regression, daemon=True)
        t.start()

    def on_feedforward_validation(self):
        friction_scales = (None if self.use_source_friction.isChecked() else
                           tuple(spin.value() for spin in self.friction_scale_spins))
        self.disable_buttons(True)
        try:
            for side, calculated, calculator, figure, canvas in (
                ("左臂", self._left_calculated, self.dyn.calculator_left, self.left_fig, self.left_canvas),
                ("右臂", self._right_calculated, self.dyn.calculator_right, self.right_fig, self.right_canvas),
            ):
                figure.clear()
                if not calculated:
                    canvas.draw()
                    continue
                try:
                    predictions = calculator.get_all_validation_predictions(friction_scales)
                    t_full = predictions["t_full"]
                    t_zero = predictions["t_zero"]
                    measured_full = predictions["measured_full"]
                    measured_zero = predictions["measured_zero"]
                    full_unscaled = predictions["full_unscaled"]
                    full_scaled = predictions["full_scaled"]
                    zero_unscaled = predictions["zero_unscaled"]
                    zero_scaled = predictions["zero_scaled"]
                    rules = predictions["rules"]
                    self.log_msg(f"[{side}] 比例来源: " +
                                 ("robot_config.yaml" if friction_scales is None else "Qt 自定义（离线）"))
                    self.log_msg(f"[{side}] 摩擦映射及速度模型来源: " + "; ".join(map(str, rules.sources)))
                    self.log_msg(f"[{side}] fv 下标/比例: {rules.viscous}; fc 下标: {rules.coulomb_indices}; "
                                 f"fc 比例={np.asarray(rules.coulomb_scale).tolist()}, "
                                 f"模型={rules.model}, tanh 平滑速度={rules.velocity_scale} rad/s, "
                                 f"Stribeck 特征速度={rules.stribeck_velocity:g} rad/s, "
                                 f"alpha={rules.stribeck_alpha:g}")
                    self.log_msg(f"[{side}] 辨识算法：{calculator.identification_algorithm}；"
                                 "验证口径：滤波 q/dq/ddq，原始电流力矩，预测力矩不额外滤波。")
                    figure.suptitle(f"{'Left' if calculator.is_left else 'Right'} validation - four conditions")
                    for joint in range(calculator.ndof):
                        ax = figure.add_subplot(3, 3, joint + 1)
                        rmse_full = np.sqrt(np.mean((full_unscaled[:, joint] - measured_full[:, joint]) ** 2))
                        rmse_zero = np.sqrt(np.mean((zero_unscaled[:, joint] - measured_zero[:, joint]) ** 2))
                        rmse_zero_scaled = np.sqrt(np.mean((zero_scaled[:, joint] - measured_zero[:, joint]) ** 2))
                        rmse_full_scaled = np.sqrt(np.mean((full_scaled[:, joint] - measured_full[:, joint]) ** 2))
                        ax.plot(t_full, measured_full[:, joint], label="Raw current / target", color="tab:red", linewidth=1)
                        ax.plot(t_full, full_unscaled[:, joint], label="Full no scale", color="tab:blue", linewidth=1)
                        ax.plot(t_zero, zero_unscaled[:, joint], label="ddq=0", color="tab:orange", linewidth=1)
                        ax.plot(t_zero, zero_scaled[:, joint], label="ddq=0 + scale", color="tab:green", linewidth=1)
                        ax.plot(t_full, full_scaled[:, joint], label="Full + scale", color="tab:purple", linewidth=1)
                        ax.set_title(
                            f"J{joint + 1} RMSE full={rmse_full:.3f}, ddq0={rmse_zero:.3f}, "
                            f"ddq0+={rmse_zero_scaled:.3f}, full+={rmse_full_scaled:.3f}",
                            fontsize=8,
                        )
                        ax.set_xlabel("Time (s)")
                        ax.set_ylabel("Torque (Nm)")
                        ax.grid(True, linestyle="--", alpha=0.4)
                        ax.legend(fontsize="xx-small")
                        self.log_msg(
                            f"[{side}] J{joint + 1} RMSE (Nm): "
                            f"完整未缩放={rmse_full:.6f}, ddq=0={rmse_zero:.6f}, "
                            f"ddq=0+摩擦缩放={rmse_zero_scaled:.6f}, "
                            f"完整+摩擦缩放={rmse_full_scaled:.6f}"
                        )
                    self.log_msg(
                        f"[{side}] 总体 RMSE (Nm): "
                        f"完整未缩放={np.sqrt(np.mean((full_unscaled - measured_full) ** 2)):.6f}, "
                        f"ddq=0={np.sqrt(np.mean((zero_unscaled - measured_zero) ** 2)):.6f}, "
                        f"ddq=0+摩擦缩放={np.sqrt(np.mean((zero_scaled - measured_zero) ** 2)):.6f}, "
                        f"完整+摩擦缩放={np.sqrt(np.mean((full_scaled - measured_full) ** 2)):.6f}"
                    )
                    canvas.draw()
                except Exception as exc:
                    self.log_msg(f"[{side}] 四种工况对比失败: {exc}")
                    canvas.draw()
            self.log_msg(
                "四种工况已输出：完整动力学未缩放、ddq=0 未缩放、ddq=0+摩擦缩放、"
                "完整动力学+摩擦缩放。结果未写入真机。"
            )
        finally:
            self.enable_buttons()

    def _thread_show_regression(self):
        try:
            QtCore.QMetaObject.invokeMethod(
                self, "_plot_regression_on_canvas", QtCore.Qt.QueuedConnection
            )
        except Exception as e:
            self.log_msg("辨识结果绘图出错: " + str(e))
            try:
                QtCore.QMetaObject.invokeMethod(
                    self, "enable_buttons", QtCore.Qt.QueuedConnection
                )
            except Exception:
                pass

    def on_calc_params(self):
        self.update_params_from_ui()
        self._calculation_running = True
        self.disable_buttons(True)
        self._cancel_requested = False
        self._left_calculated = False
        self._right_calculated = False
        self._params_calculated = False
        t = threading.Thread(target=self._thread_calc_params, daemon=True)
        t.start()

    def _thread_calc_params(self):
        try:
            self.log_msg("开始计算左臂动力学参数...")
            if not self.dyn.calculator_left.calc_dyn_params():
                self.log_msg("左臂动力学参数计算失败。")
                return
            self.log_msg("左臂动力学参数计算完成。")
            self._left_calculated = True
            try:
                self.btn_write_params_left.setDisabled(False)
            except Exception:
                pass
            if getattr(self, "_cancel_requested", False):
                self.log_msg("已停止后续计算。")
                self._plot_regression_on_canvas()
                return
            self.log_msg("开始计算右臂动力学参数...")
            if not self.dyn.calculator_right.calc_dyn_params():
                self.log_msg("右臂动力学参数计算失败。")
                self._plot_regression_on_canvas()
                return
            self.log_msg("右臂动力学参数计算完成。")
            self.log_msg("动力学参数计算全部完成。")
            # 计算完成，启用写入按钮
            self._right_calculated = True
            self._params_calculated = True
            try:
                self.btn_write_params.setDisabled(False)
                self.btn_write_params_right.setDisabled(False)
                self.btn_show_regression.setDisabled(False)
            except Exception:
                pass
            self._plot_regression_on_canvas()
        except Exception as e:
            self.log_msg("动力学参数计算出错: " + str(e))
        finally:
            self._calculation_running = False
            try:
                QtCore.QMetaObject.invokeMethod(
                    self, "enable_buttons", QtCore.Qt.QueuedConnection
                )
            except Exception:
                pass

    def on_calc_params_left(self):
        self.update_params_from_ui()
        self._calculation_running = True
        self.disable_buttons(True)
        self._cancel_requested = False
        self._left_calculated = False
        self._right_calculated = False
        self._params_calculated = False
        t = threading.Thread(target=self._thread_calc_params_left, daemon=True)
        t.start()

    def _thread_calc_params_left(self):
        try:
            self.log_msg("开始计算左臂动力学参数...")
            if not self.dyn.calculator_left.calc_dyn_params():
                self.log_msg("左臂动力学参数计算失败。")
                return
            self.log_msg("左臂动力学参数计算完成。")
            self._left_calculated = True
            try:
                self.btn_write_params_left.setDisabled(False)
                if self._left_calculated and self._right_calculated:
                    self._params_calculated = True
                    self.btn_write_params.setDisabled(False)
                self.btn_show_regression.setDisabled(False)
            except Exception:
                pass
            self._plot_regression_on_canvas()
        except Exception as e:
            self.log_msg("左臂动力学参数计算出错: " + str(e))
        finally:
            self._calculation_running = False
            try:
                QtCore.QMetaObject.invokeMethod(
                    self, "enable_buttons", QtCore.Qt.QueuedConnection
                )
            except Exception:
                pass

    def on_calc_params_right(self):
        self.update_params_from_ui()
        self._calculation_running = True
        self.disable_buttons(True)
        self._cancel_requested = False
        self._left_calculated = False
        self._right_calculated = False
        self._params_calculated = False
        t = threading.Thread(target=self._thread_calc_params_right, daemon=True)
        t.start()

    def _thread_calc_params_right(self):
        try:
            self.log_msg("开始计算右臂动力学参数...")
            if not self.dyn.calculator_right.calc_dyn_params():
                self.log_msg("右臂动力学参数计算失败。")
                return
            self.log_msg("右臂动力学参数计算完成。")
            self._right_calculated = True
            try:
                self.btn_write_params_right.setDisabled(False)
                if self._left_calculated and self._right_calculated:
                    self._params_calculated = True
                    self.btn_write_params.setDisabled(False)
                self.btn_show_regression.setDisabled(False)
            except Exception:
                pass
            self._plot_regression_on_canvas()
        except Exception as e:
            self.log_msg("右臂动力学参数计算出错: " + str(e))
        finally:
            self._calculation_running = False
            try:
                QtCore.QMetaObject.invokeMethod(
                    self, "enable_buttons", QtCore.Qt.QueuedConnection
                )
            except Exception:
                pass

    def _get_current_beta(self, is_left: bool):
        try:
            calc = self.dyn.calculator_left if is_left else self.dyn.calculator_right
            if calc.solutions_beta_starextra is None:
                raise RuntimeError("未找到辨识参数，请先计算动力学参数。")
            beta = np.matrix(calc.solutions_beta_starextra).copy()
            beta_list = np.asarray(beta).flatten().tolist()
            return beta_list
        except Exception as e:
            raise RuntimeError(f"未找到辨识参数，请先计算动力学参数。错误: {e}")

    def _updated_identification_config(self, content, version_key, parameters):
        options = self.dyn.calculator_left.friction_options
        current = friction_settings(yaml.safe_load(content).get(version_key, {}))
        if len(parameters) == 1 and current != options:
            raise ValueError("摩擦设置已改变，必须重新辨识并写入双臂，避免另一臂的旧参数与新模型不匹配。")
        return update_robot_config_section(
            content, version_key, {**options, **parameters}, remove_keys=("stribeck_smoothing_velocity",),
        )

    def on_write_params(self):
        try:
            if not (self._left_calculated and self._right_calculated):
                self.log_msg("请先完成双臂动力学参数计算后再写入。")
                return

            # 获取左右臂参数
            beta_left = self._get_current_beta(True)
            beta_right = self._get_current_beta(False)

            version_prefix = self.dyn.version[:3]
            robot_version_key = f"{self.dyn.robot_name}_{version_prefix}"

            # 构造确认弹窗（中文纯文字，无图标，置顶模态）
            msg = (
                "将写入左右臂辨识参数和摩擦模型到 robot_config.yaml：\n\n"
                f"- 摩擦模型：{self.friction_model_combo.currentText()}\n"
                f"- 版本段：{robot_version_key}\n"
                f"- 左臂字段：dyn_base_params_primary（{len(beta_left)}维）\n"
                f"- 右臂字段：dyn_base_params_secondary（{len(beta_right)}维）\n\n"
                "是否确认写入？"
            )
            mbox = QtWidgets.QMessageBox(self)
            mbox.setIcon(QtWidgets.QMessageBox.NoIcon)
            mbox.setWindowTitle("确认写入辨识参数")
            mbox.setText(msg)
            mbox.setWindowModality(QtCore.Qt.ApplicationModal)
            mbox.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint, True)
            btn_confirm = mbox.addButton("确认", QtWidgets.QMessageBox.YesRole)
            btn_cancel = mbox.addButton("取消", QtWidgets.QMessageBox.NoRole)
            mbox.setDefaultButton(btn_cancel)
            mbox.exec_()
            if mbox.clickedButton() is not btn_confirm:
                self.log_msg("已取消写入。")
                return

            # 读取配置并写入两臂
            cfg_path = str(self.dyn.robot_cfg_path)
            with open(cfg_path, "r", encoding="utf-8") as f:
                content = f.read()
            
            new_content = self._updated_identification_config(content, robot_version_key, {
                "dyn_base_params_primary": [float(value) for value in beta_left],
                "dyn_base_params_secondary": [float(value) for value in beta_right],
            })

            with open(cfg_path, "w", encoding="utf-8") as f:
                f.write(new_content)
            
            self.log_msg(
                f"已写入左右臂辨识参数到机器人配置文件{cfg_path}, 重启服务后配置生效."
            )
        except Exception as e:
            self.log_msg("写入辨识参数出错: " + str(e))

    def on_write_params_left(self):
        try:
            if not self._left_calculated:
                self.log_msg("请先完成左臂动力学参数计算后再写入。")
                return
            beta_left = self._get_current_beta(True)
            version_prefix = self.dyn.version[:3]
            robot_version_key = f"{self.dyn.robot_name}_{version_prefix}"
            msg = (
                "将写入左臂辨识参数和摩擦模型到 robot_config.yaml：\n\n"
                f"- 摩擦模型：{self.friction_model_combo.currentText()}\n"
                f"- 版本段：{robot_version_key}\n"
                f"- 左臂字段：dyn_base_params_primary（{len(beta_left)}维）\n\n"
                "是否确认写入？"
            )
            mbox = QtWidgets.QMessageBox(self)
            mbox.setIcon(QtWidgets.QMessageBox.NoIcon)
            mbox.setWindowTitle("确认写入左臂辨识参数")
            mbox.setText(msg)
            mbox.setWindowModality(QtCore.Qt.ApplicationModal)
            mbox.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint, True)
            btn_confirm = mbox.addButton("确认", QtWidgets.QMessageBox.YesRole)
            btn_cancel = mbox.addButton("取消", QtWidgets.QMessageBox.NoRole)
            mbox.setDefaultButton(btn_cancel)
            mbox.exec_()
            if mbox.clickedButton() is not btn_confirm:
                self.log_msg("已取消写入。")
                return
            cfg_path = str(self.dyn.robot_cfg_path)
            with open(cfg_path, "r", encoding="utf-8") as f:
                content = f.read()
            new_content = self._updated_identification_config(content, robot_version_key, {
                "dyn_base_params_primary": [float(value) for value in beta_left],
            })
            with open(cfg_path, "w", encoding="utf-8") as f:
                f.write(new_content)
            self.log_msg(
                f"已写入左臂辨识参数到机器人配置文件{cfg_path}, 重启服务后配置生效."
            )
        except Exception as e:
            self.log_msg("写入左臂辨识参数出错: " + str(e))

    def on_write_params_right(self):
        try:
            if not self._right_calculated:
                self.log_msg("请先完成右臂动力学参数计算后再写入。")
                return
            beta_right = self._get_current_beta(False)
            version_prefix = self.dyn.version[:3]
            robot_version_key = f"{self.dyn.robot_name}_{version_prefix}"
            msg = (
                "将写入右臂辨识参数和摩擦模型到 robot_config.yaml：\n\n"
                f"- 摩擦模型：{self.friction_model_combo.currentText()}\n"
                f"- 版本段：{robot_version_key}\n"
                f"- 右臂字段：dyn_base_params_secondary（{len(beta_right)}维）\n\n"
                "是否确认写入？"
            )
            mbox = QtWidgets.QMessageBox(self)
            mbox.setIcon(QtWidgets.QMessageBox.NoIcon)
            mbox.setWindowTitle("确认写入右臂辨识参数")
            mbox.setText(msg)
            mbox.setWindowModality(QtCore.Qt.ApplicationModal)
            mbox.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint, True)
            btn_confirm = mbox.addButton("确认", QtWidgets.QMessageBox.YesRole)
            btn_cancel = mbox.addButton("取消", QtWidgets.QMessageBox.NoRole)
            mbox.setDefaultButton(btn_cancel)
            mbox.exec_()
            if mbox.clickedButton() is not btn_confirm:
                self.log_msg("已取消写入。")
                return
            cfg_path = str(self.dyn.robot_cfg_path)
            with open(cfg_path, "r", encoding="utf-8") as f:
                content = f.read()
            new_content = self._updated_identification_config(content, robot_version_key, {
                "dyn_base_params_secondary": [float(value) for value in beta_right],
            })
            with open(cfg_path, "w", encoding="utf-8") as f:
                f.write(new_content)
            self.log_msg(
                f"已写入右臂辨识参数到机器人配置文件{cfg_path}, 重启服务后配置生效."
            )
        except Exception as e:
            self.log_msg("写入右臂辨识参数出错: " + str(e))

    @QtCore.pyqtSlot()
    def _plot_records_on_canvas(self):
        try:
            # 同时绘制左/右臂两张图，无需再次点击
            ndof = self.dyn.ndof
            cols = 3
            rows = math.ceil(ndof / cols)

            # 左臂
            try:
                self.dyn.is_left = True
                self.left_fig.clear()
                axes_left = [
                    self.left_fig.add_subplot(rows, cols, j + 1)
                    for j in range(rows * cols)
                ]
                self.dyn.plot_records_to_axes(ax_handles=axes_left)
                for k in range(len(axes_left)):
                    if k >= ndof:
                        axes_left[k].set_visible(False)
                try:
                    self.left_fig.tight_layout()
                except Exception:
                    pass
                self.left_canvas.draw()
            except Exception as e:
                self.log_msg("左臂绘图错误: " + str(e))

            # 右臂
            try:
                self.dyn.is_left = False
                self.right_fig.clear()
                axes_right = [
                    self.right_fig.add_subplot(rows, cols, j + 1)
                    for j in range(rows * cols)
                ]
                self.dyn.plot_records_to_axes(ax_handles=axes_right)
                for k in range(len(axes_right)):
                    if k >= ndof:
                        axes_right[k].set_visible(False)
                try:
                    self.right_fig.tight_layout()
                except Exception:
                    pass
                self.right_canvas.draw()
            except Exception as e:
                self.log_msg("右臂绘图错误: " + str(e))

            self.log_msg("轨迹图绘制完成.")
        finally:
            self.disable_buttons(False)

    @QtCore.pyqtSlot()
    def _plot_reference_on_canvas(self):
        try:
            ndof = self.dyn.ndof
            cols = 3
            rows = math.ceil(ndof / cols)

            # 加载参考轨迹
            traj_path = self.dyn.traj_path
            try:
                q_ref = np.loadtxt(traj_path)
            except Exception as e:
                self.log_msg("加载参考轨迹失败: " + str(e))
                return
            if q_ref.ndim == 1:
                q_ref = q_ref.reshape(-1, 1)
            t_ref = np.array([self.dyn.dt * i for i in range(q_ref.shape[0])])

            # 左臂参考
            try:
                self.left_fig.clear()
                axes_left = [
                    self.left_fig.add_subplot(rows, cols, j + 1)
                    for j in range(rows * cols)
                ]
                for i in range(min(ndof, q_ref.shape[1])):
                    ax = axes_left[i]
                    ax.clear()
                    series = q_ref[:, i]
                    ax.plot(
                        t_ref,
                        series,
                        label="Reference q",
                        color="tab:blue",
                        linewidth=1,
                    )
                    ax.set_title(f"Joint {i+1}")
                    ax.set_xlabel("Time (s)")
                    ax.set_ylabel("Angle (rad)")
                    ax.grid(True, linestyle="--", alpha=0.4)
                    ax.legend(loc="upper right", fontsize="small")
                for k in range(len(axes_left)):
                    if k >= ndof:
                        axes_left[k].set_visible(False)
                try:
                    self.left_fig.tight_layout()
                except Exception:
                    pass
                self.left_canvas.draw()
            except Exception:
                pass

            try:
                self.right_fig.clear()
                axes_right = [
                    self.right_fig.add_subplot(rows, cols, j + 1)
                    for j in range(rows * cols)
                ]
                for i in range(min(ndof, q_ref.shape[1])):
                    ax = axes_right[i]
                    ax.clear()
                    series = q_ref[:, i]
                    ax.plot(
                        t_ref,
                        series,
                        label="Reference q",
                        color="tab:blue",
                        linewidth=1,
                    )
                    ax.set_title(f"Joint {i+1}")
                    ax.set_xlabel("Time (s)")
                    ax.set_ylabel("Angle (rad)")
                    ax.grid(True, linestyle="--", alpha=0.4)
                    ax.legend(loc="upper right", fontsize="small")
                for k in range(len(axes_right)):
                    if k >= ndof:
                        axes_right[k].set_visible(False)
                try:
                    self.right_fig.tight_layout()
                except Exception:
                    pass
                self.right_canvas.draw()
            except Exception:
                pass

            self.log_msg("已绘制待采集参考轨迹。")
        except Exception as e:
            self.log_msg("待采集参考轨迹绘制出错: " + str(e))

    def on_show_reference(self):
        try:
            t = threading.Thread(target=self._thread_show_reference, daemon=True)
            t.start()
        except Exception as e:
            self.log_msg("参考轨迹绘图启动出错: " + str(e))

    def _thread_show_reference(self):
        self.log_msg("正在绘图待采集轨迹(Reference q)...")
        try:
            self.update_params_from_ui()
            self.disable_buttons(True)
            QtCore.QMetaObject.invokeMethod(
                self, "_plot_reference_on_canvas", QtCore.Qt.QueuedConnection
            )
        except Exception as e:
            self.log_msg("参考轨迹绘图出错: " + str(e))
        finally:
            try:
                QtCore.QMetaObject.invokeMethod(
                    self, "enable_buttons", QtCore.Qt.QueuedConnection
                )
            except Exception:
                pass

    @QtCore.pyqtSlot()
    def _plot_regression_on_canvas(self):
        try:
            # 同时绘制左右臂辨识结果，无需再次点击
            ndof = self.dyn.ndof
            cols = 3
            rows = math.ceil(ndof / cols)

            # 左臂辨识
            r2_list_L = []
            if self._left_calculated:
                try:
                    self.left_fig.clear()
                    axes_left = [
                        self.left_fig.add_subplot(rows, cols, j + 1)
                        for j in range(rows * cols)
                    ]
                    calc_left = self.dyn.calculator_left
                    tL, tau_current_L, tau_sensor_L, tau_pred_L, tau_used_L = calc_left.get_regr_validatation()
                    totalL = tau_used_L.shape[0]
                    for i in range(min(ndof, tau_used_L.shape[1])):
                        ax = axes_left[i]
                        ax.clear()
                        sensor_i = np.asarray(tau_sensor_L[:, i]).flatten()[:totalL]
                        estimated_i = np.asarray(tau_pred_L[:, i]).flatten()[:totalL]
                        used_i = np.asarray(tau_used_L[:, i]).flatten()[:totalL]
                        ax.plot(
                            tL[:totalL],
                            used_i,
                            label="Raw current torque / target",
                            color="tab:red",
                            linewidth=1,
                        )
                        if np.any(np.abs(sensor_i) > 1e-9):
                            ax.plot(
                                tL[:totalL],
                                sensor_i,
                                label="Sensor torque (display only)",
                                color="tab:orange",
                                linewidth=1,
                                linestyle="--",
                            )
                        ax.plot(
                            tL[:totalL],
                            estimated_i,
                            label="Full dynamics torque",
                            color="tab:blue",
                            linewidth=1,
                        )
                        ss_res = np.sum((used_i - estimated_i) ** 2)
                        ss_tot = np.sum((used_i - np.mean(used_i)) ** 2)
                        r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 0.0
                        r2_list_L.append(r2)
                        ax.set_title(f"Joint {i+1}  R²={r2*100:.1f}%")
                        ax.set_xlabel("Time (s)")
                        ax.set_ylabel("Torque (Nm)")
                        ax.grid(True, linestyle="--", alpha=0.4)
                        ax.legend(loc="upper right", fontsize="small")
                    for k in range(len(axes_left)):
                        if k >= ndof:
                            axes_left[k].set_visible(False)
                    try:
                        self.left_fig.tight_layout()
                    except Exception:
                        pass
                    self.left_canvas.draw()
                    r2_str_L = ", ".join([f"J{i+1}: {v*100:.1f}%" for i, v in enumerate(r2_list_L)])
                    self.log_msg(f"左臂力矩吻合度 R²: {r2_str_L}, 平均: {np.mean(r2_list_L)*100:.1f}%")
                except Exception as e:
                    self.log_msg(
                        "左臂辨识绘图错误: "
                        + str(e)
                        + "。请先计算动力学参数并确保数据存在。"
                    )

            # 右臂辨识
            r2_list_R = []
            if self._right_calculated:
                try:
                    self.right_fig.clear()
                    axes_right = [
                        self.right_fig.add_subplot(rows, cols, j + 1)
                        for j in range(rows * cols)
                    ]
                    calc_right = self.dyn.calculator_right
                    tR, tau_current_R, tau_sensor_R, tau_pred_R, tau_used_R = calc_right.get_regr_validatation()
                    totalR = tau_used_R.shape[0]
                    for i in range(min(ndof, tau_used_R.shape[1])):
                        ax = axes_right[i]
                        ax.clear()
                        sensor_i = np.asarray(tau_sensor_R[:, i]).flatten()[:totalR]
                        estimated_i = np.asarray(tau_pred_R[:, i]).flatten()[:totalR]
                        used_i = np.asarray(tau_used_R[:, i]).flatten()[:totalR]
                        ax.plot(
                            tR[:totalR],
                            used_i,
                            label="Raw current torque / target",
                            color="tab:red",
                            linewidth=1,
                        )
                        if np.any(np.abs(sensor_i) > 1e-9):
                            ax.plot(
                                tR[:totalR],
                                sensor_i,
                                label="Sensor torque (display only)",
                                color="tab:orange",
                                linewidth=1,
                                linestyle="--",
                            )
                        ax.plot(
                            tR[:totalR],
                            estimated_i,
                            label="Full dynamics torque",
                            color="tab:blue",
                            linewidth=1,
                        )
                        ss_res = np.sum((used_i - estimated_i) ** 2)
                        ss_tot = np.sum((used_i - np.mean(used_i)) ** 2)
                        r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 0.0
                        r2_list_R.append(r2)
                        ax.set_title(f"Joint {i+1}  R²={r2*100:.1f}%")
                        ax.set_xlabel("Time (s)")
                        ax.set_ylabel("Torque (Nm)")
                        ax.grid(True, linestyle="--", alpha=0.4)
                        ax.legend(loc="upper right", fontsize="small")
                    for k in range(len(axes_right)):
                        if k >= ndof:
                            axes_right[k].set_visible(False)
                    try:
                        self.right_fig.tight_layout()
                    except Exception:
                        pass
                    self.right_canvas.draw()
                    r2_str_R = ", ".join([f"J{i+1}: {v*100:.1f}%" for i, v in enumerate(r2_list_R)])
                    self.log_msg(f"右臂力矩吻合度 R²: {r2_str_R}, 平均: {np.mean(r2_list_R)*100:.1f}%")
                except Exception as e:
                    self.log_msg(
                        "右臂辨识绘图错误: "
                        + str(e)
                        + "。请先计算动力学参数并确保数据存在。"
                    )

            r2_threshold = 0.8
            weights = np.array([2, 2, 2, 2, 1, 1, 1])
            mean_r2_L = np.average(r2_list_L, weights=weights) if r2_list_L else None
            mean_r2_R = np.average(r2_list_R, weights=weights) if r2_list_R else None
            parts = []
            if mean_r2_L is not None:
                parts.append(f"左臂 {mean_r2_L*100:.1f}%")
            if mean_r2_R is not None:
                parts.append(f"右臂 {mean_r2_R*100:.1f}%")
            if not parts:
                pass
            else:
                r2_vals = [v for v in [mean_r2_L, mean_r2_R] if v is not None]
                all_pass = all(v >= r2_threshold for v in r2_vals)
                summary = ", ".join(parts)
                if all_pass:
                    self.log_msg(f"辨识验证通过，加权平均吻合度: {summary}, 大于 {r2_threshold*100}%")
                else:
                    self.log_msg(f"辨识验证未达标（阈值 {r2_threshold*100}%）: {summary}，仍可写入但建议检查数据。")

            self.log_msg("辨识结果图绘制完成。")
        finally:
            self.disable_buttons(False)

    @QtCore.pyqtSlot()
    def _on_resize(self, event):
        try:
            # 分别调整两个图的布局并请求重绘
            try:
                self.left_fig.tight_layout()
                self.left_canvas.draw_idle()
            except Exception:
                pass
            try:
                self.right_fig.tight_layout()
                self.right_canvas.draw_idle()
            except Exception:
                pass
        except Exception:
            pass

    def on_stop_clicked(self):
        try:
            self.log_msg("收到停止指令，正在请求停止所有运动与计算")
            self._cancel_requested = True
            self.dyn._stop_move()
        except Exception as e:
            self.log_msg("停止流程出错: " + str(e))
        finally:
            # 停止后允许再次操作
            try:
                QtCore.QMetaObject.invokeMethod(
                    self, "enable_buttons", QtCore.Qt.QueuedConnection
                )
            except Exception:
                pass

    def on_clear_clicked(self):
        try:
            self.log_msg("正在请求清除机器人错误...")
            self.dyn.clear_error()
        except Exception as e:
            self.log_msg("清除错误流程出错: " + str(e))

    def closeEvent(self, event):
        try:
            try:
                self.executor.remove_node(self.dyn)
            except Exception:
                pass
            try:
                rclpy.shutdown()
            except Exception:
                pass
            try:
                if self.ros_thread and self.ros_thread.is_alive():
                    self.ros_thread.join(timeout=0.5)
            except Exception:
                pass
            # 结束可视化子进程
            try:
                if hasattr(self, "_visual_proc") and self._visual_proc:
                    if self._visual_proc.poll() is None:
                        self.log_msg("正在结束可视化节点...")
                        # 尝试发送终止信号
                        try:
                            self._visual_proc.terminate()
                        except Exception:
                            pass
                        # 等待一段时间，若仍未结束则强制杀掉
                        try:
                            self._visual_proc.wait(timeout=2.0)
                        except Exception:
                            try:
                                self._visual_proc.kill()
                            except Exception:
                                pass
                        self.log_msg("可视化节点已结束。")
            except Exception:
                pass
        finally:
            super().closeEvent(event)


def main():
    rclpy.init()
    app = QtWidgets.QApplication(sys.argv)
    win = DynParamGUI()
    win.show()
    win.activateWindow()
    win.raise_()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
