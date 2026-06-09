import os
import sys
import math
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
from sensor_msgs.msg import JointState
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


def read_data(dof, h, rbtlogfile, trajreffile):
    rbtlog = np.loadtxt(rbtlogfile)
    t = np.array(rbtlog[:, 0])
    q = np.array(rbtlog[:, 1 : dof + 1])
    tau = np.array(rbtlog[:, dof + 1 : dof * 2 + 1])
    trajref = np.loadtxt(trajreffile)
    reft = np.array([h * i for i in range(trajref.shape[0])])
    return t, q, tau, reft, trajref


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
        self.status_sim = None
        self.joint_position_sim = None
        self.joint_torque_sim = None
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

    def _run_real_capture_once(self, traj_path, rec_path):
        with open(traj_path, "r") as file:
            q_ref = np.loadtxt(file)
        # 使用GUI设置的真机步长覆盖
        local_dt = getattr(self, "_real_dt_override", self.dt)
        rec = np.zeros((q_ref.shape[0], 1 + 7 + 7))
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
                rec[i, :] = np.concatenate(
                    [
                        np.array([float(self.timestamp - time0) * 1e-9]),
                        np.array(self.joint_position[6:13]),
                        np.array(self.joint_torque[6:13]),
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
                rec[i, :] = np.concatenate(
                    [
                        np.array([float(self.timestamp - time0) * 1e-9]),
                        np.array(self.joint_position[13:20]),
                        np.array(self.joint_torque[13:20]),
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
        rec_path = os.path.join(self.rec_dir, side_dir, rec_filename)
        traj_path = os.path.join(self.traj_dir, side_dir, f"{self.traj}.dat")

        t_raw, q_raw, tau_raw, t_ref, q_ref = read_data(
            self.ndof, self.dt, rec_path, traj_path
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

        for j in range(min(len(ax_handles), n_joints)):
            ax1 = ax_handles[j]
            ax1.clear()
            ref_series = q_ref_cut[:, j]
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
                label="Measured τ (Nm)",
                color="tab:red",
                linewidth=1,
            )
            ax2.set_ylabel("Torque (Nm)", color="tab:red")
            ax1.tick_params(axis="y", labelcolor="tab:blue")
            ax2.tick_params(axis="y", labelcolor="tab:red")
            ax1.legend(loc="upper right", fontsize="small")

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
            
            full_pos = []
            full_tau = []
            for name in self.joint_full_names:
                if name in msg_joint_names:
                    idx = msg_joint_names.index(name)
                    full_pos.append(msg_joint_positions[idx])
                    if idx < len(msg_joint_torques):
                        full_tau.append(msg_joint_torques[idx])
                    else:
                        full_tau.append(0.0)
                else:
                    full_pos.append(0.0)
                    full_tau.append(0.0)
            
            self.joint_name = self.joint_full_names
            self.joint_position = full_pos
            self.joint_torque = full_tau

    def _robot_state_callback_sim(self, msg):
        self._last_msg_time_sim = self.get_clock().now().nanoseconds / 1e9
        state = json.loads(msg.data)
        if "status" in state:
            self.status_sim = state["status"]
        
        if "joint_name" in state and "joint_position" in state:
            msg_joint_names = state["joint_name"]
            msg_joint_positions = [float(x) for x in state["joint_position"]]
            msg_joint_torques = [float(x) for x in state.get("joint_torque", [])]
            
            full_pos = []
            full_tau = []
            for name in self.joint_full_names:
                if name in msg_joint_names:
                    idx = msg_joint_names.index(name)
                    full_pos.append(msg_joint_positions[idx])
                    if idx < len(msg_joint_torques):
                        full_tau.append(msg_joint_torques[idx])
                    else:
                        full_tau.append(0.0)
                else:
                    full_pos.append(0.0)
                    full_tau.append(0.0)
            
            self.joint_position_sim = full_pos
            self.joint_torque_sim = full_tau


import sympy, sympybotics, scipy.signal, lmi_sdp, subprocess
from sympy import BlockMatrix, Matrix, eye, Identity
from lmi_sdp import LMI
from sympybotics.dynident.regression import mrepl
from dyn_params_ident.urdf2dh import get_dh_params_for_arm, get_gravity_in_arm_base

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
        self.solutions_beta_starextra = None
        self.rbt = None

    # todo: 补充v03模型
    def create_robot_model(self, robot_type):
        # 构建机器人模型, 并计算基础参数集
        arm_side = "left" if self.is_left else "right"
        dh_params = get_dh_params_for_arm(arm_side)
        self.log_msg(f"DH parameters for {robot_type} {arm_side} arm:\n{dh_params}")
        gravity = get_gravity_in_arm_base(arm_side)
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
        # 创建机器人模型并计算基础参数集
        self.log_msg("正在进行机器人建模...")
        self.rbt = self.create_robot_model(self.robot_type)
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
        self.solutions_beta_starextra = np.matrix(beta_starextra)
        # 验证beta_starextra解
        return self.test_mass_matrix_psd(self.solutions_beta_starextra)

    def process_and_cache_data(self):
        for traj in ["traj1", "traj2"]:
            t_raw, q_raw, tau_raw, t, q, dq, ddq, tau, W, omega, Q1, R1, rho1 = (
                self.process_regression_data(traj)
            )
            self.proc_data[traj] = {
                "t_raw": t_raw,
                "q_raw": q_raw,
                "tau_raw": tau_raw,
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
            }
        self.log_msg("所有轨迹数据处理完成并保存。")

    def process_regression_data(self, traj):
        # 数据滤波, 差分和回归数据生成
        fc_mult = 10
        fc = fc_mult * (self.wf / (2 * np.pi) * self.L)
        si = int(3 / self.dt)
        sf = -int(1 / self.dt)
        # 使用 Path / 拼接，并在 read_data 时转成 str
        rec_filename = f"rbtlog_{traj}.dat"
        sub_dir = "left" if self.is_left else "right"
        rec_path = os.path.join(self.rec_dir, sub_dir, rec_filename)
        traj_path = os.path.join(self.traj_dir, sub_dir, f"{traj}.dat")
        t_raw, q_raw, tau_raw, t_ref, q_ref = read_data(
            self.ndof, self.dt, rec_path, traj_path
        )

        q, dq, ddq, tau = self.diff_and_filt_data(
            dof=self.ndof,
            h=self.dt,
            q_raw=q_raw,
            tau_raw=tau_raw,
            fc_q=fc,
            fc_dq=fc,
            fc_ddq=fc,
            fc_tau=float("Inf"),
        )
        t_raw = t_raw[si:sf]
        q_raw = q_raw[si:sf]
        tau_raw = tau_raw[si:sf]
        q = q[si:sf]
        dq = dq[si:sf]
        ddq = ddq[si:sf]
        tau = tau[si:sf]
        t_raw = t_raw - t_raw[0]
        t = np.array(range(q.shape[0])) * self.dt
        W, omega, Q1, R1, rho1 = self.gen_regr_matrices(self.rbt, q, dq, ddq, tau)
        return t_raw, q_raw, tau_raw, t, q, dq, ddq, tau, W, omega, Q1, R1, rho1

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

    def gen_regr_matrices(self, rbt, q, dq, ddq, tau):
        local_dict = {}
        exec(
            sympybotics.robot_code_to_func(
                "python", rbt.H_code, "H", "regressor_func", rbt.rbtdef
            ),
            globals(),
            local_dict,
        )
        regressor_func = local_dict["regressor_func"]
        global sin, cos, sign
        sin = np.sin
        cos = np.cos
        sign = np.sign
        H_S, omega = self.regr_matrices(
            rbt.dof, rbt.dyn.n_dynparms, q, dq, ddq, tau, regressor_func
        )
        W = np.matrix(H_S[:, rbt.dyn.base_idxs])
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
        wc_tau = fc_tau * 2 * math.pi * h

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
        # 使用内存缓存的数据与解进行验证（默认 traj2 验证，参数来自 traj1）
        traj_proc = self.proc_data.get("traj2")
        if traj_proc is None:
            raise RuntimeError("未找到处理数据，请先执行计算动力学参数。")
        W = np.matrix(traj_proc["W"])
        t = traj_proc["t"]
        tau_raw = np.matrix(traj_proc["tau"])
        if self.solutions_beta_starextra is None:
            raise RuntimeError("未找到辨识结果，请先执行计算动力学参数。")
        omega_pred = W * np.matrix(self.solutions_beta_starextra)
        tau_pred = omega_pred.reshape(
            int(omega_pred.shape[0] / tau_raw.shape[1]), tau_raw.shape[1]
        )
        total = tau_raw.shape[0]
        return t[:total], tau_raw[:total, :], tau_pred[:total, :]


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
        # 停止按钮（左栏最下方，不在真机模块中）
        self.btn_stop = QtWidgets.QPushButton("停止运行")
        ctrl_layout.addStretch(1)
        # 第二部分：真机采集与控制
        bottom_group = QtWidgets.QGroupBox("轨迹真机采集")
        bottom_vbox = QtWidgets.QVBoxLayout(bottom_group)
        bottom_vbox.setContentsMargins(8, 8, 8, 8)
        bottom_vbox.setSpacing(8)
        self.btn_run_real = QtWidgets.QPushButton("真机采集轨迹")
        
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

        self.btn_calc_params = QtWidgets.QPushButton("计算动力学参数")
        self.btn_write_params = QtWidgets.QPushButton("写入辨识参数")
        # 图示已采集轨迹放在真机采集所有轨迹下方
        bottom_vbox.addWidget(self.btn_show_plot)
        ctrl_layout.addWidget(bottom_group)
        ctrl_layout.addStretch(1)
        # 第三部分：轨迹采集及计算结果
        result_group = QtWidgets.QGroupBox("计算参数与写入")
        result_vbox = QtWidgets.QVBoxLayout(result_group)
        result_vbox.setContentsMargins(8, 8, 8, 8)
        result_vbox.setSpacing(8)
        result_vbox.addWidget(self.btn_calc_params)
        result_vbox.addWidget(self.btn_show_regression)
        result_vbox.addWidget(self.btn_write_params)
        ctrl_layout.addWidget(result_group)
        ctrl_layout.addStretch(1)
        # 第四部分：停止按钮
        stop_group = QtWidgets.QGroupBox("停止运动与计算")
        stop_vbox = QtWidgets.QVBoxLayout(stop_group)
        stop_vbox.setContentsMargins(8, 8, 8, 8)
        stop_vbox.setSpacing(8)
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
        hsplitter.addWidget(ctrl_widget)
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
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.dyn)
        self.ros_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.ros_thread.start()

        self.log_signal.connect(self._append_log)

        # 连接信号
        self.btn_open_meshcat.clicked.connect(self._start_visual_node)
        self.btn_run_sim.clicked.connect(self.on_run_sim)
        self.btn_run_real.clicked.connect(self.on_run_real)
        self.btn_show_reference.clicked.connect(self.on_show_reference)
        self.btn_show_plot.clicked.connect(self.on_show_plot)
        self.btn_calc_params.clicked.connect(self.on_calc_params)
        self.btn_write_params.clicked.connect(self.on_write_params)
        # 倍速选择变化
        self.sim_speed_combo.currentTextChanged.connect(self._on_sim_speed_changed)
        self.real_speed_combo.currentTextChanged.connect(self._on_real_speed_changed)
        # 新增：停止按钮连接
        self.btn_stop.clicked.connect(self.on_stop_clicked)
        # 新增：图示辨识结果按钮
        self.btn_show_regression.clicked.connect(self.on_show_regression)
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
        self._captured_finished = False
        self._cancel_requested = False
        self.btn_show_plot.setDisabled(True)
        self.btn_show_regression.setDisabled(True)
        self.btn_write_params.setDisabled(True)
        # 控制可见性：仅当脚本以 --dev 参数运行时才显示该按钮
        self._show_dex_features = "--dev" in sys.argv
        if not getattr(self, "_show_dex_features", False):
            self.btn_calc_params.setDisabled(True)

        self._did_initial_ref_plot = False

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

    def disable_buttons(self, disabled=True):
        # 除停止按钮外均禁用/启用；部分按钮受状态标志控制
        try:
            self.btn_run_sim.setDisabled(disabled)
            self.btn_run_real.setDisabled(disabled)
            # 可视化始终可用
            self.btn_open_meshcat.setDisabled(False)
            # 待采集参考轨迹：仅随操作禁用，不受采集/计算状态限制
            self.btn_show_reference.setDisabled(disabled)
            # 仿真倍速选择在运行/计算时也禁用
            self.sim_speed_combo.setDisabled(disabled)
            # 真机倍速选择在运行/计算时也禁用
            self.real_speed_combo.setDisabled(disabled)
            # 根据采集完成状态决定是否可用
            self.btn_show_plot.setDisabled(
                True if disabled else (not self._captured_finished)
            )
            self.btn_calc_params.setDisabled(
                True if disabled else (not self._captured_finished)
            )
            # 根据计算完成状态决定是否可用
            self.btn_show_regression.setDisabled(
                True if disabled else (not self._params_calculated)
            )
            self.btn_write_params.setDisabled(
                True if disabled else (not self._params_calculated)
            )
            self.traj_combo.setDisabled(disabled)
            self.arm_combo.setDisabled(disabled)
            # 停止按钮始终可用
            self.btn_stop.setDisabled(False)
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
        self.disable_buttons(True)
        t = threading.Thread(target=self._thread_run_real, daemon=True)
        t.start()

    def _thread_run_real(self):
        try:
            if self.dyn.run_real_capture():
                self.log_msg(f"真机采集结束")
                self._captured_finished = True
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

    def on_show_plot(self):
        self.update_params_from_ui()
        self.log_msg("准备图示已采集轨迹...")
        self.disable_buttons(True)
        t = threading.Thread(target=self._thread_show_plot, daemon=True)
        t.start()

    def _thread_show_plot(self):
        try:
            # 绘图时在主线程以安全方式更新 FigureCanvas
            QtCore.QMetaObject.invokeMethod(
                self, "_plot_on_canvas", QtCore.Qt.QueuedConnection
            )
        except Exception as e:
            self.log_msg("绘图出错: " + str(e))
            try:
                QtCore.QMetaObject.invokeMethod(
                    self, "enable_buttons", QtCore.Qt.QueuedConnection
                )
            except Exception:
                pass

    def on_show_regression(self):
        self.update_params_from_ui()
        self.log_msg("准备图示辨识结果")
        self.disable_buttons(True)
        t = threading.Thread(target=self._thread_show_regression, daemon=True)
        t.start()

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
        self.disable_buttons(True)
        self._cancel_requested = False
        t = threading.Thread(target=self._thread_calc_params, daemon=True)
        t.start()

    def _thread_calc_params(self):
        try:
            self.log_msg("开始计算左臂动力学参数...")
            if not self.dyn.calculator_left.calc_dyn_params():
                self.log_msg("左臂动力学参数计算失败。")
                return
            self.log_msg("左臂动力学参数计算完成。")
            if getattr(self, "_cancel_requested", False):
                self.log_msg("已停止后续计算。")
                return
            self.log_msg("开始计算右臂动力学参数...")
            if not self.dyn.calculator_right.calc_dyn_params():
                self.log_msg("右臂动力学参数计算失败。")
                return
            self.log_msg("右臂动力学参数计算完成。")
            self.log_msg("动力学参数计算全部完成。")
            # 计算完成，启用写入按钮
            self._params_calculated = True
            try:
                self.btn_write_params.setDisabled(False)
                self.btn_show_regression.setDisabled(False)
            except Exception:
                pass
            self._plot_regression_on_canvas()
        except Exception as e:
            self.log_msg("动力学参数计算出错: " + str(e))
        finally:
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

    def on_write_params(self):
        try:
            # 未完成计算则不允许写入
            if not getattr(self, "_params_calculated", False):
                self.log_msg("请先完成动力学参数计算后再写入。")
                return

            # 获取左右臂参数
            beta_left = self._get_current_beta(True)
            beta_right = self._get_current_beta(False)

            version_prefix = self.dyn.version[:3]
            robot_version_key = f"{self.dyn.robot_name}_{version_prefix}"

            # 构造确认弹窗（中文纯文字，无图标，置顶模态）
            msg = (
                "将写入左右臂辨识参数到机器人配置文件 robot.yaml：\n\n"
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
            
            beta_left_list = [float(x) for x in beta_left]
            beta_right_list = [float(x) for x in beta_right]
            
            def format_float_list(flist):
                return "[" + ", ".join(map(str, flist)) + "]"

            import re

            primary_pattern = rf"({robot_version_key}:.*?dyn_base_params_primary:\s*)\[.*?\]"
            secondary_pattern = rf"({robot_version_key}:.*?dyn_base_params_secondary:\s*)\[.*?\]"
            
            new_content = content
            if re.search(primary_pattern, content, re.DOTALL):
                new_content = re.sub(primary_pattern, 
                                   f"\\1{format_float_list(beta_left_list)}", 
                                   new_content, flags=re.DOTALL)
                self.log_msg(f"已更新 {robot_version_key} -> dyn_base_params_primary")
            
            if re.search(secondary_pattern, new_content, re.DOTALL):
                new_content = re.sub(secondary_pattern, 
                                   f"\\1{format_float_list(beta_right_list)}", 
                                   new_content, flags=re.DOTALL)
                self.log_msg(f"已更新 {robot_version_key} -> dyn_base_params_secondary")

            with open(cfg_path, "w", encoding="utf-8") as f:
                f.write(new_content)
            
            self.log_msg(
                f"已写入左右臂辨识参数到机器人配置文件{cfg_path}, 重启服务后配置生效."
            )
        except Exception as e:
            self.log_msg("写入辨识参数出错: " + str(e))

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
            try:
                self.left_fig.clear()
                axes_left = [
                    self.left_fig.add_subplot(rows, cols, j + 1)
                    for j in range(rows * cols)
                ]
                calc_left = self.dyn.calculator_left
                tL, tau_raw_L, tau_pred_L = calc_left.get_regr_validatation()
                totalL = tau_raw_L.shape[0]
                for i in range(min(ndof, tau_raw_L.shape[1])):
                    ax = axes_left[i]
                    ax.clear()
                    ax.plot(
                        tL[:totalL],
                        np.asarray(tau_raw_L[:, i]).flatten()[:totalL],
                        label="Measured torque",
                        color="tab:red",
                        linewidth=1,
                    )
                    ax.plot(
                        tL[:totalL],
                        np.asarray(tau_pred_L[:, i]).flatten()[:totalL],
                        label="Estimated torque",
                        color="tab:blue",
                        linewidth=1,
                    )
                    ax.set_title(f"Joint {i+1}")
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
            except Exception as e:
                self.log_msg(
                    "左臂辨识绘图错误: "
                    + str(e)
                    + "。请先计算动力学参数并确保数据存在。"
                )

            # 右臂辨识
            try:
                self.right_fig.clear()
                axes_right = [
                    self.right_fig.add_subplot(rows, cols, j + 1)
                    for j in range(rows * cols)
                ]
                calc_right = self.dyn.calculator_right
                tR, tau_raw_R, tau_pred_R = calc_right.get_regr_validatation()
                totalR = tau_raw_R.shape[0]
                for i in range(min(ndof, tau_raw_R.shape[1])):
                    ax = axes_right[i]
                    ax.clear()
                    ax.plot(
                        tR[:totalR],
                        np.asarray(tau_raw_R[:, i]).flatten()[:totalR],
                        label="Measured torque",
                        color="tab:red",
                        linewidth=1,
                    )
                    ax.plot(
                        tR[:totalR],
                        np.asarray(tau_pred_R[:, i]).flatten()[:totalR],
                        label="Estimated torque",
                        color="tab:blue",
                        linewidth=1,
                    )
                    ax.set_title(f"Joint {i+1}")
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
            except Exception as e:
                self.log_msg(
                    "右臂辨识绘图错误: "
                    + str(e)
                    + "。请先计算动力学参数并确保数据存在。"
                )

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
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
