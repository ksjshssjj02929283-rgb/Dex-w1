import os
import sys
import math
import time
import json
import threading
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from joint_interfaces.msg import JointPositionControl
from std_msgs.msg import Header, String
from sensor_msgs.msg import JointState
import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')
from PyQt5 import QtWidgets, QtCore
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from ament_index_python.packages import get_package_share_directory

DEFAULT_DT = 0.01
DEFAULT_NDOF = 7

class DynParamIdentifier(Node):
    def __init__(self, traj='traj1', is_left=False):
        super().__init__('qt_dyn_params_ident')
        self.subscription = self.create_subscription(JointState, '/feedback/joint', self._joint_state_callback, 10)
        self.robot_state_subscription = self.create_subscription(String, '/feedback/robot_server_state', self._robot_state_callback, 10)
        self.subscription = self.create_subscription(JointState, '/feedback/joint_sim', self._joint_state_callback_sim, 10)
        self.robot_state_subscription_sim = self.create_subscription( String, '/feedback/robot_server_state_sim', self._robot_state_callback_sim, 10)
        self.joint_publisher = self.create_publisher(JointPositionControl, '/control/joint_position', 10)
        self.joint_publisher_sim = self.create_publisher(JointPositionControl, '/control/joint_position_sim', 10)
        self.traj = traj
        self.robot_cfg_path = Path(get_package_share_directory("w1_server")) / "robot.cfg"
        self.is_left = is_left
        with open(self.robot_cfg_path, 'r', encoding='utf-8') as f:
            cfg = json.load(f)
        robot_name = cfg.get("robot_name", "")
        self.is_anth = (robot_name == "W1Anthropomorphic")
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
        self.project_root = Path(__file__).resolve().parent
        robot_type = 'w1_anth' if self.is_anth else 'w1_indst'
        self.traj_dir = self.project_root / 'dyn_params_ident_data' / 'trajectories'
        self.rec_dir = self.project_root / 'dyn_params_ident_data' / 'recdata' / robot_type #/ arm_type
        self.cad_dir = self.project_root / 'dyn_params_ident_data' / 'robotparams' / robot_type #/ arm_type
        self.model_dir = self.project_root / 'dyn_params_ident_data' / 'tmp' / 'robotmodels' / robot_type
        self.proc_dir = self.project_root / 'dyn_params_ident_data' / 'tmp' / 'procdata' / robot_type
        self.sdpa_dir = self.project_root / 'dyn_params_ident_data' / 'tmp' / 'sdpa_dat' / robot_type
        self.solution_dir = self.project_root / 'dyn_params_ident_data' / 'tmp' / 'solutions' / robot_type
        for d in (self.traj_dir, self.rec_dir, self.cad_dir, self.model_dir,
                  self.proc_dir, self.sdpa_dir, self.solution_dir):
            d.mkdir(parents=True, exist_ok=True)
        self.traj_path = os.path.join(self.traj_dir, f"{self.traj}.dat")
        # 创建定时器检查连接状态
        self._last_msg_time = None
        self._last_msg_time_sim = None
        rclpy.spin_once(self, timeout_sec=0.1)  # 让订阅生效
        self.create_timer(0.1, self._check_connection_status)
        self._gui_logger = None
        self.log_msg(f"检测到机器人类型为: {robot_name}, 初始化并连接成功")

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

    def set_gui_logger(self, logger_callable):
        """由 GUI 注入一个可调用，用于在 Qt 窗口显示日志。
        例如传入 DynParamGUI.log_msg。
        """
        self._gui_logger = logger_callable

    def log_msg(self, msg: str):
        """统一的日志输出接口，供节点内部调用。
        在无 GUI 环境下使用节点 logger 与标准输出，避免属性缺失错误。
        """
        try:
            if self._gui_logger is not None:
                self._gui_logger(str(msg))
        except Exception:
            pass

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

    def run_traj_sim(self, traj_path):
        """
        在仿真中测试轨迹的可行性。
        """
        self._stop_flag.clear()
        with open(traj_path, 'r' ) as file:
            q_ref = np.loadtxt(file)
        if self.is_left:
            # 运动到第一个点
            target_position = [0.5, -1.0, 0.5, 0.0, 0.0, 0.0, *q_ref[0, :], -0.5, 1.2, 0.0, 1.0, 0.0, 0.5, 0.5]
            self.move_joint(is_sim=True, target_position=target_position)
            self.log_msg(f"开始执行左臂仿真轨迹: {self.traj} 当前qpos: {target_position}")
            next_t = time.monotonic()
            for i in range(q_ref.shape[0]):
                if self._stop_flag.is_set():
                    self.log_msg("仿真停止指令收到，提前结束。")
                    break
                target_position = [0.5, -1.0, 0.5, 0.0, 0.0, 0.0, *q_ref[i, :], -0.5, 1.2, 0.0, 1.0, 0.0, 0.5, 0.5]
                self._joint_publisher(is_sim=True, target_position=target_position)
                next_t = self._fixed_rate_next(next_t, self.dt)
                self._sleep_until(next_t)
        else:
            q_ref = -q_ref  # 右臂角度相反
            # 运动到第一个点
            target_position = [0.5, -1.0, 0.5, 0.0, 0.0, 0.0, 0.5, -1.2, 0.0, -1.0, 0.0, -0.5, -0.5, *q_ref[0, :]]
            self.move_joint(is_sim=True, target_position=target_position)
            self.log_msg(f"开始执行右臂仿真轨迹: {self.traj}, 当前qpos: {target_position}")
            next_t = time.monotonic()
            for i in range(q_ref.shape[0]):
                if self._stop_flag.is_set():
                    self.log_msg("仿真停止指令收到，提前结束。")
                    break
                target_position = [0.5, -1.0, 0.5, 0.0, 0.0, 0.0, 0.5, -1.2, 0.0, -1.0, 0.0, -0.5, -0.5, *q_ref[i, :]]
                self._joint_publisher(is_sim=True, target_position=target_position)
                next_t = self._fixed_rate_next(next_t, self.dt)
                self._sleep_until(next_t)
        if not self._stop_flag.is_set():
            time.sleep(1)  # 发送结束后等待1s
            self.log_msg("仿真轨迹执行完成, 返回初始姿态。")
            # 返回初始姿态
            target_position = [0.5, -1.0, 0.5, 0.0, 0.0, 0.0, 0.5, -1.2, 0.0, -1.0, 0.0, -0.5, -0.5, -0.5, 1.2, 0.0, 1.0, 0.0, 0.5, 0.5]
            self.move_joint(is_sim=True, target_position=target_position)

    def run_real_capture(self):
        """
        在真机上依次执行左臂三段轨迹 + 右臂三段轨迹，并记录轨迹数据。
        """
        self._stop_flag.clear()
        orig_is_left = self.is_left
        orig_traj = self.traj

        traj_list = ["traj1", "traj2", "traj3"]
        left_dir = os.path.join(self.rec_dir, "left")
        right_dir = os.path.join(self.rec_dir, "right")
        os.makedirs(left_dir, exist_ok=True)
        os.makedirs(right_dir, exist_ok=True)

        # 先左臂
        self.is_left = True
        for tname in traj_list:
            if self._stop_flag.is_set():
                self.log_msg("收到停止指令，终止剩余真机采集。")
                break
            self.traj = tname
            self.traj_path = os.path.join(self.traj_dir, f"{self.traj}.dat")
            rec_path = os.path.join(left_dir, f"rbtlog_{self.traj}.dat")
            self._run_real_capture_once(self.traj_path, rec_path)
        # 再右臂
        if not self._stop_flag.is_set():
            self.is_left = False
            for tname in traj_list:
                if self._stop_flag.is_set():
                    self.log_msg("收到停止指令，终止剩余真机采集。")
                    break
                self.traj = tname
                self.traj_path = os.path.join(self.traj_dir, f"{self.traj}.dat")
                rec_path = os.path.join(right_dir, f"rbtlog_{self.traj}.dat")
                self._run_real_capture_once(self.traj_path, rec_path)
        # 恢复初始状态
        self.is_left = orig_is_left
        self.traj = orig_traj
        self.traj_path = os.path.join(self.traj_dir, f"{self.traj}.dat")
        if not self._stop_flag.is_set():
            time.sleep(1)  # 发送结束后等待1s
            self.log_msg("所有轨迹采集完成, 返回初始姿态。")
            target_position = [0.5, -1.0, 0.5, 0.0, 0.0, 0.0, 0.5, -1.2, 0.0, -1.0, 0.0, -0.5, -0.5, -0.5, 1.2, 0.0, 1.0, 0.0, 0.5, 0.5]
            self.move_joint(is_sim=False, target_position=target_position)
            self.log_msg("已返回初始姿态")

    def _run_real_capture_once(self, traj_path, rec_path):
        with open(traj_path, 'r' ) as file:
            q_ref = np.loadtxt(file)
        rec = np.zeros((q_ref.shape[0], 1 + 7 + 7))
        if self.is_left:
            # 运动到第一个点
            target_position = [0.5, -1.0, 0.5, 0.0, 0.0, 0.0, *q_ref[0, :], -0.5, 1.2, 0.0, 1.0, 0.0, 0.5, 0.5]
            self.move_joint(is_sim=False, target_position=target_position)
            self.log_msg(f"开始执行左臂真机采集: {self.traj}")
            time0 = self.timestamp
            next_t = time.monotonic()
            for i in range(q_ref.shape[0]):
                if self._stop_flag.is_set():
                    self.log_msg("收到停止指令，终止当前段采集。")
                    break
                target_position = [0.5, -1.0, 0.5, 0.0, 0.0, 0.0, *q_ref[i, :], -0.5, 1.2, 0.0, 1.0, 0.0, 0.5, 0.5]
                self._joint_publisher(is_sim=False, target_position=target_position)
                rec[i, :] = np.concatenate([
                    np.array([float(self.timestamp - time0) * 1e-9]),
                    np.array(self.joint_position[6:13]),
                    np.array(self.joint_torque[6:13])
                ])
                next_t = self._fixed_rate_next(next_t, self.dt)
                self._sleep_until(next_t)
        else:
            q_ref = -q_ref  # 右臂角度相反
            # 运动到第一个点
            target_position = [0.5, -1.0, 0.5, 0.0, 0.0, 0.0, 0.5, -1.2, 0.0, -1.0, 0.0, -0.5, -0.5, *q_ref[0, :]]
            self.move_joint(is_sim=False, target_position=target_position)
            self.log_msg(f"开始执行右臂真机采集: {self.traj}")
            time0 = self.timestamp
            next_t = time.monotonic()
            for i in range(q_ref.shape[0]):
                if self._stop_flag.is_set():
                    self.log_msg("收到停止指令，终止当前段采集。")
                    break
                target_position = [0.5, -1.0, 0.5, 0.0, 0.0, 0.0, 0.5, -1.2, 0.0, -1.0, 0.0, -0.5, -0.5, *q_ref[i, :]]
                self._joint_publisher(is_sim=False, target_position=target_position)
                rec[i, :] = np.concatenate([
                    np.array([float(self.timestamp - time0) * 1e-9]),
                    np.array(self.joint_position[13:20]),
                    np.array(self.joint_torque[13:20])
                ])
                next_t = self._fixed_rate_next(next_t, self.dt)
                self._sleep_until(next_t)
        # 仅在未停止时保存
        if not self._stop_flag.is_set():
            np.savetxt(rec_path, rec, fmt="%.6f")
            time.sleep(1)  # 发送结束后等待1s
            self.log_msg(f"本段轨迹采集完成, 数据已保存到 {rec_path}")

    def move_joint(self, is_sim: bool, target_position: list, max_speed=0.3):
        if self.movej_started:
            self.log_msg("已有插值movej移动在进行中，忽略新的移动请求")
            return False
        self._stop_flag.clear()
        if is_sim:
            self.is_sim_connected()
            joint_position = self.joint_position_sim
        else:
            self.is_real_connected()
            joint_position = self.joint_position

        if joint_position is None:
            self.log_msg("当前关节位置未知，无法进行插值movej移动。")
            return False

        max_distance = max([abs(tp - cp) for tp, cp in zip(target_position, joint_position)])
        total_points = max(2, math.ceil(max_distance / max_speed / self.dt))
        interp_positions = np.linspace(np.array(joint_position), np.array(target_position), total_points)
        self.movej_started = True
        self.log_msg(f"执行move_joint, 当前是否仿真: {is_sim}, 当前关节位置: {joint_position}, 目标关节位置: {target_position}，共 {total_points} 点，轨迹时长 {total_points * self.dt} 秒")

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

    def _stop_move(self):
        """
        停止当前所有轨迹下发。
        """
        self._stop_flag.set()
        self.log_msg("已请求停止当前轨迹下发。")

    def _joint_publisher(self, is_sim: bool, target_position: list):
        if (is_sim and (self.status_sim in ("Init", "Error"))) or ((not is_sim) and (self.status in ("Init", "Error"))):
            self.log_msg("检测到机器人未处于Enable状态, 停止发送指令")
            return False
        if self.joint_name is None:
            self.log_msg("当前关节名称未知，无法移动。")
            return False
        if is_sim:
            self.is_sim_connected()
            publisher = self.joint_publisher_sim
            joint_name = self.joint_name
        else:
            self.is_real_connected()
            publisher = self.joint_publisher
            joint_name = self.joint_name

        msg = JointPositionControl()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = target_position
        msg.name = joint_name
        publisher.publish(msg)
        return True

    def plot_records_to_axes(self, ax_handles):
        """
        读取记录并绘制到传入的 matplotlib axes 列表。
        如果 ax_handles 为 None，则使用 plt.show() 弹窗（兼容单独调用）。
        """
        rec_filename = f"rbtlog_{self.traj}.dat"
        side_dir = "left" if self.is_left else "right"
        rec_path = os.path.join(self.rec_dir, side_dir, rec_filename)
        traj_path = os.path.join(self.traj_dir, f"{self.traj}.dat")

        t_raw, q_raw, tau_raw, t_ref, q_ref = self._read_data(self.ndof, self.dt, rec_path, traj_path)
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
            ref_series = q_ref_cut[:, j] if self.is_left else -q_ref_cut[:, j]
            ax1.plot(t_ref_cut, ref_series, label='Reference q', color='tab:red', linewidth=1)
            ax1.plot(t_raw_cut, q_raw_cut[:, j], label='Measured q', color='tab:green', alpha=0.8, linewidth=1)
            ax1.set_title(f'Joint {j+1}')
            ax1.grid(True, linestyle='--', alpha=0.5)
            ax1.set_xlabel('Time (s)')
            ax1.set_ylabel('Angle (rad)', color='tab:blue')
            ax2 = ax1.twinx()
            ax2.plot(t_raw_cut, tau_cut[:, j], label='Measured τ (Nm)', color='tab:blue', linewidth=1)
            ax2.set_ylabel('Torque (Nm)', color='tab:red')
            ax1.tick_params(axis='y', labelcolor='tab:blue')
            ax2.tick_params(axis='y', labelcolor='tab:red')
            ax1.legend(loc='upper right', fontsize='small')

    def _robot_state_callback(self, msg):
        self._last_msg_time = self.get_clock().now().nanoseconds / 1e9
        state = json.loads(msg.data)
        if "timestamp" in state:
            self.timestamp = state["timestamp"]
        if "status" in state:
            self.status = state["status"]
        if "joint_position" in state:
            self.joint_position = [float(x) for x in state["joint_position"]]
        if "joint_torque" in state:
            self.joint_torque = [float(x) for x in state["joint_torque"]]

    def _joint_state_callback(self, msg):
        self._last_msg_time = self.get_clock().now().nanoseconds / 1e9
        self.joint_name = msg.name

    def _joint_state_callback_sim(self, msg):
        self._last_msg_time_sim = self.get_clock().now().nanoseconds / 1e9
        self.joint_name = msg.name

    def _robot_state_callback_sim(self, msg):
        self._last_msg_time_sim = self.get_clock().now().nanoseconds / 1e9
        state = json.loads(msg.data)
        if "status" in state:
            self.status_sim = state["status"]
        if "joint_position" in state:
            self.joint_position_sim = [float(x) for x in state["joint_position"]]
        if "joint_torque" in state:
            self.joint_torque_sim = [float(x) for x in state["joint_torque"]]

    def _read_data(self, dof, h, rbtlogfile, trajreffile ):
        rbtlog = np.loadtxt(rbtlogfile)
        s = rbtlog.shape[0]
        t = np.array(rbtlog[:,0])
        q = np.array(rbtlog[:,1:dof+1])
        tau = np.array(rbtlog[:,dof+1:dof*2+1])
        trajref = np.loadtxt(trajreffile)
        reft = np.array([ h*i for i in range(trajref.shape[0]) ])
        return t, q, tau, reft, trajref


class DynParamGUI(QtWidgets.QMainWindow):
    log_signal = QtCore.pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("动力学参数辨识")
        self.resize(1400, 900)
        # 记录原始全局字体，用于日志保持不变
        original_font = QtWidgets.QApplication.font()
        # 全局进一步减小字体（除日志外）
        small_font = QtWidgets.QApplication.font()
        small_font.setPointSize(max(8, small_font.pointSize() - 3))
        QtWidgets.QApplication.setFont(small_font)
        # 保存以便后续日志控件恢复
        self._original_app_font = original_font

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
        top_layout.addWidget(ctrl_widget, 0)

        self.traj_combo = QtWidgets.QComboBox()
        self.traj_combo.addItems(["轨迹1", "轨迹2", "轨迹3"])
        self.traj_combo.setCurrentText("轨迹1")
        # 新增：臂侧下拉框（左臂/右臂）
        self.arm_combo = QtWidgets.QComboBox()
        self.arm_combo.addItems(["左臂", "右臂"])
        self.arm_combo.setCurrentText("左臂")

        app_pt = QtWidgets.QApplication.font().pointSize()
        cf = self.arm_combo.font(); cf.setPointSize(app_pt); self.arm_combo.setFont(cf)
        tf = self.traj_combo.font(); tf.setPointSize(app_pt); self.traj_combo.setFont(tf)
        # 下拉列表视图也统一字体
        arm_view = QtWidgets.QListView(); arm_view.setFont(cf); self.arm_combo.setView(arm_view)
        traj_view = QtWidgets.QListView(); traj_view.setFont(tf); self.traj_combo.setView(traj_view)

        check_group = QtWidgets.QGroupBox("单段轨迹检查与图示")
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
        # 操作按钮（仿真与图示）
        self.btn_run_sim = QtWidgets.QPushButton("运行仿真")
        self.btn_show_plot = QtWidgets.QPushButton("图示已采集轨迹")
        btn_h = max(self.btn_run_sim.sizeHint().height(), self.btn_show_plot.sizeHint().height())
        self.arm_combo.setFixedHeight(btn_h)
        self.traj_combo.setFixedHeight(btn_h)
        # 让下拉项高度也更协调
        self.arm_combo.setStyleSheet(f"QComboBox QAbstractItemView::item{{height:{btn_h - 6}px;}}")
        self.traj_combo.setStyleSheet(f"QComboBox QAbstractItemView::item{{height:{btn_h - 6}px;}}")
        check_vbox.addWidget(self.btn_run_sim)
        check_vbox.addWidget(self.btn_show_plot)
        ctrl_layout.addWidget(check_group)

        # 选择后立刻更新
        self.traj_combo.currentTextChanged.connect(self.update_params_from_ui)
        self.arm_combo.currentTextChanged.connect(self.update_params_from_ui)

        # 拉伸，将真机采集与停止组推至面板底部
        ctrl_layout.addStretch(1)
        # 底部模块：真机采集与控制
        bottom_group = QtWidgets.QGroupBox("真机采集与控制")
        bottom_vbox = QtWidgets.QVBoxLayout(bottom_group)
        bottom_vbox.setContentsMargins(8, 8, 8, 8)
        bottom_vbox.setSpacing(8)
        self.btn_run_real = QtWidgets.QPushButton("真机采集所有轨迹")
        self.btn_stop = QtWidgets.QPushButton("停止运动")
        bottom_vbox.addWidget(self.btn_run_real)
        bottom_vbox.addWidget(self.btn_stop)
        ctrl_layout.addWidget(bottom_group)

        # 底部整行日志显示窗口（可拖动调整高度）
        self.log = QtWidgets.QTextEdit()
        self.log.setReadOnly(True)
        self.log.setMinimumHeight(120)
        try:
            lf = self.log.font()
            lf.setPointSize(max(8, lf.pointSize() - 2))
            self.log.setFont(lf)
        except Exception:
            pass
        # 使用垂直分割器，允许用户拖动调整上部与日志的高度
        splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        splitter.addWidget(top_container)
        splitter.addWidget(self.log)
        # 设置初始尺寸分配：上部占较多空间，日志占较少空间
        splitter.setSizes([500, 180])
        main_layout.addWidget(splitter)

        plot_widget = QtWidgets.QWidget()
        plot_layout = QtWidgets.QVBoxLayout(plot_widget)
        top_layout.addWidget(plot_widget, 1)

        self.figure = Figure(figsize=(8, 6), constrained_layout=True)
        self.canvas = FigureCanvas(self.figure)
        plot_layout.addWidget(self.canvas)
        self.canvas.mpl_connect('resize_event', self._on_resize)
        self.dyn = DynParamIdentifier()
        # 将 GUI 日志函数注入节点，统一在窗口显示日志
        try:
            self.dyn.set_gui_logger(self.log_msg)
        except Exception:
            pass
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.dyn)
        self.ros_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.ros_thread.start()

        self.log_signal.connect(self._append_log)

        # 连接信号
        self.btn_run_sim.clicked.connect(self.on_run_sim)
        self.btn_run_real.clicked.connect(self.on_run_real)
        self.btn_show_plot.clicked.connect(self.on_show_plot)
        # 新增：停止按钮连接
        self.btn_stop.clicked.connect(self.on_stop_clicked)

    def log_msg(self, msg):
        self.log_signal.emit(msg)
        print(msg)

    @QtCore.pyqtSlot(str)
    def _append_log(self, msg):
        self.log.append(msg)

    def update_params_from_ui(self):
        # 从下拉框读取轨迹名与臂侧（显示名 -> 内部值映射）
        traj_text = self.traj_combo.currentText().strip()
        traj_map = {"轨迹1": "traj1", "轨迹2": "traj2", "轨迹3": "traj3"}
        self.dyn.traj = traj_map.get(traj_text, "traj1")
        self.dyn.is_left = (self.arm_combo.currentText() == "左臂")
        self.dyn.traj_path = os.path.join(self.dyn.traj_dir, f"{self.dyn.traj}.dat")

    def disable_buttons(self, disabled=True):
        self.btn_run_sim.setDisabled(disabled)
        self.btn_run_real.setDisabled(disabled)
        self.btn_show_plot.setDisabled(disabled)
        # 停止按钮在执行期间应可用
        self.btn_stop.setDisabled(False if disabled else False)

    @QtCore.pyqtSlot()
    def enable_buttons(self):
        self.disable_buttons(False)

    def on_run_sim(self):
        self.update_params_from_ui()
        self.log_msg("开始运行仿真...")
        self.disable_buttons(True)
        t = threading.Thread(target=self._thread_run_sim, daemon=True)
        t.start()

    def _thread_run_sim(self):
        try:
            self.dyn.run_traj_sim(self.dyn.traj_path)
            self.log_msg("仿真运行结束。")
        except Exception as e:
            self.log_msg("仿真出错: " + str(e))
        finally:
            try:
                QtCore.QMetaObject.invokeMethod(self, "enable_buttons", QtCore.Qt.QueuedConnection)
            except Exception:
                pass

    def on_run_real(self):
        self.update_params_from_ui()
        self.log_msg("开始进行真机采集, 轨迹范围较大, 请注意安全并离开机器人手臂运动范围! 左右臂各采集3段轨迹, 共6段, 采集时间预计7分钟")
        self.disable_buttons(True)
        t = threading.Thread(target=self._thread_run_real, daemon=True)
        t.start()

    def _thread_run_real(self):
        try:
            self.dyn.run_real_capture()
            self.log_msg(f"真机采集结束")
        except Exception as e:
            self.log_msg("真机采集出错: " + str(e))
        finally:
            try:
                QtCore.QMetaObject.invokeMethod(self, "enable_buttons", QtCore.Qt.QueuedConnection)
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
            QtCore.QMetaObject.invokeMethod(self, "_plot_on_canvas", QtCore.Qt.QueuedConnection)
        except Exception as e:
            self.log_msg("绘图出错: " + str(e))
            try:
                QtCore.QMetaObject.invokeMethod(self, "enable_buttons", QtCore.Qt.QueuedConnection)
            except Exception:
                pass

    @QtCore.pyqtSlot()
    def _plot_on_canvas(self):
        try:
            self.figure.clear()
            ndof = self.dyn.ndof
            cols = 3
            rows = math.ceil(ndof / cols)
            axes = []
            for j in range(rows * cols):
                ax = self.figure.add_subplot(rows, cols, j + 1)
                axes.append(ax)
            try:
                self.dyn.plot_records_to_axes(ax_handles=axes)
                for k in range(len(axes)):
                    if k >= ndof:
                        axes[k].set_visible(False)
                try:
                    self.figure.tight_layout()
                except Exception:
                    pass
                self.canvas.draw()
                self.log_msg("绘图完成。")
            except Exception as e:
                self.log_msg("绘图过程中错误: " + str(e) + "，请检查已采集数据是否存在, 未采集请先进行真机采集。")
        finally:
            self.disable_buttons(False)

    @QtCore.pyqtSlot()
    def _on_resize(self, event):
        try:
            self.figure.tight_layout()
            self.canvas.draw_idle()
        except Exception:
            # 忽略任何重布局错误，保持 UI 可用
            pass

    def on_stop_clicked(self):
        try:
            self.log_msg("收到停止指令，正在请求停止...")
            self.dyn._stop_move()
        except Exception as e:
            self.log_msg("停止流程出错: " + str(e))
        finally:
            # 停止后允许再次操作
            try:
                QtCore.QMetaObject.invokeMethod(self, "enable_buttons", QtCore.Qt.QueuedConnection)
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
        finally:
            super().closeEvent(event)


def main():
    rclpy.init()
    app = QtWidgets.QApplication(sys.argv)
    win = DynParamGUI()
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()