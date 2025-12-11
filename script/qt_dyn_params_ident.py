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

DEFAULT_DT = 0.01
DEFAULT_NDOF = 7

class DynParamIdentifier(Node):
    def __init__(self,
                 traj='traj1',
                 sim_check=False,
                 real_capture=False,
                 plot_records=False,
                 is_anth=False,
                 is_left=False):
        super().__init__('qt_dyn_params_ident')
        self.subscription = self.create_subscription(
            JointState,
            '/feedback/joint',
            self._joint_state_callback,
            10)
        self.robot_state_subscription = self.create_subscription(
            String,
            '/feedback/robot_server_state',
            self._robot_state_callback,
            10)
        self.subscription = self.create_subscription(
            JointState,
            '/feedback/joint_sim',
            self._joint_state_callback_sim,
            10)
        self.robot_state_subscription_sim = self.create_subscription(
            String,
            '/feedback/robot_server_state_sim',
            self._robot_state_callback_sim,
            10)
        self.joint_publisher = self.create_publisher(
            JointPositionControl,
            '/control/joint_position',
            10)
        self.joint_publisher_sim = self.create_publisher(
            JointPositionControl,
            '/control/joint_position_sim',
            10)
        self.traj = traj
        self.sim_check = sim_check
        self.real_capture = real_capture
        self.plot_records = plot_records
        self.is_anth = is_anth
        self.is_left = is_left
        # 初始化状态变量
        self.joint_name = None
        self.status = None
        self.joint_position = None
        self.joint_torque = None
        self.status_sim = None
        self.joint_position_sim = None
        self.joint_torque_sim = None
        self.dt = DEFAULT_DT
        self.ndof = DEFAULT_NDOF
        self.movej_started = False
        # 设置路径
        self.project_root = Path(__file__).resolve().parent
        robot_type = 'w1_anth' if self.is_anth else 'w1_indst'
        arm_type = 'left' if self.is_left else 'right'
        self.traj_dir = self.project_root / 'dyn_params_ident_data' / 'trajectories'
        self.rec_dir = self.project_root / 'dyn_params_ident_data' / 'recdata' / robot_type / arm_type
        self.cad_dir = self.project_root / 'dyn_params_ident_data' / 'robotparams' / robot_type / arm_type
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

    def run_sim(self):
        """
        在仿真中测试轨迹的可行性。
        """
        with open(self.traj_path, 'r' ) as file:
            q_ref = np.loadtxt(file)
        if self.is_left:
            target_position = [0.5, -1.0, 0.5, 0.0, 0.0, 0.0, *q_ref[0, :], -0.5, 1.2, 0.0, 1.0, 0.0, 0.5, 0.5]
            self.interp_and_movej(is_sim=True, target_position=target_position)
            self.get_logger().info(f"开始执行仿真轨迹, 当前qpos: {target_position}")
            for i in range(q_ref.shape[0]):
                target_position = [0.5, -1.0, 0.5, 0.0, 0.0, 0.0, *q_ref[i, :], -0.5, 1.2, 0.0, 1.0, 0.0, 0.5, 0.5]
                self._joint_publisher(is_sim=True, target_position=target_position)
                time.sleep(self.dt)
        else:
            q_ref = -q_ref  # 右臂角度相反
            target_position = [0.5, -1.0, 0.5, 0.0, 0.0, 0.0, 0.5, -1.2, 0.0, -1.0, 0.0, -0.5, -0.5, *q_ref[0, :]]
            self.interp_and_movej(is_sim=True, target_position=target_position)
            self.get_logger().info(f"开始执行仿真轨迹, 当前qpos: {target_position}")
            for i in range(q_ref.shape[0]):
                target_position = [0.5, -1.0, 0.5, 0.0, 0.0, 0.0, 0.5, -1.2, 0.0, -1.0, 0.0, -0.5, -0.5, *q_ref[i, :]]
                self._joint_publisher(is_sim=True, target_position=target_position)
                time.sleep(self.dt)
        time.sleep(1.5)  # 发送结束后等待1.5s
        self.get_logger().info("仿真轨迹执行完成, 返回初始姿态。")
        # 返回初始姿态
        target_position = [0.5, -1.0, 0.5, 0.0, 0.0, 0.0, 0.5, -1.2, 0.0, -1.0, 0.0, -0.5, -0.5, -0.5, 1.2, 0.0, 1.0, 0.0, 0.5, 0.5]
        self.interp_and_movej(is_sim=True, target_position=target_position)

    def run_real_capture(self, rec_filename=None):
        """
        在真机上执行并记录：会调用 data.trajectory_exe.exe_in_real。
        rec_filename: 可选，记录文件名（不含路径），默认 rbtlog_{traj}.dat
        """
        try:
            from data.trajectory_exe import exe_in_real
        except Exception as e:
            raise RuntimeError("无法导入 data.trajectory_exe.exe_in_real: " + str(e))

        try:
            import rclpy
            try:
                rclpy.shutdown()
            except Exception:
                pass
        except Exception:
            pass

        if rec_filename is None:
            rec_filename = f"rbtlog_{self.traj}.dat"
        rec_path = os.path.join(self.rec_dir, rec_filename)
        exe_in_real(self.is_left, str(self.traj_path), rec_path, 1.0 / self.dt if self.dt != 0 else 100.0)
        return rec_path

    def interp_and_movej(self, is_sim: bool, target_position: list, max_speed=0.3):
        if self.movej_started:
            self.get_logger().warn("已有插值movej移动在进行中，忽略新的移动请求")
            return False
        if is_sim:
            self.is_sim_connected()
            joint_position = self.joint_position_sim
        else:
            self.is_real_connected()
            joint_position = self.joint_position

        if joint_position is None:
            self.get_logger().warn("当前关节位置未知，无法进行插值movej移动。")
            return False

        max_distance = max([abs(tp - cp) for tp, cp in zip(target_position, joint_position)])
        total_points = max(2, math.ceil(max_distance / max_speed / self.dt))
        interp_positions = np.linspace(np.array(joint_position), np.array(target_position), total_points)
        self.movej_started = True
        self.get_logger().info(f"当前是否仿真: {is_sim}, 当前关节位置: {joint_position}, 目标关节位置: {target_position}，共 {total_points} 点，轨迹时长 {total_points * self.dt} 秒")

        for step in range(total_points):
            ret = self._joint_publisher(is_sim, interp_positions[step].tolist())
            if not ret:
                self.get_logger().warn("插值movej移动过程中被中止。")
                self.movej_started = False
                return False
            time.sleep(self.dt)
        self.movej_started = False
        time.sleep(1.5)  # 发送结束后等待1.5s
        self.get_logger().info("插值movej移动完成。")
        return True

    def _joint_publisher(self, is_sim: bool, target_position: list):
        if (is_sim and self.status_sim == "Init") or ((not is_sim) and self.status == "Init"):
            self.get_logger().warn("检测到机器人未处于Enable状态, 停止发送指令")
            return False
        if self.joint_name is None:
            self.get_logger().error("当前关节名称未知，无法移动。")
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

    def plot_records_to_axes(self, ax_handles, rec_filename=None):
        """
        读取记录并绘制到传入的 matplotlib axes 列表。
        如果 ax_handles 为 None，则使用 plt.show() 弹窗（兼容单独调用）。
        """
        if rec_filename is None:
            rec_filename = f"rbtlog_{self.traj}.dat"
        rec_path = os.path.join(self.rec_dir, rec_filename)
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

    def _joint_state_callback(self, msg):
        self._last_msg_time = self.get_clock().now().nanoseconds / 1e9
        self.joint_name = msg.name
        self.get_logger().debug(f"接收到关节状态消息: {self.joint_name}")

    def _robot_state_callback(self, msg):
        self._last_msg_time = self.get_clock().now().nanoseconds / 1e9
        state = json.loads(msg.data)
        if "status" in state:
            self.status = state["status"]
        if "joint_position" in state:
            self.joint_position = [float(x) for x in state["joint_position"]]
            self.get_logger().debug(f"当前关节位置: {self.joint_position}")
        if "joint_torque" in state:
            self.joint_torque = [float(x) for x in state["joint_torque"]]

    def _joint_state_callback_sim(self, msg):
        self._last_msg_time_sim = self.get_clock().now().nanoseconds / 1e9
        self.joint_name = msg.name
        self.get_logger().debug(f"(Sim) 接收到关节状态消息: {self.joint_name}")

    def _robot_state_callback_sim(self, msg):
        self._last_msg_time_sim = self.get_clock().now().nanoseconds / 1e9
        state = json.loads(msg.data)
        if "status" in state:
            self.status_sim = state["status"]
        if "joint_position" in state:
            self.joint_position_sim = [float(x) for x in state["joint_position"]]
            self.get_logger().debug(f"(Sim) 当前关节位置: {self.joint_position_sim}")
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
    # 新增：跨线程日志信号
    log_signal = QtCore.pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("动力学参数辨识")
        self.resize(1100, 700)

        # 中央 widget 和布局
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        main_layout = QtWidgets.QHBoxLayout(central)

        # 左侧控制面板
        ctrl_widget = QtWidgets.QWidget()
        ctrl_layout = QtWidgets.QFormLayout(ctrl_widget)
        main_layout.addWidget(ctrl_widget, 0)

        # 参数选择控件
        self.traj_edit = QtWidgets.QLineEdit("traj1")
        self.cb_sim = QtWidgets.QCheckBox("仿真检测 (sim_check)")
        self.cb_real = QtWidgets.QCheckBox("真机采集 (real_capture)")
        self.cb_plot = QtWidgets.QCheckBox("绘制记录 (plot_records)")
        self.cb_anth = QtWidgets.QCheckBox("人机型号 is_anth")
        self.cb_left = QtWidgets.QCheckBox("左臂 is_left")

        ctrl_layout.addRow("轨迹名:", self.traj_edit)
        ctrl_layout.addRow(self.cb_sim)
        ctrl_layout.addRow(self.cb_real)
        ctrl_layout.addRow(self.cb_plot)
        ctrl_layout.addRow(self.cb_anth)
        ctrl_layout.addRow(self.cb_left)

        # 操作按钮
        self.btn_run_sim = QtWidgets.QPushButton("运行仿真")
        self.btn_run_real = QtWidgets.QPushButton("运行真机采集")
        self.btn_show_plot = QtWidgets.QPushButton("显示轨迹图")
        ctrl_layout.addRow(self.btn_run_sim)
        ctrl_layout.addRow(self.btn_run_real)
        ctrl_layout.addRow(self.btn_show_plot)

        # 状态栏显示日志
        self.log = QtWidgets.QTextEdit()
        self.log.setReadOnly(True)
        ctrl_layout.addRow("日志:", self.log)

        # 右侧用于绘图的区域（滚动 + 多子图）
        plot_widget = QtWidgets.QWidget()
        plot_layout = QtWidgets.QVBoxLayout(plot_widget)
        main_layout.addWidget(plot_widget, 1)

        # 使用 constrained_layout 以便自动避免子图重叠
        self.figure = Figure(figsize=(8, 6), constrained_layout=True)
        self.canvas = FigureCanvas(self.figure)
        plot_layout.addWidget(self.canvas)

        # 当窗口大小改变时重新布局（避免标签和标题互相覆盖）
        self.canvas.mpl_connect('resize_event', self._on_resize)

        self.dyn = DynParamIdentifier()

        # 保存 executor 和线程句柄，便于关闭时清理
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.dyn)
        self.ros_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.ros_thread.start()

        # 连接信号：将跨线程日志消息串行化到主线程 UI
        self.log_signal.connect(self._append_log)

        # 连接信号
        self.btn_run_sim.clicked.connect(self.on_run_sim)
        self.btn_run_real.clicked.connect(self.on_run_real)
        self.btn_show_plot.clicked.connect(self.on_show_plot)

    # 修改：log 只发信号，不直接动 UI（线程安全）
    def log_msg(self, msg):
        self.log_signal.emit(msg)
        print(msg)

    # 新增：在主线程更新 QTextEdit，避免跨线程崩溃
    @QtCore.pyqtSlot(str)
    def _append_log(self, msg):
        self.log.append(msg)

    def update_params_from_ui(self):
        self.dyn.traj = self.traj_edit.text().strip() or "traj1"
        self.dyn.sim_check = bool(self.cb_sim.isChecked())
        self.dyn.real_capture = bool(self.cb_real.isChecked())
        self.dyn.plot_records = bool(self.cb_plot.isChecked())
        self.dyn.is_anth = bool(self.cb_anth.isChecked())
        self.dyn.is_left = bool(self.cb_left.isChecked())
        # update paths derived from is_anth/is_left
        robot_type = 'w1_anth' if self.dyn.is_anth else 'w1_indst'
        arm_type = 'left' if self.dyn.is_left else 'right'
        self.dyn.traj_dir = self.dyn.project_root / 'dyn_params_ident_data' / 'trajectories'
        self.dyn.rec_dir = self.dyn.project_root / 'dyn_params_ident_data' / 'recdata' / robot_type / arm_type

    def disable_buttons(self, disabled=True):
        self.btn_run_sim.setDisabled(disabled)
        self.btn_run_real.setDisabled(disabled)
        self.btn_show_plot.setDisabled(disabled)

    def on_run_sim(self):
        self.update_params_from_ui()
        self.log_msg("准备运行仿真...")
        self.disable_buttons(True)
        t = threading.Thread(target=self._thread_run_sim, daemon=True)
        t.start()

    def _thread_run_sim(self):
        try:
            self.dyn.run_sim()
            self.log_msg("仿真运行结束。")
        except Exception as e:
            self.log_msg("仿真出错: " + str(e))
        finally:
            QtCore.QTimer.singleShot(0, lambda: self.disable_buttons(False))

    def on_run_real(self):
        self.update_params_from_ui()
        self.log_msg("准备运行真机采集...")
        self.disable_buttons(True)
        t = threading.Thread(target=self._thread_run_real, daemon=True)
        t.start()

    def _thread_run_real(self):
        try:
            rec_path = self.dyn.run_real_capture()
            self.log_msg(f"真机采集完成，记录保存在: {rec_path}")
        except Exception as e:
            self.log_msg("真机采集出错: " + str(e))
        finally:
            QtCore.QTimer.singleShot(0, lambda: self.disable_buttons(False))

    def on_show_plot(self):
        self.update_params_from_ui()
        self.log_msg("准备绘制轨迹...")
        self.disable_buttons(True)
        t = threading.Thread(target=self._thread_show_plot, daemon=True)
        t.start()

    def _thread_show_plot(self):
        try:
            # 绘图时在主线程以安全方式更新 FigureCanvas
            QtCore.QMetaObject.invokeMethod(self, "_plot_on_canvas", QtCore.Qt.QueuedConnection)
        except Exception as e:
            self.log_msg("绘图出错: " + str(e))
            QtCore.QTimer.singleShot(0, lambda: self.disable_buttons(False))

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
                # 隐藏超过关节数量的空子图，避免空白子图占用视觉空间和产生重叠
                for k in range(len(axes)):
                    if k >= ndof:
                        axes[k].set_visible(False)
                # 再次确保布局紧凑（constrained_layout 已启用，但额外调用能改善一些边缘情况）
                try:
                    self.figure.tight_layout()
                except Exception:
                    pass
                self.canvas.draw()
                self.log_msg("绘图完成。")
            except Exception as e:
                self.log_msg("绘图过程中错误: " + str(e))
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

    # 新增：优雅关闭，减少退出时潜在崩溃
    def closeEvent(self, event):
        try:
            # 尝试移除节点和关闭 rclpy
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