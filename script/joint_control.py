# 使用 Qt 创建关节信息界面，窗口大小 1000x1200
# 每个关节在界面上的显示位置由 joint_info_position 决定

joint_names = [
    # 头
    "HEAD_YAW_J1", "HEAD_PITCH_J2",
    # 腰
    "WAIST_YAW", "WAIST_ROLL",
    # 左臂
    "SHOULDER_PITCH_L_J1", "SHOULDER_ROLL_L_J2", "ELBOW_PITCH_L_J3", "ELBOW_YAW_L_J4", "WRIST_PITCH_L_J5", "WRIST_YAW_L_J6", "WRIST_ROLL_L_J7",
    # 右臂
    "SHOULDER_PITCH_R_J1", "SHOULDER_ROLL_R_J2", "ELBOW_PITCH_R_J3", "ELBOW_YAW_R_J4", "WRIST_PITCH_R_J5", "WRIST_YAW_R_J6", "WRIST_ROLL_R_J7",
    # 左腿
    "HIP_PITCH_L_J1", "HIP_ROLL_L_J2", "HIP_YAW_L_J3", "KNEE_PITCH_L_J4", "ANKLE_PITCH_L_J5", "ANKLE_ROLL_L_J6",
    # 右腿
    "HIP_PITCH_R_J1", "HIP_ROLL_R_J2", "HIP_YAW_R_J3", "KNEE_PITCH_R_J4", "ANKLE_PITCH_R_J5", "ANKLE_ROLL_R_J6",
]

# 各个关节在qt界面上的位置
joint_info_position = {
    "HEAD_YAW_J1": (450, 200),
    "HEAD_PITCH_J2": (450, 100),

    "WAIST_YAW": (450, 300),
    "WAIST_ROLL": (450, 400),

    "SHOULDER_PITCH_L_J1": (800, 300),
    "SHOULDER_ROLL_L_J2": (800, 400),
    "ELBOW_PITCH_L_J3": (800, 500),
    "ELBOW_YAW_L_J4": (800, 600),
    "WRIST_PITCH_L_J5": (800, 700),
    "WRIST_YAW_L_J6": (800, 800),
    "WRIST_ROLL_L_J7": (800, 900),

    "SHOULDER_PITCH_R_J1": (50, 300),
    "SHOULDER_ROLL_R_J2": (50, 400),
    "ELBOW_PITCH_R_J3": (50, 500),
    "ELBOW_YAW_R_J4": (50, 600),
    "WRIST_PITCH_R_J5": (50, 700),
    "WRIST_YAW_R_J6": (50, 800),
    "WRIST_ROLL_R_J7": (50, 900),

    "HIP_PITCH_L_J1": (550, 500),
    "HIP_ROLL_L_J2": (550, 600),
    "HIP_YAW_L_J3": (550, 700),
    "KNEE_PITCH_L_J4": (550, 800),
    "ANKLE_PITCH_L_J5": (550, 900),
    "ANKLE_ROLL_L_J6": (550, 1000),

    "HIP_PITCH_R_J1": (300, 500),
    "HIP_ROLL_R_J2": (300, 600),
    "HIP_YAW_R_J3": (300, 700),
    "KNEE_PITCH_R_J4": (300, 800),
    "ANKLE_PITCH_R_J5": (300, 900),
    "ANKLE_ROLL_R_J6": (300, 1000)
}

initial_kp_kd = {
    "HEAD_YAW_J1": (200.0, 10.0),
    "HEAD_PITCH_J2": (200.0, 10.0),

    "WAIST_YAW": (200.0, 10.0),
    "WAIST_ROLL": (200.0, 10.0),

    "SHOULDER_PITCH_L_J1": (200.0, 10.0),
    "SHOULDER_ROLL_L_J2": (200.0, 10.0),
    "ELBOW_PITCH_L_J3": (200.0, 10.0),
    "ELBOW_YAW_L_J4": (200.0, 10.0),
    "WRIST_PITCH_L_J5": (200.0, 10.0),
    "WRIST_YAW_L_J6": (200.0, 10.0),
    "WRIST_ROLL_L_J7": (200.0, 10.0),

    "SHOULDER_PITCH_R_J1": (200.0, 10.0),
    "SHOULDER_ROLL_R_J2": (200.0, 10.0),
    "ELBOW_PITCH_R_J3": (200.0, 10.0),
    "ELBOW_YAW_R_J4": (200.0, 10.0),
    "WRIST_PITCH_R_J5": (200.0, 10.0),
    "WRIST_YAW_R_J6": (200.0, 10.0),
    "WRIST_ROLL_R_J7": (200.0, 10.0),

    "HIP_PITCH_L_J1": (200.0,  10.0),
    "HIP_ROLL_L_J2": (200.0,  10.0),
    "HIP_YAW_L_J3": (200.0,  10.0),
    "KNEE_PITCH_L_J4": (200.0,  10.0),
    "ANKLE_PITCH_L_J5": (200.0,  10.0),
    "ANKLE_ROLL_L_J6": (200.0,  10.0),

    "HIP_PITCH_R_J1": (200.0,  10.0),
    "HIP_ROLL_R_J2": (200.0,  10.0),
    "HIP_YAW_R_J3": (200.0,  10.0),
    "KNEE_PITCH_R_J4": (200.0,  10.0),
    "ANKLE_PITCH_R_J5": (200.0,  10.0),
    "ANKLE_ROLL_R_J6": (200.0,  10.0),
}

# 创建 Qt 界面，显示所有关节的当前状态，并允许用户输入目标位置

import json
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from joint_interfaces.msg import JointCommand
from PyQt5.QtCore import Qt
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QFileDialog, QLabel, QLineEdit, QMessageBox, QPushButton, QWidget


class StepLineEdit(QLineEdit):
    def __init__(self, field_name: str, on_step, parent=None):
        super().__init__(parent)
        self.field_name = field_name
        self.on_step = on_step

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Up:
            self.on_step(+1)
            return
        if event.key() == Qt.Key_Down:
            self.on_step(-1)
            return
        super().keyPressEvent(event)


class JointControlWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Joint Control")
        self.setGeometry(100, 100, 1050, 1200)

        self.save_button = QPushButton("Save", self)
        self.save_button.setGeometry(860, 20, 80, 30)

        self.load_button = QPushButton("Load", self)
        self.load_button.setGeometry(950, 20, 80, 30)

        self.transition_time_label = QLabel("Transition Time (s)", self)
        self.transition_time_label.setGeometry(880, 55, 140, 20)

        self.transition_time_input = QLineEdit(self)
        self.transition_time_input.setGeometry(950, 78, 80, 24)
        self.transition_time_input.setText("10")

        self.transition_countdown_label = QLabel("Countdown: 0.0 s", self)
        self.transition_countdown_label.setGeometry(880, 105, 150, 24)

        self.imu_gravity_label = QLabel("x=0.000\ny=0.000\nz=0.000", self)
        self.imu_gravity_label.setGeometry(20, 20, 220, 60)

        self.joint_name_labels = {}
        self.joint_labels = {}
        self.joint_inputs = {}

        # 某些关节共享同一坐标，做轻微纵向错位避免控件重叠。
        pos_used_count = {}
        for joint in joint_names:
            if joint not in joint_info_position:
                continue

            base_x, base_y = joint_info_position[joint]
            pos_key = (base_x, base_y)
            stacked_idx = pos_used_count.get(pos_key, 0)
            pos_used_count[pos_key] = stacked_idx + 1

            y = base_y + stacked_idx * 98

            name_label = QLabel(joint, self)
            name_label.setGeometry(base_x, y, 260, 20)

            # p/v/t 分别固定在不同 x 坐标，避免正负号导致相互挤压。
            p_label = QLabel("p: 0.000", self)
            p_label.setGeometry(base_x, y + 22, 86, 20)

            v_label = QLabel("v: 0.000", self)
            v_label.setGeometry(base_x + 78, y + 22, 86, 20)

            t_label = QLabel("t: 0.000", self)
            t_label.setGeometry(base_x + 156, y + 22, 92, 20)

            pos_input = StepLineEdit("position", lambda _d: None, self)
            pos_input.setPlaceholderText("position")
            pos_input.setGeometry(base_x, y + 44, 78, 24)

            kp_input = StepLineEdit("kp", lambda _d: None, self)
            kp_input.setPlaceholderText("kp")
            kp_input.setGeometry(base_x + 84, y + 44, 56, 24)

            kd_input = StepLineEdit("kd", lambda _d: None, self)
            kd_input.setPlaceholderText("kd")
            kd_input.setGeometry(base_x + 146, y + 44, 56, 24)

            self.joint_name_labels[joint] = name_label
            self.joint_labels[joint] = {
                "p": p_label,
                "v": v_label,
                "t": t_label,
            }
            self.joint_inputs[joint] = {
                "position": pos_input,
                "kp": kp_input,
                "kd": kd_input,
            }

    def update_imu_gravity(self, gx: float, gy: float, gz: float):
        self.imu_gravity_label.setText(
            f"x={gx:.3f}\ny={gy:.3f}\nz={gz:.3f}"
        )

    def update_transition_countdown(self, remain_sec: float):
        remain_sec = max(remain_sec, 0.0)
        self.transition_countdown_label.setText(f"Countdown: {remain_sec:.1f} s")


class JointStateReceiver(Node):
    def __init__(self, widget: JointControlWidget):
        super().__init__('joint_control_receiver')
        self.widget = widget
        self.joint_cmd_pub = self.create_publisher(JointCommand, '/hardware/joint_command', 10)
        self.current_position_by_name = {}
        self.current_velocity_by_name = {}
        self.current_torque_by_name = {}
        self.initialized_from_first_state = False
        self.send_timer = None
        self.transition_start_values = None
        self.transition_target_values = None
        self.transition_start_time_sec = 0.0
        self.transition_duration_sec = 0.0
        self.command_values = {
            joint: {
                'position': 0.0,
                'kp': initial_kp_kd.get(joint, (0.0, 0.0))[0],
                'kd': initial_kp_kd.get(joint, (0.0, 0.0))[1],
            }
            for joint in joint_names
        }

        self.widget.save_button.clicked.connect(self.save_command_values)
        self.widget.load_button.clicked.connect(self.load_command_values)

        self.bind_input_events()

        self.sub = self.create_subscription(
            JointState,
            '/hardware/joint_state',
            self.joint_state_callback,
            10,
        )
        self.imu_sub = self.create_subscription(
            Imu,
            '/hardware/imu_info',
            self.imu_callback,
            10,
        )

    def bind_input_events(self):
        for joint, fields in self.widget.joint_inputs.items():
            for field_name, input_widget in fields.items():
                input_widget.on_step = (
                    lambda direction, j=joint, f=field_name: self.step_input_value(j, f, direction)
                )
                input_widget.returnPressed.connect(
                    lambda j=joint, f=field_name: self.confirm_input_value(j, f)
                )

    def format_value_for_display(self, field_name: str, value: float) -> str:
        if field_name == 'position':
            return f"{value:.3f}"
        return f"{int(round(value))}"

    def torque_to_color(self, torque: float) -> str:
        # 使用扭矩绝对值做映射：0 为绿色，40 及以上为红色。
        ratio = min(max(abs(torque) / 40.0, 0.0), 1.0)
        red = int(255 * ratio)
        green = int(255 * (1.0 - ratio))
        return f"rgb({red}, {green}, 0)"

    def confirm_input_value(self, joint: str, field_name: str):
        input_widget = self.widget.joint_inputs[joint][field_name]
        text = input_widget.text().strip()

        if not text:
            input_widget.setText(
                self.format_value_for_display(field_name, self.command_values[joint][field_name])
            )
            return

        try:
            parsed_value = float(text)
        except ValueError:
            input_widget.setText(
                self.format_value_for_display(field_name, self.command_values[joint][field_name])
            )
            return

        self.command_values[joint][field_name] = parsed_value
        input_widget.setText(self.format_value_for_display(field_name, parsed_value))

    def step_input_value(self, joint: str, field_name: str, direction: int):
        step = 0.001 if field_name == 'position' else 1.0
        current = self.command_values[joint][field_name]
        updated = current + direction * step
        self.command_values[joint][field_name] = updated

        input_widget = self.widget.joint_inputs[joint][field_name]
        input_widget.setText(self.format_value_for_display(field_name, updated))

    def joint_state_callback(self, msg: JointState):
        state_map = dict(zip(msg.name, msg.position))
        self.current_position_by_name.update(state_map)

        for i, name in enumerate(msg.name):
            vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
            torque = msg.effort[i] if i < len(msg.effort) else 0.0
            self.current_velocity_by_name[name] = vel
            self.current_torque_by_name[name] = torque

        for i, name in enumerate(msg.name):
            if name in self.widget.joint_labels:
                pos = msg.position[i] if i < len(msg.position) else 0.0
                vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
                torque = msg.effort[i] if i < len(msg.effort) else 0.0
                torque_color = self.torque_to_color(torque)
                labels = self.widget.joint_labels[name]
                labels["p"].setText(f"p: {pos:.3f}")
                labels["v"].setText(f"v: {vel:.3f}")
                labels["t"].setText(f"t: {torque:.3f}")
                labels["t"].setStyleSheet(f"color: {torque_color};")

        if not self.initialized_from_first_state:
            self.initialize_inputs_from_first_state()
            self.initialized_from_first_state = True
            self.send_timer = self.create_timer(0.01, self.publish_joint_command)
            self.get_logger().info('收到第一帧 /hardware/joint_state，已初始化输入框并启动10ms发送。')

    def imu_callback(self, msg: Imu):
        gx = msg.linear_acceleration.x
        gy = msg.linear_acceleration.y
        gz = msg.linear_acceleration.z
        self.widget.update_imu_gravity(gx, gy, gz)

    def initialize_inputs_from_first_state(self):
        for joint in joint_names:
            if joint not in self.widget.joint_inputs:
                continue

            position = self.current_position_by_name.get(joint, 0.0)
            kp, kd = initial_kp_kd.get(joint, (0.0, 0.0))
            self.command_values[joint]['position'] = position
            self.command_values[joint]['kp'] = kp
            self.command_values[joint]['kd'] = kd

            inputs = self.widget.joint_inputs[joint]
            inputs['position'].setText(f"{position:.3f}")
            inputs['kp'].setText(f"{int(round(kp))}")
            inputs['kd'].setText(f"{int(round(kd))}")

    def refresh_inputs_from_command_values(self):
        for joint, fields in self.widget.joint_inputs.items():
            joint_values = self.command_values.get(joint)
            if joint_values is None:
                continue

            for field_name, input_widget in fields.items():
                value = joint_values.get(field_name, 0.0)
                input_widget.setText(self.format_value_for_display(field_name, float(value)))

    def parse_transition_duration(self) -> float:
        text = self.widget.transition_time_input.text().strip()
        if not text:
            self.widget.transition_time_input.setText("10")
            return 10.0

        try:
            duration = float(text)
        except ValueError:
            QMessageBox.warning(self.widget, "Invalid Transition Time", "Transition time must be a number.")
            self.widget.transition_time_input.setText("10")
            return 10.0

        if duration < 0.0:
            QMessageBox.warning(self.widget, "Invalid Transition Time", "Transition time must be >= 0.")
            self.widget.transition_time_input.setText("10")
            return 10.0

        self.widget.transition_time_input.setText(f"{duration:g}")
        return duration

    def clone_command_values(self, source_values):
        return {
            joint: {
                'position': float(fields['position']),
                'kp': float(fields['kp']),
                'kd': float(fields['kd']),
            }
            for joint, fields in source_values.items()
        }

    def update_command_values_from_transition(self):
        if self.transition_start_values is None or self.transition_target_values is None:
            self.widget.update_transition_countdown(0.0)
            return

        if self.transition_duration_sec <= 0.0:
            self.command_values = self.clone_command_values(self.transition_target_values)
            self.transition_start_values = None
            self.transition_target_values = None
            self.widget.update_transition_countdown(0.0)
            self.refresh_inputs_from_command_values()
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        elapsed_sec = now_sec - self.transition_start_time_sec
        progress = elapsed_sec / self.transition_duration_sec
        progress = min(max(progress, 0.0), 1.0)
        remain_sec = self.transition_duration_sec - elapsed_sec
        self.widget.update_transition_countdown(remain_sec)

        for joint in joint_names:
            if joint not in self.command_values:
                continue

            start_fields = self.transition_start_values.get(joint)
            target_fields = self.transition_target_values.get(joint)
            if start_fields is None or target_fields is None:
                continue

            for field_name in ('position', 'kp', 'kd'):
                start_value = start_fields[field_name]
                target_value = target_fields[field_name]
                self.command_values[joint][field_name] = start_value + (target_value - start_value) * progress

        if progress >= 1.0:
            self.command_values = self.clone_command_values(self.transition_target_values)
            self.transition_start_values = None
            self.transition_target_values = None
            self.widget.update_transition_countdown(0.0)

        self.refresh_inputs_from_command_values()

    def save_command_values(self):
        file_path, _ = QFileDialog.getSaveFileName(
            self.widget,
            "Save Joint Command Values",
            "joint_command_values.json",
            "JSON Files (*.json);;All Files (*)",
        )

        if not file_path:
            return

        try:
            with open(file_path, 'w', encoding='utf-8') as f:
                json.dump(self.command_values, f, ensure_ascii=False, indent=2)
        except OSError as e:
            QMessageBox.critical(self.widget, "Save Failed", f"Failed to save file:\n{e}")
            self.get_logger().error(f"保存失败: {e}")
            return

        QMessageBox.information(self.widget, "Save Success", f"Saved to:\n{file_path}")

    def load_command_values(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self.widget,
            "Load Joint Command Values",
            "",
            "JSON Files (*.json);;All Files (*)",
        )

        if not file_path:
            return

        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                loaded = json.load(f)
        except (OSError, json.JSONDecodeError) as e:
            QMessageBox.critical(self.widget, "Load Failed", f"Failed to load file:\n{e}")
            self.get_logger().error(f"加载失败: {e}")
            return

        if not isinstance(loaded, dict):
            QMessageBox.warning(self.widget, "Invalid File", "JSON root must be an object.")
            return

        transition_duration = self.parse_transition_duration()
        start_values = self.clone_command_values(self.command_values)
        target_values = self.clone_command_values(self.command_values)

        updated_count = 0
        for joint, fields in loaded.items():
            if joint not in target_values or not isinstance(fields, dict):
                continue

            for field_name in ('position', 'kp', 'kd'):
                if field_name not in fields:
                    continue
                try:
                    target_values[joint][field_name] = float(fields[field_name])
                except (TypeError, ValueError):
                    continue
            updated_count += 1

        if updated_count == 0:
            QMessageBox.information(self.widget, "Load Success", f"Loaded from:\n{file_path}\nUpdated joints: 0")
            return

        if transition_duration <= 0.0:
            self.command_values = target_values
            self.transition_start_values = None
            self.transition_target_values = None
            self.widget.update_transition_countdown(0.0)
            self.refresh_inputs_from_command_values()
        else:
            self.transition_start_values = start_values
            self.transition_target_values = target_values
            self.transition_duration_sec = transition_duration
            self.transition_start_time_sec = self.get_clock().now().nanoseconds * 1e-9
            self.widget.update_transition_countdown(transition_duration)

        QMessageBox.information(
            self.widget,
            "Load Success",
            f"Loaded from:\n{file_path}\nUpdated joints: {updated_count}\nTransition: {transition_duration:g} s",
        )

    def publish_joint_command(self):
        self.update_command_values_from_transition()

        msg = JointCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.torque = []
        msg.torque_feedforward = []
        msg.stiffness = []
        msg.damping = []

        for joint in joint_names:
            if joint not in self.widget.joint_inputs:
                continue

            position = self.command_values[joint]['position']
            kp = self.command_values[joint]['kp']
            kd = self.command_values[joint]['kd']

            msg.name.append(joint)
            msg.position.append(position)
            msg.velocity.append(0.0)
            msg.torque.append(0.0)
            msg.torque_feedforward.append(0.0)
            msg.stiffness.append(kp)
            msg.damping.append(kd)

        if msg.name:
            self.joint_cmd_pub.publish(msg)


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    widget = JointControlWidget()
    ros_node = JointStateReceiver(widget)

    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.0))
    ros_timer.start(10)

    widget.show()
    exit_code = app.exec_()

    ros_timer.stop()
    if ros_node.send_timer is not None:
        ros_node.destroy_timer(ros_node.send_timer)
        ros_node.send_timer = None
    ros_node.destroy_node()
    rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
