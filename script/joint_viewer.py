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

# 创建 Qt 界面，显示所有关节的当前状态

import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QLabel, QWidget


class JointViewerWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Joint Viewer")
        self.setGeometry(100, 100, 1050, 1200)

        self.imu_gravity_label = QLabel("x=0.000\ny=0.000\nz=0.000", self)
        self.imu_gravity_label.setGeometry(20, 20, 220, 60)

        self.joint_name_labels = {}
        self.joint_labels = {}

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

            self.joint_name_labels[joint] = name_label
            self.joint_labels[joint] = {
                "p": p_label,
                "v": v_label,
                "t": t_label,
            }

    def update_imu_gravity(self, gx: float, gy: float, gz: float):
        self.imu_gravity_label.setText(
            f"x={gx:.3f}\ny={gy:.3f}\nz={gz:.3f}"
        )


class JointStateReceiver(Node):
    def __init__(self, widget: JointViewerWidget):
        super().__init__('joint_viewer_receiver')
        self.widget = widget

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

    def torque_to_color(self, torque: float) -> str:
        # 使用扭矩绝对值做映射：0 为绿色，40 及以上为红色。
        ratio = min(max(abs(torque) / 40.0, 0.0), 1.0)
        red = int(255 * ratio)
        green = int(255 * (1.0 - ratio))
        return f"rgb({red}, {green}, 0)"

    def joint_state_callback(self, msg: JointState):
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

    def imu_callback(self, msg: Imu):
        gx = msg.linear_acceleration.x
        gy = msg.linear_acceleration.y
        gz = msg.linear_acceleration.z
        self.widget.update_imu_gravity(gx, gy, gz)


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    widget = JointViewerWidget()
    ros_node = JointStateReceiver(widget)

    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.0))
    ros_timer.start(10)

    widget.show()
    exit_code = app.exec_()

    ros_timer.stop()
    ros_node.destroy_node()
    rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
