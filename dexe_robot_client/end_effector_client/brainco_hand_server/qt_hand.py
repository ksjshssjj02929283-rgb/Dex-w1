# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import sys
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QSpinBox, QPushButton, QWidget
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class CustomSpinBox(QSpinBox):
    """继承 QSpinBox 并重写 keyPressEvent 监听上下键"""

    def __init__(self, parent, send_angles_callback):
        super().__init__(parent)
        self.parent_widget = parent  # 存储父类的引用
        self.send_angles_callback = send_angles_callback  # 存储 send_angles 方法的引用

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Enter or  event.key() == Qt.Key_Return:
            print(f"{self.objectName()} enter 按下，当前值 {self.value()}")
            self.send_angles_callback()
        else:
                if event.key() == Qt.Key_Right:  # Qt.Key_Up (↑)
                    print(f"{self.objectName()} ↑ 按下，上调值 {self.value() + 10}")
                    self.setValue(self.value() + 10)
                    self.send_angles_callback()
                elif event.key() == Qt.Key_Left:  # Qt.Key_Down (↓)
                    if self.value() > 10:
                        print(f"{self.objectName()} ↓ 按下，下调值 {self.value() - 10}")
                        self.setValue(self.value() - 10)
                    else:
                        self.setValue(0)
                    self.send_angles_callback()
                if event.key() == Qt.Key_Up:  # Qt.Key_Up (↑)
                    print(f"{self.objectName()} ↑ 按下，上调值 {self.value() + 1}")
                    self.setValue(self.value() + 1)
                    self.send_angles_callback()
                elif event.key() == Qt.Key_Down:  # Qt.Key_Down (↓)
                    if self.value() > 1:
                        print(f"{self.objectName()} ↓ 按下，下调值 {self.value() - 1}")
                        self.setValue(self.value() - 1)
                    else:
                        self.setValue(0)
                    self.send_angles_callback()
                else:
                    super().keyPressEvent(event)  # 调用原本的 QSpinBox 事件



class MotorControlGUI(QMainWindow):
    def __init__(self, ros_node, config):
        super().__init__()
        self.ros_node = ros_node
        self.config = config  # 配置文件中的坐标数据
        self.init_ui()
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_motor_angles)
        self.timer.start(100)  # 每100ms刷新一次界面

    def init_ui(self):
        self.setWindowTitle("BrainCo Hand Control Interface")
        self.setGeometry(100, 100, 1100, 600)

        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)

        # 当前角度和输入框的布局
        self.current_angles = [QLabel("0".ljust(7), central_widget) for _ in range(12)]
        self.spin_boxes = []

        for i in range(12):
            #print(self.config.keys()[i])
            
            # 读取矩形的左上角坐标
            x, y = self.config.get(f"motor_{i + 1}", [10, 10])

            # 显示当前值
            current_label = QLabel(f"Motor {i + 1} Current: ", central_widget)
            current_label.move(x, y)
            current_label_value = self.current_angles[i]
            current_label_value.move(x + 150, y)
            current_label_value.setFixedWidth(100)  # 设置固定宽度，保证 7 位数字有足够空间
            current_label_value.setAlignment(Qt.AlignLeft)
            current_label_value.setParent(central_widget)

            # 显示输入框
            target_label = QLabel("Target: ", central_widget)
            target_label.move(x, y + 30)
            spin_box = CustomSpinBox(central_widget,self.send_angles) # QSpinBox(central_widget)
            spin_box.setRange(0, 100)  # 支持值范围为 0 到 4095
            spin_box.setValue(0)  # 默认值设置为 0
            spin_box.move(x + 150, y + 30)
            self.spin_boxes.append(spin_box)

        # 按钮：同步当前值
        sync_button_coords = self.config.get("sync_button", [10, 700])
        self.sync_button = QPushButton("Sync with Current Values", central_widget)
        self.sync_button.move(*sync_button_coords)
        self.sync_button.clicked.connect(self.sync_with_current_values)

        # 按钮：发送目标角度
        send_button_coords = self.config.get("send_button", [300, 700])
        self.send_button = QPushButton("Send Angles", central_widget)
        self.send_button.move(*send_button_coords)
        self.send_button.clicked.connect(self.send_angles)

    def update_motor_angles(self):
        # 从 ROS2 节点获取电机角度
        left_angles, right_angles = self.ros_node.get_motor_angles()
        if left_angles and right_angles:
            for i in range(6):  # 左手电机
                self.current_angles[i].setText(f"{left_angles[i]:.2f}".ljust(7))
            for i in range(6, 12):  # 右手电机
                self.current_angles[i].setText(f"{right_angles[i-6]:.2f}".ljust(7))  # 显示右手电机数据

    def sync_with_current_values(self):
        # 将当前角度值同步到输入框
        left_angles, right_angles = self.ros_node.get_motor_angles()
        if left_angles and right_angles:
            for i, angle in enumerate(left_angles):
                self.spin_boxes[i].setValue(int(angle))  # 同步左手电机的值到输入框
            for i, angle in enumerate(right_angles):
                self.spin_boxes[i + 6].setValue(int(angle))  # 同步右手电机的值到输入框

    def send_angles(self):
        # 从界面获取角度值并发送到 ROS2
        angles = [float(spin_box.value()) for spin_box in self.spin_boxes]
        print(f"Sending angles: {angles}")  # 调试打印

        # 将角度值分为左手和右手
        left_angles = angles[:6]  # 左手的6个角度
        right_angles = angles[6:]  # 右手的6个角度

        # 发送数据到对应的 ROS 2 话题
        self.ros_node.set_motor_angles('left', left_angles)
        self.ros_node.set_motor_angles('right', right_angles)


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_gui')
        self.left_qpos_subscription = self.create_subscription(
            Float64MultiArray, '/brainco_left_hand_qpos', self.left_qpos_callback, 10)
        self.right_qpos_subscription = self.create_subscription(
            Float64MultiArray, '/brainco_right_hand_qpos', self.right_qpos_callback, 10)
        self.set_qpos_publisher_left = self.create_publisher(Float64MultiArray, '/set_brainco_left_hand_qpos', 10)
        self.set_qpos_publisher_right = self.create_publisher(Float64MultiArray, '/set_brainco_right_hand_qpos', 10)
        self.left_motor_angles = [0.0] * 6  # 初始化左手电机为0
        self.right_motor_angles = [0.0] * 6  # 初始化右手电机为0

    def left_qpos_callback(self, msg):
        self.left_motor_angles = msg.data

    def right_qpos_callback(self, msg):
        self.right_motor_angles = msg.data

    def get_motor_angles(self):
        return self.left_motor_angles, self.right_motor_angles

    def set_motor_angles(self, hand, angles):
        if not isinstance(angles, (list, tuple)) or len(angles) != 6 or not all(isinstance(a, (float, int)) for a in angles):
            self.get_logger().error(f"Invalid angles for {hand}: must be a list or tuple of 6 numbers.")
            return

        msg = Float64MultiArray()
        msg.data = [float(a) for a in angles]  # 确保转换为浮点数

        # 选择发布到哪个话题
        if hand == 'left':
            self.set_qpos_publisher_left.publish(msg)
        elif hand == 'right':
            self.set_qpos_publisher_right.publish(msg)
        else:
            self.get_logger().error(f"Unknown hand: {hand}")
            return


def load_ui_config():
    config =   {
            "motor_1": [100, 10],
            "motor_2": [100, 80],
            "motor_3": [100, 150],
            "motor_4": [100, 220],
            "motor_5": [100, 290],
            "motor_6": [100, 360],

            "motor_7": [500, 10],
            "motor_8": [500, 80],
            "motor_9": [500, 150],
            "motor_10": [500, 220],
            "motor_11": [500, 290],
            "motor_12": [500, 360],

        "sync_button": [300, 500],
        "save_button": [300, 600],
        "send_button": [500, 500]
    }
    return config


def main():
    # 加载配置文件
    config = load_ui_config()

    # 初始化 ROS2 和 Qt 应用
    rclpy.init()
    ros_node = MotorControlNode()

    app = QApplication(sys.argv)
    gui = MotorControlGUI(ros_node, config)
    gui.show()

    # 启动 Qt 主循环和 ROS2 节点
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.1))
    timer.start(50)

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
