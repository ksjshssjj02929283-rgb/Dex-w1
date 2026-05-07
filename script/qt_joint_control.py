# 用法:
#   python qt_joint_control.py              # 物理机模式
#   python qt_joint_control.py --sim        # 仿真模式
#   python qt_joint_control.py --step       # Step 模式
#   python qt_joint_control.py --sim --step # 仿真 + Step 模式
#
# 操作:
#   上/下键    : 调整位置/速度值
#   左/右键    : 快速调整
#   回车       : 确认下发 (velocity 列)
#   点击"同步反馈数据到控制表": 从反馈同步到控制表

import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QTableWidget, QTableWidgetItem,
    QVBoxLayout, QHBoxLayout, QPushButton, QLabel
)
from PyQt5.QtCore import QTimer
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from joint_interfaces.msg import JointVelocityTorqueLimitFeedback, JointPositionControl
from PyQt5.QtCore import Qt
import time  # 新增
import argparse  # 新增

class CustomTableWidget(QTableWidget):
    def __init__(self, send_data, parent=None):
        super().__init__(parent)
        self.send_msg = send_data
        self.current_cell = None

    def keyPressEvent(self, event):
        current_item = self.currentItem()
        if current_item:
            try:
                current_value = float(current_item.text())
                model = self.model()
                row = current_item.row()
                col = current_item.column()

                # 仅 velocity 列允许回车提交，position 列回车忽略
                if event.key() in (Qt.Key_Enter, Qt.Key_Return):
                    if col == 2:  # target_velocity
                        current_value = max(0.01, min(0.2, current_value))  # 手动输入回车也限幅
                        current_item.setText(f"{current_value:.2f}")
                        self.send_msg(row, col)
                    return

                delta_map = {
                    Qt.Key_Up:    +0.01,
                    Qt.Key_Right: +0.05,
                    Qt.Key_Down:  -0.01,
                    Qt.Key_Left:  -0.05,
                }
                key = event.key()
                if key in delta_map:
                    current_value += delta_map[key]
                    if col == 1:  # target_position 限幅
                        current_value = max(-3.14, min(3.14, current_value))
                    if col == 2:  # target_velocity 限幅
                        current_value = max(0.01, min(0.2, current_value))
                    current_item.setText(f"{current_value:.2f}")
                    self.send_msg(row, col)
                    index = model.index(row, col)
                    model.dataChanged.emit(index, index)
                    return
            except ValueError:
                pass
        super().keyPressEvent(event)

class JointControlWidget(QWidget):
    def __init__(self, ros_node, step_mode=False):
        super().__init__()
        self.ros_node = ros_node
        self.step_mode = step_mode
        self.setWindowState(Qt.WindowMaximized)

        self.feedback_table = QTableWidget()
        self.control_table = CustomTableWidget(self.send_control_data)

        self.sync_button = QPushButton("同步反馈数据到控制表")
        self.sync_button.clicked.connect(self.sync_from_feedback)

        self.init_tables()

        # 安装事件过滤器，点击空白处取消焦点
        self.control_table.installEventFilter(self)

        layout = QHBoxLayout()
        left = QVBoxLayout()
        right = QVBoxLayout()

        left.addWidget(QLabel("反馈数据"))
        left.addWidget(self.feedback_table)

        right.addWidget(QLabel("控制数据"))
        right.addWidget(self.control_table)
        right.addWidget(self.sync_button)

        layout.addLayout(left)
        layout.addLayout(right)
        self.setLayout(layout)
        self.setWindowTitle("Joint Control Interface")
        self.control_table.itemSelectionChanged.connect(self.onSelectionChanged)

        self.change_pos=3

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_feedback_table)
        self.timer.start(500)

        # 滚动条同步滚动
        self.feedback_vbar = self.feedback_table.verticalScrollBar()
        self.control_vbar = self.control_table.verticalScrollBar()
        self.feedback_vbar.valueChanged.connect(self.on_feedback_scroll)
        self.control_vbar.valueChanged.connect(self.on_control_scroll)
        self._syncing = False

    def init_tables(self):
        headers_feedback = ["name", "position", "velocity", "torque",
                            "pos_torque_limit", "neg_torque_limit"]
        headers_control = ["name", "target_position", "target_velocity"]  # 新增列
        self.feedback_table.setColumnCount(len(headers_feedback))
        self.feedback_table.setHorizontalHeaderLabels(headers_feedback)
        self.control_table.setColumnCount(len(headers_control))
        self.control_table.setHorizontalHeaderLabels(headers_control)

    def on_feedback_scroll(self, value):
        if self._syncing:
            return
        self._syncing = True
        self.control_vbar.setValue(value)
        self._syncing = False

    def on_control_scroll(self, value):
        if self._syncing:
            return
        self._syncing = True
        self.feedback_vbar.setValue(value)
        self._syncing = False

    def onSelectionChanged(self):
        # 获取当前选中的单元格
        selected_items = self.control_table.selectedItems()
        if selected_items:
            current_item = selected_items[0]
            current_item.setBackground(Qt.yellow)
            #print(f"onSelectionChanged:{current_item}\r\n")

            # 清除其他单元格的焦点样式
            for row in range(self.control_table.rowCount()):
                for col in range(self.control_table.columnCount()):
                    item = self.control_table.item(row, col)
                    if item and item != current_item:
                        item.setBackground(Qt.white)
        else:
            # 没有选中项时，清除所有焦点样式
            for row in range(self.control_table.rowCount()):
                for col in range(self.control_table.columnCount()):
                    item = self.control_table.item(row, col)
                    if item:
                        item.setBackground(Qt.white)

    def eventFilter(self, obj, event):
        # 点击 control_table 空白处取消焦点
        if obj == self.control_table and event.type() == event.MouseButtonPress:
            item = self.control_table.itemAt(event.pos())
            if item is None:
                self.control_table.clearSelection()
                self.onSelectionChanged()
        return super().eventFilter(obj, event)

    def update_feedback_table(self):
        joint_data = self.ros_node.joint_states
        self.feedback_table.setRowCount(len(joint_data))
        for row, joint in enumerate(joint_data):
            name = joint['name']
            limits = self.ros_node.velocity_torque_limits.get(name, {})
            values = [
                name,
                f"{joint.get('position', 0.0):.2f}",
                f"{joint.get('velocity', 0.0):.2f}",
                f"{joint.get('torque', 0.0):.2f}",
                f"{limits.get('pos_torque_limit', 0.0):.2f}",
                f"{limits.get('neg_torque_limit', 0.0):.2f}",
            ]
            for col, val in enumerate(values):
                self.feedback_table.setItem(row, col, QTableWidgetItem(val))
        self.feedback_table.resizeColumnsToContents()
        if self.change_pos > 0:
            self.change_pos -= 1
            self.sync_from_feedback()

    def sync_from_feedback(self):
        rows = self.feedback_table.rowCount()
        self.control_table.setRowCount(rows)
        for row in range(rows):
            values = [
                self.feedback_table.item(row, 0).text(),  # name
                self.feedback_table.item(row, 1).text(),  # 初始 target_position
                f"{0.10:.2f}",                            # 默认 target_velocity = 0.1
            ]
            for col, val in enumerate(values):
                item = QTableWidgetItem(val)
                # name 与 target_position 禁止手动编辑
                if col in (0, 1):
                    item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                self.control_table.setItem(row, col, item)
        self.control_table.resizeColumnsToContents()

    def send_control_data(self, changed_row, changed_col):
        if changed_row is None or changed_col is None:
            return
        if changed_col != 1:
            return
        try:
            name_item = self.control_table.item(changed_row, 0)
            pos_item = self.control_table.item(changed_row, 1)
            if not (name_item and pos_item):
                return
            name = name_item.text()
            target_position = float(pos_item.text())
            self.ros_node.publish_control([{'name': name, 'target_position': target_position}])
        except Exception as e:
            print(f"Error parsing control data row {changed_row}: {e}")

class ROS2Node(Node):
    def __init__(self, update_callback, use_sim=False):
        super().__init__('joint_control_gui')
        self.update_callback = update_callback
        self.joint_states = []
        self.velocity_torque_limits = {}

        suffix = '_sim' if use_sim else ''
        self.create_subscription(JointState, f'/feedback/joint{suffix}', self.joint_state_callback, 10)
        self.create_subscription(JointVelocityTorqueLimitFeedback, f'/feedback/joint_velocity_torque_limit{suffix}', self.limit_feedback_callback, 10)
        self.position_control_pub = self.create_publisher(JointPositionControl, f'/control/joint_position{suffix}', 10)

    def joint_state_callback(self, msg):
        self.joint_states = []
        for i, name in enumerate(msg.name):
            self.joint_states.append({
                'name': name,
                'position': msg.position[i] if i < len(msg.position) else 0.0,
                'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                'torque': msg.effort[i] if i < len(msg.effort) else 0.0,
            })
        self.update_callback()

    def limit_feedback_callback(self, msg):
        for i, name in enumerate(msg.name):
            self.velocity_torque_limits[name] = {
                'pos_vel_limit': msg.positive_velocity_limit[i],
                'neg_vel_limit': msg.negative_velocity_limit[i],
                'pos_torque_limit': msg.positive_torque_limit[i],
                'neg_torque_limit': msg.negative_torque_limit[i],
            }
        self.update_callback()

    def publish_control(self, control_data):
        msg = JointPositionControl()
        msg.name = []
        msg.position = []
        for item in control_data:
            name = item['name']
            msg.name.append(name)
            msg.position.append(item['target_position'])
        self.position_control_pub.publish(msg)

def main():
    rclpy.init()
    app = QApplication(sys.argv)

    parser = argparse.ArgumentParser()
    parser.add_argument('--sim', '-s', action='store_true', help='Use simulation topics')
    parser.add_argument('--step', action='store_true', help='Step mode: directly publish target position without interpolation')
    args = parser.parse_args()

    use_sim = args.sim
    step_mode = args.step

    node_holder = {}

    def update_callback():
        if 'widget' in node_holder:
            node_holder['widget'].update_feedback_table()

    ros_node = ROS2Node(update_callback, use_sim=use_sim)
    node_holder['ros_node'] = ros_node

    widget = JointControlWidget(ros_node, step_mode=step_mode)
    node_holder['widget'] = widget
    widget.setWindowTitle(f"Joint Control Interface{' [SIM]' if use_sim else ''}{' [STEP]' if step_mode else ''}")
    widget.show()

    qt_ros_timer = QTimer()
    qt_ros_timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.01))
    qt_ros_timer.start(10)

    sys.exit(app.exec_())

if __name__ == '__main__':
    main()