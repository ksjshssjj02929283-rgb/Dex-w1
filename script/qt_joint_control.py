# 用法:
#   python qt_joint_control.py              # 物理机模式
#   python qt_joint_control.py --sim        # 仿真模式
#   python qt_joint_control.py --step       # Step 模式 (直接下发, 无插值)
#   python qt_joint_control.py --sim --step # 仿真 + Step 模式
#
# 操作:
#   上/下键    : 调整位置/速度值 ±0.01
#   左/右键    : 快速调整 ±0.05
#   W/S 键     : 精细调整 ±0.001
#   回车       : 确认下发 (velocity 列)
#   点击"同步反馈数据到控制表": 从反馈同步到控制表

import sys
import argparse
from PyQt5.QtWidgets import (
    QApplication, QWidget, QTableWidget, QTableWidgetItem,
    QVBoxLayout, QHBoxLayout, QPushButton, QLabel
)
from PyQt5.QtGui import QColor
from PyQt5.QtCore import QTimer, Qt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from joint_interfaces.msg import JointVelocityTorqueLimitFeedback, JointPositionControl

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

                if event.key() in (Qt.Key_Enter, Qt.Key_Return):
                    if col == 2:
                        clamped = current_value
                        current_value = max(0.01, min(0.2, current_value))
                        if clamped != current_value:
                            current_item.setBackground(QColor(255, 200, 200))
                        current_item.setText(f"{current_value:.3f}")
                        self.send_msg(row, col)
                    return

                delta_map = {
                    Qt.Key_Up:    +0.01,
                    Qt.Key_Right: +0.05,
                    Qt.Key_Down:  -0.01,
                    Qt.Key_Left:  -0.05,
                    Qt.Key_W:     +0.001,
                    Qt.Key_S:     -0.001,
                }
                key = event.key()
                if key in delta_map:
                    current_value += delta_map[key]
                    if col == 1:
                        current_value = max(-6.283, min(6.283, current_value))
                    if col == 2:
                        clamped = current_value
                        current_value = max(0.01, min(0.2, current_value))
                        if clamped != current_value:
                            current_item.setBackground(QColor(255, 200, 200))
                    current_item.setText(f"{current_value:.3f}")
                    self.send_msg(row, col)
                    index = model.index(row, col)
                    model.dataChanged.emit(index, index)
                    return
            except ValueError:
                pass
        super().keyPressEvent(event)

class JointControlWidget(QWidget):
    HOLD_FRAMES_AFTER_FINISH = 5
    MOTION_DT = 0.01
    MOTION_THRESHOLD = 0.001

    def __init__(self, ros_node, step_mode=False):
        super().__init__()
        self.ros_node = ros_node
        self.step_mode = step_mode
        self.setWindowState(Qt.WindowMaximized)

        self.feedback_table = QTableWidget()
        self.control_table = CustomTableWidget(self.send_control_data)
        self.control_table.itemChanged.connect(self.onControlItemChanged)

        self.sync_button = QPushButton("同步反馈数据到控制表")
        self.sync_button.clicked.connect(self.sync_from_feedback)

        self.init_tables()

        self.control_table.setEnabled(False)
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
        self.control_table.itemSelectionChanged.connect(self.onSelectionChanged)

        self.change_pos = 3
        self._initial_syncing = True

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_feedback_table)
        self.timer.start(500)

        self.feedback_vbar = self.feedback_table.verticalScrollBar()
        self.control_vbar = self.control_table.verticalScrollBar()
        self.feedback_vbar.valueChanged.connect(self.on_feedback_scroll)
        self.control_vbar.valueChanged.connect(self.on_control_scroll)
        self._scroll_syncing = False

        self.motion_plans = {}  # { name: {target_pos, velocity} }
        self._cmd_positions = {}  # { name: last_commanded_pos }
        if not self.step_mode:
            self.motion_timer = QTimer()
            self.motion_timer.timeout.connect(self.update_motion)
            self.motion_timer.start(10)

    def init_tables(self):
        headers_feedback = ["name", "position", "velocity", "torque",
                            "pos_torque_limit", "neg_torque_limit"]
        headers_control = ["name", "target_position", "target_velocity"]
        self.feedback_table.setColumnCount(len(headers_feedback))
        self.feedback_table.setHorizontalHeaderLabels(headers_feedback)
        self.control_table.setColumnCount(len(headers_control))
        self.control_table.setHorizontalHeaderLabels(headers_control)

    def on_feedback_scroll(self, value):
        if self._scroll_syncing:
            return
        self._scroll_syncing = True
        self.control_vbar.setValue(value)
        self._scroll_syncing = False

    def on_control_scroll(self, value):
        if self._scroll_syncing:
            return
        self._scroll_syncing = True
        self.feedback_vbar.setValue(value)
        self._scroll_syncing = False

    def onSelectionChanged(self):
        selected_items = self.control_table.selectedItems()
        if selected_items:
            current_item = selected_items[0]
            current_item.setBackground(Qt.yellow)
            for row in range(self.control_table.rowCount()):
                for col in range(self.control_table.columnCount()):
                    item = self.control_table.item(row, col)
                    if item and item != current_item:
                        item.setBackground(Qt.white)
        else:
            for row in range(self.control_table.rowCount()):
                for col in range(self.control_table.columnCount()):
                    item = self.control_table.item(row, col)
                    if item:
                        item.setBackground(Qt.white)

    def eventFilter(self, obj, event):
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
                f"{joint.get('position', 0.0):.3f}",
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
        else:
            if self._initial_syncing:
                self._initial_syncing = False
                self.control_table.setEnabled(True)

    def sync_from_feedback(self):
        self.motion_plans.clear()
        self._cmd_positions.clear()
        rows = self.feedback_table.rowCount()
        self.control_table.setRowCount(rows)
        for row in range(rows):
            values = [
                self.feedback_table.item(row, 0).text(),
                self.feedback_table.item(row, 1).text(),
                f"{0.10:.3f}",
            ]
            for col, val in enumerate(values):
                item = QTableWidgetItem(val)
                if col in (0, 1):
                    item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                self.control_table.setItem(row, col, item)
        self.control_table.resizeColumnsToContents()

    def send_control_data(self, changed_row, changed_col):
        if changed_row is None or changed_col is None:
            return
        try:
            name_item = self.control_table.item(changed_row, 0)
            pos_item = self.control_table.item(changed_row, 1)
            vel_item = self.control_table.item(changed_row, 2)
            if not (name_item and pos_item and vel_item):
                return
            name = name_item.text()
            target_position = float(pos_item.text())
            target_velocity = float(vel_item.text())
            target_velocity = max(0.01, min(0.2, target_velocity))

            if self.step_mode:
                if changed_col == 1:
                    self.ros_node.publish_control([{'name': name, 'target_position': target_position}])
                return

            if changed_col == 2:
                if name in self.motion_plans:
                    self.motion_plans[name]['velocity'] = target_velocity
                return

            if changed_col == 1:
                if target_velocity <= 0.0:
                    self.motion_plans.pop(name, None)
                    self.ros_node.publish_control([{'name': name, 'target_position': target_position}])
                    return
                self.start_motion_plan(name, target_position, target_velocity)
        except Exception as e:
            print(f"Error parsing control data row {changed_row}: {e}")

    def get_current_feedback_position(self, name):
        for js in self.ros_node.joint_states:
            if js['name'] == name:
                return js.get('position', 0.0)
        return 0.0

    def start_motion_plan(self, name, target_pos, velocity):
        cmd_pos = self._cmd_positions.get(name)
        if cmd_pos is None:
            cmd_pos = self.get_current_feedback_position(name)
        if abs(target_pos - cmd_pos) < self.MOTION_THRESHOLD:
            self.motion_plans.pop(name, None)
            self._cmd_positions[name] = target_pos
            self.ros_node.publish_control([{'name': name, 'target_position': target_pos}])
            return
        self._cmd_positions[name] = cmd_pos
        self.motion_plans[name] = {
            'target_pos': target_pos,
            'velocity': velocity,
        }

    def update_motion(self):
        if not self.motion_plans:
            return
        publish_items = []
        finished = []
        for name, plan in list(self.motion_plans.items()):
            target_pos = plan['target_pos']
            velocity = plan['velocity']
            cmd_pos = self._cmd_positions.get(name, target_pos)
            remaining = target_pos - cmd_pos

            if abs(remaining) <= self.MOTION_THRESHOLD:
                new_pos = target_pos
                hold_count = plan.get('hold_count', 0) + 1
                if hold_count >= self.HOLD_FRAMES_AFTER_FINISH:
                    finished.append(name)
                else:
                    plan['hold_count'] = hold_count
            else:
                step = velocity * self.MOTION_DT
                if abs(remaining) <= step:
                    new_pos = target_pos
                else:
                    new_pos = cmd_pos + (step if remaining > 0 else -step)

            self._cmd_positions[name] = new_pos
            publish_items.append({'name': name, 'target_position': new_pos})

        if publish_items:
            self.ros_node.publish_control(publish_items)
        for name in finished:
            self.motion_plans.pop(name, None)

    def onControlItemChanged(self, item):
        if item and item.column() == 2:
            try:
                v = float(item.text())
            except ValueError:
                v = 0.10
            clamped = v
            v = max(0.01, min(0.2, v))
            if clamped != v:
                item.setBackground(QColor(255, 200, 200))
            new_text = f"{v:.3f}"
            if item.text() != new_text:
                self.control_table.blockSignals(True)
                item.setText(new_text)
                self.control_table.blockSignals(False)

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
            msg.name.append(item['name'])
            msg.position.append(item['target_position'])
        self.position_control_pub.publish(msg)

def main():
    rclpy.init()
    app = QApplication(sys.argv)

    parser = argparse.ArgumentParser()
    parser.add_argument('--sim', '-s', action='store_true', help='Use simulation topics')
    parser.add_argument('--step', action='store_true', help='Step mode: directly publish target position without interpolation')
    args = parser.parse_args()

    node_holder = {}

    def update_callback():
        if 'widget' in node_holder:
            node_holder['widget'].update_feedback_table()

    ros_node = ROS2Node(update_callback, use_sim=args.sim)
    node_holder['ros_node'] = ros_node

    widget = JointControlWidget(ros_node, step_mode=args.step)
    node_holder['widget'] = widget
    widget.setWindowTitle(f"Joint Control Interface{' [SIM]' if args.sim else ''}{' [STEP]' if args.step else ''}")
    widget.show()

    qt_ros_timer = QTimer()
    qt_ros_timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.01))
    qt_ros_timer.start(10)

    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
