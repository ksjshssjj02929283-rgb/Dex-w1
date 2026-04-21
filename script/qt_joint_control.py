import sys
import json
from PyQt5.QtWidgets import (
    QApplication, QWidget, QTableWidget, QTableWidgetItem,
    QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QScrollArea, QLineEdit, QStyledItemDelegate
)
from PyQt5.QtCore import QTimer, Qt, QEvent
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from joint_interfaces.msg import JointCommand


class CommitDelegate(QStyledItemDelegate):
    def eventFilter(self, obj, event):
        if event.type() == QEvent.KeyPress and event.key() in (Qt.Key_Enter, Qt.Key_Return):
            self.commitData.emit(obj)
            return True
        return super().eventFilter(obj, event)


class CustomTableWidget(QTableWidget):
    def __init__(self, send_data, parent=None):
        super().__init__(parent)
        self.send_msg = send_data
        self.current_cell = None
        delegate = CommitDelegate(self)
        self.setItemDelegate(delegate)
        self.itemChanged.connect(self._on_item_changed)

    def _on_item_changed(self, item):
        if item.column() in (2, 3) and item.text() != "/":
            row = item.row()
            col = item.column()
            try:
                value = float(item.text())
                if col == 2:
                    value = max(0.0, min(500.0, value))
                    item.setText(f"{value:.2f}")
                elif col == 3:
                    value = max(0.0, min(100.0, value))
                    item.setText(f"{value:.2f}")
                self.send_msg(row, col)
            except ValueError:
                pass

    def keyPressEvent(self, event):
        current_item = self.currentItem()
        if current_item:
            model = self.model()
            row = current_item.row()
            col = current_item.column()

            if event.key() in (Qt.Key_Enter, Qt.Key_Return):
                if col == 1:
                    try:
                        current_value = float(current_item.text())
                        current_value = max(-3.14, min(3.14, current_value))
                        current_item.setText(f"{current_value:.2f}")
                    except ValueError:
                        current_item.setText("0.00")
                    self.send_msg(row, col)
                return

            if col == 1:
                delta_map = {
                    Qt.Key_Up:    +0.01,
                    Qt.Key_Right: +0.1,
                    Qt.Key_Down:  -0.01,
                    Qt.Key_Left:  -0.1,
                }
                key = event.key()
                if key in delta_map:
                    try:
                        current_value = float(current_item.text())
                    except ValueError:
                        current_value = 0.0
                    current_value += delta_map[key]
                    current_value = max(-3.14, min(3.14, current_value))
                    current_item.setText(f"{current_value:.2f}")
                    self.send_msg(row, col)
                    index = model.index(row, col)
                    model.dataChanged.emit(index, index)
                    return

            return
        super().keyPressEvent(event)


class JointControlWidget(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.change_pos = 3
        self.leg_ankle_motor_torque = []

        self.feedback_table = QTableWidget()
        self.control_table = CustomTableWidget(self.send_control_data)

        self.sync_button = QPushButton("同步反馈数据到控制表")
        self.sync_button.clicked.connect(self.sync_from_feedback)

        self.init_tables()

        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)

        tables_layout = QHBoxLayout()
        left = QVBoxLayout()
        right = QVBoxLayout()

        left.addWidget(QLabel("反馈数据"))
        left.addWidget(self.feedback_table)

        right.addWidget(QLabel("控制数据"))
        right.addWidget(self.control_table)
        right.addWidget(self.sync_button)

        tables_layout.addLayout(left)
        tables_layout.addLayout(right)

        container = QWidget()
        container.setLayout(tables_layout)
        self.scroll_area.setWidget(container)

        main_layout = QVBoxLayout()
        main_layout.addWidget(self.scroll_area)
        self.setLayout(main_layout)
        self.setWindowTitle("Joint Control Interface")
        self.showMaximized()

        self.feedback_table.verticalScrollBar().valueChanged.connect(
            self.control_table.verticalScrollBar().setValue)
        self.control_table.verticalScrollBar().valueChanged.connect(
            self.feedback_table.verticalScrollBar().setValue)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_feedback_table)
        self.timer.start(500)

    def init_tables(self):
        self.feedback_table.setColumnCount(7)
        self.feedback_table.setHorizontalHeaderLabels(["name", "position", "velocity", "torque", "stiffness", "damping", "ankle_motor_torque"])
        self.control_table.setColumnCount(4)
        self.control_table.setHorizontalHeaderLabels(["name", "target_position", "stiffness", "damping"])

    def update_feedback_table(self):
        joint_data = self.ros_node.joint_states
        self.feedback_table.setRowCount(len(joint_data))
        ndof_body = len(self.ros_node.joint_names) - 12  # Assuming 12 leg joints
        ankle_motor_torques = self.ros_node.leg_ankle_motor_torque
        ankle_mapping = {4: 0, 5: 1, 10: 2, 11: 3}  # leg_idx -> ankle_motor_torque index
        for row, joint in enumerate(joint_data):
            name = joint['name']
            stiffness_val = joint.get('stiffness')
            damping_val = joint.get('damping')
            stiffness_str = f"{stiffness_val:.2f}" if stiffness_val is not None else "/"
            damping_str = f"{damping_val:.2f}" if damping_val is not None else "/"

            leg_idx = row - ndof_body if row >= ndof_body else -1
            if leg_idx in ankle_mapping and leg_idx < len(ankle_motor_torques):
                ankle_idx = ankle_mapping[leg_idx]
                if ankle_idx < len(ankle_motor_torques):
                    ankle_torque_str = f"{ankle_motor_torques[ankle_idx]:.2f}"
                else:
                    ankle_torque_str = "/"
            else:
                ankle_torque_str = "/"

            values = [
                name,
                f"{joint.get('position', 0.0):.2f}",
                f"{joint.get('velocity', 0.0):.2f}",
                f"{joint.get('torque', 0.0):.2f}",
                stiffness_str,
                damping_str,
                ankle_torque_str,
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
            stiffness_val = self.feedback_table.item(row, 4).text()
            damping_val = self.feedback_table.item(row, 5).text()
            if stiffness_val == "/" or damping_val == "/":
                stiffness_str = "/"
                damping_str = "/"
            else:
                stiffness_str = stiffness_val
                damping_str = damping_val
            values = [
                self.feedback_table.item(row, 0).text(),
                self.feedback_table.item(row, 1).text(),
                stiffness_str,
                damping_str,
            ]
            for col, val in enumerate(values):
                item = QTableWidgetItem(val)
                if col == 0 or col == 1 or val == "/":
                    item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                self.control_table.setItem(row, col, item)
        self.control_table.resizeColumnsToContents()

    def send_control_data(self, changed_row, changed_col):
        if changed_row is None or changed_col is None:
            return
        try:
            name_item = self.control_table.item(changed_row, 0)
            pos_item = self.control_table.item(changed_row, 1)
            stiffness_item = self.control_table.item(changed_row, 2)
            damping_item = self.control_table.item(changed_row, 3)
            if not (name_item and pos_item):
                return
            name = name_item.text()
            target_position = float(pos_item.text())
            stiffness = 0.0
            if stiffness_item and stiffness_item.text() != "/":
                stiffness = float(stiffness_item.text())
            damping = 0.0
            if damping_item and damping_item.text() != "/":
                damping = float(damping_item.text())

            self.ros_node.publish_control([{
                'name': name,
                'position': target_position,
                'stiffness': stiffness,
                'damping': damping
            }])
        except Exception as e:
            print(f"Error parsing control data row {changed_row}: {e}")


class ROS2Node(Node):
    def __init__(self, update_callback):
        super().__init__('joint_control_gui')
        self.update_callback = update_callback
        self.joint_states = []
        self.joint_names = []
        self.leg_ankle_motor_torque = []
        self.create_subscription(String, '/feedback/robot_server_state', self.robot_status_callback, 10)
        self.joint_cmd_pub = self.create_publisher(JointCommand, '/hardware/joint_command', 10)

    def robot_status_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.joint_names = data.get('joint_names', [])
            body_pos = data.get('body_joint_position', [])
            body_vel = data.get('body_joint_velocity', [])
            body_tor = data.get('body_joint_torque', [])
            body_stiffness = data.get('body_stiffness', [])
            body_damping = data.get('body_damping', [])
            leg_pos = data.get('leg_joint_position', [])
            leg_vel = data.get('leg_joint_velocity', [])
            leg_tor = data.get('leg_joint_torque', [])
            leg_stiffness = data.get('leg_stiffness', [])
            leg_damping = data.get('leg_damping', [])
            self.leg_ankle_motor_torque = data.get('leg_ankle_motor_torque', [])

            ndof_body = len(body_pos)

            self.joint_states = []
            for i, name in enumerate(self.joint_names):
                if i < ndof_body:
                    stiffness = body_stiffness[i] if i < len(body_stiffness) else None
                    damping = body_damping[i] if i < len(body_damping) else None
                    self.joint_states.append({
                        'name': name,
                        'position': body_pos[i] if i < len(body_pos) else 0.0,
                        'velocity': body_vel[i] if i < len(body_vel) else 0.0,
                        'torque': body_tor[i] if i < len(body_tor) else 0.0,
                        'stiffness': stiffness,
                        'damping': damping,
                    })
                else:
                    leg_idx = i - ndof_body
                    self.joint_states.append({
                        'name': name,
                        'position': leg_pos[leg_idx] if leg_idx < len(leg_pos) else 0.0,
                        'velocity': leg_vel[leg_idx] if leg_idx < len(leg_vel) else 0.0,
                        'torque': leg_tor[leg_idx] if leg_idx < len(leg_tor) else 0.0,
                        'stiffness': leg_stiffness[leg_idx] if leg_idx < len(leg_stiffness) else 100.0,
                        'damping': leg_damping[leg_idx] if leg_idx < len(leg_damping) else 10.0,
                    })
            self.update_callback()
        except Exception as e:
            self.get_logger().warn(f'Failed to parse robot status: {e}')

    def publish_control(self, control_data):
        msg = JointCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'joint_command'
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.torque = []
        msg.torque_feedforward = []
        msg.stiffness = []
        msg.damping = []
        for item in control_data:
            name = item['name']
            msg.name.append(name)
            msg.position.append(item.get('position', 0.0))
            msg.velocity.append(0.0)
            msg.torque.append(0.0)
            msg.torque_feedforward.append(0.0)
            msg.stiffness.append(item.get('stiffness', 0.0))
            msg.damping.append(item.get('damping', 0.0))
        self.joint_cmd_pub.publish(msg)


def main():
    rclpy.init()
    app = QApplication(sys.argv)

    node_holder = {}

    def update_callback():
        if 'widget' in node_holder:
            node_holder['widget'].update_feedback_table()

    ros_node = ROS2Node(update_callback)
    node_holder['ros_node'] = ros_node

    widget = JointControlWidget(ros_node)
    node_holder['widget'] = widget
    widget.show()

    qt_ros_timer = QTimer()
    qt_ros_timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.01))
    qt_ros_timer.start(10)

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()