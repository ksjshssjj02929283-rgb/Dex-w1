import sys
import json
from PyQt5.QtWidgets import (
    QApplication, QWidget, QTableWidget, QTableWidgetItem,
    QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QScrollArea
)
from PyQt5.QtCore import QTimer
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from joint_interfaces.msg import JointPositionControl
from PyQt5.QtCore import Qt

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

                # 仅回车提交position列
                if event.key() in (Qt.Key_Enter, Qt.Key_Return):
                    if col == 1:  # target_position
                        current_value = max(-3.14, min(3.14, current_value))  # 手动输入回车也限幅
                        current_item.setText(f"{current_value:.2f}")
                        self.send_msg(row, col)
                    return

                delta_map = {
                    Qt.Key_Up:    +0.01,
                    Qt.Key_Right: +0.1,
                    Qt.Key_Down:  -0.01,
                    Qt.Key_Left:  -0.1,
                }
                key = event.key()
                if key in delta_map:
                    current_value += delta_map[key]
                    if col == 1:  # target_position 限幅
                        current_value = max(-3.14, min(3.14, current_value))
                    current_item.setText(f"{current_value:.2f}")
                    self.send_msg(row, col)
                    index = model.index(row, col)
                    model.dataChanged.emit(index, index)
                    return
            except ValueError:
                pass
        super().keyPressEvent(event)

class JointControlWidget(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node

        self.feedback_table = QTableWidget()
        self.control_table = CustomTableWidget(self.send_control_data)
        self.control_table.itemChanged.connect(self.onControlItemChanged)

        self.sync_button = QPushButton("同步反馈数据到控制表")
        self.sync_button.clicked.connect(self.sync_from_feedback)

        self.init_tables()

        # 创建垂直滚动区域，放置两个表格
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

        self.control_table.itemSelectionChanged.connect(self.onSelectionChanged)

        # 同步滚动
        self.feedback_table.verticalScrollBar().valueChanged.connect(
            self.control_table.verticalScrollBar().setValue)
        self.control_table.verticalScrollBar().valueChanged.connect(
            self.feedback_table.verticalScrollBar().setValue)

        self.change_pos=3

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_feedback_table)
        self.timer.start(500)

    def init_tables(self):
        headers_feedback = ["name", "position", "velocity", "torque"]
        headers_control = ["name", "target_position"]
        self.feedback_table.setColumnCount(len(headers_feedback))
        self.feedback_table.setHorizontalHeaderLabels(headers_feedback)
        self.control_table.setColumnCount(len(headers_control))
        self.control_table.setHorizontalHeaderLabels(headers_control)

    def onSelectionChanged(self):
        # 获取当前选中的单元格
        selected_items = self.control_table.selectedItems()
        if selected_items:
            current_item = selected_items[0]
            current_item.setBackground(Qt.green)
            #print(f"onSelectionChanged:{current_item}\r\n")

            # 清除其他单元格的焦点样式
            for row in range(self.control_table.rowCount()):
                for col in range(self.control_table.columnCount()):
                    item = self.control_table.item(row, col)
                    if item and item != current_item:
                        item.setBackground(Qt.white)
        else:
            # 没有选中任何单元格时，清除所有焦点样式
            for row in range(self.control_table.rowCount()):
                for col in range(self.control_table.columnCount()):
                    item = self.control_table.item(row, col)
                    if item:
                        item.setBackground(Qt.white)

    def update_feedback_table(self):
        joint_data = self.ros_node.joint_states
        self.feedback_table.setRowCount(len(joint_data))
        for row, joint in enumerate(joint_data):
            name = joint['name']
            values = [
                name,
                f"{joint.get('position', 0.0):.2f}",
                f"{joint.get('velocity', 0.0):.2f}",
                f"{joint.get('torque', 0.0):.2f}",
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
            ]
            for col, val in enumerate(values):
                item = QTableWidgetItem(val)
                # name 禁止手动编辑
                if col == 0:
                    item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                self.control_table.setItem(row, col, item)
        self.control_table.resizeColumnsToContents()

    def send_control_data(self, changed_row, changed_col):
        if changed_row is None or changed_col is None:
            return
        try:
            name_item = self.control_table.item(changed_row, 0)
            pos_item = self.control_table.item(changed_row, 1)
            if not (name_item and pos_item):
                return
            name = name_item.text()
            target_position = float(pos_item.text())

            # 直接下发位置指令
            self.ros_node.publish_control([{'name': name, 'target_position': target_position}])
        except Exception as e:
            print(f"Error parsing control data row {changed_row}: {e}")

    def onControlItemChanged(self, item):
        # 仅对 position 列 (索引1) 手动输入进行限幅
        if item and item.column() == 1:
            try:
                v = float(item.text())
            except ValueError:
                v = 0.0
            v = max(-3.14, min(3.14, v))
            new_text = f"{v:.2f}"
            if item.text() != new_text:
                item.setText(new_text)

class ROS2Node(Node):
    def __init__(self, update_callback):
        super().__init__('joint_control_gui')
        self.update_callback = update_callback
        self.joint_states = []
        self.joint_names = []
        self.create_subscription(String, '/feedback/robot_server_state', self.robot_status_callback, 10)
        self.position_control_pub = self.create_publisher(JointPositionControl, '/hardware/joint_position_control', 10)

    def robot_status_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.joint_names = data.get('joint_names', [])
            body_pos = data.get('body_joint_position', [])
            body_vel = data.get('body_joint_velocity', [])
            body_tor = data.get('body_joint_torque', [])
            leg_pos = data.get('leg_joint_position', [])
            leg_vel = data.get('leg_joint_velocity', [])
            leg_tor = data.get('leg_joint_torque', [])

            ndof_body = len(body_pos)

            self.joint_states = []
            for i, name in enumerate(self.joint_names):
                if i < ndof_body:
                    self.joint_states.append({
                        'name': name,
                        'position': body_pos[i] if i < len(body_pos) else 0.0,
                        'velocity': body_vel[i] if i < len(body_vel) else 0.0,
                        'torque': body_tor[i] if i < len(body_tor) else 0.0,
                    })
                else:
                    leg_idx = i - ndof_body
                    self.joint_states.append({
                        'name': name,
                        'position': leg_pos[leg_idx] if leg_idx < len(leg_pos) else 0.0,
                        'velocity': leg_vel[leg_idx] if leg_idx < len(leg_vel) else 0.0,
                        'torque': leg_tor[leg_idx] if leg_idx < len(leg_tor) else 0.0,
                    })
            self.update_callback()
        except Exception as e:
            self.get_logger().warn(f'Failed to parse robot status: {e}')

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