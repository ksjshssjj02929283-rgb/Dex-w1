import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QTableWidget, QTableWidgetItem,
    QVBoxLayout, QHBoxLayout, QPushButton, QLabel
)
from PyQt5.QtCore import QTimer
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from joint_interfaces.msg import JointVelocityTorqueLimitFeedback, JointControl, JointPositionControl
from PyQt5.QtCore import Qt

class ROS2Node(Node):
    def __init__(self, update_callback):
        super().__init__('joint_control_gui')
        self.update_callback = update_callback
        self.joint_states = []
        self.velocity_torque_limits = {}

        self.create_subscription(JointState, '/feedback/joint', self.joint_state_callback, 10)
        self.create_subscription(JointVelocityTorqueLimitFeedback, '/feedback/joint_velocity_torque_limit', self.limit_feedback_callback, 10)

        self.control_pub = self.create_publisher(JointControl, '/control/joint', 10)
        self.position_control_pub = self.create_publisher(JointPositionControl, '/control/joint_position', 10)

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
        msg = JointControl()
        msg.name = []
        msg.position = []
        msg.positive_velocity_limit = []
        msg.negative_velocity_limit = []
        msg.positive_torque_limit = []
        msg.negative_torque_limit = []
        for item in control_data:
            msg.name.append(item['name'])
            msg.position.append(item['target_position'])
            msg.positive_velocity_limit.append(item['pos_vel_limit'])
            msg.negative_velocity_limit.append(item['neg_vel_limit'])
            msg.positive_torque_limit.append(item['pos_torque_limit'])
            msg.negative_torque_limit.append(item['neg_torque_limit'])
        self.control_pub.publish(msg)

        pos_msg = JointPositionControl()
        pos_msg.name = msg.name
        pos_msg.position = msg.position
        self.position_control_pub.publish(pos_msg)


class CustomTableWidget(QTableWidget):
    def __init__(self,send_data, parent=None):
        super().__init__( parent)
        self.send_msg=send_data
        self.current_cell = None

    def keyPressEvent(self, event):
        current_item = self.currentItem()
        if current_item:
            try:
                current_value = float(current_item.text())

                model = self.model()
                row = current_item.row()
                col = current_item.column()

                #if col >=4:
                #    return

                if event.key() == Qt.Key_Enter or  event.key() == Qt.Key_Return:
                    formatted_value = f"{current_value:.2f}"
                    current_item.setText(formatted_value)
                    self.send_msg()
                    return
                else:
                    if event.key() == Qt.Key_Up:
                        current_value=current_value + 0.01
                        if col == 2 or col == 3:
                            if current_value>=3.14:
                                current_value=3.14
                        formatted_value = f"{current_value:.2f}"
                        current_item.setText(formatted_value)
                        self.send_msg()
                    elif event.key() == Qt.Key_Right:
                        current_value = current_value + 0.1
                        if col == 2 or col == 3:
                            if current_value>=3.14:
                                current_value=3.14
                        formatted_value = f"{current_value:.2f}"
                        current_item.setText(formatted_value)
                        self.send_msg()

                    elif event.key() == Qt.Key_Down:
                        current_value = current_value - 0.01
                        if col == 2 or col == 3:
                            if current_value <= 0:
                                current_value = 0
                        formatted_value = f"{current_value:.2f}"
                        current_item.setText(formatted_value)
                        self.send_msg()
                    elif event.key() == Qt.Key_Left:
                        current_value = current_value - 0.1
                        if col == 2 or col == 3:
                            if current_value <=0:
                                current_value = 0
                        formatted_value = f"{current_value:.2f}"
                        current_item.setText(formatted_value)
                        self.send_msg()
                     # 触发编辑完成信号
                    index = model.index(row, col)
                    model.dataChanged.emit(index, index)
                    return
            except ValueError:
                # 如果单元格内容不能转换为数字，则不做处理
                pass
        super().keyPressEvent(event)

class JointControlWidget(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setFixedSize(1600, 700)

        self.feedback_table = QTableWidget()
        self.control_table = CustomTableWidget(self.send_control_data)

        self.sync_button = QPushButton("同步反馈数据到控制表")
        self.send_button = QPushButton("发送控制指令")
        self.sync_button.clicked.connect(self.sync_from_feedback)
        self.send_button.clicked.connect(self.send_control_data)

        self.init_tables()

        layout = QHBoxLayout()
        left = QVBoxLayout()
        right = QVBoxLayout()

        left.addWidget(QLabel("反馈数据"))
        left.addWidget(self.feedback_table)

        right.addWidget(QLabel("控制数据"))
        right.addWidget(self.control_table)
        right.addWidget(self.sync_button)
        right.addWidget(self.send_button)

        layout.addLayout(left)
        layout.addLayout(right)
        self.setLayout(layout)
        self.setWindowTitle("Joint Control Interface")
        self.control_table.itemSelectionChanged.connect(self.onSelectionChanged)

        self.change_pos=3

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_feedback_table)
        self.timer.start(500)

    def init_tables(self):
        headers_feedback = ["name", "position", "velocity", "torque",
                            "pos_vel_limit", "neg_vel_limit",
                            "pos_torque_limit", "neg_torque_limit"]
        headers_control = ["name", "target_position", "pos_vel_limit", "neg_vel_limit",
                           "pos_torque_limit", "neg_torque_limit"]
        self.feedback_table.setColumnCount(len(headers_feedback))
        self.feedback_table.setHorizontalHeaderLabels(headers_feedback)
        self.control_table.setColumnCount(len(headers_control))
        self.control_table.setHorizontalHeaderLabels(headers_control)

    def onSelectionChanged(self):
        # 获取当前选中的单元格
        selected_items = self.control_table.selectedItems()
        if selected_items:
            current_item = selected_items[0]
            # 可以在这里添加焦点样式，例如背景色
            current_item.setBackground(Qt.yellow)
            #print(f"onSelectionChanged:{current_item}\r\n")

            # 清除其他单元格的焦点样式
            for row in range(self.control_table.rowCount()):
                for col in range(self.control_table.columnCount()):
                    item = self.control_table.item(row, col)
                    if item and item != current_item:
                        item.setBackground(Qt.white)

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
                f"{limits.get('pos_vel_limit', 0.0):.2f}",
                f"{limits.get('neg_vel_limit', 0.0):.2f}",
                f"{limits.get('pos_torque_limit', 0.0):.2f}",
                f"{limits.get('neg_torque_limit', 0.0):.2f}",
            ]
            for col, val in enumerate(values):
                self.feedback_table.setItem(row, col, QTableWidgetItem(val))
        self.feedback_table.resizeColumnsToContents()
        if self.change_pos > 0:
           self.change_pos=self.change_pos-1
           self.sync_from_feedback()

    def sync_from_feedback(self):
        rows = self.feedback_table.rowCount()
        self.control_table.setRowCount(rows)
        for row in range(rows):
            values = [
                self.feedback_table.item(row, 0).text(),  # name
                self.feedback_table.item(row, 1).text(),  # position
                str(0.1),  # pos_vel_limit
                str(0.1),  # neg_vel_limit
                self.feedback_table.item(row, 6).text(),  # pos_torque_limit
                self.feedback_table.item(row, 7).text(),  # neg_torque_limit
            ]
            for col, val in enumerate(values):
                self.control_table.setItem(row, col, QTableWidgetItem(val))
        self.control_table.resizeColumnsToContents()

    def send_control_data(self):
        rows = self.control_table.rowCount()
        control_data = []
        for row in range(rows):
            try:
                control_data.append({
                    'name': self.control_table.item(row, 0).text(),
                    'target_position': float(self.control_table.item(row, 1).text()),
                    'pos_vel_limit': float(self.control_table.item(row, 2).text()),
                    'neg_vel_limit': float(self.control_table.item(row, 3).text()),
                    'pos_torque_limit': float(self.control_table.item(row, 4).text()),
                    'neg_torque_limit': float(self.control_table.item(row, 5).text()),
                })
            except Exception as e:
                print(f"Error parsing control data row {row}: {e}")
        self.ros_node.publish_control(control_data)

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