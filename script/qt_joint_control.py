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
                    Qt.Key_Right: +0.1,
                    Qt.Key_Down:  -0.01,
                    Qt.Key_Left:  -0.1,
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
        self.setFixedSize(1600, 700)

        self.feedback_table = QTableWidget()
        self.control_table = CustomTableWidget(self.send_control_data)
        self.control_table.itemChanged.connect(self.onControlItemChanged)

        self.sync_button = QPushButton("同步反馈数据到控制表")
        self.sync_button.clicked.connect(self.sync_from_feedback)

        self.init_tables()

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

        # 运动计划: { name: {start_pos, target_pos, velocity, start_time, total_time} }
        self.motion_plans = {}
        self.motion_timer = QTimer()
        self.motion_timer.timeout.connect(self.update_motion)
        self.motion_timer.start(10)  # 0.01s

    def init_tables(self):
        headers_feedback = ["name", "position", "velocity", "torque",
                            "pos_torque_limit", "neg_torque_limit"]
        headers_control = ["name", "target_position", "target_velocity"]  # 新增列
        self.feedback_table.setColumnCount(len(headers_feedback))
        self.feedback_table.setHorizontalHeaderLabels(headers_feedback)
        self.control_table.setColumnCount(len(headers_control))
        self.control_table.setHorizontalHeaderLabels(headers_control)

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

            # 修改 target_velocity 时不立即下发位置，只调整当前计划速度
            if changed_col == 2:  # velocity 列
                if name in self.motion_plans:
                    now = time.time()
                    plan = self.motion_plans[name]
                    elapsed = now - plan['start_time']
                    if plan['total_time'] > 0:
                        ratio = min(elapsed / plan['total_time'], 1.0)
                        current_pos = plan['start_pos'] + (plan['target_pos'] - plan['start_pos']) * ratio
                    else:
                        current_pos = plan['target_pos']
                    remaining_dist = plan['target_pos'] - current_pos
                    if abs(remaining_dist) < 1e-6 or target_velocity <= 0.0:
                        # 结束计划，等下一次位置修改再处理
                        self.motion_plans.pop(name, None)
                    else:
                        new_total = abs(remaining_dist) / target_velocity
                        self.motion_plans[name] = {
                            'start_pos': current_pos,
                            'target_pos': plan['target_pos'],
                            'velocity': target_velocity,
                            'start_time': now,
                            'total_time': new_total,
                        }
                return

            # 修改 target_position 时启动/更新计划
            if changed_col == 1:  # position 列
                # step 模式下直接一次性下发，不进行插值
                if self.step_mode:
                    self.motion_plans.pop(name, None)
                    self.ros_node.publish_control([{'name': name, 'target_position': target_position}])
                    return
                # 若速度为0则直接一次性下发
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
        current_time = time.time()
        # 若已有计划，先计算当前位置作为新的起点
        if name in self.motion_plans:
            plan = self.motion_plans[name]
            elapsed = current_time - plan['start_time']
            if plan['total_time'] > 0:
                ratio = min(elapsed / plan['total_time'], 1.0)
                current_pos = plan['start_pos'] + (plan['target_pos'] - plan['start_pos']) * ratio
            else:
                current_pos = plan['target_pos']
        else:
            current_pos = self.get_current_feedback_position(name)

        distance = target_pos - current_pos
        if abs(distance) < 1e-6 or velocity <= 0.0:
            # 直接到达
            self.motion_plans.pop(name, None)
            self.ros_node.publish_control([{'name': name, 'target_position': target_pos}])
            return

        total_time = abs(distance) / velocity
        self.motion_plans[name] = {
            'start_pos': current_pos,
            'target_pos': target_pos,
            'velocity': velocity,
            'start_time': current_time,
            'total_time': total_time,
        }

    def update_motion(self):
        if not self.motion_plans:
            return
        now = time.time()
        publish_items = []
        finished = []
        for name, plan in self.motion_plans.items():
            dt = now - plan['start_time']
            if plan['total_time'] <= 0:
                pos = plan['target_pos']
                finished.append(name)
            else:
                ratio = dt / plan['total_time']
                if ratio >= 1.0:
                    pos = plan['target_pos']
                    finished.append(name)
                else:
                    pos = plan['start_pos'] + (plan['target_pos'] - plan['start_pos']) * ratio
            publish_items.append({'name': name, 'target_position': pos})
        if publish_items:
            self.ros_node.publish_control(publish_items)
        for name in finished:
            self.motion_plans.pop(name, None)

    def onControlItemChanged(self, item):
        # 仅对 velocity 列 (索引2) 手动输入进行限幅
        if item and item.column() == 2:
            try:
                v = float(item.text())
            except ValueError:
                v = 0.10
            v = max(0.01, min(0.2, v))
            new_text = f"{v:.2f}"
            if item.text() != new_text:
                item.setText(new_text)

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