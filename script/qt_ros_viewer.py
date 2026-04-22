#!/usr/bin/env python3
import sys
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from joint_interfaces.msg import JointCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from PyQt5.QtWidgets import (QApplication, QMainWindow, QTableWidget, 
                             QTableWidgetItem, QVBoxLayout, QWidget, 
                             QHeaderView, QLabel, QFrame, QScrollArea)
from PyQt5.QtCore import pyqtSignal, QObject, Qt, QTimer
from PyQt5.QtGui import QFont

# --- 允许显示的关节白名单 (12个) ---
ALLOWED_JOINTS = [
    'HIP_PITCH_L_J1', 'HIP_ROLL_L_J2', 'HIP_YAW_L_J3', 'KNEE_PITCH_L_J4', 'ANKLE_PITCH_L_J5', 'ANKLE_ROLL_L_J6',
    'HIP_PITCH_R_J1', 'HIP_ROLL_R_J2', 'HIP_YAW_R_J3', 'KNEE_PITCH_R_J4', 'ANKLE_PITCH_R_J5', 'ANKLE_ROLL_R_J6'
]

class RosSignals(QObject):
    cmd_received = pyqtSignal(object)
    state_received = pyqtSignal(object)
    feedback_received = pyqtSignal(object)

class JointMultiMonitorNode(Node):
    def __init__(self, signal_emitter):
        super().__init__('joint_multi_monitor')
        self.signal_emitter = signal_emitter
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        
        self.cmd_sub = self.create_subscription(JointCommand, '/hardware/joint_command', 
            lambda msg: self.signal_emitter.cmd_received.emit(msg), qos)
        
        self.state_sub = self.create_subscription(JointState, '/hardware/joint_state', 
            lambda msg: self.signal_emitter.state_received.emit(msg), qos)
            
        self.feedback_sub = self.create_subscription(String, '/feedback/robot_server_state', 
            lambda msg: self.signal_emitter.feedback_received.emit(msg), 10)

class JointSubscriberApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.signals = RosSignals()
        self.signals.cmd_received.connect(self.process_cmd)
        self.signals.state_received.connect(self.process_state)
        self.signals.feedback_received.connect(self.process_feedback)
        
        self.cmd_row_map = {}
        self.state_row_map = {}
        self.feedback_row_map = {}
        self.initUI()
        
    def initUI(self):
        self.setWindowTitle('Robot Joint Multi-Monitor (Ankle Motor Updated)')
        self.setGeometry(100, 100, 1100, 1000)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        self.setCentralWidget(scroll)
        
        container = QWidget()
        scroll.setWidget(container)
        main_layout = QVBoxLayout(container)

        # 1. 指令表格
        main_layout.addWidget(QLabel("<b>1. 目标指令 (JointCommand)</b>"))
        self.cmd_table = self.create_table(["关节名称", "位置", "速度", "力矩", "前馈", "刚度", "阻尼"], 240)
        main_layout.addWidget(self.cmd_table)
        main_layout.addWidget(self.create_line())

        # 2. 硬件反馈表格
        main_layout.addWidget(QLabel("<b>2. 硬件反馈 (JointState)</b>"))
        self.state_table = self.create_table(["关节名称", "实际位置", "实际速度", "实际力矩"], 240)
        main_layout.addWidget(self.state_table)
        main_layout.addWidget(self.create_line())

        # 3. 服务器 Leg 反馈表格
        main_layout.addWidget(QLabel("<b>3. 服务器 Leg 内部状态 (Robot Server State)</b>"))
        self.feedback_table = self.create_table(["关节名称", "位置", "速度", "力矩", "刚度", "阻尼"], 240)
        main_layout.addWidget(self.feedback_table)
        main_layout.addWidget(self.create_line())

        # 4. 踝部电机状态表格 (修改为4行)
        main_layout.addWidget(QLabel("<b>4. 踝部电机原始状态 (Ankle Motors)</b>"))
        # 列名修改为用户指定的字段
        ankle_headers = ["电机ID", "位置反馈 (fb)", "位置指令 (cmd)", "输出力矩 (tor)"]
        self.ankle_table = QTableWidget()
        self.ankle_table.setRowCount(4)
        self.ankle_table.setColumnCount(len(ankle_headers))
        self.ankle_table.setHorizontalHeaderLabels(ankle_headers)
        self.ankle_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.ankle_table.setFixedHeight(160) # 增加高度显示4行
        self.ankle_table.setEditTriggers(QTableWidget.NoEditTriggers)
        
        # 初始化电机ID列
        motor_ids = ['4', '5', '10', '11']
        for i, m_id in enumerate(motor_ids):
            item = QTableWidgetItem(m_id)
            item.setBackground(Qt.cyan)
            item.setTextAlignment(Qt.AlignCenter)
            self.ankle_table.setItem(i, 0, item)
            
        main_layout.addWidget(self.ankle_table)

    def create_table(self, headers, height):
        table = QTableWidget()
        table.setColumnCount(len(headers))
        table.setHorizontalHeaderLabels(headers)
        table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        table.setEditTriggers(QTableWidget.NoEditTriggers)
        table.setAlternatingRowColors(True)
        table.setMinimumHeight(height)
        return table

    def create_line(self):
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        return line

    def get_row(self, table, row_map, joint_name):
        if joint_name not in ALLOWED_JOINTS: return None
        if joint_name not in row_map:
            row = table.rowCount()
            table.insertRow(row)
            row_map[joint_name] = row
            item = QTableWidgetItem(joint_name)
            item.setBackground(Qt.lightGray)
            font = QFont(); font.setBold(True)
            item.setFont(font)
            table.setItem(row, 0, item)
        return row_map[joint_name]

    def update_cell(self, table, row, col, val, precision=4):
        try:
            if val is None or val == "-":
                text = "-"
            else:
                text = f"{float(val):.{precision}f}"
        except (ValueError, TypeError):
            text = str(val)
            
        item = table.item(row, col)
        if not item:
            item = QTableWidgetItem(text)
            item.setTextAlignment(Qt.AlignCenter)
            table.setItem(row, col, item)
        else:
            item.setText(text)

    # --- 消息处理逻辑 ---

    def process_cmd(self, msg):
        for i, name in enumerate(msg.name):
            row = self.get_row(self.cmd_table, self.cmd_row_map, name)
            if row is not None:
                attrs = ['position', 'velocity', 'torque', 'torque_feedforward', 'stiffness', 'damping']
                for col, attr in enumerate(attrs, 1):
                    val_list = getattr(msg, attr)
                    val = val_list[i] if i < len(val_list) else 0.0
                    self.update_cell(self.cmd_table, row, col, val)

    def process_state(self, msg):
        for i, name in enumerate(msg.name):
            row = self.get_row(self.state_table, self.state_row_map, name)
            if row is not None:
                self.update_cell(self.state_table, row, 1, msg.position[i] if i < len(msg.position) else 0.0)
                self.update_cell(self.state_table, row, 2, msg.velocity[i] if i < len(msg.velocity) else 0.0)
                self.update_cell(self.state_table, row, 3, msg.effort[i] if i < len(msg.effort) else 0.0)

    def process_feedback(self, msg):
        try:
            data = json.loads(msg.data)
            all_names = data.get('joint_names', [])
            
            # 1. 服务器 Leg 内部状态处理
            leg_indices_in_names = [idx for idx, n in enumerate(all_names) if n in ALLOWED_JOINTS]
            keys = ['leg_joint_position', 'leg_joint_velocity', 'leg_joint_torque', 'leg_stiffness', 'leg_damping']
            
            for leg_i, name_idx in enumerate(leg_indices_in_names):
                name = all_names[name_idx]
                row = self.get_row(self.feedback_table, self.feedback_row_map, name)
                if row is not None:
                    for col, key in enumerate(keys, 1):
                        val_list = data.get(key, [])
                        val = val_list[leg_i] if leg_i < len(val_list) else "-"
                        self.update_cell(self.feedback_table, row, col, val)
            
            # 2. 踝部电机处理 (4行显示)
            # 对应字段映射
            ankle_fb = data.get('leg_ankle_motor_position_fb', [])
            ankle_cmd = data.get('leg_ankle_motor_position_cmd', [])
            ankle_tor = data.get('leg_ankle_motor_torque', [])

            for i in range(4):
                # 列1: 反馈, 列2: 指令, 列3: 力矩
                fb_val = ankle_fb[i] if i < len(ankle_fb) else "-"
                cmd_val = ankle_cmd[i] if i < len(ankle_cmd) else "-"
                tor_val = ankle_tor[i] if i < len(ankle_tor) else "-"
                
                self.update_cell(self.ankle_table, i, 1, fb_val, precision=4)
                self.update_cell(self.ankle_table, i, 2, cmd_val, precision=4)
                self.update_cell(self.ankle_table, i, 3, tor_val, precision=4)

        except Exception as e:
            print(f"Error parsing feedback JSON: {e}")

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    gui = JointSubscriberApp()
    node = JointMultiMonitorNode(gui.signals)
    
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    timer.start(10) 
    
    gui.show()
    try:
        sys.exit(app.exec_())
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()