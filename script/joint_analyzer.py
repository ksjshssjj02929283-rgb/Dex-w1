#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Joint Analyzer
Integrated Joint Control, Data Acquisition (SHM/ROS2), and Real-time Plotting.
"""

import sys
import os
import time
import csv
import ctypes
import threading
import argparse
from datetime import datetime

# PyQt5 imports
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QTableWidget, QTableWidgetItem,
    QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QLineEdit, QFileDialog,
    QTabWidget, QHeaderView, QGroupBox, QFormLayout, QDoubleSpinBox, QSplitter, QComboBox,
    QScrollArea
)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QObject
from PyQt5.QtGui import QColor, QFont

# Matplotlib for PyQt5
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

# ROS 2 imports
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState, Imu
    from joint_interfaces.msg import JointVelocityTorqueLimitFeedback, JointPositionControl
    from robot_server_interfaces.srv import SetProperty
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

# ==============================================================================
# Shared Memory Definitions (Sync with record.py / shared_memory_def.h)
# ==============================================================================
FEEDBACK_SHM_KEY = 0x1005
CONTROL_SHM_KEY = 0x2005
MAX_JOINT_NUM = 32
BUFFER_CAPACITY = 8

# Load libc
libc = ctypes.CDLL("libc.so.6")
shmget = libc.shmget
shmat = libc.shmat
shmdt = libc.shmdt
shmget.restype = ctypes.c_int
shmat.restype = ctypes.c_void_p

class Vector3f(ctypes.Structure):
    _fields_ = [("x", ctypes.c_float), ("y", ctypes.c_float), ("z", ctypes.c_float)]

class Quaternionf(ctypes.Structure):
    _fields_ = [("x", ctypes.c_float), ("y", ctypes.c_float), ("z", ctypes.c_float), ("w", ctypes.c_float)]

class ImuData(ctypes.Structure):
    _fields_ = [
        ("id", ctypes.c_uint32), ("orientation", Quaternionf),
        ("angular_velocity", Vector3f), ("linear_acceleration", Vector3f),
        ("mag", Vector3f), ("eul", Vector3f),
        ("pressure", ctypes.c_float), ("timestamp", ctypes.c_uint32),
    ]

class JointFeedback(ctypes.Structure):
    _fields_ = [
        ("joint_name", ctypes.c_char * 32), ("joint_status_word", ctypes.c_uint16),
        ("joint_error_code", ctypes.c_uint16), ("joint_position", ctypes.c_double),
        ("joint_velocity", ctypes.c_double), ("joint_torque", ctypes.c_double),
        ("joint_sensor_torque", ctypes.c_double), ("joint_temperature", ctypes.c_double),
    ]

class JointControl(ctypes.Structure):
    _fields_ = [
        ("joint_name", ctypes.c_char * 32), ("joint_mode", ctypes.c_uint16),
        ("joint_position", ctypes.c_double), ("joint_velocity", ctypes.c_double),
        ("joint_torque", ctypes.c_double), ("joint_torque_feedforward", ctypes.c_double),
        ("joint_Kp", ctypes.c_double), ("joint_Kd", ctypes.c_double),
        ("joint_positive_torque_limit", ctypes.c_double), ("joint_negative_torque_limit", ctypes.c_double),
    ]

# Keep these internal for alignment, even if not used in UI
class ChassisFeedback(ctypes.Structure):
    _fields_ = [
        ("timestamp", ctypes.c_int64), ("left_velocity", ctypes.c_double), ("right_velocity", ctypes.c_double),
        ("chassis_motor_velocity", ctypes.c_double * 4), ("chassis_motor_direction", ctypes.c_double * 4),
        ("status_word", ctypes.c_uint16), ("error_code_left", ctypes.c_uint16), ("error_code_right", ctypes.c_uint16),
        ("left_current", ctypes.c_double), ("right_current", ctypes.c_double), ("driver_temperature", ctypes.c_double),
        ("left_temperature", ctypes.c_double), ("right_temperature", ctypes.c_double),
    ]

class ChassisControl(ctypes.Structure):
    _fields_ = [
        ("cnt", ctypes.c_uint64), ("chassis_mode", ctypes.c_int),
        ("chassis_left_velocity", ctypes.c_double), ("chassis_right_velocity", ctypes.c_double),
        ("chassis_motor_velocity", ctypes.c_double * 4), ("chassis_motor_direction", ctypes.c_double * 4),
        ("chassis_estop", ctypes.c_int), ("chassis_neutral", ctypes.c_int),
    ]

class RobotFeedback(ctypes.Structure):
    _fields_ = [
        ("cnt", ctypes.c_uint64), ("timestamp", ctypes.c_uint64), ("robot_status_word", ctypes.c_uint16),
        ("al_status", ctypes.c_uint8 * MAX_JOINT_NUM), ("joint_feedback", JointFeedback * MAX_JOINT_NUM),
        ("chassis_feedback", ChassisFeedback), ("imu_data", ImuData),
    ]

class RobotControl(ctypes.Structure):
    _fields_ = [
        ("cnt", ctypes.c_uint64), ("timestamp", ctypes.c_uint64), ("robot_control_word", ctypes.c_uint16),
        ("joint_control", JointControl * MAX_JOINT_NUM), ("chassis_control", ChassisControl),
    ]

class timespec(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_long), ("tv_nsec", ctypes.c_long)]

class QueueElementFbk(ctypes.Structure):
    _fields_ = [
        ("data", RobotFeedback),
        ("cnt", ctypes.c_uint64),
        ("ts", timespec),
        ("valid", ctypes.c_bool)
    ]

class QueueElementCtrl(ctypes.Structure):
    _fields_ = [
        ("data", RobotControl),
        ("cnt", ctypes.c_uint64),
        ("ts", timespec),
        ("valid", ctypes.c_bool)
    ]

class FeedbackShmStruct(ctypes.Structure):
    _fields_ = [("ring_buffer", QueueElementFbk * BUFFER_CAPACITY), ("latest_and_reader", ctypes.c_uint64)]

class ControlShmStruct(ctypes.Structure):
    _fields_ = [("ring_buffer", QueueElementCtrl * BUFFER_CAPACITY), ("latest_and_reader", ctypes.c_uint64)]

def get_shm_ptr(key, struct_type):
    shmid = shmget(key, ctypes.sizeof(struct_type), 0o666)
    if shmid < 0: return None
    ptr = shmat(shmid, None, 0)
    if ptr == ctypes.c_void_p(-1).value: return None
    return ctypes.cast(ptr, ctypes.POINTER(struct_type))

def cstr_to_py(c_arr):
    return bytes(c_arr).split(b"\x00", 1)[0].decode("utf-8", errors="ignore")

# ==============================================================================
# Helper functions for word interpretation (Ref: w1_utils.h)
# ==============================================================================

def interpret_status_word(sw):
    mapping = {
        0: "NOT_READY",
        1: "READY",
        2: "ENABLED",
        4: "MOTOR_FAULT",
        8: "MOTOR_QUICK_STOP",
        16: "SLAVE_SCANNING"
    }
    return mapping.get(sw, str(sw))

def interpret_control_word(cw):
    flags = []
    if cw & (1 << 0): flags.append("SWITCH_ON")
    if cw & (1 << 1): flags.append("ENABLE")
    if cw & (1 << 2): flags.append("QUICK_STOP")
    if cw & (1 << 3): flags.append("CLEAR_FAULT")
    if cw & (1 << 4): flags.append("CST_MODE")
    
    if not flags:
        return str(cw)
    return "|".join(flags)

# ==============================================================================
# Data Service
# ==============================================================================

class DataService(QObject):
    updated = pyqtSignal()
    record_status = pyqtSignal(str, bool) # status_msg, is_active
    recording_done = pyqtSignal(str) # Path to CSV

    def __init__(self, use_sim=False):
        super().__init__()
        self.use_sim = use_sim
        self.fbk_ptr = get_shm_ptr(FEEDBACK_SHM_KEY, FeedbackShmStruct)
        self.ctrl_ptr = get_shm_ptr(CONTROL_SHM_KEY, ControlShmStruct)
        
        self.joint_data = {} # name -> {'pos', 'vel', 'torque', 'temp', 'target_pos', 'target_vel', 'target_torque', 'status', 'err'}
        self.joint_names = []
        self.robot_status_word = 0
        self.robot_control_word = 0
        
        self.is_recording = False
        self.record_stop_event = threading.Event()
        
        if ROS2_AVAILABLE:
            rclpy.init()
            self.node = Node('dexe_dashboard_service')
            suffix = '_sim' if use_sim else ''
            self.node.create_subscription(JointState, f'/feedback/joint{suffix}', self._ros_joint_cb, 10)
            self.pos_pub = self.node.create_publisher(JointPositionControl, f'/control/joint_position{suffix}', 10)
            self.set_prop_cli = self.node.create_client(SetProperty, '/control/set_property')
            self.ros_thread = threading.Thread(target=self._ros_spin, daemon=True)
            self.ros_thread.start()

        # Update Timer for internal data processing
        self.timer = QTimer()
        self.timer.timeout.connect(self._process_data)
        self.timer.start(20) # 50Hz internal refresh

    def _ros_spin(self):
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def _ros_joint_cb(self, msg):
        # Initialize joint names from ROS 2 topic exactly once
        if not self.joint_names and msg.name:
            self.joint_names = list(msg.name)

        for i, name in enumerate(msg.name):
            if name not in self.joint_data: 
                self.joint_data[name] = {}
            
            self.joint_data[name].update({
                'pos': msg.position[i],
                'vel': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                'torque': msg.effort[i] if i < len(msg.effort) else 0.0
            })

    def publish_control(self, commands):
        if ROS2_AVAILABLE:
            msg = JointPositionControl()
            for name, pos in commands.items():
                msg.name.append(name)
                msg.position.append(pos)
            self.pos_pub.publish(msg)

    def clear_errors(self):
        if not ROS2_AVAILABLE: return
        if not self.set_prop_cli.wait_for_service(timeout_sec=0.5):
            return
        
        req = SetProperty.Request()
        req.property_type = "ClearError"
        req.value = ""
        self.set_prop_cli.call_async(req)

    def set_property(self, prop_type, value):
        if not ROS2_AVAILABLE: return
        if not self.set_prop_cli.wait_for_service(timeout_sec=0.5):
            return
        req = SetProperty.Request()
        req.property_type = prop_type
        req.value = str(value)
        self.set_prop_cli.call_async(req)

    def start_recording(self, path, duration, period, trigger_cfg=None):
        if self.is_recording: return
        self.is_recording = True
        self.record_stop_event.clear()
        threading.Thread(target=self._record_loop, args=(path, duration, period, trigger_cfg), daemon=True).start()

    def stop_recording(self):
        self.record_stop_event.set()

    def _record_loop(self, path, duration, period, trigger_cfg=None):
        status_prefix = ""
        
        # Pre-build joint mapping once before recording
        # This ensures we use the same indices as the dashboard for the duration of the recording
        joint_mapping = [] # List of (name, fbk_idx, ctrl_idx)
        
        # We need to see what joints are available
        if self.fbk_ptr:
            latest_f = self.fbk_ptr.contents.latest_and_reader >> 32
            fbk = self.fbk_ptr.contents.ring_buffer[latest_f % BUFFER_CAPACITY]
            if fbk.valid:
                for i in range(MAX_JOINT_NUM):
                    name = cstr_to_py(fbk.data.joint_feedback[i].joint_name)
                    if not name: continue
                    
                    # Find matching control index
                    ctrl_idx = -1
                    if self.ctrl_ptr:
                        latest_c = self.ctrl_ptr.contents.latest_and_reader >> 32
                        ctrl = self.ctrl_ptr.contents.ring_buffer[latest_c % BUFFER_CAPACITY]
                        if ctrl.valid:
                            for j in range(MAX_JOINT_NUM):
                                if cstr_to_py(ctrl.data.joint_control[j].joint_name) == name:
                                    ctrl_idx = j
                                    break
                    
                    joint_mapping.append((name, i, ctrl_idx))

        if trigger_cfg and trigger_cfg['mode'] != "None":
            mode = trigger_cfg['mode']
            joint = trigger_cfg['joint']
            data_type = trigger_cfg['data']
            threshold = trigger_cfg['threshold']
            
            self.record_status.emit(f"Waiting for {mode} on {joint}...", True)
            
            # Find SHM index for trigger signal from our mapping
            trigger_fbk_idx = -1
            trigger_ctrl_idx = -1
            for name, f_idx, c_idx in joint_mapping:
                if name == joint:
                    trigger_fbk_idx = f_idx
                    trigger_ctrl_idx = c_idx
                    break
            
            if trigger_fbk_idx == -1:
                # Fallback to immediate if joint not found
                pass
            else:
                initial_val = None
                triggered = False
                while not triggered and not self.record_stop_event.is_set():
                    # Sample latest data from SHM
                    fbk_elem, ctrl_elem = None, None
                    if self.fbk_ptr:
                        latest_f = self.fbk_ptr.contents.latest_and_reader >> 32
                        fbk_elem = self.fbk_ptr.contents.ring_buffer[latest_f % BUFFER_CAPACITY]
                    if self.ctrl_ptr:
                        latest_c = self.ctrl_ptr.contents.latest_and_reader >> 32
                        ctrl_elem = self.ctrl_ptr.contents.ring_buffer[latest_c % BUFFER_CAPACITY]
                    
                    if not fbk_elem or not fbk_elem.valid:
                        time.sleep(period)
                        continue
                    
                    val = 0.0
                    fbk_data = fbk_elem.data.joint_feedback[trigger_fbk_idx]
                    ctrl_data = None
                    if trigger_ctrl_idx != -1 and ctrl_elem and ctrl_elem.valid:
                         ctrl_data = ctrl_elem.data.joint_control[trigger_ctrl_idx]
                    
                    if "Target Position" in data_type:
                        val = ctrl_data.joint_position if ctrl_data else 0.0
                    elif "Target Velocity" in data_type:
                        val = ctrl_data.joint_velocity if ctrl_data else 0.0
                    elif "Target Torque" in data_type:
                        val = ctrl_data.joint_torque_feedforward if ctrl_data else 0.0
                    elif "Position Error" in data_type:
                        p_cmd = ctrl_data.joint_position if ctrl_data else fbk_data.joint_position
                        val = p_cmd - fbk_data.joint_position
                    elif "Position" in data_type: val = fbk_data.joint_position
                    elif "Velocity" in data_type: val = fbk_data.joint_velocity
                    elif "Torque" in data_type: val = fbk_data.joint_torque
                    
                    if initial_val is None: initial_val = val
                    
                    if mode == "Rising Edge" and val > threshold: triggered = True
                    elif mode == "Falling Edge" and val < threshold: triggered = True
                    elif mode == "Delta Trigger" and abs(val - initial_val) > threshold: triggered = True
                    
                    if not triggered:
                        time.sleep(period)
                
                if self.record_stop_event.is_set():
                    self.record_status.emit("Canceled", False)
                    self.is_recording = False
                    return
                
                status_prefix = "TRIGGERED! "
                
        self.record_status.emit(f"{status_prefix}Recording to {os.path.basename(path)}...", True)
        start_time = time.perf_counter()
        end_time = start_time + duration
        
        try:
            headers = ["Timestamp"]
            # We'll record using the joint order from joint_mapping for consistency
            for name, _, _ in joint_mapping:
                headers.extend([
                    f"{name}_Pos_Fbk", f"{name}_Vel_Fbk", f"{name}_Tor_Fbk",
                    f"{name}_Pos_Cmd", f"{name}_Vel_Cmd", f"{name}_Tor_Cmd"
                ])
            
            with open(path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(headers)
                
                next_sample = start_time
                while time.perf_counter() < end_time and not self.record_stop_event.is_set():
                    now = time.perf_counter()
                    
                    row = [now - start_time]
                    fbk_elem, ctrl_elem = None, None
                    if self.fbk_ptr:
                        latest_f = self.fbk_ptr.contents.latest_and_reader >> 32
                        fbk_elem = self.fbk_ptr.contents.ring_buffer[latest_f % BUFFER_CAPACITY]
                    if self.ctrl_ptr:
                        latest_c = self.ctrl_ptr.contents.latest_and_reader >> 32
                        ctrl_elem = self.ctrl_ptr.contents.ring_buffer[latest_c % BUFFER_CAPACITY]
                    
                    fbk_valid = fbk_elem and fbk_elem.valid
                    ctrl_valid = ctrl_elem and ctrl_elem.valid

                    for name, f_idx, c_idx in joint_mapping:
                        p_fbk, v_fbk, t_fbk = 0.0, 0.0, 0.0
                        p_cmd, v_cmd, t_cmd = 0.0, 0.0, 0.0
                        
                        if fbk_valid:
                            p_fbk = fbk_elem.data.joint_feedback[f_idx].joint_position
                            v_fbk = fbk_elem.data.joint_feedback[f_idx].joint_velocity
                            t_fbk = fbk_elem.data.joint_feedback[f_idx].joint_torque
                        
                        if ctrl_valid and c_idx != -1:
                            p_cmd = ctrl_elem.data.joint_control[c_idx].joint_position
                            v_cmd = ctrl_elem.data.joint_control[c_idx].joint_velocity
                            t_cmd = ctrl_elem.data.joint_control[c_idx].joint_torque_feedforward
                        
                        row.extend([p_fbk, v_fbk, t_fbk, p_cmd, v_cmd, t_cmd])
                    
                    writer.writerow(row)
                    next_sample += period
                    sleep_t = next_sample - time.perf_counter()
                    if sleep_t > 0: time.sleep(sleep_t)
            
            self.record_status.emit(f"Saved to {path}", False)
            self.recording_done.emit(path)
        except Exception as e:
            self.record_status.emit(f"Error: {str(e)}", False)
        finally:
            self.is_recording = False

    def _process_data(self):
        # 1. Try SHM First
        shm_success = False
        if self.fbk_ptr:
            try:
                latest_f = self.fbk_ptr.contents.latest_and_reader >> 32
                idx = latest_f % BUFFER_CAPACITY
                fbk = self.fbk_ptr.contents.ring_buffer[idx]
                if fbk.valid:
                    shm_success = True
                    self.robot_status_word = fbk.data.robot_status_word
                    
                    # Collect names and their indices, but don't append to self.joint_names yet
                    shm_joints = [] # list of (index, name)
                    for i in range(MAX_JOINT_NUM):
                        name = cstr_to_py(fbk.data.joint_feedback[i].joint_name)
                        if name:
                            shm_joints.append((i, name))

                    for i, name in shm_joints:
                        if name not in self.joint_data: 
                            self.joint_data[name] = {'shm_idx': i}
                        
                        self.joint_data[name].update({
                            'shm_idx': i,
                            'pos': fbk.data.joint_feedback[i].joint_position,
                            'vel': fbk.data.joint_feedback[i].joint_velocity,
                            'torque': fbk.data.joint_feedback[i].joint_torque,
                            'temp': fbk.data.joint_feedback[i].joint_temperature,
                            'status': fbk.data.joint_feedback[i].joint_status_word,
                            'err': fbk.data.joint_feedback[i].joint_error_code
                        })
            except Exception as e:
                pass 
        
        # 2. Control SHM (Target Pos for display)
        if self.ctrl_ptr:
            try:
                latest_c = self.ctrl_ptr.contents.latest_and_reader >> 32
                idx = latest_c % BUFFER_CAPACITY
                ctrl = self.ctrl_ptr.contents.ring_buffer[idx]
                if ctrl.valid:
                    self.robot_control_word = ctrl.data.robot_control_word
                    for i in range(MAX_JOINT_NUM):
                        name = cstr_to_py(ctrl.data.joint_control[i].joint_name)
                        if not name in self.joint_data: continue
                        self.joint_data[name].update({
                            'target_pos': ctrl.data.joint_control[i].joint_position,
                            'target_vel': ctrl.data.joint_control[i].joint_velocity,
                            'target_torque': ctrl.data.joint_control[i].joint_torque_feedforward
                        })
            except:
                pass

        # 3. If SHM was not enough, ROS data (from callback) remains in self.joint_data
        
        self.updated.emit()


# ==============================================================================
# UI Components
# ==============================================================================

class StepButton(QPushButton):
    def __init__(self, label, joint_name, direction, ds, parent_ctrl):
        super().__init__(label)
        self.joint_name = joint_name
        self.direction = direction # 1 for +, -1 for -
        self.ds = ds
        self.parent_ctrl = parent_ctrl
        
        self.setFixedWidth(40)
        
        self.long_press_timer = QTimer()
        self.long_press_timer.setSingleShot(True)
        self.long_press_timer.timeout.connect(self.start_continuous)
        
        self.pulse_timer = QTimer()
        self.pulse_timer.timeout.connect(self.pulse_move)
        
        self.pressed.connect(self.on_pressed)
        self.released.connect(self.on_released)
        self.is_long_press = False

    def on_pressed(self):
        self.is_long_press = False
        self.long_press_timer.start(400)

    def on_released(self):
        if not self.is_long_press:
            # Short click: 0.001 rad (min step)
            self.parent_ctrl.increment_cmd(self.joint_name, 0.001 * self.direction)
        
        self.long_press_timer.stop()
        self.pulse_timer.stop()
        self.is_long_press = False

    def start_continuous(self):
        self.is_long_press = True
        self.pulse_timer.start(20) # 50Hz

    def pulse_move(self):
        # 0.1 rad/s -> 0.002 rad per 20ms
        self.parent_ctrl.increment_cmd(self.joint_name, 0.002 * self.direction)

class ControlWidget(QWidget):
    def __init__(self, ds):
        super().__init__()
        self.ds = ds
        main_layout = QVBoxLayout(self)
        
        # Scroll area for joints
        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.container = QWidget()
        self.container.setObjectName("jointContainer")
        self.container.setStyleSheet("#jointContainer { background-color: white; }")
        self.joint_layout = QVBoxLayout(self.container)
        self.joint_layout.setAlignment(Qt.AlignTop)
        self.joint_layout.setSpacing(2)
        
        # Header Row
        header = QWidget()
        h_layout = QHBoxLayout(header)
        h_layout.setContentsMargins(5, 5, 5, 5)
        h_layout.addWidget(QLabel("<b>Joint</b>"), 0)
        h_layout.addStretch()
        h_layout.addWidget(QLabel("<b>Position (rad)</b>"), 0)
        self.joint_layout.addWidget(header)
        
        self.scroll.setWidget(self.container)
        main_layout.addWidget(self.scroll)
        
        btn_layout = QHBoxLayout()
        self.clear_btn = QPushButton("Clear Error")
        self.clear_btn.setStyleSheet("font-weight: bold;")
        self.clear_btn.clicked.connect(self.ds.clear_errors)
        btn_layout.addWidget(self.clear_btn)
        
        main_layout.addLayout(btn_layout)
        
        # Mode Controls
        mode_layout = QHBoxLayout()
        self.drag_btn = QPushButton("Enable Drag")
        self.drag_btn.setCheckable(True)
        self.drag_btn.setStyleSheet("""
            QPushButton { background-color: #f5f5f5; font-weight: bold; }
            QPushButton:checked { background-color: #ffe082; color: #795548; }
        """)
        self.drag_btn.clicked.connect(self.toggle_drag)
        mode_layout.addWidget(self.drag_btn)
        
        main_layout.addLayout(mode_layout)
        
        self.ds.updated.connect(self.update_ui)
        self.cmd_positions = {} # name -> value
        self.joint_widgets = {} # name -> {'pos_label': QLabel, 'row_widget': QWidget}

    def update_ui(self):
        # We need a copy of the list to avoid modification during iteration
        names = list(self.ds.joint_names)
        if not names:
            # print("DEBUG: No joints found in update_ui")
            return
        
        # Add new joints
        for name in names:
            if name not in self.joint_widgets:
                # print(f"DEBUG: Adding widget for {name}")
                row_widget = QWidget()
                row_widget.setMinimumHeight(40)
                row_layout = QHBoxLayout(row_widget)
                row_layout.setContentsMargins(5, 2, 5, 2)
                
                name_label = QLabel(name)
                name_label.setFixedWidth(120)
                name_label.setStyleSheet("font-weight: bold;")
                
                pos_label = QLabel("0.0000")
                pos_label.setFixedWidth(80)
                pos_label.setAlignment(Qt.AlignCenter)
                # Plain label, no background box
                
                btn_minus = StepButton("-", name, -1, self.ds, self)
                btn_plus = StepButton("+", name, 1, self.ds, self)
                btn_minus.setFixedWidth(40)
                btn_plus.setFixedWidth(40)
                
                row_layout.addWidget(name_label)
                row_layout.addStretch()
                row_layout.addWidget(btn_minus)
                row_layout.addWidget(pos_label)
                row_layout.addWidget(btn_plus)
                
                self.joint_layout.addWidget(row_widget)
                row_widget.show()
                self.joint_widgets[name] = {
                    'pos_label': pos_label,
                    'row_widget': row_widget
                }
        
        # Update data
        for name in names:
            data = self.ds.joint_data.get(name, {})
            p = data.get('pos', 0.0)
            
            if name in self.joint_widgets:
                self.joint_widgets[name]['pos_label'].setText(f"{p:.4f}")
            
            if name not in self.cmd_positions:
                self.cmd_positions[name] = p

    def increment_cmd(self, name, delta):
        if name not in self.cmd_positions:
            self.cmd_positions[name] = self.ds.joint_data.get(name, {}).get('pos', 0.0)
        
        self.cmd_positions[name] += delta
        self.ds.publish_control({name: self.cmd_positions[name]})

    def toggle_drag(self, checked):
        val = "1" if checked else "0"
        self.drag_btn.setText("Drag Mode: ON" if checked else "Enable Drag")
        self.ds.set_property("EnableDrag", val)

class RecordWidget(QWidget):
    def __init__(self, ds):
        super().__init__()
        self.ds = ds
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        group = QGroupBox("Data Acquisition")
        group_layout = QVBoxLayout(group)
        group_layout.setContentsMargins(5, 5, 5, 5)
        group_layout.setSpacing(5)
        
        # Row 1: Trigger settings and Duration (Targeting Trigger row on top)
        trigger_layout = QHBoxLayout()
        trigger_layout.setSpacing(10)
        
        # Helper to create tight label+widget pairs
        def add_pair(lay, label_text, widget):
            pair = QHBoxLayout()
            pair.setSpacing(2)
            lbl = QLabel(label_text)
            pair.addWidget(lbl)
            pair.addWidget(widget)
            lay.addLayout(pair)

        self.trig_joint = QComboBox()
        self.trig_joint.setFixedWidth(120)
        add_pair(trigger_layout, "Trigger Joint:", self.trig_joint)
        
        self.trig_data = QComboBox()
        self.trig_data.addItems([
            "Position (rad)", "Position Error (rad)", "Velocity (rad/s)", "Torque (Nm)",
            "Target Position (rad)", "Target Velocity (rad/s)", "Target Torque (Nm)"
        ])
        self.trig_data.setFixedWidth(160)
        add_pair(trigger_layout, "Trigger Data:", self.trig_data)
        
        self.trig_mode = QComboBox()
        self.trig_mode.addItems(["None", "Rising Edge", "Falling Edge", "Delta Trigger"])
        self.trig_mode.setFixedWidth(120)
        add_pair(trigger_layout, "Trigger Mode:", self.trig_mode)
        
        self.trig_threshold = QDoubleSpinBox()
        self.trig_threshold.setRange(-1000, 1000); self.trig_threshold.setValue(0.01); self.trig_threshold.setDecimals(3)
        self.trig_threshold.setSingleStep(0.01)
        self.trig_threshold.setFixedWidth(80)
        self.trig_threshold.setGroupSeparatorShown(False)
        self.trig_threshold.setProperty("compact", True) 
        add_pair(trigger_layout, "Trigger Threshold:", self.trig_threshold)

        self.dur_spin = QDoubleSpinBox()
        self.dur_spin.setRange(1, 3600); self.dur_spin.setValue(8.0); self.dur_spin.setSuffix("s")
        self.dur_spin.setFixedWidth(80)
        add_pair(trigger_layout, "Record Duration:", self.dur_spin)

        trigger_layout.addStretch()
        group_layout.addLayout(trigger_layout)
        
        # Row 2: Path and Record button
        path_layout = QHBoxLayout()
        path_layout.setSpacing(5)
        
        self.path_edit = QLineEdit(os.path.join(os.getcwd(), f"log_{datetime.now().strftime('%H%M%S')}.csv"))
        self.path_edit.setFixedWidth(400) # Fixed width for path box
        path_btn = QPushButton("...")
        path_btn.setFixedWidth(30)
        path_btn.clicked.connect(self.browse)
        
        path_layout.addWidget(QLabel("Record Path:"))
        path_layout.addWidget(self.path_edit)
        path_layout.addWidget(path_btn)
        
        path_layout.addSpacing(20)
        
        self.live_btn = QPushButton("Resume Live")
        self.live_btn.setFixedWidth(100)
        self.live_btn.clicked.connect(self.on_resume_live_clicked)
        path_layout.addWidget(self.live_btn)

        self.rec_btn = QPushButton("Record")
        self.rec_btn.setStyleSheet("background-color: #c8e6c9; font-weight: bold;")
        self.rec_btn.clicked.connect(self.toggle_rec)
        self.rec_btn.setFixedWidth(100)
        path_layout.addWidget(self.rec_btn)
        
        self.status = QLabel("Ready")
        self.status.setMinimumWidth(200)
        path_layout.addWidget(self.status)
        
        path_layout.addStretch() # Push everything to the left
        
        group_layout.addLayout(path_layout)
        
        layout.addWidget(group)
        self.ds.record_status.connect(self.on_status)
        self.ds.updated.connect(self.update_joint_list)
        
        self.update_live_btn_style()
        
        layout.addWidget(group)
        self.ds.record_status.connect(self.on_status)
        self.ds.updated.connect(self.update_joint_list)

    def update_joint_list(self):
        current = self.trig_joint.currentText()
        if self.trig_joint.count() != len(self.ds.joint_names):
            self.trig_joint.blockSignals(True)
            self.trig_joint.clear()
            self.trig_joint.addItems(self.ds.joint_names)
            idx = self.trig_joint.findText(current)
            if idx >= 0: self.trig_joint.setCurrentIndex(idx)
            self.trig_joint.blockSignals(False)

    def browse(self):
        p, _ = QFileDialog.getSaveFileName(self, "Save Data", "", "CSV (*.csv)")
        if p: self.path_edit.setText(p)

    def toggle_rec(self):
        if not self.ds.is_recording:
            trigger_cfg = {
                'joint': self.trig_joint.currentText(),
                'data': self.trig_data.currentText(),
                'mode': self.trig_mode.currentText(),
                'threshold': self.trig_threshold.value()
            }
            self.ds.start_recording(self.path_edit.text(), self.dur_spin.value(), 0.001, trigger_cfg)
        else:
            self.ds.stop_recording()

    def on_status(self, msg, active):
        self.status.setText(msg)
        if active:
            self.rec_btn.setText("STOP")
            self.rec_btn.setStyleSheet("background-color: #ffcdd2; font-weight: bold;")
        else:
            self.rec_btn.setText("Record")
            self.rec_btn.setStyleSheet("background-color: #c8e6c9; font-weight: bold;")

    def on_resume_live_clicked(self):
        # Notify the plot widget to resume live mode
        # Since PlotWidget is a sibling/separate, we find it through MainWindow or use the ds as bridge
        # But per current structure, MainWindow owns both. Let's add a signal to DataService or call it directly if we have a ref
        main_win = self.window()
        if hasattr(main_win, 'plot_box'):
            main_win.plot_box.resume_live()
            self.update_live_btn_style()

    def update_live_btn_style(self):
        main_win = self.window()
        is_csv = False
        if hasattr(main_win, 'plot_box'):
            is_csv = main_win.plot_box.csv_data is not None
        
        if is_csv:
            self.live_btn.setEnabled(True)
            self.live_btn.setStyleSheet("background-color: #c8e6c9; font-weight: bold;") # Green when clickable
            self.rec_btn.setEnabled(False)
            self.rec_btn.setStyleSheet("background-color: #f5f5f5; color: #9e9e9e;") # Grey when viewing CSV
        else:
            self.live_btn.setEnabled(False)
            self.live_btn.setStyleSheet("background-color: #f5f5f5; color: #9e9e9e;") # Grey when disabled
            self.rec_btn.setEnabled(True)
            self.rec_btn.setStyleSheet("background-color: #c8e6c9; font-weight: bold;") # Normal record style

class PlotWidget(QWidget):
    def __init__(self, ds):
        super().__init__()
        self.ds = ds
        layout = QVBoxLayout(self)
        
        top_bar = QHBoxLayout()
        top_bar.addWidget(QLabel("Select Joint:"))
        self.joint_combo = QComboBox()
        self.joint_combo.setMinimumWidth(200) # Wider combo box
        self.joint_combo.currentIndexChanged.connect(self.on_joint_changed)
        top_bar.addWidget(self.joint_combo)
        
        self.load_btn = QPushButton("Load CSV")
        self.load_btn.clicked.connect(self.manual_load)
        top_bar.addWidget(self.load_btn)
        
        top_bar.addSpacing(20)
        top_bar.addWidget(QLabel("Live Window:"))
        self.window_spin = QDoubleSpinBox()
        self.window_spin.setRange(1.0, 60.0); self.window_spin.setValue(8.0); self.window_spin.setSuffix("s")
        self.window_spin.setFixedWidth(80)
        self.window_spin.valueChanged.connect(self.update_window_size)
        top_bar.addWidget(self.window_spin)
        
        top_bar.addStretch()
        layout.addLayout(top_bar)
        
        # Scroll area for vertical plots
        self.plot_scroll = QScrollArea()
        self.plot_scroll.setWidgetResizable(True)
        self.plot_container = QWidget()
        self.plot_layout = QVBoxLayout(self.plot_container)
        self.plot_scroll.setWidget(self.plot_container)
        layout.addWidget(self.plot_scroll)
        
        self.canvas = FigureCanvas(Figure(figsize=(8, 12)))
        # Create 4x1 subplots (vertical)
        self.axes = self.canvas.figure.subplots(4, 1, sharex=True)
        self.canvas.figure.tight_layout(pad=3.0)
        
        self.plot_layout.addWidget(NavigationToolbar(self.canvas, self))
        self.plot_layout.addWidget(self.canvas)
        
        # Ensure a minimum height for the plots to prevent squashing when windowed
        self.canvas.setMinimumHeight(1000)
        
        self.live_data = {
            't': [], 'p_fbk': [], 'p_cmd': [], 
            'p_err': [],
            'v_fbk': [], 'v_cmd': [],
            't_fbk': [], 't_cmd': []
        }
        self.max_live_points = 200 # ~4 seconds at 50Hz
        
        self.csv_data = None
        self.ds.recording_done.connect(self.load_csv)
        self.ds.updated.connect(self.update_joint_combo)
        self.ds.updated.connect(self.update_live_plots)

    def update_window_size(self, val):
        # The cleaning logic is now handled in update_live_plots via time comparison
        pass

    def update_live_plots(self):
        if self.csv_data is not None: return # Don't update live if viewing CSV
        
        joint_name = self.joint_combo.currentText()
        if not joint_name: return
        
        data = self.ds.joint_data.get(joint_name, {})
        if not data: return
        
        now = time.time()
        if not hasattr(self, '_start_time'): self._start_time = now
        rel_time = now - self._start_time
        
        p_f = data.get('pos', 0.0)
        p_c = data.get('target_pos', p_f)
        v_f = data.get('vel', 0.0)
        v_c = data.get('target_vel', 0.0)
        t_f = data.get('torque', 0.0)
        t_c = data.get('target_torque', 0.0)
        
        self.live_data['t'].append(rel_time)
        self.live_data['p_fbk'].append(p_f)
        self.live_data['p_cmd'].append(p_c)
        self.live_data['p_err'].append(p_c - p_f)
        self.live_data['v_fbk'].append(v_f)
        self.live_data['v_cmd'].append(v_c)
        self.live_data['t_fbk'].append(t_f)
        self.live_data['t_cmd'].append(t_c)
        
        # Keep buffer size based on time duration instead of point count
        # This fixes the issue where the window size didn't match the setting
        duration = self.window_spin.value()
        while len(self.live_data['t']) > 0 and (rel_time - self.live_data['t'][0] > duration):
            for key in self.live_data: 
                self.live_data[key].pop(0)
            
        # Draw
        t = self.live_data['t']
        
        # Plot 1: Position
        self.axes[0].clear()
        self.axes[0].plot(t, self.live_data['p_fbk'], 'b-', label='Fbk')
        self.axes[0].plot(t, self.live_data['p_cmd'], 'r--', label='Cmd')
        self.axes[0].set_title("Position (rad)")
        self.axes[0].legend(loc='upper right', fontsize='small')
        
        # Plot 2: Position Error
        self.axes[1].clear()
        self.axes[1].plot(t, self.live_data['p_err'], 'k-', label='Position Error')
        self.axes[1].set_title("Position Error (rad)")
        self.axes[1].legend(loc='upper right', fontsize='small')
        
        # Plot 3: Velocity
        self.axes[2].clear()
        self.axes[2].plot(t, self.live_data['v_fbk'], 'g-', label='Fbk')
        self.axes[2].plot(t, self.live_data['v_cmd'], 'r--', label='Cmd')
        self.axes[2].set_title("Velocity (rad/s)")
        self.axes[2].legend(loc='upper right', fontsize='small')
        
        # Plot 4: Torque
        self.axes[3].clear()
        self.axes[3].plot(t, self.live_data['t_fbk'], 'm-', label='Fbk')
        self.axes[3].plot(t, self.live_data['t_cmd'], 'r--', label='Cmd')
        self.axes[3].set_title("Torque (Nm)")
        self.axes[3].legend(loc='upper right', fontsize='small')
        
        for ax in self.axes: 
            ax.grid(True)
            # Ensure a minimum y-scale range to avoid noisy autoscaling on static data
            y_min, y_max = ax.get_ylim()
            if y_max - y_min < 0.01:
                center = (y_min + y_max) / 2
                ax.set_ylim(center - 0.005, center + 0.005)

        self.canvas.draw()

    def update_joint_combo(self):
        current = self.joint_combo.currentText()
        if self.joint_combo.count() != len(self.ds.joint_names):
            self.joint_combo.blockSignals(True)
            self.joint_combo.clear()
            self.joint_combo.addItems(self.ds.joint_names)
            idx = self.joint_combo.findText(current)
            if idx >= 0: self.joint_combo.setCurrentIndex(idx)
            self.joint_combo.blockSignals(False)

    def on_joint_changed(self):
        # Clear live data buffer when switching joints to avoid misleading old data
        for key in self.live_data:
            self.live_data[key] = []
        if hasattr(self, '_start_time'):
            delattr(self, '_start_time')
        
        # Clear plots immediately
        for ax in self.axes:
            ax.clear()
        self.canvas.draw()
        
        # If we have CSV data, refresh it for the new joint
        if self.csv_data is not None:
            self.refresh_plots()

    def manual_load(self):
        p, _ = QFileDialog.getOpenFileName(self, "Open CSV", "", "CSV (*.csv)")
        if p: self.load_csv(p)

    def resume_live(self):
        self.csv_data = None
        # self.live_btn.setVisible(False) # Removed as it's now handled by RecordWidget
        # Reset live data to start fresh
        for key in self.live_data:
            self.live_data[key] = []
        if hasattr(self, '_start_time'):
            delattr(self, '_start_time')
        # Clear plots to avoid ghosts
        for ax in self.axes:
            ax.clear()
        self.canvas.draw()
        
        # Update button style in the record box
        main_win = self.window()
        if hasattr(main_win, 'rec_box'):
            main_win.rec_box.update_live_btn_style()

    def load_csv(self, path):
        try:
            import pandas as pd
            self.csv_data = pd.read_csv(path)
            # self.live_btn.setVisible(True) # Removed
            self.refresh_plots()
        except ImportError:
            # Fallback to csv module if pandas not available
            self.csv_data = []
            with open(path, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    self.csv_data.append({k: float(v) for k, v in row.items()})
            # self.live_btn.setVisible(True) # Removed
            self.refresh_plots()
        except Exception as e:
            print(f"Error loading CSV: {e}")
        
        # Update button style in the record box
        main_win = self.window()
        if hasattr(main_win, 'rec_box'):
            main_win.rec_box.update_live_btn_style()

    def refresh_plots(self):
        if self.csv_data is None: return
        
        joint_name = self.joint_combo.currentText()
        if not joint_name: return
        
        # Get data columns using the new header format: {joint_name}_Pos_Fbk, etc.
        t = []
        p_fbk, p_cmd, v_fbk, v_cmd, t_fbk, t_cmd = [], [], [], [], [], []
        
        try:
            if hasattr(self.csv_data, 'iloc'): # Pandas
                t = self.csv_data['Timestamp'].values
                p_fbk = self.csv_data[f"{joint_name}_Pos_Fbk"].values
                v_fbk = self.csv_data[f"{joint_name}_Vel_Fbk"].values
                t_fbk = self.csv_data[f"{joint_name}_Tor_Fbk"].values
                p_cmd = self.csv_data[f"{joint_name}_Pos_Cmd"].values
                v_cmd = self.csv_data[f"{joint_name}_Vel_Cmd"].values
                t_cmd = self.csv_data[f"{joint_name}_Tor_Cmd"].values
            else: # List of dicts
                t = [r['Timestamp'] for r in self.csv_data]
                p_fbk = [r[f'{joint_name}_Pos_Fbk'] for r in self.csv_data]
                v_fbk = [r[f'{joint_name}_Vel_Fbk'] for r in self.csv_data]
                t_fbk = [r[f'{joint_name}_Tor_Fbk'] for r in self.csv_data]
                p_cmd = [r[f'{joint_name}_Pos_Cmd'] for r in self.csv_data]
                v_cmd = [r[f'{joint_name}_Vel_Cmd'] for r in self.csv_data]
                t_cmd = [r[f'{joint_name}_Tor_Cmd'] for r in self.csv_data]
        except (KeyError, ValueError):
            # Fallback to old format: Pos_Fbk_i, etc.
            # Find index of this joint in the current joint list
            found_idx = -1
            for i, name in enumerate(self.ds.joint_names):
                if name == joint_name:
                    found_idx = i
                    break
            if found_idx == -1: return
            
            try:
                if hasattr(self.csv_data, 'iloc'):
                    t = self.csv_data['Timestamp'].values
                    p_fbk = self.csv_data[f"Pos_Fbk_{found_idx}"].values
                    v_fbk = self.csv_data[f"Vel_Fbk_{found_idx}"].values
                    t_fbk = self.csv_data[f"Tor_Fbk_{found_idx}"].values
                    p_cmd = self.csv_data[f"Pos_Cmd_{found_idx}"].values
                    v_cmd = self.csv_data[f"Vel_Cmd_{found_idx}"].values
                    t_cmd = self.csv_data[f"Tor_Cmd_{found_idx}"].values
                else:
                    t = [float(r['Timestamp']) for r in self.csv_data]
                    p_fbk = [float(r[f'Pos_Fbk_{found_idx}']) for r in self.csv_data]
                    v_fbk = [float(r[f'Vel_Fbk_{found_idx}']) for r in self.csv_data]
                    t_fbk = [float(r[f'Tor_Fbk_{found_idx}']) for r in self.csv_data]
                    p_cmd = [float(r[f'Pos_Cmd_{found_idx}']) for r in self.csv_data]
                    v_cmd = [float(r[f'Vel_Cmd_{found_idx}']) for r in self.csv_data]
                    t_cmd = [float(r[f'Tor_Cmd_{found_idx}']) for r in self.csv_data]
            except Exception as e:
                print(f"Plotting Error: {e}")
                return

        import numpy as np
        p_err = np.array(p_cmd) - np.array(p_fbk)

        # 1. Pos Cmd vs Fbk
        ax0 = self.axes[0]
        ax0.clear()
        ax0.plot(t, p_fbk, 'b-', label='Feedback')
        ax0.plot(t, p_cmd, 'r--', label='Command')
        ax0.set_title("Position (rad)")
        ax0.legend(); ax0.grid(True)

        # 2. Pos Error
        ax1 = self.axes[1]
        ax1.clear()
        ax1.plot(t, p_err, 'g-', label='Position Error')
        ax1.set_title("Position Error (rad)")
        ax1.legend(); ax1.grid(True)

        # 3. Vel Cmd vs Fbk
        ax2 = self.axes[2]
        ax2.clear()
        ax2.plot(t, v_fbk, 'g-', label='Feedback') # Green for vel feedback
        ax2.plot(t, v_cmd, 'r--', label='Command')
        ax2.set_title("Velocity (rad/s)")
        ax2.legend(); ax2.grid(True)

        # 4. Torque Cmd vs Fbk
        ax3 = self.axes[3]
        ax3.clear()
        ax3.plot(t, t_fbk, 'm-', label='Feedback') # Magenta for torque feedback
        ax3.plot(t, t_cmd, 'r--', label='Command')
        ax3.set_title("Torque (Nm)")
        ax3.legend(); ax3.grid(True)

        for ax in self.axes:
            y_min, y_max = ax.get_ylim()
            if y_max - y_min < 0.01:
                center = (y_min + y_max) / 2
                ax.set_ylim(center - 0.005, center + 0.005)

        self.canvas.draw()

# ==============================================================================
# Main Application
# ==============================================================================

class MainWindow(QMainWindow):
    def __init__(self, use_sim=False):
        super().__init__()
        self.setWindowTitle("Joint Analyzer")
        self.resize(1600, 900) # Set a reasonable default size for windowed mode
        self.showMaximized()
        
        self.ds = DataService(use_sim)
        
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        
        self.splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(self.splitter)
        
        # Left Side: Control Only
        self.ctrl_box = ControlWidget(self.ds)
        self.ctrl_box.setMinimumWidth(300)
        
        # Right Side: Record & Plot
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)
        self.rec_box = RecordWidget(self.ds)
        self.plot_box = PlotWidget(self.ds)
        right_layout.addWidget(self.rec_box)
        right_layout.addWidget(self.plot_box)
        right_layout.setStretch(0, 0) 
        right_layout.setStretch(1, 100) # Give high stretch priority to plot area
        
        self.splitter.addWidget(self.ctrl_box)
        self.splitter.addWidget(right_widget)
        
        # Adjust initial sizes: left panel wider (e.g., 400px), right panel flexible
        self.splitter.setSizes([450, 1000])
        self.splitter.setStretchFactor(0, 0)
        self.splitter.setStretchFactor(1, 1)
        self.splitter.setHandleWidth(2)
        
        self.ctrl_box.setMinimumWidth(350)
        
        self.statusBar().showMessage("Ready")
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_status_bar)
        self.status_timer.start(500)

    def update_status_bar(self):
        # Robot global words
        sw = self.ds.robot_status_word
        cw = self.ds.robot_control_word
        
        sw_str = interpret_status_word(sw)
        cw_str = interpret_control_word(cw)
        
        shm_status = "OK" if self.ds.fbk_ptr else "SHM_FAIL"
        self.statusBar().showMessage(
            f"SHM: {shm_status} | Status: {sw_str} | Control: {cw_str} | "
            f"Joints: {len(self.ds.joint_names)}"
        )

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--sim', action='store_true', help='Simulation mode')
    args = parser.parse_args()
    
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    
    font = QFont("Segoe UI", 9)
    app.setFont(font)
    
    window = MainWindow(use_sim=args.sim)
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
