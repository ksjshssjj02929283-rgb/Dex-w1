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
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout, QGridLayout, QPushButton, QLabel, QLineEdit, QFileDialog,
    QGroupBox, QDoubleSpinBox, QSplitter, QComboBox,
    QScrollArea, QProgressBar
)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QObject, QSize, QRect, QPoint
from PyQt5.QtGui import QColor, QFont, QPainter, QBrush, QPen

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
    from robot_server_interfaces.action import MoveRobot
    from rclpy.action import ActionClient
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

# ==============================================================================
# Shared Memory Definitions (Sync with record.py / shared_memory_def.h)
# ==============================================================================
FEEDBACK_SHM_KEY = 0x1006
CONTROL_SHM_KEY = 0x2006
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
    record_progress = pyqtSignal(int) # 0-100 percentage
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
            self.move_robot_client = ActionClient(self.node, MoveRobot, '/control/move_robot')
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

    def send_move_action(self, motion_group, joint_type, target, speed=0.1, acceleration=0.1):
        if not ROS2_AVAILABLE: return
        goal = MoveRobot.Goal()
        goal.motion_group = motion_group
        goal.type = joint_type
        goal.target = [float(v) for v in target]
        goal.config = f'{{"speed":{speed},"acceleration":{acceleration}}}'
        self.move_robot_client.send_goal_async(goal)

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
                    progress = int((now - start_time) / duration * 100)
                    self.record_progress.emit(min(100, progress))
                    
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
            # Short click: Use step size from control widget
            step = self.parent_ctrl.step_spin.value()
            self.parent_ctrl.increment_cmd(self.joint_name, step * self.direction)
        
        self.long_press_timer.stop()
        self.pulse_timer.stop()
        self.is_long_press = False

    def start_continuous(self):
        self.is_long_press = True
        self.pulse_timer.start(10) # 100Hz

    def pulse_move(self):
        # Continuous move: Proportional to manual step but capped for safety
        # Cap at 0.005 rad per pulse (at 100Hz, this is 0.5 rad/s)
        step = self.parent_ctrl.step_spin.value()
        continuous_step = min(step, 0.005)
        self.parent_ctrl.increment_cmd(self.joint_name, continuous_step * self.direction)

class SwitchButton(QPushButton):
    def __init__(self, parent=None, active_color="#c8e6c9", bg_color="#eeeeee"):
        super().__init__(parent)
        self.setCheckable(True)
        self.setFixedSize(50, 22) # Reduced width and height
        self._active_color = QColor(active_color)
        self._bg_color = QColor(bg_color)
        self._circle_color = QColor("white")
        self._border_color = QColor("#cccccc")

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        
        # Draw track
        bg = self._active_color if self.isChecked() else self._bg_color
        p.setBrush(QBrush(bg))
        p.setPen(QPen(self._border_color, 1))
        p.drawRoundedRect(1, 1, self.width()-2, self.height()-2, (self.height()-2)/2, (self.height()-2)/2)
        
        # Draw thumb
        p.setBrush(QBrush(self._circle_color))
        p.setPen(QPen(self._border_color, 0.5))
        r = self.height() - 6
        x = self.width() - r - 3 if self.isChecked() else 3
        p.drawEllipse(x, 3, r, r)
        
        # Draw text (Optional, centered in remaining space)
        p.setPen(QPen(QColor("#ffffff") if self.isChecked() else QColor("#666666")))
        font = self.font()
        font.setBold(True)
        font.setPointSize(6)
        p.setFont(font)
        text = "ON" if self.isChecked() else "OFF"
        
        if self.isChecked():
            p.drawText(QRect(4, 0, 28, self.height()), Qt.AlignCenter, text)
        else:
            p.drawText(QRect(self.width()-32, 0, 28, self.height()), Qt.AlignCenter, text)

class ActionWidget(QWidget):
    PRESET_POSITIONS = {
        'WholeBody': {
            'Home': [0.0] * 20,
            'Standby': [0.6, -1.2, 0.6, 0.0, 0.0, 0.0, 0.0,
                        -1.4, 0.0, -0.6, 0.0, -0.5, -0.5,
                        0.0, 1.4, 0.0, 0.6, 0.0, 0.5, 0.5],
        },
        'Left': {
            'Home': [0.0] * 7,
            'Standby': [0.0, -1.4, 0.0, -0.6, 0.0, -0.5, -0.5],
        },
        'Right': {
            'Home': [0.0] * 7,
            'Standby': [0.0, 1.4, 0.0, 0.6, 0.0, 0.5, 0.5],
        },
    }

    def __init__(self, ds):
        super().__init__()
        self.ds = ds
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        group = QGroupBox("Quick Actions")
        group_layout = QVBoxLayout(group)
        group_layout.setContentsMargins(10, 8, 10, 8)
        group_layout.setSpacing(6)

        grid = QGridLayout()
        grid.setSpacing(4)
        grid.setHorizontalSpacing(6)

        # Header
        for col, text in enumerate(["", "Home", "Standby"]):
            lbl = QLabel(f"<b>{text}</b>")
            lbl.setAlignment(Qt.AlignCenter)
            grid.addWidget(lbl, 0, col)

        # Preset buttons
        row_labels = [("WholeBody", "Whole Body"), ("Left", "Left Arm"), ("Right", "Right Arm")]
        for row, (key, label) in enumerate(row_labels, start=1):
            grid.addWidget(QLabel(f"<b>{label}</b>"), row, 0)
            for col, preset in enumerate(["Home", "Standby"], start=1):
                btn = QPushButton(preset)
                btn.setFixedHeight(28)
                btn.clicked.connect(lambda checked, k=key, p=preset: self._send_preset(k, p))
                grid.addWidget(btn, row, col)

        group_layout.addLayout(grid)

        # Stop button
        self.stop_btn = QPushButton("Stop Motion")
        self.stop_btn.setFixedHeight(32)
        self.stop_btn.setStyleSheet(
            "QPushButton { background-color: #ffcdd2; font-weight: bold; }"
            "QPushButton:hover { background-color: #ef9a9a; }"
        )
        self.stop_btn.clicked.connect(self._stop_motion)
        group_layout.addWidget(self.stop_btn)

        layout.addWidget(group)

    def _send_preset(self, group_key, preset_name):
        group_map = {'WholeBody': 'WholeBody', 'Left': 'Primary', 'Right': 'Secondary'}
        positions = self.PRESET_POSITIONS[group_key][preset_name]
        self.ds.send_move_action(group_map[group_key], 'MoveJoint', positions)

    def _stop_motion(self):
        self.ds.send_move_action('Arm', 'StopMove', [])

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
        self.joint_layout.setSpacing(0)
        
        # Header Row
        header = QWidget()
        h_layout = QHBoxLayout(header)
        h_layout.setContentsMargins(5, 5, 5, 5)
        
        joint_lbl = QLabel("<b>Joint</b>")
        joint_lbl.setFixedWidth(120)
        joint_lbl.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        h_layout.addWidget(joint_lbl)
        
        h_layout.addStretch()
        
        pos_step_lbl = QLabel("<b>Position Step (rad)</b>")
        pos_step_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        h_layout.addWidget(pos_step_lbl)
        
        # Step Size Control - No extra spacing, tightly coupled to label
        self.step_spin = QDoubleSpinBox()
        self.step_spin.setRange(0.0001, 1.0)
        self.step_spin.setDecimals(4)
        self.step_spin.setValue(0.001)
        self.step_spin.setSingleStep(0.001)
        self.step_spin.setFixedWidth(70)
        self.step_spin.setToolTip("Click step size (rad)")
        self.step_spin.valueChanged.connect(self._update_spin_step)
        # Initialize the singleStep based on the initial value (0.001)
        self._update_spin_step(0.001)
        h_layout.addWidget(self.step_spin)
        
        self.joint_layout.addWidget(header)
        
        self.scroll.setWidget(self.container)
        main_layout.addWidget(self.scroll)

        # Advanced Controls (Ruckig, Filter, ZVD, Drag)
        adv_grid = QGridLayout()
        adv_grid.setSpacing(5)
        adv_grid.setContentsMargins(5, 5, 5, 5)

        def create_toggle(prop, default_on=True):
            btn = SwitchButton()
            btn.setChecked(default_on)
            btn.clicked.connect(lambda checked: self.ds.set_property(prop, "1" if checked else "0"))
            return btn

        # Row 0: Ruckig and Filter
        adv_grid.addWidget(QLabel("Ruckig (Interpolation):"), 0, 0)
        self.ruckig_btn = create_toggle("EnableStreamingRuckig", True)
        adv_grid.addWidget(self.ruckig_btn, 0, 1)

        adv_grid.addWidget(QLabel("Butterworth Filter:"), 0, 2)
        self.filter_btn = create_toggle("EnableButterworthFilter", True)
        adv_grid.addWidget(self.filter_btn, 0, 3)

        # Row 1: ZVD and Drag Mode
        adv_grid.addWidget(QLabel("ZVD (Input Shaper):"), 1, 0)
        self.zvd_btn = create_toggle("EnableInputShaper", True)
        adv_grid.addWidget(self.zvd_btn, 1, 1)

        adv_grid.addWidget(QLabel("Drag Mode:"), 1, 2)
        self.drag_btn = create_toggle("EnableDrag", False)
        adv_grid.addWidget(self.drag_btn, 1, 3)

        main_layout.addLayout(adv_grid)

        # Preset Positions (Home / Standby) - inline, no header
        preset_grid = QGridLayout()
        preset_grid.setSpacing(4)
        preset_grid.setContentsMargins(5, 2, 5, 2)

        preset_positions = {
            'WholeBody': {
                'Home': [0.0] * 20,
                'Standby': [0.6, -1.2, 0.6, 0.0, 0.0, 0.0, 0.0,
                            -1.4, 0.0, -0.6, 0.0, -0.5, -0.5,
                            0.0, 1.4, 0.0, 0.6, 0.0, 0.5, 0.5],
            },
            'Left': {
                'Home': [0.0] * 7,
                'Standby': [0.0, -1.4, 0.0, -0.6, 0.0, -0.5, -0.5],
            },
            'Right': {
                'Home': [0.0] * 7,
                'Standby': [0.0, 1.4, 0.0, 0.6, 0.0, 0.5, 0.5],
            },
        }
        group_map = {'WholeBody': 'WholeBody', 'Left': 'Primary', 'Right': 'Secondary'}
        row_labels = [("WholeBody", "Whole Body"), ("Left", "Left Arm"), ("Right", "Right Arm")]

        for row, (key, label) in enumerate(row_labels):
            preset_grid.addWidget(QLabel(label), row, 0)
            for col, preset in enumerate(["Home", "Standby"], start=1):
                btn = QPushButton(preset)
                btn.setFixedHeight(28)
                positions = preset_positions[key][preset]
                mg = group_map[key]
                btn.clicked.connect(lambda checked, mg=mg, pos=positions: self.ds.send_move_action(mg, 'MoveJoint', pos))
                preset_grid.addWidget(btn, row, col)

        main_layout.addLayout(preset_grid)

        # Bottom row: Clear Error (left) + Stop Motion (right)
        bottom_layout = QHBoxLayout()
        self.clear_btn = QPushButton("Clear Error")
        self.clear_btn.setFixedHeight(32)
        self.clear_btn.setStyleSheet("")
        self.clear_btn.clicked.connect(self.ds.clear_errors)
        bottom_layout.addWidget(self.clear_btn)

        self.stop_btn = QPushButton("Stop Motion")
        self.stop_btn.setFixedHeight(32)
        self.stop_btn.setStyleSheet(
            "QPushButton { background-color: #ffcdd2; font-weight: bold; }"
            "QPushButton:hover { background-color: #ef9a9a; }"
        )
        self.stop_btn.clicked.connect(lambda: self.ds.send_move_action('Arm', 'StopMove', []))
        bottom_layout.addWidget(self.stop_btn)

        main_layout.addLayout(bottom_layout)
        
        self.ds.updated.connect(self.update_ui)
        self.cmd_positions = {} # name -> value
        self.joint_widgets = {} # name -> {'pos_label': QLabel, 'row_widget': QWidget}

    def _update_spin_step(self, val):
        import math
        if val <= 0: return
        # Find the smallest power of 10 that is <= val
        # To handle floating point precision issues (e.g. 0.0001 being 0.000099999),
        # we add a small epsilon before log10.
        mag = math.floor(math.log10(val + 1e-10))
        new_step = 10**mag
        
        # Only update if it actually changed to avoid redundant signals
        if abs(self.step_spin.singleStep() - new_step) > 1e-10:
            self.step_spin.setSingleStep(new_step)

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
                row_widget.setMinimumHeight(35)
                row_layout = QHBoxLayout(row_widget)
                row_layout.setContentsMargins(5, 1, 5, 1)
                
                name_label = QLabel(name)
                name_label.setFixedWidth(120)
                name_label.setStyleSheet("")
                
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

class RecordWidget(QWidget):
    def __init__(self, ds):
        super().__init__()
        self.ds = ds
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        group = QGroupBox("Data Acquisition")
        group_layout = QVBoxLayout(group)
        group_layout.setContentsMargins(10, 10, 10, 10)
        group_layout.setSpacing(8)
        
        # 1. Path section at the top
        path_layout = QVBoxLayout()
        path_layout.setSpacing(2)
        path_layout.addWidget(QLabel("Record Path:"))
        
        path_row = QHBoxLayout()
        # Path: script/../log/ folder
        script_dir = os.path.dirname(os.path.abspath(__file__))
        log_dir = os.path.join(os.path.dirname(script_dir), "log")
        if not os.path.exists(log_dir):
            try:
                os.makedirs(log_dir)
            except:
                log_dir = os.getcwd() # Fallback

        default_name = f"record_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.path_edit = QLineEdit(os.path.join(log_dir, default_name))
        path_btn = QPushButton("...")
        path_btn.setFixedWidth(35)
        path_btn.clicked.connect(self.browse)
        path_row.addWidget(self.path_edit)
        path_row.addWidget(path_btn)
        path_layout.addLayout(path_row)
        group_layout.addLayout(path_layout)
        
        # 2. Grid layout for trigger settings and Record button
        grid = QGridLayout()
        grid.setVerticalSpacing(10)
        grid.setHorizontalSpacing(10)
        
        # Row 0: Joint & Data
        self.trig_joint = QComboBox()
        self.trig_joint.setFixedWidth(150)
        grid.addWidget(QLabel("Trigger Joint:"), 0, 0, Qt.AlignLeft)
        grid.addWidget(self.trig_joint, 0, 1)
        
        self.trig_data = QComboBox()
        self.trig_data.addItems([
            "Position (rad)", "Position Error (rad)", "Velocity (rad/s)", "Torque (Nm)",
            "Target Position (rad)", "Target Velocity (rad/s)", "Target Torque (Nm)"
        ])
        # Set "Position Error (rad)" as default
        self.trig_data.setCurrentIndex(1)
        self.trig_data.setFixedWidth(150)
        grid.addWidget(QLabel("Trigger Data:"), 0, 2, Qt.AlignLeft)
        grid.addWidget(self.trig_data, 0, 3)
        
        # Row 1: Mode & Threshold
        self.trig_mode = QComboBox()
        self.trig_mode.addItems(["None", "Rising Edge", "Falling Edge", "Delta Trigger"])
        self.trig_mode.setFixedWidth(150)
        grid.addWidget(QLabel("Trigger Mode:"), 1, 0, Qt.AlignLeft)
        grid.addWidget(self.trig_mode, 1, 1)
        
        self.trig_threshold = QDoubleSpinBox()
        self.trig_threshold.setRange(-1000, 1000); self.trig_threshold.setValue(0.01); self.trig_threshold.setDecimals(3)
        self.trig_threshold.setSingleStep(0.01)
        self.trig_threshold.setFixedWidth(150)
        grid.addWidget(QLabel("Threshold:"), 1, 2, Qt.AlignLeft)
        grid.addWidget(self.trig_threshold, 1, 3)

        # Row 2: Duration & Record button
        self.dur_spin = QDoubleSpinBox()
        self.dur_spin.setRange(1, 3600); self.dur_spin.setValue(8.0); self.dur_spin.setSuffix("s")
        self.dur_spin.setFixedWidth(150)
        grid.addWidget(QLabel("Duration:"), 2, 0, Qt.AlignLeft)
        grid.addWidget(self.dur_spin, 2, 1)
        
        self.rec_btn = QPushButton("Record")
        self.rec_btn.setFixedHeight(28)
        self.rec_btn.setStyleSheet("background-color: #c8e6c9;")
        self.rec_btn.clicked.connect(self.toggle_rec)
        # Span both label and widget columns (e.g., columns 2 and 3)
        grid.addWidget(self.rec_btn, 2, 2, 1, 2)

        group_layout.addLayout(grid)
        
        # Status Label (Hidden)
        self.status = QLabel("Ready")
        self.status.setVisible(False)
        group_layout.addWidget(self.status)
        
        # Progress Bar at the bottom
        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        self.progress.setTextVisible(True)
        self.progress.setFixedHeight(15)
        self.progress.setStyleSheet("""
            QProgressBar {
                border: 1px solid #ddd;
                border-radius: 2px;
                background-color: #f5f5f5;
                text-align: center;
            }
            QProgressBar::chunk {
                background-color: #81c784;
            }
        """)
        group_layout.addWidget(self.progress)
        
        layout.addWidget(group)
        self.ds.record_status.connect(self.on_status)
        self.ds.record_progress.connect(self.progress.setValue)
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
            self.rec_btn.setText("Stop")
            self.rec_btn.setStyleSheet("background-color: #ffcdd2;")
        else:
            self.rec_btn.setText("Record")
            self.rec_btn.setStyleSheet("background-color: #c8e6c9;")
            self.progress.setValue(0) # Reset progress on stop/finish

class PlotWidget(QWidget):
    def __init__(self, ds):
        super().__init__()
        self.ds = ds
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 2, 5, 2)
        layout.setSpacing(2)
        
        top_bar = QHBoxLayout()
        top_bar.setContentsMargins(0, 0, 0, 0)
        top_bar.setSpacing(10)
        top_bar.addWidget(QLabel("Select Joint:"))
        self.joint_combo = QComboBox()
        self.joint_combo.setMinimumWidth(200) # Wider combo box
        self.joint_combo.setFixedHeight(24)
        self.joint_combo.currentIndexChanged.connect(self.on_joint_changed)
        top_bar.addWidget(self.joint_combo)
        
        self.load_btn = QPushButton("Load CSV")
        self.load_btn.setFixedHeight(24)
        self.load_btn.clicked.connect(self.manual_load)
        top_bar.addWidget(self.load_btn)

        self.file_path_label = QLabel("")
        self.file_path_label.setStyleSheet("color: #666; font-style: italic; margin-left: 5px;")
        top_bar.addWidget(self.file_path_label)
        
        top_bar.addStretch()
        layout.addLayout(top_bar)
        layout.addSpacing(5) # Reduced gap
        
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
        # Reduce vertical distance between subplots
        self.canvas.figure.tight_layout(pad=1.5, h_pad=1.0)
        
        # More compact toolbar
        self.toolbar = NavigationToolbar(self.canvas, self)
        self.toolbar.setIconSize(QSize(20, 20))
        self.toolbar.setFixedHeight(30) # Default is usually larger
        self.plot_layout.setSpacing(1)
        self.plot_layout.setContentsMargins(0, 0, 0, 0)
        
        self.plot_layout.addWidget(self.toolbar)
        self.plot_layout.addWidget(self.canvas)
        
        # Reduced minimum height to 600 for better initial fit
        self.canvas.setMinimumHeight(600)
        
        self.csv_data = None
        self.ds.recording_done.connect(self.load_csv)
        self.ds.updated.connect(self.update_joint_combo)

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
        # Clear plots immediately
        for ax in self.axes:
            ax.clear()
        self.canvas.draw()
        
        # If we have CSV data, refresh it for the new joint
        if self.csv_data is not None:
            self.refresh_plots()

    def manual_load(self):
        # Default to the log folder (script/../log)
        script_dir = os.path.dirname(os.path.abspath(__file__))
        log_dir = os.path.join(os.path.dirname(script_dir), "log")
        if not os.path.exists(log_dir):
            log_dir = "" # Fallback to current dir if log folder doesn't exist
            
        p, _ = QFileDialog.getOpenFileName(self, "Open CSV", log_dir, "CSV (*.csv)")
        if p: self.load_csv(p)

    def load_csv(self, path):
        try:
            # Update path label (showing only filename for space)
            filename = os.path.basename(path)
            self.file_path_label.setText(f"{filename}")
            
            import pandas as pd
            self.csv_data = pd.read_csv(path)
            self.refresh_plots()
        except ImportError:
            self.csv_data = []
            with open(path, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    self.csv_data.append({k: float(v) for k, v in row.items()})
            self.refresh_plots()
        except Exception as e:
            print(f"Error loading CSV: {e}")

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
        ax0.plot(t, p_fbk, 'b-', label='Feedback', linewidth=0.8)
        ax0.plot(t, p_cmd, 'r--', label='Command', linewidth=0.8)
        ax0.set_ylabel("Position (rad)", fontweight='bold', fontsize=10)
        ax0.legend(); ax0.grid(True)

        # 2. Pos Error
        ax1 = self.axes[1]
        ax1.clear()
        ax1.plot(t, p_err, 'g-', label='Position Error', linewidth=0.8)
        ax1.set_ylabel("Pos Error (rad)", fontweight='bold', fontsize=10)
        ax1.legend(); ax1.grid(True)

        # 3. Vel Cmd vs Fbk
        ax2 = self.axes[2]
        ax2.clear()
        ax2.plot(t, v_fbk, 'g-', label='Feedback', linewidth=0.8) # Green for vel feedback
        ax2.plot(t, v_cmd, 'r--', label='Command', linewidth=0.8)
        ax2.set_ylabel("Velocity (rad/s)", fontweight='bold', fontsize=10)
        ax2.legend(); ax2.grid(True)

        # 4. Torque Cmd vs Fbk
        ax3 = self.axes[3]
        ax3.clear()
        ax3.plot(t, t_fbk, 'm-', label='Feedback', linewidth=0.8) # Magenta for torque feedback
        ax3.plot(t, t_cmd, 'r--', label='Command', linewidth=0.8)
        ax3.set_ylabel("Torque (Nm)", fontweight='bold', fontsize=10)
        ax3.set_xlabel("Time (s)")
        ax3.legend(); ax3.grid(True)

        for ax in self.axes:
            y_min, y_max = ax.get_ylim()
            if y_max - y_min < 0.01:
                center = (y_min + y_max) / 2
                ax.set_ylim(center - 0.005, center + 0.005)
            
            # Ensure X-axis labels show 3 decimal places (1ms precision)
            ax.xaxis.set_major_formatter(plt.FormatStrFormatter('%.3f'))
            # Show high precision coordinates in the toolbar status line
            ax.format_coord = lambda x, y: f't={x:.3f}s, val={y:.6f}'

        self.canvas.figure.tight_layout(pad=1.5, h_pad=1.0)
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
        
        # Left Side: Control & Record
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 0, 0)
        
        self.ctrl_box = ControlWidget(self.ds)
        self.rec_box = RecordWidget(self.ds)

        left_layout.addWidget(self.rec_box)
        left_layout.addWidget(self.ctrl_box)
        left_layout.setStretch(0, 0) # Record fixed at top
        left_layout.setStretch(1, 1) # Control takes remaining space
        
        # Right Side: Plot area (Full height)
        self.plot_box = PlotWidget(self.ds)
        
        self.splitter.addWidget(left_widget)
        self.splitter.addWidget(self.plot_box)
        
        # Initial proportions
        self.splitter.setSizes([450, 1150])
        self.splitter.setStretchFactor(0, 0)
        self.splitter.setStretchFactor(1, 1)
        self.splitter.setHandleWidth(2)
        
        left_widget.setMinimumWidth(380)
        
        # Prevent focus jumping to CSV path on startup
        self.setFocus()
        
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
