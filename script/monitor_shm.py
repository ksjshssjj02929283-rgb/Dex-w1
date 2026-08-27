import ctypes
import time
import sys
import os
import curses

# Constants
FEEDBACK_SHM_KEY = 0x1006
CONTROL_SHM_KEY = 0x2006
MAX_JOINT_NUM = 32
OMNI_MOTOR_NUM = 4
BUFFER_CAPACITY = 8

# Load libc
libc = ctypes.CDLL("libc.so.6")
shmget = libc.shmget
shmget.restype = ctypes.c_int
shmget.argtypes = [ctypes.c_int, ctypes.c_size_t, ctypes.c_int]

shmat = libc.shmat
shmat.restype = ctypes.c_void_p
shmat.argtypes = [ctypes.c_int, ctypes.c_void_p, ctypes.c_int]

shmdt = libc.shmdt
shmdt.restype = ctypes.c_int
shmdt.argtypes = [ctypes.c_void_p]
"""Python ctypes layout to match utils/shared_memory_def.h exactly."""


class ChassisFeedback(ctypes.Structure):
    _fields_ = [
        ("timestamp", ctypes.c_int64),
        ("left_velocity", ctypes.c_double),
        ("right_velocity", ctypes.c_double),
        ("chassis_motor_velocity", ctypes.c_double * OMNI_MOTOR_NUM),
        ("chassis_motor_direction", ctypes.c_double * OMNI_MOTOR_NUM),
        ("status_word", ctypes.c_uint16),
        ("error_code_left", ctypes.c_uint16),
        ("error_code_right", ctypes.c_uint16),
        ("left_current", ctypes.c_double),
        ("right_current", ctypes.c_double),
        ("driver_temperature", ctypes.c_double),
        ("left_temperature", ctypes.c_double),
        ("right_temperature", ctypes.c_double),
    ]


class Vector3f(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("z", ctypes.c_float),
    ]


class Quaternionf(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("z", ctypes.c_float),
        ("w", ctypes.c_float),
    ]


class ImuData(ctypes.Structure):
    _fields_ = [
        ("id", ctypes.c_uint32),
        ("orientation", Quaternionf),
        ("angular_velocity", Vector3f),
        ("linear_acceleration", Vector3f),
        ("mag", Vector3f),
        ("eul", Vector3f),
        ("pressure", ctypes.c_float),
        ("timestamp", ctypes.c_uint32),
    ]


class JointFeedback(ctypes.Structure):
    _fields_ = [
        ("joint_name", ctypes.c_char * 32),
        ("joint_status_word", ctypes.c_uint16),
        ("joint_error_code", ctypes.c_uint16),
        ("joint_position", ctypes.c_double),
        ("joint_velocity", ctypes.c_double),
        ("joint_torque", ctypes.c_double),
        ("joint_sensor_torque", ctypes.c_double),
        ("joint_temperature", ctypes.c_double),
    ]


class JointControl(ctypes.Structure):
    _fields_ = [
        ("joint_name", ctypes.c_char * 32),
        ("joint_mode", ctypes.c_uint16),
        ("joint_position", ctypes.c_double),
        ("joint_velocity", ctypes.c_double),
        ("joint_torque", ctypes.c_double),
        ("joint_torque_feedforward", ctypes.c_double),
        ("joint_Kp", ctypes.c_double),
        ("joint_Kd", ctypes.c_double),
        ("joint_positive_torque_limit", ctypes.c_double),
        ("joint_negative_torque_limit", ctypes.c_double),
    ]


class ChassisControl(ctypes.Structure):
    _fields_ = [
        ("cnt", ctypes.c_uint64),
        ("chassis_mode", ctypes.c_int),
        ("chassis_left_velocity", ctypes.c_double),
        ("chassis_right_velocity", ctypes.c_double),
        ("chassis_motor_velocity", ctypes.c_double * OMNI_MOTOR_NUM),
        ("chassis_motor_direction", ctypes.c_double * OMNI_MOTOR_NUM),
        ("chassis_estop", ctypes.c_int),
        ("chassis_neutral", ctypes.c_int),
    ]


class RobotFeedback(ctypes.Structure):
    _fields_ = [
        ("cnt", ctypes.c_uint64),
        ("timestamp", ctypes.c_uint64),
        ("robot_status_word", ctypes.c_uint16),
        ("al_status", ctypes.c_uint8 * MAX_JOINT_NUM),
        ("joint_feedback", JointFeedback * MAX_JOINT_NUM),
        ("chassis_feedback", ChassisFeedback),
        ("imu_data", ImuData),
    ]


class RobotControl(ctypes.Structure):
    _fields_ = [
        ("cnt", ctypes.c_uint64),
        ("timestamp", ctypes.c_uint64),
        ("robot_control_word", ctypes.c_uint16),
        ("joint_control", JointControl * MAX_JOINT_NUM),
        ("chassis_control", ChassisControl),
    ]


class timespec(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_long), ("tv_nsec", ctypes.c_long)]


class QueueElementFbk(ctypes.Structure):
    _fields_ = [
        ("data", RobotFeedback),
        ("cnt", ctypes.c_uint64),
        ("ts", timespec),
        ("valid", ctypes.c_bool),
    ]


class QueueElementCtrl(ctypes.Structure):
    _fields_ = [
        ("data", RobotControl),
        ("cnt", ctypes.c_uint64),
        ("ts", timespec),
        ("valid", ctypes.c_bool),
    ]


class FeedbackShmStruct(ctypes.Structure):
    _fields_ = [
        ("ring_buffer", QueueElementFbk * BUFFER_CAPACITY),
        ("latest_and_reader", ctypes.c_uint64),
    ]


class ControlShmStruct(ctypes.Structure):
    _fields_ = [
        ("ring_buffer", QueueElementCtrl * BUFFER_CAPACITY),
        ("latest_and_reader", ctypes.c_uint64),
    ]


def get_shm_ptr(key, struct_type):
    shmid = shmget(key, ctypes.sizeof(struct_type), 0o666)
    if shmid < 0:
        return None
    ptr = shmat(shmid, None, 0)
    if ptr == ctypes.c_void_p(-1).value:
        return None
    return ctypes.cast(ptr, ctypes.POINTER(struct_type))


def addstr_safe(stdscr, y, x, text, attr=0):
    try:
        max_y, max_x = stdscr.getmaxyx()
        if x < max_x and y < max_y:
            stdscr.addstr(y, x, str(text)[:max_x - x], attr)
    except curses.error:
        pass


def fmt_array(arr, length, fmt="{:.2f}"):
    return "[" + ", ".join(fmt.format(arr[i]) for i in range(length)) + "]"


def cstr_to_py(c_arr):
    raw = bytes(c_arr)
    return raw.split(b"\x00", 1)[0].decode("utf-8", errors="ignore")


def monitor(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(100)

    fbk_ptr = get_shm_ptr(FEEDBACK_SHM_KEY, FeedbackShmStruct)
    ctrl_ptr = get_shm_ptr(CONTROL_SHM_KEY, ControlShmStruct)

    if not fbk_ptr and not ctrl_ptr:
        addstr_safe(
            stdscr,
            0,
            0,
            ("Could not attach to shared memory "
             f"(feedback=0x{FEEDBACK_SHM_KEY:X}, "
             f"control=0x{CONTROL_SHM_KEY:X})."),
        )
        stdscr.refresh()
        time.sleep(2)
        return

    while True:
        stdscr.clear()
        row = 0

        # Feedback
        if fbk_ptr:
            latest_and_reader = fbk_ptr.contents.latest_and_reader
            latest = latest_and_reader >> 32
            idx = latest % BUFFER_CAPACITY
            elem = fbk_ptr.contents.ring_buffer[idx]
            data = elem.data

            addstr_safe(
                stdscr,
                row,
                0,
                f"=== Feedback (Latest: {latest}, Valid: {elem.valid}) ===",
                curses.A_BOLD,
            )
            row += 1
            addstr_safe(
                stdscr,
                row,
                0,
                f"Cnt: {data.cnt}  Timestamp: {data.timestamp}  StatusWord: 0x{data.robot_status_word:04X}",
            )
            row += 1
            addstr_safe(
                stdscr,
                row,
                0,
                f"Chassis: L_Vel={data.chassis_feedback.left_velocity:.2f} R_Vel={data.chassis_feedback.right_velocity:.2f}",
            )
            row += 1

            addstr_safe(
                stdscr,
                row,
                0,
                "Chassis motors vel: " +
                fmt_array(data.chassis_feedback.chassis_motor_velocity,
                          OMNI_MOTOR_NUM),
            )
            row += 1
            addstr_safe(
                stdscr,
                row,
                0,
                "Chassis motors dir: " +
                fmt_array(data.chassis_feedback.chassis_motor_direction,
                          OMNI_MOTOR_NUM),
            )
            row += 1
            addstr_safe(
                stdscr,
                row,
                0,
                f"Chassis status: 0x{data.chassis_feedback.status_word:04X} ErrL: 0x{data.chassis_feedback.error_code_left:04X} ErrR: 0x{data.chassis_feedback.error_code_right:04X}",
            )
            row += 1
            addstr_safe(
                stdscr,
                row,
                0,
                ("Chassis telemetry: "
                 f"I_L={data.chassis_feedback.left_current} "
                 f"I_R={data.chassis_feedback.right_current} "
                 f"T_drv={data.chassis_feedback.driver_temperature} "
                 f"T_L={data.chassis_feedback.left_temperature} "
                 f"T_R={data.chassis_feedback.right_temperature}"),
            )
            row += 1
            addstr_safe(
                stdscr,
                row,
                0,
                ("IMU: "
                 f"id=0x{data.imu_data.id:02X} "
                 f"linear_acceleration="
                 f"[{data.imu_data.linear_acceleration.x:.2f}, "
                 f"{data.imu_data.linear_acceleration.y:.2f}, "
                 f"{data.imu_data.linear_acceleration.z:.2f}] "
                 f"angular_velocity="
                 f"[{data.imu_data.angular_velocity.x:.2f}, "
                 f"{data.imu_data.angular_velocity.y:.2f}, "
                 f"{data.imu_data.angular_velocity.z:.2f}]"),
            )
            row += 1
            addstr_safe(
                stdscr,
                row,
                0,
                ("IMU: "
                 f"mag=[{data.imu_data.mag.x:.2f}, "
                 f"{data.imu_data.mag.y:.2f}, "
                 f"{data.imu_data.mag.z:.2f}] "
                 f"eul(RPY)=[{data.imu_data.eul.x:.2f}, "
                 f"{data.imu_data.eul.y:.2f}, "
                 f"{data.imu_data.eul.z:.2f}] "
                 f"pressure={data.imu_data.pressure:.2f} "
                 f"ts={data.imu_data.timestamp}"),
            )
            row += 1
            addstr_safe(
                stdscr,
                row,
                0,
                ("IMU orientation(xyzw): "
                 f"[{data.imu_data.orientation.x:.4f}, "
                 f"{data.imu_data.orientation.y:.4f}, "
                 f"{data.imu_data.orientation.z:.4f}, "
                 f"{data.imu_data.orientation.w:.4f}]"),
            )
            row += 1

            addstr_safe(
                stdscr, row, 0,
                "Joint  Name       Pos      Vel   Torque  SensorT  AL  Err   Temp"
            )
            row += 1
            for i in range(min(MAX_JOINT_NUM, 20)):
                joint_name = cstr_to_py(data.joint_feedback[i].joint_name)
                addstr_safe(
                    stdscr,
                    row,
                    0,
                    f"{i:2d}     {joint_name:10.10s} {data.joint_feedback[i].joint_position:6.3f}   {data.joint_feedback[i].joint_velocity:6.3f}   {data.joint_feedback[i].joint_torque:6.3f}  {data.joint_feedback[i].joint_sensor_torque:6.3f}  {data.al_status[i]:2d}  {data.joint_feedback[i].joint_error_code:04X}  {data.joint_feedback[i].joint_temperature:6.1f}",
                )
                row += 1
        else:
            addstr_safe(stdscr, row, 0, "Feedback SHM not found.")
            row += 1

        row += 1

        # Control
        if ctrl_ptr:
            latest_and_reader = ctrl_ptr.contents.latest_and_reader
            latest = latest_and_reader >> 32
            idx = latest % BUFFER_CAPACITY
            elem = ctrl_ptr.contents.ring_buffer[idx]
            data = elem.data

            addstr_safe(
                stdscr,
                row,
                0,
                f"=== Control (Latest: {latest}, Valid: {elem.valid}) ===",
                curses.A_BOLD,
            )
            row += 1
            addstr_safe(
                stdscr,
                row,
                0,
                f"Cnt: {data.cnt}  Timestamp: {data.timestamp}  CtrlWord: 0x{data.robot_control_word:04X}  ChassisMode: {data.chassis_control.chassis_mode}",
            )
            row += 1
            addstr_safe(
                stdscr,
                row,
                0,
                f"Chassis: L_Vel={data.chassis_control.chassis_left_velocity:.2f} R_Vel={data.chassis_control.chassis_right_velocity:.2f}",
            )
            row += 1

            addstr_safe(
                stdscr,
                row,
                0,
                "Chassis motors vel: " +
                fmt_array(data.chassis_control.chassis_motor_velocity,
                          OMNI_MOTOR_NUM),
            )
            row += 1
            addstr_safe(
                stdscr,
                row,
                0,
                "Chassis motors dir: " +
                fmt_array(data.chassis_control.chassis_motor_direction,
                          OMNI_MOTOR_NUM),
            )
            row += 1
            addstr_safe(
                stdscr,
                row,
                0,
                f"Chassis estop: {data.chassis_control.chassis_estop} neutral: {data.chassis_control.chassis_neutral} cnt: {data.chassis_control.cnt}",
            )
            row += 1

            addstr_safe(stdscr, row, 0,
                        "Joint  Name       Pos      Vel      TorqueFF")
            row += 1
            for i in range(min(MAX_JOINT_NUM, 20)):
                joint_name = cstr_to_py(data.joint_control[i].joint_name)
                addstr_safe(
                    stdscr,
                    row,
                    0,
                    f"{i:2d}     {joint_name:10.10s} {data.joint_control[i].joint_position:6.3f}   {data.joint_control[i].joint_velocity:6.3f}   {data.joint_control[i].joint_torque_feedforward:6.3f}",
                )
                row += 1
        else:
            addstr_safe(stdscr, row, 0, "Control SHM not found.")
            row += 1

        stdscr.refresh()

        c = stdscr.getch()
        if c == ord('q'):
            break


if __name__ == "__main__":
    curses.wrapper(monitor)