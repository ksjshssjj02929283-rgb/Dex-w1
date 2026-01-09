# 使用前安装环境 sudo apt install -y libspnav-dev libx11-dev spacenavd
import argparse
import ctypes
import ctypes.util
import rclpy
import sys
import threading
from rclpy.executors import MultiThreadedExecutor
from dexe_robot_client.robot_client import RobotClientRos2 as RobotClient
from dexe_robot_client.robot_client import MotionGroup, ReferenceFrame
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3, Twist, TwistStamped
import numpy as np
import time
from end_effector_interfaces.msg import EEJointControl

# Load libspnav
libspnav_path = ctypes.util.find_library('spnav')
if not libspnav_path:
    raise ImportError('libspnav.so not found')
libspnav = ctypes.CDLL(libspnav_path)

class SpnavEventMotion(ctypes.Structure):
    _fields_ = [
        ('type', ctypes.c_int),
        ('x', ctypes.c_int),
        ('y', ctypes.c_int),
        ('z', ctypes.c_int),
        ('rx', ctypes.c_int),
        ('ry', ctypes.c_int),
        ('rz', ctypes.c_int),
        ('period', ctypes.c_uint),
        ('data', ctypes.c_void_p)
    ]


class SpnavEventButton(ctypes.Structure):
    _fields_ = [
        ('type', ctypes.c_int),
        ('press', ctypes.c_int),
        ('bnum', ctypes.c_int)
    ]


class SpnavEvent(ctypes.Union):
    _fields_ = [
        ('type', ctypes.c_int),
        ('motion', SpnavEventMotion),
        ('button', SpnavEventButton)
    ]


SPNAV_EVENT_ANY = 0
SPNAV_EVENT_MOTION = 1
SPNAV_EVENT_BUTTON = 2

libspnav.spnav_open.restype = ctypes.c_int
libspnav.spnav_close.restype = ctypes.c_int
libspnav.spnav_poll_event.argtypes = [ctypes.POINTER(SpnavEvent)]
libspnav.spnav_poll_event.restype = ctypes.c_int


class SpacenavNode(Node):
    def __init__(self):
        super().__init__('spacenav')

        # Parameters (defaults match C++ node)
        self.declare_parameter('full_scale', 512.0)
        self.declare_parameter('angular_scale/x', 1.0)
        self.declare_parameter('angular_scale/y', 1.0)
        self.declare_parameter('angular_scale/z', 1.0)
        self.declare_parameter('linear_scale/x', 1.0)
        self.declare_parameter('linear_scale/y', 1.0)
        self.declare_parameter('linear_scale/z', 1.0)
        self.declare_parameter('static_count_threshold', 30)
        self.declare_parameter('zero_when_static', True)
        self.declare_parameter('static_trans_deadband', 0.1)
        self.declare_parameter('static_rot_deadband', 0.1)
        self.declare_parameter('use_twist_stamped', False)
        self.declare_parameter('apply_calibration', True)
        self.declare_parameter('calibration_max', 0.68359375)

        # Publishers
        self.pub_offset = self.create_publisher(Vector3, 'spacenav/offset', 10)
        self.pub_rot_offset = self.create_publisher(Vector3, 'spacenav/rot_offset', 10)
        self.use_twist_stamped = self.get_parameter('use_twist_stamped').value
        if self.use_twist_stamped:
            self.pub_twist_stamped = self.create_publisher(TwistStamped, 'spacenav/twist_stamped', 10)
        else:
            self.pub_twist = self.create_publisher(Twist, 'spacenav/twist', 10)
        self.pub_joy = self.create_publisher(Joy, 'spacenav/joy', 10)

        # State
        self.full_scale = self.get_parameter('full_scale').value
        self.linear_scale = [
            self.get_parameter('linear_scale/x').value,
            self.get_parameter('linear_scale/y').value,
            self.get_parameter('linear_scale/z').value,
        ]
        self.angular_scale = [
            self.get_parameter('angular_scale/x').value,
            self.get_parameter('angular_scale/y').value,
            self.get_parameter('angular_scale/z').value,
        ]
        self.static_count_threshold = self.get_parameter('static_count_threshold').value
        self.zero_when_static = self.get_parameter('zero_when_static').value
        self.static_trans_deadband = self.get_parameter('static_trans_deadband').value
        self.static_rot_deadband = self.get_parameter('static_rot_deadband').value

        # Calibration: map max absolute value to 1.0 (default 0.68359375)
        self.apply_calibration = self.get_parameter('apply_calibration').value
        self.calibration_max = self.get_parameter('calibration_max').value

        # dynamic state
        self.normed_vx = self.normed_vy = self.normed_vz = 0.0
        self.normed_wx = self.normed_wy = self.normed_wz = 0.0
        self.joystick_buttons = [0, 0]
        self.no_motion_count = 0
        self.motion_stale = False
        self.joy_stale = False

        # Open device
        ret = libspnav.spnav_open()
        if ret == -1:
            self.get_logger().error('Could not open the space navigator device. Did you remember to run spacenavd (as root)?')
            raise RuntimeError('spnav_open failed')
        self.get_logger().info('spnav_open succeeded')

        # Timer: poll frequently (1ms in C++ node). Use 1ms if possible, fallback to 10ms.
        self.timer = self.create_timer(0.001, self.poll_spacenav)

    def poll_spacenav(self):
        # refresh parameters (allow dynamic changes)
        self.full_scale = self.get_parameter('full_scale').value
        self.linear_scale = [
            self.get_parameter('linear_scale/x').value,
            self.get_parameter('linear_scale/y').value,
            self.get_parameter('linear_scale/z').value,
        ]
        self.angular_scale = [
            self.get_parameter('angular_scale/x').value,
            self.get_parameter('angular_scale/y').value,
            self.get_parameter('angular_scale/z').value,
        ]
        self.static_count_threshold = self.get_parameter('static_count_threshold').value
        self.zero_when_static = self.get_parameter('zero_when_static').value
        self.static_trans_deadband = self.get_parameter('static_trans_deadband').value
        self.static_rot_deadband = self.get_parameter('static_rot_deadband').value
        # refresh calibration params
        self.apply_calibration = self.get_parameter('apply_calibration').value
        self.calibration_max = self.get_parameter('calibration_max').value
        self.use_twist_stamped = self.get_parameter('use_twist_stamped').value

        queue_empty = False
        sev = SpnavEvent()
        # process all pending events
        while True:
            ret = libspnav.spnav_poll_event(ctypes.byref(sev))
            if ret == 0:
                queue_empty = True
                # handle static detection similar to C++
                self.no_motion_count += 1
                if self.no_motion_count > self.static_count_threshold:
                    if self.zero_when_static or (
                        abs(self.normed_vx) < self.static_trans_deadband and
                        abs(self.normed_vy) < self.static_trans_deadband and
                        abs(self.normed_vz) < self.static_trans_deadband
                    ):
                        self.normed_vx = self.normed_vy = self.normed_vz = 0.0

                    if self.zero_when_static or (
                        abs(self.normed_wx) < self.static_rot_deadband and
                        abs(self.normed_wy) < self.static_rot_deadband and
                        abs(self.normed_wz) < self.static_rot_deadband
                    ):
                        self.normed_wx = self.normed_wy = self.normed_wz = 0.0

                    self.no_motion_count = 0
                    self.motion_stale = True
                break

            if ret == SPNAV_EVENT_MOTION:
                # note: mapping follows C++ implementation
                self.normed_vx = sev.motion.z / float(self.full_scale)
                self.normed_vy = -sev.motion.x / float(self.full_scale)
                self.normed_vz = sev.motion.y / float(self.full_scale)

                self.normed_wx = sev.motion.rz / float(self.full_scale)
                self.normed_wy = -sev.motion.rx / float(self.full_scale)
                self.normed_wz = sev.motion.ry / float(self.full_scale)

                # apply calibration mapping so that an absolute value of
                # `calibration_max` becomes 1.0; clip to [-1.0, 1.0]
                if self.apply_calibration:
                    def _cal(v):
                        if self.calibration_max == 0:
                            return v
                        out = v / float(self.calibration_max)
                        if out > 1.0:
                            return 1.0
                        if out < -1.0:
                            return -1.0
                        return out

                    self.normed_vx = _cal(self.normed_vx)
                    self.normed_vy = _cal(self.normed_vy)
                    self.normed_vz = _cal(self.normed_vz)
                    self.normed_wx = _cal(self.normed_wx)
                    self.normed_wy = _cal(self.normed_wy)
                    self.normed_wz = _cal(self.normed_wz)

                self.motion_stale = True
                # continue to drain queue
                continue

            if ret == SPNAV_EVENT_BUTTON:
                bnum = sev.button.bnum
                press = sev.button.press
                if bnum < 0:
                    self.get_logger().warn(f'Negative spacenav buttons not supported. Got {bnum}')
                else:
                    if bnum < len(self.joystick_buttons):
                        self.joystick_buttons[bnum] = press
                    else:
                        # enlarge
                        extra = bnum + 1 - len(self.joystick_buttons)
                        self.joystick_buttons.extend([0] * extra)
                        self.joystick_buttons[bnum] = press
                    self.joy_stale = True
                continue

            # unknown type
            self.get_logger().warn('Unknown message type in spacenav.poll_spacenav')
            continue

        # publish motion if needed
        if self.motion_stale and (queue_empty or self.joy_stale):
            msg_offset = Vector3()
            msg_offset.x = self.normed_vx * self.linear_scale[0]
            msg_offset.y = self.normed_vy * self.linear_scale[1]
            msg_offset.z = self.normed_vz * self.linear_scale[2]

            msg_rot = Vector3()
            msg_rot.x = self.normed_wx * self.angular_scale[0]
            msg_rot.y = self.normed_wy * self.angular_scale[1]
            msg_rot.z = self.normed_wz * self.angular_scale[2]

            msg_twist = Twist()
            msg_twist.linear = msg_offset
            msg_twist.angular = msg_rot

            self.pub_offset.publish(msg_offset)
            self.pub_rot_offset.publish(msg_rot)
            if self.use_twist_stamped:
                ts = TwistStamped()
                ts.header.stamp = self.get_clock().now().to_msg()
                ts.twist = msg_twist
                self.pub_twist_stamped.publish(ts)
            else:
                self.pub_twist.publish(msg_twist)

            self.no_motion_count = 0
            self.motion_stale = False
            self.joy_stale = True

        if self.joy_stale:
            j = Joy()
            j.header.stamp = self.get_clock().now().to_msg()
            j.axes = [self.normed_vx, self.normed_vy, self.normed_vz, self.normed_wx, self.normed_wy, self.normed_wz]
            j.buttons = list(self.joystick_buttons)
            self.pub_joy.publish(j)
            self.joy_stale = False

    def destroy_node(self):
        try:
            libspnav.spnav_close()
        except Exception:
            pass
        super().destroy_node()


def main(argv=None):
    # 解析自定义命令行参数（保留 ROS/rclpy 的其余参数）
    parser = argparse.ArgumentParser(add_help=False)
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--right', action='store_true', help='use right -> Secondary motion group')
    parser.add_argument('--ee', action='store_true', help='use end-effector frame (ReferenceFrame.EEFrame)')
    parser.add_argument('--armbase', action='store_true', help='use arm base frame (ReferenceFrame.ArmBase). Default is WorldBase.')
    parser.add_argument('--real', action='store_true', help='use real robot (sets is_sim=False). Default is simulation.')
    known_args, remaining_args = parser.parse_known_args(sys.argv[1:] if argv is None else argv)

    print(f"[spacenav] custom flags: {known_args}")
    print("[spacenav] 如需控制真机在命令行传入 --real / 如需控制右臂在命令行传入 --right / 如需控制在末端坐标系下运动在命令行传入 --ee, 否则默认控制左臂在WorldBase坐标系下运动")

    # 选择 motion group：默认 Primary；--right 则使用 Secondary
    if known_args.right:
        motion_group = MotionGroup.Secondary
    else:
        motion_group = MotionGroup.Primary

    # 选择 reference frame：默认为 ArmBase；--ee 则使用 EEFrame
    reference_frame = ReferenceFrame.EEFrame if known_args.ee else ReferenceFrame.ArmBase if known_args.armbase else ReferenceFrame.WorldBase

    # 根据 --real 标志决定是否为模拟模式（默认为仿真）
    is_sim = not known_args.real

    # 初始化 rclpy，传入剩余参数
    rclpy.init(args=remaining_args)
    executor = MultiThreadedExecutor()
    sp_node = None
    client = None
    ros_thread = None
    try:
        sp_node = SpacenavNode()
        client = RobotClient(is_sim=is_sim)

        executor.add_node(sp_node)
        executor.add_node(client)
        # 根据所选 motion_group 选择对应臂的 gripper topic（Primary -> left, Secondary -> right）
        gripper_topic = '/control/gripper/left' if motion_group == MotionGroup.Primary else '/control/gripper/right'
        gripper_pub = client.create_publisher(EEJointControl, gripper_topic, 10)
        prev_buttons = []
        print_counter = 0
        def joy_callback(msg: Joy):
            nonlocal prev_buttons, print_counter
            axes = list(msg.axes)
            vals = np.array(axes[:6], dtype=float)
            # 找到绝对值最大的分量索引，只保留该分量，其余设为 0
            max_idx = int(np.argmax(np.abs(vals)))
            ee_velocity = np.zeros(6, dtype=float)
            ee_velocity[max_idx] = vals[max_idx] * 0.1 if max_idx < 3 else vals[max_idx] * 0.2

            # 每 10 次回调打印一次
            print_counter += 1
            if print_counter % 10 == 0:
                print(f"[spacenav] vel: {np.array2string(ee_velocity, formatter={'float_kind': lambda x: f'{x:.2f}'})}")

            client.send_streaming_command(ee_velocity, streaming_finished=False)
            buttons = list(msg.buttons)
            # ensure prev_buttons has enough length
            if len(prev_buttons) < len(buttons):
                prev_buttons.extend([0] * (len(buttons) - len(prev_buttons)))
            # rising edge: 0 -> 1
            if len(buttons) > 0 and buttons[0] and not prev_buttons[0]:
                cmd = EEJointControl()
                cmd.mode = 0
                cmd.value = [0.0]
                gripper_pub.publish(cmd)
                sp_node.get_logger().info('Gripper close command sent')
            if len(buttons) > 1 and buttons[1] and not prev_buttons[1]:
                cmd = EEJointControl()
                cmd.mode = 0
                cmd.value = [100.0]
                gripper_pub.publish(cmd)
                sp_node.get_logger().info('Gripper open command sent')
            # update previous state
            prev_buttons = buttons.copy()
        client.create_subscription(Joy, 'spacenav/joy', joy_callback, 10)
        ros_thread = threading.Thread(target=executor.spin, daemon=True)
        ros_thread.start()

        client.start_ee_velocity_streaming_request(motion_group, reference_frame=reference_frame)

        try:
            while rclpy.ok():
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
    finally:
        if client is not None:
            try:
                ee_velocity = np.zeros(6)
                client.send_streaming_command(ee_velocity, streaming_finished=True)
            except Exception:
                pass
        if sp_node is not None:
            sp_node.destroy_node()
            pass
        if client is not None:
            try:
                client.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
