import argparse
import sys
import rclpy
from rclpy.node import Node
from joint_interfaces.msg import JointPositionControl
from std_msgs.msg import Float64MultiArray, String
import json
import time
import threading
from rclpy.executors import MultiThreadedExecutor
from pathlib import Path

# record_file示例
# {
#     "frames": [
#         {
#             "frame_id": 0,
#             "timestamp": 0.001,
#             "data": {
#                 "ANKLE": 3.8487891062375237,
#                 "KNEE": 1.8470482180531642,
#                 "BUTTOCK": 3.7048343221024203,
#                 "WAIST": 3.1417664251823636,
#                 "NECK1": 3.1416465827047286,
#                 "NECK2": 2.961978740234409,
#                 "LEFT_J1": 3.4059351986330872,
#                 "LEFT_J2": 1.7481182527655117,
#                 "LEFT_J3": 3.178354333604314,
#                 "LEFT_J4": 2.554082883356088,
#                 "LEFT_J5": 3.171667123352284,
#                 "LEFT_J6": 3.1128004983379958,
#                 "LEFT_J7": 2.8102821320440876,
#                 "RIGHT_J1": 2.88352985437457,
#                 "RIGHT_J2": 4.588420925457155,
#                 "RIGHT_J3": 3.1022423760583564,
#                 "RIGHT_J4": 3.747186653698612,
#                 "RIGHT_J5": 3.3527610913064594,
#                 "RIGHT_J6": 3.088724144581134,
#                 "RIGHT_J7": 3.2949730485908857,
#                 "LEFT_HAND_THUMB1": 24.42002442002442,
#                 "LEFT_HAND_THUMB2": 24.42002442002442,
#                 "LEFT_HAND_INDEX": 24.42002442002442,
#                 "LEFT_HAND_MIDDLE": 24.42002442002442,
#                 "LEFT_HAND_RING": 24.42002442002442,
#                 "LEFT_HAND_PINKY": 24.42002442002442,
#                 "RIGHT_HAND_THUMB1": 24.42002442002442,
#                 "RIGHT_HAND_THUMB2": 24.42002442002442,
#                 "RIGHT_HAND_INDEX": 24.42002442002442,
#                 "RIGHT_HAND_MIDDLE": 24.42002442002442,
#                 "RIGHT_HAND_RING": 24.42002442002442,
#                 "RIGHT_HAND_PINKY": 24.42002442002442
#             }
#         },
#         {
#             "frame": 1,
#             "timestamp": 1,
#             "data": {

# JointPositionControl的定义如下：
# # header
# std_msgs/Header header
# # 关节名称
# string[] name
# # 关节目标位置（角度）
# float64[] position

joint_keys = [
    "ANKLE", "KNEE", "BUTTOCK", "WAIST", "NECK1", "NECK2",
    "LEFT_J1", "LEFT_J2", "LEFT_J3", "LEFT_J4", "LEFT_J5", "LEFT_J6", "LEFT_J7",
    "RIGHT_J1", "RIGHT_J2", "RIGHT_J3", "RIGHT_J4", "RIGHT_J5", "RIGHT_J6", "RIGHT_J7"
]

finger_keys = ["LEFT_HAND_THUMB1", "LEFT_HAND_THUMB2", "LEFT_HAND_INDEX", "LEFT_HAND_MIDDLE", "LEFT_HAND_RING", "LEFT_HAND_PINKY",
                    "RIGHT_HAND_THUMB1", "RIGHT_HAND_THUMB2", "RIGHT_HAND_INDEX", "RIGHT_HAND_MIDDLE", "RIGHT_HAND_RING", "RIGHT_HAND_PINKY"]

LOG_KEYS = [
    "timestamp",
    "joint_position",
    "joint_velocity",
    "joint_torque",
    "joint_position_input",
    "joint_position_after_ruckig",
    "joint_position_after_filter",
    "joint_position_cmd",
    "joint_torque_feedforward",
    "torque_ext",
    "torque_ext_raw",
    "joint_admittance_q_offset",
    "joint_admittance_dq_offset",
    "joint_position_ref_admittance",
    "joint_impedance_error_after_admittance",
    "force_sensor",
    "force_sensor_gravity",
    "raw_force_sensor",
    "force_sensor_bias",
    "force_sensor_inertia",
    "ee_omega_ee_frame",
    "ee_alpha_ee_frame",
    "ee_pose_world",
]


class Recorder:
    def __init__(self, node: Node, topic: str, log_path: str):
        self._lock = threading.Lock()
        self._log_path = log_path
        self._samples = []
        self.count = 0
        node.create_subscription(String, topic, self._callback, 50)

    def _callback(self, msg: String):
        try:
            d = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        with self._lock:
            self._samples.append(d)
            self.count += 1

    def save(self):
        with self._lock:
            samples = self._samples
        with open(self._log_path, "w") as f:
            for s in samples:
                row = {}
                for k in LOG_KEYS:
                    if k in s:
                        row[k] = s[k]
                f.write(json.dumps(row) + "\n")


def parse_args():
    parser = argparse.ArgumentParser(description="Replay recorded robot data")
    parser.add_argument("recorded_file", help="Path to the recorded JSON file")
    parser.add_argument("--log", type=str, default=None,
                        help="录制 /feedback/robot_server_state 数据到 JSONL 文件")
    return parser.parse_args()

# 读取记录文件
def read_recorded_file(file_path):
    try:
        with open(file_path, 'r') as file:
            data = json.load(file)
        return data
    except Exception as e:
        print(f"Error reading file {file_path}: {e}")
        sys.exit(1)

# 发送JointPositionControl消息 /control/joint_position
# 按照记录的时间戳顺序发送
def send_recorded_data(node, left_hand_node, right_hand_node, recorded_data):
    frames = recorded_data.get("frames", [])
    if not frames:
        print("No frames found in the recorded data.")
        return

    start_time = time.time()
    record_start_time = None

    for frame in frames:
        # 状态检测：若为 Init 则停止
        if getattr(node, "server_status", None) == "Init":
            node.get_logger().warn("检测到机器人状态 Not All Enable, 停止下发并退出。")
            break

        timestamp = frame.get("timestamp", 0) #单位是秒
        if record_start_time is None:
            record_start_time = timestamp

        joint_data = frame.get("data", {})

        elapsed = time.time() - start_time
        sleep_time = (timestamp-record_start_time) - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)
        print("sleep time:", sleep_time)
        #input("Press Enter to continue...")
        # 发送消息
        send_joint_position(node, joint_data)

        left_hand_qpos = [joint_data.get(key, 0.0) for key in finger_keys[:6]]
        right_hand_qpos = [joint_data.get(key, 0.0) for key in finger_keys[6:]]

        send_hand_qpos(left_hand_node, left_hand_qpos)
        send_hand_qpos(right_hand_node, right_hand_qpos)

# 发送JointPositionControl消息，只发送joint_keys里的关节
def send_joint_position(node, joint_data):
    msg = JointPositionControl()
    msg.header.stamp = node.get_clock().now().to_msg()

    # 只发送joint_keys中的关节
    # msg.name = joint_keys
    # msg.position = [joint_data.get(key, 0.0) for key in joint_keys]
    msg.name = []
    msg.position = []
    for key in joint_keys:
        if key in joint_data:
            msg.name.append(key)
            msg.position.append(joint_data[key])

    # 发布消息
    node.get_logger().info(f"Sending joint positions: {msg}")
    node.publisher.publish(msg)

def send_hand_qpos(hand_node, hand_qpos):
    msg = Float64MultiArray()
    msg.data = hand_qpos
    hand_node.publisher.publish(msg)

def robot_state_callback(node):
    def _cb(msg: String):
        try:
            data = json.loads(msg.data)
            node.server_status = data.get("status")
        except Exception as e:
            node.get_logger().error(f"解析/feedback/robot_server_state失败: {e}")
    return _cb

# 主函数
def main(args=None):
    rclpy.init(args=args)
    cli_args = parse_args()

    node = Node("replay_node")
    node.publisher = node.create_publisher(JointPositionControl, "/control/joint_position", 10)
    node.server_status = None
    node.create_subscription(String, "/feedback/robot_server_state", robot_state_callback(node), 10)

    recorder = None
    if cli_args.log:
        recorder = Recorder(node, "/feedback/robot_server_state", cli_args.log)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    left_hand_node = Node("left_hand_node")
    right_hand_node = Node("right_hand_node")
    left_hand_node.publisher = left_hand_node.create_publisher(Float64MultiArray, "/set_brainco_left_hand_qpos", 10)
    right_hand_node.publisher = right_hand_node.create_publisher(Float64MultiArray, "/set_brainco_right_hand_qpos", 10)

    recorded_data = read_recorded_file(cli_args.recorded_file)

    send_recorded_data(node, left_hand_node, right_hand_node, recorded_data)

    if recorder:
        recorder.save()
        print(f"[SAVED] {cli_args.log} ({recorder.count} samples)")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
