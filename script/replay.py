import sys
import rclpy
from rclpy.node import Node
from joint_interfaces.msg import JointPositionControl
from std_msgs.msg import Float64MultiArray
import json
import time

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

# 读取运行参数
def read_args():
    if len(sys.argv) < 2:
        print("Usage: python replay.py <recorded_file_path>")
        sys.exit(1)
    return sys.argv[1]

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

# 主函数
def main(args=None):
    rclpy.init(args=args)

    # 创建节点
    node = Node("replay_node")
    node.publisher = node.create_publisher(JointPositionControl, "/control/joint_position", 10)
    left_hand_node = Node("left_hand_node")
    right_hand_node = Node("right_hand_node")
    left_hand_node.publisher = left_hand_node.create_publisher(Float64MultiArray, "/set_brainco_left_hand_qpos", 10)
    right_hand_node.publisher = right_hand_node.create_publisher(Float64MultiArray, "/set_brainco_right_hand_qpos", 10)

    # 读取记录文件路径
    recorded_file_path = read_args()

    # 读取记录数据
    recorded_data = read_recorded_file(recorded_file_path)

    # 发送记录的数据
    send_recorded_data(node, left_hand_node, right_hand_node, recorded_data)

    # 销毁节点
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()