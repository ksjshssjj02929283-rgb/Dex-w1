# 1. 接收sensor_msgs.msg.JointState的消息/feedback/joint
# 2. 读取records文件夹下一个json文件（由argv指定）中第一帧的关节角度，作为目标位置
# 3. 检查当前位置和目标位置的电机name是否一致
# 4. 将当前位置和目标位置之间差值200个点，总耗时20秒
# 5. 每个点的时间间隔为0.1秒，发送到w1_control.msg.JointPositionControl的/control/joint_position话题

# w1_control.msg.JointPositionControl定义
# # header
# std_msgs/Header header
# # 关节名称
# string[] name
# # 关节目标位置（角度）
# float64[] position

# json文件示例
# {
#   "frames": [
#     {
#       "frame": 0,
#       "timestamp": 0,
#       "data": {
#         "ANKLE": 4.6715,
#         "KNEE": 0.5108,
#         ...
#       }
#     }
#   ]
# }

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from joint_interfaces.msg import JointPositionControl
from std_msgs.msg import Header, String

import json
import sys
import os
import time
import numpy as np
import threading
from rclpy.executors import ExternalShutdownException

class JointTrajectoryPublisher(Node):
    def __init__(self, json_file, n_frames=1000):
        super().__init__('joint_trajectory_publisher')
        self.subscription = self.create_subscription(
            JointState,
            '/feedback/joint',
            self.joint_state_callback,
            10)
        self.robot_state_subscription = self.create_subscription(
            String,
            '/feedback/robot_server_state',
            self.robot_state_callback,
            10)
        self.publisher = self.create_publisher(
            JointPositionControl,
            '/control/joint_position',
            10)
        self.json_file = json_file
        self.status = None
        self.current_position = None
        self.target_position = None
        self.joint_names = None
        self.timer_started = False
        self.timer = None
        self.interval = 0.02
        self.step_index = 0
        self.positions = None
        self.total_points = None
        self._shutdown_requested = False

        # 加载目标位置
        self.load_target_position()
        self.n_frames = n_frames
        print('等待接收/feedback/joint消息...')

    def robot_state_callback(self, msg):
        state = json.loads(msg.data)
        self.status = state["status"]

    def joint_state_callback(self, msg):
        if self.current_position is None:
            self.current_position = []
            self.joint_names = []
            for i in range(len(msg.position)):
                name = msg.name[i]
                position = msg.position[i]
                if name in self.target_position_dict:
                    self.current_position.append(position)
                    self.joint_names.append(name)

            self.get_logger().info('收到初始JointState')
            print(f"当前关节名称: {self.joint_names}")
            print(f"当前关节位置: {self.current_position}")

            self.target_position = [self.target_position_dict[name] for name in self.joint_names]
            print(f"目标关节位置: {self.target_position}")

            # 启动插值发布
            self.interpolate_and_publish()

    def load_target_position(self):
        if not os.path.exists(self.json_file):
            self.get_logger().error(f"文件不存在: {self.json_file}")
            rclpy.shutdown()
            return

        with open(self.json_file, 'r') as f:
            data = json.load(f)

        try:
            first_frame_data = data["frames"][0]["data"]
        except (KeyError, IndexError) as e:
            self.get_logger().error(f"无法解析 JSON: {e}")
            rclpy.shutdown()
            return

        self.target_position_dict = dict(first_frame_data)
        print("目标关节", self.target_position_dict)
        self.target_names = list(self.target_position_dict.keys())

    def interpolate_and_publish(self):
        if self.timer_started:
            return
        self.total_points = self.n_frames
        self.positions = np.linspace(
            np.array(self.current_position),
            np.array(self.target_position),
            self.total_points
        )
        self.step_index = 0
        self.timer = self.create_timer(self.interval, self._publish_step)
        self.timer_started = True
        self.get_logger().info(f"开始插值发送，共 {self.total_points} 点，间隔 {self.interval}s")

    def _publish_step(self):
        # 若状态为 Init，停止插值并退出
        if self.status == "Init":
            self.get_logger().warn("检测到机器人Not all in OP mode，停止插值并退出。")
            self._request_shutdown()
            return

        # 发送完成则退出
        if self.step_index >= len(self.positions):
            self.get_logger().info("插值完成，程序结束。")
            self._request_shutdown()
            return

        # 发送当前步
        msg = JointPositionControl()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.positions[self.step_index].tolist()

        self.publisher.publish(msg)
        if self.step_index % 100 == 0:
            print(f"发布位置: {msg}")
            self.get_logger().info(f"发送第{self.step_index+1}/{self.total_points}个位置")
        self.step_index += 1

    def _request_shutdown(self):
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
            self.timer_started = False
        if not self._shutdown_requested:
            self._shutdown_requested = True
            try:
                self.destroy_node()
            except Exception:
                pass
            threading.Thread(
                target=lambda: rclpy.shutdown(context=self.context),
                daemon=True
            ).start()

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("请提供记录文件名，例如：python3 your_node.py records/demo.json")
        return

    json_path = sys.argv[1]
    if len(sys.argv) == 3:
        n_frames = int(sys.argv[2])
    else:
        n_frames = 1000
    print("将用%d帧数据进行插值" % n_frames)

    node = JointTrajectoryPublisher(json_path, n_frames)
    rclpy.spin(node)

if __name__ == '__main__':
    main()