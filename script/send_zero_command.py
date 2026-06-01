#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from joint_interfaces.msg import JointCommand # 请确保该消息类型已 source
from std_msgs.msg import Header
import time

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher_py')
        
        # 1. 配置
        self.topic_name = '/hardware/joint_command'
        self.publisher_ = self.create_publisher(JointCommand, self.topic_name, 10)
        
        # 频率: 100Hz (0.01s)
        self.timer_period = 0.01  
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # 2. 定义数据
        self.joint_names = [
            'HIP_PITCH_L_J1', 'HIP_ROLL_L_J2', 'HIP_YAW_L_J3', 'KNEE_PITCH_L_J4', 'ANKLE_PITCH_L_J5', 'ANKLE_ROLL_L_J6',
            'HIP_PITCH_R_J1', 'HIP_ROLL_R_J2', 'HIP_YAW_R_J3', 'KNEE_PITCH_R_J4', 'ANKLE_PITCH_R_J5', 'ANKLE_ROLL_R_J6'
        ]
        
        # 转换为浮点数列表
        self.pos = [0.0] * 12
        # self.kp = [452.5, 114.3, 114.3, 452.5, 52.0, 52.0, 452.5, 114.3, 114.3, 452.5, 52.0, 52.0]
        self.kp = [1200.0] * 12
        # self.kd = [28.8, 7.3, 7.3, 28.8, 3.3, 3.3, 28.8, 7.3, 7.3, 28.8, 3.3, 3.3]
        self.kd = [20.0] * 12
        self.zeros = [0.0] * 12

        self.get_logger().info(f'已启动 100Hz 发布器: {self.topic_name}')

    def timer_callback(self):
        msg = JointCommand()
        
        # --- 填充 Header ---
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg() # 自动获取当前 ROS 时间
        #msg.header.frame_id = 'joint_command'
        
        # --- 填充数据 ---
        msg.name = self.joint_names
        msg.position = self.pos
        msg.velocity = self.zeros
        msg.torque = self.zeros
        msg.torque_feedforward = self.zeros
        msg.stiffness = self.kp
        msg.damping = self.kd
        
        # 发布
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointCommandPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户停止发送。')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
