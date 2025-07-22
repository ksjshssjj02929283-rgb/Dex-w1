# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from brainco_sdk import BraincoHands


class BraincoHandsNode(Node):
    def __init__(self):
        super().__init__('brainco_hands_node')
        
        import argparse
        parser = argparse.ArgumentParser()
        parser.add_argument('--port', default='/dev/ttyUSB_485', help='Modbus串口路径')
        args = parser.parse_args()
        
        # 初始化SDK
        self.hands = BraincoHands(args.port)
        
        # 创建订阅器和发布器
        self.create_subscription(Float64MultiArray, 'set_brainco_left_hand_qpos', self.left_callback, 10)
        self.create_subscription(Float64MultiArray, 'set_brainco_right_hand_qpos', self.right_callback, 10)
        
        self.left_pos_pub = self.create_publisher(Float64MultiArray, 'brainco_left_hand_qpos', 10)
        self.right_pos_pub = self.create_publisher(Float64MultiArray, 'brainco_right_hand_qpos', 10)

        self.hands.enable_TurboMode(self.hands.left_id, 1)
        self.hands.enable_TurboMode(self.hands.right_id, 1)
        
        # 定时器(10Hz)
        self.create_timer(0.1, self.publish_states)
        self.create_timer(0.2, self.write_modbus)
        
        self.get_logger().info(f"灵巧手节点已启动，使用串口: {args.port}")

    def left_callback(self, msg):
        if len(msg.data) == 6:
            self.left_data = [int(x) for x in msg.data]
            # try:
            #     self.hands.write_position(self.hands.left_id, [int(x) for x in msg.data])
            # except Exception as e:
            #     self.get_logger().error(f"左手设置失败: {str(e)}")

    def right_callback(self, msg):
        if len(msg.data) == 6:
            self.right_data = [int(x) for x in msg.data]
            # try:
            #     self.hands.write_position(self.hands.right_id, [int(x) for x in msg.data])
            # except Exception as e:
            #     self.get_logger().error(f"右手设置失败: {str(e)}")

    def publish_states(self):
        try:
            # 发布左手状态
            left_pos = Float64MultiArray()
            left_pos.data = [float(x) for x in self.hands.read_position(self.hands.left_id)]
            self.left_pos_pub.publish(left_pos)
            
            # 发布右手状态
            right_pos = Float64MultiArray()
            right_pos.data = [float(x) for x in self.hands.read_position(self.hands.right_id)]
            self.right_pos_pub.publish(right_pos)
            
        except Exception as e:
            self.get_logger().error(f"状态发布失败: {str(e)}")
    
    def write_modbus(self):
        try:
             self.hands.write_position(self.hands.left_id, self.left_data)
        except:
            pass
        try:
            self.hands.write_position(self.hands.right_id, self.right_data)                
        except:
            pass

def main():
    rclpy.init()
    node = BraincoHandsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
