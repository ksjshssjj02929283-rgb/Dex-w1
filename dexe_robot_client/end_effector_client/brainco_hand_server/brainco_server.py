# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from brainco_sdk import BraincoHands


class BraincoHandsNode(Node):
    def __init__(self):
        super().__init__('brainco_hands_node')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        
        import argparse
        parser = argparse.ArgumentParser()
        parser.add_argument('--port', default='/dev/ttyUSB0', help='Modbus串口路径')
        args = parser.parse_args()
        
        # 初始化SDK
        self.hands = BraincoHands(args.port)
        self.hands.enable_TurboMode(self.hands.left_id, 1)
        self.hands.enable_TurboMode(self.hands.right_id, 1)

        # 创建订阅器和发布器
        self._write_lock = threading.Lock()
        self.create_subscription(Float64MultiArray, 
                                 '/control/hand/left', 
                                 self._left_qpos_callback, 10)
        self.create_subscription(Float64MultiArray, 
                                 '/control/hand/right', 
                                 self._right_qpos_callback, 10)
        
        self.left_qpos_pub = self.create_publisher(Float64MultiArray, 
                                                   '/feedback/hand/left', 10)
        self.right_qpos_pub = self.create_publisher(Float64MultiArray, 
                                                    '/feedback/hand/right', 10)

        self.create_timer(0.1, self._publish_feedback) # 定时器(10Hz)
              
        self.get_logger().info(f"Hand initialized, use serial: {args.port}")

    def _left_qpos_callback(self, msg):
        if len(msg.data) == 6:
            left_data = [int(x) for x in msg.data]
            with self._write_lock:
                self.hands.write_position(self.hands.left_id, left_data)
                self.get_logger().info(f"Left hand qpos is set: {left_data}")
    
    def _right_qpos_callback(self, msg):
        if len(msg.data) == 6:
            right_data = [int(x) for x in msg.data]
            with self._write_lock:
                self.hands.write_position(self.hands.right_id, right_data)
                self.get_logger().info(f"Right hand qpos is set: {right_data}")

    def _publish_feedback(self):
        try:
            # 发布左手状态
            left_response = self.hands.read_position(self.hands.left_id)
            if left_response is not None:
                left_pos = Float64MultiArray()
                left_pos.data = [float(x) for x in left_response]
                self.left_qpos_pub.publish(left_pos)
                self.get_logger().debug(f"Left hand qpos: {left_response}")
            
            # 发布右手状态
            right_response = self.hands.read_position(self.hands.right_id)
            if right_response is not None:
                right_pos = Float64MultiArray()
                right_pos.data = [float(x) for x in right_response]
                self.right_qpos_pub.publish(right_pos)
                self.get_logger().debug(f"Right hand qpos: {right_response}")
            
        except Exception as e:
            self.get_logger().error(f"Feedback publish failed: {str(e)}")

    def destroy_node(self):
        """清理任务"""
        self.hands.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = BraincoHandsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
