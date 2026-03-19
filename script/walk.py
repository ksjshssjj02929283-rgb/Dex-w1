import rclpy
from rclpy.node import Node
import numpy as np
# 确保替换为您的实际消息包名
from joint_interfaces.msg import JointPositionControl

class LegAlternativeControl(Node):
    def __init__(self):
        super().__init__('leg_control_node')
        self.publisher_ = self.create_publisher(JointPositionControl, '/hardware/joint_position_control', 10)
        
        # 配置参数
        self.timer_period = 0.01   # 100Hz
        self.cycle_duration = 2.0  # 2秒一个周期
        self.limit = 3.14          # 弧度限制
        
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        # 关节名称
        self.left_names = ['LEFT_LEG_J1', 'LEFT_LEG_J2', 'LEFT_LEG_J3', 'LEFT_LEG_J4', 'LEFT_LEG_J5', 'LEFT_LEG_J6']
        self.right_names = ['RIGHT_LEG_J1', 'RIGHT_LEG_J2', 'RIGHT_LEG_J3', 'RIGHT_LEG_J4', 'RIGHT_LEG_J5', 'RIGHT_LEG_J6']
        
        # 目标抬起位置
        self.left_targets = [0.70, 0.00, -0.15, -1.20, 0.40, -0.30]
        self.right_targets = [-0.70, 0.00, 0.15, 1.20, -0.40, 0.30]

    def timer_callback(self):
        msg = JointPositionControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # 获取当前相对时间
        curr_time = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
        
        # 定义周期频率
        omega = np.pi / self.cycle_duration

        # --- 核心逻辑修改 ---
        # 左腿：保持原样，t=0 时 sin(0)=0
        left_factor = np.sin(omega * curr_time) ** 2
        
        # 右腿：我们希望它在第一个半周期保持 0，之后再开始
        # 使用 max(0, ...) 配合相位偏移，或者判断时间
        if curr_time < self.cycle_duration:
            # 在第一个周期内，右腿等待左腿完成一次起落后再开始
            # 或者如果你希望它们只是简单的交替，这里强制前段为0
            right_factor = 0.0 if curr_time < (self.cycle_duration / 2) else np.sin(omega * (curr_time - self.cycle_duration/2)) ** 2
        else:
            # 进入稳定循环后的逻辑
            right_factor = np.sin(omega * (curr_time + self.cycle_duration/2)) ** 2
        # ---------------------

        # 计算位置
        names = self.left_names + self.right_names
        positions = [t * left_factor for t in self.left_targets] + \
                    [t * right_factor for t in self.right_targets]

        # 安全断言
        for pos in positions:
            assert -self.limit <= pos <= self.limit, f"安全报警：检测到非法角度值 {pos:.4f}"

        msg.name = names
        msg.position = positions
        self.publisher_.publish(msg)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = LegAlternativeControl()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, AssertionError) as e:
        if isinstance(e, AssertionError):
            print(f"程序因断言失败停止: {e}")
        else:
            print("程序手动停止")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
