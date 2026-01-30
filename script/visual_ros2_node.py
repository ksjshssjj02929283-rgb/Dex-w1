import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import rclpy
import json
import signal
import os
from rclpy.node import Node
from std_msgs.msg import String

# 尝试导入手部消息类型
try:
    from end_effector_interfaces.msg import EEJointFeedback
except ImportError:
    try:
        from joint_interfaces.msg import EEJointFeedback
    except ImportError:
        EEJointFeedback = None

class VisualRos2Node(Node):
    def __init__(self):
        super().__init__("visual_ros2_node")
        self.robot_name = None
        self.joint_position = None
        self.ee_pose = None
        self.vis_sim = None
        self.json_error_logged = False 
        
        # 1. 最小改动：直接定义本地路径
        self.model_root = "/home/dexforce/w1/dexe_robot_server/robot_server_lib/robot_models/W1_V2_A_BRAINCO"
        
        self._init_subscriber()
        self._init_visualization()

    def _init_subscriber(self):
        self.create_subscription(String, "/feedback/robot_server_state_sim", self._subscriber_robot_server_state_sim_callback, 10)
        # 2. 最小改动：增加手部订阅
        if EEJointFeedback:
            self.create_subscription(EEJointFeedback, "/feedback_sim/hand/left", self._left_hand_callback, 10)
            self.create_subscription(EEJointFeedback, "/feedback_sim/hand/right", self._right_hand_callback, 10)

    def _init_visualization(self):
        self.parse_robot_cfg()
        urdf_path = os.path.join(self.model_root, "W1_V2_A_BRAINCO.urdf")
        
        # 最小改动：加载本地模型
        self.robot = pin.RobotWrapper.BuildFromURDF(urdf_path, [self.model_root])
        self.vis_sim = MeshcatVisualizer(self.robot.model, self.robot.collision_model, self.robot.visual_model)
        self.vis_sim.initViewer(open=False)
        self.vis_sim.loadViewerModel()
        
        # 初始化全0
        self.joint_position = pin.neutral(self.robot.model)
        self.vis_sim.display(self.joint_position)

    def parse_robot_cfg(self):
        self.robot_name = "W1_V2_A_BRAINCO"

    def _subscriber_robot_server_state_sim_callback(self, msg):
        try:
            state = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        if "joint_position" in state:
            raw = np.array(state["joint_position"])
            if self.joint_position is None: return
            
            # 3. 最小改动：只更新身体和手臂
            # 0-12: Body + Left Arm
            n = min(len(raw), 13)
            self.joint_position[0:n] = raw[0:n]
            
            # 13-19: Right Arm -> 映射到 Pinocchio 的 23-29 (跳过左手区间 13-22)
            if len(raw) >= 20:
                self.joint_position[23:30] = raw[13:20]
            
            self.vis_sim.display(self.joint_position)

    # 4. 最小改动：手部回调函数
    def _update_hand(self, msg, start_idx):
        if self.joint_position is None: return
        pos = msg.position # 顺序: [ThumbMCP, ThumbCMC, Index, Middle, Ring, Little]
        
        def val(i): 
            v = pos[i]
            # 简单的单位转换：0-100 -> 0-1.6 rad
            return (v/100.0)*1.6 if abs(v)>1.6 else v

        # Pinocchio 内部顺序: Index(2), Little(2), Middle(2), Ring(2), Thumb(2)
        base = start_idx
        # Thumb (Pinocchio 最后两个)
        self.joint_position[base+8] = val(1) # Thumb CMC
        self.joint_position[base+9] = val(0) # Thumb MCP
        # Index
        self.joint_position[base+0] = self.joint_position[base+1] = val(2)
        # Middle (跳过 Little)
        self.joint_position[base+4] = self.joint_position[base+5] = val(3)
        # Ring
        self.joint_position[base+6] = self.joint_position[base+7] = val(4)
        # Little (Pinocchio 第二组)
        self.joint_position[base+2] = self.joint_position[base+3] = val(5)
        
        self.vis_sim.display(self.joint_position)

    def _left_hand_callback(self, msg):
        self._update_hand(msg, 13) # 左手起始索引 13

    def _right_hand_callback(self, msg):
        self._update_hand(msg, 30) # 右手起始索引 30

def main(args=None):
    rclpy.init(args=args)
    visual_node = VisualRos2Node()
    shutdown_requested = False

    def signal_handler(sig, frame):
        nonlocal shutdown_requested
        shutdown_requested = True

    signal.signal(signal.SIGINT, signal_handler)

    try:
        while not shutdown_requested and rclpy.ok():
            rclpy.spin_once(visual_node, timeout_sec=0.1)
    finally:
        if visual_node:
            visual_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
