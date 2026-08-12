import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import rclpy
import json
import yaml
import signal
from rclpy.node import Node
from std_msgs.msg import String




class VisualRos2Node(Node):
    def __init__(self):
        super().__init__("visual_ros2_node")
        self.robot_name = None
        self.joint_name = None
        self.joint_position = None
        self.ee_pose = None
        self.vis_sim = None
        self.json_error_logged = False
        self._init_subscriber()
        self._init_visualization()

    def _init_subscriber(self):
        self.create_subscription(String, "/feedback/robot_server_state_sim", self._subscriber_robot_server_state_sim_callback, 10)

    def _init_visualization(self):
        user_cfg = Path.home() / "dexe_robot_server_data/robot_config.yaml"
        if user_cfg.exists():
            self.robot_cfg_path = str(user_cfg)
        else:
            share_dir = get_package_share_directory("w1_server")
            self.robot_cfg_path = str(Path(share_dir) / "robot_config.yaml")
        self.parse_robot_cfg()
        mesh_dir = Path(self.urdf_path).parent
        self.robot = pin.RobotWrapper.BuildFromURDF(self.urdf_path, mesh_dir)
        self.vis_sim = MeshcatVisualizer(self.robot.model, self.robot.collision_model, self.robot.visual_model)
        self.vis_sim.initViewer(open=False)
        self.vis_sim.loadViewerModel()
        
        # 获取按照 URDF 顺序排列的关节名称列表（忽略 'universe' 关节）
        self.urdf_joint_names = [name for name in self.robot.model.names if name != "universe"]
        print(f"URDF joint names: {self.urdf_joint_names}")
        self.q = np.zeros(self.robot.model.nq) # 缓存关节角度
        self.vis_sim.display(self.q)

    def parse_robot_cfg(self):
        with open(self.robot_cfg_path, 'r') as f:
            cfg = yaml.safe_load(f)
        self.robot_name = cfg.get("robot_name")
        
        extracted_urdf = cfg.get("file_path", {}).get("robot_urdf")
        if extracted_urdf:
            ee_urdf_path = Path(extracted_urdf).parent / "robot_with_ee.urdf"
            if ee_urdf_path.exists():
                self.urdf_path = str(ee_urdf_path)
            else:
                self.urdf_path = extracted_urdf

        # 读取 ros2 到 urdf 的名字映射
        self.ros2_to_urdf_map = cfg.get("ros2_to_urdf_map", {})
        # 同时建立反向映射，方便查找
        self.urdf_to_ros2_map = {v: k for k, v in self.ros2_to_urdf_map.items()}

    def _subscriber_robot_server_state_sim_callback(self, msg):
        try:
            state = json.loads(msg.data)
            self.json_error_logged = False
        except json.JSONDecodeError as e:
            if not self.json_error_logged:
                self.get_logger().warn(f"JSON decode error: {e}. Message data: {msg.data[:200]}...")
                self.json_error_logged = True
            return

        if "joint_name" in state and "joint_position" in state:
            joint_names = state["joint_name"]
            joint_positions = state["joint_position"]
            # 建立名称到位置的映射
            joint_map = dict(zip(joint_names, joint_positions))
            
            # 按照 URDF 顺序填充 q
            for name in self.urdf_joint_names:
                # 检查 URDF 关节名是否在映射表中（作为值存在）
                ros2_name = self.urdf_to_ros2_map.get(name)
                
                # 如果在映射表中找到了 ros2 名，且该名在当前状态中
                if ros2_name and ros2_name in joint_map:
                    val = joint_map[ros2_name]
                elif name in joint_map: # 回退方案：如果名字直接匹配
                    val = joint_map[name]
                else:
                    continue

                # 获取该关节在 q 中的起始索引 (pinocchio 内部索引)
                joint_id = self.robot.model.getJointId(name)
                idx_q = self.robot.model.joints[joint_id].idx_q
                nq = self.robot.model.joints[joint_id].nq
                self.q[idx_q : idx_q + nq] = val
                
            # 更新可视化
            if self.vis_sim is not None:
                self.vis_sim.display(self.q)


def main(args=None):
    rclpy.init(args=args)
    visual_node = VisualRos2Node()
    shutdown_requested = False
    shutdown_called = False

    def signal_handler(sig, frame):
        nonlocal shutdown_requested
        shutdown_requested = True

    signal.signal(signal.SIGINT, signal_handler)

    try:
        while not shutdown_requested and rclpy.ok():
            rclpy.spin_once(visual_node, timeout_sec=0.1)
    finally:
        visual_node.destroy_node()
        if not shutdown_called and rclpy.ok():
            rclpy.shutdown()
            shutdown_called = True

if __name__ == "__main__":
    main()
