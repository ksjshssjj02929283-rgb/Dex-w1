import numpy as np
import meshcat
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import rclpy
import json
from rclpy.node import Node
from std_msgs.msg import String

def parse_robot_cfg():
    # 读取 robot 部分
    package_share_path = get_package_share_directory("sim_server")
    robot_cfg_path = Path(package_share_path) / "robot.cfg"
    with open(robot_cfg_path, 'r') as f:
        cfg = json.load(f)

    class Params:
        pass

    params = Params()
    params.name = cfg.get("robot_name")

    return params


class VisualRos2Node(Node):
    def __init__(self):
        super().__init__("visual_ros2_node")
        self.joint_position = None
        self.ee_pose = None
        self.vis_sim = None
        self._init_subscriber()
        self._init_visualization()

    def _init_subscriber(self):
        self.create_subscription(String, "/feedback/robot_server_state_sim", self._subscriber_robot_server_state_sim_callback, 10)

    def _init_visualization(self):
        params = parse_robot_cfg()

        package_share_path = get_package_share_directory("robot_server_lib")
        urdf_path = (
            Path(package_share_path)
            / "robot_models"
            / params.name
            / (params.name + ".urdf")
        )
        mesh_dir = urdf_path.parent
        self.robot = pin.RobotWrapper.BuildFromURDF(urdf_path, mesh_dir)
        self.vis_sim = MeshcatVisualizer(self.robot.model, self.robot.collision_model, self.robot.visual_model)
        self.vis_sim.initViewer(open=False)
        self.vis_sim.loadViewerModel()
        q0 = np.zeros(self.robot.model.nq) # 初始关节角度
        self.vis_sim.display(q0)

    def _subscriber_robot_server_state_sim_callback(self, msg):
        state = json.loads(msg.data)
        if "joint_position" in state:
            idx = [0, 1, 2, 3, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 4, 5]
            arr = np.array(state["joint_position"])
            self.joint_position = arr[idx]
            self.vis_sim.display(self.joint_position)

def main(args=None):
    rclpy.init(args=args)
    visual_node = VisualRos2Node()
    rclpy.spin(visual_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
