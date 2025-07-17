import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import rclpy
import json
from rclpy.node import Node
from std_msgs.msg import String

JOINT_NAME_MAP = {
    "ANKLE": "body1j",
    "KNEE": "body2j",
    "BUTTOCK": "body3j",
    "WAIST": "body4j",
    "NECK1": "head1j",
    "NECK2": "head2j",
    "LEFT_J1": "arml1j",
    "LEFT_J2": "arml2j",
    "LEFT_J3": "arml3j",
    "LEFT_J4": "arml4j",
    "LEFT_J5": "arml5j",
    "LEFT_J6": "arml6j",
    "LEFT_J7": "arml7j",
    "RIGHT_J1": "armr1j",
    "RIGHT_J2": "armr2j",
    "RIGHT_J3": "armr3j",
    "RIGHT_J4": "armr4j",
    "RIGHT_J5": "armr5j",
    "RIGHT_J6": "armr6j",
    "RIGHT_J7": "armr7j",
}
HAND_JOINTS_NAMES = [
    "L_indexMCP",
    "L_indexPIP",
    "L_littleMCP",
    "L_littlePIP",
    "L_middleMCP",
    "L_middlePIP",
    "L_ringMCP",
    "L_ringPIP",
    "L_thumbMCP1",
    "L_thumbMCP2",
    "R_indexMCP",
    "R_indexPIP",
    "R_littleMCP",
    "R_littlePIP",
    "R_middleMCP",
    "R_middlePIP",
    "R_ringMCP",
    "R_ringPIP",
    "R_thumbMCP1",
    "R_thumbMCP2",
]

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
        self.viz = None
        self._init_subscriber()
        self._init_visualization()

    def _init_subscriber(self):
        self.create_subscription(String, "/feedback/robot_server_state", self._subscriber_robot_server_state_callback, 10)

    def _init_visualization(self):
        params = parse_robot_cfg()

        package_share_path = get_package_share_directory("robot_server_lib")
        urdf_path = (
            Path(package_share_path)
            / "robot_models"
            / params.name
            / (params.name + ".urdf")
        )
        srdf_path = (
            Path(package_share_path)
            / "robot_models"
            / params.name
            / (params.name + ".srdf")
        )

        # Load the URDF model.
        urdf_model_path = urdf_path
        mesh_dir = urdf_path.parent
        self.robot = pin.RobotWrapper.BuildFromURDF(urdf_model_path, mesh_dir)
        self.robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=HAND_JOINTS_NAMES,
            reference_configuration=np.array([0.0] * self.robot.model.nq),
        )

        self.viewer = MeshcatVisualizer(
            self.robot.model, self.robot.collision_model, self.robot.visual_model
        )
        self.viz = MeshcatVisualizer(self.robot.model, self.robot.collision_model, self.robot.visual_model)
        self.viz.initViewer(open=True)
        self.viz.loadViewerModel()
        self.joint_position = pin.neutral(self.robot.model)
        self.viz.display(self.joint_position)
        self.viz.displayVisuals(True)

    def _subscriber_robot_server_state_callback(self, msg):
        state = json.loads(msg.data)
        if "joint_position" in state:
            self.joint_position = np.array(state["joint_position"])
            self.viz.display(self.joint_position)

def main(args=None):
    rclpy.init(args=args)
    visual_node = VisualRos2Node()
    rclpy.spin(visual_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
