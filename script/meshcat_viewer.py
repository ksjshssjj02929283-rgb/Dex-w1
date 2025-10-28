import sys
import meshcat
import meshcat.geometry as mg
import numpy as np
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.visualize import MeshcatVisualizer
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from joint_interfaces.msg import JointPositionControl, JointControl

# ROS2关节名称和urdf关节名称的映射
JOINT_NAME_MAP = {
    "ANKLE": "ANKLE",
    "KNEE": "KNEE",
    "BUTTOCK": "BUTTOCK",
    "WAIST": "WAIST",
    "NECK1": "NECK1",
    "NECK2": "NECK2",
    "LEFT_J1": "LEFT_J1",
    "LEFT_J2": "LEFT_J2",
    "LEFT_J3": "LEFT_J3",
    "LEFT_J4": "LEFT_J4",
    "LEFT_J5": "LEFT_J5",
    "LEFT_J6": "LEFT_J6",
    "LEFT_J7": "LEFT_J7",
    "RIGHT_J1": "RIGHT_J1",
    "RIGHT_J2": "RIGHT_J2",
    "RIGHT_J3": "RIGHT_J3",
    "RIGHT_J4": "RIGHT_J4",
    "RIGHT_J5": "RIGHT_J5",
    "RIGHT_J6": "RIGHT_J6",
    "RIGHT_J7": "RIGHT_J7",
}

#HAND_JOINTS_NAMES = ['L_indexMCP', 'L_indexPIP', 'L_littleMCP', 'L_littlePIP', 'L_middleMCP', 'L_middlePIP', 'L_ringMCP', 'L_ringPIP', 'L_thumbMCP1', 'L_thumbMCP2', 'R_indexMCP', 'R_indexPIP', 'R_littleMCP', 'R_littlePIP', 'R_middleMCP', 'R_middlePIP', 'R_ringMCP', 'R_ringPIP', 'R_thumbMCP1', 'R_thumbMCP2']
HAND_JOINTS_NAMES = []

def draw_axes(vis, name):

    FRAME_AXIS_POSITIONS = (
        np.array([[0, 0, 0], [1, 0, 0],
                    [0, 0, 0], [0, 1, 0],
                    [0, 0, 0], [0, 0, 1]]).astype(np.float32).T
    )
    FRAME_AXIS_COLORS = (
        np.array([[1, 0, 0], [1, 0.6, 0],
                    [0, 1, 0], [0.6, 1, 0],
                    [0, 0, 1], [0, 0.6, 1]]).astype(np.float32).T
    )
    axis_length = 0.2
    axis_width = 20
    vis[name].set_object(
        mg.LineSegments(
            mg.PointsGeometry(
                position=axis_length * FRAME_AXIS_POSITIONS,
                color=FRAME_AXIS_COLORS,
            ),
            mg.LineBasicMaterial(
                linewidth=axis_width,
                vertexColors=True,
            ),
        )
    )


class MeshcatVisualizerWrapper:
    def __init__(self):

        np.set_printoptions(precision=5, suppress=True, linewidth=200)
        self.viewer = meshcat.Visualizer()

        # 加载机器人模型
        self.robot_feedback = pin.RobotWrapper.BuildFromURDF('../assets/DexforceW1_v02_1/DexforceW1_v02_1.urdf', '../assets/DexforceW1_v02_1/')
        self.robot_feedback = self.robot_feedback.buildReducedRobot(
            list_of_joints_to_lock=HAND_JOINTS_NAMES,
            reference_configuration=np.array([0.0] * self.robot_feedback.model.nq),
        )

        self.robot_control = pin.RobotWrapper.BuildFromURDF('../assets/DexforceW1_v02_1/DexforceW1_v02_1.urdf', '../assets/DexforceW1_v02_1/')
        self.robot_control = self.robot_control.buildReducedRobot(
            list_of_joints_to_lock=HAND_JOINTS_NAMES,
            reference_configuration=np.array([0.0] * self.robot_control.model.nq),
        )

        self.vis_feedback = MeshcatVisualizer(self.robot_feedback.model,
                                   self.robot_feedback.collision_model,
                                   self.robot_feedback.visual_model,
                                   )
        self.vis_control = MeshcatVisualizer(self.robot_control.model,
                                   self.robot_control.collision_model,
                                   self.robot_control.visual_model,
                                   )
        self.vis_feedback.initViewer(self.viewer)
        self.vis_control.initViewer(self.viewer)



        # self.vis = None
        # # 初始化 Meshcat 可视化器
        # self.vis = MeshcatVisualizer(self.robot.model, self.robot.collision_model, self.robot.visual_model)
        # self.vis.initViewer()
        # self.vis.loadViewerModel("pinocchio")
        # self.vis.display(pin.neutral(self.robot.model))

        # 将control的模型变为半透明
        self.vis_control.loadViewerModel("control")
        uniform_color = [1, 0.8, 0.4, 0.2]  # RGBA格式，值范围0-1
        for geom in self.vis_control.visual_model.geometryObjects:
            geom.meshColor = np.array(uniform_color)
        self.vis_control.loadViewerModel("control")


        self.vis_feedback.loadViewerModel("feedback")

        q0 = np.zeros(self.robot_feedback.model.nq) # 初始关节角度
        q1 = np.zeros(self.robot_control.model.nq)  # 初始关节角度
        self.vis_feedback.display(q0)
        self.vis_control.display(q1)

        draw_axes(self.vis_control.viewer, 'VR_head')
        draw_axes(self.vis_control.viewer, 'VR_left_hand')
        draw_axes(self.vis_control.viewer, 'VR_right_hand')

    def update_target(self, head, left_wrist, right_wrist):
        if head is not None:
            self.vis_control.viewer['VR_head'].set_transform(head)
        if left_wrist is not None:
            self.vis_control.viewer['VR_left_hand'].set_transform(left_wrist)
        if right_wrist is not None:
            self.vis_control.viewer['VR_right_hand'].set_transform(right_wrist)

class W1Subscriber(Node):
    def __init__(self):
        super().__init__('w1_pose_subscriber')
        self.first_message_received = False

        self.meshcat = MeshcatVisualizerWrapper()
        self.current_q_control = np.zeros(self.meshcat.robot_control.model.nq)

        self.feedback_joint_subscription = self.create_subscription(
            JointState,
            '/feedback/joint',
            self.feedback_joint_callback,
            10
        )
        self.control_joint_position_subscription = self.create_subscription(
            JointPositionControl,
            '/control/joint_position',
            self.control_joint_position_callback,
            10
        )

        self.control_joint_subscription = self.create_subscription(
            JointControl,
            '/control/joint',
            self.control_joint_position_callback,
            10
        )

        self.VR_head_subscription = self.create_subscription(
            Float64MultiArray,
            '/teleop/VR_head',
            self.VR_head_callback,
            10
        )

        self.VR_left_hand_subscription = self.create_subscription(
            Float64MultiArray,
            '/teleop/VR_left_hand',
            self.VR_left_hand_callback,
            10
        )

        self.VR_right_hand_subscription = self.create_subscription(
            Float64MultiArray,
            '/teleop/VR_right_hand',
            self.VR_right_hand_callback,
            10
        )

    def VR_head_callback(self, msg):
        head_pose = np.array(msg.data).reshape(4, 4)
        print('VR_head', head_pose)
        self.meshcat.update_target(head_pose, None, None)

    def VR_left_hand_callback(self, msg):
        print('VR_left_hand', msg.data)
        left_wrist = np.array(msg.data).reshape(4, 4)
        print('left_wrist', left_wrist)
        self.meshcat.update_target(None, left_wrist, None)

    def VR_right_hand_callback(self, msg):
        print('VR_right_hand', msg.data)
        right_wrist = np.array(msg.data).reshape(4, 4)
        print('right_wrist', right_wrist)
        self.meshcat.update_target(None, None, right_wrist)

    def feedback_joint_callback(self, msg):
        """
        回调函数，用于处理接收到的关节状态消息
        :param msg: 接收到的 JointState 消息
        """
        nq = self.meshcat.robot_feedback.model.nq
        joint_positions = np.zeros(nq)

        # 遍历 ROS 传来的 name 和 position
        for i, ros_name in enumerate(msg.name):
            # 查表映射
            if ros_name in JOINT_NAME_MAP:
                pin_name = JOINT_NAME_MAP[ros_name]
                # 获取在Pinocchio模型中的Joint ID (如 body1j -> 2, etc.)
                joint_id = self.meshcat.robot_feedback.model.getJointId(pin_name)
                if joint_id < 1:
                    self.get_logger().warn(f"Pinocchio joint not found for {pin_name}")
                    continue
                # 在Pinocchio中,配置向量的顺序是 getJointId(pin_name)-1 对应joint的下标
                # 对于Revolute关节，该joint占用1维
                idx = self.meshcat.robot_feedback.model.joints[joint_id].idx_q
                # 将关节角(加偏移3.14示例)放入正确位置
                joint_positions[idx] = msg.position[i]
            else:
                self.get_logger().warn(f'Unknown joint name: {ros_name}')

        # 更新机器人模型的关节位置
        self.meshcat.vis_feedback.display(joint_positions)

        # 如果是第一次接收到消息，更新robot_control的关节位置
        if not self.first_message_received:
            self.current_q_control = joint_positions.copy()
            self.meshcat.vis_control.display(self.current_q_control)
            self.first_message_received = True

    def control_joint_callback(self, msg):
        """
        回调函数，用于处理接收到的关节控制消息
        :param msg: 接收到的 JointControl 消息
        """
        #self.current_q = np.zeros(self.meshcat.robot_control.model.nq)
        #使用当前值初始化
        self.current_q = self.current_q_control

        for i, ros_name in enumerate(msg.name):
            if i < len(msg.position):
                angle = msg.position[i]
                # 查表映射
                if ros_name in JOINT_NAME_MAP:
                    pin_name = JOINT_NAME_MAP[ros_name]
                    joint_id = self.meshcat.robot_control.model.getJointId(pin_name)
                    if joint_id > 0:
                        # idx_q 是在配置向量中的起始索引
                        idx_q = self.meshcat.robot_control.model.joints[joint_id].idx_q
                        self.current_q[idx_q] = angle
                    else:
                        self.get_logger().warn(f"Pinocchio joint not found for {ros_name}")
                else:
                    self.get_logger().warn(f"Unknown joint name: {ros_name}")
            else:
                self.get_logger().warn(f"msg.position长度不足，无法更新关节 {ros_name}")

        # 更新可视化/下发指令等
        self.meshcat.vis_control.display(self.current_q)

    def control_joint_position_callback(self, msg):

        #self.current_q = np.zeros(self.meshcat.robot_control.model.nq)  # 初始化关节角度数组
        # 使用当前值初始化
        self.current_q = self.current_q_control

        for i, ros_name in enumerate(msg.name):
            if i < len(msg.position):
                angle = msg.position[i]
                # 查表映射
                if ros_name in JOINT_NAME_MAP:
                    pin_name = JOINT_NAME_MAP[ros_name]
                    joint_id = self.meshcat.robot_control.model.getJointId(pin_name)
                    if joint_id > 0:
                        # idx_q 是在配置向量中的起始索引
                        idx_q = self.meshcat.robot_control.model.joints[joint_id].idx_q
                        self.current_q[idx_q] = angle
                    else:
                        self.get_logger().warn(f"Pinocchio joint not found for {ros_name}")
                else:
                    self.get_logger().warn(f"Unknown joint name: {ros_name}")
            else:
                self.get_logger().warn(f"msg.position长度不足，无法更新关节 {ros_name}")

        # 更新可视化/下发指令等
        self.meshcat.vis_control.display(self.current_q)

        # self.robot_visualizer.update_robot(self.current_q)


def main(args=None):
    rclpy.init(args=args)

    # 创建并运行订阅者节点
    w1_subscriber = W1Subscriber()

    try:
        rclpy.spin(w1_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # 销毁节点并关闭 ROS 2
        w1_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
