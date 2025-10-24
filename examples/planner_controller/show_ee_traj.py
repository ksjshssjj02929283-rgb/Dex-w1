import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import threading
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Header

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
from dexe_robot_client.w1.path_planner.planner_pinocchio import Planner_Pin
from dexe_robot_client.common import W1Version, W1ArmType
from dexe_robot_client.w1.path_planner.utils.robot_type import *

def trajectory_callback(msg):
    global traj_publisher
    global planner_pin
    joint_points = msg.points
    joint_traj_list = [point.positions for point in joint_points]
    joint_traj = np.array(joint_traj_list, dtype=np.float32)

    path = Path()
    path.header.stamp = rclpy.clock.Clock().now().to_msg()
    path.header.frame_id = "base_link"

    if joint_traj.shape[0] != 0:
        for qpos in joint_traj:
            torso_tf, head_tf, arml_tf, armr_tf = planner_pin.get_Kinematics(qpos)
            ee_pose = arml_tf[-1][:3, 3]
            pose = PoseStamped()
            pose.header.stamp = rclpy.clock.Clock().now().to_msg()
            pose.header.frame_id = "base_link"
            pose.pose.position.x = ee_pose[0]
            pose.pose.position.y = ee_pose[1]
            pose.pose.position.z = ee_pose[2]

            path.poses.append(pose)
        traj_publisher.publish(path)


if __name__ == "__main__":

    planner_pin = Planner_Pin(robot_type=RobotType.W1V02_1, end_effector_type=None)

    if not rclpy.ok():
        rclpy.init()

    traj_executor = MultiThreadedExecutor()
    traj_node = Node("show_ee_traj_node")
    traj_publisher = traj_node.create_publisher(
        Path, "dexe_robot_client/rviz2/ee_trajectory",
        10
    )
    traj_subscriber = traj_node.create_subscription(
        JointTrajectory,
        "/dexe_robot_client/trajectory",
        trajectory_callback,
        rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)
    )

    traj_executor.add_node(traj_node)
    ros_thread = threading.Thread(target=traj_executor.spin, daemon=True)
    ros_thread.start()

    time.sleep(1000000)
    