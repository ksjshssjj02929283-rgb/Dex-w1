import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "backend",
            default_value=os.environ.get("DEXE_CHASSIS_BACKEND", "cloudia"),
            description="dexe 或 cloudia；dexe 时适配器不建立 WebSocket"),
        DeclareLaunchArgument(
            "robot_host",
            default_value=os.environ.get("CLOUDIA_HOST", ""),
            description="迦智底盘工控机地址"),
        Node(
            package="cloudia_chassis_adapter",
            executable="cloudia_chassis_adapter",
            name="cloudia_chassis_adapter",
            output="screen",
            parameters=[
                os.path.join(
                    os.path.dirname(__file__), "..", "config",
                    "cloudia_chassis.yaml"),
                {
                    "backend": LaunchConfiguration("backend"),
                    "robot_host": LaunchConfiguration("robot_host"),
                },
            ],
        ),
    ])
