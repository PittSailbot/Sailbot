from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from datetime import datetime
import os


def generate_launch_description():
    os.environ["ROS_LOG_DIR"] = f"/workspace/ros_logs/{str(datetime.now()).replace(' ', '_')}"
    os.environ["ROS_LOG_DIR_BASE"] = f"/workspace/ros_logs/{str(datetime.now()).replace(' ', '_')}"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "log_level", default_value=TextSubstitution(text=str("DEBUG"))
            ),
            Node(
                package="sailbot",
                namespace="boat",
                executable="drivers",
                name="node_drivers",
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log_level"),
                ],
            ),
            Node(
                package="sailbot", 
                namespace="boat", 
                executable="main", 
                name="node_main",
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log_level"),
                ],
            ),
        ]
    )
