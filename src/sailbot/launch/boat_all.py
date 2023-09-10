import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    os.environ[
        "ROS_LOG_DIR"
    ] = f"/workspace/ros_logs/{str(datetime.now()).replace(' ', '_')}"
    os.environ[
        "ROS_LOG_DIR_BASE"
    ] = f"/workspace/ros_logs/{str(datetime.now()).replace(' ', '_')}"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "log_level", default_value=TextSubstitution(text=str("INFO"))
            ),
            Node(
                package="sailbot",
                namespace="boat",
                executable="drivers",
                name="drivers",
            ),
            Node(package="sailbot", namespace="boat", executable="main", name="main"),
            Node(
                package="sailbot",
                namespace="boat",
                executable="compass",
                name="compass",
            ),
            Node(package="sailbot", namespace="boat", executable="gps", name="gps"),
        ]
    )
