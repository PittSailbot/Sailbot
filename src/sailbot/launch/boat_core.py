import os
from datetime import datetime

from src import LaunchDescription
from src import DeclareLaunchArgument
from src import TextSubstitution
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
        ]
    )
