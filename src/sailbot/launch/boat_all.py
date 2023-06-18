import os
from datetime import datetime

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    os.environ["ROS_LOG_DIR"] = f"/workspace/ros_logs/{str(datetime.now()).replace(' ', '_')}"
    os.environ["ROS_LOG_DIR_BASE"] = f"/workspace/ros_logs/{str(datetime.now()).replace(' ', '_')}"
    return LaunchDescription(
        [
            Node(package="sailbot", namespace="boat", executable="drivers", name="drivers"),
            Node(package="sailbot", namespace="boat", executable="main", name="main"),
            Node(package="sailbot", namespace="boat", executable="compass", name="compass"),
            Node(package="sailbot", namespace="boat", executable="gps", name="gps"),
        ]
    )
