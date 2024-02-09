"""
Handles running multiple nodes instead of running 'ros2 run sailbot <module>' for every node.
Run using ros2 launch sailbot boat_all.
"""
import os
from datetime import datetime

from src import LaunchDescription
from src import DeclareLaunchArgument
from src import TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    os.environ["ROS_LOG_DIR"] = f"/workspace/ros_logs/{str(datetime.now()).replace(' ', '_')}"
    os.environ["ROS_LOG_DIR_BASE"] = f"/workspace/ros_logs/{str(datetime.now()).replace(' ', '_')}"

    return LaunchDescription(
        [
            DeclareLaunchArgument("log_level", default_value=TextSubstitution(text=str("INFO"))),
            Node(package="sailbot", namespace="boat", executable="navigation", name="navigation"),
            Node(package="sailbot", namespace="boat", executable="compass", name="compass"),
            Node(package="sailbot", namespace="boat", executable="gps", name="gps"),
            Node(package="sailbot", namespace="boat", executable="transceiver", name="transceiver"),
            Node(package="sailbot", namespace="boat", executable="windvane", name="windvane"),
            Node(package="sailbot", namespace="boat", executable="sail", name="sail"),
            Node(package="sailbot", namespace="boat", executable="rudder", name="rudder")
        ]
    )
