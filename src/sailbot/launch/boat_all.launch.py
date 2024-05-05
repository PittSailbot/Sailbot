"""
Handles running multiple nodes instead of running 'ros2 run sailbot <module>' for every node.
Run using ros2 launch sailbot boat_all.
"""
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
            DeclareLaunchArgument("log_level", default_value=TextSubstitution(text=str("DEBUG"))),
            Node(package="sailbot",
                namespace="boat",
                executable="navigation",
                name="node_Navigation",
                arguments=["--ros-args","--log-level",LaunchConfiguration("log_level")],
            ),
            Node(
                package="sailbot",
                namespace="boat",
                executable="compass",
                name="node_Compass",
                arguments=["--ros-args","--log-level",LaunchConfiguration("log_level")],
            ),
            Node(
                package="sailbot",
                namespace="boat",
                executable="gps",
                name="node_Gps",
                arguments=["--ros-args","--log-level",LaunchConfiguration("log_level")],
            ),
            Node(
                package="sailbot",
                namespace="boat",
                executable="transceiver",
                name="node_Transceiver",
                arguments=["--ros-args","--log-level", LaunchConfiguration("log_level")]
            ),
            Node(
                package="sailbot",
                namespace="boat",
                executable="windvane",
                name="node_Windvane",
                arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")]
            ),
            Node(
                package="sailbot",
                namespace="boat",
                executable="sail",
                name="node_Sail",
                arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")]
            ),
            Node(
                package="sailbot",
                namespace="boat",
                executable="rudder",
                name="node_Rudder",
                arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")]
            ),
            Node(
                package="sailbot",
                namespace="boat",
                executable="cameraServos",
                name="node_cameraServos",
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log_level"),
                ],
            ),
            Node(
                package="sailbot",
                namespace="boat",
                executable="actionManager",
                name="node_actionManager",
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log_level"),
                ],
            ),
        ]
    )
