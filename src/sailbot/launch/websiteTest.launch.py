from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from datetime import datetime
import os


def generate_launch_description():
    os.environ["ROS_LOG_DIR"] = f"/workspace/ros_logs/{str(datetime.now()).replace(' ', '_')}"
    os.environ["ROS_LOG_DIR_BASE"] = f"/workspace/ros_logs/{str(datetime.now()).replace(' ', '_')}"

    config = os.path.join(
        get_package_share_directory('sailbot'),
        'config',
        'params_eventDefaults.yaml'
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "log_level", default_value=TextSubstitution(text=str("INFO"))
            ),
            Node(
                package="sailbot",
                namespace="boat",
                executable="virtualGPS",
                name="node_GPS",
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log_level"),
                ],
                parameters=[config],
            ),
            Node(
                package="sailbot",
                namespace="boat",
                executable="virtualCompass",
                name="node_Compass",
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log_level"),
                ],
                parameters=[config],
            ),
            Node(
                package="sailbot",
                namespace="boat",
                executable="navigation",
                name="node_Navigation",
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log_level"),
                ],
            ),
            Node(
                package="sailbot",
                namespace="boat",
                executable="virtualWindvane",
                name="node_Windvane",
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log_level"),
                ],
                parameters=[config],
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
                parameters=[config],
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
            # Node(
            #     package="sailbot",
            #     namespace="boat",
            #     executable='dummyEvent',
            #     name="node_DummyEvent",
            #     arguments=[
            #         "--ros-args",
            #         "--log-level",
            #         LaunchConfiguration("log_level"),
            #     ],
            #     parameters=[config],
            # ),
            # Node(
            #     package="sailbot",
            #     namespace="",
            #     executable="website",
            #     name="node_Website",
            #     arguments=[
            #         "--ros-args",
            #         "--log-level",
            #         LaunchConfiguration("log_level"),
            #     ]
            # ),
            
        ]
    )
