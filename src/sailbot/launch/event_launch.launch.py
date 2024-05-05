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

    default = os.path.join(
        get_package_share_directory('sailbot'),
        'config',
        'params_eventDefaults.yaml'
        )
    
    eventExecutable = LaunchConfiguration('executable')
    paramsFile = LaunchConfiguration('paramsFile', default=default)

    with open('file.txt', 'w') as w:
        w.write(str(paramsFile))

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "log_level", default_value=TextSubstitution(text=str("DEBUG"))
            ),
            Node(
                package="sailbot",
                namespace="boat",
                executable=eventExecutable,
                name="node_DummyEvent",
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log_level"),
                ],
                parameters=[paramsFile],
            )
        ]
    )
