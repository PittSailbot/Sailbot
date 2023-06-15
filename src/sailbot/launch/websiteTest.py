from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from datetime import datetime
import os

def generate_launch_description():
    os.environ['ROS_LOG_DIR'] = F"/workspace/ros_logs/{str(datetime.now()).replace(' ', '_')}"
    os.environ['ROS_LOG_DIR_BASE'] = F"/workspace/ros_logs/{str(datetime.now()).replace(' ', '_')}"
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'log_level', 
            default_value=TextSubstitution(text=str("INFO"))),

        Node(
            package='sailbot',
            namespace='boat',
            executable='virtualDrivers',
            name='virtualDrivers',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),
        Node(
            package='sailbot',
            namespace='boat',
            executable='virtualGPS',
            name='virtualGPS',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),
        Node(
            package='sailbot',
            namespace='boat',
            executable='website',
            name='website',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        )
    ])

