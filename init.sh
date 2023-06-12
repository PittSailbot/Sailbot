source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
cd /home/pi/ros2_ws
source install/local_setup.bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"
export ROS_LOG_DIR="/workspace/ros_logs"
echo "Completed"