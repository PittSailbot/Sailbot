source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
cd /home/pi/ros2_ws
rm -r build
rm -r install
colcon build
source install/local_setup.bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"
export ROS_LOG_DIR="/workspace/ros_logs/launch"
export ROS_LOG_DIR_BASE="/workspace/ros_logs/"
# colcon test
# # xmllint --format build/sailbot/pytest.xml
# python3 Utils/testResultsPrinter.py
ros2 launch sailbot websiteTest.py
echo "Completed"

