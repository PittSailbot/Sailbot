# Compiles and builds ROS2 sailbot package
# Default behavior: use cached build (incremental)
NO_CACHE=false

# Parse arguments
for arg in "$@"
do
    if [ "$arg" == "--no_cache" ]; then
        NO_CACHE=true
    fi
done

# ROS setup
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY

# Clean build and install directories if --no_cache is provided
if [ "$NO_CACHE" == true ]; then
    echo "Performing a clean build..."
    rm -rf build
    rm -rf install
else
    echo "Performing an incremental build..."
fi

# Perform the colcon build
colcon build --parallel-workers $(nproc) --continue-on-error

# Source the setup script
source install/local_setup.bash

# Set up logging formats
export RCUTILS_CONSOLE_OUTPUT_FORMAT="{message}"
export ROS_LOG_DIR="/workspace/ros_logs/launch"
export ROS_LOG_DIR_BASE="/workspace/ros_logs/"

# colcon test
# # xmllint --format build/sailbot/pytest.xml
# python3 Utils/testResultsPrinter.py
# ros2 launch sailbot websiteTest.py

echo "Build completed."

