source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
cd /home/pi/Sailbot
rm -r build
rm -r install
colcon build
 . install/local_setup.bash
