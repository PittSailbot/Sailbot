# Interfaces
This folder contains any custom messages or services to be used with ROS2. [Documentation](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html).

To add new interfaces:
1. Check ROS2's standard messages to make sure that what you are creating doesn't already exist
2. Create a new .msg or .srv file in the corresponding directory
3. Edit `CMakeLists.txt` and add the path to the message in `rosidl_generate_interfaces()`
    - If the custom message depends on another ROS2 package, then add it with `find_packages()`

Standard ROS2 messages can be found here:
- [std_msgs](https://index.ros.org/p/std_msgs/github-ros2-common_interfaces/)
- [geometry_msgs](https://docs.ros2.org/latest/api/geometry_msgs/index-msg.html)
- [geographic_msgs](https://index.ros.org/p/geographic_msgs/#humble-assets)
