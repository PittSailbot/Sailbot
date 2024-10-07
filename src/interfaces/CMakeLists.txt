cmake_minimum_required(VERSION 3.8)
project(interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add dependencies here
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)

# Add custom messages/services here
# rosidl_generate_interfaces(${PROJECT_NAME}
  # "msg/Waypoint.msg"
  # DEPENDENCIES std_msgs geometry_msgs geographic_msgs
 # )

ament_package()

