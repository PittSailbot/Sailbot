cmake_minimum_required(VERSION 3.8)
project(sailbot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Waypoint.msg"
  "msg/WaypointQueueState.msg"
  "srv/AddWaypoint.srv"
  "srv/UpdateWaypoint.srv"
  "srv/DeleteWaypoint.srv"
  "srv/ReorderWaypoints.srv"
  "srv/SetWaypointQueue.srv"
  "srv/SetCurrentIndex.srv"
  "srv/ClearWaypointQueue.srv"
  "srv/GetWaypointQueue.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

install(DIRECTORY msg srv
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
