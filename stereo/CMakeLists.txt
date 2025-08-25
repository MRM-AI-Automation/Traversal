cmake_minimum_required(VERSION 3.5)
project(stereo)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Declare the message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Aruco.msg"
  "msg/Distance.msg"
  DEPENDENCIES std_msgs
)

# Include directories
include_directories(
  include
)


# Export the necessary dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_package()

