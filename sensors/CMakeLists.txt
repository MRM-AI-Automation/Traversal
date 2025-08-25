cmake_minimum_required(VERSION 3.8)
project(sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Python3 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development)

# Declare the message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ImuData.msg"
  "srv/Data.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

add_executable(imu_node src/imu_node.cpp)
ament_target_dependencies(imu_node rclcpp geometry_msgs sensor_msgs std_msgs)

install(PROGRAMS
  scripts/ubx_parser.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install executables
install(TARGETS
  imu_node
  DESTINATION lib/${PROJECT_NAME}
)


rosidl_target_interfaces(imu_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Export the necessary dependencies
ament_export_dependencies(rosidl_default_runtime)

ament_package()
