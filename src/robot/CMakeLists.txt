cmake_minimum_required(VERSION 3.8)
project(robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensors REQUIRED)  # Find the sensors package to use its messages
find_package(rosidl_default_generators REQUIRED)  # Required for custom message dependencies

# Include directories
include_directories(
  ${sensors_INCLUDE_DIRS}  # Include the sensors package directories
)
add_executable(imu_conversion_node src/imu_conversion_node.cpp)
add_executable(imu_sim src/imu_sim.cpp)

# Link against dependencies
ament_target_dependencies(imu_conversion_node
  rclcpp
  tf2
  tf2_geometry_msgs
  sensor_msgs
  geometry_msgs
  sensors  # Link against the sensors package
)
ament_target_dependencies(imu_sim
  rclcpp
  tf2
  tf2_geometry_msgs
  sensor_msgs
  geometry_msgs
  sensors  # Link against the sensors package
)
# Install the executable
install(TARGETS
  imu_conversion_node
  imu_sim
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch meshes worlds urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
