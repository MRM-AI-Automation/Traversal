#include "include/planner.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::string imu_topic = "/imu_data";
  std::string point_cloud_topic = "/local_grid_obstacle";
  std::string aruco_topic = "/aruco_detect";
  rclcpp::spin(std::make_shared<planner::SensorCallback>(imu_topic,aruco_topic,point_cloud_topic));
  rclcpp::shutdown();
  return 0;
}