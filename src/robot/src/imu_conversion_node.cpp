#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "sensors/msg/imu_data.hpp"

class ImuConversionNode : public rclcpp::Node
{
public:
  ImuConversionNode() : Node("imu_conversion_node")
  {
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/zed/zed_node/imu/data", 10, std::bind(&ImuConversionNode::simulationImuCallback, this, std::placeholders::_1));

    imu_pub_ = this->create_publisher<sensors::msg::ImuData>("/imu_data", 10);
  }

private:
  void simulationImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(euler_msg_.orientation.x, euler_msg_.orientation.y, euler_msg_.orientation.z);

    if (euler_msg_.orientation.x < 0)
      euler_msg_.orientation.x = euler_msg_.orientation.x + M_PI * 2;

    if (euler_msg_.orientation.y < 0)
      euler_msg_.orientation.y = euler_msg_.orientation.y + M_PI * 2;

    if (euler_msg_.orientation.z < 0)
      euler_msg_.orientation.z = euler_msg_.orientation.z + M_PI * 2;

    euler_msg_.orientation.x = euler_msg_.orientation.x * 180 / M_PI;
    euler_msg_.orientation.y = euler_msg_.orientation.y * 180 / M_PI;
    euler_msg_.orientation.z = euler_msg_.orientation.z * 180 / M_PI;
    euler_msg_.acceleration.x = msg->linear_acceleration.x;
    euler_msg_.acceleration.y = msg->linear_acceleration.y;
    euler_msg_.acceleration.z = msg->linear_acceleration.z;

    imu_pub_->publish(euler_msg_);

  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensors::msg::ImuData>::SharedPtr imu_pub_;
  sensors::msg::ImuData euler_msg_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuConversionNode>());
  rclcpp::shutdown();
  return 0;
}