#include <iostream>
#include <strings.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensors/msg/imu_data.hpp"
#include "geometry_msgs/msg/twist.hpp"

std::vector<float> split(std::string str, char delimiter)
{
    std::vector<float> vals;
    std::string num = "";
    for (size_t i = 0; i < str.length(); i++)
    {
        if (str[i] == delimiter || i == str.length() - 1)
        {
            try
            {
                vals.push_back(std::stof(num));
            }
            catch (...)
            {
                // Handle any conversion errors
            }
            num = "";
        }
        else
        {
            num = num + str[i];
        }
    }

    return vals;
}

class IMUNode : public rclcpp::Node
{
public:
    IMUNode() : Node("imu_node")
    {
        pub_ = this->create_publisher<sensors::msg::ImuData>("imu", 10);

        // Defining IMU port and opening
        port_ = open("/dev/nano", O_RDWR | O_NOCTTY | O_SYNC);
        if (port_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open port");
            return;
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&IMUNode::readAndPublishIMU, this));
    }

    ~IMUNode()
    {
        close(port_);
    }

private:
    void readAndPublishIMU()
    {
        char data;
        std::string imu_data = "";
        std::vector<float> imu;
        sensors::msg::ImuData values;

        while (read(port_, &data, sizeof(data)) > 0)
        {
            if (data == '\n')
            {
                do
                {
                    read(port_, &data, sizeof(data));
                    imu_data += data;
                } while (data != '\n');

                imu = split(imu_data, ' ');

                if (imu.size() >= 6)
                {
                    values.acceleration.x = imu[0];
                    values.acceleration.y = imu[1];
                    values.acceleration.z = imu[2];

                    values.orientation.x = imu[3];
                    values.orientation.y = imu[4];
                    values.orientation.z = imu[5];

                    pub_->publish(values);
                }

                imu_data = "";
            }
        }
    }

    int port_;
    rclcpp::Publisher<sensors::msg::ImuData>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUNode>());
    rclcpp::shutdown();
    return 0;
}

