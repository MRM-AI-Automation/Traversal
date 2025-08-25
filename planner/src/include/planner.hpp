#ifndef IRC_PLANNER_H_
#define IRC_PLANNER_H_

#include <rclcpp/rclcpp.hpp>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <string>
#include <iterator>
#include <algorithm>
#include <map>
#include <functional> 

#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensors/msg/imu_data.hpp"
#include "sensors/srv/data.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/empty.hpp"
#include "stereo/msg/aruco.hpp"
#include "std_msgs/msg/int32.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

namespace planner
{
    // Static Variable Definitions

    // Rover Dimensions and Physical Properties

    const double kRoverLength = 1.40;
    const double kRoverBreadth = 1.11;

    // const double kRoverLength = 0.5;
    // const double kRoverBreadth = 0.4;

    // Rover Speed & Obstacle constraints and parameters

    const double kMaxLinearVel = 0.6;
    const double kMinLinearVel = 0;

    const double kMaxAngularVel = 0.85;
    const double kMinAngularVel = 0;

    const double kMaxObsThreshold = 3.0;
    const double kMinObsThreshold = 0.5;
    const double kMinYObsThreshold = 0;

    const double kMaxXObsDistThreshold = 2;
    const double kMinXObsDistThreshold = 1;
    const double kMaxYObjDistThreshold = 2;
    const double kMinYObjDistThreshold = 0;

    const double kStopVel = 0;


    // Goal Distance Threshold

    const double kDistanceThreshold = 1;

    // Enum for Rover state
    // Class defines different states of the rover during the misson including:
    // 1. Search Pattern following
    // 2. Arrow Following
    // 3. Cone Following
    // 4. Obstacle Avoidance
    // 5. Data Analysis

    enum State
    {
        kArucoFollowing,
        kObstacleAvoidance,
        kDataAnalysis
    };

    enum SearchPatternType
    {
        kTerminate,
        kPatternFollowing
    };

    // Struct for Latitude and Longitude

  class SensorCallback : public rclcpp::Node
    {
    public:
        SensorCallback(std::string imu_topic, std::string gps_topic,std::string aruco_topic,std::string point_cloud_topic) : Node("planner_node")
        {
            // setting custom callback queue for node

           
            // Initializing Publishers
            vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
           

            // Initializing Subscribers
           
            imu_sub = this->create_subscription<sensors::msg::ImuData>(imu_topic, 10, std::bind(&SensorCallback::imuCallback, this, std::placeholders::_1));
            aruco_sub = this->create_subscription<stereo::msg::Aruco>(aruco_topic, 10, std::bind(&SensorCallback::arucoCallback, this, std::placeholders::_1));
            state_pub = this->create_publisher<std_msgs::msg::Int32>("/state_topic", 1);


            subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(point_cloud_topic, 10, std::bind(&SensorCallback::pclCallback, this, std::placeholders::_1));


            //  Formation of Equations

            obs_avoid_linear = straightLineEquation(kMinObsThreshold, kStopVel, kMaxObsThreshold, kMaxLinearVel);
            obs_avoid_angular = straightLineEquation(kRoverBreadth / 2, kStopVel, kMinYObsThreshold, kMaxAngularVel);
            obj_follow_linear = straightLineEquation(kMaxXObsDistThreshold, kMaxLinearVel, kMinXObsDistThreshold, kStopVel);
            obj_follow_angular = straightLineEquation(kMinYObjDistThreshold, kStopVel, kMaxYObjDistThreshold, kMaxAngularVel);
           
        };

        void setAngOffset(float offset){
            setAngularOffset(offset);
        };

        float getAngOffset(){
            return getAngularOffset();
        };

    private:
        // ROS Variables
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
        rclcpp::Subscription<sensors::msg::ImuData>::SharedPtr imu_sub;
        rclcpp::Subscription<stereo::msg::Aruco>::SharedPtr aruco_sub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_pub;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;



        // Equation Constants

        std::vector<double> obs_avoid_linear;
        std::vector<double> obs_avoid_angular;
        std::vector<double> obj_follow_linear;
        std::vector<double> obj_follow_angular;
        std::vector<double> coord_follow_angular;
        std::vector<double> bearing_equation;

        // Velocity publisher variable

        geometry_msgs::msg::Twist velocity;

        // Sensor Data Variables

        double aruco_x=100, aruco_y = 100, obs_y = 100;
        double current_orientation, dest_orientation;
        double curr_time, temp_time;
        bool zero_vel_flag;
        // sensor_msgs::Image image;
        sensors::msg::ImuData imu;

        // Output Variables

        std::vector<double> output_velocity;

        // Truthtable select lines

        bool aruco_detect = false;
        bool goal_reached = false;
        bool obstacle_detect = false;


        // Rover state variables
        State CurrState;
        State PrevState;

        // Returns a enum of rover state implementing the turth table below:
        // +========+=======+============+=======+===================+
        // | Arrow  | Cone  | Obstacle   | Goal  | State             |
        // +========+=======+============+=======+===================+
        // | 0      | 0     | 0          | 0     | Search Pattern    |
        // | 1      | 0     | 0          | 0     | Arrow Following   |
        // | 0/1    | 1     | 0          | 0     | Cone Following    |
        // | 0/1    | 0/1   | 1          | 0     | Obstacle Avoiding |
        // | 0/1    | 0/1   | 0/1        | 1     | Data Analysis     |
        // +--------+-------+------------+-------+-------------------+

        void RoverStateClassifier();

        // IMU Callback

        void imuCallback(const sensors::msg::ImuData::SharedPtr imu_msg);

        // GPS Callback


        void pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);


        // Cone Callbac

        // Arrow Callback

        void arucoCallback(const stereo::msg::Aruco::SharedPtr aruco);

        void stackRun();

        void publishVel(geometry_msgs::msg::Twist& msg);

        // Calling Search Pattern following function from controller class

        // Function to publish velocity for Obstacle Avoidance

        void obstacleAvoidance();

        // Function to publish velocity to Follow Objects

        void objectFollowing();


        // Get Searcg Pattern Flag

        std::vector<double> straightLineEquation(double x1, double y1, double x2, double y2);

        // Setters & Getters

        void setArucoStatus(bool status);

        void setGoalStatus();

        State getState(bool current);

        bool getObstacleStatus();

        void setAngularOffset(float offset);

        float getAngularOffset();

        // Call State Classifier function

        void callStateClassifier();

        // Returns the least angle to travel between two IMU angles

        void obstacleClassifier();

        // Function for analysing arrow coordinates and actions after each sub-goal

        void dataAnalyzer();

        // Setting goal coordiantes for the task
       
    };

    // Function Definitons for State Classifier Class

    void SensorCallback::RoverStateClassifier()
    {
        if (goal_reached == true)
        {
            if (CurrState != kDataAnalysis)
                PrevState = CurrState;
            CurrState = kDataAnalysis;
        }
        else if (obstacle_detect == true)
        {
            if (CurrState != kObstacleAvoidance)
                PrevState = CurrState;
            CurrState = kObstacleAvoidance;
        }
        else if (aruco_detect == true)
        {
            if (CurrState != kArucoFollowing)
                PrevState = CurrState;
            CurrState = kArucoFollowing;
        }
    }


    // Function Definitions for Sensor Interpreter Class

    void SensorCallback::setArucoStatus(bool status)
    {
        aruco_detect = status;
    }

    // void SensorCallback::setFollowPattern(State);
    void SensorCallback::setGoalStatus()
    {

        if(aruco_x<=2.0){
            if (goal_reached == false)
                RCLCPP_INFO(this->get_logger(),"GOAL HAS BEEN REACHED!");
            goal_reached = true;
        }else
            goal_reached = false;
    }

    State SensorCallback::getState(bool curr)
    {
        if (curr == true)
            return CurrState;
        else
            return PrevState;
    }
    bool SensorCallback::getObstacleStatus()
    {
        return obstacle_detect;
    }

    SearchPatternType SensorCallback::getSearchPatternType()
    {
        return FollowPattern;
    }
    void SensorCallback::setSearchPatternType(SearchPatternType state)
    {
        FollowPattern = state;
    }

    void SensorCallback::callStateClassifier()
    {
        RoverStateClassifier();
    }

    double SensorCallback::bearing(double dest, double curr)
    {
        // current orientation ranges from -180 to 180 and destination orientation ranges from -180 to 180
        //RCLCPP_INFO(this->get_logger(),"dest=%f curr=%f",dest,curr);
        if (dest - curr > 180)
            return -(360 - (dest - curr));
        else if (dest - curr > 0)
            return (dest - curr);
        else if (dest - curr < -180)
            return (360 - abs(dest - curr));
        else
            return -(dest - curr);
    }

    void SensorCallback::obstacleClassifier()
    {
        
        float min_value = kMaxObsThreshold + 1.5;
        obs_x = 0.0; // Initialize to avoid garbage value
        obs_y = 0.0; 
        obstacle_detect=false;
        float x_min = 0.5, x_max =3, y_min = -0.4, y_max = 0.4;

        for (const auto& point : cloud->points)
        {
            
            //min_value = 
            obs_x = point.x; 
            obs_y = point.y; 
            if (aruco_detect && sqrt(pow((obs_x - aruco_x), 2) + pow((obs_y - aruco_y), 2)) < 1)
            {
                obstacle_detect = false; // Arrow detected, no obstacle
                return;
            }

            // If obstacle is in the path of the rover
            if (point.x >= x_min && point.x <= x_max && point.y >= y_min && point.y <= y_max)
            {
                obstacle_detect = true; // Obstacle detected
            }
        }
        if(!obstacle_detect){
            RCLCPP_INFO(this->get_logger(), "No obstacle in front.");
        }
        RCLCPP_INFO(this->get_logger(), "Obstacle detected: %s", obstacle_detect ? "true" : "false");
    }

    void SensorCallback::dataAnalyzer()
    {
        else if (PrevState == kArucoFollowing)
        {
            RCLCPP_INFO(this->get_logger(), "Aruco Reached!");
            RCLCPP_INFO(this->get_logger(), "Staying for 10 Seconds");
            rclcpp::sleep_for(std::chrono::seconds(11));

        }
    }
                
    // Function Definitions for Sensor Callback Class

    void SensorCallback::imuCallback(const sensors::msg::ImuData::SharedPtr imu_msg)
    {
        stackRun();
        current_orientation = imu_msg->orientation.z;
    }



    void SensorCallback::pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);

    
        this->cloud = cloud;

        obstacleClassifier();

        if (getObstacleStatus() == true)
            search_pattern_function = 0;
    }



    void SensorCallback::arucpCallback(const stereo::msg::Aruco::SharedPtr aruco)
    {
        setArrowStatus(aruco->is_found);
        if (aruco->is_found == true)
        {
            aruco_x = aruco->x;
            aruco_y = aruco->y;
        }
        else
        {
            aruco_x = 100;
            aruco_y = 100;
        }
       
    }

    void SensorCallback::stackRun()
    {
        setGoalStatus();

        callStateClassifier();

        State curr_temp = getState(true);
        
 
        switch (curr_temp)
        {
        case kArucoFollowing:
            objectFollowing();
            break;
        case kObstacleAvoidance:
            obstacleAvoidance();
            break;
        case kDataAnalysis:
            velocity.linear.x = 0;
            velocity.angular.z = 0;
            publishVel(velocity);
            dataAnalyzer();
            break;
        }
    }

    // Function Definitons for Equation Generator Class
    void SensorCallback::publishVel(geometry_msgs::msg::Twist& msg)
    {
        if (msg.linear.x<0)
        {
            msg.linear.x=0;
        }
        vel_pub->publish(msg);
    }
   
    std::vector<double> SensorCallback::straightLineEquation(double x1, double y1, double x2, double y2)
    {

        std::vector<double> temp;

        double m = (y2 - y1) / (x2 - x1);
        double c = 0.5;

        temp.push_back(m);
        temp.push_back(c);

        return temp;
    }


    void SensorCallback::obstacleAvoidance()
    {
        if(obs_x>kMaxObsThreshold){
            velocity.linear.x = kMaxLinearVel;
        }else{
            velocity.linear.x = obs_avoid_linear[0] * obs_x + obs_avoid_linear[1];
        }
        if(velocity.linear.x < 0){
            velocity.linear.x = 0;
            //corner case
        }
        if(obs_y == 0){
            velocity.angular.z = kMaxAngularVel;
        }
        else if(obs_y < 0){
            velocity.angular.z = obs_avoid_angular[0] / abs(obs_x) + obs_avoid_angular[1];
            if(velocity.angular.z>kMaxAngularVel){
                velocity.angular.z = kMaxAngularVel;
            }
        }
        else{
            velocity.angular.z = -(obs_avoid_angular[0] / abs(obs_x) + obs_avoid_angular[1]);
            if(velocity.angular.z<-kMaxAngularVel){
                velocity.angular.z = -kMaxAngularVel;
            }
        }

        RCLCPP_INFO(this->get_logger(),"OA,x=%f y=%f, Obstacle X = %f Obstacle Y = %f",velocity.linear.x, velocity.angular.z,obs_x,obs_y);
        publishVel(velocity);
    };
   
    void SensorCallback::objectFollowing()
    {

        if (getState(true) == kArucoFollowing)
        {
            if(aruco_x > kMaxXObsDistThreshold){ 
                velocity.linear.x = kMaxLinearVel;
            }
            else
                velocity.linear.x = obj_follow_linear[0] * aruco_x + obj_follow_linear[1];
            if(aruco_y > kMaxYObjDistThreshold)
                velocity.angular.z = -kMaxAngularVel;
            else if (aruco_y < -kMaxYObjDistThreshold)
                velocity.angular.z = kMaxAngularVel;
            else if(aruco_y > 0)
                velocity.angular.z = -(obj_follow_angular[0] * abs(aruco_y) + obj_follow_angular[1]);
            else if(aruco_y < 0)
                velocity.angular.z = obj_follow_angular[0] * abs(aruco_y) + obj_follow_angular[1];
            RCLCPP_INFO(this->get_logger(),"Aruco Following");
            publishVel(velocity);
        }
    }
}
#endif