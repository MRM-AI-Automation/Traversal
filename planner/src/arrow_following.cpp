#include <chrono>
#include <rclcpp/rclcpp.hpp>

class PlannerNode : public rclcpp::Node {
public:
    PlannerNode() 
        : Node("planner"), goal_reached_(false), search_started_(false), 
          moving_forward_(true), cone_detected_once_(false)
    {
        // Initialize with a 500ms interval for "Moving Forward" state
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&PlannerNode::logArrowFollowing, this)
        );
        start_time_ = this->now();
    }

private:
    void logArrowFollowing() {
        auto current_time = this->now();
        auto elapsed_time = (current_time - start_time_).seconds();

        // Print "Starting Search Pattern: Moving Forward..." for a specified duration (e.g., 8 seconds)
        if (moving_forward_ && elapsed_time < 10.0) {
            RCLCPP_INFO(this->get_logger(), "Starting Search Pattern: Moving Forward...");
        } else if (moving_forward_) {
            moving_forward_ = false;  // Stop printing "Moving Forward" after 8 seconds
            if (!cone_detected_once_) {
                RCLCPP_INFO(this->get_logger(), "cone detected");
                cone_detected_once_ = true;  // Ensure "cone detected" only prints once

                // Update timer to a faster interval (e.g., 200ms) for "Cone Following"
                timer_->cancel(); // Cancel the existing timer
                timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(50),
                    std::bind(&PlannerNode::logArrowFollowing, this)
                );
            }
        }

        // Switch to "Cone Following" after the "Moving Forward" period ends
        if (!moving_forward_ && !goal_reached_) {
            RCLCPP_INFO(this->get_logger(), "Cone Following");
            
            // Simulate reaching the goal after a certain total time
            if (elapsed_time >= 19.0) {  // Adjust as needed
                goal_reached_ = true;
                logGoalSequence();
            }
        }
    }

    void logGoalSequence() {
        RCLCPP_INFO(this->get_logger(), "Cone Reached!");
        RCLCPP_INFO(this->get_logger(), "Saving coordinates to CSV...");
        RCLCPP_INFO(this->get_logger(), "MISSION TERMINATED: CONE REACHED");
        rclcpp::shutdown();
    }

    rclcpp::TimerBase::SharedPtr timer_;
    bool goal_reached_;
    bool search_started_;
    bool moving_forward_;
    bool cone_detected_once_;
    rclcpp::Time start_time_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}
