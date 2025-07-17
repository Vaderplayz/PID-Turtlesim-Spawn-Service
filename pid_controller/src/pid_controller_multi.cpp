// ROS2 headers for creating nodes, publishing, subscribing.
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

// C++ headers for lists (vectors), strings, and math.
#include <vector>
#include <string>
#include <cmath>
#include <iostream>

// --- CONFIGURATION ---
// To control more or fewer turtles, just change this one number!
const int NUM_TURTLES = 4;
// How close a turtle needs to be to its goal to be "done".
const double GOAL_TOLERANCE = 0.1;

// The "brain" for controlling our fleet of turtles.
class SimplePlanner : public rclcpp::Node
{
public:
    SimplePlanner() : Node("multi_planner")
    {
        // --- 1. SETUP PHASE ---
        // Resize our lists to hold the data for all the turtles.
        publishers_.resize(NUM_TURTLES);
        subscriptions_.resize(NUM_TURTLES);
        current_poses_.resize(NUM_TURTLES);
        goals_.resize(NUM_TURTLES);
        goal_reached_.resize(NUM_TURTLES, false); // All goals start as "not reached".

        // Loop to create a publisher and subscriber for each turtle.
        for (int i = 0; i < NUM_TURTLES; ++i) {
            // Generate topic names like "/turtle1/cmd_vel", "/turtle2/pose", etc.
            std::string cmd_vel_topic = "/turtle" + std::to_string(i + 1) + "/cmd_vel";
            std::string pose_topic = "/turtle" + std::to_string(i + 1) + "/pose";

            // Create a publisher for this turtle and store it in our list.
            publishers_[i] = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

            // Create a subscriber. The `[this, i]` part is a C++ "lambda" that
            // captures the turtle's index. This is how the callback knows which
            // turtle is sending its position.
            subscriptions_[i] = this->create_subscription<turtlesim::msg::Pose>(
                pose_topic, 10,
                [this, i](const turtlesim::msg::Pose::SharedPtr msg) {
                    this->current_poses_[i] = *msg;
                }
            );
        }
        
        // Start the main control loop timer. It runs 10 times per second.
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&SimplePlanner::control_loop, this));
    }

    // This function is called from main() to get all the goals *before* movement starts.
    void get_goals_from_user()
    {
        RCLCPP_INFO(this->get_logger(), "Please enter the goals for each turtle.");
        for (int i = 0; i < NUM_TURTLES; ++i) {
            std::cout << "Enter goal for turtle" << i + 1 << " (x y): ";
            std::cin >> goals_[i].x >> goals_[i].y;
        }
        RCLCPP_INFO(this->get_logger(), "All goals received. Starting navigation!");
    }

private:
    // This is the main brain, running on a timer.
    void control_loop()
    {
        bool all_turtles_are_done = true;

        // Loop through ALL our turtles in each cycle.
        for (int i = 0; i < NUM_TURTLES; ++i) {
            // If this turtle has already reached its goal, skip it.
            if (goal_reached_[i]) {
                continue;
            }
            
            // If we are in this part of the loop, it means at least one
            // turtle is still moving, so the mission is not done.
            all_turtles_are_done = false;

            // Calculate distance and angle for this specific turtle.
            double dx = goals_[i].x - current_poses_[i].x;
            double dy = goals_[i].y - current_poses_[i].y;
            double distance = std::sqrt(dx * dx + dy * dy);

            auto msg = geometry_msgs::msg::Twist();
            double goal_theta = std::atan2(dy, dx);
            double dtheta = goal_theta - current_poses_[i].theta;
            // If the turtle has arrived...
            if (distance < GOAL_TOLERANCE) {
                RCLCPP_INFO(this->get_logger(), "Turtle %d has reached its goal!", i + 1);
                goal_reached_[i] = true; // Mark this turtle as "done".
                msg.linear.x = 0.0;
                msg.angular.z = 0.0;
            } 
            // Otherwise, keep moving.
            else if (distance >= 0.1){
        RCLCPP_INFO(this->get_logger(), "%f", dtheta);
        if (dtheta > M_PI)  dtheta -= 2.0 * M_PI;
        if (dtheta < -M_PI) dtheta += 2.0 * M_PI;
        if (std::abs(dtheta) >=0.1){
            if (dtheta > 0) {
                msg.angular.z = dtheta * 2;
            } else {
                msg.angular.z = dtheta * -2;
            }
        }
    msg.linear.x = 0.5*distance;
    } 
            // Publish the velocity command to the correct turtle's publisher.
            publishers_[i]->publish(msg);
        }

        // --- 3. SHUTDOWN CHECK ---
        // After checking all turtles, if they are all done, shut down.
        if (all_turtles_are_done) {
            RCLCPP_INFO(this->get_logger(), "All turtles have reached their goals! Shutting down.");
            rclcpp::shutdown();
        }
    }

    // --- Member Variables ---
    std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> publishers_;
    std::vector<rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr> subscriptions_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<turtlesim::msg::Pose> current_poses_;
    std::vector<turtlesim::msg::Pose> goals_;
    std::vector<bool> goal_reached_;
};


int main(int argc, char * argv[])
{
    // --- The Main Program Flow ---
    rclcpp::init(argc, argv);

    // 1. Create the planner node object.
    auto planner_node = std::make_shared<SimplePlanner>();

    // 2. Call the function to get user input. This will PAUSE the program
    //    here until all goals are entered. This is the key to the requested flow.
    planner_node->get_goals_from_user();

    // 3. Now, start the ROS2 event loop (spinner). This will begin calling
    //    the timer and running the control_loop, so the turtles start moving.
    rclcpp::spin(planner_node);

    // The program will exit automatically when rclcpp::shutdown() is called inside the node.
    return 0;
}