#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <memory>


class BasicPlanner : public rclcpp::Node
{
public:
    BasicPlanner()
    : Node("planner"), got_info(false)
    {
        subscription1 = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&BasicPlanner::pose_callback, this, std::placeholders::_1));
    
        publisher1 = this->create_publisher<geometry_msgs::msg::Twist>(
            "/turtle1/cmd_vel", 10);

        timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&BasicPlanner::publisher_vel, this));


        }



private:
      bool got_info;
      double x_goal = 5.0;
      double y_goal = 5.0;
      turtlesim::msg::Pose pose;

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) 
    {
      pose = *msg;
      got_info = true;

    }

    void publisher_vel() {

    auto msg = geometry_msgs::msg::Twist();

   if (!got_info) return;
    //linear
    double dx = x_goal - pose.x;
    double dy = y_goal - pose.y;
    double distance = std::sqrt(dx* dx + dy * dy);

    //angular
    double goal_theta = std::atan2(dy, dx);
    double dtheta = goal_theta - pose.theta;
    
    if (distance >= 0.1){
        RCLCPP_INFO(this->get_logger(), "%f", dtheta);

        if (std::abs(dtheta) >=0.1){
            if (dtheta > 0) {
                msg.angular.z = dtheta * 5;
            } else {
                msg.angular.z = dtheta * -5;
            }
        }
    msg.linear.x = 0.5*distance;
    } else if ( 0 <  distance && distance < 0.1 && 0 < dtheta && dtheta < 0.1){
        got_info = false;
        RCLCPP_INFO(this->get_logger(), "Reached the goal!");
    }
    publisher1->publish(msg);
    }


  // Member handles:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher1;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription1;

  rclcpp::TimerBase::SharedPtr                        timer_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BasicPlanner>());
    rclcpp::shutdown();
    return 0;
}
