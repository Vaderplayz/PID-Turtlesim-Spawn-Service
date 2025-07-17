#include "turtlesim/srv/spawn.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/kill.hpp"
#include <string>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <iostream>

using namespace std::chrono_literals;

class Turtle : public rclcpp::Node{

    public:
        Turtle(std::string turtle_name) : Node(turtle_name + "_controller"), 
        name_(turtle_name),
        got_info(false)
    {
    subscription1_ = this->create_subscription<turtlesim::msg::Pose>(
            "/"+ turtle_name+"/pose", 10, std::bind(&Turtle::pose_callback, this, std::placeholders::_1));

            
    publisher1_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/" + turtle_name +"/cmd_vel", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&Turtle::publisher_vel, this));
    
     goal_x_ = ((float)rand() / RAND_MAX) * 10.0f + 1.0f;   
     goal_y_ = ((float)rand() / RAND_MAX) * 10.0f + 1.0f;
}
private:
    std::string name_;
    turtlesim::msg::Pose pose_;
    bool got_info;
    float goal_x_;
    float goal_y_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription1_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher1_;
    rclcpp::TimerBase::SharedPtr timer_;

void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) 
    {
      pose_ = *msg;
      got_info = true;

    }


void publisher_vel() {

    auto msg = geometry_msgs::msg::Twist();
  

    //linear
    double dx = goal_x_ - pose_.x;
    double dy = goal_y_ - pose_.y;
    double distance = std::sqrt(dx* dx + dy * dy);

    //angular
    double goal_theta = std::atan2(dy, dx);
    double dtheta = goal_theta - pose_.theta;
    
    if (distance >= 0.1){
        RCLCPP_INFO(this->get_logger(), "%f", dtheta);
            if (dtheta > M_PI) dtheta -= 2.0 * M_PI;
            if (dtheta < -M_PI) dtheta += 2.0 * M_PI;
        if (std::abs(dtheta) >=0.1){
            if (dtheta > 0) {
                msg.angular.z = dtheta * 1.5;
            } else {
                msg.angular.z = dtheta * -1.5;
            }
        }
    msg.linear.x = 0.5*distance;
    } else if ( 0 <  distance && distance < 0.1){
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        got_info = false;
        RCLCPP_INFO(this->get_logger(), "Reached the goal!");
    }
    publisher1_->publish(msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    using namespace std::chrono_literals;


  auto node = rclcpp::Node::make_shared("spawn_n");
  auto client = node->create_client<turtlesim::srv::Spawn>("/spawn");
  auto client_kill = node->create_client<turtlesim::srv::Kill>("/kill");
  auto kill_request = std::make_shared<turtlesim::srv::Kill::Request>();
        kill_request  -> name = "turtle1";

    auto kill = client_kill->async_send_request(kill_request);

  std::vector<std::shared_ptr<Turtle>> turtles;
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  int n = std::stoi(argv[1]);
        srand(time(0));

    for (int i = 0; i < n; i++) {
          auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        float spawn_x = ((float)rand() / RAND_MAX) * 10.0f + 1.0f;   
        float spawn_y = ((float)rand() / RAND_MAX) * 10.0f + 1.0f;   // Range: 1.0 to 11.0
        float spawn_theta = ((float)rand() / RAND_MAX) * 2 * M_PI;   // Range: 0 to 2π


          request ->x = spawn_x;
          request ->y = spawn_y;
          request ->theta = spawn_theta;
          request->name = "turtle" + std::to_string(i+2);


  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  // Wait for the result.
        auto result = client->async_send_request(request);
        // Simple blocking wait (since we use SingleThreadedExecutor, safe here)
        rclcpp::spin_until_future_complete(node, result);

        // Create controller for this turtle
        auto turtle_controller = std::make_shared<Turtle>(request->name);
        turtles.push_back(turtle_controller);
        //push the node into executor
        executor->add_node(turtle_controller);
    }
//run executor 
    executor->spin();
    rclcpp::shutdown();
    return 0;

}