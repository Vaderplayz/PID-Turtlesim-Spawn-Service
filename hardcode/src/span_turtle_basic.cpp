#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include <string>
#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("spawn_4");
  auto client = node->create_client<turtlesim::srv::Spawn>("/spawn");
  
  auto request1 = std::make_shared<turtlesim::srv::Spawn::Request>();
  auto request2 = std::make_shared<turtlesim::srv::Spawn::Request>();
  auto request3 = std::make_shared<turtlesim::srv::Spawn::Request>();
  auto request4 = std::make_shared<turtlesim::srv::Spawn::Request>();

  request1->x = std::stof(argv[1]);
  request1->y = std::stof(argv[2]);
  request1->theta = std::stof(argv[3]);

  request2->x = std::stof(argv[4]);
  request2->y = std::stof(argv[5]);
  request2->theta = std::stof(argv[6]);

    request3->x = std::stof(argv[7]);
  request3->y = std::stof(argv[8]);
  request3->theta = std::stof(argv[9]);

    request4->x = std::stof(argv[10]);
  request4->y = std::stof(argv[11]);
  request4->theta = std::stof(argv[12]);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  // Wait for the result.
  auto result1 = client->async_send_request(request1);
  auto result2 = client->async_send_request(request2);
  auto result3 = client->async_send_request(request3);
  auto result4 = client->async_send_request(request4);

  if (rclcpp::spin_until_future_complete(node, result1) == rclcpp::FutureReturnCode::SUCCESS &&
rclcpp::spin_until_future_complete(node, result2) == rclcpp::FutureReturnCode::SUCCESS &&
rclcpp::spin_until_future_complete(node, result3) == rclcpp::FutureReturnCode::SUCCESS &&
rclcpp::spin_until_future_complete(node, result4) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Success");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
  }

  rclcpp::shutdown();
  return 0;
}