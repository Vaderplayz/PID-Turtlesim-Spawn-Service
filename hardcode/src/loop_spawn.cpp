#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include <string>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <iostream>
#include <cstdlib> // For rand() and srand()

using namespace std::chrono_literals;



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("spawn_n");
  auto client = node->create_client<turtlesim::srv::Spawn>("/spawn");
  
  int n = std::stof(argv[1]);
        srand(time(0));

    for (int i = 0; i < n; i++) {
          auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        float random_x = ((float)rand() / RAND_MAX) * 10.0f + 1.0f;   // Range: 1.0 to 11.0
        float random_y = ((float)rand() / RAND_MAX) * 10.0f + 1.0f;   // Range: 1.0 to 11.0
        float random_theta = ((float)rand() / RAND_MAX) * 2 * M_PI;   // Range: 0 to 2π

        int randomNumber4 = (rand() % 100000) + 1;


          request ->x = random_x;
          request ->y = random_y;
          request ->theta = random_theta;
        request->name = "turtle" + std::to_string(i+1);


  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  // Wait for the result.
  auto result = client->async_send_request(request);
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "All turtles have been spawned.");

  rclcpp::shutdown();
  return 0;
}