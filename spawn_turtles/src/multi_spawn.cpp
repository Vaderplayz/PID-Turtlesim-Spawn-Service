// Basic ROS2 headers for C++ nodes and the Spawn service
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"

// C++ libraries for handling strings, numbers, and lists
#include <string>
#include <vector>
#include <memory>

int main(int argc, char **argv) {
  // 1. Initialize ROS2
  rclcpp::init(argc, argv);

  // 2. Read Inputs from the Command Line
  //    (This version assumes the inputs are always correct)
  int num_turtles = std::stoi(argv[1]); // Get the number of turtles
  int arg_index = 2; // The turtle data starts at the 3rd argument (index 2)

  // 3. Create a ROS2 Node and a Client
  //    The node is our program's connection to ROS2.
  //    The client is a tool to call the "/spawn" service.
  auto node = rclcpp::Node::make_shared("spawn_turtles_client");
  auto client = node->create_client<turtlesim::srv::Spawn>("/spawn");

  // We need a list to hold the "receipts" (futures) for each request we send.
  std::vector<rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture> futures;

  for (int i = 0; i < num_turtles; ++i) {
    // Create a new request for this turtle
    auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
    request->x = std::stof(argv[arg_index++]);
    request->y = std::stof(argv[arg_index++]);
    request->theta = std::stof(argv[arg_index++]);
    request->name = "turtle" + std::to_string(i+2);

    // Send the request and store the receipt (future) in our list
    futures.push_back(client->async_send_request(request));
  }

  // 6. Wait for All Requests to Complete
  //    Now that all requests have been sent, we wait for each one to finish.
  for (const auto &future : futures) {
    // This line pauses until a turtle has been successfully spawned
    rclcpp::spin_until_future_complete(node, future);
  }

  RCLCPP_INFO(node->get_logger(), "All turtles have been spawned.");

  // 7. Shutdown ROS2
  rclcpp::shutdown();
  return 0;
}