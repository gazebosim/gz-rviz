//
// Created by Sarathkrishnan Ramesh on 2/26/20.
//

#include "ign-rviz/VisualizationManager.hpp"

int main(int argc, char ** argv)
{
  // Initialize ROS Node
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node;

  node = std::make_shared<rclcpp::Node>("ignition_rviz");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // Initialize Ignition RViz
  VisualizationManager rviz(argc, argv, node);

  // Start async spinner
  std::thread executor_thread(std::bind(&rclcpp::executors::MultiThreadedExecutor::spin,
    &executor));

  // Run renderer
  rviz.run();

  executor_thread.join();

  return 0;
}
