//
// Created by Sarathkrishnan Ramesh on 27/5/20.
//

#ifndef IGNITION__RVIZ__PLUGINS__MESSAGE_DISPLAY_BASE_HPP_
#define IGNITION__RVIZ__PLUGINS__MESSAGE_DISPLAY_BASE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace ignition
{
namespace rviz
{
namespace plugins
{
template<typename MessageType>
class MessageDisplayBase
{
protected:
  typename rclcpp::Subscription<MessageType>::SharedPtr subscriber;
  rclcpp::Node::SharedPtr node;
  std::string topic_name;

public:
  MessageDisplayBase() {}
  virtual ~MessageDisplayBase() {}

  /**
   * @brief ROS Visualzation plugin initialization function
   */
  virtual void initialize(rclcpp::Node::SharedPtr) = 0;

  /**
   * @brief ROS subscriber callback function
   */
  virtual void callback(typename MessageType::SharedPtr) = 0;

  /**
   * @brief Set ROS subscriber topic
   */
  virtual void setTopic(std::string) = 0;
};
}  // namespace plugins
}  // namespace rviz
}  // namespace ignition

#endif  // IGNITION__RVIZ__PLUGINS__MESSAGE_DISPLAY_BASE_HPP_
