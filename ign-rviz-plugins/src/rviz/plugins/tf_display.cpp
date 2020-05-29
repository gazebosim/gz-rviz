//
// Created by Sarathkrishnan Ramesh on 27/5/20.
//

#include "ignition/rviz/plugins/tf_display.hpp"
#include <pluginlib/class_list_macros.hpp>

#include <string>
#include <utility>

namespace ignition
{
namespace rviz
{
namespace plugins
{
TFDisplay::TFDisplay()
: MessageDisplayBase()
{
  this->engine = ignition::rendering::engine("ogre");
  this->scene = this->engine->SceneByName("scene");
}

void TFDisplay::initialize(rclcpp::Node::SharedPtr node)
{
  this->node = std::move(node);
}

void TFDisplay::setTopic(std::string topic_name)
{
  this->topic_name = topic_name;
  this->subscriber = this->node->create_subscription<tf2_msgs::msg::TFMessage>(
    this->topic_name, 10,
    std::bind(&TFDisplay::callback, this, std::placeholders::_1));
}

void TFDisplay::callback(tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  std::cout << "[" << this->topic_name << "]\tMessage received" << std::endl;
}
}  // namespace plugins
}  // namespace rviz
}  // namespace ignition

PLUGINLIB_EXPORT_CLASS(
  ignition::rviz::plugins::TFDisplay,
  ignition::rviz::plugins::MessageDisplayBase<tf2_msgs::msg::TFMessage>)
