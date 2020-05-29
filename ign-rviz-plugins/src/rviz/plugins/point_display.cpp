//
// Created by Sarathkrishnan Ramesh on 27/5/20.
//

#include "ignition/rviz/plugins/point_display.hpp"
#include <pluginlib/class_list_macros.hpp>

#include <string>
#include <utility>

namespace ignition
{
namespace rviz
{
namespace plugins
{
PointDisplay::PointDisplay()
: MessageDisplayBase()
{
  this->engine = ignition::rendering::engine("ogre");
  this->scene = this->engine->SceneByName("scene");
  auto x = this->scene->CreateAxisVisual();
}

void PointDisplay::initialize(rclcpp::Node::SharedPtr node)
{
  this->node = std::move(node);
}

void PointDisplay::setTopic(std::string topic_name)
{
  this->topic_name = topic_name;
  this->subscriber = this->node->create_subscription<geometry_msgs::msg::PointStamped>(
    this->topic_name, 10,
    std::bind(&PointDisplay::callback, this, std::placeholders::_1));
}

void PointDisplay::callback(geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  std::cout << "[" << this->topic_name << "]\tMessage received" << std::endl;

  ignition::rendering::VisualPtr box = this->scene->CreateAxisVisual();
  box->SetLocalPosition(msg->point.x, msg->point.y, msg->point.z);
  this->scene->RootVisual()->AddChild(box);
}
}  // namespace plugins
}  // namespace rviz
}  // namespace ignition

PLUGINLIB_EXPORT_CLASS(
  ignition::rviz::plugins::PointDisplay,
  ignition::rviz::plugins::MessageDisplayBase<geometry_msgs::msg::PointStamped>)
