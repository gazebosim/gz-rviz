//
// Created by Sarathkrishnan Ramesh on 27/5/20.
//

#ifndef IGNITION__RVIZ__PLUGINS__POINT_DISPLAY_HPP_
#define IGNITION__RVIZ__PLUGINS__POINT_DISPLAY_HPP_

#include <ignition/rendering.hh>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <string>

#include "ignition/rviz/plugins/message_display_base.hpp"


namespace ignition
{
namespace rviz
{
namespace plugins
{
class PointDisplay : public MessageDisplayBase<geometry_msgs::msg::PointStamped>
{
public:
  PointDisplay();
  void initialize(rclcpp::Node::SharedPtr);
  void callback(const geometry_msgs::msg::PointStamped::SharedPtr);
  void setTopic(std::string);

private:
  ignition::rendering::RenderEngine * engine;
  ignition::rendering::ScenePtr scene;
};
}  // namespace plugins
}  // namespace rviz
}  // namespace ignition

#endif  // IGNITION__RVIZ__PLUGINS__POINT_DISPLAY_HPP_
