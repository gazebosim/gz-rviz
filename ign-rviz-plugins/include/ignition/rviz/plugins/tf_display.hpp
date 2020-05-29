//
// Created by Sarathkrishnan Ramesh on 27/5/20.
//

#ifndef IGNITION__RVIZ__PLUGINS__TF_DISPLAY_HPP_
#define IGNITION__RVIZ__PLUGINS__TF_DISPLAY_HPP_

#include <ignition/rendering.hh>
#include <tf2_msgs/msg/tf_message.hpp>

#include <string>

#include "ignition/rviz/plugins/message_display_base.hpp"


namespace ignition
{
namespace rviz
{
namespace plugins
{
class TFDisplay : public MessageDisplayBase<tf2_msgs::msg::TFMessage>
{
public:
  TFDisplay();
  void initialize(rclcpp::Node::SharedPtr);
  void callback(const tf2_msgs::msg::TFMessage::SharedPtr);
  void setTopic(std::string);

private:
  ignition::rendering::RenderEngine * engine;
  ignition::rendering::ScenePtr scene;
};
}  // namespace plugins
}  // namespace rviz
}  // namespace ignition

#endif  // IGNITION__RVIZ__PLUGINS__TF_DISPLAY_HPP_
