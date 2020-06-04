// Copyright (c) 2020 Sarathkrishnan Ramesh
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
: MessageDisplay()
{
  this->engine = ignition::rendering::engine("ogre");
  this->scene = this->engine->SceneByName("scene");
}

TFDisplay::~TFDisplay() {

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
  for(const auto transform : msg->transforms) {
    std::cout << transform.header.frame_id << std::endl;
  }

  std::cout << "[" << this->topic_name << "]\tMessage received" << std::endl;
}
}  // namespace plugins
}  // namespace rviz
}  // namespace ignition

PLUGINLIB_EXPORT_CLASS(
  ignition::rviz::plugins::TFDisplay,
  ignition::rviz::plugins::MessageDisplayBase)
