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
class PointDisplay : public MessageDisplay<geometry_msgs::msg::PointStamped>
{
  // Q_OBJECT

public:
  PointDisplay();
  ~PointDisplay();

  /**
   * @brief PointStamped ROS Visualzation plugin initialization
   * @param ROS Node shared pointer
   * @throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   */
  void initialize(rclcpp::Node::SharedPtr);

  /**
   * @brief ROS subscriber callback function
   * @param Shared pointer of PointStamped ROS message
   * @throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   */
  void callback(const geometry_msgs::msg::PointStamped::SharedPtr);

  /**
   * @brief Set ROS subscriber topic
   * @param ROS topic name
   * @throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   */
  void setTopic(std::string);

  // void LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/);

private:
  ignition::rendering::RenderEngine * engine;
  ignition::rendering::ScenePtr scene;
};
}  // namespace plugins
}  // namespace rviz
}  // namespace ignition

#endif  // IGNITION__RVIZ__PLUGINS__POINT_DISPLAY_HPP_
