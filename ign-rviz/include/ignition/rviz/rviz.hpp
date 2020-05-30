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

#ifndef IGNITION__RVIZ__RVIZ_HPP_
#define IGNITION__RVIZ__RVIZ_HPP_

#include <iostream>

#ifndef Q_MOC_RUN
  #include <ignition/gui/qt.h>
  #include <ignition/gui/Application.hh>
#endif

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <pluginlib/class_loader.hpp>
#include <ignition/rviz/plugins/message_display_base.hpp>

#include <memory>

namespace ignition
{
namespace rviz
{
class RViz : public QObject
{
  Q_OBJECT

public:
  RViz()
  : tf_loader("ign-rviz-plugins", "ignition::rviz::plugins::MessageDisplayBase"), point_loader(
      "ign-rviz-plugins", "ignition::rviz::plugins::MessageDisplayBase")
  {
  }

  /**
   * @brief Add Grid visual to Scene3D
   */
  Q_INVOKABLE void addGrid3D() const
  {
    ignition::gui::App()->LoadPlugin("Grid3D");
  }

  /**
   * @brief Loads TF Visualization Plugin
   */
  Q_INVOKABLE void addTFDisplay()
  {
    try {
      tf_plugin = tf_loader.createSharedInstance("ignition/rviz/plugins/TFDisplay");
      tf_plugin->initialize(this->node);
      tf_plugin->setTopic("/tf");
    } catch (pluginlib::PluginlibException & ex) {
      std::cout << ex.what() << std::endl;
    }
  }

  /**
   * @brief Load PointStamped Visualization Plugin
   */
  Q_INVOKABLE void addPointStampedDisplay()
  {
    try {
      point_plugin = point_loader.createSharedInstance("ignition/rviz/plugins/PointDisplay");
      point_plugin->initialize(this->node);
      point_plugin->setTopic("/point");
    } catch (pluginlib::PluginlibException & ex) {
      std::cout << ex.what() << std::endl;
    }
  }

  /**
   * @brief Initialize ignition RViz ROS node
   * @throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   */
  void init_ros()
  {
    this->node = std::make_shared<rclcpp::Node>("ignition_rviz");
  }

  /**
   * @brief Returns ignition RViz ROS node
   * @return ROS Node shared pointer
   */
  rclcpp::Node::SharedPtr get_node()
  {
    return this->node;
  }

private:
  // Data Members
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<plugins::MessageDisplayBase<tf2_msgs::msg::TFMessage>> tf_plugin;
  pluginlib::ClassLoader<plugins::MessageDisplayBase<tf2_msgs::msg::TFMessage>>
  tf_loader;
  std::shared_ptr<plugins::MessageDisplayBase<geometry_msgs::msg::PointStamped>>
  point_plugin;
  pluginlib::ClassLoader<plugins::MessageDisplayBase<geometry_msgs::msg::PointStamped>>
  point_loader;
};
}  // namespace rviz
}  // namespace ignition

#endif  // IGNITION__RVIZ__RVIZ_HPP_
