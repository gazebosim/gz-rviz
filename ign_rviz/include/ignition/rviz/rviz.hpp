// Copyright (c) 2020 Open Source Robotics Foundation, Inc.
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
#include <sensor_msgs/msg/laser_scan.hpp>

#include <ignition/rviz/plugins/message_display_base.hpp>

#include <memory>
#include <vector>

namespace ignition
{
namespace rviz
{

template<typename MessageType>
using DisplayPlugin = plugins::MessageDisplay<MessageType>;

class RViz : public QObject
{
  Q_OBJECT

public:
  RViz() {}

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
    // Load plugin
    if (ignition::gui::App()->LoadPlugin("TFDisplay")) {
      auto tfDisplayPlugins =
        ignition::gui::App()->findChildren<DisplayPlugin<tf2_msgs::msg::TFMessage> *>();
      int tfDisplayCount = tfDisplayPlugins.size() - 1;

      // Set frame manager and install event filter for recently added plugin
      tfDisplayPlugins[tfDisplayCount]->initialize(this->node);
      tfDisplayPlugins[tfDisplayCount]->setFrameManager(this->frameManager);
      ignition::gui::App()->findChild<ignition::gui::MainWindow *>()->installEventFilter(
        tfDisplayPlugins[tfDisplayCount]);
    }
  }

  /**
   * @brief Loads LaserScan Visualization Plugin
   */
  Q_INVOKABLE void addLaserScanDisplay()
  {
    // Load plugin
    if (ignition::gui::App()->LoadPlugin("LaserScanDisplay")) {
      auto laserScanPlugin =
        ignition::gui::App()->findChildren<DisplayPlugin<sensor_msgs::msg::LaserScan> *>();
      int pluginCount = laserScanPlugin.size() - 1;

      // Set frame manager and install event filter for recently added plugin
      laserScanPlugin[pluginCount]->initialize(this->node);
      laserScanPlugin[pluginCount]->setTopic("/scan");
      laserScanPlugin[pluginCount]->setFrameManager(this->frameManager);
      ignition::gui::App()->findChild<ignition::gui::MainWindow *>()->installEventFilter(
        laserScanPlugin[pluginCount]);
    }
  }

  /**
   * @brief Loads Axes Visualization Plugin
   */
  Q_INVOKABLE void addAxesDisplay()
  {
    // Load plugin
    if (ignition::gui::App()->LoadPlugin("AxesDisplay")) {
      auto axes_plugins =
        ignition::gui::App()->findChildren<ignition::rviz::plugins::MessageDisplayBase *>();
      int axes_plugin_count = axes_plugins.size() - 1;

      // Set frame manager and install event filter for recently added plugin
      axes_plugins[axes_plugin_count]->setFrameManager(this->frameManager);
      ignition::gui::App()->findChild<ignition::gui::MainWindow *>()->installEventFilter(
        axes_plugins[axes_plugin_count]);
    }
  }

  /**
   * @brief Initialize ignition RViz ROS node and frame manager
   * @throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   */
  void init_ros()
  {
    this->node = std::make_shared<rclcpp::Node>("ignition_rviz");
    this->frameManager = std::make_shared<common::FrameManager>(this->node);
    this->frameManager->setFixedFrame("world");

    // Load Global Options plugin
    if (ignition::gui::App()->LoadPlugin("GlobalOptions")) {
      auto globalOptionsPlugin =
        ignition::gui::App()->findChild<ignition::rviz::plugins::MessageDisplayBase *>();

      // Set frame manager and install
      globalOptionsPlugin->setFrameManager(this->frameManager);

      // Install event filter
      ignition::gui::App()->findChild<ignition::gui::MainWindow *>()->installEventFilter(
        globalOptionsPlugin);
    }
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

  std::shared_ptr<common::FrameManager> frameManager;
};

}  // namespace rviz
}  // namespace ignition

#endif  // IGNITION__RVIZ__RVIZ_HPP_
