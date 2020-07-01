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

#include <pluginlib/class_loader.hpp>
#include <ignition/rviz/plugins/message_display_base.hpp>

#include <memory>
#include <vector>
#include <chrono>
#include <thread>

namespace ignition
{
namespace rviz
{

template<typename MessageType>
using DisplayPlugin = std::shared_ptr<plugins::MessageDisplay<MessageType>>;

template<typename MessageType>
using DisplayPluginList = std::vector<DisplayPlugin<MessageType>>;

class RViz : public QObject
{
  Q_OBJECT

public:
  RViz()
  : plugin_loader("ign_rviz_plugins", "ignition::rviz::plugins::MessageDisplayBase")
  {}

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
      // Create new instance of plugin
      DisplayPlugin<tf2_msgs::msg::TFMessage> tf_plugin =
        std::dynamic_pointer_cast<plugins::MessageDisplay<tf2_msgs::msg::TFMessage>>(
        plugin_loader.createSharedInstance(
          "ignition/rviz/plugins/TFDisplay"));
      tf_plugin->initialize(this->node);
      tf_plugin->setFrameManager(this->frameManager);
      tf_plugin->installEventFilter(ignition::gui::App()->findChild<ignition::gui::MainWindow *>());

      // Add the new plugin to the list
      tf_plugins.push_back(tf_plugin);
    } catch (pluginlib::PluginlibException & ex) {
      std::cout << ex.what() << std::endl;
    }
  }

  /**
   * @brief Loads LaserScan Visualization Plugin
   */
  Q_INVOKABLE void addLaserScanDisplay()
  {
    try {
      // Create new instance of plugin
      DisplayPlugin<sensor_msgs::msg::LaserScan> laser_scan_plugin =
        std::dynamic_pointer_cast<plugins::MessageDisplay<sensor_msgs::msg::LaserScan>>(
        plugin_loader.createSharedInstance(
          "ignition/rviz/plugins/LaserScanDisplay"));
      laser_scan_plugin->initialize(this->node);
      laser_scan_plugin->setTopic("/scan");
      laser_scan_plugin->setFrameManager(this->frameManager);
      laser_scan_plugin->installEventFilter(
        ignition::gui::App()->findChild<ignition::gui::MainWindow *>());

      // Add the new plugin to the list
      laser_scan_plugins.push_back(laser_scan_plugin);
    } catch (pluginlib::PluginlibException & ex) {
      std::cout << ex.what() << std::endl;
    }
  }

  Q_INVOKABLE void addAxesDisplay()
  {
    // Load plugin
    ignition::gui::App()->LoadPlugin("AxesDisplay");

    auto axes_plugins =
      ignition::gui::App()->findChildren<ignition::rviz::plugins::MessageDisplayBase *>();
    int axes_plugin_count = axes_plugins.size() - 1;

    // Set frame manager and install event filter for recently added plugin
    axes_plugins[axes_plugin_count]->setFrameManager(this->frameManager);
    ignition::gui::App()->findChild<ignition::gui::MainWindow *>()->installEventFilter(
      axes_plugins[
        axes_plugin_count]);
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

    // TODO(Sarathkrishnan Ramesh): Add splash screen to hide this delay

    // Small delay for tfBuffer to get populated
    std::this_thread::sleep_for(std::chrono::milliseconds(800));

    // Load Global Options plugin
    ignition::gui::App()->LoadPlugin("GlobalOptions");
    auto globalOptionsPlugin =
      ignition::gui::App()->findChild<ignition::rviz::plugins::MessageDisplayBase *>();

    // Set frame manager and install
    globalOptionsPlugin->setFrameManager(this->frameManager);
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

  // Plugins list
  DisplayPluginList<tf2_msgs::msg::TFMessage> tf_plugins;
  DisplayPluginList<sensor_msgs::msg::LaserScan> laser_scan_plugins;

  // Plugin Loader
  pluginlib::ClassLoader<plugins::MessageDisplayBase> plugin_loader;

  std::shared_ptr<common::FrameManager> frameManager;
};
}  // namespace rviz
}  // namespace ignition

#endif  // IGNITION__RVIZ__RVIZ_HPP_
