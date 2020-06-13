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

#ifndef IGNITION__RVIZ__PLUGINS__MESSAGE_DISPLAY_BASE_HPP_
#define IGNITION__RVIZ__PLUGINS__MESSAGE_DISPLAY_BASE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory>

#ifndef Q_MOC_RUN
  #include <ignition/gui/qt.h>
  #include <ignition/gui/Plugin.hh>
#endif
#include <ignition/gui/MainWindow.hh>

#include "ignition/rviz/common/frame_manager.hpp"

namespace ignition
{
namespace rviz
{
namespace plugins
{
class MessageDisplayBase : public QObject
{
  Q_OBJECT

public:
  MessageDisplayBase()
  : QObject() {}

  /**
   * @brief Initialization function for visualization plugins
   * @param[in] node: ROS Node shared pointer
   * @throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   */
  virtual void initialize(rclcpp::Node::SharedPtr) {}
};

template<typename MessageType>
class MessageDisplay : public MessageDisplayBase
{
  // No Q_OBJECT macro here, moc does not support Q_OBJECT in a templated class.

public:
  MessageDisplay()
  : MessageDisplayBase() {}

  virtual ~MessageDisplay() {}

  // Documentation Inherited
  virtual void initialize(rclcpp::Node::SharedPtr) {}

  /**
   * @brief ROS subscriber callback function
   * @param[in] msg: ROS message type shared pointer
   * @throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   */
  virtual void callback(typename MessageType::SharedPtr) {}

  /**
   * @brief Set ROS subscriber topic
   * @param[in] topic_name: ROS topic name
   */
  virtual void setTopic(std::string) {}

  /**
   * @brief Install an event filter on ths object
   * @param[in] window: Application main window
   */
  virtual void installEventFilter(ignition::gui::MainWindow *) {}

  /**
   * @brief Store reference to FrameManager
   * @param[in] frameManager: Shared pointer to FrameManager object
   */
  virtual void setFrameManager(std::shared_ptr<common::FrameManager>) {}

protected:
  typename rclcpp::Subscription<MessageType>::SharedPtr subscriber;
  rclcpp::Node::SharedPtr node;
  std::string topic_name;
  std::shared_ptr<common::FrameManager> frameManager;
};
}  // namespace plugins
}  // namespace rviz
}  // namespace ignition
#endif  // IGNITION__RVIZ__PLUGINS__MESSAGE_DISPLAY_BASE_HPP_
