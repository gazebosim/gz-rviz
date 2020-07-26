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
#include <rclcpp/qos.hpp>

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
/**
 * @brief Namespace for all plugins.
 */
namespace plugins
{
/**
 * @brief Base class for all display plugins
 */
class MessageDisplayBase : public gui::Plugin
{
  Q_OBJECT

public:
  MessageDisplayBase()
  : Plugin() {}

  /**
   * @brief Initialization function for visualization plugins
   * @param[in] _node: ROS Node shared pointer
   * @throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   */
  virtual void initialize(rclcpp::Node::SharedPtr) {}

  /**
   * @brief Store reference to FrameManager
   * @param[in] _frameManager: Shared pointer to FrameManager object
   */
  virtual void setFrameManager(std::shared_ptr<common::FrameManager>) {}

protected:
  /**
   * @brief Reference to FrameManager
   */
  std::shared_ptr<common::FrameManager> frameManager;
};

/**
 * @brief Base class for all ROS visualization plugins
 * @tparam MessageType ROS2 message type
 */
template<typename MessageType>
class MessageDisplay : public MessageDisplayBase
{
  // No Q_OBJECT macro here, moc does not support Q_OBJECT in a templated class.

public:
  MessageDisplay()
  : MessageDisplayBase(), qos(5)
  {
    qos = qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    qos = qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos = qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  }

  virtual ~MessageDisplay() {}

  /**
   * @brief ROS subscriber callback function
   * @param[in] _msg: ROS message type shared pointer
   * @throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   */
  virtual void callback(const typename MessageType::SharedPtr) {}

  /**
   * @brief Set ROS subscriber topic
   * @param[in] topic_name: ROS topic name
   */
  virtual void setTopic(const std::string &) {}

protected:
  /**
   * @brief Create new ROS topic subscription
   */
  virtual void subscribe() {}

  /**
   * @brief Unsubscribe to topic
   */
  virtual void unsubscribe()
  {
    this->subscriber.reset();
  }

  /**
   * @brief Reset visualization.
   * Override this method and add visualization reset implementation
   */
  virtual void reset() {}

  /**
   * @brief Update visualization.
   * Override this method and add visualization reset implementation
   */
  virtual void update() {}

  /**
   * @brief Set history depth for keep last history QoS policy
   * @param _depth History depth
   */
  void setHistoryDepth(const int & _depth)
  {
    this->qos.keep_last(_depth);
  }

  /**
   * @brief Set History QoS policy
   * @param _historyPolicy Index of selected history policy
   */
  void setHistoryPolicy(const int & _historyPolicy)
  {
    switch (_historyPolicy) {
      case 0: this->qos.history(RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT);
        break;
      case 1: this->qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
        break;
      case 2: this->qos.history(RMW_QOS_POLICY_HISTORY_KEEP_ALL);
        break;
    }
  }

  /**
   * @brief Set Reliability QoS policy
   * @param _reliabilityPolicy Index of selected reliability policy
   */
  void setReliabilityPolicy(const int & _reliabilityPolicy)
  {
    switch (_reliabilityPolicy) {
      case 0: this->qos.reliability(RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
        break;
      case 1: this->qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        break;
      case 2: this->qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        break;
    }
  }

  /**
   * @brief Set Durability QoS policy
   * @param _durabilityPolicy Index of selected durability policy
   */
  void setDurabilityPolicy(const int & _durabilityPolicy)
  {
    switch (_durabilityPolicy) {
      case 0: this->qos.durability(RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT);
        break;
      case 1: this->qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
        break;
      case 2: this->qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        break;
    }
  }

protected:
  typename rclcpp::Subscription<MessageType>::SharedPtr subscriber;
  rclcpp::Node::SharedPtr node;
  rclcpp::QoS qos;
  std::string topic_name;
};

}  // namespace plugins
}  // namespace rviz
}  // namespace ignition

#endif  // IGNITION__RVIZ__PLUGINS__MESSAGE_DISPLAY_BASE_HPP_
