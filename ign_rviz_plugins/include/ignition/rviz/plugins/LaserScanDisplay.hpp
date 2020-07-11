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

#ifndef IGNITION__RVIZ__PLUGINS__LASERSCANDISPLAY_HPP_
#define IGNITION__RVIZ__PLUGINS__LASERSCANDISPLAY_HPP_

#include <ignition/rendering.hh>

#include <sensor_msgs/msg/laser_scan.hpp>

#include <string>
#include <mutex>
#include <memory>
#include <vector>

#include "ignition/rviz/plugins/message_display_base.hpp"

namespace ignition
{
namespace rviz
{
namespace plugins
{
class LaserScanDisplay : public MessageDisplay<sensor_msgs::msg::LaserScan>
{
  Q_OBJECT

  /**
   *  @brief Topic List
   */
  Q_PROPERTY(
    QStringList topicList
    READ getTopicList
    NOTIFY topicListChanged
  )

public:
  /**
   * Constructor for laser scan visualization plugin
   */
  LaserScanDisplay();

  // Destructor
  ~LaserScanDisplay();

  // Documentation Inherited
  void LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/);

  // Documentation Inherited
  void initialize(rclcpp::Node::SharedPtr _node);

  // Documentation Inherited
  void callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg);

  // Documentation inherited
  void setTopic(std::string topic_name);

  // Documentation inherited
  void subscribe();

  // Documentation inherited
  void reset();

  /**
   * @brief Set ROS Subscriber topic through GUI
   * @param[in] topic_name ROS Topic Name
   */
  Q_INVOKABLE void setTopic(QString topic_name);

  /**
   * @brief Qt eventFilters. Original documentation can be found
   * <a href="https://doc.qt.io/qt-5/qobject.html#eventFilter">here</a>
   */
  bool eventFilter(QObject * _object, QEvent * _event);

  // Documentation inherited
  void setFrameManager(std::shared_ptr<common::FrameManager> _frameManager);

  /**
   * @brief Get the frame list as a string
   * @return List of frames
   */
  Q_INVOKABLE QStringList getTopicList() const;

signals:
  /**
   * @brief Notify that topic list has changed
   */
  void topicListChanged();

signals:
  /**
   * @brief Set combo box index
   * @param index Combo box index
   */
  void setCurrentIndex(const int index);

public slots:
  /**
   * @brief Callback when refresh button is pressed.
   */
  void onRefresh();

protected:
  /**
   * @brief Update laser scan visualization
   */
  void update();

private:
  ignition::rendering::AxisVisualPtr axis;
  ignition::rendering::RenderEngine * engine;
  ignition::rendering::ScenePtr scene;
  ignition::rendering::VisualPtr rootVisual;
  std::mutex lock;
  std::string fixedFrame;
  sensor_msgs::msg::LaserScan::SharedPtr msg;
  QStringList topicList;
};

}  // namespace plugins
}  // namespace rviz
}  // namespace ignition

#endif  // IGNITION__RVIZ__PLUGINS__LASERSCANDISPLAY_HPP_
