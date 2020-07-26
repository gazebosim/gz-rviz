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
/**
 * @brief Renders data from sensor_msgs::msg::LaserScanDisplay message as points in the world,
 * drawn as points, rays and triangles.
 */
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
  void LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/) override;

  // Documentation Inherited
  void initialize(rclcpp::Node::SharedPtr _node) override;

  // Documentation Inherited
  void callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg) override;

  // Documentation inherited
  void setTopic(const std::string & topic_name) override;

  // Documentation inherited
  void subscribe() override;

  // Documentation inherited
  void reset() override;

  /**
   * @brief Set ROS Subscriber topic through GUI
   * @param[in] topic_name ROS Topic Name
   */
  Q_INVOKABLE void setTopic(const QString & topic_name);

  /**
   * @brief Set visual type of LidarVisual
   * @param[in] _type Index of selected visual type
   */
  Q_INVOKABLE void setVisualType(const int & _type);

  /**
   * @brief Update subscription Quality of Service
   * @param[in] _depth Queue size of keep last history policy
   * @param[in] _history Index of history policy
   * @param[in] _reliability Index of reliability policy
   * @param[in] _durability Index of durability policy
   */
  Q_INVOKABLE void updateQoS(
    const int & _depth, const int & _history, const int & _reliability,
    const int & _durability);

  /**
   * @brief Qt eventFilters. Original documentation can be found
   * <a href="https://doc.qt.io/qt-5/qobject.html#eventFilter">here</a>
   */
  bool eventFilter(QObject * _object, QEvent * _event);

  // Documentation inherited
  void setFrameManager(std::shared_ptr<common::FrameManager> _frameManager) override;

  /**
   * @brief Get the topic list as a string
   * @return List of topics
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
  ignition::rendering::RenderEngine * engine;
  ignition::rendering::ScenePtr scene;
  ignition::rendering::LidarVisualPtr rootVisual;
  std::mutex lock;
  std::string fixedFrame;
  sensor_msgs::msg::LaserScan::SharedPtr msg;
  QStringList topicList;
  enum rendering::LidarVisualType visualType;
};

}  // namespace plugins
}  // namespace rviz
}  // namespace ignition

#endif  // IGNITION__RVIZ__PLUGINS__LASERSCANDISPLAY_HPP_
