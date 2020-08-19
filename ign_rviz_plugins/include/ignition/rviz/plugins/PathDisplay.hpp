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

#ifndef IGNITION__RVIZ__PLUGINS__PATHDISPLAY_HPP_
#define IGNITION__RVIZ__PLUGINS__PATHDISPLAY_HPP_

#include <ignition/rendering.hh>

#include <nav_msgs/msg/path.hpp>

#include <QColor>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "ignition/rviz/plugins/message_display_base.hpp"

namespace ignition
{
namespace rviz
{
namespace plugins
{
/**
 * @brief Renders data from nav_msgs::msg::Path message as lines, arrows or axes
 */
class PathDisplay : public MessageDisplay<nav_msgs::msg::Path>
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
   * Constructor for Path visualization plugin
   */
  PathDisplay();

  // Destructor
  ~PathDisplay();

  // Documentation inherited
  void LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/) override;

  // Documentation inherited
  void initialize(rclcpp::Node::SharedPtr _node) override;

  // Documentation inherited
  void callback(const nav_msgs::msg::Path::SharedPtr _msg) override;

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

  /**
   * @brief Set path visual shape
   * @param _index Visual shape. 0: None; 1: Arrow; 2: Axes
   */
  Q_INVOKABLE void setShape(const int & _index);

  /**
   * @brief Set axis arrow head visibility
   */
  Q_INVOKABLE void setAxisHeadVisibility(const bool & _visible);

  /**
   * @brief Set axis dimensions
   * @param _length Axis length
   * @param _radius Axis radius
   */
  Q_INVOKABLE void setAxisDimensions(const float & _length, const float & _radius);

  /**
   * @brief Set arrow dimensions
   * @param _shaftLength Arrow shaft length
   * @param _shaftRadius Arrow shaft radius
   * @param _headLength Arrow head length
   * @param _headRadius Arrow head radius
   */
  Q_INVOKABLE void setArrowDimensions(
    const float & _shaftLength, const float & _shaftRadius,
    const float & _headLength, const float & _headRadius);

  /**
   * @brief Set visual color and transparency
   * @param _color Color and transparency of visual
   */
  Q_INVOKABLE void setColor(const QColor & _color);

  /**
   * @brief Set line color and transparency
   * @param _color Color and transparency of line
   */
  Q_INVOKABLE void setLineColor(const QColor & _color);

  /**
   * @brief Set path offset
   * @param _x X Offset
   * @param _y Y Offset
   * @param _z Z Offset
   */
  Q_INVOKABLE void setOffset(const float & _x, const float & _y, const float & _z);

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
   * @brief Update Path data visualization
   */
  void update() override;

  /**
   * @brief Update path visual dimensions
   * @param _index Index of path visual
   */
  void updateVisual(int _index);

private:
  ignition::rendering::RenderEngine * engine;
  ignition::rendering::ScenePtr scene;
  ignition::rendering::VisualPtr rootVisual;
  std::mutex lock;
  nav_msgs::msg::Path::SharedPtr msg;
  QStringList topicList;
  bool dirty;
  int visualShape;  // 0: None; 1: Arrow; 2: Axis
  math::Vector3d offset;

  // Arrows
  std::vector<rendering::ArrowVisualPtr> arrows;
  ignition::rendering::MaterialPtr mat;
  float shaftLength;
  float shaftRadius;
  float headLength;
  float headRadius;

  // Axes
  std::vector<rendering::AxisVisualPtr> axes;
  float axisLength;
  float axisRadius;
  bool axisHeadVisible;

  // Line
  math::Color color;
  bool createMarker;
};

}  // namespace plugins
}  // namespace rviz
}  // namespace ignition

#endif  // IGNITION__RVIZ__PLUGINS__PATHDISPLAY_HPP_
