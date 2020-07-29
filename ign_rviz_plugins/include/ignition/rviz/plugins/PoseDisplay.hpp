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

#ifndef IGNITION__RVIZ__PLUGINS__POSEDISPLAY_HPP_
#define IGNITION__RVIZ__PLUGINS__POSEDISPLAY_HPP_

#include <ignition/rendering.hh>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <QColor>

#include <deque>
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
 * @brief Helper for visualization PoseStamped data as Axis
 */
struct AxisVisualPrivate
{
  /**
   * @brief Update the axis visual length, radius and head visibility
   */
  void updateVisual()
  {
    for (int i = 0; i < 3; ++i) {
      auto arrow = std::dynamic_pointer_cast<rendering::ArrowVisual>(visual->ChildByIndex(i));
      arrow->SetLocalScale(radius * 20, radius * 20, length * 2);
    }
    visual->ShowAxisHead(this->headVisible);
  }

  ignition::rendering::AxisVisualPtr visual;
  float length = 1.0;
  float radius = 0.1;
  bool headVisible = false;
};

/**
 * @brief Renders data from geometry_msgs::msg::PoseStamped message as arrows or axes
 */
class PoseDisplay : public MessageDisplay<geometry_msgs::msg::PoseStamped>
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
   * Constructor for PoseStamped visualization plugin
   */
  PoseDisplay();

  // Destructor
  ~PoseDisplay();

  // Documentation Inherited
  void LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/) override;

  // Documentation Inherited
  void initialize(rclcpp::Node::SharedPtr _node) override;

  // Documentation Inherited
  void callback(const geometry_msgs::msg::PoseStamped::SharedPtr _msg) override;

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
   * @brief Set pose visual shape
   * @param _shape Visual shape. True: Arrow; False Axis
   */
  Q_INVOKABLE void setShape(const bool & _shape);

  /**
   * @brief Set axis arrow head visibility
   */
  Q_INVOKABLE void setAxisHeadVisibility(const bool & _visible);

  /**
   * @brief Set axis length
   * @param _length Axis length
   */
  Q_INVOKABLE void setAxisLength(const float & _length);

  /**
   * @brief Set axis radius
   * @param _radius Axis radius
   */
  Q_INVOKABLE void setAxisRadius(const float & _radius);

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
   * @brief Update PoseStamped data visualization
   */
  void update() override;

private:
  ignition::rendering::RenderEngine * engine;
  ignition::rendering::ScenePtr scene;
  ignition::rendering::VisualPtr rootVisual;
  ignition::rendering::ArrowVisualPtr arrow;
  ignition::rendering::MaterialPtr mat;
  std::mutex lock;
  geometry_msgs::msg::PoseStamped::SharedPtr msg;
  QStringList topicList;
  AxisVisualPrivate axis;
  bool visualShape;  // True: Arrow; False: Axis
  bool dirty;
};

}  // namespace plugins
}  // namespace rviz
}  // namespace ignition

#endif  // IGNITION__RVIZ__PLUGINS__POSEDISPLAY_HPP_
