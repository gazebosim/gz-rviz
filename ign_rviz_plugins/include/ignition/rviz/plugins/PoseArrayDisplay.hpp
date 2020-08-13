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

#ifndef IGNITION__RVIZ__PLUGINS__POSEARRAYDISPLAY_HPP_
#define IGNITION__RVIZ__PLUGINS__POSEARRAYDISPLAY_HPP_

#include <ignition/rendering.hh>

#include <geometry_msgs/msg/pose_array.hpp>

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
struct PoseArrayVisual
{
  void updateVisual(int _index)
  {
    // Update Arrow
    arrows[_index]->Shaft()->SetLocalScale(shaftRadius * 2.0, shaftRadius * 2.0, shaftLength);
    arrows[_index]->SetOrigin(0, 0, -shaftLength);
    arrows[_index]->Head()->SetLocalScale(headRadius * 2.0, headRadius * 2.0, headLength * 2.0);

    // Update Axis
    for (int i = 0; i < 3; ++i) {
      auto arrow = std::dynamic_pointer_cast<rendering::ArrowVisual>(axes[_index]->ChildByIndex(i));
      arrow->SetLocalScale(axisRadius * 20, axisRadius * 20, axisLength * 2);
    }
  }

  bool visualShape{true};  // True: Arrow; False: Axis

  // Arrow
  std::vector<rendering::ArrowVisualPtr> arrows;
  // ignition::rendering::MaterialPtr mat;
  float shaftLength = 0.23;
  float shaftRadius = 0.01;
  float headLength = 0.07;
  float headRadius = 0.03;

  // Axes
  std::vector<rendering::AxisVisualPtr> axes;
  float axisLength = 0.3;
  float axisRadius = 0.03;
  bool axisHeadVisisble = false;
};

/**
 * @brief Renders data from geometry_msgs::msg::PoseArray message as arrows or axes
 */
class PoseArrayDisplay : public MessageDisplay<geometry_msgs::msg::PoseArray>
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
   * Constructor for PoseArray visualization plugin
   */
  PoseArrayDisplay();

  // Destructor
  ~PoseArrayDisplay();

  // Documentation inherited
  void LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/) override;

  // Documentation inherited
  void initialize(rclcpp::Node::SharedPtr _node) override;

  // Documentation inherited
  void callback(const geometry_msgs::msg::PoseArray::SharedPtr _msg) override;

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
   * @brief Set pose array visual shape
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
   * @param _radius Axis radius
   */
  Q_INVOKABLE void setAxisDimentions(const float & _length, const float & _radius);

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
   * @brief Update PoseArray data visualization
   */
  void update() override;

private:
  ignition::rendering::RenderEngine * engine;
  ignition::rendering::ScenePtr scene;
  ignition::rendering::VisualPtr rootVisual;
  std::mutex lock;
  geometry_msgs::msg::PoseArray::SharedPtr msg;
  QStringList topicList;
  PoseArrayVisual poseArrayVisual;
  bool dirty;
};

}  // namespace plugins
}  // namespace rviz
}  // namespace ignition

#endif  // IGNITION__RVIZ__PLUGINS__POSEARRAYDISPLAY_HPP_
