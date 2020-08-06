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

#ifndef IGNITION__RVIZ__PLUGINS__ROBOTMODELDISPLAY_HPP_
#define IGNITION__RVIZ__PLUGINS__ROBOTMODELDISPLAY_HPP_

#include <urdf/model.h>

#include <ignition/rendering.hh>

#include <std_msgs/msg/string.hpp>

#include <QString>

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ignition/rviz/plugins/message_display_base.hpp"

namespace ignition
{
namespace rviz
{
namespace plugins
{
/**
 * @brief RobotModelDisplay plugin renders robot model
 */
class RobotModelDisplay : public MessageDisplay<std_msgs::msg::String>
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
  // Constructor
  RobotModelDisplay();

  // Destructor
  ~RobotModelDisplay();

  // Documentation Inherited
  void LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/) override;

  // Documentation Inherited
  void initialize(rclcpp::Node::SharedPtr _node) override;

  // Documentation Inherited
  void callback(const std_msgs::msg::String::SharedPtr _msg) override;

  // Documentation inherited
  void setTopic(const std::string & topic_name) override;

  // Documentation inherited
  void subscribe() override;

  // Documentation inherited
  void reset() override;

  // Documentation inherited
  void update() override;

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

public slots:
  /**
   * @brief Callback when refresh button is pressed.
   */
  void onRefresh();

  /**
   * @brief Get the topic list as a string
   * @return List of topics
   */
  Q_INVOKABLE QStringList getTopicList() const;

  /**
   * @brief Load RobotModel from file
   * @param[in] _file Robot model file URI
   */
  Q_INVOKABLE void openFile(const QString & _file);

  /**
   * @brief Callback for description source change
   * @param[in] _source Index of source. 0: Topic, 1: File
   */
  Q_INVOKABLE void sourceChanged(const int & _source);

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

private:
  /**
   * @brief Render robot model by reading the data
   */
  void loadRobotModel();

  /**
   * @brief Create robot model link
   * @param[in] _link Link to be created
   */
  void createLink(const urdf::LinkSharedPtr & _link);

  /**
   * @brief Create geometry for link's visual element
   * @param[in] _link Link with visual information
   */
  void addLinkVisual(const urdf::Link * _link);

private:
  std::recursive_mutex lock;
  ignition::rendering::RenderEngine * engine;
  ignition::rendering::ScenePtr scene;
  ignition::rendering::VisualPtr rootVisual;
  std::unordered_map<std::string, rendering::VisualPtr> robotVisualLinks;
  std_msgs::msg::String::SharedPtr msg;
  QStringList topicList;
  urdf::Model robotModel;
  bool modelLoaded;
  bool destroyModel;
};

}  // namespace plugins
}  // namespace rviz
}  // namespace ignition

#endif  // IGNITION__RVIZ__PLUGINS__ROBOTMODELDISPLAY_HPP_
