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
#include <map>
#include <utility>
#include <vector>

#include "ignition/rviz/plugins/message_display_base.hpp"

namespace ignition
{
namespace rviz
{
namespace plugins
{
////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Helper class to render robot model link tree view
 */
class RobotLinkModel : public QStandardItemModel
{
  Q_OBJECT

public:
  /**
   * @brief Roles for tree view of robot model links
   */
  enum RobotLinkRoles
  {
    NameRole = Qt::UserRole + 2
  };

  // Constructor
  explicit RobotLinkModel(QObject * _parent = 0);

  /**
   * @brief Add robot link to tree view
   * @param[in] _name Robot link name
   * @param[in] _parentItem Pointer to tree view parent item
   */
  Q_INVOKABLE void addLink(const QString & _name, QStandardItem * _parentItem);

  /**
   * @brief Add a parent item to tree view
   * @param[in] _name Item name
   * @return Pointer to tree view parent item
   */
  Q_INVOKABLE QStandardItem * addParentRow(const QString & _name);

  // Documentation inherited
  QVariant data(const QModelIndex & _index, int _role = Qt::DisplayRole) const;

protected:
  // Documentation inherited
  QHash<int, QByteArray> roleNames() const;
};

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Wrapper struct for robot link visual, collision and visibility properties
 */
struct RobotLinkProperties
{
  bool visible = true;
  rendering::VisualPtr visual = nullptr;
  rendering::VisualPtr collision = nullptr;
};

////////////////////////////////////////////////////////////////////////////////
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

  /**
   * @brief Robot model link tree model
   */
  Q_PROPERTY(
    RobotLinkModel * robotLinkModel
    READ getRobotLinkModel
    NOTIFY robotLinkModelChanged
  )

public:
  // Constructor
  RobotModelDisplay();

  // Destructor
  ~RobotModelDisplay();

  // Documentation inherited
  void LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/) override;

  // Documentation inherited
  void initialize(rclcpp::Node::SharedPtr _node) override;

  // Documentation inherited
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
   * @brief Set robot link visibility
   * @param[in] _link Robot link name
   * @param[in] _visible Robot link visibility
   */
  Q_INVOKABLE void setLinkVisibility(const QString & _link, const bool & _visible);

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

  /**
   * @brief Set link visual visibility
   * @param[in] _enabled Visual visibility
   */
  Q_INVOKABLE void visualEnabled(const bool & _enabled);

  /**
   * @brief Set link collision visibility
   * @param[in] _enabled Collision visibility
   */
  Q_INVOKABLE void collisionEnabled(const bool & _enabled);

  /**
   * @brief Set robot model transparency
   * @param[in] _alpha Robot model transparency
   */
  Q_INVOKABLE void setAlpha(const float & _alpha);

  /**
   * @brief Get the tree view model
   * @return Tree view model
   */
  Q_INVOKABLE RobotLinkModel * getRobotLinkModel() const
  {
    return this->robotLinkModel;
  }

signals:
  /**
   * @brief Notify that tree view has changed
   */
  void robotLinkModelChanged();

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
   * @brief Add robot model link
   * @param[in] _link Robot Link
   */
  void addLink(const urdf::LinkSharedPtr & _link);

  /**
   * @brief Create a robot link (visual and collision)
   * @param[in] _link Robot Link
   */
  void createLink(const urdf::Link * _link);

  /**
   * @brief Create robot model visual using using geometry information
   * @param[in] _geometry Link goemetry information
   * @return Link visual with described geometry
   */
  rendering::VisualPtr createLinkGeometry(const urdf::GeometrySharedPtr & _geometry);

public:
  // Tree view robot link model
  RobotLinkModel * robotLinkModel;

private:
  std::recursive_mutex lock;
  ignition::rendering::RenderEngine * engine;
  ignition::rendering::ScenePtr scene;
  ignition::rendering::VisualPtr rootVisual;
  std::map<std::string, RobotLinkProperties> robotVisualLinks;
  std_msgs::msg::String::SharedPtr msg;
  QStringList topicList;
  urdf::Model robotModel;
  bool modelLoaded;
  bool destroyModel;
  bool showVisual;
  bool showCollision;
  bool dirty;
  float alpha;
  QStandardItem * parentRow;
};

}  // namespace plugins
}  // namespace rviz
}  // namespace ignition

#endif  // IGNITION__RVIZ__PLUGINS__ROBOTMODELDISPLAY_HPP_
