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

#include <ignition/rviz/plugins/message_display_base.hpp>

#include <memory>
#include <string>
#include <vector>

namespace ignition
{
namespace rviz
{
template<typename MessageType>
using DisplayPlugin = plugins::MessageDisplay<MessageType>;

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Helper class to render ros topics in a list
 */
class TopicModel : public QStandardItemModel
{
  Q_OBJECT

public:
  /**
   * @brief Roles for topics
   */
  enum TopicRoles
  {
    NameRole = Qt::UserRole + 3,
    TypeRole
  };

  // Constructor
  explicit TopicModel(QObject * _parent = 0);

  /**
   * @brief Add a topic to list view
   * @param[in] _name Topic name
   * @param[in] _msgType Message type
   */
  Q_INVOKABLE void addTopic(const std::string & _name, const std::string & _msgType);

  // Documentation inherited
  QVariant data(const QModelIndex & _index, int _role = Qt::DisplayRole) const;

protected:
  // Documentation inherited
  QHash<int, QByteArray> roleNames() const;
};

////////////////////////////////////////////////////////////////////////////////
class RViz : public QObject
{
  Q_OBJECT

  /**
   * @brief Topic model
   */
  Q_PROPERTY(
    TopicModel * topicModel
    READ getTopicModel
    NOTIFY topicModelChanged
  )

public:
  RViz();

signals:
  /**
   * @brief Notify topic model has changed
   */
  void topicModelChanged();

public:
  /**
   * @brief Get topic model
   * @return Topic model
   */
  Q_INVOKABLE TopicModel * getTopicModel() const;

  /**
   * @brief Refreshes the supported display topic list
   */
  Q_INVOKABLE void refreshTopicList() const;

  /**
   * @brief Add Grid visual to Scene3D
   */
  Q_INVOKABLE void addGrid3D() const;

  /**
   * @brief Loads TF Visualization Plugin
   */
  Q_INVOKABLE void addTFDisplay() const;

  /**
   * @brief Loads LaserScan Visualization Plugin
   * @param[in] _topic Topic name
   */
  Q_INVOKABLE void addLaserScanDisplay(const QString & _topic = "/scan") const;

  /**
   * @brief Loads GPS Visualization Plugin
   * @param[in] _topic Topic name
   */
  Q_INVOKABLE void addGPSDisplay(const QString & _topic = "/gps") const;

  /**
   * @brief Loads Marker Visualization Plugin
   * @param[in] _topic Topic name
   */
  Q_INVOKABLE void addMarkerDisplay(const QString & _topic = "/marker") const;

  /**
   * @brief Loads MarkerArray Visualization Plugin
   * @param[in] _topic Topic name
   */
  Q_INVOKABLE void addMarkerArrayDisplay(const QString & _topic = "/marker_array") const;

  /**
   * @brief Loads PointStamped Visualization Plugin
   * @param[in] _topic Topic name
   */
  Q_INVOKABLE void addPointStampedDisplay(const QString & _topic = "/point") const;

  /**
   * @brief Loads Polygon Visualization Plugin
   * @param[in] _topic Topic name
   */
  Q_INVOKABLE void addPolygonDisplay(const QString & _topic = "/polygon") const;

  /**
   * @brief Loads Pose Visualization Plugin
   * @param[in] _topic Topic name
   */
  Q_INVOKABLE void addPoseDisplay(const QString & _topic = "/pose") const;

  /**
   * @brief Loads PoseArray Visualization Plugin
   * @param[in] _topic Topic name
   */
  Q_INVOKABLE void addPoseArrayDisplay(const QString & _topic = "/pose_array") const;

  /**
   * @brief Loads Path Visualization Plugin
   * @param[in] _topic Topic name
   */
  Q_INVOKABLE void addPathDisplay(const QString & _topic = "/path") const;

  /**
   * @brief Loads RobotModel Dispaly Plugin
   */
  Q_INVOKABLE void addRobotModelDisplay() const;

  /**
   * @brief Loads Image Display Plugin
   * @param[in] _topic Topic name
   */
  Q_INVOKABLE void addImageDisplay(const QString & _topic = "/image") const;

  /**
   * @brief Loads Axes Visualization Plugin
   */
  Q_INVOKABLE void addAxesDisplay() const;

  /**
   * @brief Initialize ignition RViz ROS node and frame manager
   * @throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   */
  void init_ros();

  /**
   * @brief Returns ignition RViz ROS node
   * @return ROS Node shared pointer
   */
  rclcpp::Node::SharedPtr get_node();

private:
  // Data Members
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<common::FrameManager> frameManager;
  std::vector<std::string> supportedDisplays;

  // Topic model
  TopicModel * topicModel;
};

}  // namespace rviz
}  // namespace ignition

#endif  // IGNITION__RVIZ__RVIZ_HPP_
