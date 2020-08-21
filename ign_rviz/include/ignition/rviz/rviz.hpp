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

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

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
  explicit TopicModel(QObject * _parent = 0)
  : QStandardItemModel(_parent) {}

  /**
   * @brief Add a topic to list view
   * @param[in] _name Topic name
   */
  Q_INVOKABLE void addTopic(const std::string & _name, const std::string & _msgType)
  {
    QStandardItem * entry = new QStandardItem();
    entry->setData(QString::fromStdString(_name), NameRole);
    entry->setData(QString::fromStdString(_msgType), TypeRole);
    appendRow(entry);
  }

  // Documentation inherited
  QVariant data(const QModelIndex & _index, int _role = Qt::DisplayRole) const
  {
    QStandardItem * myItem = itemFromIndex(_index);

    if (_role == NameRole) {
      return myItem->data(NameRole);
    }

    if (_role == TypeRole) {
      return myItem->data(TypeRole);
    }

    return QVariant();
  }

protected:
  // Documentation inherited
  QHash<int, QByteArray> roleNames() const
  {
    QHash<int, QByteArray> roles;
    roles[NameRole] = "topic";
    roles[TypeRole] = "msgType";
    return roles;
  }
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
  RViz()
  {
    this->topicModel = new TopicModel();
  }

  std::vector<std::string> supportedDisplays = {
    "geometry_msgs/msg/PointStamped",
    "geometry_msgs/msg/PolygonStamped",
    "geometry_msgs/msg/Pose",
    "geometry_msgs/msg/PoseArray",
    "nav_msgs/msg/Path",
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/LaserScan"
  };

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
  Q_INVOKABLE TopicModel * getTopicModel() const
  {
    return this->topicModel;
  }

  /**
   * @brief Refreshes the supported display topic list
   */
  Q_INVOKABLE void refreshTopicList() const
  {
    this->topicModel->removeRows(0, this->topicModel->rowCount());
    auto topics = this->node->get_topic_names_and_types();
    for (const auto & topic : topics) {
      for (const auto & topicType : topic.second) {
        if (std::find(
            supportedDisplays.begin(), supportedDisplays.end(),
            topicType) != this->supportedDisplays.end())
        {
          RCLCPP_INFO(this->node->get_logger(), "%s", topic.first.c_str());
          this->topicModel->addTopic(topic.first, topicType);
        }
      }
    }
  }

  /**
   * @brief Add Grid visual to Scene3D
   */
  Q_INVOKABLE void addGrid3D() const
  {
    ignition::gui::App()->LoadPlugin("Grid3D");
  }

  /**
   * @brief Loads TF Visualization Plugin
   */
  Q_INVOKABLE void addTFDisplay()
  {
    // Load plugin
    if (ignition::gui::App()->LoadPlugin("TFDisplay")) {
      auto tfDisplayPlugins =
        ignition::gui::App()->findChildren<DisplayPlugin<tf2_msgs::msg::TFMessage> *>();
      int tfDisplayCount = tfDisplayPlugins.size() - 1;

      // Set frame manager and install event filter for recently added plugin
      tfDisplayPlugins[tfDisplayCount]->initialize(this->node);
      tfDisplayPlugins[tfDisplayCount]->setFrameManager(this->frameManager);
      ignition::gui::App()->findChild<ignition::gui::MainWindow *>()->installEventFilter(
        tfDisplayPlugins[tfDisplayCount]);
    }
  }

  /**
   * @brief Loads LaserScan Visualization Plugin
   */
  Q_INVOKABLE void addLaserScanDisplay()
  {
    // Load plugin
    if (ignition::gui::App()->LoadPlugin("LaserScanDisplay")) {
      auto laserScanPlugin =
        ignition::gui::App()->findChildren<DisplayPlugin<sensor_msgs::msg::LaserScan> *>();
      int pluginCount = laserScanPlugin.size() - 1;

      // Set frame manager and install event filter for recently added plugin
      laserScanPlugin[pluginCount]->initialize(this->node);
      laserScanPlugin[pluginCount]->setTopic("/scan");
      laserScanPlugin[pluginCount]->setFrameManager(this->frameManager);
      ignition::gui::App()->findChild<ignition::gui::MainWindow *>()->installEventFilter(
        laserScanPlugin[pluginCount]);
    }
  }

  /**
   * @brief Loads PointStamped Visualization Plugin
   */
  Q_INVOKABLE void addPointStampedDisplay()
  {
    // Load plugin
    if (ignition::gui::App()->LoadPlugin("PointStampedDisplay")) {
      auto pointStampedPlugin =
        ignition::gui::App()->findChildren<DisplayPlugin<geometry_msgs::msg::PointStamped> *>();
      int pluginCount = pointStampedPlugin.size() - 1;

      // Set frame manager and install event filter for recently added plugin
      pointStampedPlugin[pluginCount]->initialize(this->node);
      pointStampedPlugin[pluginCount]->setTopic("/point");
      pointStampedPlugin[pluginCount]->setFrameManager(this->frameManager);
      ignition::gui::App()->findChild<ignition::gui::MainWindow *>()->installEventFilter(
        pointStampedPlugin[pluginCount]);
    }
  }


  /**
   * @brief Loads Polygon Visualization Plugin
   */
  Q_INVOKABLE void addPolygonDisplay()
  {
    // Load plugin
    if (ignition::gui::App()->LoadPlugin("PolygonDisplay")) {
      auto polygonPlugin =
        ignition::gui::App()->findChildren<DisplayPlugin<geometry_msgs::msg::PolygonStamped> *>();
      int pluginCount = polygonPlugin.size() - 1;

      // Set frame manager and install event filter for recently added plugin
      polygonPlugin[pluginCount]->initialize(this->node);
      polygonPlugin[pluginCount]->setTopic("/polygon");
      polygonPlugin[pluginCount]->setFrameManager(this->frameManager);
      ignition::gui::App()->findChild<ignition::gui::MainWindow *>()->installEventFilter(
        polygonPlugin[pluginCount]);
    }
  }

  /**
   * @brief Loads Pose Visualization Plugin
   */
  Q_INVOKABLE void addPoseDisplay()
  {
    // Load plugin
    if (ignition::gui::App()->LoadPlugin("PoseDisplay")) {
      auto posePlugin =
        ignition::gui::App()->findChildren<DisplayPlugin<geometry_msgs::msg::PoseStamped> *>();
      int pluginCount = posePlugin.size() - 1;

      // Set frame manager and install event filter for recently added plugin
      posePlugin[pluginCount]->initialize(this->node);
      posePlugin[pluginCount]->setTopic("/pose");
      posePlugin[pluginCount]->setFrameManager(this->frameManager);
      ignition::gui::App()->findChild<ignition::gui::MainWindow *>()->installEventFilter(
        posePlugin[pluginCount]);
    }
  }

  /**
   * @brief Loads PoseArray Visualization Plugin
   */
  Q_INVOKABLE void addPoseArrayDisplay()
  {
    // Load plugin
    if (ignition::gui::App()->LoadPlugin("PoseArrayDisplay")) {
      auto poseArrayPlugin =
        ignition::gui::App()->findChildren<DisplayPlugin<geometry_msgs::msg::PoseArray> *>();
      int pluginCount = poseArrayPlugin.size() - 1;

      // Set frame manager and install event filter for recently added plugin
      poseArrayPlugin[pluginCount]->initialize(this->node);
      poseArrayPlugin[pluginCount]->setTopic("/pose_array");
      poseArrayPlugin[pluginCount]->setFrameManager(this->frameManager);
      ignition::gui::App()->findChild<ignition::gui::MainWindow *>()->installEventFilter(
        poseArrayPlugin[pluginCount]);
    }
  }

  /**
   * @brief Loads Path Visualization Plugin
   */
  Q_INVOKABLE void addPathDisplay()
  {
    // Load plugin
    if (ignition::gui::App()->LoadPlugin("PathDisplay")) {
      auto pathPlugin =
        ignition::gui::App()->findChildren<DisplayPlugin<nav_msgs::msg::Path> *>();
      int pluginCount = pathPlugin.size() - 1;

      // Set frame manager and install event filter for recently added plugin
      pathPlugin[pluginCount]->initialize(this->node);
      pathPlugin[pluginCount]->setTopic("/path");
      pathPlugin[pluginCount]->setFrameManager(this->frameManager);
      ignition::gui::App()->findChild<ignition::gui::MainWindow *>()->installEventFilter(
        pathPlugin[pluginCount]);
    }
  }

  /**
   * @brief Loads RobotModel Dispaly Plugin
   */
  Q_INVOKABLE void addRobotModelDisplay()
  {
    // Load plugin
    if (ignition::gui::App()->LoadPlugin("RobotModelDisplay")) {
      auto robotModelPlugin =
        ignition::gui::App()->findChildren<DisplayPlugin<std_msgs::msg::String> *>();
      int pluginCount = robotModelPlugin.size() - 1;

      // Set frame manager and install event filter for recently added plugin
      robotModelPlugin[pluginCount]->initialize(this->node);
      robotModelPlugin[pluginCount]->setFrameManager(this->frameManager);
      robotModelPlugin[pluginCount]->setTopic("/robot_description");
      ignition::gui::App()->findChild<ignition::gui::MainWindow *>()->installEventFilter(
        robotModelPlugin[pluginCount]);
    }
  }

  /**
   * @brief Loads Image Display Plugin
   */
  Q_INVOKABLE void addImageDisplay()
  {
    // Load plugin
    if (ignition::gui::App()->LoadPlugin("ImageDisplay")) {
      auto imageDisplayPlugin =
        ignition::gui::App()->findChildren<DisplayPlugin<sensor_msgs::msg::Image> *>();
      int pluginCount = imageDisplayPlugin.size() - 1;

      // Set frame manager and install event filter for recently added plugin
      imageDisplayPlugin[pluginCount]->initialize(this->node);
      imageDisplayPlugin[pluginCount]->setTopic("/image");
    }
  }

  /**
   * @brief Loads Axes Visualization Plugin
   */
  Q_INVOKABLE void addAxesDisplay()
  {
    // Load plugin
    if (ignition::gui::App()->LoadPlugin("AxesDisplay")) {
      auto axes_plugins =
        ignition::gui::App()->findChildren<ignition::rviz::plugins::MessageDisplayBase *>();
      int axes_plugin_count = axes_plugins.size() - 1;

      // Set frame manager and install event filter for recently added plugin
      axes_plugins[axes_plugin_count]->setFrameManager(this->frameManager);
      ignition::gui::App()->findChild<ignition::gui::MainWindow *>()->installEventFilter(
        axes_plugins[axes_plugin_count]);
    }
  }

  /**
   * @brief Initialize ignition RViz ROS node and frame manager
   * @throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   */
  void init_ros()
  {
    this->node = std::make_shared<rclcpp::Node>("ignition_rviz");
    this->frameManager = std::make_shared<common::FrameManager>(this->node);
    this->frameManager->setFixedFrame("world");

    // Load Global Options plugin
    if (ignition::gui::App()->LoadPlugin("GlobalOptions")) {
      auto globalOptionsPlugin =
        ignition::gui::App()->findChild<ignition::rviz::plugins::MessageDisplayBase *>();

      // Set frame manager and install
      globalOptionsPlugin->setFrameManager(this->frameManager);

      // Install event filter
      ignition::gui::App()->findChild<ignition::gui::MainWindow *>()->installEventFilter(
        globalOptionsPlugin);
    }
  }

  /**
   * @brief Returns ignition RViz ROS node
   * @return ROS Node shared pointer
   */
  rclcpp::Node::SharedPtr get_node()
  {
    return this->node;
  }

private:
  // Data Members
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<common::FrameManager> frameManager;

  // Topic model
  TopicModel * topicModel;
};

}  // namespace rviz
}  // namespace ignition

#endif  // IGNITION__RVIZ__RVIZ_HPP_
