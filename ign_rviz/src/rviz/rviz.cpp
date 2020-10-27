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

#include "ignition/rviz/rviz.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>
#include <vector>

namespace ignition
{
namespace rviz
{
////////////////////////////////////////////////////////////////////////////////
TopicModel::TopicModel(QObject * _parent)
: QStandardItemModel(_parent) {}

////////////////////////////////////////////////////////////////////////////////
void TopicModel::addTopic(const std::string & _name, const std::string & _msgType)
{
  QStandardItem * entry = new QStandardItem();
  entry->setData(QString::fromStdString(_name), NameRole);
  entry->setData(QString::fromStdString(_msgType), TypeRole);
  appendRow(entry);
}

////////////////////////////////////////////////////////////////////////////////
QVariant TopicModel::data(const QModelIndex & _index, int _role) const
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

////////////////////////////////////////////////////////////////////////////////
QHash<int, QByteArray> TopicModel::roleNames() const
{
  QHash<int, QByteArray> roles;
  roles[NameRole] = "topic";
  roles[TypeRole] = "msgType";
  return roles;
}

////////////////////////////////////////////////////////////////////////////////
RViz::RViz()
{
  this->topicModel = new TopicModel();

  this->supportedDisplays = {
    "geometry_msgs/msg/PointStamped",
    "geometry_msgs/msg/PolygonStamped",
    "geometry_msgs/msg/PoseStamped",
    "geometry_msgs/msg/PoseArray",
    "nav_msgs/msg/Path",
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/LaserScan",
    "sensor_msgs/msg/NavSatFix",
    "visualization_msgs/msg/Marker",
    "visualization_msgs/msg/MarkerArray"
  };
}

////////////////////////////////////////////////////////////////////////////////
TopicModel * RViz::getTopicModel() const
{
  return this->topicModel;
}

////////////////////////////////////////////////////////////////////////////////
void RViz::refreshTopicList() const
{
  this->topicModel->removeRows(0, this->topicModel->rowCount());
  auto topics = this->node->get_topic_names_and_types();
  for (const auto & topic : topics) {
    for (const auto & topicType : topic.second) {
      if (std::find(
          supportedDisplays.begin(), supportedDisplays.end(),
          topicType) != this->supportedDisplays.end())
      {
        this->topicModel->addTopic(topic.first, topicType);
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void RViz::addGrid3D() const
{
  ignition::gui::App()->LoadPlugin("Grid3D");
}

////////////////////////////////////////////////////////////////////////////////
void RViz::addTFDisplay() const
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

////////////////////////////////////////////////////////////////////////////////
void RViz::addLaserScanDisplay(const QString & _topic) const
{
  // Load plugin
  if (ignition::gui::App()->LoadPlugin("LaserScanDisplay")) {
    auto laserScanPlugin =
      ignition::gui::App()->findChildren<DisplayPlugin<sensor_msgs::msg::LaserScan> *>();
    int pluginCount = laserScanPlugin.size() - 1;

    // Set frame manager and install event filter for recently added plugin
    laserScanPlugin[pluginCount]->initialize(this->node);
    laserScanPlugin[pluginCount]->setTopic(_topic.toStdString());
    laserScanPlugin[pluginCount]->setFrameManager(this->frameManager);
    ignition::gui::App()->findChild<ignition::gui::MainWindow *>()->installEventFilter(
      laserScanPlugin[pluginCount]);
  }
}
////////////////////////////////////////////////////////////////////////////////
void RViz::addGPSDisplay(const QString & _topic) const
{
  // Load plugin
  if (ignition::gui::App()->LoadPlugin("GPSDisplay")) {
    auto gpsDisplay =
      ignition::gui::App()->findChildren<DisplayPlugin<sensor_msgs::msg::NavSatFix> *>();
    int pluginCount = gpsDisplay.size() - 1;

    // Set frame manager and install event filter for recently added plugin
    gpsDisplay[pluginCount]->initialize(this->node);
    gpsDisplay[pluginCount]->setTopic(_topic.toStdString());
  }
}

////////////////////////////////////////////////////////////////////////////////
void RViz::addMarkerDisplay(const QString & _topic) const
{
  // Load plugin
  if (ignition::gui::App()->LoadPlugin("MarkerDisplay")) {
    auto markerDisplay =
      ignition::gui::App()->findChildren<DisplayPlugin<visualization_msgs::msg::Marker> *>();
    int pluginCount = markerDisplay.size() - 1;

    // Set frame manager and install event filter for recently added plugin
    markerDisplay[pluginCount]->initialize(this->node);
    markerDisplay[pluginCount]->setTopic(_topic.toStdString());
    markerDisplay[pluginCount]->setFrameManager(this->frameManager);
    ignition::gui::App()->findChild<ignition::gui::MainWindow *>()->installEventFilter(
      markerDisplay[pluginCount]);
  }
}

////////////////////////////////////////////////////////////////////////////////
void RViz::addMarkerArrayDisplay(const QString & _topic) const
{
  // Load plugin
  if (ignition::gui::App()->LoadPlugin("MarkerArrayDisplay")) {
    auto markerArrayDisplay =
      ignition::gui::App()->findChildren<DisplayPlugin<visualization_msgs::msg::MarkerArray> *>();
    int pluginCount = markerArrayDisplay.size() - 1;

    // Set frame manager and install event filter for recently added plugin
    markerArrayDisplay[pluginCount]->initialize(this->node);
    markerArrayDisplay[pluginCount]->setTopic(_topic.toStdString());
    markerArrayDisplay[pluginCount]->setFrameManager(this->frameManager);
    ignition::gui::App()->findChild<ignition::gui::MainWindow *>()->installEventFilter(
      markerArrayDisplay[pluginCount]);
  }
}

////////////////////////////////////////////////////////////////////////////////
void RViz::addPointStampedDisplay(const QString & _topic) const
{
  // Load plugin
  if (ignition::gui::App()->LoadPlugin("PointStampedDisplay")) {
    auto pointStampedPlugin =
      ignition::gui::App()->findChildren<DisplayPlugin<geometry_msgs::msg::PointStamped> *>();
    int pluginCount = pointStampedPlugin.size() - 1;

    // Set frame manager and install event filter for recently added plugin
    pointStampedPlugin[pluginCount]->initialize(this->node);
    pointStampedPlugin[pluginCount]->setTopic(_topic.toStdString());
    pointStampedPlugin[pluginCount]->setFrameManager(this->frameManager);
    ignition::gui::App()->findChild<ignition::gui::MainWindow *>()->installEventFilter(
      pointStampedPlugin[pluginCount]);
  }
}

////////////////////////////////////////////////////////////////////////////////
void RViz::addPolygonDisplay(const QString & _topic) const
{
  // Load plugin
  if (ignition::gui::App()->LoadPlugin("PolygonDisplay")) {
    auto polygonPlugin =
      ignition::gui::App()->findChildren<DisplayPlugin<geometry_msgs::msg::PolygonStamped> *>();
    int pluginCount = polygonPlugin.size() - 1;

    // Set frame manager and install event filter for recently added plugin
    polygonPlugin[pluginCount]->initialize(this->node);
    polygonPlugin[pluginCount]->setTopic(_topic.toStdString());
    polygonPlugin[pluginCount]->setFrameManager(this->frameManager);
    ignition::gui::App()->findChild<ignition::gui::MainWindow *>()->installEventFilter(
      polygonPlugin[pluginCount]);
  }
}

////////////////////////////////////////////////////////////////////////////////
void RViz::addPoseDisplay(const QString & _topic) const
{
  // Load plugin
  if (ignition::gui::App()->LoadPlugin("PoseDisplay")) {
    auto posePlugin =
      ignition::gui::App()->findChildren<DisplayPlugin<geometry_msgs::msg::PoseStamped> *>();
    int pluginCount = posePlugin.size() - 1;

    // Set frame manager and install event filter for recently added plugin
    posePlugin[pluginCount]->initialize(this->node);
    posePlugin[pluginCount]->setTopic(_topic.toStdString());
    posePlugin[pluginCount]->setFrameManager(this->frameManager);
    ignition::gui::App()->findChild<ignition::gui::MainWindow *>()->installEventFilter(
      posePlugin[pluginCount]);
  }
}

////////////////////////////////////////////////////////////////////////////////
void RViz::addPoseArrayDisplay(const QString & _topic) const
{
  // Load plugin
  if (ignition::gui::App()->LoadPlugin("PoseArrayDisplay")) {
    auto poseArrayPlugin =
      ignition::gui::App()->findChildren<DisplayPlugin<geometry_msgs::msg::PoseArray> *>();
    int pluginCount = poseArrayPlugin.size() - 1;

    // Set frame manager and install event filter for recently added plugin
    poseArrayPlugin[pluginCount]->initialize(this->node);
    poseArrayPlugin[pluginCount]->setTopic(_topic.toStdString());
    poseArrayPlugin[pluginCount]->setFrameManager(this->frameManager);
    ignition::gui::App()->findChild<ignition::gui::MainWindow *>()->installEventFilter(
      poseArrayPlugin[pluginCount]);
  }
}

////////////////////////////////////////////////////////////////////////////////
void RViz::addPathDisplay(const QString & _topic) const
{
  // Load plugin
  if (ignition::gui::App()->LoadPlugin("PathDisplay")) {
    auto pathPlugin =
      ignition::gui::App()->findChildren<DisplayPlugin<nav_msgs::msg::Path> *>();
    int pluginCount = pathPlugin.size() - 1;

    // Set frame manager and install event filter for recently added plugin
    pathPlugin[pluginCount]->initialize(this->node);
    pathPlugin[pluginCount]->setTopic(_topic.toStdString());
    pathPlugin[pluginCount]->setFrameManager(this->frameManager);
    ignition::gui::App()->findChild<ignition::gui::MainWindow *>()->installEventFilter(
      pathPlugin[pluginCount]);
  }
}

////////////////////////////////////////////////////////////////////////////////
void RViz::addRobotModelDisplay() const
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

////////////////////////////////////////////////////////////////////////////////
void RViz::addImageDisplay(const QString & _topic) const
{
  // Load plugin
  if (ignition::gui::App()->LoadPlugin("ImageDisplay")) {
    auto imageDisplayPlugin =
      ignition::gui::App()->findChildren<DisplayPlugin<sensor_msgs::msg::Image> *>();
    int pluginCount = imageDisplayPlugin.size() - 1;

    // Set frame manager and install event filter for recently added plugin
    imageDisplayPlugin[pluginCount]->initialize(this->node);
    imageDisplayPlugin[pluginCount]->setTopic(_topic.toStdString());
  }
}

////////////////////////////////////////////////////////////////////////////////
void RViz::addAxesDisplay() const
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

////////////////////////////////////////////////////////////////////////////////
void RViz::init_ros()
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

////////////////////////////////////////////////////////////////////////////////
rclcpp::Node::SharedPtr RViz::get_node()
{
  return this->node;
}

}  // namespace rviz
}  // namespace ignition
