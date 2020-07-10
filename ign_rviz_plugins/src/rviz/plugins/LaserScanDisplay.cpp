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

#include "ignition/rviz/plugins/LaserScanDisplay.hpp"

#include <ignition/math.hh>
#include <ignition/math/Color.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/GuiEvents.hh>

#include <string>
#include <utility>
#include <memory>
#include <vector>

namespace ignition
{
namespace rviz
{
namespace plugins
{
////////////////////////////////////////////////////////////////////////////////
LaserScanDisplay::LaserScanDisplay()
: MessageDisplay()
{
  // Get reference to scene
  this->engine = ignition::rendering::engine("ogre");
  this->scene = this->engine->SceneByName("scene");

  // Create root visual for laser scan
  this->rootVisual = this->scene->CreateVisual();

  // Attach root visual to scene
  this->scene->RootVisual()->AddChild(this->rootVisual);
}

////////////////////////////////////////////////////////////////////////////////
LaserScanDisplay::~LaserScanDisplay()
{
  std::lock_guard<std::mutex>(this->lock);
  // Delete visual
  ignition::gui::App()->findChild<ignition::gui::MainWindow *>()->removeEventFilter(this);
  this->scene->DestroyVisual(this->rootVisual, true);
}

////////////////////////////////////////////////////////////////////////////////
void LaserScanDisplay::initialize(rclcpp::Node::SharedPtr _node)
{
  std::lock_guard<std::mutex>(this->lock);
  this->node = std::move(_node);
}

////////////////////////////////////////////////////////////////////////////////
void LaserScanDisplay::setTopic(std::string topic_name)
{
  std::lock_guard<std::mutex>(this->lock);
  this->topic_name = topic_name;
  this->subscriber = this->node->create_subscription<sensor_msgs::msg::LaserScan>(
    this->topic_name,
    1,
    std::bind(&LaserScanDisplay::callback, this, std::placeholders::_1));

  // Refresh combo-box on plugin load
  this->onRefresh();
}

////////////////////////////////////////////////////////////////////////////////
void LaserScanDisplay::setTopic(QString topic_name)
{
  std::lock_guard<std::mutex>(this->lock);
  this->topic_name = topic_name.toStdString();

  // Destroy previous subscription
  if (this->subscriber != nullptr) {
    this->subscriber.reset();
  }

  // Create new subscription
  this->subscriber = this->node->create_subscription<sensor_msgs::msg::LaserScan>(
    this->topic_name,
    10,
    std::bind(&LaserScanDisplay::callback, this, std::placeholders::_1));
}

////////////////////////////////////////////////////////////////////////////////
void LaserScanDisplay::callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg)
{
  std::lock_guard<std::mutex>(this->lock);
  this->msg = std::move(_msg);
}

////////////////////////////////////////////////////////////////////////////////
/**
 * Update laser scan visualization only when ign::gui render event is received
 */
bool LaserScanDisplay::eventFilter(QObject * _object, QEvent * _event)
{
  if (_event->type() == gui::events::Render::kType) {
    std::lock_guard<std::mutex>(this->lock);
    // Attach a point geometry to root visual
    if (static_cast<int>(this->rootVisual->GeometryCount()) == 0) {
      rendering::MarkerPtr marker = this->scene->CreateMarker();
      marker->SetType(rendering::MarkerType::MT_POINTS);

      this->rootVisual->AddGeometry(marker);
      this->rootVisual->SetGeometryMaterial(this->scene->Material("Default/White"), true);
    }

    update();
  }

  return QObject::eventFilter(_object, _event);
}

////////////////////////////////////////////////////////////////////////////////
void LaserScanDisplay::update()
{
  std::lock_guard<std::mutex>(this->lock);
  if (!this->msg) {
    return;
  }

  rendering::MarkerPtr marker = std::dynamic_pointer_cast<rendering::Marker>(
    this->rootVisual->GeometryByIndex(0));

  // Clear all points
  marker->ClearPoints();

  // Iterate through laser scan ranges and add points
  for (int i = 0; i < static_cast<int>(this->msg->ranges.size()); ++i) {
    float angle = this->msg->angle_min + (i * this->msg->angle_increment);
    marker->AddPoint(
      (this->msg->range_min + this->msg->ranges[i]) * std::cos(angle),
      (this->msg->range_min + this->msg->ranges[i]) * std::sin(angle),
      0, ignition::math::Color::White);
  }

  // Set position and orientation of the frame link
  math::Pose3d pose;
  bool poseAvailable = this->frameManager->getFramePose(this->msg->header.frame_id, pose);
  if (poseAvailable) {
    this->rootVisual->SetLocalPose(pose);
  }
}

////////////////////////////////////////////////////////////////////////////////
void LaserScanDisplay::setFrameManager(std::shared_ptr<common::FrameManager> _frameManager)
{
  std::lock_guard<std::mutex>(this->lock);
  this->frameManager = std::move(_frameManager);
  this->fixedFrame = this->frameManager->getFixedFrame();
}


////////////////////////////////////////////////////////////////////////////////
QStringList LaserScanDisplay::getTopicList() const
{
  return this->topicList;
}

////////////////////////////////////////////////////////////////////////////////
void LaserScanDisplay::onRefresh()
{
  std::lock_guard<std::mutex>(this->lock);

  // Clear
  this->topicList.clear();

  int index = 0, position = 0;

  // Get topic list
  auto topics = this->node->get_topic_names_and_types();
  for (const auto & topic : topics) {
    for (const auto & topicType : topic.second) {
      if (topicType == "sensor_msgs/msg/LaserScan") {
        this->topicList.push_back(QString::fromStdString(topic.first));
        if (topic.first == this->topic_name) {
          position = index;
        }
        index++;
      }
    }
  }
  // Update combo-box
  this->topicListChanged();
  emit setCurrentIndex(position);
}

////////////////////////////////////////////////////////////////////////////////
void LaserScanDisplay::LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/)
{
  if (this->title.empty()) {
    this->title = "Laser Scan";
  }
}

}  // namespace plugins
}  // namespace rviz
}  // namespace ignition

IGNITION_ADD_PLUGIN(
  ignition::rviz::plugins::LaserScanDisplay,
  ignition::gui::Plugin)
