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

#include "ignition/rviz/plugins/laser_scan_display.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <ignition/math.hh>
#include <ignition/math/Color.hh>
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

  // Attach a point geometry to root visual
  rendering::MarkerPtr marker = this->scene->CreateMarker();
  marker->SetType(rendering::MarkerType::MT_POINTS);

  this->rootVisual->AddGeometry(marker);
  this->rootVisual->SetGeometryMaterial(this->scene->Material("Default/White"), false);

  // Attach root visual to scene
  this->scene->RootVisual()->AddChild(this->rootVisual);
}

////////////////////////////////////////////////////////////////////////////////
LaserScanDisplay::~LaserScanDisplay()
{}

////////////////////////////////////////////////////////////////////////////////
void LaserScanDisplay::initialize(rclcpp::Node::SharedPtr node)
{
  this->node = std::move(node);
}

////////////////////////////////////////////////////////////////////////////////
void LaserScanDisplay::setTopic(std::string topic_name)
{
  this->topic_name = topic_name;
  this->subscriber = this->node->create_subscription<sensor_msgs::msg::LaserScan>(
    this->topic_name,
    10,
    std::bind(&LaserScanDisplay::callback, this, std::placeholders::_1));
}

////////////////////////////////////////////////////////////////////////////////
void LaserScanDisplay::callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex>(this->lock);
  this->msg = std::move(msg);
}

////////////////////////////////////////////////////////////////////////////////
/**
 * Update laser scan visualization only when ign::gui render event is received
 */
bool LaserScanDisplay::eventFilter(QObject * object, QEvent * event)
{
  if (event->type() == gui::events::Render::kType) {
    update();
  }

  return QObject::eventFilter(object, event);
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
void LaserScanDisplay::installEventFilter(ignition::gui::MainWindow * window)
{
  window->installEventFilter(this);
}

////////////////////////////////////////////////////////////////////////////////
void LaserScanDisplay::setFrameManager(std::shared_ptr<common::FrameManager> frameManager)
{
  this->frameManager = std::move(frameManager);
  this->fixedFrame = this->frameManager->getFixedFrame();
}

}  // namespace plugins
}  // namespace rviz
}  // namespace ignition

PLUGINLIB_EXPORT_CLASS(
  ignition::rviz::plugins::LaserScanDisplay,
  ignition::rviz::plugins::MessageDisplayBase)
