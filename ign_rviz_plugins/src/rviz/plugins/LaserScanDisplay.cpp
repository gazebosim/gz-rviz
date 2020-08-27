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

#include <ignition/gui/Application.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/plugin/Register.hh>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace ignition
{
namespace rviz
{
namespace plugins
{
////////////////////////////////////////////////////////////////////////////////
LaserScanDisplay::LaserScanDisplay()
: MessageDisplay(), visualType(rendering::LidarVisualType::LVT_POINTS)
{
  // Get reference to scene
  this->engine = ignition::rendering::engine("ogre");
  this->scene = this->engine->SceneByName("scene");

  // Create root visual for laser scan
  this->rootVisual = this->scene->CreateLidarVisual();
  this->rootVisual->SetType(visualType);

  // Attach root visual to scene
  this->scene->RootVisual()->AddChild(this->rootVisual);
}

////////////////////////////////////////////////////////////////////////////////
LaserScanDisplay::~LaserScanDisplay()
{
  std::lock_guard<std::mutex>(this->lock);
  // Delete visual
  ignition::gui::App()->findChild<ignition::gui::MainWindow *>()->removeEventFilter(this);
  this->scene->DestroyVisual(this->rootVisual);
}

////////////////////////////////////////////////////////////////////////////////
void LaserScanDisplay::initialize(rclcpp::Node::SharedPtr _node)
{
  std::lock_guard<std::mutex>(this->lock);
  this->node = std::move(_node);
}

////////////////////////////////////////////////////////////////////////////////
void LaserScanDisplay::subscribe()
{
  std::lock_guard<std::mutex>(this->lock);

  this->subscriber = this->node->create_subscription<sensor_msgs::msg::LaserScan>(
    this->topic_name,
    this->qos,
    std::bind(&LaserScanDisplay::callback, this, std::placeholders::_1));
}

////////////////////////////////////////////////////////////////////////////////
void LaserScanDisplay::setTopic(const std::string & topic_name)
{
  std::lock_guard<std::mutex>(this->lock);
  this->topic_name = topic_name;

  this->subscribe();

  // Refresh combo-box on plugin load
  this->onRefresh();
}

////////////////////////////////////////////////////////////////////////////////
void LaserScanDisplay::setTopic(const QString & topic_name)
{
  std::lock_guard<std::mutex>(this->lock);
  this->topic_name = topic_name.toStdString();

  // Destroy previous subscription
  this->unsubscribe();
  // Reset visualization
  this->reset();
  // Create new subscription
  this->subscribe();
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
    update();
  }

  return QObject::eventFilter(_object, _event);
}

////////////////////////////////////////////////////////////////////////////////
void LaserScanDisplay::reset()
{
  if (this->rootVisual != nullptr) {
    this->rootVisual->ClearPoints();
  }
  this->msg.reset();
}

////////////////////////////////////////////////////////////////////////////////
void LaserScanDisplay::update()
{
  std::lock_guard<std::mutex>(this->lock);
  if (!this->msg) {
    return;
  }

  // Update data
  this->rootVisual->SetMinHorizontalAngle(this->msg->angle_min);
  this->rootVisual->SetMaxHorizontalAngle(this->msg->angle_max);
  this->rootVisual->SetMaxRange(this->msg->range_max);
  this->rootVisual->SetMinRange(this->msg->range_min);
  this->rootVisual->SetHorizontalRayCount(this->msg->ranges.size());
  this->rootVisual->SetType(this->visualType);
  this->rootVisual->SetPoints(
    std::vector<double>(
      this->msg->ranges.begin(),
      this->msg->ranges.end()));

  // Update visualization
  this->rootVisual->Update();

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
void LaserScanDisplay::setVisualType(const int & _type)
{
  std::lock_guard<std::mutex>(this->lock);

  // Set visual type
  switch (_type) {
    case 0: this->visualType = rendering::LidarVisualType::LVT_POINTS;
      break;
    case 1: this->visualType = rendering::LidarVisualType::LVT_RAY_LINES;
      break;
    case 2: this->visualType = rendering::LidarVisualType::LVT_TRIANGLE_STRIPS;
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////
void LaserScanDisplay::updateQoS(
  const int & _depth, const int & _history, const int & _reliability,
  const int & _durability)
{
  std::lock_guard<std::mutex>(this->lock);
  this->setHistoryDepth(_depth);
  this->setHistoryPolicy(_history);
  this->setReliabilityPolicy(_reliability);
  this->setDurabilityPolicy(_durability);

  // Resubscribe with updated QoS profile
  this->unsubscribe();
  this->reset();
  this->subscribe();
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
