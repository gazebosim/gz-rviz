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

#include "ignition/rviz/plugins/PointStampedDisplay.hpp"

#include <ignition/gui/Application.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/plugin/Register.hh>

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
PointStampedDisplay::PointStampedDisplay()
: MessageDisplay(), historyLength(5), radius(0.2)
{
  // Get reference to scene
  this->engine = ignition::rendering::engine("ogre");
  this->scene = this->engine->SceneByName("scene");

  this->rootVisual = this->scene->CreateVisual();
  this->scene->RootVisual()->AddChild(this->rootVisual);

  this->mat = this->scene->CreateMaterial();
  this->mat->SetAmbient(0.8, 0.161, 0.8);
  this->mat->SetDiffuse(0.8, 0.161, 0.8);
  this->mat->SetEmissive(0.8, 0.161, 0.8);
}

////////////////////////////////////////////////////////////////////////////////
PointStampedDisplay::~PointStampedDisplay()
{
  std::lock_guard<std::mutex>(this->lock);
  // Delete visual
  ignition::gui::App()->findChild<ignition::gui::MainWindow *>()->removeEventFilter(this);
  this->scene->DestroyVisual(this->rootVisual, true);
}

////////////////////////////////////////////////////////////////////////////////
void PointStampedDisplay::initialize(rclcpp::Node::SharedPtr _node)
{
  std::lock_guard<std::mutex>(this->lock);
  this->node = std::move(_node);
}

////////////////////////////////////////////////////////////////////////////////
void PointStampedDisplay::subscribe()
{
  std::lock_guard<std::mutex>(this->lock);

  this->subscriber = this->node->create_subscription<geometry_msgs::msg::PointStamped>(
    this->topic_name,
    this->qos,
    std::bind(&PointStampedDisplay::callback, this, std::placeholders::_1));
}

////////////////////////////////////////////////////////////////////////////////
void PointStampedDisplay::setTopic(const std::string & topic_name)
{
  std::lock_guard<std::mutex>(this->lock);
  this->topic_name = topic_name;

  this->subscribe();

  // Refresh combo-box on plugin load
  this->onRefresh();
}

////////////////////////////////////////////////////////////////////////////////
void PointStampedDisplay::setTopic(const QString & topic_name)
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
void PointStampedDisplay::callback(const geometry_msgs::msg::PointStamped::SharedPtr _msg)
{
  std::lock_guard<std::mutex>(this->lock);
  this->msg = std::move(_msg);
}

////////////////////////////////////////////////////////////////////////////////
bool PointStampedDisplay::eventFilter(QObject * _object, QEvent * _event)
{
  if (_event->type() == gui::events::Render::kType) {
    update();
  }

  return QObject::eventFilter(_object, _event);
}

////////////////////////////////////////////////////////////////////////////////
void PointStampedDisplay::reset()
{
  while (this->visuals.size() != 0) {
    removeOldestPointVisual();
  }

  this->msg.reset();
}

////////////////////////////////////////////////////////////////////////////////
void PointStampedDisplay::update()
{
  std::lock_guard<std::mutex>(this->lock);

  // Remove visuals exceeding history length
  while (this->visuals.size() > this->historyLength) {
    this->removeOldestPointVisual();
  }

  if (!this->msg) {
    return;
  }

  math::Pose3d pose;
  bool poseAvailable = this->frameManager->getFramePose(this->msg->header.frame_id, pose);

  if (!poseAvailable) {
    RCLCPP_ERROR(
      this->node->get_logger(), "Unable to get frame pose: %s",
      this->msg->header.frame_id.c_str());
    return;
  }

  this->rootVisual->SetLocalPose(pose);

  if (this->visuals.size() >= this->historyLength) {
    this->removeOldestPointVisual();
  }

  createNewPointVisual(this->msg);

  // Avoid visualizing same data again
  this->msg.reset();
}

////////////////////////////////////////////////////////////////////////////////
void PointStampedDisplay::createNewPointVisual(
  const geometry_msgs::msg::PointStamped::SharedPtr _msg)
{
  std::lock_guard<std::mutex>(this->lock);
  // Create Visual
  auto visual = this->scene->CreateVisual();
  visual->AddGeometry(this->scene->CreateSphere());
  visual->SetLocalPosition(_msg->point.x, _msg->point.y, _msg->point.z);
  visual->SetLocalScale(this->radius);
  visual->SetMaterial(mat);

  // Add visual
  this->rootVisual->AddChild(visual);
  this->visuals.push_back(visual);
}

////////////////////////////////////////////////////////////////////////////////
void PointStampedDisplay::removeOldestPointVisual()
{
  auto visual = this->visuals.front();
  this->rootVisual->RemoveChild(visual);
  this->scene->DestroyVisual(visual);
  this->visuals.pop_front();
}

////////////////////////////////////////////////////////////////////////////////
void PointStampedDisplay::setFrameManager(std::shared_ptr<common::FrameManager> _frameManager)
{
  std::lock_guard<std::mutex>(this->lock);
  this->frameManager = std::move(_frameManager);
}

////////////////////////////////////////////////////////////////////////////////
void PointStampedDisplay::setHistoryLength(const int & _length)
{
  std::lock_guard<std::mutex>(this->lock);
  // Update history length
  this->historyLength = _length;
}

////////////////////////////////////////////////////////////////////////////////
void PointStampedDisplay::setRadius(const float & _radius)
{
  std::lock_guard<std::mutex>(this->lock);
  for (const auto & visual : this->visuals) {
    visual->SetLocalScale(_radius);
  }
  this->radius = _radius;
}

////////////////////////////////////////////////////////////////////////////////
void PointStampedDisplay::setColor(const QColor & _color)
{
  std::lock_guard<std::mutex>(this->lock);
  this->mat->SetAmbient(_color.redF(), _color.greenF(), _color.blueF(), _color.alphaF());
  this->mat->SetDiffuse(_color.redF(), _color.greenF(), _color.blueF(), _color.alphaF());
  this->mat->SetEmissive(_color.redF(), _color.greenF(), _color.blueF(), _color.alphaF());

  for (const auto & visual : this->visuals) {
    visual->SetMaterial(this->mat);
  }
}

////////////////////////////////////////////////////////////////////////////////
QStringList PointStampedDisplay::getTopicList() const
{
  return this->topicList;
}

////////////////////////////////////////////////////////////////////////////////
void PointStampedDisplay::onRefresh()
{
  std::lock_guard<std::mutex>(this->lock);

  // Clear
  this->topicList.clear();

  int index = 0, position = 0;

  // Get topic list
  auto topics = this->node->get_topic_names_and_types();
  for (const auto & topic : topics) {
    for (const auto & topicType : topic.second) {
      if (topicType == "geometry_msgs/msg/PointStamped") {
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
void PointStampedDisplay::updateQoS(
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
void PointStampedDisplay::LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/)
{
  if (this->title.empty()) {
    this->title = "PointStamped";
  }
}

}  // namespace plugins
}  // namespace rviz
}  // namespace ignition

IGNITION_ADD_PLUGIN(
  ignition::rviz::plugins::PointStampedDisplay,
  ignition::gui::Plugin)
