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

#include "ignition/rviz/plugins/PolygonDisplay.hpp"

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
PolygonDisplay::PolygonDisplay()
: MessageDisplay(), color(0.098, 1.0, 0.2), createMarker(true)
{
  // Get reference to scene
  this->engine = ignition::rendering::engine("ogre");
  this->scene = this->engine->SceneByName("scene");

  this->rootVisual = this->scene->CreateVisual();
  this->scene->RootVisual()->AddChild(this->rootVisual);
}

////////////////////////////////////////////////////////////////////////////////
PolygonDisplay::~PolygonDisplay()
{
  std::lock_guard<std::mutex>(this->lock);
  // Delete visual
  ignition::gui::App()->findChild<ignition::gui::MainWindow *>()->removeEventFilter(this);
  this->scene->DestroyVisual(this->rootVisual, true);
}

////////////////////////////////////////////////////////////////////////////////
void PolygonDisplay::initialize(rclcpp::Node::SharedPtr _node)
{
  std::lock_guard<std::mutex>(this->lock);
  this->node = std::move(_node);
}

////////////////////////////////////////////////////////////////////////////////
void PolygonDisplay::subscribe()
{
  std::lock_guard<std::mutex>(this->lock);

  this->subscriber = this->node->create_subscription<geometry_msgs::msg::PolygonStamped>(
    this->topic_name,
    this->qos,
    std::bind(&PolygonDisplay::callback, this, std::placeholders::_1));
}

////////////////////////////////////////////////////////////////////////////////
void PolygonDisplay::setTopic(const std::string & topic_name)
{
  std::lock_guard<std::mutex>(this->lock);
  this->topic_name = topic_name;

  this->subscribe();

  // Refresh combo-box on plugin load
  this->onRefresh();
}

////////////////////////////////////////////////////////////////////////////////
void PolygonDisplay::setTopic(const QString & topic_name)
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
void PolygonDisplay::callback(const geometry_msgs::msg::PolygonStamped::SharedPtr _msg)
{
  std::lock_guard<std::mutex>(this->lock);
  this->msg = std::move(_msg);
}

////////////////////////////////////////////////////////////////////////////////
bool PolygonDisplay::eventFilter(QObject * _object, QEvent * _event)
{
  if (_event->type() == gui::events::Render::kType) {
    update();
  }

  return QObject::eventFilter(_object, _event);
}

////////////////////////////////////////////////////////////////////////////////
void PolygonDisplay::reset()
{
  this->msg.reset();

  auto marker = std::dynamic_pointer_cast<rendering::Marker>(this->rootVisual->GeometryByIndex(0));

  if (marker == nullptr) {
    return;
  }

  marker->ClearPoints();
  // Add two points to clear polygon visual
  marker->AddPoint(0.0, 0.0, 0.0, this->color);
  marker->AddPoint(0.0, 0.0, 0.0, this->color);
}

////////////////////////////////////////////////////////////////////////////////
void PolygonDisplay::update()
{
  std::lock_guard<std::mutex>(this->lock);

  if (!this->msg) {
    return;
  }

  if (createMarker) {
    // Delete previous marker geometry.
    this->rootVisual->RemoveGeometries();

    // Create marker and set type to line strip
    rendering::MarkerPtr marker = this->scene->CreateMarker();
    marker->SetType(rendering::MarkerType::MT_LINE_STRIP);

    // This material is not used anywhere but is required to set
    // point color in marker AddPoint method
    marker->SetMaterial(this->scene->Material("Default/TransGreen"));

    this->rootVisual->AddGeometry(marker);
    createMarker = false;
  }

  math::Pose3d pose;
  bool poseAvailable = this->frameManager->getFramePose(this->msg->header.frame_id, pose);

  if (!poseAvailable) {
    RCLCPP_ERROR(
      this->node->get_logger(), "Unable to get frame pose: %s",
      this->msg->header.frame_id.c_str());
    return;
  }

  auto marker = std::dynamic_pointer_cast<rendering::Marker>(this->rootVisual->GeometryByIndex(0));

  marker->ClearPoints();

  if (this->msg->polygon.points.size() == 0) {
    // Add two points to clear polygon visual
    marker->AddPoint(0.0, 0.0, 0.0, this->color);
    marker->AddPoint(0.0, 0.0, 0.0, this->color);
    return;
  }

  // Add polygon vertices
  for (const auto & point : this->msg->polygon.points) {
    marker->AddPoint(point.x, point.y, point.z, this->color);
  }

  // Adding fist point again to close polygon
  const auto & point = this->msg->polygon.points.front();
  marker->AddPoint(point.x, point.y, point.z, this->color);

  this->rootVisual->SetLocalPose(pose);
}

////////////////////////////////////////////////////////////////////////////////
void PolygonDisplay::setColor(const QColor & _color)
{
  std::lock_guard<std::mutex>(this->lock);
  this->color.Set(_color.redF(), _color.greenF(), _color.blueF(), _color.alphaF());

  // Recreating marker is the only way to change color and transparency
  this->createMarker = true;
}

////////////////////////////////////////////////////////////////////////////////
void PolygonDisplay::setFrameManager(std::shared_ptr<common::FrameManager> _frameManager)
{
  std::lock_guard<std::mutex>(this->lock);
  this->frameManager = std::move(_frameManager);
}

////////////////////////////////////////////////////////////////////////////////
QStringList PolygonDisplay::getTopicList() const
{
  return this->topicList;
}

////////////////////////////////////////////////////////////////////////////////
void PolygonDisplay::onRefresh()
{
  std::lock_guard<std::mutex>(this->lock);

  // Clear
  this->topicList.clear();

  int index = 0, position = 0;

  // Get topic list
  auto topics = this->node->get_topic_names_and_types();
  for (const auto & topic : topics) {
    for (const auto & topicType : topic.second) {
      if (topicType == "geometry_msgs/msg/PolygonStamped") {
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
void PolygonDisplay::updateQoS(
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
void PolygonDisplay::LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/)
{
  if (this->title.empty()) {
    this->title = "Polygon";
  }
}

}  // namespace plugins
}  // namespace rviz
}  // namespace ignition

IGNITION_ADD_PLUGIN(
  ignition::rviz::plugins::PolygonDisplay,
  ignition::gui::Plugin)
