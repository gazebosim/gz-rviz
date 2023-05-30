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

#include "gz/rviz/plugins/MarkerDisplay.hpp"
#ifdef GZ_HEADERS
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/plugin/Register.hh>
using namespace gz;
#else
#include <ignition/gui/Application.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/plugin/Register.hh>
using namespace gz;
#endif
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace gz
{
namespace rviz
{
namespace plugins
{
////////////////////////////////////////////////////////////////////////////////
MarkerDisplay::MarkerDisplay()
: MessageDisplay(), markerManager(std::make_unique<MarkerManager>()) {}

////////////////////////////////////////////////////////////////////////////////
MarkerDisplay::~MarkerDisplay()
{
  std::lock_guard<std::mutex>(this->lock);
  gui::App()->findChild<gui::MainWindow *>()->removeEventFilter(this);
}

////////////////////////////////////////////////////////////////////////////////
void MarkerDisplay::initialize(rclcpp::Node::SharedPtr _node)
{
  std::lock_guard<std::mutex>(this->lock);
  this->node = std::move(_node);
}

////////////////////////////////////////////////////////////////////////////////
void MarkerDisplay::subscribe()
{
  std::lock_guard<std::mutex>(this->lock);

  this->subscriber = this->node->create_subscription<visualization_msgs::msg::Marker>(
    this->topic_name,
    this->qos,
    std::bind(&MarkerDisplay::callback, this, std::placeholders::_1));
}

////////////////////////////////////////////////////////////////////////////////
void MarkerDisplay::setTopic(const std::string & topic_name)
{
  std::lock_guard<std::mutex>(this->lock);
  this->topic_name = topic_name;

  this->subscribe();

  // Refresh combo-box on plugin load
  this->onRefresh();
}

////////////////////////////////////////////////////////////////////////////////
void MarkerDisplay::setTopic(const QString & topic_name)
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
void MarkerDisplay::callback(const visualization_msgs::msg::Marker::SharedPtr _msg)
{
  std::lock_guard<std::mutex>(this->lock);
  this->msg = std::move(_msg);
}

////////////////////////////////////////////////////////////////////////////////
bool MarkerDisplay::eventFilter(QObject * _object, QEvent * _event)
{
  if (_event->type() == gui::events::Render::kType) {
    update();
  }

  return QObject::eventFilter(_object, _event);
}

////////////////////////////////////////////////////////////////////////////////
void MarkerDisplay::reset()
{
  this->msg.reset();
}

////////////////////////////////////////////////////////////////////////////////
void MarkerDisplay::update()
{
  std::lock_guard<std::mutex>(this->lock);

  if (!this->msg) {
    return;
  }

  markerManager->processMessage(*this->msg);

  // Avoid visualizing same data again
  this->msg.reset();
}

////////////////////////////////////////////////////////////////////////////////
void MarkerDisplay::setFrameManager(std::shared_ptr<common::FrameManager> _frameManager)
{
  std::lock_guard<std::mutex>(this->lock);
  this->frameManager = std::move(_frameManager);
}

////////////////////////////////////////////////////////////////////////////////
QStringList MarkerDisplay::getTopicList() const
{
  return this->topicList;
}

////////////////////////////////////////////////////////////////////////////////
void MarkerDisplay::onRefresh()
{
  std::lock_guard<std::mutex>(this->lock);

  // Clear
  this->topicList.clear();

  int index = 0, position = 0;

  // Get topic list
  auto topics = this->node->get_topic_names_and_types();
  for (const auto & topic : topics) {
    for (const auto & topicType : topic.second) {
      if (topicType == "visualization_msgs/msg/Marker") {
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
void MarkerDisplay::updateQoS(
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
void MarkerDisplay::LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/)
{
  if (this->title.empty()) {
    this->title = "Marker";
  }
}

}  // namespace plugins
}  // namespace rviz
}  // namespace gz

#ifdef GZ_HEADERS
GZ_ADD_PLUGIN(
  rviz::plugins::MarkerDisplay,
  gui::Plugin)
#else
IGNITION_ADD_PLUGIN(
  rviz::plugins::MarkerDisplay,
  gui::Plugin)
#endif
