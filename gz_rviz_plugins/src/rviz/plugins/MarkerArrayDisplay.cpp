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

#include "gz/rviz/plugins/MarkerArrayDisplay.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/plugin/Register.hh>

namespace gz
{
namespace rviz
{
namespace plugins
{
////////////////////////////////////////////////////////////////////////////////
MarkerArrayDisplay::MarkerArrayDisplay()
: MessageDisplay(), markerManager(std::make_unique<MarkerManager>()) {}

////////////////////////////////////////////////////////////////////////////////
MarkerArrayDisplay::~MarkerArrayDisplay()
{
  std::lock_guard<std::mutex>(this->lock);
  gui::App()->findChild<gui::MainWindow *>()->removeEventFilter(this);
}

////////////////////////////////////////////////////////////////////////////////
void MarkerArrayDisplay::initialize(rclcpp::Node::SharedPtr _node)
{
  std::lock_guard<std::mutex>(this->lock);
  this->node = std::move(_node);
}

////////////////////////////////////////////////////////////////////////////////
void MarkerArrayDisplay::subscribe()
{
  std::lock_guard<std::mutex>(this->lock);

  this->subscriber = this->node->create_subscription<visualization_msgs::msg::MarkerArray>(
    this->topic_name,
    this->qos,
    std::bind(&MarkerArrayDisplay::callback, this, std::placeholders::_1));
}

////////////////////////////////////////////////////////////////////////////////
void MarkerArrayDisplay::setTopic(const std::string & topic_name)
{
  std::lock_guard<std::mutex>(this->lock);
  this->topic_name = topic_name;

  this->subscribe();

  // Refresh combo-box on plugin load
  this->onRefresh();
}

////////////////////////////////////////////////////////////////////////////////
void MarkerArrayDisplay::setTopic(const QString & topic_name)
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
void MarkerArrayDisplay::callback(const visualization_msgs::msg::MarkerArray::SharedPtr _msg)
{
  std::lock_guard<std::mutex>(this->lock);
  this->msg = std::move(_msg);
}

////////////////////////////////////////////////////////////////////////////////
bool MarkerArrayDisplay::eventFilter(QObject * _object, QEvent * _event)
{
  if (_event->type() == gui::events::Render::kType) {
    update();
  }

  return QObject::eventFilter(_object, _event);
}

////////////////////////////////////////////////////////////////////////////////
void MarkerArrayDisplay::reset()
{
  this->msg.reset();
}

////////////////////////////////////////////////////////////////////////////////
void MarkerArrayDisplay::update()
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
void MarkerArrayDisplay::setFrameManager(std::shared_ptr<common::FrameManager> _frameManager)
{
  std::lock_guard<std::mutex>(this->lock);
  this->frameManager = std::move(_frameManager);
}

////////////////////////////////////////////////////////////////////////////////
QStringList MarkerArrayDisplay::getTopicList() const
{
  return this->topicList;
}

////////////////////////////////////////////////////////////////////////////////
void MarkerArrayDisplay::onRefresh()
{
  std::lock_guard<std::mutex>(this->lock);

  // Clear
  this->topicList.clear();

  int index = 0, position = 0;

  // Get topic list
  auto topics = this->node->get_topic_names_and_types();
  for (const auto & topic : topics) {
    for (const auto & topicType : topic.second) {
      if (topicType == "visualization_msgs/msg/MarkerArray") {
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
void MarkerArrayDisplay::updateQoS(
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
void MarkerArrayDisplay::LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/)
{
  if (this->title.empty()) {
    this->title = "MarkerArray";
  }
}

}  // namespace plugins
}  // namespace rviz
}  // namespace gz

#ifdef GZ_HEADERS
GZ_ADD_PLUGIN(
  gz::rviz::plugins::MarkerArrayDisplay,
  gui::Plugin)
#else
IGNITION_ADD_PLUGIN(
  gz::rviz::plugins::MarkerArrayDisplay,
  gui::Plugin)
#endif
