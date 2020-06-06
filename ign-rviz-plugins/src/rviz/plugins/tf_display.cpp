// Copyright (c) 2020 Sarathkrishnan Ramesh
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

#include "ignition/rviz/plugins/tf_display.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <ignition/math.hh>

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
TFDisplay::TFDisplay()
: MessageDisplay()
{
  this->engine = ignition::rendering::engine("ogre");
  this->scene = this->engine->SceneByName("scene");
  rendering::AxisVisualPtr axis = this->scene->CreateAxisVisual();
  this->visualFrames.resize(6);
  for (int i = 0; i < 6; i++) {
    this->visualFrames[i] = this->scene->CreateAxisVisual();
    this->visualFrames[i]->SetLocalScale(0.75);
    this->scene->RootVisual()->AddChild(this->visualFrames[i]);
  }
}

TFDisplay::~TFDisplay()
{}

void TFDisplay::initialize(rclcpp::Node::SharedPtr node)
{
  this->node = std::move(node);
}

void TFDisplay::setTopic(std::string topic_name)
{
  this->topic_name = topic_name;
  this->subscriber = this->node->create_subscription<tf2_msgs::msg::TFMessage>(
    this->topic_name, 10,
    std::bind(&TFDisplay::callback, this, std::placeholders::_1));
}

bool TFDisplay::eventFilter(QObject * object, QEvent * event)
{
  std::lock_guard<std::mutex>(this->lock);
  std::vector<std::string> frameIds;
  frameManager->getFrames(frameIds);

  for (int i = 0; i < static_cast<int>(frameIds.size()); i++) {
    ignition::math::Pose3d pose;

    this->frameManager->getFramePose(frameIds[i], pose);
    this->visualFrames[i]->SetLocalPosition(pose.Pos());
    this->visualFrames[i]->SetLocalRotation(pose.Rot());
  }

  return QObject::eventFilter(object, event);
}

void TFDisplay::installEventFilter(ignition::gui::MainWindow * window)
{
  window->installEventFilter(this);
}

void TFDisplay::setFrameManager(std::shared_ptr<common::FrameManager> frameManager)
{
  this->frameManager = std::move(frameManager);
}
}  // namespace plugins
}  // namespace rviz
}  // namespace ignition

PLUGINLIB_EXPORT_CLASS(
  ignition::rviz::plugins::TFDisplay,
  ignition::rviz::plugins::MessageDisplayBase)
