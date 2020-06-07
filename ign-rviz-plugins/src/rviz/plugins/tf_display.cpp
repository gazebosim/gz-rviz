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
#include <ignition/math/Color.hh>

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
  this->visualFrames.resize(6);
  this->tfArrows.resize(6);

  // Register pink material if not registered
  rendering::MaterialPtr pink;
  if (!this->scene->MaterialRegistered("Default/TransPink")) {
    pink = this->scene->CreateMaterial("Default/TransPink");
    pink->SetAmbient(1.0, 0.0, 1.0);
    pink->SetDiffuse(1.0, 0.0, 1.0);
    pink->SetEmissive(1.0, 1.0, 0.0);
    pink->SetTransparency(0.5);
    pink->SetCastShadows(false);
    pink->SetReceiveShadows(false);
    pink->SetLightingEnabled(false);
  } else {
    pink = this->scene->Material("Default/TransPink");
  }

  // Default Yellow Material
  rendering::MaterialPtr yellow = this->scene->Material("Default/TransYellow");

  for (int i = 0; i < 6; i++) {
    this->visualFrames[i] = this->scene->CreateVisual();

    // Add axis
    rendering::AxisVisualPtr axis = this->scene->CreateAxisVisual();
    axis->SetLocalScale(0.5);
    this->visualFrames[i]->AddChild(axis);

    // Add arrow
    rendering::ArrowVisualPtr arrow = this->createTfArrow();
    arrow->SetVisible(false);
    this->tfArrows[i] = arrow;

    // Add text
    rendering::TextPtr frameName = this->scene->CreateText();
    frameName->SetTextString("frame");
    frameName->SetShowOnTop(true);
    frameName->SetTextAlignment(
      rendering::TextHorizontalAlign::CENTER,
      rendering::TextVerticalAlign::CENTER);
    frameName->SetCharHeight(0.15);
    this->visualFrames[i]->AddGeometry(frameName);
    this->scene->RootVisual()->AddChild(this->visualFrames[i]);
    this->scene->RootVisual()->AddChild(this->tfArrows[i]);
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

rendering::ArrowVisualPtr TFDisplay::createTfArrow()
{
  rendering::ArrowVisualPtr arrow = this->scene->CreateArrowVisual();
  rendering::VisualPtr head = arrow->Head();
  head->SetMaterial(this->scene->Material("Default/TransPink"));
  rendering::VisualPtr shaft = arrow->Shaft();
  shaft->Scale(0.5, 0.5, 1.5);
  arrow->SetOrigin(0, 0, -0.75);
  shaft->SetMaterial(this->scene->Material("Default/TransYellow"));
  return arrow;
}

bool TFDisplay::eventFilter(QObject * object, QEvent * event)
{
  std::lock_guard<std::mutex>(this->lock);
  std::vector<std::string> frameIds;
  frameManager->getFrames(frameIds);

  for (int i = 0; i < static_cast<int>(frameIds.size()); i++) {
    ignition::math::Pose3d pose, parentPose;
    this->frameManager->getFramePose(frameIds[i], pose);

    // Set Frame Text
    rendering::TextPtr frameName = std::dynamic_pointer_cast<rendering::Text>(
      this->visualFrames[i]->GeometryByIndex(0));
    frameName->SetTextString(frameIds[i]);

    this->visualFrames[i]->SetLocalPose(pose);

    bool result = this->frameManager->getParentPose(frameIds[i], parentPose);
    if (result) {
      math::Vector3d dir = parentPose.Pos() - pose.Pos();
      double dist = dir.Length();
      if (dist > 0.25) {
        this->tfArrows[i]->SetVisible(true);
        math::Quaterniond quat;
        quat.From2Axes(-math::Vector3d::UnitZ, dir);
        quat *= math::Quaterniond::EulerToQuaternion(M_PI, 0, 0);
        this->tfArrows[i]->SetLocalRotation(quat);
        this->tfArrows[i]->SetLocalScale(1, 1, dist);
        this->tfArrows[i]->SetLocalPosition(pose.Pos());
      } else {
        this->tfArrows[i]->SetVisible(false);
      }
    }
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
