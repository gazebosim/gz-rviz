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

#include "ignition/rviz/plugins/AxesDisplay.hpp"
#include <ignition/plugin/Register.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/GuiEvents.hh>

#include <memory>
#include <utility>

namespace ignition
{
namespace rviz
{
namespace plugins
{
////////////////////////////////////////////////////////////////////////////////
AxesDisplay::AxesDisplay()
{
  ignition::gui::App()->findChild<ignition::gui::MainWindow *>()->installEventFilter(this);
  this->engine = rendering::engine("ogre");
  this->scene = this->engine->SceneByName("scene");
}

////////////////////////////////////////////////////////////////////////////////
bool AxesDisplay::eventFilter(QObject * object, QEvent * event)
{
  if (event->type() == gui::events::Render::kType) {
    if (rootVisual == nullptr) {
      rootVisual = this->scene->CreateAxisVisual();
      this->scene->RootVisual()->AddChild(rootVisual);
    }
    // Update pose
    {
      std::lock_guard(this->lock);
      math::Pose3d pose;
      if (this->frameManager->getFramePose(frame, pose)) {
        rootVisual->SetLocalPose(pose);
      }
    }
  }

  return QObject::eventFilter(object, event);
}

////////////////////////////////////////////////////////////////////////////////
void AxesDisplay::setFrame(QString frame)
{
  std::lock_guard(this->lock);
  this->frame = frame.toStdString();
}

////////////////////////////////////////////////////////////////////////////////
AxesDisplay::~AxesDisplay()
{
  std::lock_guard(this->lock);
  ignition::gui::App()->findChild<ignition::gui::MainWindow *>()->removeEventFilter(this);
  this->scene->DestroyVisual(this->rootVisual);
}

////////////////////////////////////////////////////////////////////////////////
void AxesDisplay::setFrameManager(std::shared_ptr<common::FrameManager> frameManager)
{
  std::lock_guard(this->lock);
  // Delete visual
  this->frameManager = std::move(frameManager);
  this->frame = this->frameManager->getFixedFrame();
}

////////////////////////////////////////////////////////////////////////////////
void AxesDisplay::LoadConfig(const tinyxml2::XMLElement */*_pluginElem*/)
{
  if (this->title.empty()) {
    this->title = "Axis";
  }
}

}  // namespace plugins
}  // namespace rviz
}  // namespace ignition

IGNITION_ADD_PLUGIN(
  ignition::rviz::plugins::AxesDisplay,
  ignition::gui::Plugin)
