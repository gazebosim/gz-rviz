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

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>
#include <string>

#include "ignition/rviz/common/rviz_events.hpp"

namespace ignition
{
namespace rviz
{
namespace plugins
{
////////////////////////////////////////////////////////////////////////////////
AxesDisplay::AxesDisplay()
: length(1.0), radius(0.1), headVisible(false), dirty(false)
{
  // TODO(Sarathkrishnan Ramesh)
  // Add support to select render engine using config file
  this->engine = rendering::engine("ogre");
  if (!this->engine) {
    igndbg << "Engine '" << "ogre" << "' is not supported" << std::endl;
    return;
  }

  this->scene = this->engine->SceneByName("scene");
}

////////////////////////////////////////////////////////////////////////////////
void AxesDisplay::setScale()
{
  for (int i = 0; i < 3; ++i) {
    auto arrow =
      std::dynamic_pointer_cast<rendering::ArrowVisual>(this->rootVisual->ChildByIndex(i));
    arrow->SetLocalScale(this->radius * 20, this->radius * 20, this->length * 2);
    arrow->ShowArrowHead(this->headVisible);
  }
}

////////////////////////////////////////////////////////////////////////////////
bool AxesDisplay::eventFilter(QObject * object, QEvent * event)
{
  if (event->type() == gui::events::Render::kType) {
    if (rootVisual == nullptr) {
      rootVisual = this->scene->CreateAxisVisual();
      setScale();
      this->scene->RootVisual()->AddChild(rootVisual);
    }
    // Update pose
    {
      std::lock_guard(this->lock);

      // Origin pose. Fixed Frame always at origin.
      math::Pose3d pose;

      if (frame == "<Fixed Frame>") {
        rootVisual->SetLocalPose(pose);
      } else if (this->frameManager->getFramePose(frame, pose)) {
        rootVisual->SetLocalPose(pose);
      }

      if (dirty) {
        setScale();
        this->dirty = false;
      }
    }
  }

  // Update combo-box on frame list change
  if (event->type() == rviz::events::FrameListChanged::kType) {
    this->onRefresh();
  }

  return QObject::eventFilter(object, event);
}

////////////////////////////////////////////////////////////////////////////////
void AxesDisplay::setFrame(const QString & frame)
{
  std::lock_guard(this->lock);
  this->frame = frame.toStdString();
}

////////////////////////////////////////////////////////////////////////////////
void AxesDisplay::setLength(const float & length)
{
  std::lock_guard(this->lock);
  if (!isnan(length)) {
    this->length = length;
    this->dirty = true;
  }
}

////////////////////////////////////////////////////////////////////////////////
void AxesDisplay::setRadius(const float & radius)
{
  std::lock_guard(this->lock);
  if (!isnan(radius)) {
    this->radius = radius;
    this->dirty = true;
  }
}

////////////////////////////////////////////////////////////////////////////////
void AxesDisplay::setHeadVisibility(const bool & visible)
{
  std::lock_guard(this->lock);
  if (!isnan(radius)) {
    this->headVisible = visible;
    this->dirty = true;
  }
}

////////////////////////////////////////////////////////////////////////////////
AxesDisplay::~AxesDisplay()
{
  std::lock_guard(this->lock);
  // Delete visual
  ignition::gui::App()->findChild<ignition::gui::MainWindow *>()->removeEventFilter(this);
  this->scene->DestroyVisual(this->rootVisual);
}

////////////////////////////////////////////////////////////////////////////////
void AxesDisplay::setFrameManager(std::shared_ptr<common::FrameManager> _frameManager)
{
  std::lock_guard(this->lock);
  this->frameManager = std::move(_frameManager);
  this->frame = this->frameManager->getFixedFrame();

  // Update frame list
  this->onRefresh();
}

////////////////////////////////////////////////////////////////////////////////
QStringList AxesDisplay::getFrameList() const
{
  return this->frameList;
}

////////////////////////////////////////////////////////////////////////////////
void AxesDisplay::onRefresh()
{
  // Clear
  this->frameList.clear();

  // Get updated list
  std::vector<std::string> allFrames;
  this->frameManager->getFrames(allFrames);
  std::sort(allFrames.begin(), allFrames.end());

  this->frameList.push_back(QString::fromStdString("<Fixed Frame>"));

  for (const auto frame : allFrames) {
    this->frameList.push_back(QString::fromStdString(frame));
  }

  this->frameListChanged();
}

////////////////////////////////////////////////////////////////////////////////
void AxesDisplay::LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/)
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
