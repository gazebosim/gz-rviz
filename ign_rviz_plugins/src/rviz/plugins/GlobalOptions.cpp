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

#include <ignition/plugin/Register.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/GuiEvents.hh>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ignition/rviz/common/rviz_events.hpp"
#include "ignition/rviz/plugins/GlobalOptions.hpp"

namespace ignition
{
namespace rviz
{
namespace plugins
{
////////////////////////////////////////////////////////////////////////////////
GlobalOptions::GlobalOptions()
: dirty(false)
{
  // TODO(Sarathkrishnan Ramesh)
  // Add support to select render engine using config file
  this->engine = rendering::engine("ogre");
  if (!this->engine) {
    igndbg << "Engine '" << "ogre" << "' is not supported" << std::endl;
    return;
  }
}

////////////////////////////////////////////////////////////////////////////////
void GlobalOptions::setFrame(const QString & _frame)
{
  std::lock_guard(this->lock);
  this->frameManager->setFixedFrame(_frame.toStdString());
}

////////////////////////////////////////////////////////////////////////////////
void GlobalOptions::setSceneBackground(const QColor & _color)
{
  std::lock_guard(this->lock);
  this->color = _color;
  this->dirty = true;
}

////////////////////////////////////////////////////////////////////////////////
void GlobalOptions::setFrameManager(std::shared_ptr<common::FrameManager> _frameManager)
{
  std::lock_guard(this->lock);
  this->frameManager = std::move(_frameManager);
  this->frameList.clear();

  // Update frame list
  this->onRefresh();
}

////////////////////////////////////////////////////////////////////////////////
bool GlobalOptions::eventFilter(QObject * _object, QEvent * _event)
{
  if (_event->type() == gui::events::Render::kType) {
    std::lock_guard(this->lock);
    // Update background color
    if (dirty) {
      if (this->scene == nullptr) {
        this->scene = this->engine->SceneByName("scene");
      }
      this->scene->SetBackgroundColor(math::Color(color.redF(), color.greenF(), color.blue()));
      this->dirty = false;
    }
  }

  // Update combo-box on frame list change
  if (_event->type() == rviz::events::FrameListChanged::kType) {
    this->onRefresh();
  }

  return QObject::eventFilter(_object, _event);
}


////////////////////////////////////////////////////////////////////////////////
void GlobalOptions::setFrameList(const QStringList & _frameList)
{
  this->frameList = _frameList;
}

////////////////////////////////////////////////////////////////////////////////
QStringList GlobalOptions::getFrameList() const
{
  return this->frameList;
}

////////////////////////////////////////////////////////////////////////////////
void GlobalOptions::onRefresh()
{
  // Clear
  this->frameList.clear();

  // Get updated list
  std::vector<std::string> allFrames;
  this->frameManager->getFrames(allFrames);
  std::sort(allFrames.begin(), allFrames.end());

  for (const auto frame : allFrames) {
    this->frameList.push_back(QString::fromStdString(frame));
  }

  this->frameListChanged();
}

////////////////////////////////////////////////////////////////////////////////
void GlobalOptions::LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/)
{
  if (this->title.empty()) {
    this->title = "Global Options";
  }
}

}  // namespace plugins
}  // namespace rviz
}  // namespace ignition

IGNITION_ADD_PLUGIN(
  ignition::rviz::plugins::GlobalOptions,
  ignition::gui::Plugin)
