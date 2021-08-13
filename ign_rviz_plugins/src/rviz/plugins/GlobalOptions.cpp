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
TFStatus::TFStatus()
: status(" "), message(" "), color("green")
{}

////////////////////////////////////////////////////////////////////////////////
void TFStatus::update(std::string & _fixedFrame, std::vector<std::string> & _allFrames)
{
  // Check for tf data
  auto framePosition = std::find(_allFrames.begin(), _allFrames.end(), _fixedFrame);

  if (_allFrames.empty()) {
    std::string msg = "No tf data. Frame [" + _fixedFrame + "] does not exist";

    this->status = QString::fromStdString("Fixed Frame [Warn]");
    this->message = QString::fromStdString(msg);
    this->color = QString::fromStdString("orange");
  } else if (framePosition == _allFrames.end()) {
    std::string msg = "Frame [" + _fixedFrame + "] does not exist";

    this->status = QString::fromStdString("Fixed Frame [Error]");
    this->message = QString::fromStdString(msg);
    this->color = QString::fromStdString("red");
  } else {
    std::string msg = "OK";

    this->status = QString::fromStdString("Fixed Frame");
    this->message = QString::fromStdString(msg);
    this->color = QString::fromStdString("green");
  }
}

////////////////////////////////////////////////////////////////////////////////
QString TFStatus::getStatus() const
{
  return this->status;
}

////////////////////////////////////////////////////////////////////////////////
QString TFStatus::getMessage() const
{
  return this->message;
}

////////////////////////////////////////////////////////////////////////////////
QString TFStatus::getColor() const
{
  return this->color;
}

////////////////////////////////////////////////////////////////////////////////
GlobalOptions::GlobalOptions()
: dirty(false), initialized(false), populated(false), color("#303030")
{
  // TODO(Sarathkrishnan Ramesh)
  // Add support to select render engine using config file
  this->engine = rendering::engine("ogre");
  if (!this->engine) {
    igndbg << "Engine '" << "ogre" << "' is not supported" << std::endl;
    return;
  }
  this->tfStatus = new TFStatus();
  this->frameList.push_back("world");
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
    if (!initialized) {
      if (this->scene == nullptr) {
        this->scene = this->engine->SceneByName("scene");

        // Configure scene lighting
        auto light = this->scene->CreatePointLight();
        light->SetDiffuseColor(0.8, 0.8, 0.8);
        light->SetSpecularColor(0.8, 0.8, 0.8);
        light->SetLocalPosition(0, 0, 8);
        this->scene->RootVisual()->AddChild(light);

        initialized = true;
      }
    }

    // Update background color
    if (dirty) {
      this->scene->SetBackgroundColor(math::Color(color.redF(), color.greenF(), color.blue()));
      this->dirty = false;
    }

    // Update tf status and message
    std::vector<std::string> allFrames;
    this->frameManager->getFrames(allFrames);
    std::string fixedFrame = this->frameManager->getFixedFrame();
    this->tfStatus->update(fixedFrame, allFrames);
    emit tfStatusChanged();
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
TFStatus * GlobalOptions::getTfStatus() const
{
  return this->tfStatus;
}

////////////////////////////////////////////////////////////////////////////////
void GlobalOptions::onRefresh()
{
  // Get updated list
  std::vector<std::string> allFrames;
  this->frameManager->getFrames(allFrames);

  if (allFrames.size() != 0) {
    // Clear
    this->frameList.clear();

    std::sort(allFrames.begin(), allFrames.end());

    int index = 0;
    std::string fixedFrame = this->frameManager->getFixedFrame();

    for (int i = 0; i < static_cast<int>(allFrames.size()); ++i) {
      if (allFrames[i] == fixedFrame) {
        index = i;
      }
      this->frameList.push_back(QString::fromStdString(allFrames[i]));
    }

    this->frameListChanged();

    if (!populated) {
      emit setCurrentIndex(index);
      populated = false;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void GlobalOptions::LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/)
{
  if (this->title.empty()) {
    this->title = "Global Options";
  }

  auto cardItem = this->CardItem();
  cardItem->setProperty("showCloseButton", false);
  cardItem->setProperty("showDockButton", false);
}

}  // namespace plugins
}  // namespace rviz
}  // namespace ignition

IGNITION_ADD_PLUGIN(
  ignition::rviz::plugins::GlobalOptions,
  ignition::gui::Plugin)
