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

#include "ignition/rviz/plugins/TFDisplay.hpp"

#include <ignition/gui/Application.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/plugin/Register.hh>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ignition/rviz/common/rviz_events.hpp"

namespace ignition
{
namespace rviz
{
namespace plugins
{
#define MIN_FRAME_DISTANCE 0.25
////////////////////////////////////////////////////////////////////////////////
FrameModel::FrameModel(QObject * _parent)
: QStandardItemModel(_parent)
{}

////////////////////////////////////////////////////////////////////////////////
void FrameModel::addFrame(const QString & _name, QStandardItem * _parentItem)
{
  QStandardItem * frameRow = new QStandardItem();
  frameRow->setData(_name, NameRole);
  frameRow->setCheckState(Qt::CheckState::Checked);
  _parentItem->appendRow(frameRow);
}

////////////////////////////////////////////////////////////////////////////////
QStandardItem * FrameModel::addParentRow(const QString & _name)
{
  QStandardItem * entry = new QStandardItem();
  entry->setData(_name, NameRole);
  entry->setCheckState(Qt::CheckState::Checked);
  appendRow(entry);
  return entry;
}

////////////////////////////////////////////////////////////////////////////////
QVariant FrameModel::data(const QModelIndex & _index, int _role) const
{
  QStandardItem * myItem = itemFromIndex(_index);

  if (_role == NameRole) {
    return myItem->data(NameRole);
  }

  if (_role == Qt::CheckStateRole) {
    return myItem->data(Qt::CheckStateRole);
  }

  return QVariant();
}

////////////////////////////////////////////////////////////////////////////////
QHash<int, QByteArray> FrameModel::roleNames() const
{
  QHash<int, QByteArray> roles;
  roles[NameRole] = "name";
  roles[Qt::CheckStateRole] = "checked";

  return roles;
}

////////////////////////////////////////////////////////////////////////////////
TFDisplay::TFDisplay()
: MessageDisplay(), axesVisible(true), arrowsVisible(true), namesVisible(true),
  axesHeadVisible(false), markerScale(0.4)
{
  // Get reference to scene
  this->engine = ignition::rendering::engine("ogre");
  this->scene = this->engine->SceneByName("scene");

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

  // Create a root visual for tf visualization
  this->tfRootVisual = this->scene->CreateVisual();
  this->scene->RootVisual()->AddChild(tfRootVisual);

  this->frameModel = new FrameModel();
  parentRow = this->frameModel->addParentRow(QString::fromStdString("All Frames"));
}

////////////////////////////////////////////////////////////////////////////////
TFDisplay::~TFDisplay()
{
  std::lock_guard<std::mutex>(this->lock);
  // Delete visual
  ignition::gui::App()->findChild<ignition::gui::MainWindow *>()->removeEventFilter(this);
  this->scene->DestroyVisual(this->tfRootVisual);
}

////////////////////////////////////////////////////////////////////////////////
void TFDisplay::initialize(rclcpp::Node::SharedPtr _node)
{
  this->node = std::move(_node);
}

////////////////////////////////////////////////////////////////////////////////
rendering::ArrowVisualPtr TFDisplay::createTfArrow()
{
  // Create an arrow
  rendering::ArrowVisualPtr arrow = this->scene->CreateArrowVisual();

  // Set arrow head material to pink
  rendering::VisualPtr head = arrow->Head();
  head->SetMaterial(this->scene->Material("Default/TransPink"));

  // Set arrow shaft material to yellow
  rendering::VisualPtr shaft = arrow->Shaft();
  shaft->SetMaterial(this->scene->Material("Default/TransYellow"));

  // Set arrow (head and shaft) to unit length
  shaft->Scale(0.5, 0.5, 1.5);
  arrow->SetOrigin(0, 0, -0.75);

  return arrow;
}

////////////////////////////////////////////////////////////////////////////////
rendering::VisualPtr TFDisplay::createVisualFrame()
{
  rendering::VisualPtr visualFrame = this->scene->CreateVisual();

  // Add axis
  rendering::AxisVisualPtr axis = this->scene->CreateAxisVisual();
  axis->SetLocalScale(this->markerScale);
  visualFrame->AddChild(axis);

  // Add arrow
  rendering::ArrowVisualPtr arrow = this->createTfArrow();
  arrow->SetLocalPosition(0, 0, 0);
  arrow->SetVisible(false);

  visualFrame->AddChild(arrow);

  // Add text
  rendering::TextPtr frameName = this->scene->CreateText();
  frameName->SetTextString("frame");
  frameName->SetShowOnTop(true);
  frameName->SetTextAlignment(
    rendering::TextHorizontalAlign::CENTER,
    rendering::TextVerticalAlign::CENTER);
  frameName->SetCharHeight(0.15);
  visualFrame->AddGeometry(frameName);

  return visualFrame;
}

////////////////////////////////////////////////////////////////////////////////
/**
 * Update tf visualization only when ign::gui render event is received
 */
bool TFDisplay::eventFilter(QObject * _object, QEvent * _event)
{
  if (_event->type() == gui::events::Render::kType) {
    // Create a default visual frame
    if ((static_cast<int>(this->tfRootVisual->ChildCount()) == 0) && this->frameManager) {
      rendering::VisualPtr visualFrame = this->createVisualFrame();

      // Display Axis with fixed frame name
      rendering::TextPtr frameName = std::dynamic_pointer_cast<rendering::Text>(
        visualFrame->GeometryByIndex(0));
      frameName->SetTextString(this->frameManager->getFixedFrame());

      tfRootVisual->AddChild(visualFrame);
    }

    update();
  }

  if (_event->type() == rviz::events::FrameListChanged::kType) {
    // Refresh Tree View
    this->refresh();
  }

  return QObject::eventFilter(_object, _event);
}

////////////////////////////////////////////////////////////////////////////////
void TFDisplay::update()
{
  std::lock_guard<std::mutex>(this->lock);

  // Create tf visual frames
  for (int i = tfRootVisual->ChildCount(); i < static_cast<int>(frameInfo.size()); ++i) {
    rendering::VisualPtr visualFrame = this->createVisualFrame();
    this->tfRootVisual->AddChild(visualFrame);
  }

  int i = -1;
  // Update tf visual frames
  for (const auto & frame : frameInfo) {
    i++;
    ignition::math::Pose3d pose, parentPose;

    rendering::VisualPtr visualFrame = std::dynamic_pointer_cast<rendering::Visual>(
      this->tfRootVisual->ChildByIndex(i));

    // Set frame visibility
    visualFrame->SetVisible(frame.second);

    // Skip processing if frame not visible
    if (!frame.second) {
      continue;
    }

    // Set frame text
    rendering::TextPtr frameName = std::dynamic_pointer_cast<rendering::Text>(
      visualFrame->GeometryByIndex(0));
    frameName->SetTextString(frame.first);

    visualFrame->SetVisible(this->namesVisible);

    // Set frame position
    if (this->frameManager->getFramePose(frame.first, pose)) {
      visualFrame->SetLocalPosition(pose.Pos());
    }

    // Set axis orientation
    rendering::AxisVisualPtr axis = std::dynamic_pointer_cast<rendering::AxisVisual>(
      visualFrame->ChildByIndex(1));
    axis->SetLocalRotation(pose.Rot());
    axis->SetLocalScale(this->markerScale);
    axis->SetVisible(this->axesVisible);
    for (int i = 0; i < 3 && this->axesVisible; ++i) {
      auto arrow = std::dynamic_pointer_cast<rendering::ArrowVisual>(axis->ChildByIndex(i));
      arrow->ShowArrowHead(this->axesHeadVisible);
    }

    // Get parent pose for tf links
    bool result = this->frameManager->getParentPose(frame.first, parentPose);
    rendering::ArrowVisualPtr arrow = std::dynamic_pointer_cast<rendering::ArrowVisual>(
      visualFrame->ChildByIndex(0));
    if (result) {
      // Get direction and distance from child to parent frame
      math::Vector3d dir = parentPose.Pos() - pose.Pos();
      double dist = dir.Length();

      if (dist >= MIN_FRAME_DISTANCE) {
        // Update tf arrow visual orientation to point fron child to parent frame
        arrow->SetVisible(this->arrowsVisible);

        math::Quaterniond quat;
        quat.From2Axes(-math::Vector3d::UnitZ, dir);
        quat *= math::Quaterniond::EulerToQuaternion(M_PI, 0, 0);
        arrow->SetLocalRotation(quat);
        arrow->SetLocalScale(this->markerScale, this->markerScale, dist);
      } else {
        arrow->SetVisible(false);
      }
    } else {
      arrow->SetVisible(false);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void TFDisplay::setFrameManager(std::shared_ptr<common::FrameManager> _frameManager)
{
  this->frameManager = std::move(_frameManager);

  // Refresh Tree View
  this->refresh();
}

////////////////////////////////////////////////////////////////////////////////
void TFDisplay::refresh()
{
  std::lock_guard<std::mutex>(this->lock);

  std::vector<std::string> frames;
  this->frameManager->getFrames(frames);

  if (frames.size() > 0) {
    for (const auto frame : frames) {
      this->frameInfo.insert({frame, true});
    }

    // Clear rows
    parentRow->removeRows(0, parentRow->rowCount());

    for (auto frame : frameInfo) {
      this->frameModel->addFrame(QString::fromStdString(frame.first), parentRow);
    }
    // Notify model update
    frameModelChanged();
  }
}

////////////////////////////////////////////////////////////////////////////////
void TFDisplay::LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/)
{
  if (this->title.empty()) {
    this->title = "TF";
  }
}

////////////////////////////////////////////////////////////////////////////////
void TFDisplay::showAxes(const bool & _visible)
{
  std::lock_guard<std::mutex>(this->lock);
  this->axesVisible = _visible;
}

////////////////////////////////////////////////////////////////////////////////
void TFDisplay::showNames(const bool & _visible)
{
  std::lock_guard<std::mutex>(this->lock);
  this->namesVisible = _visible;
}

////////////////////////////////////////////////////////////////////////////////
void TFDisplay::showArrows(const bool & _visible)
{
  std::lock_guard<std::mutex>(this->lock);
  this->arrowsVisible = _visible;
}

////////////////////////////////////////////////////////////////////////////////
void TFDisplay::showAxesHead(const bool & _visible)
{
  std::lock_guard<std::mutex>(this->lock);
  this->axesHeadVisible = _visible;
}

////////////////////////////////////////////////////////////////////////////////
void TFDisplay::setMarkerScale(const float & _scale)
{
  std::lock_guard<std::mutex>(this->lock);
  this->markerScale = _scale * 0.4;
}

////////////////////////////////////////////////////////////////////////////////
void TFDisplay::setFrameVisibility(const QString & _frame, const bool & _visible)
{
  std::lock_guard<std::mutex>(this->lock);

  if (_frame == "All Frames") {
    // Update frame GUI checkboxes
    for (int i = 0; i < parentRow->rowCount(); ++i) {
      parentRow->child(i)->setData(_visible, Qt::CheckStateRole);
    }
    // Update frame visibility info
    for (auto & frame : frameInfo) {
      frame.second = _visible;
    }

    return;
  }

  // Update frame visibility
  bool frameStatus = true;
  for (auto & frame : frameInfo) {
    if (frame.first == _frame.toStdString()) {
      frame.second = _visible;
    }
    frameStatus &= frame.second;
  }

  // All Frames checkbox checked if all child frames are visible
  parentRow->setData(frameStatus, Qt::CheckStateRole);

  // Notify model update
  frameModelChanged();
}

}  // namespace plugins
}  // namespace rviz
}  // namespace ignition

IGNITION_ADD_PLUGIN(
  ignition::rviz::plugins::TFDisplay,
  ignition::gui::Plugin)
