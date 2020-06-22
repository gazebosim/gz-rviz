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

#include "ignition/rviz/plugins/tf_display.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <ignition/math.hh>
#include <ignition/math/Color.hh>
#include <ignition/gui/GuiEvents.hh>

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
#define MIN_FRAME_DISTANCE 0.25

////////////////////////////////////////////////////////////////////////////////
TFDisplay::TFDisplay()
: MessageDisplay()
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
}

////////////////////////////////////////////////////////////////////////////////
TFDisplay::~TFDisplay()
{}

////////////////////////////////////////////////////////////////////////////////
void TFDisplay::loadGUIConfig(gui::Application *app) {
  RCLCPP_INFO(this->node->get_logger(), "Loading plugin qml");
  std::string filename = "tf_display";
  this->context = new QQmlContext(app->Engine()->rootContext());
  this->context->setContextProperty(QString::fromStdString(filename), this);

  // Instantiate plugin QML file into a component
  std::string qmlFile(":/" + filename + "/" + filename + ".qml");
  QQmlComponent component(app->Engine(), QString::fromStdString(qmlFile));

  // Create an item for the plugin
  this->pluginItem = qobject_cast<QQuickItem *>(component.create(this->context));
  if (!this->pluginItem)
  {
    ignerr << "Failed to instantiate QML file [" << qmlFile << "]." << std::endl
           << "* Are you sure it's been added to the .qrc file?" << std::endl
           << "* Are you sure the file is valid QML? "
           << "You can check with the `qmlscene` tool" << std::endl;
    return;
  }
  this->title = "TF Plugin";
  RCLCPP_INFO(this->node->get_logger(), "Success!");
}

////////////////////////////////////////////////////////////////////////////////
void TFDisplay::initialize(rclcpp::Node::SharedPtr node)
{
  this->node = std::move(node);
}

////////////////////////////////////////////////////////////////////////////////
void TFDisplay::setTopic(std::string topic_name)
{
  this->topic_name = topic_name;
  this->subscriber = this->node->create_subscription<tf2_msgs::msg::TFMessage>(
    this->topic_name, 10,
    std::bind(&TFDisplay::callback, this, std::placeholders::_1));
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
  axis->SetLocalScale(0.5);
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
bool TFDisplay::eventFilter(QObject * object, QEvent * event)
{
  if (event->type() == gui::events::Render::kType) {
    // Create a default visual frame
    if ((static_cast<int>(this->tfRootVisual->ChildCount()) == 0) && this->frameManager) {
      rendering::VisualPtr visualFrame = this->createVisualFrame();

      // Display Axis with fixed frame name
      rendering::TextPtr frameName = std::dynamic_pointer_cast<rendering::Text>(
        visualFrame->GeometryByIndex(0));
      frameName->SetTextString(this->frameManager->getFixedFrame());

      tfRootVisual->AddChild(visualFrame);
    }

    updateTF();
  }

  return QObject::eventFilter(object, event);
}

////////////////////////////////////////////////////////////////////////////////
void TFDisplay::updateTF()
{
  std::lock_guard<std::mutex>(this->lock);

  // Get available tf frames
  std::vector<std::string> frameIds;
  frameManager->getFrames(frameIds);

  // Create tf visual frames
  for (int i = tfRootVisual->ChildCount(); i < static_cast<int>(frameIds.size()); ++i) {
    rendering::VisualPtr visualFrame = this->createVisualFrame();
    this->tfRootVisual->AddChild(visualFrame);
  }

  // Update tf visual frames
  for (int i = 0; i < static_cast<int>(frameIds.size()); ++i) {
    ignition::math::Pose3d pose, parentPose;

    rendering::VisualPtr visualFrame = std::dynamic_pointer_cast<rendering::Visual>(
      this->tfRootVisual->ChildByIndex(i));

    bool result = this->frameManager->getFramePose(frameIds[i], pose);

    // Set frame text
    rendering::TextPtr frameName = std::dynamic_pointer_cast<rendering::Text>(
      visualFrame->GeometryByIndex(0));
    frameName->SetTextString(frameIds[i]);

    // Set frame position
    visualFrame->SetLocalPosition(pose.Pos());

    // Set axis orientation
    rendering::AxisVisualPtr axis = std::dynamic_pointer_cast<rendering::AxisVisual>(
      visualFrame->ChildByIndex(1));
    axis->SetLocalRotation(pose.Rot());

    // Get parent pose for tf links
    result = this->frameManager->getParentPose(frameIds[i], parentPose);
    if (result) {
      rendering::ArrowVisualPtr arrow = std::dynamic_pointer_cast<rendering::ArrowVisual>(
        visualFrame->ChildByIndex(
          0));

      // Get direction and distance from child to parent frame
      math::Vector3d dir = parentPose.Pos() - pose.Pos();
      double dist = dir.Length();

      if (dist >= MIN_FRAME_DISTANCE) {
        // Update tf arrow visual orientation to point fron child to parent frame
        arrow->SetVisible(true);
        math::Quaterniond quat;
        quat.From2Axes(-math::Vector3d::UnitZ, dir);
        quat *= math::Quaterniond::EulerToQuaternion(M_PI, 0, 0);
        arrow->SetLocalRotation(quat);
        arrow->SetLocalScale(1, 1, dist);
      } else {
        arrow->SetVisible(false);
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void TFDisplay::installEventFilter(ignition::gui::MainWindow * window)
{
  window->installEventFilter(this);
}

////////////////////////////////////////////////////////////////////////////////
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
