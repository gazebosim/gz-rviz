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

#include "ignition/rviz/plugins/RobotModelDisplay.hpp"

#include <ignition/math.hh>
#include <ignition/math/Color.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/GuiEvents.hh>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <memory>
#include <string>
#include <utility>

namespace ignition
{
namespace rviz
{
namespace plugins
{
////////////////////////////////////////////////////////////////////////////////
RobotModelDisplay::RobotModelDisplay()
: MessageDisplay(), modelLoaded(true)
{
  // Get reference to scene
  this->engine = ignition::rendering::engine("ogre");
  this->scene = this->engine->SceneByName("scene");
}

////////////////////////////////////////////////////////////////////////////////
RobotModelDisplay::~RobotModelDisplay()
{
  std::lock_guard<std::recursive_mutex>(this->lock);
  // Delete visual
  ignition::gui::App()->findChild<ignition::gui::MainWindow *>()->removeEventFilter(this);
  this->scene->DestroyVisual(this->rootVisual, true);
}

////////////////////////////////////////////////////////////////////////////////
void RobotModelDisplay::initialize(rclcpp::Node::SharedPtr _node)
{
  std::lock_guard<std::recursive_mutex>(this->lock);
  this->node = std::move(_node);

  this->qos = this->qos.keep_last(1).transient_local();
}

////////////////////////////////////////////////////////////////////////////////
void RobotModelDisplay::setFrameManager(std::shared_ptr<common::FrameManager> _frameManager)
{
  std::lock_guard<std::recursive_mutex>(this->lock);
  this->frameManager = std::move(_frameManager);
}

////////////////////////////////////////////////////////////////////////////////
void RobotModelDisplay::subscribe()
{
  this->subscriber = this->node->create_subscription<std_msgs::msg::String>(
    this->topic_name,
    this->qos,
    std::bind(&RobotModelDisplay::callback, this, std::placeholders::_1));
}

////////////////////////////////////////////////////////////////////////////////
void RobotModelDisplay::setTopic(const std::string & topic_name)
{
  std::lock_guard<std::recursive_mutex>(this->lock);
  this->topic_name = topic_name;

  this->subscribe();

  // Refresh combo-box on plugin load
  this->onRefresh();
}

////////////////////////////////////////////////////////////////////////////////
void RobotModelDisplay::setTopic(const QString & topic_name)
{
  std::lock_guard<std::recursive_mutex>(this->lock);
  this->topic_name = topic_name.toStdString();

  // Destroy previous subscription
  this->unsubscribe();

  // Create new subscription
  this->subscribe();
}

////////////////////////////////////////////////////////////////////////////////
void RobotModelDisplay::callback(const std_msgs::msg::String::SharedPtr _msg)
{
  std::lock_guard<std::recursive_mutex>(this->lock);
  if (!_msg) {
    return;
  }
  RCLCPP_INFO(this->node->get_logger(), "%s", _msg->data.c_str());
  this->msg = std::move(_msg);

  if (!this->robotModel.initString(this->msg->data)) {
    RCLCPP_ERROR(this->node->get_logger(), "FAILED TO LOAD THE URDF STRING");
  } else {
    RCLCPP_INFO(this->node->get_logger(), "SUCCESSFULLY LOADED THE URDF STRING");

    // Recursively destroy all visuals
    this->scene->DestroyVisual(this->rootVisual, true);
    this->rootVisual = this->scene->CreateVisual();
    this->scene->RootVisual()->AddChild(this->rootVisual);

    this->robotVisualLinks.clear();
    this->modelLoaded = false;
  }
}


////////////////////////////////////////////////////////////////////////////////
bool RobotModelDisplay::eventFilter(QObject * _object, QEvent * _event)
{
  if (_event->type() == gui::events::Render::kType) {
    update();
  }

  return QObject::eventFilter(_object, _event);
}

////////////////////////////////////////////////////////////////////////////////
void RobotModelDisplay::update()
{
  std::lock_guard<std::recursive_mutex>(this->lock);
  // Load model
  if (!this->modelLoaded) {
    loadRobotModel();
    this->modelLoaded = true;
  }

  // Update robot model poses
  for (const auto & link : this->robotVisualLinks) {
    math::Pose3d linkPose, framePose;

    auto linkInfo = this->robotModel.getLink(link.first);
    if (linkInfo->visual != nullptr) {
      const auto & origin = linkInfo->visual->origin;
      linkPose += math::Pose3d(
        origin.position.x, origin.position.y, origin.position.z,
        origin.rotation.w, origin.rotation.x, origin.rotation.y, origin.rotation.z);
    }

    this->frameManager->getFramePose(link.first, framePose);

    link.second->SetLocalPose(linkPose + framePose);
  }
}

////////////////////////////////////////////////////////////////////////////////
void RobotModelDisplay::loadRobotModel()
{
  std::lock_guard<std::recursive_mutex>(this->lock);
  const auto & root = this->robotModel.getRoot();

  addLinkVisual(root.get());

  for (const auto & link : root->child_links) {
    this->createLink(link);
  }
}

////////////////////////////////////////////////////////////////////////////////
void RobotModelDisplay::createLink(const urdf::LinkSharedPtr & _link)
{
  std::lock_guard<std::recursive_mutex>(this->lock);

  addLinkVisual(_link.get());

  for (const auto & link : _link->child_links) {
    this->createLink(link);
  }
}

////////////////////////////////////////////////////////////////////////////////
void RobotModelDisplay::addLinkVisual(const urdf::Link * _link)
{
  std::lock_guard<std::recursive_mutex>(this->lock);
  if (_link->visual == nullptr) {
    return;
  }

  if (_link->visual->geometry == nullptr) {
    return;
  }

  rendering::VisualPtr visual = this->scene->CreateVisual();

  switch (_link->visual->geometry->type) {
    case urdf::Geometry::BOX: {
        auto boxInfo = std::dynamic_pointer_cast<urdf::Box>(_link->visual->geometry);
        rendering::GeometryPtr box = this->scene->CreateBox();
        visual->AddGeometry(box);
        visual->SetLocalScale(boxInfo->dim.x, boxInfo->dim.y, boxInfo->dim.z);
        break;
      }
    case urdf::Geometry::SPHERE: {
        auto sphereInfo = std::dynamic_pointer_cast<urdf::Sphere>(_link->visual->geometry);
        rendering::GeometryPtr sphere = this->scene->CreateSphere();
        visual->AddGeometry(sphere);
        visual->SetLocalScale(sphereInfo->radius * 2.0);
        break;
      }
    case urdf::Geometry::CYLINDER: {
        auto cylinderInfo = std::dynamic_pointer_cast<urdf::Cylinder>(_link->visual->geometry);
        rendering::GeometryPtr cylinder = this->scene->CreateCylinder();
        visual->AddGeometry(cylinder);
        visual->SetLocalScale(
          cylinderInfo->radius * 2, cylinderInfo->radius * 2,
          cylinderInfo->length);
        break;
      }
    case urdf::Geometry::MESH: {
        auto meshInfo = std::dynamic_pointer_cast<urdf::Mesh>(_link->visual->geometry);

        if (meshInfo->filename.rfind("package://") == 0) {
          int p = meshInfo->filename.find_first_of('/', 10);
          auto package_name = meshInfo->filename.substr(10, p - 10);


          try {
            std::string filepath = ament_index_cpp::get_package_share_directory(package_name);
            filepath += meshInfo->filename.substr(p);
            // Load Mesh
            rendering::MeshDescriptor descriptor;
            descriptor.meshName = filepath;
            ignition::common::MeshManager * meshManager = ignition::common::MeshManager::Instance();
            descriptor.mesh = meshManager->Load(descriptor.meshName);
            rendering::MeshPtr mesh = this->scene->CreateMesh(descriptor);

            visual->AddGeometry(mesh);
            visual->SetLocalScale(meshInfo->scale.x, meshInfo->scale.y, meshInfo->scale.z);
          } catch (ament_index_cpp::PackageNotFoundError & e) {
            RCLCPP_ERROR(this->node->get_logger(), e.what());
            this->scene->DestroyVisual(visual);
            return;
          }

        } else if (meshInfo->filename.rfind("file://") == 0) {
          // Load Mesh
          rendering::MeshDescriptor descriptor;
          descriptor.meshName = meshInfo->filename.substr(6);
          ignition::common::MeshManager * meshManager = ignition::common::MeshManager::Instance();
          descriptor.mesh = meshManager->Load(descriptor.meshName);
          rendering::MeshPtr mesh = this->scene->CreateMesh(descriptor);

          visual->AddGeometry(mesh);
          visual->SetLocalScale(meshInfo->scale.x, meshInfo->scale.y, meshInfo->scale.z);
        }
        break;
      }
  }

  visual->SetGeometryMaterial(this->scene->Material("Default/TransRed"));
  this->rootVisual->AddChild(visual);
  this->robotVisualLinks.insert({_link->name, visual});
}

////////////////////////////////////////////////////////////////////////////////
void RobotModelDisplay::reset() {}

////////////////////////////////////////////////////////////////////////////////
void RobotModelDisplay::openFile(const QString & _file)
{
  std::lock_guard<std::recursive_mutex>(this->lock);

  if (!this->robotModel.initFile(_file.toStdString().substr(7))) {
    RCLCPP_ERROR(this->node->get_logger(), "FAILED TO LOAD THE FILE");
  } else {
    RCLCPP_INFO(this->node->get_logger(), "SUCCESSFULLY LOADED THE FILE");

    // Recursively destroy all visuals
    this->scene->DestroyVisual(this->rootVisual, true);
    this->rootVisual = this->scene->CreateVisual();
    this->scene->RootVisual()->AddChild(this->rootVisual);

    this->robotVisualLinks.clear();
    this->modelLoaded = false;
  }
}

////////////////////////////////////////////////////////////////////////////////
QStringList RobotModelDisplay::getTopicList() const
{
  return this->topicList;
}

////////////////////////////////////////////////////////////////////////////////
void RobotModelDisplay::onRefresh()
{
  std::lock_guard<std::recursive_mutex>(this->lock);

  // Clear
  this->topicList.clear();

  int index = 0, position = 0;

  // Get topic list
  auto topics = this->node->get_topic_names_and_types();
  for (const auto & topic : topics) {
    for (const auto & topicType : topic.second) {
      if (topicType == "std_msgs/msg/String") {
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
void RobotModelDisplay::updateQoS(
  const int & _depth, const int & _history, const int & _reliability,
  const int & _durability)
{
  std::lock_guard<std::recursive_mutex>(this->lock);
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
void RobotModelDisplay::LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/)
{
  if (this->title.empty()) {
    this->title = "RobotModel";
  }
}

}  // namespace plugins
}  // namespace rviz
}  // namespace ignition


IGNITION_ADD_PLUGIN(
  ignition::rviz::plugins::RobotModelDisplay,
  ignition::gui::Plugin)
