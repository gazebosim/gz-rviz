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
: MessageDisplay(), modelLoaded(true), destroyModel(false), showVisual(true), showCollision(false)
{
  // Get reference to scene
  this->engine = ignition::rendering::engine("ogre");
  this->scene = this->engine->SceneByName("scene");

  // Create red material if not registered
  if (!this->scene->MaterialRegistered("RobotModel/Red")) {
    rendering::MaterialPtr mat = this->scene->CreateMaterial("RobotModel/Red");
    mat->SetAmbient(ignition::math::Color::Red);
    mat->SetDiffuse(ignition::math::Color::Red);
    mat->SetEmissive(ignition::math::Color::Red);
  }
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
  // Reset visualization
  this->reset();
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

  this->msg = std::move(_msg);

  if (!this->robotModel.initString(this->msg->data)) {
    RCLCPP_ERROR(this->node->get_logger(), "FAILED TO LOAD THE URDF STRING");
  } else {
    this->destroyModel = true;
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

  if (this->destroyModel) {
    // Recursively destroy all visuals
    if (this->rootVisual != nullptr) {
      this->scene->DestroyVisual(this->rootVisual, true);
      this->rootVisual.reset();
    }

    this->robotVisualLinks.clear();
    this->destroyModel = false;
  }

  // Load model
  if (!this->modelLoaded) {
    loadRobotModel();
    this->modelLoaded = true;
  }

  // Update robot model poses
  for (const auto & link : this->robotVisualLinks) {
    math::Pose3d framePose;
    auto linkInfo = this->robotModel.getLink(link.first);
    this->frameManager->getFramePose(link.first, framePose);

    // Update link visual pose and visibility
    if (link.second.first != nullptr) {
      math::Pose3d linkPose;
      if (linkInfo->visual != nullptr) {
        const auto & origin = linkInfo->visual->origin;
        linkPose += math::Pose3d(
          origin.position.x, origin.position.y, origin.position.z,
          origin.rotation.w, origin.rotation.x, origin.rotation.y, origin.rotation.z);
      }

      link.second.first->SetLocalPose(linkPose + framePose);
      link.second.first->SetVisible(showVisual);
    }

    // Update link collision pose and visibility
    if (link.second.second != nullptr) {
      math::Pose3d linkPose;
      if (linkInfo->collision != nullptr) {
        const auto & origin = linkInfo->collision->origin;
        linkPose += math::Pose3d(
          origin.position.x, origin.position.y, origin.position.z,
          origin.rotation.w, origin.rotation.x, origin.rotation.y, origin.rotation.z);
      }

      link.second.second->SetLocalPose(linkPose + framePose);
      link.second.second->SetVisible(showCollision);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void RobotModelDisplay::loadRobotModel()
{
  std::lock_guard<std::recursive_mutex>(this->lock);

  if (this->rootVisual == nullptr) {
    this->rootVisual = this->scene->CreateVisual();
    this->scene->RootVisual()->AddChild(this->rootVisual);
  }

  const auto & root = this->robotModel.getRoot();

  createLink(root.get());

  const auto & materials = this->robotModel.materials_;
  for (const auto & material : materials) {
    // Skip registering material if already registered or material has not name
    if (material.first.empty() || this->scene->MaterialRegistered(material.first)) {
      continue;
    }

    rendering::MaterialPtr mat = this->scene->CreateMaterial(material.first);
    const auto & color = material.second->color;
    mat->SetAmbient(color.r, color.g, color.b, color.a);
    mat->SetDiffuse(color.r, color.g, color.b, color.a);
    mat->SetEmissive(color.r, color.g, color.b, color.a);
  }

  for (const auto & link : root->child_links) {
    this->addLink(link);
  }
}

////////////////////////////////////////////////////////////////////////////////
void RobotModelDisplay::addLink(const urdf::LinkSharedPtr & _link)
{
  std::lock_guard<std::recursive_mutex>(this->lock);

  createLink(_link.get());

  for (const auto & link : _link->child_links) {
    this->addLink(link);
  }
}

////////////////////////////////////////////////////////////////////////////////
void RobotModelDisplay::createLink(const urdf::Link * _link)
{
  std::lock_guard<std::recursive_mutex>(this->lock);

  rendering::VisualPtr linkVisual = nullptr, linkCollision = nullptr;

  // Add visual for link visual element
  if (_link->visual != nullptr && _link->visual->geometry != nullptr) {
    linkVisual = createLinkGeometry(_link->visual->geometry);
    if (linkVisual != nullptr) {
      if (_link->visual->material == nullptr) {
        // Use default material
        linkVisual->SetMaterial(this->scene->Material("RobotModel/Red"));
      } else if (!_link->visual->material_name.empty()) {
        // Use registered material
        linkVisual->SetMaterial(this->scene->Material(_link->visual->material_name));
      } else {
        // Create new material
        rendering::MaterialPtr mat = this->scene->CreateMaterial();
        const auto & color = _link->visual->material->color;
        mat->SetAmbient(color.r, color.g, color.b, color.a);
        mat->SetDiffuse(color.r, color.g, color.b, color.a);
        mat->SetEmissive(color.r, color.g, color.b, color.a);
        linkVisual->SetMaterial(mat);
      }
      this->rootVisual->AddChild(linkVisual);
    }
  }

  // Add visual link collision element
  if (_link->collision != nullptr && _link->collision->geometry != nullptr) {
    linkCollision = createLinkGeometry(_link->collision->geometry);
    if (linkCollision != nullptr) {
      linkCollision->SetMaterial(this->scene->Material("Default/TransBlue"));
      this->rootVisual->AddChild(linkCollision);
    }
  }

  this->robotVisualLinks.insert({_link->name, std::make_pair(linkVisual, linkCollision)});
}

////////////////////////////////////////////////////////////////////////////////
rendering::VisualPtr RobotModelDisplay::createLinkGeometry(
  const urdf::GeometrySharedPtr & _geometry)
{
  rendering::VisualPtr visual = this->scene->CreateVisual();

  switch (_geometry->type) {
    case urdf::Geometry::BOX: {
        auto boxInfo = std::dynamic_pointer_cast<urdf::Box>(_geometry);
        rendering::GeometryPtr box = this->scene->CreateBox();
        visual->AddGeometry(box);
        visual->SetLocalScale(boxInfo->dim.x, boxInfo->dim.y, boxInfo->dim.z);
        break;
      }
    case urdf::Geometry::SPHERE: {
        auto sphereInfo = std::dynamic_pointer_cast<urdf::Sphere>(_geometry);
        rendering::GeometryPtr sphere = this->scene->CreateSphere();
        visual->AddGeometry(sphere);
        visual->SetLocalScale(sphereInfo->radius * 2.0);
        break;
      }
    case urdf::Geometry::CYLINDER: {
        auto cylinderInfo = std::dynamic_pointer_cast<urdf::Cylinder>(_geometry);
        rendering::GeometryPtr cylinder = this->scene->CreateCylinder();
        visual->AddGeometry(cylinder);
        visual->SetLocalScale(
          cylinderInfo->radius * 2, cylinderInfo->radius * 2,
          cylinderInfo->length);
        break;
      }
    case urdf::Geometry::MESH: {
        auto meshInfo = std::dynamic_pointer_cast<urdf::Mesh>(_geometry);

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
            return nullptr;
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

  return visual;
}

////////////////////////////////////////////////////////////////////////////////
void RobotModelDisplay::reset()
{
  std::lock_guard<std::recursive_mutex>(this->lock);
  this->destroyModel = true;
}

////////////////////////////////////////////////////////////////////////////////
void RobotModelDisplay::sourceChanged(const int & _source)
{
  std::lock_guard<std::recursive_mutex>(this->lock);
  // Source: Topic
  if (_source == 0) {
    this->reset();
    this->subscribe();
    return;
  }
  // Source: File
  this->unsubscribe();
  this->reset();
}

////////////////////////////////////////////////////////////////////////////////
void RobotModelDisplay::openFile(const QString & _file)
{
  std::lock_guard<std::recursive_mutex>(this->lock);
  // Reset model visualziation
  this->destroyModel = true;

  if (_file.isEmpty()) {
    RCLCPP_ERROR(this->node->get_logger(), "FAILED TO LOAD THE FILE");
    return;
  }

  std::string file = _file.toStdString();
  if (_file.startsWith("file://")) {
    file = _file.mid(7).toStdString();
  }

  if (!this->robotModel.initFile(file)) {
    RCLCPP_ERROR(this->node->get_logger(), "FAILED TO LOAD THE FILE");
  } else {
    this->modelLoaded = false;
  }
}

////////////////////////////////////////////////////////////////////////////////
void RobotModelDisplay::visualEnabled(const bool & _enabled)
{
  std::lock_guard<std::recursive_mutex>(this->lock);
  this->showVisual = _enabled;
}

////////////////////////////////////////////////////////////////////////////////
void RobotModelDisplay::collisionEnabled(const bool & _enabled)
{
  std::lock_guard<std::recursive_mutex>(this->lock);
  this->showCollision = _enabled;
}

////////////////////////////////////////////////////////////////////////////////
void RobotModelDisplay::setAlpha(const float & _alpha)
{
  std::lock_guard<std::recursive_mutex>(this->lock);

  for (const auto & link : this->robotVisualLinks) {
    if (link.second.first != nullptr) {
      auto mat = link.second.first->Material();
      auto color = mat->Ambient();
      color.A(_alpha);
      mat->SetAmbient(color);
      mat->SetDiffuse(color);
      mat->SetEmissive(color);
      link.second.first->SetMaterial(mat);
    }
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
