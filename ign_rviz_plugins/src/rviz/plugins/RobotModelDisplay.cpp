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

#include <ignition/gui/Application.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/math/Color.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/plugin/Register.hh>

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
RobotLinkModel::RobotLinkModel(QObject * _parent)
: QStandardItemModel(_parent)
{}

////////////////////////////////////////////////////////////////////////////////
void RobotLinkModel::addLink(const QString & _name, QStandardItem * _parentItem)
{
  QStandardItem * linkRow = new QStandardItem();
  linkRow->setData(_name, NameRole);
  linkRow->setCheckState(Qt::CheckState::Checked);
  _parentItem->appendRow(linkRow);
}

////////////////////////////////////////////////////////////////////////////////
QStandardItem * RobotLinkModel::addParentRow(const QString & _name)
{
  QStandardItem * entry = new QStandardItem();
  entry->setData(_name, NameRole);
  entry->setCheckState(Qt::CheckState::Checked);
  appendRow(entry);
  return entry;
}

////////////////////////////////////////////////////////////////////////////////
QVariant RobotLinkModel::data(const QModelIndex & _index, int _role) const
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
QHash<int, QByteArray> RobotLinkModel::roleNames() const
{
  QHash<int, QByteArray> roles;
  roles[NameRole] = "name";
  roles[Qt::CheckStateRole] = "checked";

  return roles;
}

////////////////////////////////////////////////////////////////////////////////
RobotModelDisplay::RobotModelDisplay()
: MessageDisplay(), modelLoaded(true), destroyModel(false), showVisual(true), showCollision(false),
  dirty(false), alpha(1.0)
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

  this->robotLinkModel = new RobotLinkModel();
  parentRow = this->robotLinkModel->addParentRow(QString::fromStdString("All Links"));
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
    if (link.second.visual != nullptr) {
      math::Pose3d linkPose;
      if (linkInfo->visual != nullptr) {
        const auto & origin = linkInfo->visual->origin;
        linkPose += math::Pose3d(
          origin.position.x, origin.position.y, origin.position.z,
          origin.rotation.w, origin.rotation.x, origin.rotation.y, origin.rotation.z);
      }

      link.second.visual->SetLocalPose(linkPose + framePose);
      link.second.visual->SetVisible(showVisual && link.second.visible);

      // Update alpha if dirty
      if (dirty) {
        auto mat = link.second.visual->Material();

        if (mat == nullptr) {
          // Update alpha of mesh with textures
          auto geometry = link.second.visual->GeometryByIndex(0);
          if (geometry != nullptr) {
            auto geometryMat = geometry->Material();
            if (geometryMat != nullptr) {
              geometryMat->SetTransparency(1 - this->alpha);
              geometry->SetMaterial(geometryMat);
            }
          }
          continue;
        }
        auto color = mat->Ambient();
        color.A(this->alpha);
        mat->SetAmbient(color);
        mat->SetDiffuse(color);
        mat->SetEmissive(color);
        link.second.visual->SetMaterial(mat);
      }
    }

    // Update link collision pose and visibility
    if (link.second.collision != nullptr) {
      math::Pose3d linkPose;
      if (linkInfo->collision != nullptr) {
        const auto & origin = linkInfo->collision->origin;
        linkPose += math::Pose3d(
          origin.position.x, origin.position.y, origin.position.z,
          origin.rotation.w, origin.rotation.x, origin.rotation.y, origin.rotation.z);
      }

      link.second.collision->SetLocalPose(linkPose + framePose);
      link.second.collision->SetVisible(showCollision && link.second.visible);
    }
  }
  dirty = false;
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

  // Create links
  createLink(root.get());

  for (const auto & link : root->child_links) {
    this->addLink(link);
  }

  // Populate tree view
  for (const auto & link : this->robotVisualLinks) {
    this->robotLinkModel->addLink(QString::fromStdString(link.first), this->parentRow);
  }
  robotLinkModelChanged();
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
  RobotLinkProperties robotLink;

  // Add visual for link visual element
  if (_link->visual != nullptr && _link->visual->geometry != nullptr) {
    robotLink.visual = createLinkGeometry(_link->visual->geometry);
    if (robotLink.visual != nullptr) {
      // Set material only if the visual mesh doesn't have textures
      bool meshWithTexture = false;
      const auto meshInfo = std::dynamic_pointer_cast<urdf::Mesh>(_link->visual->geometry);
      if (meshInfo != nullptr) {
        const auto fileExtension = meshInfo->filename.substr(meshInfo->filename.size() - 4);
        meshWithTexture = (fileExtension == ".dae" || fileExtension == ".obj");
      }

      if (meshWithTexture) {
        // Set alpha of mesh with textures
        auto geometry = robotLink.visual->GeometryByIndex(0);
        if (geometry != nullptr) {
          auto mat = geometry->Material();
          if (mat != nullptr) {
            mat->SetTransparency(1 - this->alpha);
            geometry->SetMaterial(mat);
          }
        }
      } else if (_link->visual->material == nullptr) {
        // Use default material
        const auto mat = this->scene->Material("RobotModel/Red");
        // Update alpha
        auto color = mat->Ambient();
        color.A(this->alpha);
        mat->SetAmbient(color);
        mat->SetDiffuse(color);
        mat->SetEmissive(color);
        robotLink.visual->SetMaterial(mat);
      } else if (!_link->visual->material_name.empty()) {
        // Use registered material
        const auto mat = this->scene->Material(_link->visual->material_name);
        // Update alpha
        auto color = mat->Ambient();
        color.A(this->alpha);
        mat->SetAmbient(color);
        mat->SetDiffuse(color);
        mat->SetEmissive(color);
        robotLink.visual->SetMaterial(mat);
      } else {
        // Create new material
        rendering::MaterialPtr mat = this->scene->CreateMaterial();
        const auto & color = _link->visual->material->color;
        mat->SetAmbient(color.r, color.g, color.b, this->alpha);
        mat->SetDiffuse(color.r, color.g, color.b, this->alpha);
        mat->SetEmissive(color.r, color.g, color.b, this->alpha);
        robotLink.visual->SetMaterial(mat);
      }
      this->rootVisual->AddChild(robotLink.visual);
    }
  }

  // Add visual link collision element
  if (_link->collision != nullptr && _link->collision->geometry != nullptr) {
    robotLink.collision = createLinkGeometry(_link->collision->geometry);
    if (robotLink.collision != nullptr) {
      robotLink.collision->SetMaterial(this->scene->Material("Default/TransBlue"));
      this->rootVisual->AddChild(robotLink.collision);
    }
  }

  this->robotVisualLinks.insert({_link->name, robotLink});
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
          descriptor.meshName = meshInfo->filename.substr(7);
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
  // Clear tree view
  this->parentRow->removeRows(0, parentRow->rowCount());
  robotLinkModelChanged();

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
  // Clear tree view
  this->parentRow->removeRows(0, parentRow->rowCount());
  robotLinkModelChanged();

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
  this->alpha = _alpha;
  this->dirty = true;
}

////////////////////////////////////////////////////////////////////////////////
void RobotModelDisplay::setLinkVisibility(const QString & _link, const bool & _visible)
{
  std::lock_guard<std::recursive_mutex>(this->lock);

  if (_link == "All Links") {
    // Update frame GUI checkboxes
    for (int i = 0; i < parentRow->rowCount(); ++i) {
      parentRow->child(i)->setData(_visible, Qt::CheckStateRole);
    }
    // Update frame visibility info
    for (auto & link : robotVisualLinks) {
      link.second.visible = _visible;
    }
    return;
  }

  // Update frame visibility
  bool linkStatus = true;
  for (auto & link : robotVisualLinks) {
    if (link.first == _link.toStdString()) {
      link.second.visible = _visible;
    }
    linkStatus &= link.second.visible;
  }

  // All Frames checkbox checked if all child frames are visible
  parentRow->setData(linkStatus, Qt::CheckStateRole);

  // Notify model update
  robotLinkModelChanged();
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
