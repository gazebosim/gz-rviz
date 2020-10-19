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

#include "ignition/rviz/plugins/MarkerManager.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <string>

namespace ignition
{
namespace rviz
{
namespace plugins
{
////////////////////////////////////////////////////////////////////////////////
MarkerManager::MarkerManager()
{
  // Get reference to scene
  this->engine = ignition::rendering::engine("ogre");
  this->scene = this->engine->SceneByName("scene");

  this->rootVisual = this->scene->CreateVisual();
  this->scene->RootVisual()->AddChild(this->rootVisual);
}

////////////////////////////////////////////////////////////////////////////////
MarkerManager::~MarkerManager()
{
  // Delete all markers
  this->scene->DestroyVisual(this->rootVisual, true);
}

////////////////////////////////////////////////////////////////////////////////
void MarkerManager::processMessage(const visualization_msgs::msg::MarkerArray & _msg)
{
  for (const auto & markerMsg : _msg.markers) {
    processMessage(markerMsg);
  }
}

////////////////////////////////////////////////////////////////////////////////
void MarkerManager::processMessage(const visualization_msgs::msg::Marker & _msg)
{
  switch (_msg.action) {
    case visualization_msgs::msg::Marker::ADD: {
        createMarker(_msg);
        break;
      }
    case visualization_msgs::msg::Marker::DELETE: {
        deleteMarker(_msg.id);
        break;
      }
    case visualization_msgs::msg::Marker::DELETEALL: {
        deleteAllMarkers();
        break;
      }
  }
}

////////////////////////////////////////////////////////////////////////////////
void MarkerManager::createMarker(const visualization_msgs::msg::Marker & _msg)
{
  switch (_msg.type) {
    case visualization_msgs::msg::Marker::ARROW: {
        createArrowMarker(_msg);
        break;
      }
    case visualization_msgs::msg::Marker::CUBE: {
        createBasicGeometry(_msg, rendering::MarkerType::MT_BOX);
        break;
      }
    case visualization_msgs::msg::Marker::SPHERE: {
        createBasicGeometry(_msg, rendering::MarkerType::MT_SPHERE);
        break;
      }
    case visualization_msgs::msg::Marker::CYLINDER: {
        createBasicGeometry(_msg, rendering::MarkerType::MT_CYLINDER);
        break;
      }
    case visualization_msgs::msg::Marker::LINE_STRIP: {
        createListGeometry(_msg, rendering::MarkerType::MT_LINE_STRIP);
        break;
      }
    case visualization_msgs::msg::Marker::LINE_LIST: {
        createListGeometry(_msg, rendering::MarkerType::MT_LINE_LIST);
        break;
      }
    case visualization_msgs::msg::Marker::TRIANGLE_LIST: {
        createListGeometry(_msg, rendering::MarkerType::MT_TRIANGLE_LIST);
        break;
      }
    case visualization_msgs::msg::Marker::POINTS: {
        createListGeometry(_msg, rendering::MarkerType::MT_POINTS);
        break;
      }
    case visualization_msgs::msg::Marker::CUBE_LIST: {
        createListVisual(_msg);
        break;
      }
    case visualization_msgs::msg::Marker::SPHERE_LIST: {
        createListVisual(_msg);
        break;
      }
    case visualization_msgs::msg::Marker::TEXT_VIEW_FACING: {
        createTextMarker(_msg);
        break;
      }
    case visualization_msgs::msg::Marker::MESH_RESOURCE: {
        createMeshMarker(_msg);
        break;
      }
  }
}

////////////////////////////////////////////////////////////////////////////////
void MarkerManager::createBasicGeometry(
  const visualization_msgs::msg::Marker & _msg,
  rendering::MarkerType _geometryType)
{
  rendering::VisualPtr visual = this->scene->CreateVisual();
  insertOrUpdateVisual(_msg.id, visual);

  // Create marker
  auto marker = this->scene->CreateMarker();
  marker->SetType(_geometryType);

  // Set material
  marker->SetMaterial(createMaterial(_msg.color));

  // Add geometry and set scale
  visual->AddGeometry(marker);
  visual->SetLocalScale(_msg.scale.x, _msg.scale.y, _msg.scale.z);
  visual->SetLocalPose(msgToPose(_msg.pose));

  this->rootVisual->AddChild(visual);
}

////////////////////////////////////////////////////////////////////////////////
void MarkerManager::createListGeometry(
  const visualization_msgs::msg::Marker & _msg,
  rendering::MarkerType _geometryType)
{
  rendering::VisualPtr visual = this->scene->CreateVisual();
  insertOrUpdateVisual(_msg.id, visual);

  auto marker = this->scene->CreateMarker();
  marker->SetType(_geometryType);

  if (_msg.colors.size() == _msg.points.size()) {
    for (unsigned int i = 0; i < _msg.points.size(); ++i) {
      const auto & point = _msg.points[i];
      const auto color = math::Color(
        _msg.colors[i].r, _msg.colors[i].g, _msg.colors[i].b, _msg.colors[i].a);
      marker->AddPoint(point.x, point.y, point.z, color);
    }
  } else {
    if (_msg.colors.size() != 0) {
      RCLCPP_WARN(
        rclcpp::get_logger("MarkerManager"), "Marker color and point array size doesn't match.");
    }
    const auto color = math::Color(_msg.color.r, _msg.color.g, _msg.color.b, _msg.color.a);
    for (const auto & point : _msg.points) {
      marker->AddPoint(point.x, point.y, point.z, color);
    }
  }

  // This material is not used anywhere but is required to set
  // point color in marker AddPoint method
  marker->SetMaterial(this->scene->Material("Default/TransGreen"));

  visual->AddGeometry(marker);
  visual->SetLocalPose(msgToPose(_msg.pose));

  this->rootVisual->AddChild(visual);
}

////////////////////////////////////////////////////////////////////////////////
void MarkerManager::createArrowMarker(const visualization_msgs::msg::Marker & _msg)
{
  auto visual = this->scene->CreateArrowVisual();
  insertOrUpdateVisual(_msg.id, visual);

  visual->SetMaterial(createMaterial(_msg.color));
  visual->SetLocalScale(_msg.scale.x, _msg.scale.y, _msg.scale.z);

  math::Pose3d pose = msgToPose(_msg.pose);
  visual->SetLocalPosition(pose.Pos());
  visual->SetLocalRotation(pose.Rot() * math::Quaterniond(0, 1.57, 0));

  this->rootVisual->AddChild(visual);
}

////////////////////////////////////////////////////////////////////////////////
void MarkerManager::createTextMarker(const visualization_msgs::msg::Marker & _msg)
{
  rendering::VisualPtr visual = this->scene->CreateVisual();
  insertOrUpdateVisual(_msg.id, visual);

  // Create text marker
  auto textMarker = this->scene->CreateText();
  textMarker->SetTextString(_msg.text);
  textMarker->SetShowOnTop(true);
  textMarker->SetTextAlignment(
    rendering::TextHorizontalAlign::CENTER,
    rendering::TextVerticalAlign::CENTER);
  textMarker->SetCharHeight(0.15);
  textMarker->SetMaterial(createMaterial(_msg.color));

  // Add geometry and set scale
  visual->AddGeometry(textMarker);
  visual->SetLocalScale(_msg.scale.x, _msg.scale.y, _msg.scale.z);

  visual->SetLocalPose(msgToPose(_msg.pose));

  this->rootVisual->AddChild(visual);
}

////////////////////////////////////////////////////////////////////////////////
void MarkerManager::createMeshMarker(const visualization_msgs::msg::Marker & _msg)
{
  rendering::MeshDescriptor descriptor;

  if (_msg.mesh_resource.rfind("package://") == 0) {
    int p = _msg.mesh_resource.find_first_of('/', 10);
    auto package_name = _msg.mesh_resource.substr(10, p - 10);

    try {
      std::string filepath = ament_index_cpp::get_package_share_directory(package_name);
      filepath += _msg.mesh_resource.substr(p);
      descriptor.meshName = filepath;
    } catch (ament_index_cpp::PackageNotFoundError & e) {
      RCLCPP_ERROR(rclcpp::get_logger("MarkerManager"), e.what());
      return;
    }
  } else if (_msg.mesh_resource.rfind("file://") == 0) {
    descriptor.meshName = _msg.mesh_resource.substr(7);
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "MarkerManager"), "Unable to find file %s", _msg.mesh_resource.c_str());
    return;
  }

  // Load Mesh
  ignition::common::MeshManager * meshManager = ignition::common::MeshManager::Instance();
  descriptor.mesh = meshManager->Load(descriptor.meshName);

  // Error loading mesh
  if (descriptor.mesh == nullptr) {
    return;
  }

  rendering::MeshPtr mesh = this->scene->CreateMesh(descriptor);

  rendering::VisualPtr visual = this->scene->CreateVisual();
  insertOrUpdateVisual(_msg.id, visual);

  if (!_msg.mesh_use_embedded_materials) {
    mesh->SetMaterial(createMaterial(_msg.color));
  }

  visual->AddGeometry(mesh);
  visual->SetLocalScale(_msg.scale.x, _msg.scale.y, _msg.scale.z);

  visual->SetLocalPose(msgToPose(_msg.pose));

  this->rootVisual->AddChild(visual);
}

////////////////////////////////////////////////////////////////////////////////
void MarkerManager::createListVisual(const visualization_msgs::msg::Marker & _msg)
{
  rendering::VisualPtr visual = this->scene->CreateVisual();
  insertOrUpdateVisual(_msg.id, visual);

  if (_msg.colors.size() == _msg.points.size()) {
    for (unsigned int i = 0; i < _msg.points.size(); ++i) {
      auto geometry = (_msg.type == visualization_msgs::msg::Marker::CUBE_LIST) ?
        this->scene->CreateBox() : this->scene->CreateSphere();

      geometry->SetMaterial(createMaterial(_msg.colors[i]));
      auto marker = this->scene->CreateVisual();
      marker->SetLocalPosition(_msg.points[i].x, _msg.points[i].y, _msg.points[i].z);
      marker->SetLocalScale(_msg.scale.x, _msg.scale.y, _msg.scale.z);
      marker->AddGeometry(geometry);

      visual->AddChild(marker);
    }
  } else {
    auto mat = createMaterial(_msg.color);
    for (const auto & point : _msg.points) {
      auto geometry = (_msg.type == visualization_msgs::msg::Marker::CUBE_LIST) ?
        this->scene->CreateBox() : this->scene->CreateSphere();

      geometry->SetMaterial(mat, false);
      auto marker = this->scene->CreateVisual();
      marker->SetLocalPosition(point.x, point.y, point.z);
      marker->SetLocalScale(_msg.scale.x, _msg.scale.y, _msg.scale.z);
      marker->AddGeometry(geometry);

      visual->AddChild(marker);
    }
  }

  visual->SetLocalPose(msgToPose(_msg.pose));

  this->rootVisual->AddChild(visual);
}

////////////////////////////////////////////////////////////////////////////////
rendering::MaterialPtr MarkerManager::createMaterial(const std_msgs::msg::ColorRGBA & _color)
{
  auto mat = this->scene->CreateMaterial();
  mat->SetAmbient(_color.r, _color.g, _color.b, _color.a);
  mat->SetDiffuse(_color.r, _color.g, _color.b, _color.a);
  mat->SetEmissive(_color.r, _color.g, _color.b, _color.a);

  return mat;
}

////////////////////////////////////////////////////////////////////////////////
math::Pose3d MarkerManager::msgToPose(const geometry_msgs::msg::Pose & _pose)
{
  return math::Pose3d(
    _pose.position.x, _pose.position.y, _pose.position.z,
    _pose.orientation.w, _pose.orientation.x, _pose.orientation.y, _pose.orientation.z);
}

////////////////////////////////////////////////////////////////////////////////
void MarkerManager::insertOrUpdateVisual(unsigned int _id, rendering::VisualPtr _visual)
{
  auto it = visuals.find(_id);
  if (it != visuals.end()) {
    // Destroy previously created visual with same ID
    this->scene->DestroyVisual(it->second, true);
    it->second = _visual;
  } else {
    visuals.insert({_id, _visual});
  }
}

////////////////////////////////////////////////////////////////////////////////
void MarkerManager::deleteMarker(unsigned int _id)
{
  auto it = visuals.find(_id);
  if (it != visuals.end()) {
    this->scene->DestroyVisual(it->second, true);
    visuals.erase(_id);
  } else {
    RCLCPP_WARN(rclcpp::get_logger("MarkerManager"), "Marker with id %d not found", _id);
  }
}

////////////////////////////////////////////////////////////////////////////////
void MarkerManager::deleteAllMarkers()
{
  for (auto visual : visuals) {
    this->scene->DestroyVisual(visual.second, true);
  }
  visuals.clear();
}

}  // namespace plugins
}  // namespace rviz
}  // namespace ignition
