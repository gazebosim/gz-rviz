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

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

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
void MarkerManager::processMessage(const visualization_msgs::msg::Marker::SharedPtr _msg)
{
  switch (_msg->action) {
    case visualization_msgs::msg::Marker::ADD: {
        createMarker(_msg);
        break;
      }
    case visualization_msgs::msg::Marker::DELETE: {
        deleteMarker(_msg->id);
        break;
      }
    case visualization_msgs::msg::Marker::DELETEALL: {
        deleteAllMarkers();
        break;
      }
  }
}

////////////////////////////////////////////////////////////////////////////////
void MarkerManager::createMarker(const visualization_msgs::msg::Marker::SharedPtr _msg)
{
  switch (_msg->type) {
    case visualization_msgs::msg::Marker::ARROW: {
        auto visual = this->scene->CreateArrowVisual();
        insertOrUpdateVisual(_msg->id, visual);

        visual->SetMaterial(createMaterial(_msg->color));
        visual->SetLocalScale(_msg->scale.x, _msg->scale.y, _msg->scale.z);

        math::Pose3d pose(_msg->pose.position.x, _msg->pose.position.y, _msg->pose.position.z,
          _msg->pose.orientation.w, _msg->pose.orientation.x, _msg->pose.orientation.y,
          _msg->pose.orientation.z);
        visual->SetLocalPosition(pose.Pos());
        visual->SetLocalRotation(pose.Rot() * math::Quaterniond(0, 1.57, 0));

        this->rootVisual->AddChild(visual);
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
    // TODO(Sarathkrishnan Ramesh): Add support for following marker types
    case visualization_msgs::msg::Marker::CUBE_LIST: {
        break;
      }
    case visualization_msgs::msg::Marker::SPHERE_LIST: {
        break;
      }
    case visualization_msgs::msg::Marker::TEXT_VIEW_FACING: {
        break;
      }
    case visualization_msgs::msg::Marker::MESH_RESOURCE: {
        break;
      }
  }
}

////////////////////////////////////////////////////////////////////////////////
void MarkerManager::createBasicGeometry(
  const visualization_msgs::msg::Marker::SharedPtr _msg,
  rendering::MarkerType _geometryType)
{
  rendering::VisualPtr visual = this->scene->CreateVisual();
  insertOrUpdateVisual(_msg->id, visual);

  // Create marker
  auto marker = this->scene->CreateMarker();
  marker->SetType(_geometryType);

  // Set material
  marker->SetMaterial(createMaterial(_msg->color));

  // Add geometry and set scale
  visual->AddGeometry(marker);
  visual->SetLocalScale(_msg->scale.x, _msg->scale.y, _msg->scale.z);
  visual->SetLocalPose(
    math::Pose3d(
      _msg->pose.position.x, _msg->pose.position.y, _msg->pose.position.z,
      _msg->pose.orientation.w, _msg->pose.orientation.x, _msg->pose.orientation.y,
      _msg->pose.orientation.z));

  this->rootVisual->AddChild(visual);
}

////////////////////////////////////////////////////////////////////////////////
void MarkerManager::createListGeometry(
  const visualization_msgs::msg::Marker::SharedPtr _msg,
  rendering::MarkerType _geometryType)
{
  rendering::VisualPtr visual = this->scene->CreateVisual();
  insertOrUpdateVisual(_msg->id, visual);

  auto marker = this->scene->CreateMarker();
  marker->SetType(_geometryType);

  if (_msg->colors.size() == _msg->points.size()) {
    for (unsigned int i = 0; i < _msg->points.size(); ++i) {
      const auto & point = _msg->points[i];
      const auto color = math::Color(
        _msg->colors[i].r, _msg->colors[i].g, _msg->colors[i].b, _msg->colors[i].a);
      marker->AddPoint(point.x, point.y, point.z, color);
    }
  } else {
    if (_msg->colors.size() != 0) {
      RCLCPP_WARN(
        rclcpp::get_logger("MarkerManager"), "Marker color and point array size doesn't.");
    }
    const auto color = math::Color(_msg->color.r, _msg->color.g, _msg->color.b, _msg->color.a);
    for (const auto & point : _msg->points) {
      marker->AddPoint(point.x, point.y, point.z, color);
    }
  }

  // This material is not used anywhere but is required to set
  // point color in marker AddPoint method
  marker->SetMaterial(this->scene->Material("Default/TransGreen"));

  visual->AddGeometry(marker);
  visual->SetLocalPose(
    math::Pose3d(
      _msg->pose.position.x, _msg->pose.position.y, _msg->pose.position.z,
      _msg->pose.orientation.w, _msg->pose.orientation.x, _msg->pose.orientation.y,
      _msg->pose.orientation.z));

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