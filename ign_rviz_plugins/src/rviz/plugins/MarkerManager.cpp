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
void MarkerManager::processMessage(const visualization_msgs::msg::Marker::SharedPtr _msg)
{
  switch (_msg->action) {
    case visualization_msgs::msg::Marker::ADD: {
        createMarker(_msg);
        break;
      }
    case visualization_msgs::msg::Marker::DELETE: {
        // TODO(Sarathkrishnan Ramesh): Delete marker
        break;
      }
    case visualization_msgs::msg::Marker::DELETEALL: {
        // TODO(Sarathkrishnan Ramesh): Delete all markers
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
        visual->SetLocalPose(math::Pose3d(0.0, 0.0, 0.0, 0.0, 1.57, 0.0));
        visual->SetLocalScale(_msg->scale.x, _msg->scale.y, _msg->scale.z);
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
    // TODO(Sarathkrishnan Ramesh): Add support for following marker types
    case visualization_msgs::msg::Marker::LINE_STRIP: {
        break;
      }
    case visualization_msgs::msg::Marker::LINE_LIST: {
        break;
      }
    case visualization_msgs::msg::Marker::CUBE_LIST: {
        break;
      }
    case visualization_msgs::msg::Marker::SPHERE_LIST: {
        break;
      }
    case visualization_msgs::msg::Marker::POINTS: {
        break;
      }
    case visualization_msgs::msg::Marker::TEXT_VIEW_FACING: {
        break;
      }
    case visualization_msgs::msg::Marker::MESH_RESOURCE: {
        break;
      }
    case visualization_msgs::msg::Marker::TRIANGLE_LIST: {
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
void MarkerManager::insertOrUpdateVisual(unsigned int id, rendering::VisualPtr _visual)
{
  auto it = visuals.find(id);
  if (it != visuals.end()) {
    // Destroy previously created visual with same ID
    this->scene->DestroyVisual(it->second, true);
    it->second = _visual;
  } else {
    visuals.insert({id, _visual});
  }
}

}  // namespace plugins
}  // namespace rviz
}  // namespace ignition
