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

#ifndef IGNITION__RVIZ__PLUGINS__MARKERMANAGER_HPP_
#define IGNITION__RVIZ__PLUGINS__MARKERMANAGER_HPP_

#include <ignition/rendering.hh>

#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <unordered_map>

namespace ignition
{
namespace rviz
{
namespace plugins
{
class MarkerManager
{
public:
  MarkerManager();
  void insertOrUpdateVisual(unsigned int id, rendering::VisualPtr _visual);
  void processMessage(const visualization_msgs::msg::Marker::SharedPtr _msg);
  void createMarker(const visualization_msgs::msg::Marker::SharedPtr _msg);
  void createBasicGeometry(
    const visualization_msgs::msg::Marker::SharedPtr _msg, rendering::MarkerType _geometryType);
  rendering::MaterialPtr createMaterial(const std_msgs::msg::ColorRGBA & _color);
  void deleteMarker(unsigned int id);
  void deleteAllMarkers();

private:
  ignition::rendering::RenderEngine * engine;
  ignition::rendering::ScenePtr scene;
  ignition::rendering::VisualPtr rootVisual;
  std::unordered_map<unsigned int, rendering::VisualPtr> visuals;
};
}  // namespace plugins
}  // namespace rviz
}  // namespace ignition

#endif  // IGNITION__RVIZ__PLUGINS__MARKERMANAGER_HPP_
