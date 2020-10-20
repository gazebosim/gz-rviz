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

#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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
  // Constructor
  MarkerManager();

  // Destructor
  ~MarkerManager();

  /**
   * @brief Insert or Update a new marker visual with same ID
   * @param[in] _id Marker ID
   * @param[in] _visual Marker visual
   */
  void insertOrUpdateVisual(unsigned int _id, rendering::VisualPtr _visual);

  /**
   * @brief Processes message to handle Add/Modify, Delete and Delete All marker actions
   * @param[in] _msg Marker message
   */
  void processMessage(const visualization_msgs::msg::Marker & _msg);

  /**
   * @brief Processes message to handle Add/Modify, Delete and Delete All marker actions
   * @param[in] _msg MarkerArray message
   */
  void processMessage(const visualization_msgs::msg::MarkerArray & _msg);

  /**
   * @brief Creates marker visual using message
   * @param[in] _msg Marker message
   */
  void createMarker(const visualization_msgs::msg::Marker & _msg);

  /**
   * @brief Creates basic marker geometry
   *
   * Handles the following geometry types: Cube, Sphere, Cyinder
   *
   * @param[in] _msg Marker message
   * @param[in] _geometryType Marker geometry type
   */
  void createBasicGeometry(
    const visualization_msgs::msg::Marker & _msg, rendering::MarkerType _geometryType);

  /**
   * @brief Creates marker list geometry
   *
   * Create marker geometry using points array availabe in message.
   * Handles the following geometry types:
   * Line List, Line Strip, Points, Triangle List
   *
   * @param[in] _msg Marker message
   * @param[in] _geometryType Marker geometry type
   */
  void createListGeometry(
    const visualization_msgs::msg::Marker & _msg, rendering::MarkerType _geometryType);

  /**
   * @brief Create arrow marker
   * @param[in] _msg Marker message
   */
  void createArrowMarker(const visualization_msgs::msg::Marker & _msg);

  /**
   * @brief Create a text marker
   * @param[in] _msg Marker message
   */
  void createTextMarker(const visualization_msgs::msg::Marker & _msg);

  /**
   * @brief Create mesh marker
   * @param[in] _msg Marker message
   */
  void createMeshMarker(const visualization_msgs::msg::Marker & _msg);

  /**
   * @brief Create list visual
   *
   * Create marker geometry using points array availabe in message.
   * Handles the following geometry types:
   * Cube List and Sphere List
   *
   * @param[in] _msg Marker message
   */
  void createListVisual(const visualization_msgs::msg::Marker & _msg);

  /**
   * @brief Create a new material from message
   * @param[in] _color Color message
   * @return Material Material created using color message
   */
  rendering::MaterialPtr createMaterial(const std_msgs::msg::ColorRGBA & _color);

  /**
   * @brief Convert pose message
   * @param[in] _pose Pose message
   * @return Pose Converted pose
   */
  math::Pose3d msgToPose(const geometry_msgs::msg::Pose & _pose);

  /**
   * @brief Delete a specific marker from scene
   * @param[in] _id Marker ID
   */
  void deleteMarker(unsigned int _id);

  /**
   * @brief Delete all the markers from scene
   */
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
