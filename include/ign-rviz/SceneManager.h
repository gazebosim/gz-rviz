//
// Created by sarath on 2/26/20.
//

#ifndef IGN_RVIZ_SCENEMANAGER_H
#define IGN_RVIZ_SCENEMANAGER_H

#include <vector>
#include <ignition/rendering.hh>

using namespace ignition;
using namespace rendering;

enum class IGN_GEOMETRY {
  PLANE,
  BOX,
  CONE,
  CYLINDER,
  SPHERE
};

class SceneManager {
 protected:
  std::vector<CameraPtr> cameras;
  std::vector<std::string> engine_names;
  std::vector<NodePtr> nodes;
 public:
  SceneManager();
  void build_scene(ScenePtr);
  CameraPtr create_camera(const std::string &);
  ScenePtr get_scene();
  GeometryPtr get_geometry(const int &geometry);
};

#endif //IGN_RVIZ_SCENEMANAGER_H
