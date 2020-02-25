//
// Created by Sarathkrishnan Ramesh on 2/25/20.
//

#ifndef IGN_RVIZ_IGN_RVIZ_H
#define IGN_RVIZ_IGN_RVIZ_H

#if defined(__APPLE__)
#include <OpenGL/gl.h>
  #include <GLUT/glut.h>
#elif not defined(_WIN32)
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glut.h>
#endif

// General Headers
#include <iostream>
#include <vector>

// Ignition Rendering headers
#include <ignition/common/Console.hh>
#include <ignition/rendering.hh>
#include "ign-rviz/GlutWindow.hh"

using namespace ignition;
using namespace rendering;

class RVizScene {
 private:
  std::vector<CameraPtr> cameras;
  std::vector<std::string> engineNames;
  std::vector<NodePtr> nodes;

 public:
  RVizScene(int, char**);
  void build_scene(ScenePtr scene);
  CameraPtr create_camera(const std::string &);
  void run();

};

#endif //IGN_RVIZ_IGN_RVIZ_H
