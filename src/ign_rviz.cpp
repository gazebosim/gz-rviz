//
// Created by Sarathkrishnan Ramesh on 2/25/20.
//

#include "ign-rviz/ign_rviz.h"
#include <iostream>

/************************************************************************************/
RVizScene::RVizScene(int argc, char** argv) {
  glutInit(&argc, argv);
  common::Console::SetVerbosity(4);

  this->engineNames.emplace_back("ogre");

  for (const auto& engineName : engineNames) {
    try {
//      std::cout << "here\n";
      CameraPtr camera = create_camera(engineName);
      if (camera) {
        cameras.push_back(camera);

        ScenePtr ptr = camera->Scene();
        VisualPtr root = ptr->RootVisual();
      }
    }
    catch (...)
    {
      std::cerr << "Error starting up: " << engineName << std::endl;
    }
  }
}

/************************************************************************************/
void RVizScene::build_scene(ScenePtr scene)
{
  // Initialize scene
  scene->SetAmbientLight(0.3, 0.3, 0.3);
  VisualPtr root = scene->RootVisual();

  // Create directional light
  DirectionalLightPtr light0 = scene->CreateDirectionalLight();
  light0->SetDirection(-0.5, 0.5, -1);
  light0->SetDiffuseColor(0.8, 0.8, 0.8);
  light0->SetSpecularColor(0.5, 0.5, 0.5);
  root->AddChild(light0);

  // Create gray material
  MaterialPtr gray = scene->CreateMaterial();
  gray->SetAmbient(0.7, 0.7, 0.7);
  gray->SetDiffuse(0.7, 0.7, 0.7);
  gray->SetSpecular(0.7, 0.7, 0.7);

  // Create grid visual
  GridPtr grid_geometry = scene->CreateGrid();
  if (grid_geometry)
  {
    VisualPtr grid = scene->CreateVisual();
    grid_geometry->SetCellCount(10);
    grid_geometry->SetCellLength(1);
    grid_geometry->SetVerticalCellCount(0);
    grid->AddGeometry(grid_geometry);
    grid->SetLocalPosition(0, 0, 0.0);
    grid->SetMaterial(gray);
    root->AddChild(grid);
  }

  // Create axis visual
  AxisVisualPtr axis = scene->CreateAxisVisual();
  axis->SetLocalPosition(0, 0, 0);
  root->AddChild(axis);

  // Create camera
  CameraPtr camera = scene->CreateCamera("camera");
  camera->SetLocalPosition(-6.0, 6.0, 5.0);
  camera->SetLocalRotation(0.0, IGN_PI / 6, -IGN_PI / 4);
  camera->SetImageWidth(800);
  camera->SetImageHeight(600);
  camera->SetAntiAliasing(2);
  camera->SetAspectRatio(1.333);
  camera->SetHFOV(IGN_PI / 2);
  root->AddChild(camera);
}

/************************************************************************************/
CameraPtr RVizScene::create_camera(const std::string &_engineName)
{
  // Create and populate scene
  RenderEngine *engine = rendering::engine(_engineName);
  if (!engine) {
    std::cout << "Engine '" << _engineName << "' is not supported" << std::endl;
    return CameraPtr();
  }

  ScenePtr scene = engine->CreateScene("scene");
  build_scene(scene);

  // Return camera sensor
  SensorPtr sensor = scene->SensorByName("camera");
  return std::dynamic_pointer_cast<Camera>(sensor);
}

/************************************************************************************/
void RVizScene::run() {
  ::run(this->cameras, this->nodes);
}

/************************************************************************************/

int main(int argc, char** argv) {
  RVizScene rviz(argc, argv);
  rviz.run();
  return 0;
}