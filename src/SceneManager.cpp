//
// Created by Sarathkrishnan Ramesh on 2/26/20.
//

#include "ign-rviz/SceneManager.hpp"

SceneManager::SceneManager()
{
  engine_names.emplace_back("ogre");

  for (const auto & engine_name : engine_names) {
    try {
      CameraPtr camera = create_camera(engine_name);
      if (camera) {
        cameras.push_back(camera);

        ScenePtr ptr = camera->Scene();
        VisualPtr root = ptr->RootVisual();
      }
    } catch (...) {
      std::cerr << "Error starting up: " << engine_name << std::endl;
    }
  }
}

CameraPtr SceneManager::create_camera(const std::string & engine_name)
{
  // Create and populate scene
  engine = rendering::engine(engine_name);
  if (!engine) {
    std::cout << "Engine '" << engine_name << "' is not supported" << std::endl;
    return CameraPtr();
  }

  // Create scene
  ScenePtr scene = engine->CreateScene("scene");
  build_scene(scene);

  // Return camera sensor
  SensorPtr sensor = scene->SensorByName("camera");
  return std::dynamic_pointer_cast<Camera>(sensor);
}

void SceneManager::build_scene(ScenePtr scene)
{
  // Initialize scene
  scene->SetAmbientLight(0.4, 0.4, 0.4);
  scene->SetBackgroundColor(0.07, 0.07, 0.07);
  VisualPtr root = scene->RootVisual();

  _sphere_geometry = scene->CreateSphere();
  _plane_geometry = scene->CreatePlane();
  _box_geometry = scene->CreateBox();

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
  if (grid_geometry) {
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
  axis->SetLocalScale(0.5);
  root->AddChild(axis);

  // Create camera
  CameraPtr camera = scene->CreateCamera("camera");
  camera->SetLocalPosition(-6.0, 6.0, 4);
  camera->SetLocalRotation(0.0, IGN_PI / 5, -IGN_PI / 4);
  camera->SetImageWidth(800);
  camera->SetImageHeight(600);
  camera->SetAntiAliasing(2);
  camera->SetAspectRatio(1.333);
  camera->SetHFOV(IGN_PI / 2);
  root->AddChild(camera);
}

GeometryPtr SceneManager::get_geometry(const int & geometry)
{
  ScenePtr scene = get_scene();
  switch (geometry) {
    case (int) IGN_GEOMETRY::PLANE: {
        return scene->CreatePlane();
      } case (int) IGN_GEOMETRY::BOX: {
        return scene->CreateBox();
      } case (int) IGN_GEOMETRY::CONE: {
        return scene->CreateCone();
      } case (int) IGN_GEOMETRY::CYLINDER: {
        return scene->CreateCylinder();
      } case (int) IGN_GEOMETRY::SPHERE: {
        return scene->CreateSphere();
      } default: return nullptr;
  }
}

ScenePtr SceneManager::get_scene()
{
  return this->engine->SceneByName("scene");
}
