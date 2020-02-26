//
// Created by Sarathkrishnan Ramesh on 2/26/20.
//

#include "ign-rviz/VisualizationManager.h"

VisualizationManager::VisualizationManager(int &argc, char **argv) {
  glutInit(&argc, argv);
  common::Console::SetVerbosity(4);
  point_subscriber = nh.subscribe("point", 1, &VisualizationManager::point_callback, this);
  pose_subscriber = nh.subscribe("pose", 1, &VisualizationManager::pose_callback, this);
}

void VisualizationManager::point_callback(const geometry_msgs::PointStampedConstPtr& msg) {
  float x = msg->point.x;
  float y = msg->point.y;
  float z = msg->point.z;

  ROS_DEBUG("[ Point Received ]\tX:%f, Y:%f,  Z:%f", x, y, z);

  ScenePtr scene = get_scene();

  // Create material
  MaterialPtr color = scene->CreateMaterial();
  color->SetAmbient(0, 1, 1);
  color->SetDiffuse(0, 1, 1);
  color->SetSpecular(0.5, 0.5, 0.5);
  color->SetShininess(50);
  color->SetReflectivity(0);

  // Create box visual
  VisualPtr box = scene->CreateVisual();
  box->AddGeometry(get_geometry((int) IGN_GEOMETRY::CYLINDER));
  box->SetWorldScale(0.5);
  box->SetLocalPosition(x, y, z);
  box->SetMaterial(color);

  VisualPtr root = scene->RootVisual();
  root->AddChild(box);
}

void VisualizationManager::pose_callback(const geometry_msgs::PoseStampedConstPtr &msg) {
  ScenePtr ptr = get_scene();
  VisualPtr root = ptr->RootVisual();

  ROS_DEBUG("[ Pose Received ]");

  ArrowVisualPtr pose_arrow = ptr->CreateArrowVisual();

  MaterialPtr color = ptr->CreateMaterial();
  color->SetAmbient(1, 0, 0);
  color->SetDiffuse(1, 0, 0);

  tf2::Quaternion quat_orig, quat_new;
  tf2::convert(msg->pose.orientation, quat_orig);
  quat_new.setRPY(0, M_PI / 2, 0);
  quat_new = quat_orig * quat_new;
  quat_new.normalize();

  pose_arrow->SetLocalPosition(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  pose_arrow->SetMaterial(color);

  pose_arrow->SetLocalRotation(quat_new.w(), quat_new.x(), quat_new.y(), quat_new.z());

  root->AddChild(pose_arrow);
}

void VisualizationManager::run() {
  ::run(cameras, nodes);
}
