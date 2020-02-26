//
// Created by Sarathkrishnan Ramesh on 2/26/20.
//

#include "ign-rviz/VisualizationManager.h"

VisualizationManager::VisualizationManager(int &argc, char **argv) {
  glutInit(&argc, argv);
  common::Console::SetVerbosity(4);

  ros::NodeHandle private_nh("~");

  std::string point_topic = "/point";
  std::string pose_topic = "/pose";
  std::string imu_topic = "/imu";

  private_nh.getParam("/ign_rviz/point_topic", point_topic);
  private_nh.getParam("/ign_rviz/pose_topic", pose_topic);
  private_nh.getParam("/ign_rviz/imu", imu_topic);

  point_subscriber = nh.subscribe(point_topic, 1, &VisualizationManager::point_callback, this);
  pose_subscriber = nh.subscribe(pose_topic, 1, &VisualizationManager::pose_callback, this);
  orientation_subscriber = nh.subscribe(imu_topic, 100, &VisualizationManager::orientation_callback, this);


  ScenePtr scene = get_scene();
  VisualPtr root = scene->RootVisual();
  axis = scene->CreateAxisVisual("axis");
  axis->SetLocalPosition(0, 0, 0);
  root->AddChild(axis);
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
  ScenePtr scene = get_scene();
  VisualPtr root = scene->RootVisual();

  ROS_DEBUG("[ Pose Received ]");

  ArrowVisualPtr pose_arrow = scene->CreateArrowVisual();

  MaterialPtr color = scene->CreateMaterial();
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

void VisualizationManager::orientation_callback(const sensor_msgs::ImuConstPtr &msg) {
  axis->SetWorldScale(2);
  axis->SetLocalRotation(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
}

void VisualizationManager::run() {
  ::run(cameras, nodes);
}
