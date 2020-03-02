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
  std::string marker_topic = "/visualization_marker";

  private_nh.getParam("/ign_rviz/point_topic", point_topic);
  private_nh.getParam("/ign_rviz/pose_topic", pose_topic);
  private_nh.getParam("/ign_rviz/imu_topic", imu_topic);
  private_nh.getParam("/ign_rviz/marker_topic", marker_topic);

  point_subscriber = nh.subscribe(point_topic, 1, &VisualizationManager::point_callback, this);
  pose_subscriber = nh.subscribe(pose_topic, 1, &VisualizationManager::pose_callback, this);
  orientation_subscriber = nh.subscribe(imu_topic, 100, &VisualizationManager::orientation_callback, this);
  marker_subscriber = nh.subscribe(marker_topic, 100, &VisualizationManager::marker_callback, this);

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

  // Create visual
  VisualPtr marker = scene->CreateVisual();
  marker->AddGeometry(get_geometry((int)IGN_GEOMETRY::BOX));
  marker->SetMaterial(color);
  marker->SetLocalPosition(x, y, z);
  marker->SetWorldScale(0.1);

  VisualPtr root = scene->RootVisual();
  root->AddChild(marker);
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
  ROS_DEBUG("[Orientation Received]");
}

void VisualizationManager::marker_callback(const visualization_msgs::MarkerConstPtr &msg) {
  ScenePtr scene = get_scene();
  MarkerPtr marker = scene->CreateMarker();

  bool add_marker = false;
  std::string unique_name = msg->ns + std::to_string(msg->id);
  VisualPtr vis = scene->VisualByName(unique_name);

  // TODO: Implement DELETEALL markers action and fix delete arrow bug

  // Delete marker
  if (msg->action == visualization_msgs::Marker::DELETE) {
    if(vis != nullptr)
      vis->RemoveGeometries();
    return;
  }

  // Add marker
  if (vis == nullptr) {
    if (msg->type == visualization_msgs::Marker::ARROW) {
      vis = scene->CreateArrowVisual(unique_name);
    }
    else
      vis = scene->CreateVisual(unique_name);
    add_marker = true;
  }
  else
    vis->RemoveGeometries();

  // TODO: Implement visualization for Triangle, Cube & Sphere List, Text Message and Mesh Resource marker type

  if (msg->type == visualization_msgs::Marker::LINE_LIST || msg->type == visualization_msgs::Marker::LINE_STRIP
      || msg->type == visualization_msgs::Marker::POINTS || msg->type == visualization_msgs::Marker::CUBE_LIST
      || msg->type == visualization_msgs::Marker::SPHERE_LIST) {

    for (const auto &point : msg->points)
      marker->AddPoint(point.x, point.y, point.z, math::Color::White);

    if (msg->type == visualization_msgs::Marker::LINE_STRIP)
      marker->SetType(MarkerType::MT_LINE_STRIP);
    else if (msg->type == visualization_msgs::Marker::LINE_LIST)
      marker->SetType(MarkerType::MT_LINE_LIST);
    else
      marker->SetType(MarkerType::MT_POINTS);

  } else if (msg->type == visualization_msgs::Marker::SPHERE) {
    marker->AddPoint(0, 0, 0, math::Color::White);
    marker->SetType(MarkerType::MT_SPHERE);
  } else if (msg->type == visualization_msgs::Marker::CUBE) {
    marker->AddPoint(0, 0, 0, math::Color::White);
    marker->SetType(MarkerType::MT_BOX);
  } else if (msg->type == visualization_msgs::Marker::CYLINDER) {
    marker->AddPoint(0, 0, 0, math::Color::White);
    marker->SetType(MarkerType::MT_CYLINDER);
  }

  MaterialPtr color = scene->CreateMaterial();
  if (msg->type == visualization_msgs::Marker::ARROW) {
    color->SetAmbient(1,0,0,1);
    color->SetDiffuse(1,0,0,1);
    tf2::Quaternion quat_orig, quat_new;
    tf2::convert(msg->pose.orientation, quat_orig);
    quat_new.setRPY(0, M_PI / 2, 0);
    quat_new = quat_orig * quat_new;
    quat_new.normalize();
    vis->SetLocalRotation(quat_new.w(), quat_new.x(), quat_new.y(), quat_new.z());
  } else {
    color->SetAmbient(msg->color.r, msg->color.g, msg->color.b, msg->color.a);
    color->SetDiffuse(msg->color.r, msg->color.g, msg->color.b, msg->color.a);
    vis->AddGeometry(marker);
    vis->SetLocalRotation(1, 0, 0, 0);
  }

  vis->SetLocalPosition(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  vis->SetMaterial(color);

  VisualPtr root = scene->RootVisual();

  if(add_marker)
    root->AddChild(vis);
}

void VisualizationManager::run() {
  ::run(cameras, nodes);
}
