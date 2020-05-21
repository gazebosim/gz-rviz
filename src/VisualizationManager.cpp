//
// Created by Sarathkrishnan Ramesh on 2/26/20.
//

#include "ign-rviz/VisualizationManager.hpp"

#include <utility>
#include <mutex>

std::mutex tf_lock;

VisualizationManager::VisualizationManager(int & argc, char ** argv, rclcpp::Node::SharedPtr node)
: nh(std::move(node))
{
  glutInit(&argc, argv);
  common::Console::SetVerbosity(4);

  tfBuffer = std::make_shared<tf2_ros::Buffer>(nh->get_clock());
  tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

  nh->declare_parameter("point_topic", "/point");
  nh->declare_parameter("pose_topic", "/pose");
  nh->declare_parameter("imu_topic", "/imu");
  nh->declare_parameter("marker_topic", "/visualization_marker");
  nh->declare_parameter("tf_topic", "/tf_static");
  nh->declare_parameter("pointcloud_topic", "/cloud_in");
  nh->declare_parameter("fixed_frame", "base_link");

  std::string point_topic = nh->get_parameter("point_topic").as_string();
  std::string pose_topic = nh->get_parameter("pose_topic").as_string();
  std::string imu_topic = nh->get_parameter("imu_topic").as_string();
  std::string marker_topic = nh->get_parameter("marker_topic").as_string();
  std::string tf_topic = nh->get_parameter("tf_topic").as_string();
  std::string pointcloud_topic = nh->get_parameter("pointcloud_topic").as_string();
  this->fixed_frame = nh->get_parameter("fixed_frame").as_string();

  point_subscriber = nh->create_subscription<geometry_msgs::msg::PointStamped>(point_topic, 10,
      std::bind(&VisualizationManager::point_callback, this, std::placeholders::_1));
  pose_subscriber = nh->create_subscription<geometry_msgs::msg::PoseStamped>(pose_topic, 10,
      std::bind(&VisualizationManager::pose_callback, this, std::placeholders::_1));
  orientation_subscriber = nh->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10,
      std::bind(&VisualizationManager::orientation_callback, this, std::placeholders::_1));
  marker_subscriber = nh->create_subscription<visualization_msgs::msg::Marker>(marker_topic, 10,
      std::bind(&VisualizationManager::marker_callback, this, std::placeholders::_1));
  tf_subscriber = nh->create_subscription<tf2_msgs::msg::TFMessage>(tf_topic, 10,
      std::bind(&VisualizationManager::tf_callback, this, std::placeholders::_1));
  pointcloud_subscriber = nh->create_subscription<sensor_msgs::msg::PointCloud2>(pointcloud_topic,
      10,
      std::bind(&VisualizationManager::cloud_callback, this, std::placeholders::_1));

  ScenePtr scene = get_scene();
  VisualPtr root = scene->RootVisual();
  axis = scene->CreateAxisVisual("axis");
  axis->SetLocalPosition(0, 0, 0);
  root->AddChild(axis);

  MaterialPtr color = scene->CreateMaterial("red");
  color->SetAmbient(1, 0, 0);
  color->SetDiffuse(1, 0, 0);

  pcl_marker = scene->CreateMarker();
  pcl_marker->SetType(MarkerType::MT_POINTS);

  VisualPtr pcl_visual = scene->CreateVisual("pcl");
  pcl_visual->AddGeometry(pcl_marker);
  root->AddChild(pcl_visual);

  VisualPtr tf_visual = scene->CreateVisual("tf_visual");
  root->AddChild(tf_visual);
}

void VisualizationManager::point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  float x = msg->point.x;
  float y = msg->point.y;
  float z = msg->point.z;

  RCLCPP_DEBUG(nh->get_logger(), "[ Point Received ]\tX:%f, Y:%f,  Z:%f", x, y, z);

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

void VisualizationManager::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  ScenePtr scene = get_scene();
  VisualPtr root = scene->RootVisual();

  RCLCPP_INFO(nh->get_logger(), "[ Pose Received ]");

  ArrowVisualPtr pose_arrow = scene->CreateArrowVisual();

  MaterialPtr color = scene->CreateMaterial();
  color->SetAmbient(1, 0, 0);
  color->SetDiffuse(1, 0, 0);

  math::Quaternion quat_orig(msg->pose.orientation.w, msg->pose.orientation.x,
    msg->pose.orientation.y, msg->pose.orientation.z);
  math::Quaternion quat_new(math::Vector3d(0, M_PI / 2, 0));
  quat_new = quat_orig * quat_new;
  quat_new.Normalize();

  pose_arrow->SetLocalPosition(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  pose_arrow->SetMaterial(color);

  pose_arrow->SetLocalRotation(quat_new.W(), quat_new.X(), quat_new.Y(), quat_new.Z());

  root->AddChild(pose_arrow);
}

void VisualizationManager::orientation_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  axis->SetWorldScale(2);
  axis->SetLocalRotation(msg->orientation.w, msg->orientation.x, msg->orientation.y,
    msg->orientation.z);
  RCLCPP_INFO(nh->get_logger(), "[Orientation Received]");
}

void VisualizationManager::marker_callback(const visualization_msgs::msg::Marker::SharedPtr msg)
{
  ScenePtr scene = get_scene();
  MarkerPtr marker = scene->CreateMarker();

  bool add_marker = false;
  std::string unique_name = msg->ns + std::to_string(msg->id);
  VisualPtr vis = scene->VisualByName(unique_name);

  // TODO: Implement DELETEALL markers action and fix delete arrow bug

  // Delete marker
  if (msg->action == visualization_msgs::msg::Marker::DELETE) {
    if (vis != nullptr) {
      vis->RemoveGeometries();
    }
    return;
  }

  // Add marker
  if (vis == nullptr) {
    if (msg->type == visualization_msgs::msg::Marker::ARROW) {
      vis = scene->CreateArrowVisual(unique_name);
    } else {
      vis = scene->CreateVisual(unique_name);
    }
    add_marker = true;
  } else {
    vis->RemoveGeometries();
  }

  // TODO: Implement visualization for Triangle, Cube & Sphere List, Text Message and Mesh Resource marker type

  if (msg->type == visualization_msgs::msg::Marker::LINE_LIST ||
    msg->type == visualization_msgs::msg::Marker::LINE_STRIP ||
    msg->type == visualization_msgs::msg::Marker::POINTS ||
    msg->type == visualization_msgs::msg::Marker::CUBE_LIST ||
    msg->type == visualization_msgs::msg::Marker::SPHERE_LIST)
  {

    for (const auto & point : msg->points) {
      marker->AddPoint(point.x, point.y, point.z, math::Color::White);
    }

    if (msg->type == visualization_msgs::msg::Marker::LINE_STRIP) {
      marker->SetType(MarkerType::MT_LINE_STRIP);
    } else if (msg->type == visualization_msgs::msg::Marker::LINE_LIST) {
      marker->SetType(MarkerType::MT_LINE_LIST);
    } else {
      marker->SetType(MarkerType::MT_POINTS);
    }

  } else if (msg->type == visualization_msgs::msg::Marker::SPHERE) {
    marker->AddPoint(0, 0, 0, math::Color::White);
    marker->SetType(MarkerType::MT_SPHERE);
  } else if (msg->type == visualization_msgs::msg::Marker::CUBE) {
    marker->AddPoint(0, 0, 0, math::Color::White);
    marker->SetType(MarkerType::MT_BOX);
  } else if (msg->type == visualization_msgs::msg::Marker::CYLINDER) {
    marker->AddPoint(0, 0, 0, math::Color::White);
    marker->SetType(MarkerType::MT_CYLINDER);
  }

  MaterialPtr color = scene->CreateMaterial();
  if (msg->type == visualization_msgs::msg::Marker::ARROW) {
    color->SetAmbient(1, 0, 0, 1);
    color->SetDiffuse(1, 0, 0, 1);

    math::Quaternion quat_orig(msg->pose.orientation.w, msg->pose.orientation.x,
      msg->pose.orientation.y, msg->pose.orientation.z);
    math::Quaternion quat_new(math::Vector3d(0, M_PI / 2, 0));
    quat_new = quat_orig * quat_new;
    quat_new.Normalize();
    vis->SetLocalRotation(quat_new.W(), quat_new.X(), quat_new.Y(), quat_new.Z());
  } else {
    color->SetAmbient(msg->color.r, msg->color.g, msg->color.b, msg->color.a);
    color->SetDiffuse(msg->color.r, msg->color.g, msg->color.b, msg->color.a);
    vis->AddGeometry(marker);
    vis->SetLocalRotation(1, 0, 0, 0);
  }

  vis->SetLocalPosition(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  vis->SetMaterial(color);

  VisualPtr root = scene->RootVisual();

  if (add_marker) {
    root->AddChild(vis);
  }
}

tf2::Quaternion get_rotation_to(const math::Vector3d & source, const math::Vector3d & destination)
{
  const math::Vector3d & fallbackAxis = math::Vector3d::Zero;
  tf2::Quaternion q;

  math::Vector3d v0 = source;
  math::Vector3d v1 = destination;
  v0.Normalize();
  v1.Normalize();

  double d = v0.Dot(v1);

  if (d >= 1.0f) {
    return q;
  }

  if (d < (1e-6f - 1.0f)) {
    if (fallbackAxis != math::Vector3d::Zero) {
      tf2::Vector3 temp(fallbackAxis.X(), fallbackAxis.Y(), fallbackAxis.Z());
      q.setRotation(temp, M_PI);
    } else {
      math::Vector3d axis = math::Vector3d::UnitX.Cross(source);
      if (axis.Length() == 0) {
        axis = math::Vector3d::UnitY.Cross(source);
      }
      axis.Normalize();
      tf2::Vector3 temp(axis.X(), axis.Y(), axis.Z());
      q.setRotation(temp, M_PI);
    }
  } else {
    double s = sqrt((1 + d) * 2);
    double inverse = 1 / s;

    math::Vector3d c = v0.Cross(v1);

    q.setX(c.X() * inverse);
    q.setY(c.Y() * inverse);
    q.setZ(c.Z() * inverse);
    q.setW(s * 0.5f);
    q.normalize();
  }
  return q;
}

void VisualizationManager::tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(tf_lock);

  // Get availaable TF Frames
  std::vector<std::string> frame_ids;
  tfBuffer->_getFrameStrings(frame_ids);

  std::cout << "Fixed frame: " << fixed_frame << std::endl;

  // Generate Timpoint from tf header
  builtin_interfaces::msg::Time time_stamp = msg->transforms[0].header.stamp;
  tf2::TimePoint time_point = tf2::TimePoint(std::chrono::seconds(
        time_stamp.sec) + std::chrono::nanoseconds(time_stamp.nanosec));


  // Update map of latest tf pose (given fixed frame)
  for (const auto fr : frame_ids) {
    try {
      geometry_msgs::msg::TransformStamped tff = tfBuffer->lookupTransform(fixed_frame, time_point,
          fr, time_point,
          fixed_frame);
      tree_tf[tff.child_frame_id] = ignition::math::Pose3d(tff.transform.translation.x,
          tff.transform.translation.y,
          tff.transform.translation.z,
          tff.transform.rotation.w,
          tff.transform.rotation.x,
          tff.transform.rotation.y,
          tff.transform.rotation.z);
    } catch (...) {
      // std::cout << "[Transform lookup error] Using old tf" << std::endl;
    }
  }

  ScenePtr scene = get_scene();
  VisualPtr tf_visual = scene->VisualByName("tf_visual");

  for (const auto frame : tree_tf) {
    std::cout << fixed_frame << " -> " << frame.first << " " << frame.second.Pos().X() << " " <<
      frame.second.Pos().Y() << " " << frame.second.Pos().Z() << std::endl;

    NodePtr axis = tf_visual->ChildByName("axis_" + frame.first);
    if (axis == nullptr) {
      axis = scene->CreateAxisVisual("axis_" + frame.first);
      axis->SetLocalPose(frame.second);
      tf_visual->AddChild(axis);
      std::cout << "Created new axis" << std::endl;
    } else {
      axis->SetLocalPose(frame.second);
    }
  }

  tf_visual->PreRender();

  std::cout << "################" << std::endl;

  // Naive Delay
  for (int i = 0; i < 10000; i++) {
    for (int j = 0; j < 2500; j++) {
    }
  }
}

void VisualizationManager::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZ> laser_cloud_in, laser_cloud_out;
  pcl::fromROSMsg(*msg, laser_cloud_in);

  std::vector<int> index;
  pcl::removeNaNFromPointCloud(laser_cloud_in, laser_cloud_out, index);

  pcl_marker->ClearPoints();

  for (int i = 0; i < static_cast<int>(laser_cloud_out.size()); ++i) {
    pcl_marker->AddPoint(laser_cloud_out.points[i].x, laser_cloud_out.points[i].y,
      laser_cloud_out.points[i].z, math::Color::Red);
  }

  ScenePtr scene = get_scene();
  VisualPtr pcl_visual = scene->VisualByName("pcl");
  pcl_visual->SetGeometryMaterial(scene->Material("red"));

  try {
    geometry_msgs::msg::TransformStamped transformStamped = tfBuffer->lookupTransform(
      "base_link", msg->header.frame_id.c_str(), msg->header.stamp);

    pcl_visual->SetLocalPosition(transformStamped.transform.translation.x,
      transformStamped.transform.translation.y,
      transformStamped.transform.translation.z);

    pcl_visual->SetLocalRotation(transformStamped.transform.rotation.w,
      transformStamped.transform.rotation.x,
      transformStamped.transform.rotation.y,
      transformStamped.transform.rotation.z);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(nh->get_logger(), "%s", ex.what());
  }
}

void VisualizationManager::run()
{
  ::run(cameras, nodes);
}
