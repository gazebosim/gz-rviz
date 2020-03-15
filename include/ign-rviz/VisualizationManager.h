//
// Created by sarath on 2/26/20.
//

#ifndef IGN_RVIZ_VISUALIZATIONMANAGER_H
#define IGN_RVIZ_VISUALIZATIONMANAGER_H

#include "ign-rviz/SceneManager.h"
#include "ign-rviz/GlutWindow.hh"

#if defined(__APPLE__)
#include <OpenGL/gl.h>
  #include <GLUT/glut.h>
#elif not defined(_WIN32)
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glut.h>
#endif

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <tf2_msgs/TFMessage.h>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/filter.h>
//#include <tf2_ros/transform_listener.h>

class VisualizationManager : public SceneManager {
 private:
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr orientation_subscriber;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr marker_subscriber;
//  rclcpp::Subscriber tf_subscriber;
//  rclcpp::Subscriber pointcloud_subscriber;
  rclcpp::Node::SharedPtr nh;
  AxisVisualPtr axis;
  MarkerPtr pcl_marker;

//  tf2_ros::Buffer tfBuffer;
//  tf2_ros::TransformListener tfListener;

 public:
  VisualizationManager(int &argc, char** argv, rclcpp::Node::SharedPtr);
  void point_callback(const geometry_msgs::msg::PointStamped::SharedPtr);
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr);
  void orientation_callback(const sensor_msgs::msg::Imu::SharedPtr);
  void marker_callback(const visualization_msgs::msg::Marker::SharedPtr);
//  void create_tf_visual(std::unordered_map<std::string, std::vector<geometry_msgs::TransformStamped>> &,
//                        std::string &, MarkerPtr &, math::Vector3f);
//  void tf_callback(const tf2_msgs::TFMessageConstPtr &);
//  void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &);
  void run();
};

#endif //IGN_RVIZ_VISUALIZATIONMANAGER_H
