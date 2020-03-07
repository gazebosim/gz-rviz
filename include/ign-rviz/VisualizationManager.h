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

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/TFMessage.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

class VisualizationManager : public SceneManager {
 private:
  ros::Subscriber point_subscriber;
  ros::Subscriber pose_subscriber;
  ros::Subscriber orientation_subscriber;
  ros::Subscriber marker_subscriber;
  ros::Subscriber tf_subscriber;
  ros::Subscriber pointcloud_subscriber;
  ros::NodeHandle nh;
  AxisVisualPtr axis;
  MarkerPtr pcl_marker;

 public:
  VisualizationManager(int &argc, char** argv);
  void point_callback(const geometry_msgs::PointStampedConstPtr&);
  void pose_callback(const geometry_msgs::PoseStampedConstPtr&);
  void orientation_callback(const sensor_msgs::ImuConstPtr&);
  void marker_callback(const visualization_msgs::MarkerConstPtr&);
  void create_tf_visual(std::unordered_map<std::string, std::vector<geometry_msgs::TransformStamped>> &,
                        std::string &, MarkerPtr &, math::Vector3f);
  void tf_callback(const tf2_msgs::TFMessageConstPtr &);
  void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &);
  void run();
};

#endif //IGN_RVIZ_VISUALIZATIONMANAGER_H
