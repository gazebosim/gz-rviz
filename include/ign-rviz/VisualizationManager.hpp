//
// Created by sarath on 2/26/20.
//

#ifndef IGN_RVIZ_VISUALIZATIONMANAGER_H
#define IGN_RVIZ_VISUALIZATIONMANAGER_H

#include "ign-rviz/SceneManager.hpp"
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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <tf2_ros/transform_listener.h>

class VisualizationManager : public SceneManager
{
private:
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr orientation_subscriber;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr marker_subscriber;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscriber;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber;
  rclcpp::Node::SharedPtr nh;
  AxisVisualPtr axis;
  MarkerPtr pcl_marker;

  std::string fixed_frame;
  std::map<std::string, ignition::math::Pose3d> tree_tf;

  std::shared_ptr<tf2_ros::Buffer> tfBuffer;
  std::shared_ptr<tf2_ros::TransformListener> tfListener;

public:
  /**
   * Initializes the Visualization Manager responsible for visualization
   * of ROS2 messages
   * Initializes the rendering window and Transform Listener.
   * Subscribers initialized using the topic names specified as parameters in the launch file
   *
   * \param program arguments along with count, shared pointer of ROS2 Node
   * \throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   */
  VisualizationManager(int & argc, char ** argv, rclcpp::Node::SharedPtr);

  /**
   * Callback function for visualization of geometry_msgs::msg::PointStamped msgs
   * PointStamped messages are visualized using cyan boxes
   *
   * \param constant shared pointer of type geometry_msgs::msg::PointStamped
   * \throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   */
  void point_callback(const geometry_msgs::msg::PointStamped::SharedPtr);

  /**
   * Callback function for visualization of geometry_msgs::msg::PoseStamped msgs
   * PointStamped messages are visualized using Arrows
   *
   * \param constant shared pointer of type geometry_msgs::msg::PoseStamped
   * \throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   */
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr);

  /**
   * Callback function for visualization of sensor_msgs::msg::Imu msgs
   * Imu data is visualized through orientation of axis
   *
   * \param constant shared pointer of type sensor_msgs::msg::Imu
   * \throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   */
  void orientation_callback(const sensor_msgs::msg::Imu::SharedPtr);

  /**
   * Callback function for visualization of visualization_msgs::msg::Marker msgs
   * Marker data visualization currently supports Sphere, Cube, Cylinder, Arrow, Line Strip,
   * Line List and Points with actions add, delete and modify.
   *
   * \param constant shared pointer of type sensor_msgs::msg::Imu
   * \throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   */
  void marker_callback(const visualization_msgs::msg::Marker::SharedPtr);

  /**
   * Helper function to that generates tf_visual geometry using tf_tree
   *
   * \params tf_tree, base_frame_id, pointer to marker visual, pose of current frame
   * \throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   */
  void create_tf_visual(
    std::unordered_map<std::string, std::vector<geometry_msgs::msg::TransformStamped>> &,
    std::string &, MarkerPtr &, math::Vector3f);

  /**
   * Callback function for visualization of tf2_msgs::msg::TFMessage msgs
   * Visualizes static transforms from tf_tree. Line describes links, cones point
   * to the parent link, axis describe the orientation of line and text
   * display link name.
   *
   * \param constant shared pointer of type sensor_msgs::msg::Imu
   * \throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   */
  void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr);

  /**
   * Callback function for visualization of sensor_msgs::msg::PointCloud2 msgs
   * PointCloud2 data is visualized using Marker with geometry as point type
   * PointCloud2 data is filtered by removing NaN points before visualization
   *
   * \param constant shared pointer of type sensor_msgs::msg::PointCloud2
   * \throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   */
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr);

  /**
   * Initializes keyboard and mouse callbacks.
   * Renders camera data and displays on window.
   */
  void run();
};

#endif //IGN_RVIZ_VISUALIZATIONMANAGER_H
