// Copyright (c) 2020 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef IGNITION__RVIZ__COMMON__FRAME_MANAGER_HPP_
#define IGNITION__RVIZ__COMMON__FRAME_MANAGER_HPP_

#include <ignition/math/Pose3.hh>

#include <QObject>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>


namespace ignition
{
namespace rviz
{
/**
 * @brief Namespace for all common tools
 */
namespace common
{
/**
 * @brief Manages and provides information about all the frames
 *
 * Subscribes to TF data to determine the location and orientation of frames
 */
class FrameManager : public QObject
{
  Q_OBJECT

public:
  /**
   * @brief Constructor for FrameManager
   * @param[in] _node: ROS Node shared pointer
   */
  explicit FrameManager(rclcpp::Node::SharedPtr _node);

  /**
   * @brief Sets fixed frame for frame tranformations
   * @param[in] _fixedFrame: Fixed frame
   */
  void setFixedFrame(const std::string & _fixedFrame);

  /**
   * @brief Get frame pose (position and orientation)
   * @param[in] _frame: Frame name
   * @param[out] _pose: Frame pose
   * @return Pose validity (true if pose is valid, else false)
   */
  bool getFramePose(const std::string & _frame, ignition::math::Pose3d & _pose);

  /**
   * @brief Get parent frame pose (position and orientation)
   * @param[in] _child: Child frame name
   * @param[out] _pose: Parent frame pose
   * @return Pose validity (true if pose is valid, else false)
   */
  bool getParentPose(const std::string & _child, ignition::math::Pose3d & _pose);

  /**
   * @brief Get available tf frames
   * @param[out] _frames: List of available frames
   */
  void getFrames(std::vector<std::string> & _frames);

  /**
   *  @brief Get fixed frame
   *  @return Fixed frame
   */
  std::string getFixedFrame();

protected:
  /**
   * @brief Callback function to received transform messages
   * @param[in] _msg: Transform message
   */
  void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr _msg);

private:
  rclcpp::Node::SharedPtr node;
  std::mutex tf_mutex_;
  std::string fixedFrame;
  std::shared_ptr<tf2_ros::Buffer> tfBuffer;
  std::shared_ptr<tf2_ros::TransformListener> tfListener;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscriber;
  std::unordered_map<std::string, ignition::math::Pose3d> tfTree;
  tf2::TimePoint timePoint;
  unsigned int frameCount;
};
}  // namespace common
}  // namespace rviz
}  // namespace ignition

#endif  // IGNITION__RVIZ__COMMON__FRAME_MANAGER_HPP_
