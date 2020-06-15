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
namespace common
{
class FrameManager
{
public:
  /**
   * @brief Constructor for FrameManager
   * @param[in] node: ROS Node shared pointer
   */
  explicit FrameManager(rclcpp::Node::SharedPtr);

  /**
   * @brief Sets fixed frame for frame tranformations
   * @param[in] fixedFrame: Fixed frame
   */
  void setFixedFrame(std::string);

  /**
   * @brief Get frame pose (position and orientation)
   * @param[in] frame: Frame name
   * @param[out] pose: Frame pose
   * @return Pose validity (true if pose is valid, else false)
   */
  bool getFramePose(std::string &, ignition::math::Pose3d &);

  /**
   * @brief Get parent frame pose (position and orientation)
   * @param[in] frame: Child frame name
   * @param[out] pose: Parent frame pose
   * @return Pose validity (true if pose is valid, else false)
   */
  bool getParentPose(std::string & /*child*/, ignition::math::Pose3d &);

  /**
   * @brief Get available tf frames
   * @param[out] frames: List of available frames
   */
  void getFrames(std::vector<std::string> &);

  /**
   *  @brief Get fixed frame
   *  @return Fixed frame
   */
  std::string getFixedFrame();

protected:
  /**
   * @brief Callback function to received transform messages
   * @param[in] msg: Transform message
   */
  void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr);

private:
  rclcpp::Node::SharedPtr node;
  std::mutex tf_mutex_;
  std::string fixedFrame;
  std::shared_ptr<tf2_ros::Buffer> tfBuffer;
  std::shared_ptr<tf2_ros::TransformListener> tfListener;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscriber;
  std::unordered_map<std::string, ignition::math::Pose3d> tfTree;
  tf2::TimePoint timePoint;
};
}  // namespace common
}  // namespace rviz
}  // namespace ignition

#endif  // IGNITION__RVIZ__COMMON__FRAME_MANAGER_HPP_
