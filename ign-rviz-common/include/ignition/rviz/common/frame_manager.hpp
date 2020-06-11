// Copyright (c) 2020 Sarathkrishnan Ramesh
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
  explicit FrameManager(rclcpp::Node::SharedPtr);

  /*
   * @brief Sets fixed frame for frame tranformations
   * @param Fixed frame
   */
  void setFixedFrame(std::string);

  bool getFramePose(std::string &, ignition::math::Pose3d &);

  bool getParentPose(std::string & /*child*/, ignition::math::Pose3d &);

  void getFrames(std::vector<std::string> &);

  std::string getFixedFrame();

protected:
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
