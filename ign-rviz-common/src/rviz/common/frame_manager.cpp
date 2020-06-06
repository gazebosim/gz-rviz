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

#include "ignition/rviz/common/frame_manager.hpp"

#include <string>
#include <utility>
#include <memory>
#include <vector>

namespace ignition
{
namespace rviz
{
namespace common
{
FrameManager::FrameManager(rclcpp::Node::SharedPtr node)
{
  this->node = std::move(node);

  tfBuffer = std::make_shared<tf2_ros::Buffer>(this->node->get_clock());
  tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

  this->subscriber = this->node->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", 10,
    std::bind(&FrameManager::tf_callback, this, std::placeholders::_1));
}

void FrameManager::setFixedFrame(std::string fixedFrame)
{
  this->fixedFrame = fixedFrame;
}

void FrameManager::getFrames(std::vector<std::string> & frames)
{
  tfBuffer->_getFrameStrings(frames);
}

void FrameManager::tf_callback(tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  std::lock_guard<std::mutex>(this->tf_mutex_);

  if (this->fixedFrame.empty()) {
    RCLCPP_ERROR(this->node->get_logger(), "No frame id specified");
    return;
  }

  std::vector<std::string> frame_ids;
  tfBuffer->_getFrameStrings(frame_ids);

  builtin_interfaces::msg::Time timeStamp = msg->transforms[0].header.stamp;
  tf2::TimePoint timePoint = tf2::TimePoint(
    std::chrono::seconds(
      timeStamp.sec) + std::chrono::nanoseconds(timeStamp.nanosec));

  for (const auto frame : frame_ids) {
    try {
      geometry_msgs::msg::TransformStamped tf = tfBuffer->lookupTransform(
        fixedFrame, timePoint,
        frame, timePoint,
        fixedFrame, tf2::Duration(5000));

      tfTree[tf.child_frame_id] = ignition::math::Pose3d(
        tf.transform.translation.x,
        tf.transform.translation.y,
        tf.transform.translation.z,
        tf.transform.rotation.w,
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z);
    } catch (tf2::LookupException e) {
      RCLCPP_ERROR(this->node->get_logger(), e.what());
    } catch (tf2::ConnectivityException e) {
      RCLCPP_ERROR(this->node->get_logger(), e.what());
    } catch (tf2::ExtrapolationException e) {
      RCLCPP_ERROR(this->node->get_logger(), e.what());
    } catch (tf2::InvalidArgumentException e) {
      RCLCPP_ERROR(this->node->get_logger(), e.what());
    }
  }
}

bool FrameManager::getFramePose(std::string & frame, ignition::math::Pose3d & pose)
{
  std::lock_guard<std::mutex>(this->tf_mutex_);

  pose = math::Pose3d::Zero;

  auto it = this->tfTree.find(frame);
  if (it != tfTree.end()) {
    pose = it->second;
    return true;
  }

  return false;
}

}  // namespace common
}  // namespace rviz
}  // namespace ignition
