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

#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>

#include <string>
#include <utility>
#include <memory>
#include <vector>

#include "ignition/rviz/common/rviz_events.hpp"
#include "ignition/rviz/common/frame_manager.hpp"

namespace ignition
{
namespace rviz
{
namespace common
{
/**
 * Stores reference to ROS Node
 * Creates a tfBuffer and tfListener.
 * Creates a tf subscription and binds callback to it.
 */
FrameManager::FrameManager(rclcpp::Node::SharedPtr node)
: QObject()
{
  this->node = std::move(node);

  tfBuffer = std::make_shared<tf2_ros::Buffer>(this->node->get_clock());
  tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

  this->subscriber = this->node->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", 10,
    std::bind(&FrameManager::tf_callback, this, std::placeholders::_1));
}

////////////////////////////////////////////////////////////////////////////////
void FrameManager::setFixedFrame(std::string fixedFrame)
{
  std::lock_guard<std::mutex>(this->tf_mutex_);

  this->tfTree.clear();
  this->fixedFrame = fixedFrame;

  // Send fixed frame changed event
  if (ignition::gui::App()) {
    ignition::gui::App()->sendEvent(
      ignition::gui::App()->findChild<ignition::gui::MainWindow *>(),
      new events::FixedFrameChanged());
  }
}

////////////////////////////////////////////////////////////////////////////////
std::string FrameManager::getFixedFrame()
{
  std::lock_guard<std::mutex>(this->tf_mutex_);
  return this->fixedFrame;
}

////////////////////////////////////////////////////////////////////////////////
void FrameManager::getFrames(std::vector<std::string> & frames)
{
  tfBuffer->_getFrameStrings(frames);
}

////////////////////////////////////////////////////////////////////////////////
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
  timePoint =
    tf2::TimePoint(
    std::chrono::seconds(timeStamp.sec) +
    std::chrono::nanoseconds(timeStamp.nanosec));

  for (const auto frame : frame_ids) {
    try {
      /*
       * TODO(Sarathkrishnan Ramesh): Reducing the tiemout for lookupTransform affects
       * smoothness of tf visualization.
       */
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
    } catch (tf2::LookupException & e) {
      RCLCPP_WARN(this->node->get_logger(), e.what());
    } catch (tf2::ConnectivityException & e) {
      RCLCPP_WARN(this->node->get_logger(), e.what());
    } catch (tf2::ExtrapolationException & e) {
      RCLCPP_WARN(this->node->get_logger(), e.what());
    } catch (tf2::InvalidArgumentException & e) {
      RCLCPP_WARN(this->node->get_logger(), e.what());
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
bool FrameManager::getParentPose(std::string & child, ignition::math::Pose3d & pose)
{
  std::lock_guard<std::mutex>(this->tf_mutex_);

  std::string parent;
  bool parentAvailable = tfBuffer->_getParent(child, this->timePoint, parent);

  if (!parentAvailable) {
    return false;
  }

  return this->getFramePose(parent, pose);
}

}  // namespace common
}  // namespace rviz
}  // namespace ignition
