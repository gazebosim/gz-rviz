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
FrameManager::FrameManager(rclcpp::Node::SharedPtr _node)
: QObject(), node(std::move(_node))
{
  tfBuffer = std::make_shared<tf2_ros::Buffer>(this->node->get_clock());
  tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
  frameList = getFrames();
  connect(&frameListTimer, SIGNAL(timeout()), this, SLOT(updateFrameList()));
  frameListTimer.start(1000);
}

////////////////////////////////////////////////////////////////////////////////
void FrameManager::setFixedFrame(const std::string & _fixedFrame)
{
  std::lock_guard<std::mutex>(this->tf_mutex_);

  this->fixedFrame = _fixedFrame;

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
std::vector<std::string> FrameManager::getFrames()
{
  return tfBuffer->getAllFrameNames();
}

////////////////////////////////////////////////////////////////////////////////
bool FrameManager::getFramePose(const std::string & _frame, ignition::math::Pose3d & _pose)
{
  tf_mutex_.lock();
  std::string fixedFrame = this->fixedFrame;
  tf_mutex_.unlock();

  if (fixedFrame.empty()) {
    RCLCPP_ERROR(this->node->get_logger(), "No fixed frame specified");
    return false;
  }

  try {
    geometry_msgs::msg::TransformStamped tf = tfBuffer->lookupTransform(
      fixedFrame,
      _frame, rclcpp::Time(0),
      tf2::Duration(1000));

    _pose = ignition::math::Pose3d(
      tf.transform.translation.x,
      tf.transform.translation.y,
      tf.transform.translation.z,
      tf.transform.rotation.w,
      tf.transform.rotation.x,
      tf.transform.rotation.y,
      tf.transform.rotation.z);

    return true;
  } catch (tf2::TransformException & e) {
    RCLCPP_WARN(this->node->get_logger(), e.what());
  }

  return false;
}

////////////////////////////////////////////////////////////////////////////////
bool FrameManager::getParentPose(const std::string & _child, ignition::math::Pose3d & _pose)
{
  std::string parent;
  // TODO(shrijitsingh99): The _getParent() API is only present for backwards compatability,
  // use some alternative method instead
  if (tfBuffer->_getParent(_child, tf2::TimePointZero, parent)) {
    return this->getFramePose(parent, _pose);
  }

  return false;
}

////////////////////////////////////////////////////////////////////////////////
void FrameManager::updateFrameList()
{
  std::lock_guard<std::mutex>(this->tf_mutex_);
  auto updatedFrameList = getFrames();
  if (frameList != updatedFrameList) {
    // Send frame list changed event
    if (ignition::gui::App()) {
      ignition::gui::App()->sendEvent(
        ignition::gui::App()->findChild<ignition::gui::MainWindow *>(),
        new events::FrameListChanged());
    }
  }
  frameList = std::move(updatedFrameList);
}


}  // namespace common
}  // namespace rviz
}  // namespace ignition
