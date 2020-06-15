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

#ifndef IGNITION__RVIZ__PLUGINS__TF_DISPLAY_HPP_
#define IGNITION__RVIZ__PLUGINS__TF_DISPLAY_HPP_

#include <ignition/rendering.hh>
#include <tf2_msgs/msg/tf_message.hpp>

#include <string>
#include <mutex>
#include <memory>
#include <vector>

#include "ignition/rviz/plugins/message_display_base.hpp"

namespace ignition
{
namespace rviz
{
namespace plugins
{
class TFDisplay : public MessageDisplay<tf2_msgs::msg::TFMessage>
{
  Q_OBJECT

public:
  /**
   * Constructor for tf visualization plugin
   */
  TFDisplay();

  ~TFDisplay();

  // Documentation Inherited
  void initialize(rclcpp::Node::SharedPtr);

  // Documentation Inherited
  void callback(const tf2_msgs::msg::TFMessage::SharedPtr) {}

  // Documentation inherited
  void setTopic(std::string);

  /**
   * @brief Qt eventFilters. Original documentation can be found
   * <a href="https://doc.qt.io/qt-5/qobject.html#eventFilter">here</a>
   */
  bool eventFilter(QObject *, QEvent *);

  // Documentation inherited
  void installEventFilter(ignition::gui::MainWindow *);

  // Documentation inherited
  void setFrameManager(std::shared_ptr<common::FrameManager>);

protected:
  /**
   * @brief Create custom arrow visual for visualizing tf links
   * @return tf arrow visual
   */
  rendering::ArrowVisualPtr createTfArrow();

  /**
   * @brief Update tf visualization
   */
  void updateTF();

  /**
   * @brief Creates a frame visual which includes an axis
   * an arrow, and text visual
   * @return A frame visual
   */
  rendering::VisualPtr createVisualFrame();

private:
  ignition::rendering::AxisVisualPtr axis;
  ignition::rendering::RenderEngine * engine;
  ignition::rendering::ScenePtr scene;
  std::mutex lock;
  ignition::rendering::VisualPtr tfRootVisual;
};
}  // namespace plugins
}  // namespace rviz
}  // namespace ignition
#endif  // IGNITION__RVIZ__PLUGINS__TF_DISPLAY_HPP_
