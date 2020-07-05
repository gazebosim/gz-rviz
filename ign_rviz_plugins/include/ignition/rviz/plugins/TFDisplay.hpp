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

#ifndef IGNITION__RVIZ__PLUGINS__TFDISPLAY_HPP_
#define IGNITION__RVIZ__PLUGINS__TFDISPLAY_HPP_

#include <ignition/rendering.hh>
#include <tf2_msgs/msg/tf_message.hpp>

#include <QStandardItem>

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
////////////////////////////////////////////////////////////////////////////////
class FrameModel : public QStandardItemModel
{
  Q_OBJECT

public:
  enum FrameRoles
  {
    NameRole = Qt::UserRole + 1
  };

  explicit FrameModel(QObject * parent = 0);

  Q_INVOKABLE void addFrame(const QString & _name, QStandardItem * _parentItem);
  Q_INVOKABLE QStandardItem * addParentRow(const QString & _name);

  QVariant data(const QModelIndex & _index, int _role = Qt::DisplayRole) const;

protected:
  QHash<int, QByteArray> roleNames() const;
};

////////////////////////////////////////////////////////////////////////////////
class TFDisplay : public MessageDisplay<tf2_msgs::msg::TFMessage>
{
  Q_OBJECT

public:
  /**
   * Constructor for tf visualization plugin
   */
  TFDisplay();

  // Destructor
  ~TFDisplay();

  // Documentation inherited
  void LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/);

  // Documentation Inherited
  void initialize(rclcpp::Node::SharedPtr);

  // Documentation Inherited
  void callback(const tf2_msgs::msg::TFMessage::SharedPtr) {}

  // Documentation inherited
  void setTopic(std::string) {}

  /**
   * @brief Qt eventFilters. Original documentation can be found
   * <a href="https://doc.qt.io/qt-5/qobject.html#eventFilter">here</a>
   */
  bool eventFilter(QObject *, QEvent *);

  // Documentation inherited
  void setFrameManager(std::shared_ptr<common::FrameManager>);

  /**
   * @brief Set axis visibility
   */
  Q_INVOKABLE void showAxes(const bool & _visible);

  /**
   * @brief Set arrow visibility
   */
  Q_INVOKABLE void showArrows(const bool & _visible);

  /**
   * @brief Set frame name visibility
   */
  Q_INVOKABLE void showNames(const bool & _visible);

  /**
   * @brief Set axes arrow head visibility
   */
  Q_INVOKABLE void showAxesHead(const bool & _visible);

  /**
   * @brief Set marker scale
   */
  Q_INVOKABLE void setMarkerScale(const float & _scale);

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

  void refresh();

private:
  ignition::rendering::AxisVisualPtr axis;
  ignition::rendering::RenderEngine * engine;
  ignition::rendering::ScenePtr scene;
  std::mutex lock;
  ignition::rendering::VisualPtr tfRootVisual;
  bool axesVisible;
  bool arrowsVisible;
  bool namesVisible;
  bool axesHeadVisible;
  float markerScale;
  QStandardItem * parentRow;

public:
  FrameModel * model;
};

}  // namespace plugins
}  // namespace rviz
}  // namespace ignition

#endif  // IGNITION__RVIZ__PLUGINS__TFDISPLAY_HPP_
