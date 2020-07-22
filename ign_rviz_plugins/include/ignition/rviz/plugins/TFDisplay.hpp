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

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "ignition/rviz/plugins/message_display_base.hpp"

namespace ignition
{
namespace rviz
{
namespace plugins
{
////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Helper class to render TF tree view
 */
class FrameModel : public QStandardItemModel
{
  Q_OBJECT

public:
  /**
   * @brief Roles for tree view frames
   */
  enum FrameRoles
  {
    NameRole = Qt::UserRole + 1
  };

  // Constructor
  explicit FrameModel(QObject * _parent = 0);

  /**
   * @brief Add frame to tree view
   * @param[in] _name Frame name
   * @param[in] _parentItem Pointer to tree view parent item
   */
  Q_INVOKABLE void addFrame(const QString & _name, QStandardItem * _parentItem);

  /**
   * @brief Add a parent item to tree view
   * @param[in] _name Item name
   * @return Pointer to tree view parent item
   */
  Q_INVOKABLE QStandardItem * addParentRow(const QString & _name);

  // Documentation inherited
  QVariant data(const QModelIndex & _index, int _role = Qt::DisplayRole) const;

protected:
  // Documentation inherited
  QHash<int, QByteArray> roleNames() const;
};

////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Displays the TF transform hierarchy
 */
class TFDisplay : public MessageDisplay<tf2_msgs::msg::TFMessage>
{
  Q_OBJECT

  /**
   * @brief TF frame tree model
   */
  Q_PROPERTY(
    FrameModel * frameModel
    READ getFrameModel
    NOTIFY frameModelChanged
  )

public:
  /**
   * Constructor for tf visualization plugin
   */
  TFDisplay();

  // Destructor
  ~TFDisplay();

  // Documentation inherited
  void LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/) override;

  // Documentation Inherited
  void initialize(rclcpp::Node::SharedPtr _node) override;

  /**
   * @brief Qt eventFilters. Original documentation can be found
   * <a href="https://doc.qt.io/qt-5/qobject.html#eventFilter">here</a>
   */
  bool eventFilter(QObject * _object, QEvent * _event);

  // Documentation inherited
  void setFrameManager(std::shared_ptr<common::FrameManager> _frameManager) override;

  /**
   * @brief Set axis visibility
   * @param[in] _visible Axes visibility
   */
  Q_INVOKABLE void showAxes(const bool & _visible);

  /**
   * @brief Set arrow visibility
   * @param[in] _visible Arrow visibility
   */
  Q_INVOKABLE void showArrows(const bool & _visible);

  /**
   * @brief Set frame name visibility
   * @param[in] _visible Frame name visibility
   */
  Q_INVOKABLE void showNames(const bool & _visible);

  /**
   * @brief Set axes arrow head visibility
   * @param[in] _visible Axes arrow head visibility
   */
  Q_INVOKABLE void showAxesHead(const bool & _visible);

  /**
   * @brief Set marker scale
   * @param _scale TF visual marker scale
   */
  Q_INVOKABLE void setMarkerScale(const float & _scale);

  /**
   * @brief Set frame visibility
   * @param[in] _frame Frame name
   * @param[in] _visible Frame visibility
   */
  Q_INVOKABLE void setFrameVisibility(const QString & _frame, const bool & _visible);

  /**
   * @brief Get the tree view model
   * @return Tree view model
   */
  Q_INVOKABLE FrameModel * getFrameModel() const
  {
    return this->frameModel;
  }

signals:
  /**
   * @brief Notify that tree view has changed
   */
  void frameModelChanged();

protected:
  /**
   * @brief Create custom arrow visual for visualizing tf links
   * @return tf arrow visual
   */
  rendering::ArrowVisualPtr createTfArrow();

  /**
   * @brief Update tf visualization
   */
  void update() override;

  /**
   * @brief Creates a frame visual which includes an axis
   * an arrow, and text visual
   * @return A frame visual
   */
  rendering::VisualPtr createVisualFrame();

  /**
   * @brief Update tree view and local frame list
   */
  void refresh();

public:
  // Tree view frame model
  FrameModel * frameModel;

private:
  ignition::rendering::AxisVisualPtr axis;
  ignition::rendering::RenderEngine * engine;
  ignition::rendering::ScenePtr scene;
  ignition::rendering::VisualPtr tfRootVisual;
  std::mutex lock;
  bool axesVisible;
  bool arrowsVisible;
  bool namesVisible;
  bool axesHeadVisible;
  float markerScale;
  QStandardItem * parentRow;
  std::map<std::string, bool> frameInfo;
};

}  // namespace plugins
}  // namespace rviz
}  // namespace ignition

#endif  // IGNITION__RVIZ__PLUGINS__TFDISPLAY_HPP_
