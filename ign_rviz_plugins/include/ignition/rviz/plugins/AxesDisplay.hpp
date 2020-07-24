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

#ifndef IGNITION__RVIZ__PLUGINS__AXESDISPLAY_HPP_
#define IGNITION__RVIZ__PLUGINS__AXESDISPLAY_HPP_

#include "ignition/rviz/plugins/message_display_base.hpp"

#ifndef Q_MOC_RUN
  #include <ignition/gui/qt.h>
  #include <ignition/gui/Plugin.hh>
#endif
#include <ignition/rendering.hh>

#include <string>
#include <memory>
#include <utility>

namespace ignition
{
namespace rviz
{
namespace plugins
{
/**
 * @brief AxesDisplay plugin renders an axis at the Target Frame's origin
 */
class AxesDisplay : public MessageDisplayBase
{
  Q_OBJECT

  /**
   *  @brief Frame List
   */
  Q_PROPERTY(
    QStringList frameList
    READ getFrameList
    NOTIFY frameListChanged
  )

public:
  // Constructor
  AxesDisplay();

  // Destructor
  ~AxesDisplay();

  // Documentation Inherited
  void LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/) override;

  // Documentation Inherited
  void setFrameManager(std::shared_ptr<common::FrameManager> _frameManager) override;

  /**
   * @brief Qt eventFilters. Original documentation can be found
   * <a href="https://doc.qt.io/qt-5/qobject.html#eventFilter">here</a>
   */
  bool eventFilter(QObject *, QEvent *);

  /**
   * @brief Set axis frame
   */
  Q_INVOKABLE void setFrame(const QString &);

  /**
   * @brief Set axis length
   */
  Q_INVOKABLE void setLength(const float &);

  /**
   * @brief Set axis radius
   */
  Q_INVOKABLE void setRadius(const float &);

  /**
   * @brief Set axis arrow head visibility
   */
  Q_INVOKABLE void setHeadVisibility(const bool &);

  /**
   * @brief Get the frame list as a string
   * @return List of frames
   */
  Q_INVOKABLE QStringList getFrameList() const;

signals:
  /**
   * @brief Notify that frame list has changed
   */
  void frameListChanged();

public slots:
  /**
   * @brief Callback when refresh button is pressed.
   */
  void onRefresh();

private:
  void setScale();
  std::mutex lock;
  float length;
  float radius;
  bool headVisible;
  bool dirty;
  rendering::RenderEngine * engine;
  rendering::ScenePtr scene;
  rendering::AxisVisualPtr rootVisual;
  std::string frame;
  QStringList frameList;
};

}  // namespace plugins
}  // namespace rviz
}  // namespace ignition
#endif  // IGNITION__RVIZ__PLUGINS__AXESDISPLAY_HPP_
