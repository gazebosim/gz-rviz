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

#ifndef IGNITION__RVIZ__PLUGINS__GLOBALOPTIONS_HPP_
#define IGNITION__RVIZ__PLUGINS__GLOBALOPTIONS_HPP_

#ifndef Q_MOC_RUN
  #include <ignition/gui/qt.h>
  #include <ignition/gui/Plugin.hh>
#endif

#include <ignition/rendering.hh>
#include <ignition/math/Color.hh>
#include <QColor>
#include <string>
#include <memory>
#include <utility>

#include "ignition/rviz/plugins/message_display_base.hpp"

#include "ignition/rviz/common/frame_manager.hpp"

namespace ignition
{
namespace rviz
{
namespace plugins
{
/**
 * @brief Configure global option of ignition rviz
 *
 * Provides methods to change fixed frame and scene background color
 */
class GlobalOptions : public MessageDisplayBase
{
  Q_OBJECT

  /**
   *  @brief Frame List
   */
  Q_PROPERTY(
    QStringList frameList
    READ getFrameList
    WRITE setFrameList
    NOTIFY frameListChanged
  )

public:
  // Constructor
  GlobalOptions();

  // Destructor
  ~GlobalOptions() {}

  // Documentation Inherited
  void LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/) override;

  // Documentation Inherited
  void setFrameManager(std::shared_ptr<common::FrameManager> _frameManager) override;

  /**
   * @brief Qt eventFilters. Original documentation can be found
   * <a href="https://doc.qt.io/qt-5/qobject.html#eventFilter">here</a>
   */
  bool eventFilter(QObject * _object, QEvent * _event);

  /**
   * @brief Set fixed frame
   * @param[in] _frame Fixed frame name
   */
  Q_INVOKABLE void setFrame(const QString & _frame);

  /**
   * @brief Set scene background color
   * @param[in] _color Background color
   */
  Q_INVOKABLE void setSceneBackground(const QColor & _color);

  /**
   * @brief Get the frame list as a string
   * @return List of frames
   */
  Q_INVOKABLE QStringList getFrameList() const;

  /**
   * @brief Set the frame list from a string
   * @param[in] _frameList List of frames
   */
  Q_INVOKABLE void setFrameList(const QStringList & _frameList);

signals:
  /**
   * @brief Notify that frame list has changed
   */
  void frameListChanged();

signals:
  /**
   * @brief Set combo box index
   * @param index Combo box index
   */
  void setCurrentIndex(const int index);

public slots:
  /**
   * @brief Callback when refresh button is pressed.
   */
  void onRefresh();

private:
  std::mutex lock;
  QStringList frameList;
  rendering::RenderEngine * engine;
  rendering::ScenePtr scene;
  bool dirty;
  bool initialized;
  bool populated;
  QColor color;
};

}  // namespace plugins
}  // namespace rviz
}  // namespace ignition

#endif  // IGNITION__RVIZ__PLUGINS__GLOBALOPTIONS_HPP_
