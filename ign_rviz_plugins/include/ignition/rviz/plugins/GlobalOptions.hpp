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
#include <vector>

#include "ignition/rviz/plugins/message_display_base.hpp"

#include "ignition/rviz/common/frame_manager.hpp"

namespace ignition
{
namespace rviz
{
namespace plugins
{
/**
 * @brief TF status and message
 *
 * Displays tf status and message under GlobalOptions
 */
class TFStatus : public QObject
{
  Q_OBJECT

  /**
   *  @brief TF status
   */
  Q_PROPERTY(
    QString status
    READ getStatus
    NOTIFY statusChanged
  )

  /**
   *  @brief TF status message
   */
  Q_PROPERTY(
    QString message
    READ getMessage
    NOTIFY messageChanged
  )

  /**
   *  @brief TF status color
   */
  Q_PROPERTY(
    QString color
    READ getColor
    NOTIFY colorChanged
  )

public:
  // Constructor
  TFStatus();

  // Destructor
  ~TFStatus() {}

  /**
   * @brief Update tf status and message
   * @param[in] _fixedFrame FixedFrame name
   * @param[in] _allFrames List of all frames
   */
  void update(std::string & _fixedFrame, std::vector<std::string> & _allFrames);

  /**
   * @brief Get the TF status as a string
   * @return TF status
   */
  Q_INVOKABLE QString getStatus() const;

  /**
   * @brief Get the TF status message as a string
   * @return TF status message
   */
  Q_INVOKABLE QString getMessage() const;

  /**
   * @brief Get the TF status color as a string
   * @return TF status color
   */
  Q_INVOKABLE QString getColor() const;

signals:
  /**
   * @brief Notify that TF status has changed
   */
  void statusChanged();

signals:
  /**
   * @brief Notify that TF message has changed
   */
  void messageChanged();

signals:
  /**
   * @brief Notify that TF color has changed
   */
  void colorChanged();

private:
  QString status;
  QString message;
  QString color;
};

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

  /**
   *  @brief TF status
   */
  Q_PROPERTY(
    TFStatus * tfStatus
    READ getTfStatus
    NOTIFY tfStatusChanged
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
   * @brief Get the TF status as a TFStatus object
   * @return TFStatus object
   */
  Q_INVOKABLE TFStatus * getTfStatus() const;

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
   * @brief Notify that TF status has changed
   */
  void tfStatusChanged();

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
  TFStatus * tfStatus;
};

}  // namespace plugins
}  // namespace rviz
}  // namespace ignition

#endif  // IGNITION__RVIZ__PLUGINS__GLOBALOPTIONS_HPP_
