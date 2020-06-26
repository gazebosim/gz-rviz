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
class AxesDisplay : public MessageDisplayBase
{
  Q_OBJECT

public:
  // Constructor
  AxesDisplay();
  
  // Destructor
  ~AxesDisplay();
  
  // Documentation Inherited
  void LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/);
  
  // Documentation Inherited
  void setFrameManager(std::shared_ptr<common::FrameManager> frameManager);

  /**
   * @brief Qt eventFilters. Original documentation can be found
   * <a href="https://doc.qt.io/qt-5/qobject.html#eventFilter">here</a>
   */
  bool eventFilter(QObject *, QEvent *);

  /**
   * @brief Set axis frame
   */
  Q_INVOKABLE void setFrame(QString);

private:
  std::mutex lock;
  rendering::RenderEngine * engine;
  rendering::ScenePtr scene;
  rendering::AxisVisualPtr rootVisual;
  std::string frame;
};

}  // namespace plugins
}  // namespace rviz
}  // namespace ignition
#endif  // IGNITION__RVIZ__PLUGINS__AXESDISPLAY_HPP_
