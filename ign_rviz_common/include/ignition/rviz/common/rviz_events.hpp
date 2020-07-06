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

#ifndef IGNITION__RVIZ__COMMON__RVIZ_EVENTS_HPP_
#define IGNITION__RVIZ__COMMON__RVIZ_EVENTS_HPP_

#include <QEvent>

namespace ignition
{
namespace rviz
{
/**
 * @brief Namespace for all events.
 */
namespace events
{
/**
 * Ignition RViz starting event id (50000) and counting up
 * to avoid collision with ign-gui and ign-gazebo events.
 */
static const unsigned int startingEventId = 50000;

/**
 * @brief Event is generated when the fixed frame changes
 */
class FixedFrameChanged : public QEvent
{
public:
  FixedFrameChanged()
  : QEvent(kType)
  {}
  /// \brief Unique type for this event.
  static const QEvent::Type kType = QEvent::Type(startingEventId);
};

/**
 * @brief Event is generated when the frame list is changed
 */
class FrameListChanged : public QEvent
{
public:
  FrameListChanged()
  : QEvent(kType)
  {}
  /// \brief Unique type for this event.
  static const QEvent::Type kType = QEvent::Type(startingEventId + 1);
};

}  // namespace events
}  // namespace rviz
}  // namespace ignition

#endif  // IGNITION__RVIZ__COMMON__RVIZ_EVENTS_HPP_
