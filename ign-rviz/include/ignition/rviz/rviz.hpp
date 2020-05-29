//
// Created by Sarathkrishnan Ramesh on 22/5/20.
//

#ifndef IGNITION__RVIZ__RVIZ_HPP_
#define IGNITION__RVIZ__RVIZ_HPP_

#include <iostream>

#ifndef Q_MOC_RUN
  #include <ignition/gui/qt.h>
  #include <ignition/gui/Application.hh>
#endif

namespace ignition
{
namespace rviz
{
class RViz : public QObject
{
  Q_OBJECT

public:
  Q_INVOKABLE void addGrid3D() const
  {
    ignition::gui::App()->LoadPlugin("Grid3D");
  }
};
}  // namespace rviz
}  // namespace ignition

#endif  // IGNITION__RVIZ__RVIZ_HPP_
