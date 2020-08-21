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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ignition/common/Console.hh>
#include <string>

#ifndef Q_MOC_RUN
  #include <ignition/gui/Application.hh>
  #include <ignition/gui/MainWindow.hh>
  #include <ignition/gui/qt.h>

  #include "ignition/rviz/rviz.hpp"
#endif

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  ignition::common::Console::SetVerbosity(4);

  ignition::gui::Application app(argc, argv);

  std::string package_share_directory = ament_index_cpp::get_package_share_directory("ign_rviz");
  app.LoadConfig(package_share_directory + "/config/rviz.config");

  std::string plugin_directory = ament_index_cpp::get_package_prefix("ign_rviz_plugins");
  ignition::gui::App()->AddPluginPath(plugin_directory + "/lib");

  ignition::rviz::RViz rviz;

  rviz.init_ros();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(rviz.get_node());
  std::thread executor_thread(std::bind(
      &rclcpp::executors::MultiThreadedExecutor::spin,
      &executor));

  auto context = new QQmlContext(app.Engine()->rootContext());
  context->setContextProperty("RViz", &rviz);

  QQmlComponent component(app.Engine(), ":/RViz/RVizDrawer.qml");
  auto item = qobject_cast<QQuickItem *>(component.create(context));
  if (!item) {
    ignerr << "Failed to initialize" << std::endl;
    return 1;
  }

  QQmlEngine::setObjectOwnership(item, QQmlEngine::CppOwnership);

  auto win = app.findChild<ignition::gui::MainWindow *>()->QuickWindow();
  win->setTitle("Ignition RViz");

  auto displayType = win->findChild<QQuickItem *>("sideDrawer");

  item->setParentItem(displayType);
  item->setParent(app.Engine());

  app.exec();

  executor.cancel();
  executor_thread.join();

  return 0;
}
