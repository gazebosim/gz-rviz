//
// Created by Sarathkrishnan Ramesh on 22/5/20.
//

#include <ignition/common/Console.hh>

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

  app.LoadPlugin("Scene3D");

  ignition::rviz::RViz rviz_app;

  rviz_app.init_ros();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(rviz_app.node);
  std::thread executor_thread(std::bind(
      &rclcpp::executors::MultiThreadedExecutor::spin,
      &executor));

  auto context = new QQmlContext(app.Engine()->rootContext());
  context->setContextProperty("RViz", &rviz_app);

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
