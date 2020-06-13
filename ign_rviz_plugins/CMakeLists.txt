cmake_minimum_required(VERSION 3.5)
project(ign_rviz_plugins)

set(CMAKE_AUTOMOC ON)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)

# Ignition libraries
find_package(ignition-common3 REQUIRED)
find_package(ignition-rendering4 REQUIRED)
find_package(ignition-gui4 REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(Qt5Widgets CONFIG REQUIRED)

find_package(ign_rviz_common REQUIRED)

include_directories(include)

include_directories(SYSTEM
  ${IGNITION-COMMON_INCLUDE_DIRS}
  ${IGNITION-GUI_INCLUDE_DIRS}
  ${IGNITION-MATH_INCLUDE_DIRS}
)

set(ign_rviz_plugins_headers_to_moc
  "include/ignition/rviz/plugins/message_display_base.hpp"
  "include/ignition/rviz/plugins/tf_display.hpp"
)

add_library(ign_rviz_plugins SHARED
  ${ign_rviz_plugins_headers_to_moc}
  src/rviz/plugins/tf_display.cpp
)

ament_target_dependencies(ign_rviz_plugins
  ignition-rendering4
  pluginlib
  rclcpp
  geometry_msgs
  tf2_msgs
  tf2_ros
  ignition-gui4
  ignition-math6
  ign_rviz_common
)

target_include_directories(ign_rviz_plugins
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Qt5Widgets_INCLUDE_DIRS}
  ${IGNITION-GUI_INCLUDE_DIRS}
)

pluginlib_export_plugin_description_file(ign_rviz_plugins ign_rviz_plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(ign_rviz_plugins)

install(
  TARGETS ign_rviz_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
