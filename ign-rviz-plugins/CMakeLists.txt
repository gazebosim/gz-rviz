cmake_minimum_required(VERSION 3.10)
project(ign-rviz-plugins)

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

# Ignition libraries
find_package(ignition-common3 REQUIRED)
find_package(ignition-rendering4 REQUIRED)

include_directories(include)

include_directories(SYSTEM
  ${IGNITION-COMMON_INCLUDE_DIRS}
)

file(GLOB_RECURSE default_plugins "src/rviz/plugins/*.cpp")

add_library(ign-rviz-plugins SHARED
  ${default_plugins}
)

ament_target_dependencies(ign-rviz-plugins
  ignition-rendering4
  pluginlib
  rclcpp
  geometry_msgs
  tf2_msgs
)

target_include_directories(ign-rviz-plugins
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

pluginlib_export_plugin_description_file(ign-rviz-plugins ign-rviz-plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

install(
  TARGETS ign-rviz-plugins
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
