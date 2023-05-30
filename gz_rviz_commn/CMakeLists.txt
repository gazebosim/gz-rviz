cmake_minimum_required(VERSION 3.5)
project(gz_rviz_common)

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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(Qt5Widgets CONFIG REQUIRED)

# Gazebo Libraries
if("$ENV{GZ_VERSION}" STREQUAL "fortress")
  find_package(ignition-math6)

  # Default to Fortress
  find_package(ignition-gui6 REQUIRED)

  set(GZ_MATH ignition-math6)
  set(GZ_GUI ignition-gui6)
  message(STATUS "Compiling against Gazebo Fortress")
else()
  find_package(gz-math7)

  # Default to Fortress
  find_package(gz-gui7 REQUIRED)

  set(GZ_MATH gz-math7)
  set(GZ_GUI gz-gui7)
  message(STATUS "Compiling against Gazebo Garden")
  add_definitions(-DGZ_HEADERS)
endif()

include_directories(include)

add_library(gz_rviz_common SHARED
  include/gz/rviz/common/frame_manager.hpp
  src/rviz/common/frame_manager.cpp
)

ament_target_dependencies(gz_rviz_common
  rclcpp
  tf2_ros
  tf2_msgs
  geometry_msgs
  tf2_geometry_msgs
  ${GZ_MATH}
  ${GZ_GUI}
)

target_include_directories(gz_rviz_common
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Qt5Widgets_INCLUDE_DIRS}
  ${IGNITION-GUI_INCLUDE_DIRS}
)

install(
  TARGETS gz_rviz_common
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
