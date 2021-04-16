cmake_minimum_required(VERSION 3.5)
project(ign_rviz_common)

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

# Igntiion Libraries
find_package(ignition-math6)

# Dome
if("$ENV{IGNITION_VERSION}" STREQUAL "dome")
  find_package(ignition-gui4 REQUIRED)
  set(IGN_GUI_VER ${ignition-gui4_VERSION_MAJOR})

  message(STATUS "Compiling against Ignition Dome")
# Default to Edifice
else()
  find_package(ignition-gui5 REQUIRED)
  set(IGN_GUI_VER ${ignition-gui5_VERSION_MAJOR})

  message(STATUS "Compiling against Ignition Edifice")
endif()

include_directories(include)

include_directories(SYSTEM
  ${IGNITION-MATH_INCLUDE_DIRS}
)

add_library(ign_rviz_common SHARED
  include/ignition/rviz/common/frame_manager.hpp
  src/rviz/common/frame_manager.cpp
)

ament_target_dependencies(ign_rviz_common
  rclcpp
  tf2_ros
  tf2_msgs
  geometry_msgs
  tf2_geometry_msgs
  ignition-math6
  ignition-gui${IGN_GUI_VER}
)

target_include_directories(ign_rviz_common
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Qt5Widgets_INCLUDE_DIRS}
  ${IGNITION-GUI_INCLUDE_DIRS}
)

install(
  TARGETS ign_rviz_common
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

install(
  TARGETS ign_rviz_common
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
