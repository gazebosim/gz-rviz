cmake_minimum_required(VERSION 3.5)
project(gz_rviz_plugins)

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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)

# Gazebo Libraries
if("$ENV{GZ_VERSION}" STREQUAL "fortress")
  find_package(ignition-math6 REQUIRED)
  set(GZ_MATH ignition-math6)

  find_package(ignition-gui6 REQUIRED)
  set(GZ_GUI ignition-gui6)

  find_package(ignition-common4 REQUIRED)
  set(GZ_COMMON ignition-common4)

  find_package(ignition-plugin1 REQUIRED)
  set(GZ_PLUGIN ignition-plugin1)

  find_package(ignition-rendering6 REQUIRED)
  set(GZ_RENDERING ignition-rendering6)
  message(STATUS "Compiling against Gazebo Fortress")
else()
  find_package(gz-math7 REQUIRED)
  set(GZ_MATH gz-math7)

  find_package(gz-gui7 REQUIRED)
  set(GZ_GUI gz-gui7)

  find_package(gz-common5 REQUIRED)
  set(GZ_COMMON gz-common5)

  find_package(gz-plugin2 REQUIRED)
  set(GZ_PLUGIN gz-plugin2)

  find_package(gz-rendering7 REQUIRED)
  set(GZ_RENDERING gz-rendering7)
  message(STATUS "Compiling against Gazebo Garden")
  add_definitions(-DGZ_HEADERS)
endif()

find_package(Qt5Widgets CONFIG REQUIRED)

find_package(gz_rviz_common REQUIRED)

include_directories(include)

qt5_add_resources(resources_RCC res/Plugins.qrc)

########################################################################
macro(add_gz_rviz_plugin)
  set(oneValueArgs NAME)
  set(multiValueArgs EXTRA_FILES DEPENDENCIES)
  cmake_parse_arguments(GZ_RVIZ_PLUGIN "" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  add_library(${GZ_RVIZ_PLUGIN_NAME} SHARED ${headers_MOC}
    include/gz/rviz/plugins/${GZ_RVIZ_PLUGIN_NAME}.hpp
    include/gz/rviz/plugins/message_display_base.hpp
    src/rviz/plugins/${GZ_RVIZ_PLUGIN_NAME}.cpp
    ${GZ_RVIZ_PLUGIN_EXTRA_FILES}
    ${resources_RCC}
  )

  ament_target_dependencies(
    ${GZ_RVIZ_PLUGIN_NAME}
    ${GZ_RVIZ_PLUGIN_DEPENDENCIES}
  )

  target_link_libraries(
    ${GZ_RVIZ_PLUGIN_NAME}
    Qt5::Core Qt5::Qml Qt5::Quick Qt5::QuickControls2
    ${GZ_GUI}
  )

  target_include_directories(${GZ_RVIZ_PLUGIN_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${Qt5Widgets_INCLUDE_DIRS}
  )
endmacro()

########################################################################
add_gz_rviz_plugin(
  NAME AxesDisplay
  DEPENDENCIES
    gz_rviz_common
    ${GZ_GUI}
    ${GZ_MATH}
    ${GZ_PLUGIN}
    ${GZ_RENDERING}
    tf2_ros
)

########################################################################
add_gz_rviz_plugin(
  NAME GlobalOptions
  DEPENDENCIES
    gz_rviz_common
    ${GZ_GUI}
    ${GZ_MATH}
    ${GZ_PLUGIN}
    ${GZ_RENDERING}
    tf2_ros
)

########################################################################
add_gz_rviz_plugin(
  NAME GPSDisplay
  DEPENDENCIES
    gz_rviz_common
    ${GZ_GUI}
    ${GZ_PLUGIN}
    ${GZ_MATH}
    sensor_msgs
    tf2_ros
)

########################################################################
add_gz_rviz_plugin(
  NAME ImageDisplay
  DEPENDENCIES
    gz_rviz_common
    ${GZ_GUI}
    ${GZ_MATH}
    ${GZ_PLUGIN}
    ${GZ_RENDERING}
    sensor_msgs
    tf2_ros
)

########################################################################
add_gz_rviz_plugin(
  NAME LaserScanDisplay
  DEPENDENCIES
    tf2_ros
    sensor_msgs
    ${GZ_GUI}
    ${GZ_MATH}
    ${GZ_PLUGIN}
    ${GZ_RENDERING}
    gz_rviz_common
)

########################################################################
add_gz_rviz_plugin(
  NAME MarkerDisplay
  EXTRA_FILES
    src/rviz/plugins/MarkerManager.cpp
  DEPENDENCIES
    geometry_msgs
    gz_rviz_common
    ${GZ_GUI}
    ${GZ_MATH}
    ${GZ_PLUGIN}
    ${GZ_RENDERING}
    tf2_ros
    visualization_msgs
)

########################################################################
add_gz_rviz_plugin(
  NAME MarkerArrayDisplay
  EXTRA_FILES
    src/rviz/plugins/MarkerManager.cpp
  DEPENDENCIES
    geometry_msgs
    gz_rviz_common
    ${GZ_GUI}
    ${GZ_MATH}
    ${GZ_PLUGIN}
    ${GZ_RENDERING}
    tf2_ros
    visualization_msgs
)

########################################################################
add_gz_rviz_plugin(
  NAME PathDisplay
  DEPENDENCIES
    geometry_msgs
    gz_rviz_common
    ${GZ_GUI}
    ${GZ_MATH}
    ${GZ_PLUGIN}
    ${GZ_RENDERING}
    nav_msgs
    tf2_ros
)

########################################################################
add_gz_rviz_plugin(
  NAME PointStampedDisplay
  DEPENDENCIES
    geometry_msgs
    gz_rviz_common
    ${GZ_GUI}
    ${GZ_MATH}
    ${GZ_PLUGIN}
    ${GZ_RENDERING}
    tf2_ros
)

########################################################################
add_gz_rviz_plugin(
  NAME PolygonDisplay
  DEPENDENCIES
    geometry_msgs
    gz_rviz_common
    ${GZ_GUI}
    ${GZ_MATH}
    ${GZ_PLUGIN}
    ${GZ_RENDERING}
    tf2_ros
)

########################################################################
add_gz_rviz_plugin(
  NAME PoseDisplay
  DEPENDENCIES
    geometry_msgs
    gz_rviz_common
    ${GZ_GUI}
    ${GZ_MATH}
    ${GZ_PLUGIN}
    ${GZ_RENDERING}
    tf2_ros
)

########################################################################
add_gz_rviz_plugin(
  NAME PoseArrayDisplay
  DEPENDENCIES
    geometry_msgs
    gz_rviz_common
    ${GZ_GUI}
    ${GZ_MATH}
    ${GZ_PLUGIN}
    ${GZ_RENDERING}
    tf2_ros
)

########################################################################
add_gz_rviz_plugin(
  NAME RobotModelDisplay
  DEPENDENCIES
    gz_rviz_common
    ${GZ_GUI}
    ${GZ_MATH}
    ${GZ_PLUGIN}
    ${GZ_RENDERING}
    std_msgs
    tf2_ros
    urdf
)

########################################################################
add_gz_rviz_plugin(
  NAME TFDisplay
  DEPENDENCIES
    tf2_ros
    ${GZ_GUI}
    ${GZ_MATH}
    ${GZ_PLUGIN}
    ${GZ_RENDERING}
    gz_rviz_common
)

########################################################################
add_library(gz_rviz_plugins SHARED
  "include/gz/rviz/plugins/message_display_base.hpp"
)

ament_target_dependencies(gz_rviz_plugins
  gz_rviz_common
  ${GZ_GUI}
  rclcpp
  tf2_ros
)

target_include_directories(gz_rviz_plugins
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Qt5Widgets_INCLUDE_DIRS}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

ament_export_libraries(gz_rviz_plugins)

install(
  TARGETS gz_rviz_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

########################################################################
ament_export_libraries(
  AxesDisplay
  GlobalOptions
  GPSDisplay
  ImageDisplay
  LaserScanDisplay
  MarkerDisplay
  MarkerArrayDisplay
  PathDisplay
  PointStampedDisplay
  PolygonDisplay
  PoseDisplay
  PoseArrayDisplay
  RobotModelDisplay
  TFDisplay
)

install(
  TARGETS
    AxesDisplay
    GlobalOptions
    GPSDisplay
    ImageDisplay
    LaserScanDisplay
    MarkerDisplay
    MarkerArrayDisplay
    PathDisplay
    PointStampedDisplay
    PolygonDisplay
    PoseDisplay
    PoseArrayDisplay
    RobotModelDisplay
    TFDisplay
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
