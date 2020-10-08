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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)

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

qt5_add_resources(resources_RCC res/Plugins.qrc)

########################################################################
macro(add_ign_rviz_plugin)
  set(oneValueArgs NAME)
  set(multiValueArgs DEPENDENCIES)
  cmake_parse_arguments(IGN_RVIZ_PLUGIN "" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  add_library(${IGN_RVIZ_PLUGIN_NAME} SHARED ${headers_MOC}
    include/ignition/rviz/plugins/${IGN_RVIZ_PLUGIN_NAME}.hpp
    include/ignition/rviz/plugins/message_display_base.hpp
    src/rviz/plugins/${IGN_RVIZ_PLUGIN_NAME}.cpp
    ${resources_RCC}
  )

  ament_target_dependencies(
    ${IGN_RVIZ_PLUGIN_NAME}
    ${IGN_RVIZ_PLUGIN_DEPENDENCIES}
  )

  target_link_libraries(
    ${IGN_RVIZ_PLUGIN_NAME}
    Qt5::Core Qt5::Qml Qt5::Quick Qt5::QuickControls2
  )

  target_include_directories(${IGN_RVIZ_PLUGIN_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${Qt5Widgets_INCLUDE_DIRS}
    ${IGNITION-GUI_INCLUDE_DIRS}
  )
endmacro()

########################################################################
add_ign_rviz_plugin(
  NAME AxesDisplay
  DEPENDENCIES
    ign_rviz_common
    ignition-gui4
    ignition-math6
    ignition-rendering4
    tf2_ros
)

########################################################################
add_ign_rviz_plugin(
  NAME GlobalOptions
  DEPENDENCIES
    ign_rviz_common
    ignition-gui4
    ignition-math6
    ignition-rendering4
    tf2_ros
)

########################################################################
add_ign_rviz_plugin(
  NAME ImageDisplay
  DEPENDENCIES
    ign_rviz_common
    ignition-gui4
    ignition-math6
    ignition-rendering4
    sensor_msgs
    tf2_ros
)

########################################################################
add_ign_rviz_plugin(
  NAME LaserScanDisplay
  DEPENDENCIES
    tf2_ros
    sensor_msgs
    ignition-gui4
    ignition-math6
    ignition-rendering4
    ign_rviz_common
)

########################################################################
add_ign_rviz_plugin(
  NAME MarkerDisplay
  DEPENDENCIES
    tf2_ros
    geometry_msgs
    visualization_msgs
    ignition-gui4
    ignition-math6
    ignition-rendering4
    ign_rviz_common
)

########################################################################
add_ign_rviz_plugin(
  NAME PathDisplay
  DEPENDENCIES
    geometry_msgs
    ign_rviz_common
    ignition-gui4
    ignition-math6
    ignition-rendering4
    nav_msgs
    tf2_ros
)

########################################################################
add_ign_rviz_plugin(
  NAME PointStampedDisplay
  DEPENDENCIES
    geometry_msgs
    ign_rviz_common
    ignition-gui4
    ignition-math6
    ignition-rendering4
    tf2_ros
)

########################################################################
add_ign_rviz_plugin(
  NAME PolygonDisplay
  DEPENDENCIES
    geometry_msgs
    ign_rviz_common
    ignition-gui4
    ignition-math6
    ignition-rendering4
    tf2_ros
)

########################################################################
add_ign_rviz_plugin(
  NAME PoseDisplay
  DEPENDENCIES
    geometry_msgs
    ign_rviz_common
    ignition-gui4
    ignition-math6
    ignition-rendering4
    tf2_ros
)

########################################################################
add_ign_rviz_plugin(
  NAME PoseArrayDisplay
  DEPENDENCIES
    geometry_msgs
    ign_rviz_common
    ignition-gui4
    ignition-math6
    ignition-rendering4
    tf2_ros
)

########################################################################
add_ign_rviz_plugin(
  NAME RobotModelDisplay
  DEPENDENCIES
    ign_rviz_common
    ignition-gui4
    ignition-math6
    ignition-rendering4
    std_msgs
    tf2_ros
    urdf
)

########################################################################
add_ign_rviz_plugin(
  NAME TFDisplay
  DEPENDENCIES
    tf2_ros
    ignition-gui4
    ignition-math6
    ignition-rendering4
    ign_rviz_common
)

########################################################################
add_library(ign_rviz_plugins SHARED
  "include/ignition/rviz/plugins/message_display_base.hpp"
)

ament_target_dependencies(ign_rviz_plugins
  ign_rviz_common
  ignition-gui4
  rclcpp
  tf2_ros
)

target_include_directories(ign_rviz_plugins
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Qt5Widgets_INCLUDE_DIRS}
  ${IGNITION-GUI_INCLUDE_DIRS}
)

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

########################################################################
ament_export_libraries(
  AxesDisplay
  GlobalOptions
  ImageDisplay
  LaserScanDisplay
  MarkerDisplay
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
    ImageDisplay
    LaserScanDisplay
    MarkerDisplay
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
