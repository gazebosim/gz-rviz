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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)

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
add_library(AxesDisplay SHARED ${headers_MOC}
  include/ignition/rviz/plugins/AxesDisplay.hpp
  include/ignition/rviz/plugins/message_display_base.hpp
  src/rviz/plugins/AxesDisplay.cpp
  ${resources_RCC}
)

ament_target_dependencies(AxesDisplay
  ign_rviz_common
  ignition-gui4
  ignition-math6
  ignition-rendering4
  tf2_ros
)

target_link_libraries(AxesDisplay Qt5::Core Qt5::Qml Qt5::Quick Qt5::QuickControls2)

target_include_directories(AxesDisplay
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Qt5Widgets_INCLUDE_DIRS}
  ${IGNITION-GUI_INCLUDE_DIRS}
)

########################################################################
add_library(GlobalOptions SHARED ${headers_MOC}
  include/ignition/rviz/plugins/GlobalOptions.hpp
  src/rviz/plugins/GlobalOptions.cpp
  ${resources_RCC}
)

ament_target_dependencies(GlobalOptions
  ign_rviz_common
  ignition-gui4
  ignition-math6
  ignition-rendering4
  tf2_ros
)

target_link_libraries(GlobalOptions Qt5::Core Qt5::Qml Qt5::Quick Qt5::QuickControls2)

target_include_directories(GlobalOptions
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Qt5Widgets_INCLUDE_DIRS}
  ${IGNITION-GUI_INCLUDE_DIRS}
)

########################################################################
add_library(ImageDisplay SHARED ${headers_MOC}
  include/ignition/rviz/plugins/ImageDisplay.hpp
  include/ignition/rviz/plugins/message_display_base.hpp
  src/rviz/plugins/ImageDisplay.cpp
  ${resources_RCC}
)

ament_target_dependencies(ImageDisplay
  ign_rviz_common
  ignition-gui4
  ignition-math6
  ignition-rendering4
  sensor_msgs
  tf2_ros
)

target_link_libraries(ImageDisplay Qt5::Core Qt5::Qml Qt5::Quick Qt5::QuickControls2)

target_include_directories(ImageDisplay
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Qt5Widgets_INCLUDE_DIRS}
  ${IGNITION-GUI_INCLUDE_DIRS}
)

########################################################################
add_library(LaserScanDisplay SHARED ${headers_MOC}
  include/ignition/rviz/plugins/LaserScanDisplay.hpp
  src/rviz/plugins/LaserScanDisplay.cpp
  ${resources_RCC}
)

ament_target_dependencies(LaserScanDisplay
  ign_rviz_common
  ignition-gui4
  ignition-math6
  ignition-rendering4
  sensor_msgs
  tf2_ros
)

target_link_libraries(LaserScanDisplay Qt5::Core Qt5::Qml Qt5::Quick Qt5::QuickControls2)

target_include_directories(LaserScanDisplay
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Qt5Widgets_INCLUDE_DIRS}
  ${IGNITION-GUI_INCLUDE_DIRS}
)

########################################################################
add_library(PointStampedDisplay SHARED ${headers_MOC}
  include/ignition/rviz/plugins/PointStampedDisplay.hpp
  src/rviz/plugins/PointStampedDisplay.cpp
  ${resources_RCC}
)

ament_target_dependencies(PointStampedDisplay
  geometry_msgs
  ign_rviz_common
  ignition-gui4
  ignition-math6
  ignition-rendering4
  tf2_ros
)

target_link_libraries(PointStampedDisplay Qt5::Core Qt5::Qml Qt5::Quick Qt5::QuickControls2)

target_include_directories(PointStampedDisplay
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Qt5Widgets_INCLUDE_DIRS}
  ${IGNITION-GUI_INCLUDE_DIRS}
)

########################################################################
add_library(PolygonDisplay SHARED ${headers_MOC}
  include/ignition/rviz/plugins/PolygonDisplay.hpp
  src/rviz/plugins/PolygonDisplay.cpp
  ${resources_RCC}
)

ament_target_dependencies(PolygonDisplay
  geometry_msgs
  ign_rviz_common
  ignition-gui4
  ignition-math6
  ignition-rendering4
  tf2_ros
)

target_link_libraries(PolygonDisplay Qt5::Core Qt5::Qml Qt5::Quick Qt5::QuickControls2)

target_include_directories(PolygonDisplay
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Qt5Widgets_INCLUDE_DIRS}
  ${IGNITION-GUI_INCLUDE_DIRS}
)

########################################################################
add_library(PoseDisplay SHARED ${headers_MOC}
  include/ignition/rviz/plugins/PoseDisplay.hpp
  src/rviz/plugins/PoseDisplay.cpp
  ${resources_RCC}
)

ament_target_dependencies(PoseDisplay
  geometry_msgs
  ign_rviz_common
  ignition-gui4
  ignition-math6
  ignition-rendering4
  tf2_ros
)

target_link_libraries(PoseDisplay Qt5::Core Qt5::Qml Qt5::Quick Qt5::QuickControls2)

target_include_directories(PoseDisplay
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Qt5Widgets_INCLUDE_DIRS}
  ${IGNITION-GUI_INCLUDE_DIRS}
)

########################################################################
add_library(RobotModelDisplay SHARED ${headers_MOC}
  include/ignition/rviz/plugins/RobotModelDisplay.hpp
  src/rviz/plugins/RobotModelDisplay.cpp
  ${resources_RCC}
)

ament_target_dependencies(RobotModelDisplay
  ign_rviz_common
  ignition-gui4
  ignition-math6
  ignition-rendering4
  std_msgs
  tf2_ros
  urdf
)

target_link_libraries(RobotModelDisplay Qt5::Core Qt5::Qml Qt5::Quick Qt5::QuickControls2)

target_include_directories(RobotModelDisplay
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Qt5Widgets_INCLUDE_DIRS}
  ${IGNITION-GUI_INCLUDE_DIRS}
)

########################################################################
add_library(TFDisplay SHARED ${headers_MOC}
  include/ignition/rviz/plugins/TFDisplay.hpp
  include/ignition/rviz/plugins/message_display_base.hpp
  src/rviz/plugins/TFDisplay.cpp
  ${resources_RCC}
)

ament_target_dependencies(TFDisplay
  tf2_ros
  ignition-gui4
  ignition-math6
  ignition-rendering4
  ign_rviz_common
)

target_link_libraries(TFDisplay Qt5::Core Qt5::Qml Qt5::Quick Qt5::QuickControls2)

target_include_directories(TFDisplay
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Qt5Widgets_INCLUDE_DIRS}
  ${IGNITION-GUI_INCLUDE_DIRS}
)

########################################################################
add_library(ign_rviz_plugins SHARED
  "include/ignition/rviz/plugins/message_display_base.hpp"
)

ament_target_dependencies(ign_rviz_plugins
  rclcpp
  tf2_ros
  ignition-gui4
  ign_rviz_common
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
  PointStampedDisplay
  PolygonDisplay
  PoseDisplay
  RobotModelDisplay
  TFDisplay
)

install(
  TARGETS
    AxesDisplay
    GlobalOptions
    ImageDisplay
    LaserScanDisplay
    PointStampedDisplay
    PolygonDisplay
    PoseDisplay
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
