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
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)

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
  tf2_ros
  ignition-gui4
  ignition-math6
  ignition-rendering4
  ign_rviz_common
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
  tf2_ros
  ignition-gui4
  ignition-math6
  ignition-rendering4
  ign_rviz_common
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
  tf2_ros
  sensor_msgs
  ignition-gui4
  ignition-math6
  ignition-rendering4
  ign_rviz_common
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
  tf2_ros
  sensor_msgs
  ignition-gui4
  ignition-math6
  ignition-rendering4
  ign_rviz_common
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

ament_export_libraries(AxesDisplay)

install(
  TARGETS AxesDisplay
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_libraries(GlobalOptions)

install(
  TARGETS GlobalOptions
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_libraries(TFDisplay)

install(
  TARGETS TFDisplay
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_libraries(LaserScanDisplay)

install(
  TARGETS LaserScanDisplay
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_libraries(ImageDisplay)

install(
  TARGETS ImageDisplay
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
