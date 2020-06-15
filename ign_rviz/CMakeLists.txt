cmake_minimum_required(VERSION 3.5)
project(ign_rviz)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)

# Qt packages
find_package(Qt5 COMPONENTS
  Core
  Quick
  QuickControls2
  REQUIRED
)

# ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ign_rviz_plugins REQUIRED)
find_package(ign_rviz_common REQUIRED)

# Ignition libraries
find_package(ignition-gui4 REQUIRED)
find_package(ignition-common3 REQUIRED)
find_package(ignition-rendering4 REQUIRED)
find_package(ignition-math6 REQUIRED)

qt5_add_resources(resources_RCC include/ignition/rviz/rviz.qrc)

include_directories(SYSTEM
  ${IGNITION-COMMON_INCLUDE_DIRS}
  ${IGNITION-GUI_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Qml_INCLUDE_DIRS}
  ${Qt5Quick_INCLUDE_DIRS}
  ${Qt5QuickControls2_INCLUDE_DIRS}
)

link_directories(
  ${IGNITION-COMMON_LIBRARY_DIRS}
  ${IGNITION-GUI_LIBRARY_DIRS}
  ${IGNITION-MATH_LIBRARY_DIRS}
)

include_directories(include)

add_executable(ign_rviz src/rviz/rviz.cpp include/ignition/rviz/rviz.hpp ${resources_RCC})

ament_target_dependencies(ign_rviz
  ignition-gui4
  rclcpp
  pluginlib
  geometry_msgs
  tf2_msgs
  tf2_ros
  ign_rviz_plugins
  ign_rviz_common
  ignition-math6
)

target_link_libraries(ign_rviz Qt5::Core Qt5::Qml Qt5::Quick Qt5::QuickControls2)

install(TARGETS ign_rviz
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
