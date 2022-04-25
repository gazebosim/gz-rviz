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
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# Ignition libraries
find_package(ign_rviz_common REQUIRED)
find_package(ign_rviz_plugins REQUIRED)
find_package(ignition-math6 REQUIRED)

# Dome
if("$ENV{IGNITION_VERSION}" STREQUAL "dome")
  find_package(ignition-gui4 REQUIRED)
  set(IGN_GUI_VER ${ignition-gui4_VERSION_MAJOR})

  find_package(ignition-common3 REQUIRED)
  set(IGN_COMMON_VER ${ignition-common3_VERSION_MAJOR})

  message(STATUS "Compiling against Ignition Dome")
# Default to Edifice
else()
  find_package(ignition-gui5 REQUIRED)
  set(IGN_GUI_VER ${ignition-gui5_VERSION_MAJOR})

  find_package(ignition-common4 REQUIRED)
  set(IGN_COMMON_VER ${ignition-common4_VERSION_MAJOR})

  message(STATUS "Compiling against Ignition Edifice")
endif()

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

add_executable(ign_rviz
  src/rviz/application.cpp
  src/rviz/rviz.cpp
  include/ignition/rviz/rviz.hpp
  ${resources_RCC}
)

ament_target_dependencies(ign_rviz
  ament_index_cpp
  ign_rviz_common
  ign_rviz_plugins
  ignition-math6
  ignition-gui${IGN_GUI_VER}
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2_msgs
  tf2_ros
  visualization_msgs
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
