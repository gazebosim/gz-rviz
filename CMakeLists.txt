cmake_minimum_required(VERSION 3.10)
project(ign-rviz)
add_compile_options(-std=c++14)
set(CMAKE_AUTOMOC ON)

add_compile_options(-std=c++11 -O2)

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
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Ignition libraries
find_package(ignition-gui4 REQUIRED)
find_package(ignition-common3 REQUIRED)
find_package(ignition-rendering4 REQUIRED)

qt5_add_resources(resources_RCC include/rviz/gui/rviz.qrc)

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
)

include_directories(include)

add_executable(ign-rviz src/gui/rviz.cpp include/rviz/gui/rviz.hpp ${resources_RCC})

ament_target_dependencies(ign-rviz
  ignition-gui4
)

target_link_libraries(ign-rviz Qt5::Core Qt5::Qml Qt5::Quick Qt5::QuickControls2)

install(TARGETS ign-rviz
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
