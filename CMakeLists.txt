cmake_minimum_required(VERSION 3.5)
project(ign-rviz)
add_compile_options(-std=c++14 -O2)
set(PYTHON_EXECUTABLE "/usr/bin/python3")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ignition-rendering4 REQUIRED)


include_directories(include)

find_package(GLUT REQUIRED)
include_directories(SYSTEM ${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})

find_package(OpenGL REQUIRED)
include_directories(SYSTEM ${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})

if (NOT APPLE)
  find_package(GLEW REQUIRED)
  include_directories(SYSTEM ${GLEW_INCLUDE_DIRS})
  link_directories(${GLEW_LIBRARY_DIRS})
endif()

add_executable(ign-rviz src/GlutWindow.cc src/SceneManager.cpp src/VisualizationManager.cpp src/rviz.cpp)

ament_target_dependencies(ign-rviz
  rclcpp
  std_msgs geometry_msgs sensor_msgs visualization_msgs
  tf2_geometry_msgs tf2_msgs tf2_ros
  pcl_ros pcl_conversions
  GLUT OPENGL GLEW
  ignition-rendering4
)

install(TARGETS ign-rviz
        DESTINATION lib/${PROJECT_NAME})

ament_package()
