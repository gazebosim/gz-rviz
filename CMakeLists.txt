cmake_minimum_required(VERSION 2.8.3)
project(ign-rviz)

add_compile_options(-std=c++11 -O2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  pcl_conversions
)

find_package(ignition-rendering3 REQUIRED)

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

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ign-rviz
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ign_rviz src/GlutWindow.cc src/SceneManager.cpp src/VisualizationManager.cpp src/rviz.cpp)

target_link_libraries(ign_rviz
  ${catkin_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${IGNITION-RENDERING_LIBRARIES}
)
