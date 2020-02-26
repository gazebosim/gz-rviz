//
// Created by Sarathkrishnan Ramesh on 2/26/20.
//

#include "ign-rviz/VisualizationManager.h"

int main(int argc, char** argv) {
  // Initialize ROS Node
  ros::init(argc, argv, "ignition_rviz");
  ros::AsyncSpinner spinner(4);

  // Initialize Ignition RViz
  VisualizationManager rviz(argc, argv);

  // Start async spinner
  spinner.start();

  // Run renderer
  rviz.run();

  return 0;
}
