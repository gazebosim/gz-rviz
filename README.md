# Ignition-Rviz
ROS package for Robot Visualization(RViz) developed using Ignition Rendering Library.

### Instructions
Build using catkin then run the following command to run

    roslaunch ign-rviz rviz.launch

This project is still under development. Currently only supports limited topic visualization which can be configured in the launch file.

#### Configurations
|Parameter|Description|
|---|---|
|`point_topic`|Topic name for visualizing `geometry_msgs/PointStamped` msgs|
|`pose_topic`|Topic name for visualizing `geometry_msgs/PoseStamped` msgs|
|`imu_topic` |Topic name for IMU data visualization|
|`marker_topic` |Topic name for visualization markers|
|`tf_topic` |Topic name for tf_static data visualization|
|`pointcloud_topic` |Topic name for PointCloud2 data visualization|