# Ignition-Rviz
ROS package for Robot Visualization(RViz) developed using Ignition Rendering Library.

Switch to branch `melodic-devel` for ign-rviz for ROS1

### Instructions
Build using catkin then run the following command to run

    ros2 launch ign-rviz rviz.py

This project is still under development. Currently only supports limited topic visualization which can be configured in the launch file.

#### Configurations
|Parameter|Description|
|---|---|
|`point_topic`|Topic name for visualizing `geometry_msgs/msg/PointStamped` msgs|
|`pose_topic`|Topic name for visualizing `geometry_msgs/msg/PoseStamped` msgs|
|`imu_topic` |Topic name for IMU data visualization|
|`marker_topic` |Topic name for visualization markers|
|`tf_topic` |Topic name for tf_static data visualization|
|`pointcloud_topic` |Topic name for PointCloud2 data visualization|