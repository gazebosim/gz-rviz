from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='ign-rviz', node_executable='ign-rviz', output='screen', parameters=[
                {
                    "point_topic": "/point",
                    "pose_topic": "/pose",
                    "imu_topic": "/imu",
                    "marker_topic": "/visualization_marker",
                    "tf_topic": "/tf",
                    "pointcloud_topic": "/cloud_in",
                    "fixed_frame": "world"
                },
            ]),
    ])
