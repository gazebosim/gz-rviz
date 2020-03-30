import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker
from marker_TEST import create_line
from marker_TEST import create_marker


class PointPublisher(Node):

    def __init__(self):
        super().__init__('point_msg_TEST')
        self.publisher_ = self.create_publisher(PointStamped, 'point', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = PointStamped()
        msg.header.frame_id = "base_link"

        msg.point.x = 1.0
        msg.point.y = 2.0
        msg.point.z = 3.0

        print(msg)

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing point')
        self.i += 1


class PosePublisher(Node):

    def __init__(self):
        super().__init__('pose_msg_TEST')
        self.publisher_ = self.create_publisher(PoseStamped, 'pose', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = PoseStamped()
        msg.header.frame_id = "base_link"

        msg.pose.position.x = -2.0
        msg.pose.position.y = -2.0
        msg.pose.position.z = 2.0

        msg.pose.orientation.x = 0.707
        msg.pose.orientation.y = 0.707
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 0.0

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing pose')
        self.i += 1


class OrientationPublisher(Node):

    def __init__(self):
        super().__init__('imu_msg_TEST')
        self.publisher_ = self.create_publisher(Imu, 'imu', 10)
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Imu()
        msg.header.frame_id = "base_link"

        msg.orientation.x = 0.503
        msg.orientation.y = 0.0
        msg.orientation.z = 0.503
        msg.orientation.w = 0.704

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing orientation')
        self.i += 1


class MarkerPublisher(Node):

    def __init__(self):
        super().__init__('marker_msg_TEST')
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = create_marker(0, "marker_viz", "cube")
        msg.header.stamp.sec, msg.header.stamp.nanosec = self.get_clock().now().seconds_nanoseconds()
        self.publisher_.publish(msg)

        msg = create_marker(1, "marker_viz", "sphere")
        msg.header.stamp.sec, msg.header.stamp.nanosec = self.get_clock().now().seconds_nanoseconds()
        self.publisher_.publish(msg)

        msg = create_marker(2, "marker_viz", "cylinder")
        msg.header.stamp.sec, msg.header.stamp.nanosec = self.get_clock().now().seconds_nanoseconds()
        self.publisher_.publish(msg)

        msg = create_marker(3, "marker_viz", "arrow")
        msg.header.stamp.sec, msg.header.stamp.nanosec = self.get_clock().now().seconds_nanoseconds()
        self.publisher_.publish(msg)

        msg = create_line(0, "line_viz", "list")
        msg.header.stamp.sec, msg.header.stamp.nanosec = self.get_clock().now().seconds_nanoseconds()
        self.publisher_.publish(msg)

        msg = create_line(1, "line_viz", "strip")
        msg.header.stamp.sec, msg.header.stamp.nanosec = self.get_clock().now().seconds_nanoseconds()
        self.publisher_.publish(msg)

        self.get_logger().info('Published markers')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    viz_publisher = MarkerPublisher()

    rclpy.spin(viz_publisher)

    viz_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
