from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


def create_line(marker_id, ns, marker_type):
    marker = Marker()

    # marker.header.stamp = rospy.Time().now()
    marker.ns = ns
    marker.id = marker_id

    flag = 1

    if marker_type == "strip":
        marker.type = marker.LINE_STRIP
        flag = -1
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

    else:
        marker.type = marker.LINE_LIST
        flag = 1
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

    for i in range(12):
        p = Point()
        p.x = float(-i)
        p.y = float(i * flag)
        p.z = float(i)
        marker.points.append(p)

    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0

    return marker


def create_marker(marker_id, ns, marker_type):
    marker = Marker()

    # marker.header.stamp = rospy.Time().now()
    marker.ns = ns
    marker.id = marker_id

    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    if marker_type == "cylinder":
        marker.type = Marker.CYLINDER
        marker.pose.position.x = 1.0
        marker.pose.position.y = 1.0
        marker.pose.position.z = 1.0

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

    elif marker_type == "cube":
        marker.type = Marker.CUBE
        marker.pose.position.x = 2.0
        marker.pose.position.y = 2.0
        marker.pose.position.z = 2.0

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

    elif marker_type == "sphere":
        marker.type = Marker.SPHERE
        marker.pose.position.x = 3.0
        marker.pose.position.y = 3.0
        marker.pose.position.z = 3.0

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

    else:
        marker.type = Marker.ARROW
        marker.pose.position.x = 4.0
        marker.pose.position.y = 4.0
        marker.pose.position.z = 4.0

        marker.pose.orientation.x = 0.5
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.5
        marker.pose.orientation.w = 0.707

    marker.action = Marker.ADD

    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0

    return marker


if __name__ == "__main__":
    import rospy
    rospy.init_node("marker_test_node")
    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
    rate = rospy.Rate(1)
    count = 0
    while not rospy.is_shutdown():
        marker_cube = create_marker(0, "marker_viz", "cube")
        marker_sphere = create_marker(1, "marker_viz", "sphere")
        marker_cylinder = create_marker(2, "marker_viz", "cylinder")
        marker_arrow = create_marker(3, "marker_viz", "arrow")
        marker_line_list = create_line(0, "line_viz", "list")
        marker_line_strip = create_line(1, "line_viz", "strip")
        rate.sleep()
        marker_pub.publish(marker_cube)
        marker_pub.publish(marker_sphere)
        marker_pub.publish(marker_arrow)
        marker_pub.publish(marker_cylinder)
        marker_pub.publish(marker_line_strip)
        marker_pub.publish(marker_line_list)
        break
