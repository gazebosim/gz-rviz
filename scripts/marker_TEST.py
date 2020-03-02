import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


def create_line(marker_id, ns, marker_type):
    marker = Marker()

    marker.header.stamp = rospy.Time().now()
    marker.ns = ns
    marker.id = marker_id

    if marker_type == "strip":
        marker.type = marker.LINE_STRIP
    else:
        marker.type = marker.LINE_LIST

    for i in range(12):
        p = Point()
        p.x = i
        p.y = -i
        p.z = i
        marker.points.append(p)

    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1

    marker.color.r = 1
    marker.color.g = 1
    marker.color.b = 0
    marker.color.a = 1

    return marker


def create_marker(marker_id, ns, marker_type):
    marker = Marker()

    marker.header.stamp = rospy.Time().now()
    marker.ns = ns
    marker.id = marker_id

    if marker_type == "cylinder":
        marker.type = Marker.CYLINDER
    elif marker_type == "cube":
        marker.type = Marker.CUBE
    else:
        marker.type = Marker.SPHERE

    marker.pose.position.x = 4
    marker.pose.position.y = 4
    marker.pose.position.z = 4

    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1

    marker.color.r = 1
    marker.color.g = 0
    marker.color.b = 1
    marker.color.a = 1

    return marker


if __name__ == "__main__":
    rospy.init_node("marker_test_node")
    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
    rate = rospy.Rate(1)
    count = 0
    while not rospy.is_shutdown():
        marker = create_marker(0, "marker_viz", "sphere")
        marker_line = create_line(0, "line_viz", "list")
        rate.sleep()
        marker_pub.publish(marker)
        marker_pub.publish(marker_line)
        break
