#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def publish_marker():
    rospy.init_node('marker_publisher', anonymous=True)
    marker_pub = rospy.Publisher('/goal/marker', Marker, queue_size=1)

    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()

        marker.ns = "goal_ns"
        marker.id = 0

        marker.type = Marker.CUBE  # Other types include ARROW, CUBE, etc.
        marker.action = Marker.ADD

        marker.pose.position.x = 1.0
        marker.pose.position.y = 1.0
        marker.pose.position.z = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.a = 1.0  # Don't forget to set the alpha!
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        marker_pub.publish(marker)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_marker()
    except rospy.ROSInterruptException:
        pass
