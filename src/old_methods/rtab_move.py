#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist, PoseStamped

class test_node:
    def __init__(self):
        rospy.init_node('test_node')
        self.goal_pub = rospy.Publisher("/rtabmap/goal", PoseStamped, queue_size=1)
        self.move_to(14.0, 0.0)

    def move_to(self, x, y):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"

        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = 0.1
        goal.pose.orientation.w = 0.1

        print(f"INIT: {self.goal_pub.get_num_connections()}")
        while self.goal_pub.get_num_connections() <= 0:
            pass

        self.goal_pub.publish(goal)

if __name__ == '__main__':
    test_node()
    # try:
    #     rospy.spin()
    # except KeyboardInterrupt:
    #     rospy.on_shutdown(lambda shutdown: print("Shutting Down!"))
    # pub = rospy.Publisher("/rtabmap/goal", PoseStamped, queue_size=1)
    # rospy.init_node('test_node')
    # r = rospy.Rate(10) # 10hz
    # while not rospy.is_shutdown():
    #     goal = PoseStamped()
    #     goal.header.stamp = rospy.Time.now()
    #     goal.header.frame_id = "map"

    #     goal.pose.position.x = 14.0
    #     goal.pose.position.y = 0.0
    #     goal.pose.orientation.z = 0.1
    #     goal.pose.orientation.w = 0.1

    #     pub.publish(goal)
    #     r.sleep()