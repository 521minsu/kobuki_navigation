#!/usr/bin/env python

import rospy
import math

from kobuki_navigation.srv import goal_req, goal_reqResponse

from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from kobuki_navigation.msg import image_coord

from tf.transformations import euler_from_quaternion


class goal_eval_node():
    def __init__(self):
        print("Goal Eval Python Node Initialised!")
        rospy.init_node("goal_eval_node")

        self.image_x = 0
        self.image_y = 0
        self.prev_image_x = 0
        self.prev_image_y = 0
        self.size = 0
        self.dist = 0.0

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw = 0.0

        self.published = False

        self.goal_coord_pub = rospy.Publisher("/goal/marker", Marker, queue_size=1)
        while self.goal_coord_pub.get_num_connections() <= 0:
            pass
        print(self.goal_coord_pub.get_num_connections())
        # self.coord_sub = rospy.Subscriber("/goal/raw", image_coord, self.get_coord)

    def eval_goal(self, data):
        # print("GOAL_EVAL STARTED!")
        self.coord_sub = rospy.Subscriber("/goal/raw", image_coord, self.get_coord)
        resp = goal_reqResponse()
        resp.goal_x, resp.goal_y = self.goal_x, self.goal_y
        resp.pos_x, resp.pos_y, resp.yaw = self.pos_x, self.pos_y, self.yaw
        return resp


    def get_coord(self, data):
        self.dist = data.dist
        self.size = data.size
        self.image_x = data.image_x
        self.image_y = data.image_y

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.get_odom) 
    

    def get_odom(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        position = msg.pose.pose.position

        self.locate_goal(position.x, position.y, yaw)
    
    def locate_goal(self, x, y, yaw):
        self.goal_x, self.goal_y = x + self.dist * math.cos(yaw), y + self.dist * math.sin(yaw)
        self.pos_x, self.pos_y, self.yaw = x, y, yaw

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()

        marker.ns = f"({self.goal_x:.2f}, {self.goal_y:.2f})"
        marker.id = 0

        marker.type = Marker.CUBE  # Other types include ARROW, CUBE, etc.
        marker.action = Marker.ADD

        marker.pose.position.x = self.goal_x
        marker.pose.position.y = self.goal_y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.a = 1.0  # Don't forget to set the alpha!
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # if (325 > self.image_x > 315) and self.size > 20 and (self.image_x != self.prev_image_x and self.image_y != self.prev_image_y) and not self.published:
        if not self.published:
            # rospy.Debug(f"GOAL: ({marker.pose.position.linear.x}, {marker.pose.position.linear.y}) [({x}, {y}), {yaw:.3f}] Published!")
            # rospy.loginfo(f"GOAL: ({marker.pose.position.linear.x}, {marker.pose.position.linear.y}) [({x}, {y}), {yaw:.3f}] Published!")
            print(f"GOAL: ({marker.pose.position.x}, {marker.pose.position.y}) [({x}, {y}), {yaw:.3f}, {self.dist}] Published!")

            self.goal_x, self.goal_y = marker.pose.position.x, marker.pose.position.y
            self.pos_x, self.pos_y, self.yaw = x, y, yaw

            self.goal_coord_pub.publish(marker)
            self.prev_image_x, self.prev_image_y = self.image_x, self.image_y
            self.published = True
        
        # print(f"GOAL?? ({self.goal_x}, {self.goal_y}) [({x}, {y}), {yaw:.3f}, {self.dist}] Published!")
        # print(f"GOAL: ({self.goal_x:.2f}, {self.goal_y:.2f}) | POSE: ({self.pos_x:.2f}, {self.pos_y:.2f}) | ANG: {self.yaw:.3f} | DEP: {self.depth:.3f}")




if __name__ == '__main__':
    goal_eval = goal_eval_node()
    s = rospy.Service('goal_req', goal_req, goal_eval.eval_goal)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.on_shutdown(lambda shutdown: print("Shutting Down!"))