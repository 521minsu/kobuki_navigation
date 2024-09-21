#!/usr/bin/env python
from __future__ import division
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
import numpy as np
import threading

class autonav_node():
    def __init__(self):
        self.min_distance = 0.6
        self.new_obstacle = False

        rospy.init_node("autonav_node")
        self.cv_bridge = CvBridge()
        self.pub = rospy.Publisher("cmd_vel_mux/input/teleop", Twist, queue_size=10)
        self.straight = Twist()
        self.straight.linear.x = 0.2
        self.rotate_vel = Twist()
        self.rotate_vel.angular.z = -1.0
        self.pub.publish(self.straight)
        self.sub = rospy.Subscriber("camera/depth/image_raw", Image, self.process_depth_image)
        # rospy.sleep(0.5)
        rospy.spin()
    
    def process_depth_image(self, msg):
        im_orig = self.cv_bridge.imgmsg_to_cv2(msg)
        im = im_orig.copy()
        im = np.where(np.isnan(im), 999, im)
        # im[im.isnan()] = 999
        min_point = im[im.nonzero()].min()
        print(min_point)
        if min_point < self.min_distance:
            # if not self.avoiding:
            threading.Thread(target=self.avoid_obstacle).start()
        else:
            self.new_obstacle = True
            self.pub.publish(self.straight)
    
    def avoid_obstacle(self):
        print("AVOIDING")
        self.pub.publish(self.rotate_vel)
        if self.new_obstacle:
            if self.rotate_vel.angular.z == -1.0:
                self.rotate_vel.angular.z = 1.0
            else:
                self.rotate_vel.angular.z = -1.0
        # while self.avoiding:
        rospy.sleep(1/20)
        self.pub.publish(self.straight)
        print("DONE AVOIDING")
        self.new_obstacle = False

if __name__ == '__main__':
    autonav = autonav_node()