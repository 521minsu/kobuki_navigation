#!/usr/bin/env python

import sys

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from kobuki_navigation.msg import image_coord

from cv_bridge import CvBridge, CvBridgeError



FONT = cv2.FONT_HERSHEY_DUPLEX
BLUE = (255, 0, 0)
GREEN = (0, 255, 0)
RED = (0, 0, 255)
FILTER_RATIO = 0.85


def get_contours(img, min_area, is_simple=False):
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    result = []

    for cnt in contours:
        if cv2.contourArea(cnt) > min_area:
            result.append(cnt)

    return result


def draw_points(img, cnt, epsilon, color):
    cnt_length = cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, epsilon * cnt_length, True)
    
    for point in approx:
        cv2.circle(img, (point[0][0], point[0][1]), 3, color, -1)


def narrowness(points):
    points = np.squeeze(np.array(points))
    sorted_points = points[points[:,1].argsort()]
    
    upper_points = sorted_points[:2]
    lower_points = sorted_points[-2:]
    
    upper_width = abs(upper_points[0][0] - upper_points[1][0])
    lower_width = abs(lower_points[0][0] - lower_points[1][0])
    
    return [upper_width, lower_width]


class image_processing_node:
    def __init__(self):
        print("Image Processing Python Node Initialised!")
        rospy.init_node('image_processing_node', anonymous=False)
        #--- Publisher of the edited frame
        self.goal_pub = rospy.Publisher("/goal/raw", image_coord, queue_size=1)
        self.image_pub = rospy.Publisher("/camera/rgb/image_proc",Image,queue_size=10)

        #--- Initialise the custom message
        self.goal_coord = image_coord()
        self.goal_coord_prev = image_coord()

        self.goal_dist = 0.0
        self.refreshed = False

        #--- Subscriber to the camera flow
        self.bridge = CvBridge()
        self.rgb_sub()


    def rgb_sub(self):
        self.rgb_image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.rgb_process)
    
    def dep_sub(self):
        self.dep_image_sub = rospy.Subscriber("/camera/depth/image_raw",Image,self.dep_process)


    def rgb_process(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        #--- If a valid frame is received
        (rows,cols,channels) = cv_image.shape
        if cols > 20 and rows > 20:
            cv_image = cv2.resize(cv_image, (640, 480), interpolation=cv2.INTER_LINEAR)

            lower_range = (0, 255, 50)
            upper_range = (10, 255, 255)

            proc_img = cv_image.copy()
            hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_img, lower_range, upper_range)

            red_contours = get_contours(mask, 50, False)

            for cnt in red_contours:
                cnt_length = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.1 * cnt_length, True)
                for point in approx:
                    cv2.circle(proc_img, (point[0][0], point[0][1]), 3, GREEN, -1)
                
                diff = narrowness(approx)

                # finding center point of shape 
                M = cv2.moments(cnt) 
                if M['m00'] != 0.0: 
                    x = int(M['m10']/M['m00']) 
                    y = int(M['m01']/M['m00'])
                
                if diff[0] > 10:
                    self.goal_coord.image_x = x
                    self.goal_coord.image_y = y
                    self.goal_coord.size = diff[0]
                    self.refreshed = True
                else:
                    self.refreshed = False

                
                # print(f"{diff} at ({x}, {y}) / LENGTH: {cnt_length}")
            
                cv2.putText(proc_img, str(diff), (x,y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2) # Show Difference
        self.dep_sub()

        # self.goal_coord.dist = self.goal_dist
            


        #--- Debug ---#
        # cv2.imshow("Raw Mask", mask)
        # cv2.waitKey(3)
        # cv2.imshow("Processed Image", proc_img)
        # cv2.waitKey(3)

        #--- Publish the modified frame to a new topic
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(proc_img, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def dep_process(self, image):
        cv_image = self.bridge.imgmsg_to_cv2(image)
        if self.refreshed:
            self.goal_coord.dist = cv_image[self.goal_coord.image_y, self.goal_coord.image_x]

        if self.goal_coord != self.goal_coord_prev:
            self.goal_pub.publish(self.goal_coord)
            self.goal_coord_prev.dist = self.goal_coord.dist
            self.goal_coord_prev.image_x = self.goal_coord.image_x
            self.goal_coord_prev.image_y = self.goal_coord.image_y
            self.goal_coord_prev.size = self.goal_coord.size



#--------------- MAIN LOOP
def main(args):
    ip = image_processing_node()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.on_shutdown(lambda shutdown: print("Shutting Down!"))
        
    cv2.destroyAllWindows()

if __name__ == '__main__':
        main(sys.argv)