#!/usr/bin/env python

import sys

import rospy
import cv2
import numpy as np
import math

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from kobuki_navigation.msg import image_coord


FONT = cv2.FONT_HERSHEY_DUPLEX
BLUE = (255, 0, 0)
GREEN = (0, 255, 0)
RED = (0, 0, 255)
FILTER_RATIO = 0.85


def get_contours(img, min_area, is_simple=False):
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    result = []

    # 경계선 개수만큼 반복
    for cnt in contours:
        # 경계선의 너비가 최소 영역 이상일 때만 result 배열에 추가
        if cv2.contourArea(cnt) > min_area:
            result.append(cnt)

    return result


# 꼭짓점을 그리는 함수
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

def nothing(x):
    pass

class image_converter:

    def __init__(self):
        rospy.init_node('goal_node', anonymous=False)
        #--- Publisher of the edited frame
        self.image_pub = rospy.Publisher("/camera/rgb/image_proc",Image,queue_size=1)
        self.pot_goal_pub = rospy.Publisher("/goal/potential", image_coord, queue_size=1)
        self.goal_pub = rospy.Publisher("/goal", image_coord, queue_size=1)

        #--- Initialise the custom message
        self.goal_coord = image_coord()
        self.goal_pot_coord = image_coord()
        self.goal_coord_prev = image_coord()
        self.goal_pot_coord_prev = image_coord()

        self.goal_stabiliser = 0
        self.pot_goal_stabiliser = 0

        #--- Subscriber to the camera flow
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)


        # cv2.namedWindow('Image')

        # # Create trackbars for color change
        # cv2.createTrackbar('LowerH', 'Image', 0, 179, nothing)
        # cv2.createTrackbar('LowerS', 'Image', 0, 255, nothing)
        # cv2.createTrackbar('LowerV', 'Image', 0, 255, nothing)
        # cv2.createTrackbar('UpperH', 'Image', 179, 179, nothing)
        # cv2.createTrackbar('UpperS', 'Image', 255, 255, nothing)
        # cv2.createTrackbar('UpperV', 'Image', 50, 255, nothing)

    def callback(self,data):  #--- Callback function


    
        #--- Read the frame and convert it using bridge
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



            # # Get current positions of the trackbars
            # lh = cv2.getTrackbarPos('LowerH', 'Image')
            # ls = cv2.getTrackbarPos('LowerS', 'Image')
            # lv = cv2.getTrackbarPos('LowerV', 'Image')
            # uh = cv2.getTrackbarPos('UpperH', 'Image')
            # us = cv2.getTrackbarPos('UpperS', 'Image')
            # uv = cv2.getTrackbarPos('UpperV', 'Image')

            # # Convert to HSV color space
            # hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # # Define the lower and upper bounds of the black color in HSV
            # lower_black = np.array([lh, ls, lv])
            # upper_black = np.array([uh, us, uv])

            # # Create a mask for black color
            # black_mask = cv2.inRange(hsv, lower_black, upper_black)

            # # Bitwise-AND mask and original image
            # # result = cv2.bitwise_and(cv_image, cv_image, mask=black_mask)

            # # Display the original image and the result
            # cv2.imshow('Image', black_mask)
            # cv2.waitKey(3)

            proc_img = cv_image.copy()
            hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_img, lower_range, upper_range)


            red_contours = get_contours(mask, 50, False)

            for cnt in red_contours:
                cnt_length = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.1 * cnt_length, True) # Corner Coords
                diff = narrowness(approx)

                # finding center point of shape 
                M = cv2.moments(cnt) 
                if M['m00'] != 0.0: 
                    x = int(M['m10']/M['m00']) 
                    y = int(M['m01']/M['m00'])
                
                if diff[0] > 30:
                    self.goal_coord.size = diff[0]
                    self.goal_coord.image_x = x
                    self.goal_coord.image_y = y
                
                print(f"Goal? {diff} at ({x}, {y})")
                

                cv2.putText(proc_img, str(diff), (x,y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2) # Show Difference
            
        # print('1',self.goal_coord.size, self.goal_coord_prev.size)
        

        # if self.goal_pot_coord != self.goal_pot_coord_prev :
        #     self.pot_goal_pub.publish(self.goal_pot_coord)
        #     self.goal_pot_coord_prev.size = self.goal_pot_coord.size
        #     self.goal_pot_coord_prev.image_x = self.goal_pot_coord.image_x
        #     self.goal_pot_coord_prev.image_y = self.goal_pot_coord.image_y
        if self.goal_coord != self.goal_coord_prev :
            # print(self.goal_coord.size, self.goal_coord_prev.size)
            self.goal_pub.publish(self.goal_coord)
            self.goal_coord_prev.size = self.goal_coord.size
            self.goal_coord_prev.image_x = self.goal_coord.image_x
            self.goal_coord_prev.image_y = self.goal_coord.image_y



        #     hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        #     mask = cv2.inRange(hsv_img, lower_range, upper_range)
        #     mask_black = cv2.inRange(hsv_img, (0,0,0), (180,255,30))
        #     mask[0:240, 0:640] = 0

        #     proc_img = cv_image.copy()

        #     #new frame, array for black coords, then compare with potential red coords one by one 
        #     black_coords = []

        #     ignore = False


        #     # 경계선 가져오기
        #     red_contours = get_contours(mask, 50, False)
        #     black_contours = get_contours(mask_black, 20, False)

        #     for cnt in black_contours:
        #         # finding center point of shape 
        #         M = cv2.moments(cnt) 
        #         if M['m00'] != 0.0: 
        #             x = int(M['m10']/M['m00']) 
        #             y = int(M['m01']/M['m00']) 
        #         cv2.putText(proc_img, 
        #                             str((x,y)), (x,y), 
        #                             FONT, 0.8, RED)
        #         black_coords.append((x, y))

            
        #     for cnt in red_contours:
        #         # finding center point of shape 
        #         M = cv2.moments(cnt) 
        #         if M['m00'] != 0.0: 
        #             x = int(M['m10']/M['m00']) 
        #             y = int(M['m01']/M['m00'])
                
        #         for coord in black_coords:
        #             if abs(x - coord[0]) <= 5 and y < coord[1]: # Cone Detected
        #                 print(f"Red: ({x}, {y}), Black: ({coord[0]}, {coord[1]})")
        #                 ignore = True
        #             elif (x < 40 or x > 600) and y < coord[1]:
        #                 print(f"EDGE! Red: ({x}, {y}), Black: ({coord[0]}, {coord[1]})")
        #                 ignore = True
                        
        #         cnt_length = cv2.arcLength(cnt, True)
        #         approx = cv2.approxPolyDP(cnt, 0.1 * cnt_length, True) # Corner Coords
        #         diff = narrowness(approx)
        #         if ((diff[0] - diff[1]) >= -1 and not ignore): # Goal Detected
        #             if (diff[0] > 50 and diff[1] > 50):
        #                 cv2.drawContours(proc_img, cnt, -1, BLUE, 5)
        #                 draw_points(proc_img, cnt, 0.1, RED)
        #                 # Text Alert
        #                 cv2.putText(proc_img, 
        #                             "Goal Detected! " + str(diff) + " at " + str((x,y)), (0, 25), 
        #                             FONT, 0.8, RED)
        #                 self.goal_coord.size = diff[0]
        #                 self.goal_coord.image_x = x
        #                 self.goal_coord.image_y = y
        #             elif (((diff[0] + diff[1]) / 2) > 10):
        #                 print(f"{diff[0]}, {diff[1]}")
        #                 self.goal_pot_coord.size = diff[0]
        #                 self.goal_pot_coord.image_x = x
        #                 self.goal_pot_coord.image_y = y
                    
                    
        #             print(f"Goal? ({diff[0]}, {diff[1]}), {x}, {y}, {black_coords}")
                

        #         cv2.putText(proc_img, str(diff), (x,y), 
        #                     cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2) # Show Difference
            
        # # print('1',self.goal_coord.size, self.goal_coord_prev.size)
        

        # if self.goal_pot_coord != self.goal_pot_coord_prev :
        #     self.pot_goal_pub.publish(self.goal_pot_coord)
        #     self.goal_pot_coord_prev.size = self.goal_pot_coord.size
        #     self.goal_pot_coord_prev.image_x = self.goal_pot_coord.image_x
        #     self.goal_pot_coord_prev.image_y = self.goal_pot_coord.image_y
        # if self.goal_coord != self.goal_coord_prev :
        #     # print(self.goal_coord.size, self.goal_coord_prev.size)
        #     self.goal_pub.publish(self.goal_coord)
        #     self.goal_coord_prev.size = self.goal_coord.size
        #     self.goal_coord_prev.image_x = self.goal_coord.image_x
        #     self.goal_coord_prev.image_y = self.goal_coord.image_y
            



        #--- Debug ---#
        # cv2.imshow("Raw Image", cv_image)
        # cv2.waitKey(3)
        cv2.imshow("Raw Mask", mask)
        cv2.waitKey(3)
        cv2.imshow("Processed Image", proc_img)
        cv2.waitKey(3)

        #--- Publish the modified frame to a new topic
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(proc_img, "bgr8"))
        except CvBridgeError as e:
            print(e)

#--------------- MAIN LOOP
def main(args):
    #--- Create the object from the class we defined before
    ic = image_converter()
    
    #--- Initialize the ROS node
    # rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        
    #--- In the end remember to close all cv windows
    cv2.destroyAllWindows()

if __name__ == '__main__':
        main(sys.argv)