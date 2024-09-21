#! /usr/bin/env python3

import rospy
from kobuki_navigation.msg import image_coord

class goal_sub_node():
    def __init__(self):
        rospy.init_node('goal_sub_node', anonymous=False)
        rospy.Subscriber('/goal', image_coord, self.callback)
        rospy.spin()

        self.size = 0
        self.image_x = 0
        self.image_y = 0
    
    def callback(self, data):
        self.size = data.size
        self.image_x = data.image_x
        self.image_y = data.image_y
        print('callback', self.size, self.image_x, self.image_y)
    

if __name__ == '__main__':
    print('started')
    gs = goal_sub_node()
    print('gs')
    print(gs.is_detected())
    print(gs.info())