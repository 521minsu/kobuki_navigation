#!/usr/bin/env python
import rospy
import math

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

roll = pitch = yaw = 0.0
x = y = z = 0.0

def get_rotation(msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    yaw_degrees = yaw * 180.0 / math.pi
    if yaw_degrees < 0:
        yaw_degrees += 360

    
    global x, y, z
    position = msg.pose.pose.position
    x, y, z = position.x, position.y, position.z


    print(f"POSE: ({x:.2f}, {y:.2f}) || ANGLE RAW: {yaw:.3f} | DEG: {yaw_degrees:1.0f}")



rospy.init_node('rotation_test')
sub = rospy.Subscriber('/odom', Odometry, get_rotation)
r = rospy.Rate(1)
while not rospy.is_shutdown():
    r.sleep()