#!/usr/bin/env python

import math
import rospy, rosnode
import actionlib
import threading

from kobuki_navigation.srv import *

from geometry_msgs.msg import Twist, PoseStamped
from visualization_msgs.msg import Marker
from kobuki_navigation.msg import image_coord
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


# Callbacks definition
def active_cb():
    rospy.loginfo("Goal pose being processed")

def feedback_cb(feedback):
    # rospy.loginfo("Current location: "+str(feedback))
    pass

def done_cb(status, result):
    if status == 3:
        rospy.loginfo("Goal Reached")
    if status == 2 or status == 8:
        rospy.loginfo("Goal Cancelled")
    if status == 4:
        rospy.loginfo("Goal Aborted")


class manoeuvre_node:
    def __init__(self):
        print("Manoeuvre Python Node Initialised!")
        rospy.init_node('manoeuvre_node')
        self.goal_detected = False
        self.goal_evaluated = False
        self.returned = False

        self.move_pub = rospy.Publisher("cmd_vel_mux/input/teleop", Twist, queue_size=10)

        self.navclient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.navclient.wait_for_server()

        rospy.wait_for_service('goal_req')
    
    def work(self):
        self.goal_sub = rospy.Subscriber("/goal/raw", image_coord, self.goal_fn)
        # self.goal_coord_sub = rospy.Subscriber("/goal/marker", Marker, self.return_process)
        # if not self.goal_evaluated:
        #     self.goal_sub = rospy.Subscriber("/goal/raw", image_coord, self.goal_fn)
        # if not self.returned:
        #     self.goal_coord_sub = rospy.Subscriber("/goal/marker", Marker, self.return_process)
        #     self.returned = True
        
    
    def move_to(self, x, y):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0


        self.navclient.send_goal(goal, done_cb, active_cb, feedback_cb)
    
    def goal_fn(self, data):
        if ( (not self.goal_detected) and (not math.isnan(data.dist)) and (data.size > 25)):
            # New Goal detected!
            print(f"Goal Detected! dist:{data.dist:.2f}, pose:{data.image_x}, {data.image_y}")
            rosnode.kill_nodes(['explore'])
            self.navclient.cancel_all_goals()
            self.goal_detected = True
        

        # print("GOAL_DATA")
        cmd_vel = Twist()
        if not self.goal_evaluated and self.goal_detected:
            if 325 > data.image_x > 315:
                # self.move_pub.unregister()
                self.move_pub.publish(cmd_vel)
                # self.move_pub.unregister()
                # self.goal_coord_sub = rospy.Subscriber("/goal/marker", Marker, self.return_process)
                print(f"RETURNING! dist:{data.dist:.2f}, pose:{data.image_x}, {data.image_y}")
                # sleep(1)
                # self.move_to(0.0, 0.0)
                # threading.Thread(target=self.return_process).start()

                try:
                    client = rospy.ServiceProxy('goal_req', goal_req)
                    resp = client(True)
                    print(f"Goal Located! ({resp.goal_x}, {resp.goal_y}) | Raw: ({resp.pos_x}, {resp.pos_y}), {resp.yaw}")
                    self.move_to(0.0, 0.0)
                except rospy.ServiceException as e:
                    print (f"Service call failed: {e}")

                self.goal_evaluated = True
            elif data.image_x < 315:
                cmd_vel.angular.z = 0.1
                self.move_pub.publish(cmd_vel)
            elif data.image_x > 325:
                cmd_vel.angular.z = -0.1
                self.move_pub.publish(cmd_vel)
    
    def return_process(self):
        sleep(1)
        print("RETURNING...")
        self.move_to(0.0, 0.0)
    
    
if __name__ == '__main__':
    mn = manoeuvre_node()
    # while not rospy.is_shutdown():
    mn.work()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.on_shutdown(lambda shutdown: print("Shutting Down!"))

    



