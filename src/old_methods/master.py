#!/usr/bin/env python

import rospy

from visualization_msgs.msg import Marker

from helper_functions.goal_eval        import goal_eval_node         as ge_node
from helper_functions.manoeuvre        import manoeuvre_node           as mn_node
from helper_functions.image_processing import image_processing_node    as ip_node


class master_node:
    def __init__(self):
        rospy.init_node('master_node')
        rospy.loginfo("Master Python Node Initialised?")

        self.goal_evaluated = False

        self.ip = ip_node()
        self.mn = mn_node()
        self.ge = ge_node()


    def goal_cb(self, data):
        return data



if __name__ == '__main__':
    master = master_node()
    rospy.loginfo("Master Python Node Initialised!")

    goal_requested = False

    while not rospy.is_shutdown():
        master.mn.work()   # Subscribes to /goal/raw to check if there's a goal object in sight
        goal_sub = rospy.Subscriber("/goal/marker", Marker, master.goal_cb)

        if master.mn.returned and not goal_requested:
            print("Master: Requesting Goal!")
            master.ge.eval_goal()
            goal_requested = True

        if master.goal_evaluated:
            print("Master: Goal evaluated... moving back to starting point!")
            master.mn.move_to(0.0, 0.0)
