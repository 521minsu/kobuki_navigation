#!/usr/bin/env python

import rospy
import rosnode

node_name = 'explore'
try:
    rosnode.kill_nodes([node_name])
    rospy.loginfo(f"Node {node_name} has been terminated.")
except rosnode.ROSNodeIOException as e:
    rospy.logerr(f"Failed to terminate node {node_name}: {e}")
