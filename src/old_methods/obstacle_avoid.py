#!/usr/bin/env python3

import rospy
import statistics, math
from geometry_msgs.msg import Twist #ros msg that deals with moving the robot
from sensor_msgs.msg import LaserScan #ros msg that gets the laser scans


class Avoider():

	def __init__(self, vel_obj, obstacle_threshold=0.65, 
				       normal_lin_vel=0.2, trans_lin_vel=-0.09, trans_ang_vel=1.00):
		'''
		:param vel_obj           : Velocity object; will contain velocity commands(data); Twist()
		:param obstacle_threshold: Objects a this distance or below are considered obstacles
		:param regional_angle    : The angle on which each region extends
		:param normal_lin_vel    : When there's no obstacles, the robot will move with this linear velocity
		:param trans_lin_vel     : After detecting an obstacle, the robot will back up (negative) 
								   while rotating to help in case it can't perform a stationary rotation
		:param trans_ang_vel 	 : The robot always rotates with the same value of angular velocity
		'''
		self.vel_obj        = vel_obj
		self.OBSTACLE_DIST  = obstacle_threshold
		self.NORMAL_LIN_VEL = normal_lin_vel
		self.TRANS_LIN_VEL  = trans_lin_vel
		self.TRANS_ANG_VEL  = trans_ang_vel
		self.dist = [-1,-1,-1]

	def indentify_regions(self, scan):
		'''
		:param scan: Scan object that contains the lidar data 
		'''
		
		ranges = [10 if math.isnan(x) else x for x in scan.ranges]
		count = len(ranges)
		# print(type(ranges[-1]))
		self.dist = [statistics.mean(ranges[2*count//3+1:]), 
			      statistics.mean(ranges[count//3+1:2*count//3]), 
				  statistics.mean(ranges[:count//3])]
		
		print(self.dist)

	def avoid(self):
		act, ang_vel = self._clearance_test()
		# If act is False, and_vel is set to 0
		self._steer(act, (act*ang_vel))
		return self.vel_obj

	def _clearance_test(self):

		goal = "front_C"
		closest = 10e6
		regional_dist = 0
		maxima = {"destination": "back_C", "distance": 10e-6}
		for region in self.Regions_Report.items():
			regional_dist = abs(self.Regions_Distances[region[0]]-self.Regions_Distances[goal])
			#if there're no obstacles in that region
			if not len(region[1]):
				#check if it's the cheapest option
				if (regional_dist < closest):
					closest = regional_dist
					maxima["distance"] = self.OBSTACLE_DIST
					maxima["destination"] = region[0]
			#check if it's the clearest option
			elif(max(region[1]) > maxima["distance"]):
				maxima["distance"] = max(region[1])
				maxima["destination"] = region[0]
		#calculate the cost to the chosen orientation
		regional_dist = self.Regions_Distances[maxima["destination"]]-self.Regions_Distances[goal]
		
		# Return whether to act or not, and the angular velocity with the appropriate sign
		return (closest != 0), (regional_dist/[abs(regional_dist) if regional_dist != 0 else 1][0])*self.TRANS_ANG_VEL

	def _steer(self, steer=False, ang_vel=0):
		'''
		:param steer  : Whether to avoid and obstacle or keep on going straigt
		:param ang_vel: The angular velocity of the robot
		'''
		if not steer:
			self.vel_obj.linear.x = self.NORMAL_LIN_VEL
		else:
			self.vel_obj.linear.x = self.TRANS_LIN_VEL
		self.vel_obj.linear.y  = 0
		self.vel_obj.linear.z  = 0
		self.vel_obj.angular.x = 0
		self.vel_obj.angular.y = 0
		self.vel_obj.angular.z = ang_vel



def main():

    vel = Twist()
    # Instanciate our avoider object
    avoider = Avoider(vel)
    # Initialize our node
    rospy.init_node("Laser_Obs_Avoid_node")
    # Subscribe to the "/scan" topic in order to read laser scans data from it
    rospy.Subscriber("/scan", LaserScan, avoider.indentify_regions)
    rospy.spin()
    #create our publisher that'll publish to the "/cmd_vel" topic
    # pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size = 1)
    # #ros will try to run this code 10 times/second
    # rate = rospy.Rate(10) #10Hz
    
    # #keep running while the ros-master isn't shutdown
    # while not rospy.is_shutdown():
    #     vel = avoider.avoid()
    #     pub.publish(vel)
    #     rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
