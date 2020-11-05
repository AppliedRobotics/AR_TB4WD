#!/usr/bin/env python3  

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseActionGoal
from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseActionFeedback
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32MultiArray, Int16MultiArray, Int16, Bool
from sensor_msgs.msg import LaserScan
from aruco_msgs.msg import Marker, MarkerArray
from nav_msgs.srv import GetMap, GetPlan
import tf
from math import pi, cos, sin, sqrt, atan2
import numpy as np
import json
import os
from move import LineController
class Mobile_Robot_Machine:
	def __init__(self):
		self.client_move_base = actionlib.SimpleActionClient('move_base',MoveBaseAction)
		self.client_move_base.wait_for_server()
		print("succeded connect to move base")
		self.points_coordinates = self.get_points_coordinates()
		print("get points coordinates")
		print(self.points_coordinates)
		self.state = 0 # 0 somewhere, 1-6 parked on the zone
		self.status_msg = Int16()
		self.status_pub = command_pub = rospy.Publisher("status", Int16, queue_size = 1) 
		self.sub_command = rospy.Subscriber("command",Int16MultiArray, self.command_cb)
		print("complete setup for publishers and subscribers")
		line_control = LineController()
		print("load line controller class")
	def command_cb(self,data):
		if data.data[0] == 1 and data.data[1] != self.state: # mean you can move and not already parked on the target zone
			target_point = self.points_coordinates["p_"+str(data.data[1])]
			result = self.reach_goal(target_point)
			if result == 3:
				#parking alghoritm
				self.state = data.data[1]
				self.status_msg.data = self.state
				self.status_pub.publish(self.status_msg)
			else:
				a = 1 
				#clear costmap and repeat the goal action
	def to_table(self, table):
		result = self.reach_goal(self.table_coordinates[table])
		if result == 3:
			return result
		else:
			return False

	def get_points_coordinates(self):
		dir_path = os.path.dirname(os.path.realpath(__file__))	
		with open(dir_path+"/points.json", "r") as read_file:
			data = json.load(read_file)
		points_coord = dict(zip(data["position_id"], data["point_coordinate"]))	
		return points_coord

	def reach_goal(self, coord): #go to target point 
		x = coord[0]
		y = coord[1]
		theta = coord[2]
		target_quat = tf.transformations.quaternion_from_euler(0, 0, theta)
		t0=rospy.Time.now()
		goal=MoveBaseGoal()
		goal.target_pose.header.stamp=t0
		goal.target_pose.header.frame_id="map"
		goal.target_pose.pose.position = Point(x, y, 0)
		goal.target_pose.pose.orientation.x = target_quat[0]
		goal.target_pose.pose.orientation.y = target_quat[1]
		goal.target_pose.pose.orientation.z = target_quat[2]
		goal.target_pose.pose.orientation.w = target_quat[3]  
		self.client_move_base.send_goal(goal)
		reached_the_goal = self.client_move_base.wait_for_result(rospy.Duration.from_sec(60))
		if not reached_the_goal:
			rospy.logwarn("I was not able to reach the goal within the\
					allocated time")
			self.client_move_base.cancel_goal()    
		else:
			print("reach the goal","\n--------")
		return self.client_move_base.get_state()


if __name__ == '__main__':
	rospy.init_node('state_machine')
	state = Mobile_Robot_Machine()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

        
