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
from std_srvs.srv import Empty
import tf
from math import pi, cos, sin, sqrt, atan2
import numpy as np
import json
import os
from time import sleep
class Mobile_Robot_Machine():
	def __init__(self):
		self.ranges = []
		self.park_flag = 0 #1 - park left, 2 - park right
		self.sub_front = rospy.Subscriber("ir_array",UInt32MultiArray, self.ir_cb, queue_size = 1)
		self.sub_scan = rospy.Subscriber("scan", LaserScan, self.scan_cb)
		self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		self.light_data = [[],[],[],[]] #front, left, right, back		
		dir_path = os.path.dirname(os.path.realpath(__file__))	
		with open(dir_path+"/calibration.json", "r") as read_file:
			data = json.load(read_file)
		self.light_medium = [data["medium_front"],
							data["medium_left"],
							data["medium_right"],
							data["medium_back"]]
		self.goal_light = [[0,0,0,0,0,0,0],[0,0,0,0,0,0,0],[0,0,0,0,0,0,0],[0,0,0,0,0,0,0]]
		
		self.client_move_base = actionlib.SimpleActionClient('move_base',MoveBaseAction)
		self.client_move_base.wait_for_server()
		self.client_clear_costmap = rospy.ServiceProxy('/move_base_node/clear_costmaps', Empty)
		print("succeded connect to move base")
		self.points_coordinates = self.get_points_coordinates()
		print("get points coordinates")
		print(self.points_coordinates)
		self.state = 0 # 0 somewhere, 1-6 parked on the zone
		self.status_msg = Int16()
		self.integral=0
		self.status_pub = command_pub = rospy.Publisher("status", Int16, queue_size = 1) 
		self.sub_command = rospy.Subscriber("command",Int16MultiArray, self.command_cb)
		print("complete setup for publishers and subscribers")
		print("load line controller class")
		self.done_parking = False
		
	def scan_cb(self, data):
		self.ranges = data.ranges
	
	def ir_cb(self, data):
		self.light_data[0] = data.data[0:7]
		self.light_data[1] = data.data[7:14]
		self.light_data[2] = data.data[14:21]
		self.light_data[3] = data.data[21:28]
		for j in range(0,4):
			for i in range(0,7):
				if self.light_data[j][i] > self.light_medium[j][i]:
					self.goal_light[j][i] = 0
				else:
					# print("light_data: "+str(self.light_data[j][i])+" medium: "+str(self.light_medium[j][i]))
					self.goal_light[j][i] = 1
		if self.park_flag == 1:
			self.parallel_parking_left()
		if self.park_flag == 2:
			self.parallel_parking_right()
	def command_cb(self,data):
		self.command_start(data.data[0], data.data[1])
	def command_start(self, enable, zone):
		if enable == 1 and zone != self.state: # mean you can move and not already parked on the target zone
			target_point = self.points_coordinates["p_"+str(zone)]
			result = self.reach_goal(target_point)
			if result == 3:
				self.parking(zone)
				self.state = zone
				self.status_msg.data = self.state
				self.status_pub.publish(self.status_msg)
			else:
				print("can't reach goal, clearing costmap")
				self.client_clear_costmap()
				self.command_start(enable, zone)

	def parking(self,zone):
		if zone == 1 or zone == 2:
			self.park_flag = 1
			while self.done_parking == False:
				sleep(0.1)
			print("parking confirmed")
			self.done_parking = False
			self.park_flag = 0
			self.speed_publisher(0,0,0)
		elif zone > 0 and zone < 7:
			self.park_flag = 2
			while self.done_parking == False:
				sleep(0.1)
			self.done_parking = False
			print("parking confirmed")
			self.park_flag = 0
			self.speed_publisher(0,0,0)
			
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
	
	def parallel_parking_left(self):
		error_z = self.ranges[235] - self.ranges[300]
		up_z = 3.5*error_z
		error_y = self.ranges[235] + self.ranges[300] - 0.45
		up_y = -error_y
		if abs(up_y) > 0.08:
			up_y = self.sign(up_y)*0.08
		if abs(up_z) > 0.1:
			up_z = self.sign(up_z)*0.1		
		# print("235:"+str(self.ranges[235]))
		# print("300:"+str(self.ranges[300]))
		error_x_1 = 0
		error_x_2 = 0
		for i in range(0,3):
			error_x_1 += self.goal_light[1][i] - self.goal_light[1][6-i]
		for i in range(0,3):
			error_x_2 += self.goal_light[2][i] - self.goal_light[2][6-i]  
		error = error_x_1-error_x_2
		self.integral+=error
		p = 0.01*error
		up_x = p+0.0002*self.integral
		if abs(up_x) <= 0.02 and abs(up_y) <= 0.02 and abs(up_z) <= 0.02:
			self.done_parking = True

		# print("up_x: "+str(up_x))
		# print("up_y: "+str(up_y))
		# print("up_z: "+str(up_z))
		self.speed_publisher(up_x,up_y,up_z)

	def parallel_parking_right(self):
		error_z = self.ranges[46] - self.ranges[125] 
		up_z = 3.5*error_z
		error_y = self.ranges[46] + self.ranges[125] - 0.48
		# print("44: "+str(self.ranges[46]))
		# print("125: "+str(self.ranges[125]))
		up_y = error_y
		if abs(up_y) > 0.05:
			up_y = self.sign(up_y)*0.05
		if abs(up_z) > 0.07:
			up_z = self.sign(up_z)*0.05
		error_x_1 = 0
		error_x_2 = 0
		for i in range(0,3):
			error_x_2 += self.goal_light[2][i] - self.goal_light[2][6-i]
		for i in range(0,3):
			error_x_1 += self.goal_light[1][i] - self.goal_light[1][6-i] 
		error = error_x_1-error_x_2
		p = 0.01*error
		self.integral+=error
		up_x = p+0.0003*self.integral
		if abs(up_x) <= 0.02 and abs(up_y) <= 0.02 and abs(up_z) <= 0.02:
			self.done_parking = True
		self.speed_publisher(up_x,up_y,up_z)

	def sign(self, data):
		if data > 0:
			return 1
		else:
			return -1
	def speed_publisher(self,x,y,z):
		msg = Twist()
		msg.linear.x = x
		msg.linear.y = -y
		msg.angular.z = z 
		self.cmd_pub.publish(msg)


if __name__ == '__main__':
	rospy.init_node('state_machine')
	state = Mobile_Robot_Machine()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

        
