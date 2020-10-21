#!/usr/bin/env python3  
from __future__ import division
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32MultiArray
from time import sleep
import json

class LineController:
	def __init__(self):
		self.front_light = [[],[],[],[],[],[],[]]
		self.left_light = [[],[],[],[],[],[],[]]
		self.right_light = [[],[],[],[],[],[],[]]
		self.back_light = [[],[],[],[],[],[],[]]
		self.min_light = [[],[],[],[]] #front, left, right, back
		self.max_light = [[],[],[],[]]
		self.medium_light = [[],[],[],[]]
		self.start_measures = True

		self.sub_front = rospy.Subscriber("ir_array",UInt32MultiArray, self.ir_cb)
		self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		sleep(1)
		print("ready for calibration")

	def calibrate(self):
		print("start calibration")
		for i in range(0,2):
			for i in range(0,35):
				self.speed_publisher(0,-0.09,0)
				sleep(0.1)
			for i in range(0,35):
				self.speed_publisher(0,0.09,0)
				sleep(0.1)
		
		self.speed_publisher(0,0,0)
		for i in range(0,7):
			self.min_light[0].append(min(self.front_light[i]))
		for i in range(0,7):
			self.min_light[1].append(min(self.left_light[i]))
		for i in range(0,7):
			self.min_light[2].append(min(self.right_light[i]))
		for i in range(0,7):
			self.min_light[3].append(min(self.back_light[i]))
		for i in range(0,7):
			self.max_light[0].append(max(self.front_light[i]))
		for i in range(0,7):
			self.max_light[1].append(max(self.left_light[i]))
		for i in range(0,7):
			self.max_light[2].append(max(self.right_light[i]))
		for i in range(0,7):
			self.max_light[3].append(max(self.back_light[i]))
		print("min light: "+str(self.min_light))
		print("max light: "+str(self.max_light))
		for i in range(0,4):
			for j in range(0,7):
				self.medium_light[i].append((self.min_light[i][j] + self.max_light[i][j])/2)
		print("medium: "+str(self.medium_light))
		data = {"medium_front":self.medium_light[0],
				"medium_left":self.medium_light[1],
				"medium_right":self.medium_light[2],
				"medium_back":self.medium_light[3]}
		with open("/home/ubuntu/ros_package_rp_worldskills/src/worldskills/src/calibration.json","w") as file:
			json.dump(data, file)
	def ir_cb(self, data):
		data.data = self.clean_data(data.data)
		if self.start_measures == True:
			for i in range(0,7):
				self.front_light[i].append(data.data[i])
			for i in range(7,14):
				self.left_light[i-7].append(data.data[i])
			for i in range(14,21):
				self.right_light[i-14].append(data.data[i])
			for i in range(21,28):
				self.back_light[i-21].append(data.data[i])
	def clean_data(self, array):
		data = []
		for i in range(0,28):

			if array[i] < 10 or array[i] > 1000:
				data.append(300)
			else:
				data.append(array[i])
		return data

	def speed_publisher(self,x,y,z):
		msg = Twist()
		msg.linear.x = x
		msg.linear.y = y
		msg.angular.z = z 
		for i in range(0,5):
			self.cmd_pub.publish(msg)
			sleep(0.01)
	

if __name__ == '__main__':
	rospy.init_node('line_controller')
	controller = LineController()
	print("enter s to start calibration")
	value = input()
	if(value == "s"):
		controller.calibrate()
		print("calibration done")
	else:
		print("wrong key")
	