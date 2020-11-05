#!/usr/bin/env python3  
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32MultiArray, Int16MultiArray, Int16
from sensor_msgs.msg import LaserScan
from time import sleep, time
from aruco_msgs.msg import Marker, MarkerArray
import json
import os
class LineController:
	def __init__(self):
		self.light_data = [[],[],[],[]] #front, left, right, back		
		dir_path = os.path.dirname(os.path.realpath(__file__))	
		with open(dir_path+"/calibration.json", "r") as read_file:
			data = json.load(read_file)
		self.light_medium = [data["medium_front"],
							data["medium_left"],
							data["medium_right"],
							data["medium_back"]]
		self.goal_light = [[0,0,0,0,0,0,0],[0,0,0,0,0,0,0],[0,0,0,0,0,0,0],[0,0,0,0,0,0,0]]
		self.previous_scan_time = 0
		self.current_zone = 0 # 0 - go left, 1 - go front, 2 - go right, 3 - go back
		self.k_1 = 0.015
		self.k_2 = 0.06
		self.target_marker = 0 #target parking place
		self.old_target = 0
		self.target_point = False #parking flag
		self.start_move = 0 #start moving command
		self.go_back_to_line = 0
		self.state = 0 # 0 - waiting, 1 - leftline, 2 - leftline, 3 - rightline, 4 - backline, 5 - parkingleft, 6 - parkingright
		self.speed = 0.11
		self.start_park = True
		self.status_msg = Int16()
		self.status_msg.data = 0
		self.count = 0
		self.integral = 0 
		print("succesfully load medium values")    						
		self.ranges = []
		self.sub_front = rospy.Subscriber("ir_array",UInt32MultiArray, self.ir_cb, queue_size = 1)
		self.sub_scan = rospy.Subscriber("scan", LaserScan, self.scan_cb)
		self.sub_mark = rospy.Subscriber("markers", MarkerArray, self.mark_cb)
		self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		
	def mark_cb(self, data):
		left = self.goal_light[1][1] + self.goal_light[1][2] + self.goal_light[1][3] + self.goal_light[1][4] + self.goal_light[1][5]  
		right = self.goal_light[2][1] + self.goal_light[2][2] + self.goal_light[2][3] + self.goal_light[2][4] + self.goal_light[2][5] 
		
		for marker in data.markers:
			if marker.pose.pose.position.z < 0.7 and marker.id == self.target_marker:
				if marker.id < 3 and self.current_zone == 1 and left > 0:
					self.target_point = True
				elif right > 0 and marker.id > 2 and marker.id < 7 and self.current_zone == 3:
					self.target_point = True

	def scan_cb(self, data):
		self.ranges = data.ranges
	
	def ir_cb(self, data):
		self.light_data[0] = data.data[0:7]
		self.light_data[1] = data.data[7:14]
		self.light_data[2] = data.data[14:21]
		self.light_data[3] = data.data[21:28]
		# print(self.light_data[0])
		for j in range(0,4):
			for i in range(0,7):
				if self.light_data[j][i] > self.light_medium[j][i]:
					self.goal_light[j][i] = 0
				else:
					# print("light_data: "+str(self.light_data[j][i])+" medium: "+str(self.light_medium[j][i]))
					self.goal_light[j][i] = 1

	def from_park_left(self):
		front = self.goal_light[0][1] + self.goal_light[0][2] + self.goal_light[0][3] + self.goal_light[0][4] + self.goal_light[0][5] 
		back = self.goal_light[3][1] + self.goal_light[3][2] + self.goal_light[3][3] + self.goal_light[3][4] + self.goal_light[3][5] 
		if(front > 0 and back>0):
			self.speed_publisher(0,0,0)
			return(True)
		else:
			self.speed_publisher(0, 0.14, 0)
			return(False)		
	
	def from_park_right(self):
		front = self.goal_light[0][1] + self.goal_light[0][2] + self.goal_light[0][3] + self.goal_light[0][4] + self.goal_light[0][5] 
		back = self.goal_light[3][1] + self.goal_light[3][2] + self.goal_light[3][3] + self.goal_light[3][4] + self.goal_light[3][5] 
		if(front > 0 and back>0):
			self.speed_publisher(0,0,0)
			return(True)
		else:
			self.speed_publisher(0, -0.14, 0)
			return(False)		
	
	def to_park_left(self):
		right = self.goal_light[2][1] + self.goal_light[2][2] + self.goal_light[2][3] + self.goal_light[2][4] + self.goal_light[2][5] + self.goal_light[2][0] + self.goal_light[2][6]
		error_x_1 = 0
		error_x_2 = 0
		for i in range(0,3):
			error_x_1 += self.goal_light[1][i] - self.goal_light[1][6-i]
		for i in range(0,3):
			error_x_2 += self.goal_light[2][i] - self.goal_light[2][6-i]  
		up_x = 0.013*(error_x_1-error_x_2)
		if(right > 1):
			self.speed_publisher(0,0,0)
			return(True)
		else:
			if(self.target_marker < 2):
				self.speed_publisher(up_x-0.04, -0.16, 0)
			else:
				self.speed_publisher(up_x+0.1, -0.16, 0)
			return(False)
	
	def to_park_right(self):
		left = self.goal_light[1][1] + self.goal_light[1][2] + self.goal_light[1][3] + self.goal_light[1][4] + self.goal_light[1][5]  + self.goal_light[1][0] + self.goal_light[1][6]
		error_x_1 = 0
		error_x_2 = 0
		for i in range(0,3):
			error_x_2 += self.goal_light[2][i] - self.goal_light[2][6-i]
		for i in range(0,3):
			error_x_1 += self.goal_light[1][i] - self.goal_light[1][6-i] 
		up_x = 0.013*(error_x_1-error_x_2)
		if(left > 1):
			self.speed_publisher(0,0,0)
			return(True)
		else:
			self.speed_publisher(up_x, 0.12, 0)
			return(False)
	
	def parallel_parking_left(self):
		error_z = self.ranges[228] - self.ranges[305]
		up_z = 3.5*error_z
		error_y = self.ranges[228] + self.ranges[305] - 0.45
		up_y = -error_y
		if abs(up_y) > 0.08:
			up_y = self.sign(up_y)*0.08
		if abs(up_z) > 0.1:
			up_y = self.sign(up_z)*0.1
		error_x_1 = 0
		error_x_2 = 0
		for i in range(0,3):
			error_x_1 += self.goal_light[1][i] - self.goal_light[1][6-i]
		for i in range(0,3):
			error_x_2 += self.goal_light[2][i] - self.goal_light[2][6-i]  
		error = error_x_1-error_x_2
		self.integral+=error
		p = 0.01*error
		if(abs(p) <= 0.02):
			p = 0
			self.status_msg.data = 0
			self.status_pub.publish(self.status_msg)

		up_x = p+0.0002*self.integral
		# print("up_x: "+str(up_x))
		# print("up_x: "+str(up_x))
		# print("up_y: "+str(up_y))
		# print("up_z: "+str(up_z))
		self.speed_publisher(up_x,up_y,up_z)
		self.state = 5
	
	def parallel_parking_right(self):
		error_z = self.ranges[45] - self.ranges[125] 
		up_z = 3.5*error_z
		error_y = self.ranges[45] + self.ranges[125] - 0.48
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
		# up_x = 0.02*(error_x_1-error_x_2)
		
		error = error_x_1-error_x_2
		p = 0.01*error
		if(abs(p) <= 0.02):
			p = 0
			self.status_msg.data = 0
			self.status_pub.publish(self.status_msg)
		self.integral+=error
		up_x = p+0.0003*self.integral
		
		# print("up_x: "+str(up_x))
		# if(abs(up_x) <= 0.02):
			# up_x=0
		# print("up_x: "+str(up_x))
		# print("up_y: "+str(up_y))
		# print("up_z: "+str(up_z))
		self.speed_publisher(up_x,up_y,up_z)
		self.state = 6
	
	def go_on_line_front(self):
		error_y = 0
		error_z = 0
		for i in range(0,3):
			error_z += self.goal_light[0][i] - self.goal_light[0][6-i]
		for i in range(0,3):
			error_y += self.goal_light[3][i] - self.goal_light[3][6-i]
		up_y = self.k_1*(error_y + error_z)
		up_z = self.k_2*(error_z - error_y) 
		self.speed_publisher(self.speed, -up_y, up_z)
		self.state = 2
	def go_on_line_back(self):
		error_y = 0
		error_z = 0
		for i in range(0,3):
			error_z += self.goal_light[3][i] - self.goal_light[3][6-i]
		for i in range(0,3):
			error_y += self.goal_light[0][i] - self.goal_light[0][6-i] 
		up_y = self.k_1*(error_y+error_z)
		up_z = self.k_2*(error_z-error_y)
		self.speed_publisher(-self.speed, -up_y, -up_z)
		self.state = 4
	def go_on_line_left(self):
		error_x = 0
		error_z = 0
		for i in range(0,3):
			error_z += self.goal_light[1][i] - self.goal_light[1][6-i]
		for i in range(0,3):
			error_x += self.goal_light[2][i] - self.goal_light[2][6-i]  
		up_x = self.k_1*(error_x-error_z)
		up_z = self.k_2*(error_z+error_x)
		self.speed_publisher(-up_x, -self.speed, -up_z)
		self.state = 1
	def go_on_line_right(self):
		error_x = 0
		error_z = 0
		for i in range(0,3):
			error_z += self.goal_light[2][i] - self.goal_light[2][6-i]
		for i in range(0,3):
			error_x += self.goal_light[1][i] - self.goal_light[1][6-i] 
		up_x = self.k_1*(error_x-error_z)
		up_z = self.k_2*(error_z+error_x)
		self.speed_publisher(up_x, self.speed, -up_z)
		self.state = 3
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

# if __name__ == '__main__':
	# rospy.init_node('line_controller')
	# controller = LineController()
	# sleep(1)
	# while not rospy.is_shutdown():
		# try:
			
			# sleep(0.05)
		# except KeyboardInterrupt:
			# print("something goes wrong")
		# finally:
			# controller.speed_publisher(0,0,0)
	
