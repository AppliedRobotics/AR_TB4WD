#!/usr/bin/env python3  

import rospy
import serial
import struct
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Header
from time import sleep, time
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32MultiArray, Float32
class SerialControl():
	def __init__(self, num_of_wheels):
		self.ser = serial.Serial('/dev/ttyACM0', 57600 ,timeout=1.0)
		self.line = []
		self.jointstate = JointState()
		self.num_of_wheels = num_of_wheels
		self.jointstate.header.frame_id = 'base_link'
		for i in range(0, self.num_of_wheels):
			self.jointstate.name.append("motor_"+str(i+1))
			self.jointstate.position.append(0)
			self.jointstate.effort.append(0)
			self.jointstate.velocity.append(0)
		self.array_msg = UInt32MultiArray()
		self.imu = Imu()
		self.imu.header.frame_id = "imu"
		self.imu.angular_velocity_covariance = [0.02,0,0,0,0.02,0,0,0,0.02]
		self.imu.linear_acceleration_covariance = [0.04,0,0,0,0.04,0,0,0,0.04] 
		self.imu.orientation_covariance = [0.0025,0,0,0,0.0025,0,0,0,0.0025]  
		self.voltage = Float32()
		self.previous_cmd_time = time()
		self.sub_cmd = rospy.Subscriber("cmd_vel", Twist, self.cmd_cb)
		self.js_pub = rospy.Publisher('joint_state', JointState, queue_size=1)
		self.ir_pub = rospy.Publisher('ir_array', UInt32MultiArray, queue_size=1)
		self.imu_pub = rospy.Publisher('imu', Imu, queue_size=1)
		self.bat_pub = rospy.Publisher('battery_voltage', Float32, queue_size=1)

	def read(self):
		b = str(self.ser.readline())
		b = b.replace('b', '')
		b = b.replace("'",'')
		b = b.replace("n",'')
		b = b.replace("r",'')
		b = b.replace("\\",'')
		b = b.split(',')
		if b[0] == 'eff':
			for i in range(0, self.num_of_wheels):
				self.jointstate.effort[i] = float(b[i+1])
		if b[0] == 'pos':
			for i in range(0, self.num_of_wheels):
				self.jointstate.position[i] = float(b[i+1])
		if b[0] == 'vel':
			for i in range(0, self.num_of_wheels):
				self.jointstate.velocity[i] = float(b[i+1])
			self.jointstate.header.stamp = rospy.Time.now()
			self.js_pub.publish(self.jointstate)
		if b[0] == 'light':
			for i in range(1,29):
				self.array_msg.data.append(int(b[i]))
			self.ir_pub.publish(self.array_msg)
			self.array_msg.data.clear()
		if b[0] == 'imu':
			self.imu.header.stamp = rospy.Time.now()
			self.imu.angular_velocity.x = float(b[1])
			self.imu.angular_velocity.y = float(b[2])
			self.imu.angular_velocity.z = float(b[3])
			self.imu.linear_acceleration.x = float(b[4])
			self.imu.linear_acceleration.y = float(b[5])
			self.imu.linear_acceleration.z = float(b[6])
			self.imu.orientation.x = float(b[7])
			self.imu.orientation.y = float(b[8])
			self.imu.orientation.z = float(b[9])
			self.imu.orientation.w = float(b[10])
			self.imu_pub.publish(self.imu)
		if b[0] == 'vol':
			self.voltage.data = float(b[1])
			self.bat_pub.publish(self.voltage)
		# print(b)
	def cmd_cb(self, data):
		if(float(time() - self.previous_cmd_time) > 0.12):
			string = str(data.linear.x) + "," + str(data.linear.y) + "," + str(data.angular.z) +"\n"
			self.ser.write(string.encode())
			self.previous_cmd_time = time()

if __name__ == '__main__':
	rospy.init_node('serial_controller')
	controller = SerialControl(4)
	while not rospy.is_shutdown():
		try:
			controller.read()
		except Exception as e:
			print(e)
	
