#!/usr/bin/env python  
import rospy
from sensor_msgs.msg import LaserScan
from math import pi

def lidar_cb(data):
	msg = LaserScan()
	msg.header = data.header
	msg.angle_min = data.angle_min
	msg.angle_max = data.angle_max
	msg.angle_increment = data.angle_increment
	msg.scan_time = data.scan_time
	msg.range_min = data.range_min
	msg.range_max = data.range_max
	current_angle = data.angle_min
	i = 0
	for range in data.ranges:
		if((i >= 30 and i <= 40) or (i>=75 and i <=90) or (i>=265 and i <= 280) or (i>=310 and i <=320)):
			msg.ranges.append(0)
			msg.intensities.append(0)
		else:
			msg.ranges.append(data.ranges[i])
			msg.intensities.append(data.intensities[i])
		i+=1
	publisher.publish(msg)


if __name__ == '__main__':
	rospy.init_node('remap_scan_node',anonymous=True)
	rospy.Subscriber("scan_bad",LaserScan, lidar_cb)
	publisher = rospy.Publisher('scan',LaserScan,queue_size=10)
	rospy.spin()