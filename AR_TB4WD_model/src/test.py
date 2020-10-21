#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

rospy.init_node("sim_velocity")
pub = rospy.Publisher('/joint_state',JointState,queue_size=10)

step = 0.1
pose = [0,0,0,0]
vel = [step,-step,step,-step]
js_msg = JointState()
rate  = rospy.Rate(10)

while not rospy.is_shutdown():
    js_msg.header.stamp = rospy.Time.now()
    js_msg.position = pose
    js_msg.velocity = vel
    pose = [el+step for el in pose]
    pub.publish(js_msg)
    rate.sleep()