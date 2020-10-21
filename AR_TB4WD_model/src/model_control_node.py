#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from math import pi
from nav_msgs.msg import Odometry

pub_wheel_state = rospy.Publisher('/ar_tb4wd/joint_states',JointState,queue_size = 10)
joitn_name = ['tb4_wheel_1_joint','tb4_wheel_2_joint','tb4_wheel_3_joint','tb4_wheel_4_joint'] 

def update_joint_state(cur_joint_state):
    joint_state_msg = JointState()
    joint_state_msg.name = joitn_name
    joint_state_msg.position = cur_joint_state.position
    joint_state_msg.header.stamp = rospy.Time.now()
    pub_wheel_state.publish(joint_state_msg)

if __name__=="__main__":
    rospy.init_node('model_control')
    rospy.Subscriber('/joint_state',JointState,update_joint_state)
    rospy.spin()
