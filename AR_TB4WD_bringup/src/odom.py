#!/usr/bin/env python3  
#include libraries
import rospy
import tf
from math import cos, sin, pi
from sensor_msgs.msg import JointState
from geometry_msgs.msg import  Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry

jointstates = JointState()

jointstates.name = ['wheel0','wheel1','wheel2','wheel3']

pubodom = rospy.Publisher('odom',Odometry,queue_size=10)

broadcaster = tf.TransformBroadcaster()
map_broadcaster = tf.TransformBroadcaster()

timeold=0

vel = Twist()

x=y=theta=0
counter = 0
def jstatecb(jointstatesraw):
    global jointstates
    global timeold
    global x,y,theta, counter
    
    jointstates.velocity= jointstatesraw.velocity
    jointstates.effort= jointstatesraw.effort
    jointstates.header.stamp= jointstatesraw.header.stamp
    
    time = rospy.Time.now()
    timenew= time.to_sec() 
    delta = timenew -timeold
    
    if delta >=0.01:
        current_time = timenew
        timeold = timenew
        Vx,Vy, Vtheta = calculate_odom(delta, jointstates.velocity[0], jointstates.velocity[1],jointstates.velocity[2], jointstates.velocity[3])
        odom_quat = tf.transformations.quaternion_from_euler(0,0,theta)
       	broadcaster.sendTransform((x,y,0.0),odom_quat,rospy.Time.now(),"base_link","odom")
        odom = Odometry()
        odom.header.stamp = time
        odom.header.frame_id = "odom"
        odom.pose.pose = Pose(Point(x,y,0.), Quaternion(*odom_quat))
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(Vx,Vy,0.),Vector3(0,0,Vtheta))
        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[7] = 0.05
        odom.twist.covariance[35] = 0.01
        #publishing  
        pubodom.publish(odom)
def calculate_odom(delta,vel0,vel1,vel2,vel3):
    global x, y, theta
    R = 0.05
    wheel_separation = (0.355+0.355)/2

    Vx=(vel0+vel1-vel2-vel3)*(R/4)
    Vy=(-vel0-vel2+vel3+vel1)*(R/4)
    Vtheta=(-vel0+vel2-vel3+vel1)*(R/(4*wheel_separation))
    # print("vx: "+str(Vx))
    # print("vy: "+str(Vy))
    theta += delta*Vtheta
    if Vx <= 2 and Vy <= 2 and Vtheta <= 4:
    	x += delta* (cos(theta)*Vx - sin(theta)*Vy)
    	y += delta* (sin(theta)*Vx + cos(theta)*Vy)
    	return Vx,Vy,Vtheta
    else:
    	return 0,0,0
    
if __name__ == '__main__':
	rospy.init_node('odom_node',anonymous=False)
	rospy.Subscriber("joint_state",JointState,jstatecb)
	timeold = rospy.get_time()
	while not rospy.is_shutdown():
		tc_left_quat = tf.transformations.quaternion_from_euler(0,0,pi/2)
		lidar_quat = tf.transformations.quaternion_from_euler(0,0,pi)
		# map_broadcaster.sendTransform((0,0,0), (0,0,0,1), rospy.Time.now(), "odom", "map")
		map_broadcaster.sendTransform((0.1,0.0,0.1),lidar_quat,rospy.Time.now(),"laser","base_link")
		rospy.sleep(1)
		