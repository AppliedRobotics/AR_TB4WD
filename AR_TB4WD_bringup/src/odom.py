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

rospy.init_node('odom_node',anonymous=True)

pubodom = rospy.Publisher('odom',Odometry,queue_size=10)

broadcaster = tf.TransformBroadcaster()
timeold=0

vel = Twist()

x=y=theta=0

def jstatecb(jointstatesraw):
    global jointstates
    global timeold
    global x,y,theta
    
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
        
        lidar_quat = tf.transformations.quaternion_from_euler(0,0,pi)
        broadcaster.sendTransform((0.1,0.0,0.1),lidar_quat,time,"laser","base_link")
        odom_quat = tf.transformations.quaternion_from_euler(0,0,theta)
       
        broadcaster.sendTransform((x,y,0.0),odom_quat,time,"base_link","odom")
        # broadcaster.sendTransform((0,0,0), odom_quat, time, "odom", "map")
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
    x += delta* (cos(theta)*Vx - sin(theta)*Vy)
    y += delta* (sin(theta)*Vx + cos(theta)*Vy)
    return Vx,Vy,Vtheta

    
if __name__ == '__main__':

    timeold = rospy.get_time()
    rospy.Subscriber("joint_state",JointState,jstatecb)
    rospy.spin()
    # print ("im here")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()


