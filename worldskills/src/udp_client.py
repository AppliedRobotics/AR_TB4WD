#!/usr/bin/env python3  
import roslib
import rospy
import math  
from geometry_msgs.msg import Twist
import socket
import re
from std_msgs.msg import Int16MultiArray, Int16

UDP_PORT_IN = 8888 
UDP_PORT_OUT = 8888
UDP_SERVER_ADDRESS = 'rp' #rp adress
UDP_CLIENT_ADDRESS = 'pc' #pc adress
last_ip = "240"
MAX_UDP_PACKET=128 # max size of incoming packet to avoid sending too much data to the micro

udp_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
udp_socket.bind((UDP_SERVER_ADDRESS, UDP_PORT_IN)) 
zone = 1 
move_flag = 0 
timer = 0 
status = 0
def parser(str): 
    global move_flag, zone
    f = str.split(":") 
    if(f[0] == "s"): 
        move_flag = int(re.sub(r'[^0-9]','',f[1]))
        zone = int(re.sub(r'[^0-9]','',f[2])) 
        # print(move_flag," ", zone) 

def connect_udp():
    try: 
        udp_socket.connect((UDP_CLIENT_ADDRESS, UDP_PORT_OUT)) 
        print('connection succesful')
        udp_socket.settimeout(0.5) 
        return True
    except Exception as e: 
        print('connection failed', e)
        return False
def status_cb(data):
    global status
    status = data.data

if __name__ == '__main__': 
    connected = connect_udp()
    rospy.init_node('udp_handler_node') 
    command_pub = rospy.Publisher("command", Int16MultiArray, queue_size = 1) 
    command_msg = Int16MultiArray()
    command_msg.data = [0,0]
    rospy.Subscriber("status", Int16, status_cb) 
    rate = rospy.Rate(10.0) 
    while not rospy.is_shutdown():
        data = "" 
        try:
            try:
                data, udp_client = udp_socket.recvfrom(MAX_UDP_PACKET)  
                parser(data.decode())
                if int(zone) == 0:
                    zone = 200
                command_msg.data = [int(move_flag), int(zone)]
                command_pub.publish(command_msg)
            except Exception as e:
                if not str(e) == 'timed out':
                    print("parsing error, receive: " + str(data) +" "+ str(e)) 
            if(rospy.get_time() - timer > 1): 
                timer = rospy.get_time()
                # udp_socket.sendto("S:211:"+str(1)+":"+str(1)+"#\n".encode(), (UDP_CLIENT_ADDRESS, UDP_PORT_OUT)) 
                # zone_pub.publish(zone) 
                # move_pub.publish(move_flag)
                payload = "S:"+last_ip+":"+str(status)+":"+str(zone)+"#\n"
                udp_socket.sendto(payload.encode(), (UDP_CLIENT_ADDRESS, UDP_PORT_OUT))
            rate.sleep()
        except KeyboardInterrupt:
            udp_socket.close()
            break
