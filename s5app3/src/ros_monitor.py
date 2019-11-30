#!/usr/bin/env python

import rospy
import socket
import threading
import pickle       #truc rajouter
import time
from struct import *
from tf.transformations import euler_from_quaternion


from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class ROSMonitor:
    def __init__(self):
        # Add your subscriber here (odom? laserscan?):
        # self.sub_odom = rospy.Subcriber(...)
        # self.sub_laser = rospy.Subscriber(...)
        
        #data available detection
	self.pos_available = False
	self.obs_available = False

        # Current robot state:
        self.id = 63
        self.pos = [0,0,0]
        self.obstacle = False
        self.pos_broadcast_ip = '10.0.0.255'

        # Params :
        self.remote_request_port = rospy.get_param("remote_request_port", 65432)
        self.pos_broadcast_port  = rospy.get_param("pos_broadcast_port", 65431)

        # Thread for RemoteRequest handling:
        self.rr_thread = threading.Thread(target=self.rr_loop)

        # Thread for PositionBroadcast handling:
        self.pb_thread = threading.Thread(target=self.positionBroadcast)

        print("ROSMonitor started.")        

       

        #subscribe to the node odometry for posistion of the vehicule
        rospy.Subscriber("/odom", Odometry, self.scan_odom)

        rospy.Subscriber("/scan", LaserScan, self.obstacle_detection)

    def quaternion_to_yaw(self, quat):
        # Uses TF transforms to convert a quaternion to a rotation angle around Z.
        # Usage with an Odometry message: 
        #   yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        (roll, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return yaw

    def scan_odom(self, msg):
	self.pos_available = True
        self.pos[0] = msg.pose.pose.position.x
        self.pos[1] = msg.pose.pose.position.y
        self.pos[2] = self.quaternion_to_yaw(msg.pose.pose.orientation)

    def obstacle_detection(self, msg):
        self.obs_available = True
	if min(msg.ranges) <= 1:        
            self.obstacle =True
	    #print(min(msg.ranges))
        else:
            self.obstacle = False

    def rr_loop(self):
        # Init your socket here :
        # self.rr_socket = socket.Socket(...)self.rr_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.rr_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)        
        self.rr_socket.bind(('10.0.0.4', self.remote_request_port))
        self.rr_socket.listen(3)
        while True:
            try:
                (conn, addr) = self.rr_socket.accept()          
            
                while True:
                    format = "!Ixxxxxxxxxxxx"
                    try :
                        #unpack the command (1,2 or 3) else go into the except
                        command = unpack(format, conn.recv(1024))[0]
                        print("RPC received from")
                        if command == 1:
                            format = "!fffxxxx"
                            if self.pos_available:
                                conn.send(pack(format,self.pos[0], self.pos[1], self.pos[2]))    
                            else:
                                format = "!cxxxxxxxxxxxx"
                                conn.send(pack(format,'E'))
                        elif command == 2:
	                        format = "!Ixxxxxxxxxxxx"
	                        if self.obs_available:
                                	conn.send(pack(format,self.obstacle))
                            	else:
                                	format = "!cxxxxxxxxxxxx"
                                	conn.send(pack(format,'E'))
                        elif command == 3:
                            format = "!Ixxxxxxxxxxxx"
                            conn.send(pack(format,self.id))
                        else:
                            print("RPC note recognised")
                    except error:
	                    print("Client disconnect")
                            conn.close()
	                    break
            except socket.timeout:
                print("Timeout.. Trying again")
	


    def positionBroadcast(self):
        # https: // wiki.python.org / moin / UdpCommunication

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        format = "!fffI"     #unsinged int =Id;float=x ;float=y ;float= theta
        enc = pack(format,self.pos[0], self.pos[1], self.pos[2],self.id)

        while True:
            # Make sure that message is usigned int, float,float,float (need mutex ?)
            pos_msg = pack(format, self.pos[0], self.pos[1], self.pos[2], self.id)
            #print(unpack(format, pos_msg))
            sock.sendto((pos_msg), (self.pos_broadcast_ip, self.pos_broadcast_port))

            time.sleep(1)  # 1 Hz loop


if __name__=="__main__":
    rospy.init_node("ros_monitor")

 # Start threads and set them in Daemon so they can be killed with Ctrl-C
    node = ROSMonitor()
    node.pb_thread.setDaemon(True)
    node.rr_thread.setDaemon(True)
    node.pb_thread.start()
    node.rr_thread.start()

    rospy.spin()



