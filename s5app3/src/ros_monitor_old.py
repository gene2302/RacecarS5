#!/usr/bin/env python

import rospy
import socket
import threading

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

class ROSMonitor:
    def __init__(self):
        # Add your subscriber here (odom? laserscan?):
        # self.sub_odom = rospy.Subcriber(...)
        # self.sub_laser = rospy.Subscriber(...)

        # Current robot state:
        self.id = 0xFFFF
        self.pos = (0,0,0)
        self.obstacle = False

        # Params :
        self.remote_request_port = rospy.get_param("remote_request_port", 65432)
        self.pos_broadcast_port  = rospy.get_param("pos_broadcast_port", 65431)

        # Thread for RemoteRequest handling:
        self.rr_thread = threading.Thread(target=self.rr_loop)
        self.rr_thread.start()

        print("ROSMonitor started.")

    def scan_odom(self, msg):
            theta_z = quaternion_to_yaw(msg.pose.pose.orientation)
            print("Position x : " )
            print(msg.pose.pose.position.x)
            print("Angle Z : " )
            print(theta_z)

    def rr_loop(self):
        # Init your socket here :
        # self.rr_socket = socket.Socket(...)
        self.rr_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.rr_socket.bind(('127.0.0.1', self.remote_request_port))
        self.rr_socket.listen(1)
        (conn, addr) = self.rr_socket.accept()
        while True:
            p = conn.recv(1024)
            if not data: break
            conn.send(str(data))
        conn.close()
        

if __name__=="__main__":
    rospy.init_node("ros_monitor")

    node = ROSMonitor()

    rospy.spin()


