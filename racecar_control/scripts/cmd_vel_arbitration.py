#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class Arbitration:
    def __init__(self):
        self._delay_sec = rospy.get_param('~delay_sec', 0.5)

        self._cmd_vel_pub = rospy.Publisher('cmd_vel_output', Twist, queue_size=1)
        self._cmd_vel_sub0 = rospy.Subscriber('cmd_vel_input_0', Twist, self.cmd_vel_callback0, queue_size=1)
        self._cmd_vel_sub1 = rospy.Subscriber('cmd_vel_input_1', Twist, self.cmd_vel_callback1, queue_size=1)
        self._cmd_vel_sub2 = rospy.Subscriber('cmd_vel_input_2', Twist, self.cmd_vel_callback2, queue_size=1)
        self._cmd_vel_sub3 = rospy.Subscriber('cmd_vel_input_3', Twist, self.cmd_vel_callback3, queue_size=1)
        t = rospy.get_time()
        self._timeCalled = [t,t,t,t]

    def cmd_vel_callback(self, msg, priority):
        self._timeCalled[priority] = rospy.get_time();
        pub = True
        for i in range(1,priority+1):
            if self._timeCalled[priority] - self._timeCalled[i-1] < self._delay_sec:
                pub=False
                break
        if pub:
            self._cmd_vel_pub.publish(msg);

    def cmd_vel_callback0(self, msg):
        self.cmd_vel_callback(msg, 0)

    def cmd_vel_callback1(self, msg):
        self.cmd_vel_callback(msg, 1)

    def cmd_vel_callback2(self, msg):
        self.cmd_vel_callback(msg, 2)

    def cmd_vel_callback3(self, msg):
        self.cmd_vel_callback(msg, 3)

def main():
    rospy.init_node('arbitration')
    arbitration = Arbitration()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

