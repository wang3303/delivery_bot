#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

rospy.init_node('doubler_log')

def callback(msg):
	print msg.data
	
sub = rospy.Subscriber('doubled', Int32, callback)
rospy.spin()
