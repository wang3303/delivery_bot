#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
rospy.init_node('cmd_vel_fake')
r = rospy.Rate(10)

while not rospy.is_shutdown():
    twist_data = Twist()
    twist_data.linear.x = 5
    twist_data.linear.y = 10
    twist_data.angular.z = 3
    
    pub.publish(twist_data)
