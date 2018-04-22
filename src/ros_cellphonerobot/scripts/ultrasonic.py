#!/usr/bin/env python
"""
This node is for reading distance from a ultrasonic sensor and publishing to /ultrasonic.
"""
import rospy
from sensor_msgs.msg import Range
from random import random
from hardware import Ultrasonic

def get_distance():
    return sensor.request_distance()*100

def publish():
    while not rospy.is_shutdown():
        pub = rospy.Publisher("distance", Range, queue_size=10)
        rospy.init_node("ultrasonic", anonymous=True)

        rate = rospy.Rate(4) #4Hz
        msg = Range(radiation_type = 0)
        ## ULTRASOUND = 
        ## IR = 1
        msg.range = get_distance()
        msg.min_range = min_range
        msg.max_range = max_range
        if msg.range < max_range and msg.range > min_range:
            pub.publish(msg)
        rate.sleep()
        rospy.loginfo("%.2f" % (msg.range))


if __name__ == '__main__':
    try:
        rospy.init_node('ultrasonic', anonymous=True)
        min_range = rospy.get_param('~min_distance')
        max_range = rospy.get_param("~max_distance")
        pinnumber = rospy.get_param("~pin")
        sensor = Ultrasonic(max_distance=max_range, threshold_distance =0.01, echo=pinnumber[0], trigger=pinnumber[1])
        publish()
    except rospy.ROSInterruptException:
        rospy.logerr("Interrupted")
