#!/usr/bin/env python
"""
This node is for reading encoder ticks and publishes them to /ticks.
"""
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Int16
from random import random
from hardware import DCmotor

def get_right_ticks():
	return motor_r.request_encoder_ticks()

def get_left_ticks():
	return motor_l.request_encoder_ticks()

def publish():
    while not rospy.is_shutdown():
		rospy.init_node("encoders", anonymous=True)
		pub_left = rospy.Publisher('lwheel', Int16, queue_size=20)
		pub_right = rospy.Publisher('rwheel', Int16, queue_size=20)
		limit = 32768
		rate = rospy.Rate(200) 
		right_ticks = Int16()
		left_ticks = Int16()
		r_buffer= get_right_ticks()
		l_buffer = get_left_ticks()
		# if r_buffer >= limit-1:
		# 	r_buffer = r_buffer - (limit*2-1)
		# elif r_buffer <= -limit:
		# 	r_buffer = r_buffer + (limit*2-1)	
		
		# if l_buffer >= limit-1:
		# 	l_buffer = l_buffer - (limit*2-1)
		# elif l_buffer <= -limit:
		# 	l_buffer = l_buffer + (limit*2 -1)
		while l_buffer >= limit-1:
			l_buffer -= (2*limit - 1)
		while l_buffer <= 0-limit:
			l_buffer += (2*limit - 1)
		while r_buffer >= limit-1:
			r_buffer -= (2*limit - 1)
		while r_buffer <= 0-limit:
			r_buffer += (2*limit - 1)
		
		# rospy.loginfo("$$$$$$$$ %s %s" % (l_buffer, r_buffer))
		right_ticks.data = r_buffer
		left_ticks.data = l_buffer

		pub_left.publish(left_ticks)
		pub_right.publish(right_ticks)
		
		rate.sleep()
		rospy.loginfo("Left encoder A = %s", left_ticks.data)
		rospy.loginfo("Right encoder A = %s", right_ticks.data)

if __name__ == '__main__':
    try:
        # rospy.init_node('encoders', anonymous=True)
        # motor = rospy.get_param("/motor")
        # motor_r_pin = motor['motor_r']#rospy.get_param('/execution/'+motor[0])
        # motor_l_pin = motor['motor_l']#rospy.get_param('/execution/'+motor[1])
        # motor_r = DCmotor(motor_r_pin[0],motor_r_pin[1],motor_r_pin[2],motor_r_pin[3],motor_r_pin[4],motor_r_pin[5],use_encoder=True,use_motor=False)
        # motor_l = DCmotor(motor_l_pin[0],motor_l_pin[1],motor_l_pin[2],motor_l_pin[3],motor_l_pin[4],motor_l_pin[5],use_encoder=True,use_motor=False)
        motor_l = DCmotor(18,10,22,2,3,use_encoder=True,use_motor=False)
        motor_r = DCmotor(17,4,27,14,15,use_encoder=True,use_motor=False)
        publish()
    except rospy.ROSInterruptException:
        rospy.logerr("Interrupted")
