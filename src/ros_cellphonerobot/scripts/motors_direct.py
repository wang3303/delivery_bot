#!/usr/bin/env python

"""
This node is created to translate cmd_vel commands into motor commands.
"""

import rospy
from hardware import DCmotor
from std_msgs.msg import Float32

def leftcallback(data):
    motor_l.change_duty_cycle(data.data)
    # rospy.loginfo("Left motor reading: %s", data.data)

def rightcallback(data):
    motor_r.change_duty_cycle(data.data)
    # rospy.loginfo("Right motor reading: %s", data.data)

def listener():
	rospy.init_node('motor_controller')
	rospy.Subscriber("lmotor_cmd", Float32, leftcallback)
	rospy.Subscriber("rmotor_cmd", Float32, rightcallback)
	rospy.spin()
	

motor = rospy.get_param("/motor")

motor_r_pin = motor['rpid_velocity'] #rospy.get_param('/execution/'+motor[0])
motor_l_pin = motor['lpid_velocity'] #rospy.get_param('/execution/'+motor[1])
motor_r = DCmotor(motor_r_pin[0],motor_r_pin[1],motor_r_pin[2],motor_r_pin[3],motor_r_pin[4],use_encoder=False,use_motor=True)
motor_l = DCmotor(motor_l_pin[0],motor_l_pin[1],motor_l_pin[2],motor_l_pin[3],motor_l_pin[4],use_encoder=False,use_motor=True)

# motor_l = DCmotor(18,10,22,2,3)

try:
	# listener()
	pass
except rospy.ROSInterruptException:
	motor_r.cleanup()
	motor_l.cleanup()
