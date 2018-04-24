#!/usr/bin/env python
"""
This script is for sending a status code to Topic /move_base_bytestatus,
informing android app of the current state of robot.
"""


import rospy
from std_msgs.msg import Int8, String
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from move_base_msgs.msg import MoveBaseActionFeedback


pub_byte = rospy.Publisher('move_base_bytestatus', Int8, queue_size=10)
# pub_text = rospy.Publisher('move_base_textstatus', String, queue_size=10)

# this flag indicates whether a goal has been accomplished
# This flag prohibits sending more than one success code
# which will cause android listener to assume multiple success
flag = False

def callback(data):
	global pub_byte
	global pub_text, flag
	# Getting status code
	# Rospy resolves the GoalStatusArray as strings and thus status code can't be
	# retrieved as a dictionary entry
	if (len(data.status_list) ):
		l = str(data.status_list[0])
		index = l.find("status:")
		if flag:
			pub_byte.publish(int(l[index+8]))
		if int(l[index+8]) == 3:
			# The goal was achieved successfully by the action server (Terminal State)
			flag = False
		elif int(l[index+8]) == 1:
			# The goal has yet to be processed by the action server
			flag = True

def listener():
	rospy.init_node('move_base_status_listener', anonymous=True)
	rospy.Subscriber("move_base/status", GoalStatusArray, callback)
	rospy.spin()
	
if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass
