#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8, String
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from move_base_msgs.msg import MoveBaseActionFeedback


pub_byte = rospy.Publisher('move_base_bytestatus', Int8, queue_size=10)
pub_text = rospy.Publisher('move_base_textstatus', String, queue_size=10)


flag = False

def callback(data):
	global pub_byte
	global pub_text, flag
	if (len(data.status_list) ):
		l = str(data.status_list[0])
		index = l.find("status:")
		# status_int = status
		# status_text = data.status_list.text
		# rospy.loginfo((l[index+8]))
		if flag:
			pub_byte.publish(int(l[index+8]))
		if int(l[index+8]) == 3:
			flag = False
		elif int(l[index+8]) == 1:
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
