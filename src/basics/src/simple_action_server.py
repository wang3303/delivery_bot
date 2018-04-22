#!/usr/bin/env python

import rospy
import time
from basics.msg import TimerAction, TimerGoal, TimerResult
import actionlib

def do_timer(goal):
	start_timer = time.time()
	time.sleep(goal.time_to_wait.to_sec())
	result = TimerResult()
	result.time_elapsed = rospy.Duration.from_sec(time.time() - start_timer)
	result.updates_sent = 0
	server.set_succeeded(result)

rospy.init_node('timer_action_server')
server = actionlib.SimpleActionServer('timer', TimerAction, do_timer, False)
server.start()
rospy.spin()
