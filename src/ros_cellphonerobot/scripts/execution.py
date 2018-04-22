#!/usr/bin/env python
"""
This is the execution node sending commands published on Topic "/action" to execution server.
It is also listening to Topic "sensor_reading" where up-to-date sensor reading is published.
The sensor reading is processed and will affect the action being executed.
In this node, when ultrasonic sensor reading is below a threshold, the action will be aborted.
There is also a Topic called "flag" which indicates whether the ROS system is busy executing
a command or waiting to get a command. This is useful in that it would let the Android App
know the status quo of the ROS system. 0 means busy and 1 means free.
You should define your action action here as member function of the class.
"""
import rospy
import actionlib
from ros_cellphonerobot.msg import ExecutionResult,ExecutionFeedback,ExecutionGoal,ExecutionAction
import time
from std_msgs.msg import String
from ros_cellphonerobot.msg import Sensors
from std_msgs.msg import Int8

# action_client state code explanation
# uint8 PENDING=0
# uint8 ACTIVE=1
# uint8 PREEMPTED=2
# uint8 SUCCEEDED=3
# uint8 ABORTED=4
# uint8 REJECTED=5
# uint8 PREEMPTING=6
# uint8 RECALLING=7
# uint8 RECALLED=8
# uint8 LOST=9

state = 1

def actioncb(msg):
    global state
    pub.publish(0)# busy
    state = 0
    act_client.wait_for_server()
    rospy.loginfo('Find exe server')
    goal = ExecutionGoal(time_to_wait=default_exe_time)
    goal.action = str(msg.data)
    act_client.send_goal(goal, feedback_cb=feedback_cb)
    act_client.wait_for_result()
    rospy.loginfo('%d: Flag: %.2f' % (act_client.get_state(), act_client.get_result().flag))
    pub.publish(1)  # free
    state = 1


def feedback_cb(feedback):
    rospy.loginfo('Motor:Time elapsed: %.2f' % (feedback.time_elapsed.to_sec()))

def abort(msg):
    global state
    if msg.distance.range < mindistance:
        if state == 0:
            act_client.cancel_goal()
            rospy.loginfo('Range alert')
            state = 1
            pub.publish(1)  # free

def abort_mission(msg):
    act_client.cancel_all_goals()
    rospy.loginfo('cancel_all_goals')

if __name__ == '__main__':
    try:
        rospy.init_node('execution')
        mindistance =rospy.get_param('/alert_distance')
        default_exe_time = rospy.Duration.from_sec(rospy.get_param('/exe_time'))
        act_client = actionlib.SimpleActionClient('robot', ExecutionAction)
        rospy.Subscriber('action', String, actioncb)
        rospy.Subscriber('sensor_reading', Sensors, abort)
        rospy.Subscriber('abort_mission',String,abort_mission)
        pub = rospy.Publisher("flag", Int8, queue_size=10)
        time.sleep(0.)
        rospy.spin()
    except rospy.ROSInterruptException,e:
        rospy.logerr(e)