#!/usr/bin/env python

'''
This script will subsribe to Topic /action and generate 
corresponding reference velocity at Topic /cmd_vel
keys
	q w e
	a s d 
	z x c
represents various direction and 's' means stop.
'''
import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# Publishing rate
RATE = 50
# Reference angular velocity
angular = .6
# Reference linear velocity
velocity = .4
key_mapping = { 'w': [ 0, velocity], 'x': [ 0, -velocity],
                'a': [ angular, 0], 'd': [-angular,  0],
                's': [ 0, 0] , 'q': [angular, velocity],
                'e': [-angular,velocity], 'c': [angular, -velocity],
                'z': [-angular, -velocity]
                }

g_twist_pub = None
g_target_twist = None
g_last_twist = None
g_last_send_time = None
g_vel_scales = [1, 1] # default to very slow
g_vel_ramps = [1, 1] # units: meters per second^2


def ramped_vel(v_prev, v_target, t_prev, t_now, ramp_rate):
    # compute nect step velocity
    step = ramp_rate * (t_now - t_prev).to_sec()
    sign = 1.0 if (v_target > v_prev) else -1.0
    error = math.fabs(v_target - v_prev)
    if error < step: # we can get there within this timestep-we're done.
        return v_target
    else:
        return v_prev + sign * step  # take a step toward the target


def ramped_twist(prev, target, t_prev, t_now, ramps):
    # generate next step Twist message
    tw = Twist()
    tw.angular.z = ramped_vel(prev.angular.z, target.angular.z, t_prev,
                            t_now, ramps[0])
    tw.linear.x = ramped_vel(prev.linear.x, target.linear.x, t_prev,
                           t_now, ramps[1])
    return tw


def send_twist():
	# publish Twist messages with time stamps
    global g_last_twist_send_time, g_target_twist, g_last_twist,\
         g_vel_scales, g_vel_ramps, g_twist_pub
    t_now = rospy.Time.now()
    g_last_twist = ramped_twist(g_last_twist, g_target_twist,
                              g_last_twist_send_time, t_now, g_vel_ramps)
    g_last_twist_send_time = t_now
    g_twist_pub.publish(g_last_twist)


def keys_cb(msg):
	# /action subscribers callback function
	# update target velocity
    global g_target_twist, g_last_twist, g_vel_scales
    if len(msg.data) == 0 or not key_mapping.has_key(msg.data[0]):
        return # unknown key
    vels = key_mapping[msg.data[0]]
    g_target_twist.angular.z = vels[0] * g_vel_scales[0]
    g_target_twist.linear.x = vels[1] * g_vel_scales[1]


def fetch_param(name, default):
	# get parameters from 
    if rospy.has_param(name):
        return rospy.get_param(name)
    else:
        print "parameter [%s] not defined. Defaulting to %.3f" % (name, default)
        return default


if __name__ == '__main__':
    rospy.init_node('keys_to_twist')
    g_last_twist_send_time = rospy.Time.now()
    g_twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('action', String, keys_cb)
    g_target_twist = Twist() # initializes to zero
    g_last_twist = Twist()
    # getting velocity scale and acceleration limit
    g_vel_scales[0] = fetch_param('~angular_scale', 1)
    g_vel_scales[1] = fetch_param('~linear_scale', 1)
    g_vel_ramps[0] = fetch_param('~angular_accel', 1.0)
    g_vel_ramps[1] = fetch_param('~linear_accel', 1.0)
    rate = rospy.Rate(RATE)
    while not rospy.is_shutdown():
        send_twist()
        rate.sleep()
