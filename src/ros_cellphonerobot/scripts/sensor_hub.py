#!/usr/bin/env python
'''
This node is for combining all the sensor readings into a message and publishing on
/sensor_reading.
'''
import rospy
from ros_cellphonerobot.msg import Sensors
from sensor_msgs.msg import Range

def cb(msg):
    global sensor_readings
    message = Sensors()
    message = sensor_readings
    message.distance = msg
    pub.publish(message)
    rospy.loginfo("Update distance")


if __name__ == '__main__':
    try:
        sensor_readings = Sensors()
        rospy.init_node("sensor_hub", anonymous=True)
        rospy.Subscriber("distance", Range ,cb)
        pub = rospy.Publisher("sensor_reading", Sensors, queue_size=10)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
