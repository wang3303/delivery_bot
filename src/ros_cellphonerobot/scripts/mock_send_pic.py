#!/usr/bin/env python
'''
A simple publisher is used to mock sending images to /image for image recognition.
Change the image you want to send before run the node.
'''
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

def talker():
    bridge = CvBridge()
    # TODO Change the image you want to send
    img = cv2.imread("/home/pi/barber.jpeg")
    pub = rospy.Publisher('image', Image ,queue_size = 10)
    rospy.init_node('mock_send_pic', anonymous=True)
    rate = rospy.Rate(0.3)
    while not rospy.is_shutdown():
        cvimage = bridge.cv2_to_imgmsg(img, "bgr8")
        rospy.loginfo("Upload picture")
        pub.publish(cvimage)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS INTERRUPT")
