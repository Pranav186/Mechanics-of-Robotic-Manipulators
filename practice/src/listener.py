#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
	rospy.loginfo("I heard %s", data.data)

rospy.init_node('listener_node', anonymous=True)
rospy.Subscriber('chatter', String, callback)
rospy.spin()
