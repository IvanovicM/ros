#! /usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(dana):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", dana.data)

def listener():
    rospy.init_node('hello_world_subscriber', anonymous=True)
    rospy.Subscriber('measurement_pub', String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
