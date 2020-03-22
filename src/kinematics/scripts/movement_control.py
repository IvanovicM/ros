#! /usr/bin/env python

import rospy
from std_msgs.msg import String

def start_movement_control():
    measurements_publisher = rospy.Publisher(
        'movement_topic', String, queue_size = 10
    )
    rospy.init_node('movement_control', anonymous=False)
    r = rospy.Rate(100)
    rospy.loginfo('Movement control node is available.')

    while True:
        rospy.loginfo('Movement control is running.')

if __name__ == '__main__':
    try:
        start_movement_control()
    except rospy.ROSInterruptException:
        pass
