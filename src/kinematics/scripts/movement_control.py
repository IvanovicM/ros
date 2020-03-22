#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def collect_odometry(odom, command_publisher):
    rospy.loginfo(odom)

def start_movement_control():
    rospy.init_node('movement_control', anonymous=False)
    command_publisher = rospy.Publisher('cmd_vel', Twist)
    rospy.Subscriber('odom', Odometry, collect_odometry, command_publisher)
    rospy.spin()

    rospy.loginfo('Movement control node is available.')

if __name__ == '__main__':
    try:
        start_movement_control()
    except rospy.ROSInterruptException:
        pass
