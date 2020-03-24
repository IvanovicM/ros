#! /usr/bin/env python

import rospy
from control.controller import StateController
from geometry_msgs.msg import Point # float64 x, y z, z
from geometry_msgs.msg import Pose # Point position, Quaternion quaternion, TwistWithCovariance twist
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry

def collect_odometry(odometry, controller):
    controller.process_odometry(odometry)

def start_movement_control():
    rospy.init_node('movement_control', anonymous=False)
    command_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    controller = StateController(command_publisher)
    rospy.Subscriber('odom', Odometry, collect_odometry, controller)

    rospy.loginfo('Movement control node is available.')
    controller.talk_to_user()

if __name__ == '__main__':
    try:
        start_movement_control()
    except rospy.ROSInterruptException:
        pass

