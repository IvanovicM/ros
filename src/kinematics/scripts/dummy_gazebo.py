#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def process_command(twist):
    rospy.info('New command for the robot: {}'.format(twist))

def start_dummy_gazebo():
    rospy.init_node('dummy_gazebo', anonymous=False)
    gazebo_publisher = rospy.Publisher('odom', Odometry, queue_size=10)
    rospy.Subscriber('cmd_vel', Twist, process_command)
    r = rospy.Rate(100)
    
    while not rospy.is_shutdown():
        gazebo_publisher.publish(Odometry())
        r.sleep()

if __name__ == '__main__':
    try:
        start_dummy_gazebo()
    except rospy.ROSInterruptException:
        pass


