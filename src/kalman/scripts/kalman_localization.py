#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan
from filter.kfilter import KalmanFilter

def filter_with_kalman(laser_scan, kalman_filter):
    # TODO
    print('New laser scan')

def start_kalman_localization():
    rospy.init_node('kalman_localization', anonymous=False)
    kalman_filter = KalmanFilter()
    rospy.Subscriber(
        'kobuki/laser/scan', LaserScan, filter_with_kalman, kalman_filter
    )

    rospy.loginfo('Kalman localization node is available.')
    rospy.spin()

if __name__ == '__main__':
    try:
        start_kalman_localization()
    except rospy.ROSInterruptException:
        pass
