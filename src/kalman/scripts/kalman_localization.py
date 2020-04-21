#! /usr/bin/env python

import rospy

from laser_line_extraction.msg import LineSegmentList
from kfilter.kfilter import KalmanFilter
from sensor_msgs.msg import JointState

def collect_joint_states(joint_states, kalman_filter):
    kalman_filter.save_joint_states(joint_states)

def collect_line_segments(line_segments, kalman_filter):
    kalman_filter.save_line_segments(line_segments)

def start_kalman_localization():
    rospy.init_node('kalman_localization', anonymous=False)
    kalman_filter = KalmanFilter()

    rospy.Subscriber(
        'joint_states', JointState, collect_joint_states, kalman_filter
    )
    rospy.Subscriber(
        'line_segments', LineSegmentList, collect_line_segments, kalman_filter
    )

    rospy.loginfo('Kalman localization node is available.')
    rospy.spin()

if __name__ == '__main__':
    try:
        start_kalman_localization()
    except rospy.ROSInterruptException:
        pass
