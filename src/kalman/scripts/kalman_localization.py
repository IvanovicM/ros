#! /usr/bin/env python

import rospy

from laser_line_extraction.msg import LineSegmentList
from kfilter.kfilter import KalmanFilter
from kfilter.visualize import RvizMarkerPublisher
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray

def collect_joint_states(joint_states, args):
    kalman_filter = args[0]
    marker_publisher = args[1]

    kalman_filter.save_joint_states(joint_states)
    kalman_pos = kalman_filter.perform()
    if kalman_pos is not None:
        marker_publisher.publish(kalman_pos)

def collect_line_segments(line_segments, kalman_filter):
    kalman_filter.save_line_segments(line_segments)

def start_kalman_localization():
    rospy.init_node('kalman_localization', anonymous=False)
    marker_topic_publisher = rospy.Publisher(
        'visualization_marker_array', MarkerArray, queue_size=10
    )

    kalman_filter = KalmanFilter()
    marker_publisher = RvizMarkerPublisher(marker_topic_publisher)
    rospy.Subscriber(
        'joint_states', JointState, collect_joint_states, 
        (kalman_filter, marker_publisher)
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
