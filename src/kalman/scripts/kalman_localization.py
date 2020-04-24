#! /usr/bin/env python

import rospy

from laser_line_extraction.msg import LineSegmentList
from kfilter.kfilter import KalmanFilter
from kfilter.visualize import RvizMarkerPublisher
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray

joint_states_counter = -1
line_segments_counter = -1
detection_period = 10

def collect_joint_states(joint_states, args):
    global joint_states_counter, detection_period
    joint_states_counter = (joint_states_counter + 1) % detection_period
    if joint_states_counter != 0:
        return

    kalman_filter = args[0]
    marker_publisher = args[1]

    kalman_filter.save_joint_states(joint_states)
    pred_matched = kalman_filter.filter()

    marker_publisher.show_estimated_position(pred_matched)
    marker_publisher.show_global_map(kalman_filter.global_map)

def collect_line_segments(line_segments, kalman_filter):
    global line_segments_counter, detection_period
    line_segments_counter = (line_segments_counter + 1) % detection_period
    if line_segments_counter != 0:
        return
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
