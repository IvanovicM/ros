#! /usr/bin/env python

import rospy

from geometry_msgs.msg import Point
from laser_line_extraction.msg import LineSegmentList
from kfilter.kfilter import KalmanFilter
from kfilter.visualize import RvizMarkerPublisher
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray

def collect_joint_states(joint_states, kalman_filter):
    kalman_filter.save_joint_states(joint_states)

def collect_line_segments(line_segments, kalman_filter):
    kalman_filter.save_line_segments(line_segments)

def collect_odometry(od, marker_publisher):
    od_pos = Point(x=od.pose.pose.position.x, y=od.pose.pose.position.y)
    marker_publisher.show_odometry_position(od_pos)

def filter_with_kalman(kalman_filter, marker_publisher):
    while True:
        estimated_position = kalman_filter.filter()
        marker_publisher.show_estimated_position(estimated_position)
        marker_publisher.show_global_map(kalman_filter.global_map)

        rospy.sleep(0.1)

def start_kalman_localization():
    rospy.init_node('kalman_localization', anonymous=False)
    marker_topic_publisher = rospy.Publisher(
        'visualization_marker_array', MarkerArray, queue_size=10
    )

    kalman_filter = KalmanFilter()
    marker_publisher = RvizMarkerPublisher(marker_topic_publisher)
    rospy.Subscriber(
        'joint_states', JointState, collect_joint_states, kalman_filter
    )
    rospy.Subscriber(
        'line_segments', LineSegmentList, collect_line_segments, kalman_filter
    )
    rospy.Subscriber('odom', Odometry, collect_odometry, marker_publisher)

    rospy.loginfo('Kalman localization node is available.')
    filter_with_kalman(kalman_filter, marker_publisher)

if __name__ == '__main__':
    try:
        start_kalman_localization()
    except rospy.ROSInterruptException:
        pass
