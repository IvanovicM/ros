#! /usr/bin/env python

import rospy
from lines.detector import LinesDetector
from lines.marker import MarkerPublisher
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray

def collect_laser_scan(laser_scan, args):
    detector = args[0]
    marker_publisher = args[1]

    points, lines = detector.detect_lines(laser_scan)
    marker_publisher.publish(lines)

def start_line_detection():
    rospy.init_node('line_detection', anonymous=False)
    detector = LinesDetector()
    topic_publisher = rospy.Publisher(
        'visualization_marker_array', MarkerArray, queue_size=10
    )

    marker_publisher = MarkerPublisher(topic_publisher)
    rospy.Subscriber(
        'kobuki/laser/scan', LaserScan, collect_laser_scan, 
        (detector, marker_publisher)
    )

    rospy.loginfo('Line detection node is available.')
    rospy.spin()

if __name__ == '__main__':
    try:
        start_line_detection()
    except rospy.ROSInterruptException:
        pass
