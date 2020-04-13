#! /usr/bin/env python

import math
import rospy

from lines.geometry import line2polar
from lines.detector import LinesDetector
from lines.marker import MarkerPublisher
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray

def print_lines(lines):
    if lines is None:
        return

    print('==============================')
    for line in lines:
        rho, alpha = line2polar(line)
        print('a = ({:.2f}, {:.2f}) b = ({:.2f}, {:.2f})'.format(
            line[0].x, line[0].y, line[1].x, line[1].y)
        )
        print('rho = {:.2f}, alpha = {:.2f} * pi\n'.format(rho, alpha/math.pi))

def collect_laser_scan(laser_scan, args):
    detector = args[0]
    marker_publisher = args[1]

    points, lines = detector.detect_lines(laser_scan)
    marker_publisher.publish(lines)
    print_lines(lines)

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
