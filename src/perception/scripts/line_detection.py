#! /usr/bin/env python

import rospy
from lines.detector import LinesDetector
from sensor_msgs.msg import LaserScan

def collect_laser_scan(laser_scan, detector):
    detector.detect_lines(laser_scan)

def start_line_detection():
    rospy.init_node('line_detection', anonymous=False)
    detector = LinesDetector()
    rospy.Subscriber(
        'kobuki/laser/scan', LaserScan, collect_laser_scan, detector
    )

    rospy.loginfo('Line detection node is available.')
    rospy.spin()

if __name__ == '__main__':
    try:
        start_line_detection()
    except rospy.ROSInterruptException:
        pass
