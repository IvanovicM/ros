#! /usr/bin/env python

import rospy
from sensors.msg import ProcessedMeasurement
from sensors.msg import DiffActionGoal, DiffActionResult, DiffActionFeedback

def start_diff_action():
    rospy.init_node('diff_action', anonymous=False)

if __name__ == '__main__':
    try:
        start_diff_action()
    except rospy.ROSInterruptException:
        pass

