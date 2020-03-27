#! /usr/bin/env python

import actionlib
import rospy
from sensors.msg import ProcessedMeasurement
from sensors.msg import BigTempDiffAction, BigTempDiffGoal, BigTempDiffResult, BigTempDiffFeedback

class BigTempDiffActionServer:

    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            'big_temp_diff', BigTempDiffAction, self.execute, False
        )
        self.server.start()
        self.diff_counter = 0

    def execute(self, goal):
        self.diff_counter += goal.inc_num
        self.server.set_succeeded()
        
        rospy.loginfo(
            'Action executed. Diff counter is: {}'.format(self.diff_counter)
        )

def start_diff_action():
    rospy.init_node('big_temp_diff_action_server')
    server = BigTempDiffActionServer()
    rospy.loginfo('Action node is available.')
    rospy.spin()

if __name__ == '__main__':
    try:
        start_diff_action()
    except rospy.ROSInterruptException:
        pass
