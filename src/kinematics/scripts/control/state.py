#! /usr/bin/env python

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry

class State(object):

    def __init__(self, command_publisher):
        self.command_publisher = command_publisher

    def send_cmd(self, v, w):
        linear = Vector3(v, 0, 0)
        angular = Vector3(0, 0, w)
        cmd = Twist(linear, angular)
        self.command_publisher.publish(cmd)

    def print_message(self):
        print('To choose mode input \'man\' OR \'auto\'.')
        print('To exit input \'exit\'.')

    def process_odometry(self, odometry):
        pass

    def process_vw_update(self, v, w):
        pass

class ManualState(State):

    def __init__(self, command_publisher):
        super(ManualState, self).__init__(command_publisher)
    
    def print_message(self):
        print('\nMANUAL MODE')
        super(ManualState, self).print_message()
        print('To specify lin/ang velocity input (v w).')

    def process_vw_update(self, v, w):
        # TODO(jana): super().send_cmd(v, w) + dobar odnos v i w, kao iz pdf
        pass

class AutoState(State):

    def __init__(self, command_publisher):
        super(AutoState, self).__init__(command_publisher)
        self.target_x = 0.0
        self.target_y = 0.0
    
    def print_message(self):
        print('\nAUTO MODE')
        super(AutoState, self).print_message()
        print('To specify target input (x y).')

    def set_target(self, x, y):
        self.target_x = x 
        self.target_y = y
        print('New target is set.')

    def process_odometry(self, odometry):
        # TODO(jana): kontroler, na osnovu odometry odrediti v i w pa
        # send_cmd(v, w) + dobar odnos v i w, kao iz pdf
        pass

class ExitState(State):

    def __init__(self, command_publisher):
        super(ExitState, self).__init__(command_publisher)
    
    def print_message(self):
        rospy.loginfo('\nExit...')
