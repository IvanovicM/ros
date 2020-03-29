#! /usr/bin/env python

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
import numpy as np

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

class ManualState(State):

    def __init__(self, command_publisher):
        super(ManualState, self).__init__(command_publisher)
    
    def print_message(self):
        print('\nMANUAL MODE')
        super(ManualState, self).print_message()
        print('To specify lin/ang velocity input (v w).')

class AutoState(State):

    def __init__(self, command_publisher):
        super(AutoState, self).__init__(command_publisher)
        self.target_x = 0.0
        self.target_y = 0.0
        self.v_wanted = 1
        self.k_p = 1
        self.k_a = 2
        self.k_b = -1
    
    def print_message(self):
        print('\nAUTO MODE')
        super(AutoState, self).print_message()
        print('To specify target input and speed (x y v).'
              'Default speed value: 1m/s.') 

    def set_target(self, x, y, v_wanted = 1):
        self.target_x = x 
        self.target_y = y
        self.v_wanted = v_wanted
        print('New target is set.')

    def xyz2polar(self, odometry):
        delta_x = self.target_x - odometry.position.x
        delta_y = self.target_y - odometry.position.y
        rho = sqrt(delta_x**2 + delta_y**2)
        theta = np.arctan2(odometry.position.x**2 + odometry.position.y**2)
        alpha = np.arctan2(delta_y / delta_x) - theta
        beta = -theta - alpha

        if alpha < -np.pi/2 or alpha > np.pi/2:
            if alpha > np.pi/2:
                alpha = alpha - np.pi/3 - theta
                beta = np.pi - beta
            else:
                alpha = alpha + np.pi/3 + theta
                beta = np.pi + beta
            self.k_p = -abs(self.k_p)
        else:
            self.k_p = abs(self.k_p)
        return (rho, alpha, beta)

    def process_odometry(self, odometry):
        rho, alpha, beta = xyz2polar(odometry)
        v = self.k_p * rho
        w = self.k_a * alpha + self.k_b * beta
        speed_scaling_const = v / w
        w = self.v_wanted / speed_scaling_const

class ExitState(State):

    def __init__(self, command_publisher):
        super(ExitState, self).__init__(command_publisher)
    
    def print_message(self):
        rospy.loginfo('\nExit...')
