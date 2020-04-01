#! /usr/bin/env python

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
import numpy as np
import math

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
        self.target_theta = 0.0
        self.v_wanted = 0.3
        self.k_p = 0.05
        self.k_a = 0.1
        self.k_b = -0.05
    
    def print_message(self):
        print('\nAUTO MODE')
        super(AutoState, self).print_message()
        print('To specify target [and v] input (x y theta [v]). [Default v: 0.3m/s]')

    def set_target(self, x, y, theta, v_wanted = 0.3):
        self.target_x = x 
        self.target_y = y
        self.target_theta = theta
        self.v_wanted = v_wanted
        print('New target is set.')

    def _quaternion_to_euler(self, quat):
        t0 = +2.0 * (quat.w * quat.x + quat.y * quat.z)
        t1 = +1.0 - 2.0 * (quat.x * quat.x + quat.y * quat.y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (quat.w * quat.y - quat.z * quat.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (quat.w * quat.z + quat.x * quat.y)
        t4 = +1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        yaw = math.atan2(t3, t4)
        return yaw, pitch, roll

    def _xyz2polar(self, odometry, eps=0.01):
        delta_x = self.target_x - odometry.pose.pose.position.x
        delta_y = self.target_y - odometry.pose.pose.position.y
        if abs(delta_x) < eps and abs(delta_y) < eps:
            return (0, 0, 0)

        rho = math.sqrt(delta_x**2 + delta_y**2)
        theta, _, _ = self._quaternion_to_euler(odometry.pose.pose.orientation)
        alpha = np.arctan2(delta_y, delta_x) - theta
        beta = - theta - alpha

        # # # Backwards?
        if (alpha <= -np.pi/2 or alpha >= np.pi/2):
            alpha = np.mod(np.arctan2(delta_y, delta_x) - np.pi - theta, np.pi / 2)
            beta = np.mod(-(np.pi / 2 - beta) - np.pi + theta, np.pi)
            self.k_p = -abs(self.k_p)
        else:
            self.k_p = abs(self.k_p)

        return (rho, alpha, beta)

    def process_odometry(self, odometry, eps = 0.01):
        rho, alpha, beta = self._xyz2polar(odometry)

        v = self.k_p * rho
        w = self.k_a * alpha + self.k_b * beta
        if abs(v) >= eps:
            w = abs(self.v_wanted / v) * w
            v = np.sign(v) * self.v_wanted

        super(AutoState, self).send_cmd(v, w)

class ExitState(State):

    def __init__(self, command_publisher):
        super(ExitState, self).__init__(command_publisher)
    
    def print_message(self):
        rospy.loginfo('\nExit...')
