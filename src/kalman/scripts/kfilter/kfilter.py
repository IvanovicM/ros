import numpy as np
import threading

from gmap import GlobalMap
from gposition import GlobalPosition
from math import cos, sin

def synchronized(func):
    func.__lock__ = threading.Lock()
    def synced_func(*args, **kws):
        with func.__lock__:
            return func(*args, **kws)
    return synced_func

class KalmanFilter():

    def __init__(self):
        self.pos = GlobalPosition()
        self.map = GlobalMap()
        self.s_r = None
        self.ds_r = None
        self.s_l = None
        self.ds_l = None
        self.line_segments = None

        self.b = 0.230 # 230 mm
        self.wheel_r = 0.035 # 35 mm

        self.k_r = 1
        self.k_l = 1
        self.P = np.ones(shape=(3,3))

    def perform(self):
        pos_pred, P_pred = self._predict_position()
        self._observe_measurement()
        self._match_prediction_and_measurement()
        self._filter_position()

        return pos_pred

    @synchronized
    def _predict_position(self):
        ds = (self.ds_r + self.ds_l) / 2
        dtheta = (self.ds_r - self.ds_l) / self.b
        
        # Position prediction
        x_pred = self.pos.x + ds * cos(self.pos.theta + dtheta / 2)
        y_pred = self.pos.y + ds * sin(self.pos.theta + dtheta / 2)
        theta_pred = self.pos.theta + dtheta
        pos_pred = GlobalPosition(x_pred, y_pred, theta_pred)

        # Prediction and error
        Q = np.array([
            [self.k_r * abs(self.ds_r), 0],
            [0, self.k_l * abs(self.ds_l)]
        ])
        F_x = np.array([
            [1, 0, -ds * sin(self.pos.theta + dtheta/2)],
            [0, 1, ds * cos(self.pos.theta + dtheta/2)],
            [0, 0, 1],
        ])
        F_u = np.array([
            [(1/2 * cos(self.pos.theta + dtheta/2) - 
              ds/(2*self.b) * sin(self.pos.theta + dtheta/2)/2),
              (1/2 * cos(self.pos.theta + dtheta/2) + 
              ds/(2*self.b) * sin(self.pos.theta + dtheta/2)/2)],
            [(1/2 * sin(self.pos.theta + dtheta/2) + 
              ds/(2*self.b) * cos(self.pos.theta + dtheta/2)/2),
              (1/2 * sin(self.pos.theta + dtheta/2) - 
              ds/(2*self.b) * cos(self.pos.theta + dtheta/2)/2)],
            [1 / self.b, 1 / self.b],
        ])
        P_pred = (
            np.matmul(np.matmul(F_x, self.P), F_x.T) +
            np.matmul(np.matmul(F_u, Q), F_u.T)
        )
        
        self.pos = pos_pred
        return pos_pred, P_pred

    @synchronized
    def _observe_measurement(self):
        pass

    @synchronized
    def _match_prediction_and_measurement(self):
        pass

    @synchronized
    def _filter_position(self):
        pass

    @synchronized
    def save_joint_states(self, joint_states):
        for i in range(len(joint_states.name)):
            if joint_states.name[i] == 'wheel_right_joint':
                self.ds_r = (
                    0.0 if self.s_r is None
                    else joint_states.position[i] * self.wheel_r - self.s_r
                )
                self.s_r = joint_states.position[i] * self.wheel_r
            if joint_states.name[i] == 'wheel_left_joint':
                self.ds_l = (
                    0.0 if self.s_l is None
                    else joint_states.position[i] * self.wheel_r - self.s_l
                )
                self.s_l = joint_states.position[i] * self.wheel_r

    @synchronized
    def save_line_segments(self, line_segments):
        self.line_segments = line_segments