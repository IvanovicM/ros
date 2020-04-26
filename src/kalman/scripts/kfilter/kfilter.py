import math
import numpy as np

from gmap import GlobalMap
from gposition import GlobalPosition
from math import cos, sin
from threading import Lock

def scale_angle(angle):
    while angle <= -math.pi:
        angle = angle + 2*math.pi
    while angle > math.pi:
        angle = angle - 2*math.pi
    return angle

class KalmanFilter():

    def __init__(self):
        self.mutex = Lock()

        self.pos = GlobalPosition()
        self.global_map = GlobalMap()
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
        self.g = 10

    def filter(self):
        self.mutex.acquire()
        pos_pred, P_pred = self._predict_position()

        # Any measurement?
        if self.line_segments is None or len(self.line_segments) == 0:
            self.pos = pos_pred
            self.P = P_pred
            self.mutex.release()
            return self.pos

        # Fix the prediction, based on measurements
        mes_pred, H = self._predict_measurement(pos_pred)
        v, R = self._match_prediction_and_measurement(mes_pred, H, P_pred)
        self._filter_position(pos_pred, P_pred, H, R, v)
        self.mutex.release()
        return self.pos

    def _predict_position(self):
        ds = (self.ds_r + self.ds_l) / 2
        dtheta = (self.ds_r - self.ds_l) / self.b
        
        # Position prediction
        x_pred = self.pos.x + ds * cos(self.pos.theta + dtheta / 2)
        y_pred = self.pos.y + ds * sin(self.pos.theta + dtheta / 2)
        theta_pred = scale_angle(self.pos.theta + dtheta)
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
              ds/(2*self.b) * sin(self.pos.theta + dtheta/2)),
              (1/2 * cos(self.pos.theta + dtheta/2) + 
              ds/(2*self.b) * sin(self.pos.theta + dtheta/2))],
            [(1/2 * sin(self.pos.theta + dtheta/2) + 
              ds/(2*self.b) * cos(self.pos.theta + dtheta/2)),
              (1/2 * sin(self.pos.theta + dtheta/2) - 
              ds/(2*self.b) * cos(self.pos.theta + dtheta/2))],
            [1 / self.b, -1 / self.b],
        ])
        P_pred = (
            np.matmul(np.matmul(F_x, self.P), F_x.T) +
            np.matmul(np.matmul(F_u, Q), F_u.T)
        )
        
        return pos_pred, P_pred

    def _predict_measurement(self, pos_pred):
        measurement_pred = []
        H = []
        for wall in self.global_map.walls:
            alpha_pred = scale_angle(wall.angle - pos_pred.theta)
            rho_pred = wall.radius - (
                pos_pred.x * cos(wall.angle) + pos_pred.y * sin(wall.angle)
            )
            measurement_pred.append([alpha_pred, rho_pred])
            H.append([[0, 0, -1], [-cos(wall.angle), -sin(wall.angle), 0]])

        return measurement_pred, H

    def _match_prediction_and_measurement(self, mes_pred, H_pred, P_pred):
        if self.line_segments is None:
            return None, None, None
        v_matched = []
        R_matched = []

        for i in range(len(mes_pred)):
            m_pred_i = np.array(mes_pred[i])
            H_i = np.array(H_pred[i])
            for j in range(len(self.line_segments)):
                m_real_j = np.array([
                    self.line_segments[j].angle, self.line_segments[j].radius
                ])
                R_j = np.array([
                    self.line_segments[j].covariance[0:2],
                    self.line_segments[j].covariance[2:4]
                ])

                v_ij = m_real_j - m_pred_i
                sigma = np.matmul(np.matmul(H_i, P_pred), H_i.T) + R_j
                d_ij = np.matmul(np.matmul(v_ij.T, np.linalg.inv(sigma)), v_ij)

                if d_ij <= self.g:
                    v_matched.append(v_ij)
                    R_matched.append(R_j)

        return v_matched, R_matched

    def _filter_position(self, pos_pred, P_pred, H, R, v):
        self.pos = pos_pred

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

    def save_line_segments(self, line_segments):
        self.mutex.acquire()
        self.line_segments = line_segments.line_segments
        self.mutex.release()