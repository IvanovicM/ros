import numpy as np
import threading

def synchronized(func):
    func.__lock__ = threading.Lock()
    def synced_func(*args, **kws):
        with func.__lock__:
            return func(*args, **kws)
    return synced_func

class KalmanFilter():

    def __init__(self):
        self.position = [0, 0, 0]
        self.map = None # TODO
        self.s_r = None
        self.ds_r = None
        self.s_l = None
        self.ds_l = None
        self.line_segments = None

    def perform(self):
        self._predict_position()
        self._observe_measurement()
        self._match_prediction_and_measurement()
        self._filter_position()

    @synchronized
    def _predict_position(self):
        # TODO
        x = 0

    @synchronized
    def _observe_measurement(self):
        # TODO
        x = 0

    @synchronized
    def _match_prediction_and_measurement(self):
        # TODO
        x = 0

    @synchronized
    def _filter_position(self):
        # TODO
        x = 0

    @synchronized
    def save_joint_states(self, joint_states):
        for i in range(len(joint_states.name)):
            if joint_states.name[i] == 'wheel_left_joint':
                self.ds_r = (
                    0.0 if self.s_r is None
                    else joint_states.position[i] - self.s_r
                )
                self.s_r = joint_states.position[i]
            if joint_states.name[i] == 'wheel_right_joint':
                self.ds_l = (
                    0.0 if self.s_l is None
                    else joint_states.position[i] - self.s_l
                )
                self.s_l = joint_states.position[i]

    @synchronized
    def save_line_segments(self, line_segments):
        self.line_segments = line_segments