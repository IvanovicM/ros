#! /usr/bin/env python

import numpy as np
import re
import rospy
import threading
from state import ManualState, AutoState, ExitState
	
def synchronized(method):    
    outer_lock = threading.Lock()
    lock_name = "__"+method.__name__+"_lock"+"__"
    
    def sync_method(self, *args, **kws):
        with outer_lock:
            if not hasattr(self, lock_name):
                setattr(self, lock_name, threading.Lock())
            lock = getattr(self, lock_name)
            with lock:
                return method(self, *args, **kws)  

    return sync_method

class StateController():

    def __init__(self, command_publisher):
        self.manual_state = ManualState(command_publisher)
        self.auto_state = AutoState(command_publisher)
        self.exit_state = ExitState(command_publisher)
        self.curr_state = self.manual_state
    
    def talk_to_user(self):
        while self.curr_state is not self.exit_state:
            self.curr_state.print_message()
            
            cmd = raw_input('---> ')
            self._parse_cmd(cmd)

    @synchronized
    def _parse_cmd(self, cmd):
        state_changed = self._maybe_change_state(cmd)
        if state_changed:
            return
        
        cmd_sent = self._maybe_send_cmd(cmd)
        if cmd_sent:
            return

        print('Wrong command.')

    @synchronized
    def process_odometry(self, odometry):
        self.curr_state.process_odometry(odometry)

    def _maybe_change_state(self, cmd):
        if cmd == 'man':
            self.curr_state = self.manual_state
            return True
        if cmd == 'auto':
            self.curr_state = self.auto_state
            return True
        if cmd == 'exit':
            self.curr_state = self.exit_state
            return True
        return False
        
    def _maybe_send_cmd(self, cmd):
        cmds = self._maybe_num_cmd(cmd)
        if cmds is None:
            return False

        if self.curr_state == self.auto_state:
            if len(cmds) == 3:
                self.curr_state.set_target(cmds[0], cmds[1], cmds[2])
                return True
            if len(cmds) == 4:
                self.curr_state.set_target(cmds[0], cmds[1], cmds[2], cmds[3])
                return True

        if self.curr_state == self.manual_state:
            if len(cmds) == 2:
                self.curr_state.send_cmd(cmds[0], cmds[1])
                return True

        return False
    
    def _maybe_num_cmd(self, cmd_string):
        cmds = cmd_string.split()
        command_num = len(cmds)

        float_cmds = np.zeros(command_num)
        for cmd_i in range(command_num):
            if (re.match(r'^-?\d+(?:\.\d+)?$', cmds[cmd_i]) is None):
                return None
            float_cmds[cmd_i] = float(cmds[cmd_i])
        
        return float_cmds
