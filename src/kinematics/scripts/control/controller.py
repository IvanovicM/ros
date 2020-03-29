#! /usr/bin/env python

import re
import rospy
import threading
from state import ManualState, AutoState, ExitState
	
def synchronized(func):
    func.__lock__ = threading.Lock()
		
    def synced_func(*args, **kws):
        with func.__lock__:
            return func(*args, **kws)

    return synced_func

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
        cmd_correct, first_cmd, second_cmd = self._maybe_num_cmd(cmd)
        if not cmd_correct:
            return False
    
        if self.curr_state == self.auto_state:
            self.curr_state.set_target(first_cmd, second_cmd)
            return True
        if self.curr_state == self.manual_state:
            self.curr_state.send_cmd(first_cmd, second_cmd)
            return True
        return False
    
    def _maybe_num_cmd(self, command):
        cmds = command.split()
        if len(cmds) != 2:
            return False, None, None
        
        if (re.match(r'^-?\d+(?:\.\d+)?$', cmds[0]) is None) or (
            re.match(r'^-?\d+(?:\.\d+)?$', cmds[1]) is None):
            return False, None, None
        
        return True, float(cmds[0]), float(cmds[1])
