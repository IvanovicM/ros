#! /usr/bin/env python

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
            self.maybe_change_state(cmd)

    def _parse_message(self, msg):
        # Do something with the message
        # Will this work like it's in another thread? I doubt it
        self.print_message()

    @synchronized
    def process_odometry(self, odometry):
        self.curr_state.process_odometry(odometry)

    @synchronized
    def maybe_change_state(self, cmd):
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
        
    def update_commands(self, cmd):
        # TODO(marina)
        self.cmd = cmd