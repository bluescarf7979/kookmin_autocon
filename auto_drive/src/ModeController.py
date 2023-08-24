#! /usr/bin/env python
# -*- coding:utf-8 -*-

import numpy as np

class ModeController(object):
    '''
    Detects mode based on imu sensor data
    Counts laps
    '''

    def __init__(self, yaw0, timer):
        self.mode = 'findingpark'
        self.timer = timer
        self.yaw0 = yaw0
        # self.lap = 0
        # self.lap_target = 3

    def set_mode(self, mode):
        self.mode = mode

    def set_yaw0(self, yaw0):
        self.yaw0 = yaw0

    def get_mode(self):
        return self.mode
