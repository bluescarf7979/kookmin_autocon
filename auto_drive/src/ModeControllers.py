#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import numpy as np
import math
from xycar_msgs.msg import xycar_motor
class ModeControllerS(object):
    '''
    Detects mode based on imu sensor data
    Counts laps
    '''

    def __init__(self, yaw0, timer):
        self.mode = 'long straight'
        self.timer = timer
        self.yaw0 = yaw0
        self.lap = 0
        self.angle = 0
        self.lap_target = 3
        self.angle_buffer = [0]
        self.error = 0.2
        self.yaw_prev = 0
        self.on = False
  
        rospy.Subscriber("/xycar_motor",xycar_motor,self.motor_callback)
    def motor_callback(self,data) :
        self.angle_buffer.append(self.angle)
        self.angle = data.angle

    def set_mode(self, mode):
        self.mode = mode

    def set_yaw0(self, yaw0):
        self.yaw0 = yaw0

    def get_mode(self):
        return self.mode

    # def __call__(self, yaw):
    #     '''
    #     updates and returns current mode
    #     '''
        # diff_yaw = abs(yaw - self.yaw0)
    #     print("angle",self.angle)
    #     if diff_yaw > math.pi:
    #         diff_yaw = 2*math.pi - diff_yaw
    #     print('abs:',abs(self.angle - self.angle_buffer[-1]))
    #     if self.mode == 'long straight' and abs(self.angle - self.angle_buffer[-1]) > 9:
    #         self.mode = 'sharp curve1'
    #         print('sharp curve1')
    #         self.timer.update()
    #     elif self.mode =='sharp curve1' and self.timer() > 1.0 :
    #         self.mode = 'short straight'
    #         print('short straight')

           
    #     elif self.mode == 'short straight' and  abs(self.angle - self.angle_buffer[-1]) > 9:
    #         self.mode = 'sharp curve2'
    #         print('sharp curve2')
    #         self.timer.update()
    #     elif self.mode =='sharp curve2' and self.timer() > 1.0 :
    #         self.mode = 'long straight2'
    #         print("long straight2")

    #     elif self.mode == 'long straight2' and abs(self.angle - self.angle_buffer[-1]) > 9:
    #         self.mode = 'curve'
    #         self.timer.update()
    #         print("curve")
    
    #     elif self.mode == 'curve' and  self.timer() > 5.0:
    #         self.lap += 1
    #         print('finish lap {}'.format(self.lap))
    #         self.timer.update()
    #         if self.lap < self.lap_target:
    #             self.mode = 'long straight'
    #             print("long straight")
    #             #self.stanley_k = 0.7
    #         else:
    #             print('stop')
    #             self.mode = 'poweroff'
    #     return self.mode




        ###imu version###
    def __call__(self, yaw):
        '''
        updates and returns current m22ode
        '''

        # print("yaw:", yaw)
        # print("prev_yaw:", self.yaw_prev)
        if self.on:
            if 1 <= abs(yaw - self.yaw_prev) <= 5:
                yaw = self.yaw_prev
        
        if yaw - self.yaw_prev >= 0:
            yaw1 = yaw
        else:
            yaw1 = yaw + 2*np.pi

        diff_yaw = abs(yaw1 - self.yaw0)
        
        if diff_yaw > np.pi:
            diff_yaw = 2*np.pi - diff_yaw

        # print("yaw:", yaw)
        # print("yaw0:", self.yaw0)
        # print("diff_yaw:", diff_yaw)

        self.yaw_prev = yaw
        self.on = True
        # print("prev_yaw:", self.yaw_prev)
 
        if self.mode == 'long straight' and  np.pi/2.0 - self.error - 0.15 < diff_yaw < np.pi/2.0 + 0.05:
            self.mode = 'short straight'
            self.yaw0 = yaw
            print("short",diff_yaw)
            self.timer.update()
            # print("======================Straight2 Start======================")
            # print("======================Straight2 Start======================")
            # print("======================Straight2 Start======================")
            # print("======================Straight2 Start======================")
            # print("======================Straight2 Start======================")

        elif self.mode == 'short straight' and  np.pi/2.0 - self.error - 0.15 < diff_yaw < np.pi/2.0 + 0.05:
            self.mode = 'long straight2'
            self.yaw0 = yaw
            print("long straight2",diff_yaw)
            self.timer.update()
            # print("======================Straight3 Start======================")
            # print("======================Straight3 Start======================")
            # print("======================Straight3 Start======================")
            # print("======================Straight3 Start======================")
            # print("======================Straight3 Start======================")

        elif self.mode == 'long straight2' and diff_yaw > 0.7 - self.error:
            self.mode = 'curve'
            self.yaw0 = yaw
            print("curve",diff_yaw)
            self.timer.update()
            print("======================Curve Start======================")
            print("======================Curve Start======================")
            print("======================Curve Start======================")
            print("======================Curve Start======================")
            print("======================Curve Start======================")
    
        elif self.mode == 'curve' and  np.pi - self.error - 0.85 < diff_yaw < np.pi + 0.05:
            self.lap += 1
            print('finish lap {}'.format(self.lap))
            print("======================Curve End======================")
            print("======================Curve End======================")
            print("======================Curve End======================")
            print("======================Curve End======================")
            print("======================Curve End======================")
            self.timer.update()
            if self.lap < self.lap_target:
                self.mode = 'long straight'
                self.yaw0 = yaw
                print("long straight")
                #self.stanley_k = 0.7
            else:
                print('stop')
                self.yaw0 = yaw
                self.mode = 'long straight2'
        
        # elif self.mode == 'long straight' and -0.15 < diff_yaw < 0.05:
            # print("======================Straight1 Start======================")
        return self.mode








# angle version
    # def __call__(self, yaw):
    #     '''
    #     updates and returns current mode
    #     '''
    #     diff_yaw = abs(yaw - self.yaw0)

    #     if diff_yaw > math.pi:
    #         diff_yaw = 2*math.pi - diff_yaw
    #     print('abs:',abs(self.angle - self.angle_buffer[-1]))
    #     if self.mode == 'long straight' and  abs(self.angle - self.angle_buffer[-1]) > 10:
    #         self.mode = 'short straight'
    #         print("short",diff_yaw)
    #         self.timer.update()
    #     elif self.mode == 'short straight' and  abs(self.angle - self.angle_buffer[-1]) > 10:
    #         self.timer.update()
    #         self.mode = 'long straight2'
    #         print("long straight2",diff_yaw)

    #     elif self.mode == 'long straight2' and abs(self.angle - self.angle_buffer[-1]) > 10:
    #         self.mode = 'curve'
    #         self.timer.update()
    #         print("cureve",diff_yaw)
    
    #     elif self.mode == 'curve' and  abs(self.angle - self.angle_buffer[-1]) > 10:
    #         self.lap += 1
    #         print('finish lap {}'.format(self.lap))
    #         self.timer.update()
    #         if self.lap < self.lap_target:
    #             self.mode = 'long straight'
    #             print("long straight")
    #             #self.stanley_k = 0.7
    #         else:
    #             print('stop')
    #             self.mode = 'poweroff'
    #     return self.mode
