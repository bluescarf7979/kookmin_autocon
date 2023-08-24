#! /usr/bin/env python
# -*- coding:utf-8 -*-

import cv2
import rospy
import numpy as np
from xycar_msgs.msg import xycar_motor
from Timer import Timer
from XycarSensor import XycarSensor
from LaneDetector import LaneDetector
from ModeControllers import ModeControllerS
from StanleyControllerS import StanleyControllerS

class Stanley(object):
    '''
    Main class for AutoDriving
    '''

    def __init__(self, hz=10):
        
        self.rate = rospy.Rate(hz)
        self.pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        self.msg = xycar_motor()

        self.timer = Timer()
        self.sensor = XycarSensor()
        yaw0 = self.sensor.init(self.rate)

        self.lane_detector = LaneDetector()

        self.stanley_controller = StanleyControllerS(self.timer)
        self.mode_controller = ModeControllerS(yaw0, self.timer)
       
        # steer_img = cv2.imread(rospy.get_param("steer_img"))
        # self.steer_img = cv2.resize(steer_img, dsize=(0,0), fx=0.2, fy=0.2)

        self.target_lane = 'middle'
        self.control_dict = {
            'long straight' : self.stanley,
            'short straight': self.stanley,
            'curve': self.stanley,
            'poweroff' : self.poweroff,
            'long straight2' : self.stanley,
            'sharp curve1': self.stanley,
            'sharp curve2': self.stanley
          
        }

    def stanley(self):
        angle, target = self.lane_detector(self.sensor.cam, self.target_lane)
        self.msg.angle, self.msg.speed = self.stanley_controller(angle, target, self.mode_controller.get_mode())
        # print("output : ", self.msg.speed)
        self.pub.publish(self.msg)
        self.rate.sleep()


    def control(self):
        '''
        main controller
        uses method based on current mode
        '''
        # cv2.imshow('cam', self.sensor.cam)
        mode = self.mode_controller(self.sensor.yaw)
        self.control_dict[mode]()
        # cv2.waitKey(1)
        
    
    def poweroff(self):
        self.msg.speed, self.msg.angle = 0, 0
        self.pub.publish(self.msg)
        self.rate.sleep()

if __name__ =="__main__":
    rospy.init_node('juststanley')
    xycar = Stanley()

    while not rospy.is_shutdown():
        xycar.control()




# #! /usr/bin/env python
# # -*- coding:utf-8 -*-

# import cv2
# import rospy
# import numpy as np
# from xycar_msgs.msg import xycar_motor
# from Timer import Timer
# from XycarSensor import XycarSensor
# from LaneDetector import LaneDetector
# from ModeControllers import ModeControllerS
# from StanleyControllerS import StanleyControllerS

# class Stanley(object):
#     '''
#     Main class for AutoDriving
#     '''

#     def __init__(self, hz=10):
        
#         self.rate = rospy.Rate(hz)
#         self.pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
#         self.msg = xycar_motor()

#         self.timer = Timer()
#         self.sensor = XycarSensor()
#         yaw0 = self.sensor.init(self.rate)

#         self.lane_detector = LaneDetector()

#         self.stanley_controller = StanleyControllerS(self.timer)
#         self.mode_controller = ModeControllerS(yaw0, self.timer)
       
#         # steer_img = cv2.imread(rospy.get_param("steer_img"))
#         # self.steer_img = cv2.resize(steer_img, dsize=(0,0), fx=0.2, fy=0.2)

#         self.target_lane = 'middle'
#         self.control_dict = {
#             'long straight' : self.stanley,
#             'short straight': self.stanley,
#             'curve': self.stanley,
#             'poweroff' : self.poweroff
          
#         }

#     def stanley(self):
#         angle, target = self.lane_detector(self.sensor.cam, self.target_lane)
#         self.msg.angle, self.msg.speed = self.stanley_controller(angle, target, self.mode_controller.get_mode())
#         self.pub.publish(self.msg)
#         self.rate.sleep()


#     def control(self):
#         '''
#         main controller
#         uses method based on current mode
#         '''
#         cv2.imshow('cam', self.sensor.cam)
#         mode = self.mode_controller(self.sensor.yaw)
#         self.control_dict[mode]()
#         cv2.waitKey(1)
    
#     def poweroff(self):
#         self.msg.speed, self.msg.angle = 0, 0
#         self.pub.publish(self.msg)
#         self.rate.sleep()

# if __name__ =="__main__":
#     rospy.init_node('juststanley')
#     xycar = Stanley()

#     while not rospy.is_shutdown():
#         xycar.control()