#! /usr/bin/env python
# -*- coding:utf-8 -*-

import cv2
import rospy
import numpy as np
from xycar_msgs.msg import xycar_motor
from std_msgs.msg import String
from Timer import Timer
from XycarSensor import XycarSensor
import hough_drive as hough_drive_a2
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class Vertical(object):
    '''
    AutoDriving? ?? ? ???
    '''

    def __init__(self, hz=10):
        # rospy.init_node('Vertical')
        print("yolo parking start")

        self.image = np.empty(shape=(480, 640, 3))
        self.rate = rospy.Rate(hz)
        self.pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        self.msg = xycar_motor()
        self.timer = Timer()
        self.sensor = XycarSensor()

        self.first_time = True
        self.start = True
        self.Width = 640
        self.bridge = CvBridge()
        rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)
        self.yaw0 = 0
        self.parking_yaw = None  # ??? ??? IMU yaw ?? ???? ??
        self.target_lane = 'right'

        self.mission_end = False
        self.yolo = "Left"
        rospy.Subscriber("/yolo_result",String,self.yolo_callback)

        while not self.mission_end :
            if self.yolo == "Right" :
                # print('parking to Right')
                self.findparkingV_Right()
            elif self.yolo == "Left" :
                # print('parking to left')
                self.findparkingV_Left()
            else :
                self.driving()
        print("yolo_parking end")


    def img_callback(self, data):
        # ??? ?? ??
        self.image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

    def yolo_callback(self,data) :
        self.yolo = data.data

#======================= Version 2 =====================================    30 24 13
    def findparkingV_Right(self):
        # if self.first_time:
        #     self.timer.update()
        #     self.first_time = False
            # original ***18second*** 0 for test
        if self.timer() > 16 :
            # ?? ?? ? ??
            if self.sensor.ultra_data is not None:
                if 50 <= self.sensor.ultra_data[4] <= 140:
                    self.poweroff()
                    for _ in range(28):
                        self.msg.angle, self.msg.speed = 16, 4

                        self.pub.publish(self.msg)
                        self.rate.sleep()
                        
                    for _ in range(23):
                        self.msg.angle, self.msg.speed = 50, -4

                        self.pub.publish(self.msg)
                        self.rate.sleep()

                    for _ in range(13):
                        self.msg.angle, self.msg.speed = -50, 4

                        self.pub.publish(self.msg)
                        self.rate.sleep()
                    
                    self.verticalparking_right()
                else:
                    self.driving()
        else:
            self.driving()

    def verticalparking_right(self):
        self.back = abs(self.sensor.ultra_data[7] - self.sensor.ultra_data[5])
        self.lateral = abs(self.sensor.ultra_data[0] - self.sensor.ultra_data[3])
        for _ in range(41):
            self.msg.angle, self.msg.speed = 50, -3
            self.pub.publish(self.msg)
            self.rate.sleep()
        for _ in range(7) :
            self.msg.angle, self.msg.speed = 0, -3
            self.pub.publish(self.msg)
            self.rate.sleep()        
        

        self.tescape_right()


    def tescape_right(self):
        for _ in range(30) :
            self.msg.angle, self.msg.speed = 0, 0  # ??
            self.pub.publish(self.msg)
            self.rate.sleep()


        # for _ in range(4):
        #     self.msg.angle, self.msg.speed = 0, -3  # ??
        #     self.pub.publish(self.msg)
        #     self.rate.sleep()

       
        for _ in range(6):
            self.msg.angle, self.msg.speed = 0, 4  # ??
            self.pub.publish(self.msg)
            self.rate.sleep()

        for _ in range(6) :
            self.msg.angle, self.msg.speed = 0, 3
            self.pub.publish(self.msg)
            self.rate.sleep()  

        for _ in range(50):
            self.msg.angle, self.msg.speed = 50, 3
            self.pub.publish(self.msg)
            self.rate.sleep()
        
        self.mission_end = True
        return
        # self.driving()

#============================== Left parking ========================================
    def findparkingV_Left(self):
        if self.first_time:
            self.timer.update()
            self.first_time = False
            # original ***18second*** 0 for test
        if self.timer() > 16 :
            # ?? ?? ? ??
            if self.sensor.ultra_data is not None:
                if 50 <= self.sensor.ultra_data[0] <= 140:
                    self.poweroff()
                    for _ in range(25):
                        self.msg.angle, self.msg.speed = -4, 4

                        self.pub.publish(self.msg)
                        self.rate.sleep()
                        
                    for _ in range(24):
                        self.msg.angle, self.msg.speed = -40, -4

                        self.pub.publish(self.msg)
                        self.rate.sleep()

                    for _ in range(13):
                        self.msg.angle, self.msg.speed = 50, 4

                        self.pub.publish(self.msg)
                        self.rate.sleep()
                    
                    self.verticalparking_left()
                else:
                    self.driving()
        else:
            self.driving()


    def verticalparking_left(self):
        self.back = abs(self.sensor.ultra_data[7] - self.sensor.ultra_data[5])
        self.lateral = abs(self.sensor.ultra_data[0] - self.sensor.ultra_data[3])
        for _ in range(26):
            self.msg.angle, self.msg.speed = -40, -3
            self.pub.publish(self.msg)
            self.rate.sleep()
        for _ in range(18) :
            self.msg.angle, self.msg.speed = 0, -3
            self.pub.publish(self.msg)
            self.rate.sleep()
        self.tescape_left()



    def tescape_left(self):
        # for _ in range(4):
        #     self.msg.angle, self.msg.speed = 0, -3  # ??
        #     self.pub.publish(self.msg)
        #     self.rate.sleep()
        for _ in range(30) :
            self.msg.angle, self.msg.speed = 0, 0  # ??
            self.pub.publish(self.msg)
            self.rate.sleep()
       
        for _ in range(15):
            self.msg.angle, self.msg.speed = 0, 4  # ??
            self.pub.publish(self.msg)
            self.rate.sleep()

        for _ in range(32):
            self.msg.angle, self.msg.speed = -40, 3
            self.pub.publish(self.msg)
            self.rate.sleep()

        self.mission_end = True
        return
        # self.driving()

#=========================================================================
    def driving(self):
        lpos, rpos = hough_drive_a2.process_image(self.image)
     
        center = (rpos+lpos)/2
       
        angle = -(self.Width/2 - center)
        self.msg.angle = angle
        self.msg.speed = 4
        self.pub.publish(self.msg)
   
    def poweroff(self):
        for _ in range(20):

            self.msg.speed, self.msg.angle = 0, 0
            self.pub.publish(self.msg)
            self.rate.sleep()
        

#========================== version 1 ====================================
    # def findparkingV_Right(self):
    #     if self.first_time:
    #         self.timer.update()
    #         self.first_time = False
    #         # original ***18second*** 0 for test
    #     if self.timer() > 2 and self.direction == 1:
    #         # ?? ?? ? ??
    #         if self.sensor.ultra_data is not None:
    #             if 50 <= self.sensor.ultra_data[4] <= 140:
    #                 self.poweroff()
    #                 for _ in range(20):
    #                     self.msg.angle, self.msg.speed = 8, 4

    #                     self.pub.publish(self.msg)
    #                     self.rate.sleep()
                        
    #                 for _ in range(13):
    #                     self.msg.angle, self.msg.speed = 50, -4

    #                     self.pub.publish(self.msg)
    #                     self.rate.sleep()

    #                 for _ in range(13):
    #                     self.msg.angle, self.msg.speed = -50, 4

    #                     self.pub.publish(self.msg)
    #                     self.rate.sleep()
                    
    #                 self.verticalparking_right()
    #             else:
    #                 self.driving()
    #     else:
    #         self.driving()



    # def verticalparking_right(self):
    #     self.back = abs(self.sensor.ultra_data[7] - self.sensor.ultra_data[5])
    #     self.lateral = abs(self.sensor.ultra_data[0] - self.sensor.ultra_data[3])
    #     for _ in range(43):
    #         self.msg.angle, self.msg.speed = 50, -3
    #         self.pub.publish(self.msg)
    #         self.rate.sleep()

    #     self.tescape_right()

            

    # def driving(self):
    #     lpos, rpos = hough_drive_a2.process_image(self.image)
     
    #     center = (rpos+lpos)/2
       
    #     angle = -(self.Width/2 - center)
    #     self.msg.angle = angle
    #     self.msg.speed = 4
    #     self.pub.publish(self.msg)

   

    # def poweroff(self):
    #     for _ in range(20):

    #         self.msg.speed, self.msg.angle = 0, 0
    #         self.pub.publish(self.msg)
    #         self.rate.sleep()
        


    # def tescape_right(self):
    #     for _ in range(6):
    #         self.msg.angle, self.msg.speed = 0, -3  # ??
    #         self.pub.publish(self.msg)
    #         self.rate.sleep()

       
    #     for _ in range(10):
    #         self.msg.angle, self.msg.speed = 0, 4  # ??
    #         self.pub.publish(self.msg)
    #         self.rate.sleep()

    #     for _ in range(44):
    #         self.msg.angle, self.msg.speed = 50, 3
    #         self.pub.publish(self.msg)
    #         self.rate.sleep()
        
    #     self.driving()
#=============================================================================
        
# if __name__ == "__main__":

#     vertical = Vertical()

#     while not rospy.is_shutdown():
#         vertical.findparkingV()




