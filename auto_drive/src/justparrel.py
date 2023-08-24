#! /usr/bin/env python
# -*- coding:utf-8 -*-

import cv2
import rospy
import numpy as np
from xycar_msgs.msg import xycar_motor
from XycarSensor import XycarSensor

from Timer import Timer
import hough_drive as hough_drive_a2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from XycarSensor import XycarSensor





class Parrel(object):
    '''
    Main class for AutoDriving
    '''

    def __init__(self, hz=10):
        # rospy.init_node('justparrel')
        print('======================P-parking start=========')
        self.image = np.empty(shape = (480,640,3))
        self.rate = rospy.Rate(hz)
        self.rate2 = rospy.Rate(100)
        self.pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        self.msg = xycar_motor()
        self.sensor = XycarSensor()
        self.timer = Timer()
        self.Width = 640
        self.first_time = True
        self.parking_first_done = False
        self.bridge = CvBridge()
        rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)

        self.reverse = False
        self.ahead =0
        
        self.mission_end = False 
        while not self.mission_end :
            self.findparking()
        print('===================P-parking end==================')
        
            
    def img_callback(self,data) :
        # print("---------------------callback------------------------")
        self.image = self.bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")


    def findparking(self):
        if self.first_time:
            self.timer.update()
            self.first_time = False
        # 17
        if self.timer() > 17.5:
            if self.sensor.ultra_data is not None and not self.parking_first_done:
                if 10 <= self.sensor.ultra_data[4] <= 70:
                    self.poweroff()
                    # print('fffffffffffff')
                    for _ in range(15):
                        self.msg.angle, self.msg.speed = 0, 4
                        self.pub.publish(self.msg)
                        self.rate.sleep()
                    self.parallelpark()
                
                else:
                    self.driving()
        else:
            self.driving()



            
            
    def parallelpark(self):
        
        if self.parking_first_done == False:
            for _ in range(28):
                print("parking")
                self.msg.angle, self.msg.speed = 50, -5
                self.pub.publish(self.msg)
                # rospy.sleep()
                self.rate.sleep()
                #15
            for _ in range(13):
                self.msg.angle, self.msg.speed = -50, -5
                self.pub.publish(self.msg)
    
                # rospy.sleep()
                self.rate.sleep()
            self.poweroff()
            for _ in range(4):
                self.msg.angle , self.msg.speed = 0,3
            
            self.parking_first_done = True
        
        self.arparking()
            


    def arparking(self):
        
        if -0.02 < self.sensor.ar_x + 0.01 < 0.02 and 0.1 < self.sensor.ar_y < 0.53 and abs(self.sensor.ar_yaw) < 0.09:
            print("out")
            self.msg.angle, self.msg.speed = 0, 0
            self.pub.publish(self.msg)
            self.rate2.sleep()
            # self.poweroff()
            self.out()
        
        else:
            if self.reverse:
                print("self.foward")
                if self.sensor.ar_y > 0.45:
                    self.reverse = False 
                    self.ahead = (self.sensor.ar_x+0.01) * 150
            else:
                print('self.reverse')
                if self.sensor.ar_y < 0.6:
                    self.reverse = True
            
            if self.reverse:
                print("reverse")
                angle = -300 * (self.sensor.ar_yaw - 0.05) 
                speed = -10
                
                self.msg.angle, self.msg.speed = angle, speed
                self.pub.publish(self.msg)
                self.rate2.sleep()
                # self.out()
                # self.poweroff()
                return
                
            else:
            
                angle = -self.ahead + 20 if self.sensor.ar_y > 0.55 else self.ahead +20
                speed = 3
        
                self.msg.angle, self.msg.speed = angle*1.5, speed
                self.pub.publish(self.msg)
                self.rate.sleep()
                # self.poweroff()
                self.out()                                 
                # self.out()
               
                # self.out()
                # angle = -self.ahead +20 if self.sensor.ar_y < 0 else self.ahead + 20     
                # speed = 3
                # self.msg.angle, self.msg.speed = angle, speed
                # self.pub.publish(self.msg)
                # self.rate2.sleep()
                return
                       

    def out(self):
        print('out')
        self.poweroff()
        for _ in range(15):
            self.msg.speed, self.msg.angle = 0, 0
            self.pub.publish(self.msg)
            self.rate.sleep()
        for _ in range(9):
            self.msg.speed, self.msg.angle = -4, 0
            self.pub.publish(self.msg)
            self.rate.sleep()
        for _ in range(24): 
            self.msg.speed, self.msg.angle = 4, -50
            self.pub.publish(self.msg)
            self.rate.sleep()
        for _ in range(15):
            self.msg.speed, self.msg.angle = 4, 40
            self.pub.publish(self.msg)
            self.rate.sleep()
        # for _ in range(20):
        #     self.msg.speed, self.msg.angle = 4, 0
        #     self.pub.publish(self.msg)
        #     self.rate.sleep()
        for _ in range(15) :
            self.msg.speed, self.msg.angle = 4, 30
            self.pub.publish(self.msg)
            self.rate.sleep()
        self.mission_end = True
        # print('end')
        # self.driving()

    def poweroff(self):
        for _ in range(30):
            self.msg.speed, self.msg.angle = 0, 0
            self.pub.publish(self.msg)
            self.rate.sleep()

    def driving(self):
        # self.image
        lpos, rpos = hough_drive_a2.process_image(self.image)
        center = (lpos + rpos) / 2

        angle = -(self.Width/2 - center)
        self.msg.angle = angle
        self.msg.speed = 4
        self.pub.publish(self.msg)

    
        
# if __name__ == "__main__":
#     parrel = Parrel()

#     while not rospy.is_shutdown():
#         parrel.findparking()

