#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2 
import math
import numpy as np
import rospy
import hough_drive as hough
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

elapsed_time = None

class Cross_walk_mission :
    def __init__(self) :
        print('========Cross walk mission start=======')
        # rospy.init_node("cross_walk_node", anonymous=True)
        self.anlge = 0
        self.speed = 0
        self.crosswalk_mission_pass = False
        self.detect_stopline = False
        self.first_detect = True
        self.Width = 640
        self.image = np.empty(shape = (480,640,3))
        self.image_original = np.empty(shape = (480,640,3))
        self.result_img = np.empty(shape = (480,640,3))

        self.rate = rospy.Rate(16)
        self.pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        self.bridge = CvBridge()
        # print('---------------start sub')
        rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)
        while not self.crosswalk_mission_pass :
            self.rate.sleep()

        print('=========cross walk mission end=====')

    def img_callback(self,data) :
        # print("---------------------callback------------------------")
        if not self.crosswalk_mission_pass :
            self.image_original = self.bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")
            self.image = self.bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")
            # cv2.imshow('img',self.image_original)
            # cv2.waitKey(1)
            self.preprocess()
            self.main()

            motor_msg = xycar_motor()
            motor_msg.angle = self.angle
            motor_msg.speed = self.speed
            self.pub.publish(motor_msg)


    def roi_image(self) :
        # 국민대 (226,402), (162,63)    
        # 동방 ???..
        (x,y), (w,h) = (226,468), (162,63)
        self.roi_image_1 = self.image[y:y+h, x:x+w]
    
    def white_filter(self) :
        # self.roi_image_1 = np.float32(self.roi_image_1)
        hsv = cv2.cvtColor(self.roi_image_1, cv2.COLOR_RGB2HSV)
        white_lower = np.array([0,0,200])
        white_upper = np.array([90,80,255])
        white_mask = cv2.inRange(hsv,white_lower,white_upper)
        self.white_masked = cv2.bitwise_and(self.roi_image_1, self.roi_image_1, mask=white_mask)
    
    def preprocess(self) :
        # print('---------------------preprocess')
        self.roi_image()

        self.white_filter()
        # self.blurred_img = cv2.GaussianBlur(self.white_masked,(1,1),3)

        # self.blurred_img = np.uint8(self.blurred_img)

        # self.result_img = cv2.Canny(self.blurred_img, 200, 400, None, 3, False)
        self.result_img = cv2.Canny(self.white_masked, 200, 400, None, 3, False)

        self.result_img_cr = cv2.cvtColor(self.result_img,cv2.COLOR_GRAY2BGR)

        

    def main(self) :
        # print('======main====')
        global elapsed_time
        # rospy.sleep(1)
        # 50
        lines = cv2.HoughLines(self.result_img, 1, np.pi/180, 50)
        if lines is not None : #일단 라인이 검출 된 경우 
            for line in lines :
                rho = line[0][0]
                theta = line[0][1]
                a = math.cos(theta)
                b = math.sin(theta)
                x0 = a * rho
                y0 = b * rho
                pt1 = (int(x0 + 1000 * (-b))), int(y0 + 1000 * (a))
                pt2 = (int(x0 - 1000 * (-b))), int(y0 - 1000 * (a))
                slope = 90 - np.degrees(np.arctan2(b,a))
                cv2.line(self.image_original,pt1, pt2, (255,0,0), 2)
                # print(slope)
                if abs(slope) < 2 : #정지선 처리
                    # print(slope)
                    cv2.line(self.result_img_cr, pt1, pt2, (0,0,255), 2)
                    self.detect_stopline = True
                    # print(self.detect_stopline)
                    if self.first_detect :
                        self.start_time = rospy.get_time()
                        self.first_detect = False
        
        # cv2.imshow('cany_stopLine',self.result_img_cr)
        # cv2.waitKey(1)
        # print(self.detect_stopline, self.crosswalk_mission_pass, elapsed_time)
        #정지선이 있고, 미션이 끝나지 않은 경우에 정지
        if self.detect_stopline and not self.crosswalk_mission_pass :
            # print('-----------recording time---------------')
            if elapsed_time >= 5.5 :
                self.crosswalk_mission_pass= True
                # print("------------------Misssion end--------------------")

            self.speed = 0
            self.angle = 0
            elapsed_time = rospy.get_time() - self.start_time
            
        # 이외의 경우에는 그냥 주행
        else :
            # print('------------Just hough drive-----------------')
            lpos, rpos = hough.process_image(self.image_original)
            center = (lpos + rpos) / 2
            # print(lpos, rpos)
            angle = -(self.Width/2 - center)

            self.angle = angle
            self.speed = 4
        


# if __name__ == '__main__' :
#     Cross_walk_mission()
#     rospy.spin()
