#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import numpy as np
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
import hough_drive as hough
from Timer import Timer

MIN = 0.10000000149011612
TIME_WANT_FIRST = 0.6
TIME_WANT = 1.0
LRTHRESHOLD = 90
DISTANCE = 0.26

class Obstacle() :
    def __init__(self) :
        # rospy.init_node('obstacle_node',anonymous=True)
        print("Obstacle start")
        self.rate = rospy.Rate(16)
        self.move = None
        self.time = Timer()
        self.bridge = CvBridge()
        self.image = np.empty(shape = (480,640,3))

        self.lidar = []
        self.nz = None
        self.count_obstacle = 0
        self.obstacle_isit = False

        self.mission_end = False
        rospy.Subscriber('usb_cam/image_raw', Image, self.image_callback)
        rospy.Subscriber('scan',LaserScan,self.lidar_callback)

        self.pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        self.motor_msg = xycar_motor()
        self.motor_msg.speed = 3
        self.turn_angle = 0
        self.hough_angle = 0

        while not self.mission_end:
            # ??? ???
            if self.lidar != [] :
                for i in range(len(self.lidar)) :
                    if self.lidar[i] != 0 :
                        self.obstacle_isit = True
                        find_lr = i
                        # print(i)
                        break
                self.nz = np.nonzero(self.lidar)

                if self.obstacle_isit :
                    if find_lr < LRTHRESHOLD :
                        self.move = "Right"
                        # print(self.nz)
                    else :
                        self.move = "Left" 
                        # print(self.nz)
            # print(self.nz)
            # ??? ???? ???? ???? ??
            if not self.obstacle_isit or self.mission_end == True: 
                # print(self.obstacle_isit, self.mission_end)
                # print("==================just drive==================")
                # print(self.lidar)
                pass
            
            # ???? ?? ??
            else :
                if self.time > 1 :
                    starting_time = rospy.get_time()
                    if self.count_obstacle == 0 :
                        while (rospy.get_time() - starting_time < TIME_WANT_FIRST*1.8) :
                            if self.move == "Right" :
                                print("=========== Move to Right =============", self.count_obstacle)
                                self.motor_msg.angle = 50
                                self.pub.publish(self.motor_msg)
                                self.rate.sleep()
                            elif self.move == "Left" :
                                print("=========== Move to Left ==============", self.count_obstacle)
                                self.motor_msg.angle = -25
                                self.pub.publish(self.motor_msg)
                                self.rate.sleep()
                        starting_time = rospy.get_time()
                        while (rospy.get_time() - starting_time < TIME_WANT_FIRST*1.3) :
                            if self.move == "Right" :
                                print("======== Back =======")
                                self.motor_msg.angle = -40
                                self.pub.publish(self.motor_msg)
                                self.rate.sleep()   
                            elif self.move == "Left" :
                                print("=========== Back ===========", self.count_obstacle)
                                self.motor_msg.angle = 50
                                self.pub.publish(self.motor_msg)
                                self.rate.sleep()
                    elif self.count_obstacle == 1 :  
                        while (rospy.get_time() - starting_time < TIME_WANT) :
                            if self.move == "Right" :
                                print("=========== Move to Right =============", self.count_obstacle)
                                self.motor_msg.angle = 45
                                self.pub.publish(self.motor_msg)
                                self.rate.sleep()
                            elif self.move == "Left" :
                                print("=========== Move to Left ==============", self.count_obstacle)
                                self.motor_msg.angle = -35
                                self.pub.publish(self.motor_msg)
                                self.rate.sleep()
                        starting_time = rospy.get_time()
                        while (rospy.get_time() - starting_time < TIME_WANT*1.4 ) :
                            if self.move == "Right" :
                                print("======== Back =======")
                                self.motor_msg.angle = -35
                                self.pub.publish(self.motor_msg)
                                self.rate.sleep()   
                            elif self.move == "Left" :
                                print("=========== Back ===========", self.count_obstacle)
                                self.motor_msg.angle = 50
                                self.pub.publish(self.motor_msg)
                                self.rate.sleep() 
                    elif self.count_obstacle == 2 :  
                        while (rospy.get_time() - starting_time < TIME_WANT) :
                            if self.move == "Right" :
                                print("=========== Move to Right =============", self.count_obstacle)
                                self.motor_msg.angle = 50
                                self.pub.publish(self.motor_msg)
                                self.rate.sleep()
                            elif self.move == "Left" :
                                print("=========== Move to Left ==============", self.count_obstacle)
                                self.motor_msg.angle = -30
                                self.pub.publish(self.motor_msg)
                                self.rate.sleep()
                        starting_time = rospy.get_time()
                        while (rospy.get_time() - starting_time < TIME_WANT*1.6 ) :
                            if self.move == "Right" :
                                print("======== Back =======")
                                self.motor_msg.angle = -30
                                self.pub.publish(self.motor_msg)
                                self.rate.sleep()   
                            elif self.move == "Left" :
                                print("=========== Back ===========", self.count_obstacle)
                                self.motor_msg.angle = 35
                                self.pub.publish(self.motor_msg)
                                self.rate.sleep()


                    self.count_obstacle += 1

                    if self.count_obstacle == 3 :
                        self.mission_end = True

            self.motor_msg.angle = self.hough_angle
            self.pub.publish(self.motor_msg)
            self.rate.sleep()
        
            


    def image_callback(self, msg) :
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")
        lpos, rpos = hough.process_image(self.image)
        # if self.move == "Left" :
        #     rpos = 320
        # elif self.move == "Right" :
        #     lpos = 320
        center = int((lpos + rpos) / 2)
        # print(lpos, rpos)
        self.hough_angle = -(320 - center) 

    def lidar_callback(self,msg) :
        if self.mission_end :
            return
        # numpy transfer
        self.lidar = msg.ranges[:int(505*3/12)] + msg.ranges[int(505*9/12):]
        self.lidar = np.array(self.lidar)
        # print(self.lidar)

        self.obstacle_isit = False
        for i in range(len(self.lidar)) :
            # ?? ? ??? ?? 0?? ???
            if self.lidar[i] > DISTANCE :
                self.lidar[i] = 0.0

        # self.nz ? ???? ??? ??? ??.
        


# import rospy
# import numpy as np
# from cv_bridge import CvBridge
# from xycar_msgs.msg import xycar_motor
# from sensor_msgs.msg import LaserScan
# from sensor_msgs.msg import Image
# import hough_drive as hough


# MIN = 0.10000000149011612
# TIME_WANT_FIRST = 0.6
# TIME_WANT = 1.0
# LRTHRESHOLD = 90
# DISTANCE = 0.25


# class Obstacle() :
#     def __init__(self) :
#         rospy.init_node('obstacle_node',anonymous=True)

#         self.rate = rospy.Rate(10)
#         self.move = None
        
#         self.bridge = CvBridge()
#         self.image = np.empty(shape = (480,640,3))

#         self.lidar = []
#         self.nz = None
#         self.count_obstacle = 0
#         self.obstacle_isit = False

#         self.mission_end = False
#         rospy.Subscriber('usb_cam/image_raw', Image, self.image_callback)
#         rospy.Subscriber('scan',LaserScan,self.lidar_callback)

#         self.pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
#         self.motor_msg = xycar_motor()
#         self.motor_msg.speed = 3
#         self.turn_angle = 0
#         self.hough_angle = 0

        




#     def image_callback(self, msg) :
#         self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")
#         lpos, rpos = hough.process_image(self.image)
#         # if self.move == "Left" :
#         #     rpos = 320
#         # elif self.move == "Right" :
#         #     lpos = 320
#         center = int((lpos + rpos) / 2)
#         # print(lpos, rpos)
#         self.hough_angle = -(320 - center) 

#     def lidar_callback(self,msg) :
#         # numpy transfer
#         self.lidar = msg.ranges[:int(505*3/11)] + msg.ranges[int(505*8/11):]
#         self.lidar = np.array(self.lidar)
#         # print(self.lidar)

#         self.obstacle_isit = False
#         for i in range(len(self.lidar)) :
#             # ?? ? ??? ?? 0?? ???
#             if self.lidar[i] > DISTANCE :
#                 self.lidar[i] = 0.0

#         # self.nz ? ???? ??? ??? ??.
        




                    # starting_time = rospy.get_time()
                    # if self.count_obstacle == 0 :
                    #     while (rospy.get_time() - starting_time < TIME_WANT_FIRST*1.2) :
                    #         if self.move == "Right" :
                    #             print("=========== Move to Right =============", self.count_obstacle)
                    #             self.motor_msg.angle = 50
                    #             self.pub.publish(self.motor_msg)
                    #             self.rate.sleep()
                    #         elif self.move == "Left" :
                    #             print("=========== Move to Left ==============", self.count_obstacle)
                    #             self.motor_msg.angle = -40
                    #             self.pub.publish(self.motor_msg)
                    #             self.rate.sleep()
                    #     starting_time = rospy.get_time()
                    #     while (rospy.get_time() - starting_time < TIME_WANT_FIRST*1.45 ) :
                    #         if self.move == "Right" :
                    #             print("======== Back =======")
                    #             self.motor_msg.angle = -50
                    #             self.pub.publish(self.motor_msg)
                    #             self.rate.sleep()   
                    #         elif self.move == "Left" :
                    #             print("=========== Back ===========", self.count_obstacle)
                    #             self.motor_msg.angle = 50
                    #             self.pub.publish(self.motor_msg)
                    #             self.rate.sleep()
                    # elif self.count_obstacle == 1 :  
                    #     while (rospy.get_time() - starting_time < TIME_WANT*1.1) :
                    #         if self.move == "Right" :
                    #             print("=========== Move to Right =============", self.count_obstacle)
                    #             self.motor_msg.angle = 40
                    #             self.pub.publish(self.motor_msg)
                    #             self.rate.sleep()
                    #         elif self.move == "Left" :
                    #             print("=========== Move to Left ==============", self.count_obstacle)
                    #             self.motor_msg.angle = -40
                    #             self.pub.publish(self.motor_msg)
                    #             self.rate.sleep()
                    #     starting_time = rospy.get_time()
                    #     while (rospy.get_time() - starting_time < TIME_WANT*1.2 ) :
                    #         if self.move == "Right" :
                    #             print("======== Back =======")
                    #             self.motor_msg.angle = -50
                    #             self.pub.publish(self.motor_msg)
                    #             self.rate.sleep()   
                    #         elif self.move == "Left" :
                    #             print("=========== Back ===========", self.count_obstacle)
                    #             self.motor_msg.angle = 50
                    #             self.pub.publish(self.motor_msg)
                    #             self.rate.sleep() 
                    # elif self.count_obstacle == 2 :  
                    #     while (rospy.get_time() - starting_time < TIME_WANT) :
                    #         if self.move == "Right" :
                    #             print("=========== Move to Right =============", self.count_obstacle)
                    #             self.motor_msg.angle = 35
                    #             self.pub.publish(self.motor_msg)
                    #             self.rate.sleep()
                    #         elif self.move == "Left" :
                    #             print("=========== Move to Left ==============", self.count_obstacle)
                    #             self.motor_msg.angle = -35
                    #             self.pub.publish(self.motor_msg)
                    #             self.rate.sleep()
                    #     starting_time = rospy.get_time()
                    #     while (rospy.get_time() - starting_time < TIME_WANT*1.6 ) :
                    #         if self.move == "Right" :
                    #             print("======== Back =======")
                    #             self.motor_msg.angle = -50
                    #             self.pub.publish(self.motor_msg)
                    #             self.rate.sleep()   
                    #         elif self.move == "Left" :
                    #             print("=========== Back ===========", self.count_obstacle)
                    #             self.motor_msg.angle = 50
                    #             self.pub.publish(self.motor_msg)
                    #             self.rate.sleep()

    #    while not self.mission_end:
    #         # 데이터 전처리
    #         if self.lidar != [] :
    #             for i in range(len(self.lidar)) :
    #                 if self.lidar[i] != 0 :
    #                     self.obstacle_isit = True
    #                     find_lr = i
    #                     # print(i)
    #                     break
    #             self.nz = np.nonzero(self.lidar)

    #             if self.obstacle_isit :
    #                 if find_lr < LRTHRESHOLD :
    #                     self.move = "Right"
    #                     # print(self.nz)
    #                 else :
    #                     self.move = "Left" 
    #                     # print(self.nz)
    #         # print(self.nz)
    #         # ??? ???? ???? ???? ??
    #         if not self.obstacle_isit or self.mission_end == True: 
    #             # if self.mission_end == True :
    #             #     print("=======Obstacle mission end=======")
    #             #     break
    #             # print(self.obstacle_isit, self.mission_end)
    #             # print("==================just drive==================")
    #             # print(self.lidar)
    #             pass
            
    #         # ???? ?? ??

    #         else :
    #             if self.time() > 1 : # 벽같은거 인식하는거 방지 10ㅊㅗ
    #                 if self.move == "Right" :
    #                     starting_time = rospy.get_time()
    #                     if self.count_obstacle == 0 :
    #                         while(rospy.get_time() - starting_time < TIME_WANT_FIRST*1.2) :
    #                             print("=========== Move to Right =============", self.count_obstacle)
    #                             self.motor_msg.angle = 50
    #                             self.pub.publish(self.motor_msg)
    #                             self.rate.sleep()
    #                         starting_time = rospy.get_time() 
    #                         while (rospy.get_time() - starting_time < TIME_WANT_FIRST*1.4 ) :
    #                             print("======== Back =======")
    #                             self.motor_msg.angle = -50
    #                             self.pub.publish(self.motor_msg)
    #                             self.rate.sleep() 
    #                     elif self.count_obstacle == 1 :
    #                         while(rospy.get_time() - starting_time < TIME_WANT_FIRST*2.0) :
    #                             print("=========== Move to Right =============", self.count_obstacle)
    #                             self.motor_msg.angle = 50
    #                             self.pub.publish(self.motor_msg)
    #                             self.rate.sleep() 
    #                         starting_time = rospy.get_time()     
    #                         while (rospy.get_time() - starting_time < TIME_WANT_FIRST*1.7 ) :
    #                             print("======== Back =======")
    #                             self.motor_msg.angle = -50
    #                             self.pub.publish(self.motor_msg)
    #                             self.rate.sleep()  
    #                     elif self.count_obstacle == 2 :
    #                         while(rospy.get_time() - starting_time < TIME_WANT_FIRST*1.45) :
    #                             print("=========== Move to Right =============", self.count_obstacle)
    #                             self.motor_msg.angle = 50
    #                             self.pub.publish(self.motor_msg)
    #                             self.rate.sleep()  
    #                         starting_time = rospy.get_time()
    #                         while (rospy.get_time() - starting_time < TIME_WANT*1.9 ) :
    #                             print("======== Back =======")
    #                             self.motor_msg.angle = -50
    #                             self.pub.publish(self.motor_msg)
    #                             self.rate.sleep() 
    #                 elif self.move == "Left" :
    #                     starting_time = rospy.get_time()
    #                     if self.count_obstacle == 0 :
    #                         while(rospy.get_time() - starting_time < TIME_WANT_FIRST*1.4) :
    #                             print("=========== Move to Left =============", self.count_obstacle)
    #                             self.motor_msg.angle = -50
    #                             self.pub.publish(self.motor_msg)
    #                             self.rate.sleep() 
    #                         starting_time = rospy.get_time()
    #                         while (rospy.get_time() - starting_time < TIME_WANT_FIRST*1.7 ) :
    #                             print("======== Back =======")
    #                             self.motor_msg.angle = 50
    #                             self.pub.publish(self.motor_msg)
    #                             self.rate.sleep() 
    #                     elif self.count_obstacle == 1 :
    #                         while(rospy.get_time() - starting_time < TIME_WANT_FIRST*1.1) :
    #                             print("=========== Move to Left =============", self.count_obstacle)
    #                             self.motor_msg.angle = -50
    #                             self.pub.publish(self.motor_msg)
    #                             self.rate.sleep()  
    #                         starting_time = rospy.get_time()
    #                         while (rospy.get_time() - starting_time < TIME_WANT_FIRST*1.2 ) :
    #                             print("======== Back =======")
    #                             self.motor_msg.angle = 50
    #                             self.pub.publish(self.motor_msg)
    #                             self.rate.sleep()  
    #                     elif self.count_obstacle == 2 :
    #                         while(rospy.get_time() - starting_time < TIME_WANT_FIRST*1.45) :
    #                             print("=========== Move to Left =============", self.count_obstacle)
    #                             self.motor_msg.angle = -50
    #                             self.pub.publish(self.motor_msg)
    #                             self.rate.sleep()
    #                         starting_time = rospy.get_time()  
    #                         while (rospy.get_time() - starting_time < TIME_WANT*1.8 ) :
    #                             print("======== Back =======")
    #                             self.motor_msg.angle = 50
    #                             self.pub.publish(self.motor_msg)
    #                             self.rate.sleep() 

    #                 self.count_obstacle += 1

    #                 if self.count_obstacle == 3 :
    #                     self.mission_end = True

    #         self.motor_msg.angle = self.hough_angle
    #         self.pub.publish(self.motor_msg)
    #         self.rate.sleep()
    #     print('==========obstacle mission end==========')