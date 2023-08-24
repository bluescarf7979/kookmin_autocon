#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import rospy
import numpy as np
import math
import hough_drive as hough

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers
from sensor_msgs.msg import Image
from xycar_msgs.msg import xycar_motor

#================  if marker.id == 3  -> mission start   ============================
#====== id 순서 :  3 6 8 5 4 7 
CAR_POSE = (320,240)
GOAL_THRESHOLD = 75
LAST_AR_THRESHOLD = 30
PLAN_VELOCITY = 20
OFFSET = 300 # 원래 태그는 +320에 위치
OFFSET_FAR = 260
FORWARD_TIME = 20

class ar_tracking :
    def __init__(self) :
        # rospy.init_node('AR_Tracking_node', anonymous=True)
        print("======ar mission start=====")
        self.angle, self.speed = 0,0
        self.ar_pose_cv = (0,0)
        self.tracking_idx = 0
        self.distance_from_ar = 0
        self.angle_error = 0
        self.Width = 640
        self.goal = None
        self.change_goal = False
        self.path = []
        self.goal_id = 3

        self.mission_end = False
        self.yaw = 0
        self.image_original = np.empty(shape = (480,640,3))

        self.pub = rospy.Publisher('xycar_motor',xycar_motor,queue_size=1)
        self.bridge = CvBridge()
        self.rate = rospy.Rate(16)

        rospy.Subscriber("/usb_cam/image_raw", Image , self.img_callback)
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_callback)
        
        while not self.mission_end :
            self.rate.sleep()

        for _ in range(FORWARD_TIME) :
            print("fffffffffff")
            self.angle = 19
            self.speed = 3
            self.pub_xycar()
            self.rate.sleep()

        print('ar mission end.')

    def img_callback(self,data) :
        self.image_original = self.bridge.imgmsg_to_cv2(data,"bgr8")

    # def imu_callback(self, data) :
    #     _,_,yaw = euler_from_quaternion((data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))
    #     self.yaw = yaw % (2*np.pi)
    #     self.yaw = math.degrees(self.yaw)-128
        # print(self.yaw)

    def ar_callback(self,data) :
         # print("ar_ComeIn====================")
        if self.mission_end == False :
            # cv2.imshow('origin',self.image_original)
            # cv2.waitKey(1)

            self.current_frame = np.zeros((480, 640), np.uint8)
            cv2.circle(self.current_frame,CAR_POSE,10,255,-1)
            points = []
            for marker in data.markers :
                self.pose = marker.pose.pose.position
                origin_points_x = int(self.pose.x*100) + 320
                origin_points_y = -int(self.pose.z*100) + 240
                # ar 태그가 실제로 위치하는 지점
                cv2.circle(self.current_frame,(origin_points_x,origin_points_y),
                        3,255,-1)
                # 목표로 찍고 실질적으로 이동할 지점
                # 처음이랑 마지막 태그는 OFFSET 더 작게 설정 -> 왼쪽으로 목표를 더 이격시켜서 생성  
                if marker.id == 3 or marker.id ==  7 :
                    points_x = int(self.pose.x*100) + OFFSET_FAR
                else :
                    points_x = int(self.pose.x*100) + OFFSET
                
                points_y = -int(self.pose.z*100) + 240
                points.append([(points_x, points_y), marker.id])
                # print(marker.id)

            if points == [] : # AR태그가 없는 경우 그냥 주행 
                if self.goal_id == 7 :
                    self.mission_end  = True
                    return
                lpos, rpos = hough.process_image(self.image_original)
                center = (lpos + rpos) / 2
                angle = -(self.Width/2 - center)
                self.angle = angle
                self.speed = 3
                print('ar h drive ing...')           
                self.pub_xycar()
                # 마지막 태그가 시야에서 사라진 경우 -> 일단 차선 나가도록 직진 하드코딩

            else :
                for i in range(len(points)) :
                    cv2.circle(self.current_frame,points[i][0], 10, 255, -1)
                    self.distance_from_ar = math.sqrt((CAR_POSE[0]-points[i][0][0])**2 + (CAR_POSE[1]-points[i][0][1])**2)
                    # 점의 좌표를 넣은 리스트의 2번째 인덱스에 거리 저장
                    points[i].append(self.distance_from_ar)
                    # AR 태그에 바운더리 생성 마지막 태그는 바운더리 절반으로 생성
                    if points[i][1] == 7 :
                        cv2.circle(self.current_frame,points[i][0], LAST_AR_THRESHOLD, 255, 1)
                    else :
                        cv2.circle(self.current_frame,points[i][0], GOAL_THRESHOLD, 255, 1)

                points = sorted(points, key = lambda x : x[2])
                self.goal = [points[0][0][0], points[0][0][1]] # 일단 가장 가까운 점을 goal로 지정
                # print("Before goal : ", points[0][1])
                self.change_goal = False
                if len(points) > 1 :
                    for idx in range(1,len(points)) :
                        # print(points[idx][1])
                        # 태그 바운더리 안으로 접근한 경우
                        if points[idx-1][2] <= GOAL_THRESHOLD :
                            # 변수에 트루값 할당
                            self.change_goal = True

                        # 앞의 태그가 바운더리 안에 있고, 그 뒤의 태그는 바운더리 밖인 경우에 goal을 넘김.
                        if self.change_goal and idx >=1 and points[i][2]>GOAL_THRESHOLD :
                            # 현재 마지막 ar태그에 위치한 경우 
                            if points[idx][1] == 7:
                                if points[idx][2] > LAST_AR_THRESHOLD :
                                    self.goal = [points[idx][0][0], points[idx][0][1]]
                                    self.goal_id = 7
                                    print("Last goal :", self.goal_id)
                                    break
                                else :
                                    self.mission_end = True
                                    break
                            self.goal = [points[idx][0][0], points[idx][0][1]]
                            # print(points[idx][1])
                            self.goal_id = points[idx][1]
                            print("change goal : ",self.goal_id)
                            break

                if not self.mission_end : # 미션이 진행중인 경우에는 경로생성 후 미션주행
                    # self.planning()
                    self.mission_driving()
                    self.pub_xycar()

        # cv2.imshow("AR",self.current_frame)
        # cv2.waitKey(1) 

    # def planning(self) :
    #     # self.path에 그때그때의 일직선 경로를 생성
    #     self.path = []
    #     distance = math.sqrt((CAR_POSE[0]-self.goal[0])**2 + (CAR_POSE[1]-self.goal[1])**2)

    #     num_points = int(distance/PLAN_VELOCITY)
    #     x_dis = self.goal[0]-CAR_POSE[0]
    #     y_dis = self.goal[1]-CAR_POSE[1]
    #     for i in range(num_points) :
    #         nx = int(CAR_POSE[0] + i/(num_points-1) * x_dis)
    #         ny = int(CAR_POSE[1] + i/(num_points-1) * y_dis)
    #         self.path.append([nx,ny])
    #         cv2.circle(self.current_frame,(nx,ny),5,255,-1)

    #     cv2.imshow("AR",self.current_frame)
    #     cv2.waitKey(1)     
        

    def find_angle_error(self,car_angle, position_curr,
                        position_targ):  # 현재 차체의 각도, 현재 차체의 위치, 선택한 타겟점의 위치

        # 타겟 방향의 벡터 = 타겟 위치 - 현재 위치
        targetdir_x = position_targ[0] - position_curr[0]  # x방향 벡터
        targetdir_y = position_targ[1] - position_curr[1]  # y방향 벡터
        
        # position_curr과 position_targ 사이의 각도 (x축 기준)
        target_angle = np.arctan2(targetdir_y, targetdir_x) * 180 / np.pi  
        # 생성한 각도가 반시계방향이 양의 방향이므로, 시계방향을 양의방향으로 지정해서 조정해야 해서 -값을 곱함
        target_angle = -target_angle 
        
        # 각 변환 (position_curr과 position_targ 사이의 각도 (y축 기준))
        target_angle = (450 - target_angle) % 360  # 0도 ~ 360도 사이의 값을 얻게 됨. (North = 0도)
       
        # 마찬가지로 차량의 각도도 y축 기준으로 변환
        # car_angle = (450 - car_angle) % 360

        # y축 기준으로 정렬한 각도들의 차이를 구해서 차량이 틀어야 할 각도를 생성
        angle_error = target_angle - car_angle  # target_angle에서 car_angle(yaw data)을 빼서 각도 오차를 구함

        if angle_error >= 180:  # 각도 오차의 절댓값이 180보다 커지면 보정
            angle_error -= 360
        if angle_error <= -180:  # 각도 오차의 절댓값이 -180보다 작아지면 보정
            angle_error += 360
        # -180도 ~ 180도 사이의 angle_error 를 얻게 됨. 이 만큼을 조정할 것이므로, 이 값을 리턴
        return angle_error


    def mission_driving(self) :
        # 생성된 경로를 바탕으로 tracking을 진행
        self.angle_error = self.find_angle_error(self.yaw, CAR_POSE, self.goal)
        self.speed = 3
        self.angle = self.angle_error
        self.pub_xycar()


    def pub_xycar(self) :
        motor_msg = xycar_motor()
        self.angle = int(self.angle)
        motor_msg.speed = self.speed
        motor_msg.angle = self.angle
        self.pub.publish(motor_msg)


# if __name__ == "__main__" :
#     ar_mission = ar_tracking()
    # rospy.spin()