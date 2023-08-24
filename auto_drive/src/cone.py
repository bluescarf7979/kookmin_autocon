#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math

import cv2
import matplotlib.pyplot as plt
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from xycar_msgs.msg import xycar_motor

import hough_drive as hough

img= np.empty(shape=[0])
angle_list=[]
ANGLE_LIST_SIZE=10
bridge = CvBridge()
pub = None
WIDTH = 640
HEIGHT = 360
obj_b = cv2.imread('/home/nvidia/xycar_ws/src/auto_drive/src/corn_data/corn1.png', cv2.IMREAD_GRAYSCALE)#wad
obj_s = cv2.imread('/home/nvidia/xycar_ws/src/auto_drive/src/corn_data/corn2.png', cv2.IMREAD_GRAYSCALE)#wad
obj_contours_b,_=cv2.findContours(obj_b, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)#wad
obj_contours_s,_=cv2.findContours(obj_s, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)#wad
obj_pts_b=obj_contours_b[0]#wad
obj_pts_s=obj_contours_s[0]#wad
kernel_size=5
low_threshold=30
high_threshold=255
theta=np.pi/180
threshold=90
p_r_m=0
p_r_n=0
p_l_m=0
p_l_n=0
def img_callback(data):
    global img
    try:
        img= bridge.imgmsg_to_cv2(data, "bgr8")
        cone_drive(img, speed=3)
    except CvBridgeError as e:
        print(e)
# publish xycar_motor msg
def drive(Angle, Speed):
    global pub
    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed
    pub.publish(msg)
def grayscale(img):
    hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower=(0, 150,80)
    upper=(125, 255, 255)
    mask_hsv=cv2.inRange(hsv, lower, upper)
    img = cv2.bitwise_and(img, img, mask=mask_hsv)
    img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    return img
def gaussian_blur(img, kernel_size):
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)
def threshold(img, low_threshold, high_threshold):
    """Applies the Canny transform"""
    return cv2.threshold(img, low_threshold, high_threshold, cv2.THRESH_BINARY)
def region_of_interest(img, vertices):
    mask=np.zeros_like(img)
    if len(img.shape)>2:
        channel_count = img.shape[2]
        ignore_mask_color=(255,)*channel_count
    else:
        ignore_mask_color=255
    cv2.fillPoly(mask, vertices, ignore_mask_color)
    masked_image=cv2.bitwise_and(img,mask)
    return masked_image
def depart_points(img, points):
    r_list = []
    l_list = []
    center_x = int(WIDTH / 2)
    for p in points:
        x, y = p
        if x > center_x:  # Right side of the centerline
            r_list.append(p)
        else:  # Left side of the centerline
            l_list.append(p)
    return r_list, l_list
def gradient(p1, p2):
    return (p2[1]-p1[1])/(p1[0]-p2[0])
def y_intercept(m, p):
    return (-1)*p[1]-m*p[0]
def linear_reg(img, left, right):
    if not left or not right:  # Check if either list is empty
        return img  # Skip linear_reg calculations and return the original image
    left_x = []
    left_y = []
    right_x = []
    right_y = []
    global p_r_m
    global p_r_n
    global p_l_m
    global p_l_n
    for i in range(len(left)):
        left_x.append(left[i][0])
        left_y.append(left[i][1])
    for i in range(len(right)):
        right_x.append(right[i][0])
        right_y.append(right[i][1])
    left_calculated_weight = 0
    right_calculated_weight = 0
    if len(left) < 2:
        left_calculated_weight = p_l_m
        left_calculated_bias = p_l_n
    else:
        mean_lx = np.mean(left_x)
        mean_ly = np.mean(left_y)
        left_calculated_weight = least_square(left_x, left_y, mean_lx, mean_ly)
        left_calculated_bias = mean_ly - left_calculated_weight * mean_lx
    target_l = left_calculated_weight * WIDTH + left_calculated_bias
    #print(f"y = {left_calculated_weight} * X + {left_calculated_bias}")
    if len(right) < 2:
        right_calculated_weight = p_r_m
        right_calculated_bias = p_r_n
    else:
        mean_rx = np.mean(right_x)
        mean_ry = np.mean(right_y)
        right_calculated_weight = least_square(right_x, right_y, mean_rx, mean_ry)
        right_calculated_bias = mean_ry - right_calculated_weight * mean_rx
    target_r = right_calculated_weight * WIDTH + right_calculated_bias
    #print(f"y = {right_calculated_weight} * X + {right_calculated_bias}")
    img = cv2.line(img, (int(WIDTH/2), HEIGHT), (int(WIDTH/2), int(0)), (255, 0, 0), 3)
    if left_calculated_weight != right_calculated_weight:  # Avoid division by zero
        cross_x = (right_calculated_bias - left_calculated_bias) / (left_calculated_weight - right_calculated_weight)
        cross_y = left_calculated_weight * ((right_calculated_bias - left_calculated_bias) / (left_calculated_weight - right_calculated_weight)) + left_calculated_bias
    else:
        # Handle the situation where the slopes are the same (division by zero)
        # You can set cross_x and cross_y to some default values or handle it differently based on your application
        cross_x = 0
        cross_y = 0
    if np.isnan(cross_x) != True and np.isnan(cross_y) != True:
        img = cv2.line(img, (0, int(left_calculated_bias)), (int(WIDTH), int(target_l)), (0, 0, 0), 10)
        img = cv2.line(img, (int(0), int(right_calculated_bias)), (WIDTH, int(target_r)), (0, 0, 0), 10)
        cv2.circle(img, (int(cross_x), int(cross_y)), 10, (0, 0, 255), -1, cv2.LINE_AA)
        #if 80 < steering_theta(left_calculated_weight, right_calculated_weight) < 100:
         #   print('sosil angle: ', steering_vanishing_point(cross_x))
        steer_angle=steering_vanishing_point(cross_x)
        #else:
         #   print("gioolgi angle: ", steering_theta(left_calculated_weight, right_calculated_weight))
          #  steer_angle=steering_theta(left_calculated_weight, right_calculated_weight)
    p_l_m = left_calculated_weight
    p_r_m = right_calculated_weight
    p_l_n = left_calculated_bias
    p_r_n = right_calculated_bias
    #print('Done.')
    return img,steer_angle
def least_square(val_x, val_y, mean_x, mean_y):
    return ((val_x - mean_x) * (val_y - mean_y)).sum() / ((val_x - mean_x)**2).sum()
def steering_theta(w1, w2):
    if np.abs(w1) > np.abs(w2):  # 우회전
        if w1 * w2 < 0:  #정방향 or 약간 틀어진 방향
            w1 = -w1
            angle = np.arctan(np.abs(math.tan(w1) - math.tan(w2)) / (1 + math.tan(w1)*math.tan(w2)))
            theta = matching(angle, 0, np.pi/2, 90, 180)
        elif w1 * w2 > 0:  #극한으로 틀어진 방향
            if w1 > w2:
                theta = 90
            else:
                theta = 90
        else:
            theta = 0
    elif np.abs(w1) < np.abs(w2) :  # 좌회전
        if w1 * w2 < 0:  #정방향 or 약간 틀어진 방향
            w1 = -w1
            angle = np.arctan(np.abs(math.tan(w1) - math.tan(w2)) / (1 + math.tan(w1)*math.tan(w2)))
            theta = matching(angle, 0, np.pi/2, 90, 0)
        elif w1 * w2 > 0:  #극한으로 틀어진 방향
            if w1 > w2:
                theta = 90
            else:
                theta = 90
        else:
            theta = 0
    else:
        theta = 90
    return theta
def matching(x,input_min,input_max,output_min,output_max):
    return (x-input_min)*(output_max-output_min)/(input_max-input_min)+output_min #map()함수 정의.
def steering_vanishing_point(x):
    standard_x = int(WIDTH/2)
    diff = standard_x - x
    if diff > 0:   #좌회전
        theta = matching(diff, 0, WIDTH/2, 90, 45)
    elif diff < 0:
        theta = matching(diff, 0, -WIDTH/2, 90, 135)
    return theta
def process_image(img):
    gray = grayscale(img)
    blur_gray = gaussian_blur(gray, kernel_size)
    if len(img.shape) > 2:
        channel_count = img.shape[2]
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255
    mask = np.zeros_like(img)
    vertices = np.array([[(10, 305),
                          (10, 200),
                          (180, 150),
                          (450, 150),
                          (630, 200),
                          (630, 305)]], dtype=np.int32)
    cv2.fillPoly(mask, vertices, ignore_mask_color)
    masked_image = region_of_interest(blur_gray, vertices)
    ret2, edges = threshold(masked_image, low_threshold, high_threshold)
    bigimg = edges
    contours, _ = cv2.findContours(bigimg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    r_mid = []
    l_mid = []
    m_mid = []
    for pts in contours:
        if cv2.contourArea(pts) < 100:
            continue
        rc = cv2.boundingRect(pts)
        dist_b = cv2.matchShapes(obj_pts_b, pts, cv2.CONTOURS_MATCH_I3, 0)
        dist_s = cv2.matchShapes(obj_pts_s, pts, cv2.CONTOURS_MATCH_I3, 0)
        if dist_b < 0.4 or dist_s < 0.2:
            cv2.rectangle(bigimg, rc, (255, 0, 0), 1)
            mid = [int((rc[0]*2 + rc[2]) / 2), int((rc[1]*2 + rc[3]) / 2)]
            m_mid.append(mid)
    r_mid, l_mid = depart_points(img, m_mid)
    #print(f"r_mid: {r_mid}")
    #print(f"l_mid: {l_mid}")
    return r_mid,l_mid
def cone_drive(img, speed):
    global cone_drive_start
    r_mid, l_mid = process_image(img)
    if len(r_mid) > 0 and len(l_mid) > 0:
        cone_drive_start = True
        print("-----------------Cone Driving----------------")
        for r in r_mid:
            if r:
                cv2.circle(img, tuple(r), 10, (0, 0, 255), -1, cv2.LINE_AA)  # 빨강
        for l in l_mid:
            if l:
                cv2.circle(img, tuple(l), 10, (0, 255, 0), -1, cv2.LINE_AA)  # 초록
        linear_img, steer_angle = linear_reg(img, l_mid, r_mid)
        cv2.imshow("lineariimg",linear_img)
        angle_list.append(steer_angle)
        if len(angle_list)>ANGLE_LIST_SIZE:
            angle_list.pop(0)
        smoothed_angle=sum(angle_list)/len(angle_list)
        speed = 3
        drive(smoothed_angle, speed)
        cv2.imshow("result img", linear_img)
        cv2.waitKey(1)
        
    else:
        cone_drive_start = False
    if not cone_drive_start:
        print('------------Just hough drive-----------------')
        lpos, rpos = hough.process_image(img)
        center = (lpos + rpos) / 2
        print(lpos, rpos)
        angle = -(WIDTH/2 - center)
        speed = 3
        drive(angle, speed)
def start():
    global pub
    global img
    global WIDTH, HEIGHT
    rospy.init_node('cone_node')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    # image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    image_sub = rospy.Subscriber("/usb_cam/image_rect_color", Image, img_callback)
    print "---------cone mission---------"
    rospy.sleep(2)
if __name__ == '__main__':
    try:
        start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
