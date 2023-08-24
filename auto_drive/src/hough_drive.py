#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2, random, math
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

import sys
import os
import signal
import time
import datetime

def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

image = np.empty(shape=[0])
image_raw = np.empty(shape=[0])
bridge = CvBridge()
pub = None
Width = 640
Height = 480
Offset = 340
Gap = 40

before_angle = 0
lidar_points = None
img_array = []
img_raw_array = []


def save_video(img_array):
    nowDate = datetime.datetime.now()
    path = r'/home/nvidia/xycar_ws/src/auto_drive/video/'
    out = cv2.VideoWriter(path + nowDate.strftime("%Y-%m-%d_%H-%M") + ".avi", cv2.VideoWriter_fourcc(*'DIVX'), 15,
                          (Width, Height))
    for i in range(len(img_array)):
        out.write(img_array[i])
    out.release()
    print("#--------------------------------#")
    print("Video record Done!!!")
    print("#--------------------------------#")

def save_image(img):
    nowDate = datetime.datetime.now()
    path = r'/home/nvidia/xycar_ws/src/auto_drive/image/'
    cv2.imwrite(path + nowDate.strftime("%Y-%m-%d_%H%M") + ".jpg", img)

def img_callback(data):
    global image, img_array, img_raw_array
    try:
        image = bridge.imgmsg_to_cv2(data, "bgr8")
        image_raw = bridge.imgmsg_to_cv2(data, "bgr8")
        # image = calibration(image, mapx, mapy)
        # image = image[y:+y + h, x:x + w]
        img_array.append(image)
        img_raw_array.append(image_raw)
    except CvBridgeError as e:
        print(e)

def lidar_callback(data):
    global lidar_points
    lidar_points = data.ranges
    # RAD2DEG(x) = x*180/np.pi

# publish xycar_motor msg
def drive(Angle, Speed): 
    global pub

    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed

    pub.publish(msg)

# draw lines
def draw_lines(img, lines):
    global Offset
    for line in lines:
        x1, y1, x2, y2 = line[0]
        color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        img = cv2.line(img, (x1, y1+Offset), (x2, y2+Offset), color, 2)
    return img

# draw rectangle
def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) / 2

    cv2.rectangle(img, (lpos - 5, 15 + offset),
                       (lpos + 5, 25 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (rpos - 5, 15 + offset),
                       (rpos + 5, 25 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (center-5, 15 + offset),
                       (center+5, 25 + offset),
                       (0, 255, 0), 2)    
    cv2.rectangle(img, (315, 15 + offset),
                       (325, 25 + offset),
                       (0, 0, 255), 2)
    return img

# left lines, right lines
def divide_left_right(lines):
    global Width

    low_slope_threshold = 0
    high_slope_threshold = 10

    # calculate slope & filtering with threshold
    slopes = []
    new_lines = []

    for line in lines:
        x1, y1, x2, y2 = line[0]

        if x2 - x1 == 0:
            slope = 0
        else:
            slope = float(y2-y1) / float(x2-x1)
        
        if abs(slope) > low_slope_threshold and abs(slope) < high_slope_threshold:
            slopes.append(slope)
            new_lines.append(line[0])

    # divide lines left to right
    left_lines = []
    right_lines = []

    for j in range(len(slopes)):
        Line = new_lines[j]
        slope = slopes[j]

        x1, y1, x2, y2 = Line

        if (slope < 0) and (x2 < Width/2 - 90):
            left_lines.append([Line.tolist()])
        elif (slope > 0) and (x1 > Width/2 + 90):
            right_lines.append([Line.tolist()])

    return left_lines, right_lines

# get average m, b of lines
def get_line_params(lines):
    # sum of x, y, m
    x_sum = 0.0
    y_sum = 0.0
    m_sum = 0.0

    size = len(lines)
    if size == 0:
        return 0, 0

    for line in lines:
        x1, y1, x2, y2 = line[0]

        x_sum += x1 + x2
        y_sum += y1 + y2
        m_sum += float(y2 - y1) / float(x2 - x1)

    x_avg = x_sum / (size * 2)
    y_avg = y_sum / (size * 2)
    m = m_sum / size
    b = y_avg - m * x_avg

    return m, b

# get lpos, rpos
def get_line_pos(img, lines, left=False, right=False):
    global Width, Height
    global Offset, Gap

    m, b = get_line_params(lines)

    if m == 0 and b == 0:
        if left:
            pos = 0
        if right:
            pos = Width
    else:
        y = Gap / 2
        pos = (y - b) / m

        b += Offset
        x1 = (Height - b) / float(m)
        x2 = ((Height/2) - b) / float(m)

        cv2.line(img, (int(x1), Height), (int(x2), (Height/2)), (255, 0,0), 3)

    return img, int(pos)

# show image and return lpos, rpos
def process_image(frame):
    global Width
    global Offset, Gap
    
    frame = np.float32(frame)

    # gray
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    # blur
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)

    # canny edge
    low_threshold = 60
    high_threshold = 70
    edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)

    # HoughLinesP
    roi = edge_img[Offset : Offset+Gap, 0 : Width]
    all_lines = cv2.HoughLinesP(roi,1,math.pi/180,30,30,10)

    # divide left, right lines
    if all_lines is None:
        return 0, 640
    left_lines, right_lines = divide_left_right(all_lines)

    # get center of lines
    frame, lpos = get_line_pos(frame, left_lines, left=True)
    frame, rpos = get_line_pos(frame, right_lines, right=True)

    # draw lines
    # frame = draw_lines(frame, left_lines)
    # frame = draw_lines(frame, right_lines)
    # frame = cv2.line(frame, (230, 235), (410, 235), (255,255,255), 2)
                                 
    # draw rectangle
    # frame = draw_rectangle(frame, lpos, rpos, offset=Offset)
    #roi2 = cv2.cvtColor(roi, cv2.COLOR_GRAY2BGR)
    #roi2 = draw_rectangle(roi2, lpos, rpos)

    # show image
    # cv2.imshow('calibration', frame)
    
    # write image


    return lpos, rpos

def lane_drive(image, speed):
    # global before_angle
    lpos, rpos = process_image(image)

    center = (lpos + rpos) / 2
    angle = -(Width / 2 - center)

    drive(angle, speed)
    return angle

def lane_tracking_drive(speed):
    global image, image_raw
    global img_array
    print("Lane tracking driving")

    while True:
        while not image.size == (Height * Width * 3):
            continue

        lane_drive(image, speed)

        if (cv2.waitKey(1) & 0xFF == ord('q')) or cv2.waitKey(1) == 27:
            # save_video(img_array)  # Camera recording with hough line
            # save_video(img_raw_array)  # Camera recording (raw data)
            break

def check_lidar_data():
    global lidar_points

    while not rospy.is_shutdown():
        if lidar_points == None:
            continue
        
        print(lidar_points[180])
        print(len(lidar_points))
        print("----------------------------------------------")
        time.sleep(0.5)

def start():
    global pub
    global image, image_raw
    global cap
    global Width, Height
    global img_array, img_raw_array

    rospy.init_node('auto_drive')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    # 터미널 창 추가로 띄우고 명령어로 "ROS_NAMESPACE=usb_cam rosrun image_proc image_proc" 입력한 후 런치파일 실행시켜야 합니다.

    # image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    image_sub = rospy.Subscriber("/usb_cam/image_raw", img_callback)
    lidar_sub = rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)

    rospy.sleep(2)

    lane_tracking_drive(-10)  # speed : -50 ~ 50
    # check_lidar_data()

    # save_video(img_raw_array)  # Camera recording


if __name__ == '__main__':
    try:
        start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

