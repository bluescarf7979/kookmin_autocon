#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, rospkg
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

front_lidar_data = []
MIN_DISTANCE = 10
MAX_DISTANCE = 150
OBSTACLE_RADIUS = 50
ACTUAL_RADIUS = 100
# Arc angle for detection
ARC_ANGLE = 120


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
    angle_increment = data.angle_increment
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
        img = cv2.line(img, (x1, y1 + Offset), (x2, y2 + Offset), color, 2)
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
    cv2.rectangle(img, (center - 5, 15 + offset),
                  (center + 5, 25 + offset),
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
            slope = float(y2 - y1) / float(x2 - x1)

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

        if (slope < 0) and (x2 < Width / 2 - 90):
            left_lines.append([Line.tolist()])
        elif (slope > 0) and (x1 > Width / 2 + 90):
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
        x2 = ((Height / 2) - b) / float(m)

        cv2.line(img, (int(x1), Height), (int(x2), (Height / 2)), (255, 0, 0), 3)

    return img, int(pos)


# show image and return lpos, rpos
def process_image(frame):
    global Width
    global Offset, Gap

    # gray
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # blur
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)

    # canny edge
    low_threshold = 60
    high_threshold = 70
    edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)

    # HoughLinesP
    roi = edge_img[Offset: Offset + Gap, 0: Width]
    all_lines = cv2.HoughLinesP(roi, 1, math.pi / 180, 30, 30, 10)

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
    # roi2 = cv2.cvtColor(roi, cv2.COLOR_GRAY2BGR)
    # roi2 = draw_rectangle(roi2, lpos, rpos)

    # show image
    cv2.imshow('calibration', frame)

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
        time.sleep(0.5)

        if lidar_points == None:
            continue
        else:
            print("----------------------LIDAR ON----------------------\n")
            break


def cone():
    global lidar_points, front_lidar_data
    global image
    global MIN_DISTANCE, MAX_DISTANCE, OBSTACLE_RADIUS, ACTUAL_RADIUS, ARC_ANGLE
    
    while True:
        ACT_RAD = np.int32(ACTUAL_RADIUS)
        front_lidar_data = lidar_points[505 * 3 / 4:] + lidar_points[:505 / 4]  # front_lidar_data : 253
        front_lidar_data = np.array(front_lidar_data) * 200  # Àü¹æ ÃøÁ¤ ½ÃÀÛ, ¹Ý½Ã°è ¹æÇâÀ¸·Î 360µµ, 505°³ data
        front_lidar_data[front_lidar_data <= MIN_DISTANCE] = 0.0
        front_lidar_data[front_lidar_data >= MAX_DISTANCE] = 0.0
        # print("LEN:", len(front_lidar_data))
        # print("FRONT:", front_lidar_data)

        current_frame = np.zeros((ACT_RAD, ACT_RAD * 2), np.uint8)

        # Make array full with -1000
        # Point - (x, y)
        points = np.full((len(front_lidar_data), 2), -1000, np.int32)

        # r
        r = np.array([front_lidar_data, front_lidar_data])
        # theta
        # interval = 180 / (253 - 1)
        angle = [float(i * 180 / (253 - 1)) for i in range(253)]
        theta = np.radians(angle)
        cos_sin = np.array([np.cos(theta), np.sin(theta)])
        # Transformation (x,y) = (r*cos(theta), r*sin(theta))
        xy = r * cos_sin

        # Mark points on the map
        points = np.array(np.transpose([np.round(xy[0]) + ACT_RAD, ACT_RAD - np.round(xy)[1]]), dtype=np.int32)
        for point in points:
            if (point[0], point[1]) == (ACT_RAD, ACT_RAD):
                pass
            else:
                cv2.circle(current_frame, tuple(point), OBSTACLE_RADIUS, 255, -1)

        # cv2.imshow("result", current_frame)
        # cv2.waitKey(1)

        AUX_RANGE = np.int32((180 - ARC_ANGLE) / 2)
        data = np.zeros((ARC_ANGLE + 1, 2), np.int32)
        target = None

        if current_frame is not None:
            for theta in range(ARC_ANGLE + 1):
                if theta % 2 != 0:
                    continue
                for r in range(ACT_RAD):
                    x = ACT_RAD + int(r * np.cos((theta + AUX_RANGE) * np.pi/180)) - 1
                    y = ACT_RAD - int(r * np.sin((theta + AUX_RANGE)* np.pi/180)) - 1
                    if data[theta][0] == 0:
                        data[theta][1] = r

                    if current_frame[y][x] != 0:
                        data[theta][0] = 1
            data_transposed = np.transpose(data)

            for i in range(0, ARC_ANGLE + 1):
                x = ACT_RAD + int(data_transposed[1][i] * np.cos(np.radians(i + AUX_RANGE))) - 1
                y = ACT_RAD - int(data_transposed[1][i] * np.sin(np.radians(i + AUX_RANGE))) - 1
                cv2.line(current_frame, (ACT_RAD, ACT_RAD), (x, y), 255)

            color = cv2.cvtColor(current_frame, cv2.COLOR_GRAY2BGR)

            count = np.sum(data_transposed[0])

            if count <= ARC_ANGLE - 1:
                relative_position = np.argwhere(data_transposed[0] == 0) - 90 + AUX_RANGE
                minimum_distance = int(min(abs(relative_position)))

                for i in range(0, len(relative_position)):
                    if abs(relative_position[i]) == minimum_distance:
                        target = int(90 + relative_position[i])

            else:
                target = int(np.argmax(data_transposed[1]) + AUX_RANGE)

            if target >= 0:
                x_target = ACT_RAD + int(data_transposed[1][int(target) - AUX_RANGE] * np.cos(np.radians(int(target))))
                y_target = ACT_RAD - int(data_transposed[1][int(target) - AUX_RANGE] * np.sin(np.radians(int(target))))
                cv2.line(color, (ACT_RAD, ACT_RAD), (x_target, y_target), (0, 0, 255), 2)
            else:
                x_target = ACT_RAD + int(100 * np.cos(np.radians(int(-target)))) - 1
                y_target = ACT_RAD - int(100 * np.sin(np.radians(int(-target)))) - 1
                cv2.line(color, (ACT_RAD, ACT_RAD), (x_target, y_target), (0, 0, 255), 2)
                target *= -1

            # if target >= 0:
            #     target *= 1
            # else:
            #     target *= -1

        print("target:", target)
        print("time:", time.time())  # 60 : 0.5s / 90 : 0.8s / 120 : 1.1s
        cv2.imshow("result", color)
        cv2.waitKey(1)

        # drive(90-target, 3)


def start():
    global pub
    global image, image_raw
    global cap
    global Width, Height
    global img_array, img_raw_array

    rospy.init_node('cone_ing')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    # ÅÍ¹Ì³Î Ã¢ Ãß°¡·Î ¶ç¿ì°í ¸í·É¾î·Î "ROS_NAMESPACE=usb_cam rosrun image_proc image_proc" ÀÔ·ÂÇÑ ÈÄ ·±Ä¡ÆÄÀÏ ½ÇÇà½ÃÄÑ¾ß ÇÕ´Ï´Ù.

    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    # image_sub = rospy.Subscriber("/usb_cam/image_rect_color", Image, img_callback)
    lidar_sub = rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)
    print
    "---------- Xycar A2 v1.0 ----------"
    rospy.sleep(2)

    # lane_tracking_drive(6)  # speed : -50 ~ 50
    check_lidar_data()
    cone()


    # save_video(img_raw_array)  # Camera recording


if __name__ == '__main__':
    try:
        start()
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass