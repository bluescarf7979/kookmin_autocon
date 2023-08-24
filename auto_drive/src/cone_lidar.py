#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import cv2
import time
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import LaserScan
# Don't change
MAX_DISTANCE = 60
OBSTACLE_RADIUS = 10
ACTUAL_RADIUS = 30
# Arc angle for detection
ARC_ANGLE = 150
bridge = CvBridge()
pub = None
lidar_raw_data = None
fin_degree=0
def drive(Angle, Speed):
    global pub
    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed
    pub.publish(msg)


def lidar_callback(data):
    global MAX_DISTANCE, OBSTACLE_RADIUS, ACTUAL_RADIUS
    ACT_RAD = int(ACTUAL_RADIUS)
    # Use front lidar data & make a local map
    lidar_raw_data = np.array(data.ranges[:181]) * 10
    # Filter 'inf' value
    lidar_raw_data[lidar_raw_data >= MAX_DISTANCE] = -1
    current_frame = np.zeros((ACT_RAD, ACT_RAD * 2), np.uint8)
    # Make array full with -1000
    # Point - (x, y)
    points = np.full((181, 2), -1000, np.int32)
    # r
    r = np.array([lidar_raw_data, lidar_raw_data])
    # theta
    theta = np.radians(range(181))
    cos_sin = np.array([np.cos(theta), np.sin(theta)])
    # Transformation (x,y) = (r*cos(theta), r*sin(theta))
    xy = r * cos_sin
    # Mark points on the map
    points = np.array(np.transpose([np.round(xy[0]) + ACT_RAD, ACT_RAD - np.round(xy)[1]]), dtype=np.int32)
    # Separate points into left and right groups based on x-coordinate
    for point in points:
        cv2.circle(current_frame,tuple(point),OBSTACLE_RADIUS,255,-1)

 
    middle_point=(left_boundary[-1]+right_boundary[-1])//2
    cv2.polylines(current_frame,[left_boundary,right_boundary],isClosed=False,color=150,thickness=2)
    cv2.line(current_frame,tuple(middle_point),(ACT_RAD,middle_point[1]),200,2)
    cv2.imshow("result",current_frame)
    cv2.waitKey(1)

    middle_angle_rad=np.arctan2(middle_point[1]-ACT_RAD,middle_point[0]-ACT_RAD)
    fin_degree=int(np.degrees(middle_angle_rad))

    
    # Print the calculated middle angle
    print("Middle Angle:", fin_degree)
    # Draw circles for the left and right points
   
    cv2.waitKey(1)
    return fin_degree
def cone_drive(fin_degree,speed):
    print("cone driving")
    drive(fin_degree,speed)
def start():
    global pub
    rospy.init_node("conedrive_node")
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    lidar_sub = rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)
    print "----------conedrive ----------"
    rospy.sleep(2)
    speed=3
    cone_drive(fin_degree,speed)
if __name__ == "__main__":
    try:
      start()
      rospy.spin()
    except rospy.ROSInterruptException:
        pass