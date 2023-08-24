#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import cv2
import time
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import LaserScan


angle=0
speed=3

def drive(Angle, Speed):

    global pub
    msg=xycar_motor()
    msg.angle=Angle
    msg.speed=Speed
    pub.publish(msg)

def lidar_callback(data):
     
    global angle
    global speed


    ranges = np.array(data.ranges)
    for i in range(130):
        ranges[i]=0.0
    for i in range(370,len(ranges)):
        ranges[i]=0.0
    
    zeros=np.where(ranges==0.0)

    print("ranges",ranges)
    left_mask = np.where((ranges>=0.1)&(ranges<=0.3)&(ranges<np.median(zeros)))
    right_mask= np.where((ranges>=0.1)&(ranges<=0.3)&(ranges>np.median(zeros)))
    print("left_mask",left_mask)
    print("right_mask",right_mask)
    left_density=len(left_mask[0])
    print("left_density",left_density)
    right_density=len(right_mask[0])
    print("right_density",right_density)

    if left_density>right_density*1.3: #go right
        target_angle=30
    elif right_density>left_density*1.3: #go left
        target_angle=-30
    else:
        target_angle=0

    angle_change=target_angle-angle
    angle+=angle_change*0.1
    speed=3
    return angle,speed

                 
def start():
    global pub
    rospy.init_node("conee_node")
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    lidar_sub = rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)
    print "----------conedrive ----------"
    rospy.sleep(2)
    drive(angle,speed)


if __name__ == "__main__":
    try:
        start()
        rospy.spin()
    except rospy.ROSInterruptException:
            
        pass