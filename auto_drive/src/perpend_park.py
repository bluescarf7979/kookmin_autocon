#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import LaserScan
import math
import hough_drive_a2

"""Global Variables"""
LENGTH_OF_SPOT = 0.8 # The parking spot 80cm
#STOP = Twist(linear=Vector3(0,0,0), angular=Vector3(0,0,0))
SPEED = 3 #30cm/s
Perpendmode=False
#FORWARD = Twist(linear=Vector3(SPEED,0.0,0.0), angular=Vector3(0.0,0.0,0.0))
class PerpParkingNode(object):
    # just right parking spot

    def __init__(self):
        self.r = rospy.Rate(5)
        self.pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        # Instance Variables
        self.timestamp1 = None
        self.timestamp2 = None
        self.dist2Car = None
        self.dist2Wall = None
        self.widthOfSpot = None
        self.angle=0
        self.speed=0
        self.radius = None
        # Adjustment to be made before moving along the arc
        self.adjustment = 0
        self.isAligned = False
    def drive(self,Angle,Speed):
        msg=xycar_motor()
        msg.angle=Angle
        msg.speed=Speed
        self.pub.publish(msg)
    def driving(self):
        
    def process_scan(self, data):
        """ This function is the callback for our laser subscriber. """
        currDist = data.ranges[412] # right right point
        # check if the xycar has passed the spot. If not, move forward
        if self.timestamp2 is None:
            #print("start perpend park")
            self.angle=0
            self.speed=SPEED
            # if thers is no distance to car parked, store the lidar reading
            #  as distance to car
            if self.dist2Car is None:
                self.dist2Car = currDist
                print ("dist2Car: ", self.dist2Car)
            # else check whether the current lidar reading is within the range of wall distance
            elif currDist > (self.dist2Car + LENGTH_OF_SPOT - .05):
                # if so, update radius and set the first timestamp
                self.dist2Wall = currDist
                print("dist2wall: ", self.dist2Wall)
                self.radius = (self.dist2Wall/2.0)
                if self.timestamp1 is None:
                    self.timestamp1 = rospy.Time.now()
                    print ("TIME1: ", self.timestamp1)
            # if the first timestamp has been set, check if the current lidar reading is
            # close enough to dist2car, we assume the xycar has passed the spot
            if abs(currDist - self.dist2Car) <= 0.2 and self.timestamp1 is not None:
                self.timestamp2 = rospy.Time.now()
                print ( "TIME2: ",  self.timestamp2 )
                self.angle = 0
                self.speed = 0
        # If we have both timestamps, we can determine the width of the spot, and begin parking.
        elif self.timestamp1 is not None and self.timestamp2 is not None:
            self.adjustment = -0.1
            self.widthOfSpot = SPEED/0.1 * (self.timestamp2.secs - self.timestamp1.secs)
            if self.dist2Car >= 0.2 and not self.isAligned: #determine later what the threshold should be
                self.align_with_origin()
                self.park()
                rospy.signal_shutdown("Done perpendicular parking.")
            elif self.dist2Car < 0.2:
                
                print ("Xycar was too close to park!")
    def align_with_origin(self):
        """After stopping next to the second parked Car,
        this function will align us properly so that we can successfully drive our circle."""
        # distance to move before drive the arc
        dist = self.radius - self.widthOfSpot/2.0 + self.adjustment
        now = rospy.Time.now()
        travelTime = dist/SPEED/0.1 #dist/speed = time
        while rospy.Time.now() - now <= rospy.Duration(travelTime):
            self.angle = 0
            self.speed= SPEED
        self.angle = 0
        self.speed= 0
        self.isAligned = True
    def drive_arc(self, omega, travelTime, sign):
        '''given the omega, travel time and direction, drive in the corresponding arc'''
        # The third parameter (sign) represents whether the forward velocity of the twist will be positive or negative
        now = rospy.Time.now()
        while rospy.Time.now() - now <= travelTime:
            self.angle = omega
            self.speed=sign*SPEED
    def park(self):
        '''Given the parking mode, plan the travel route'''
            # drive into spot
        omega = SPEED/0.1 / (self.radius + 0.15)
        travelTime = rospy.Duration(math.pi/2.0/omega)
        self.drive_arc(omega, travelTime, -1)
            # drive back to fully enter spot
        self.drive_arc(0, rospy.Duration(1), -1)
        self.angle = 0
        self.speed=0
    #wait for 3 seconds
        rospy.sleep(3)
        self.drive_arc(0, rospy.Duration(1), 1)
    # Drive out of the spot
        omega = SPEED/0.1 / (self.radius + 0.15)
        travelTime = rospy.Duration(math.pi / 2.0 / omega)
        self.drive_arc(omega, travelTime, 1)

        print("perpend park end")
        
        #perpendpark finish=>back to hough drive again
        ######################
        self.angle=0
        self.speed=0
        self.drive(self.angle, self.speed)
        print('------------Just hough drive-----------------')
        lpos, rpos = hough.process_image(self.image_original)
        center = (lpos + rpos) / 2
        # print(lpos, rpos)
        angle = -(320 - center)

        self.angle = angle
        self.speed = 4


    def run(self):
        """ This function is the main run loop."""
        
        while not rospy.is_shutdown():
            self.drive(self.angle, self.speed)
            self.r.sleep()
if __name__ == '__main__':
    rospy.init_node('perpend_park')
    perpparking_node = PerpParkingNode()
    perpparking_node.run()