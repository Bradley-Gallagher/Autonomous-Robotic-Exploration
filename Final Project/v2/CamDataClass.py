#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist, Pose2D, Pose, Point, Quaternion
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from math import sqrt, radians, pi
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler
from actionlib_msgs.msg import GoalID

from OdomDataClass import OdomData
from LasersClass import Lasers
from OccDataClass import OccData

import tf.transformations
import cv2, cv_bridge
import argparse
import actionlib
import actionlib_tutorials.msg
import actionlib_msgs
import numpy
import math
import time


#Set Up Camera & Mask Related Data
class CamData:
    def __init__(self):
        self.imagesub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.camera_callback)
        self.pub = rospy.Publisher('camera2', Image, queue_size=10)
        self.moverPub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.bridge = cv_bridge.CvBridge()
        self.greenerrorx = 0
        self.greenerrory = 0
        self.rederrorx = 0
        self.rederrory = 0
        self.blueerrorx = 0
        self.blueerrory = 0
        #cv2.namedWindow("original",1)     

    def camera_callback(self, msg): 
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        (h, w) = img.shape[:2]
        img_resized = cv2.resize(img, (int(w/4), int(h/4)))
        img = img_resized
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lowergreen = numpy.array([42, 100, 130], dtype = "uint8")
        uppergreen = numpy.array([48, 255, 255], dtype = "uint8") 
        lowerred = numpy.array([0, 150, 150], dtype = "uint8") 
        upperred = numpy.array([5, 255, 200], dtype = "uint8")
        lowerblue = numpy.array([95, 150, 150], dtype = "uint8") 
        upperblue = numpy.array([105, 255, 255], dtype = "uint8")
        greenmask = cv2.inRange(hsv, lowergreen, uppergreen)
        bluemask = cv2.inRange(hsv, lowerblue, upperblue)
        redmask = cv2.inRange(hsv, lowerred, upperred)
        greenres = cv2.bitwise_and(img, img, None, mask=greenmask)
        gm = cv2.moments(greenmask, False)
        redres = cv2.bitwise_and(img, img, None, mask=redmask)
        rm = cv2.moments(redmask, False)
        blueres = cv2.bitwise_and(img, img, None, mask=bluemask)
        bm = cv2.moments(bluemask, False)
        height, width, channels = img.shape
        try:
            gcx, gcy = gm['m10']/gm['m00'], gm['m01']/gm['m00']
        except ZeroDivisionError:
            gcx, gcy = height/2, width/2
        try:
            rcx, rcy = rm['m10']/rm['m00'], rm['m01']/rm['m00']
        except ZeroDivisionError:
            rcx, rcy = height/2, width/2
        try:
            bcx, bcy = bm['m10']/bm['m00'], bm['m01']/bm['m00']
        except ZeroDivisionError:
            bcx, bcy = height/2, width/2
        
        cv2.circle(greenres, (int(gcx), int(gcy)), 10,(0,0,255),-1)
        cv2.circle(redres, (int(rcx), int(rcy)), 10,(0,0,255),-1)
        cv2.circle(blueres, (int(bcx), int(bcy)), 10,(0,0,255),-1)
        self.greenerrorx = gcx - width/2
        self.greenerrory = gcy - height/2
        self.rederrorx = rcx - width/2
        self.rederrory = rcy - height/2
        self.blueerrorx = bcx - width/2
        self.blueerrory = bcy - height/2
        #cv2.imshow("original", greenres)
        #cv2.imshow("blue", blueres) #try to comment out when done
        #cv2.imshow("green", greenres)
        #cv2.imshow("red", redres)
        #cv2.waitKey(3)    

    #Move Towards the RED Object of Interest
    def gotored(self, redobjno, laser, action, mover, occ, odom):
        #occ.occupy()
        checkobst = laser.obstacle(mover)
        if checkobst == True:
            laser.avoidobstacle()
            return False
        #If False - Cancel Movebase
        if laser.front > 0.7 or laser.front == 0:
            mover.linear.x = 0.2
            mover.angular.z = -self.rederrorx / 10
            self.moverPub.publish(mover)
        elif laser.front < 0.7 and laser.front > 0.5:
            mover.linear.x = 0.1
            mover.angular.z = -self.rederrorx / 10
            self.moverPub.publish(mover)
        elif laser.front < 0.5 and laser.front != 0:
            mover.linear.x = 0
            mover.angular.z = 0
            self.moverPub.publish(mover)
            occ.logredobject(redobjno, 4, laser, action, odom)
            return True #True = Complete
        return False # False = Ongoing
    
    #Move Towards the Green Object of Interest
    def gotogreen(self, greenobjno, laser, mover, occ):
        #occ.occupy()
        checkobst = laser.obstacle(mover)
        if checkobst == True:
            laser.avoidobstacle()
            return False
        #If False - Cancel Movebase
        if laser.front > 0.7 or laser.front == 0:
            mover.linear.x = 0.2
            mover.angular.z = -self.greenerrorx / 10
            self.moverPub.publish(mover)
        elif laser.front < 0.7 and laser.front > 0.5:
            mover.linear.x = 0.1
            mover.angular.z = -self.greenerrorx / 10
            self.moverPub.publish(mover)
        elif laser.front < 0.5 and laser.front != 0:
            mover.linear.x = 0
            mover.angular.z = 0
            self.moverPub.publish(mover)
            occ.loggreenobject(greenobjno, 3) # 3 = Green
            return True #True = Complete
        return False # False = Ongoing

    #Turn Away From BLUE Objects Too Close to the Robot
    def avoidblue(self, mover):
        #occ.occupy()
        if self.blueerrory > 30 and self.blueerrorx > -55 and self.blueerrorx < 55: 
            if self.blueerrory < 75: #Currently Testing
                mover.linear.x = 0.1 #TRY MAINTAINING SPEED
            else:
                mover.linear.x = 0
            mover.angular.z = self.blueerrorx / 100
            if self.blueerrorx / 100 < 0.05 and self.blueerrorx / 100 > -0.05: 
                while self.blueerrorx != -20 and self.blueerrory != 20: 
                    mover.linear.x = 0 
                    mover.angular.z = 0.2 #If Centroid is central, give manual direction for tiebreake
                    self.moverPub.publish(mover)
            self.moverPub.publish(mover)
            return False # False = Ongoing
        #else catch
        mover.linear.x = 0
        mover.angular.z = 0
        self.moverPub.publish(mover)
        return True # True = Complete

    #Check IF Action is Required for BLUE Objects
    def IsBlueDetected(self):
        return(self.blueerrorx != -20 and self.blueerrory != 20 and self.blueerrory > 30 and self.blueerrorx > -55 and self.blueerrorx < 55)
    
    #Check IF Action is Required for GREEN Objects
    def IsGreenDetected(self, laser, occ, odom):
        if self.greenerrorx != -20 and self.greenerrory != 20:
            ignoredirection = self.ignore(3, occ, odom)
            if ignoredirection == True:
                rospy.loginfo('GREEN IN LINE - IGNORE')
                return False
            if str(laser.front) == 'inf' or laser.front == 0:
                return True
            elif occ.checkobjectlogged(3, odom, laser) == False:
                return True
            elif occ.checkobjectlogged(3, odom, laser) == True:
                return False
        
        return False
        
    #Check IF Action is Required for RED Objects    
    def IsRedDetected(self, laser, occ, odom):
        if self.rederrorx != -20 and self.rederrory != 20:
            ignoredirection = self.ignore(4, occ, odom)
            if ignoredirection == True:
                rospy.loginfo('RED IN LINE - IGNORE')
                return False
            if str(laser.front) == 'inf' or laser.front == 0:
                return True
            elif occ.checkobjectlogged(4, odom, laser) == False:
                return True
            elif occ.checkobjectlogged(4, odom, laser) == True:
                return False
        else:
            return False
    
    #Check if an Object of Interest has Been Found in That Direction & Can be Ignored
    def ignore(self, itemno, occ, odom):
        botpos = occ.to_grid(odom.prevx, odom.prevy)
        objectcounter = 0
        if occ.checkobjectsurrounding(botpos[0], botpos[1], itemno) > 0:
            return True
        if odom.cardinaldirection() == 1:
            for x in range(botpos[0], 61):
                for y in range(botpos[1]-4, botpos[1]+4):
                    if occ.grid[x][y] == itemno:
                        objectcounter += 1
        if odom.cardinaldirection() == 2:
            for y in range(0, botpos[1]):
                for x in range(botpos[0]-4, botpos[0]+4):
                    if occ.grid[x][y] == itemno:
                        objectcounter += 1
        if odom.cardinaldirection() == 3:
            for x in range(0, botpos[0]):
                for y in range(botpos[1]-4, botpos[1]+4):
                    if occ.grid[x][y] == itemno:
                        objectcounter += 1
        if odom.cardinaldirection() == 4:
            for y in range(botpos[1], 61):
                for x in range(botpos[0]-4, botpos[0]+4):
                    if occ.grid[x][y] == itemno:
                        objectcounter += 1
        #if in line has object ignore
        if objectcounter == 0:
            return False
        else:
            return True
