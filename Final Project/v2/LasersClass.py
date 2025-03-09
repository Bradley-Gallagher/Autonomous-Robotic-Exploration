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

import tf.transformations
import cv2, cv_bridge
import argparse
import actionlib
import actionlib_tutorials.msg
import actionlib_msgs
import numpy
import math
import time


#Set up Laser Scan Related Data
class Lasers:
    def __init__(self):
        self.sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.pub = rospy.Publisher('scan2', LaserScan, queue_size=10)
        self.moverPub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan = LaserScan()

        #Cardinal Directions
        self.front = 0
        self.left = 0
        self.back = 0
        self.right = 0

        #Ordinal Directions/Intercardinal Directions
        self.frontleft = 0
        self.frontright = 0
        self.backleft = 0
        self.backright = 0

        #2nd InterCardinal Directions
        self.frontfrontleft = 0
        self.leftfrontleft = 0
        self.frontfrontright = 0
        self.rightfrontright = 0

        self.backbackleft = 0
        self.leftbackleft = 0
        self.backbackright = 0
        self.rightbackright = 0

        #Obstacle Avoidance Lasers
        self.laserfrleft = 0
        self.laserfrright = 0
        self.laserbkleft = 0
        self.laserbkright = 0
        self.full = 0

    def laser_callback(self, msg): 
        try:
            self.pub.publish(self.scan)
        except rospy.exceptions.ROSException as e:
            if(e.find("publish()")):
                rospy.loginfo("Laser Scan turned off and didnt turn on > hoping and praying from here")
            else:
                rospy.loginfo(f"e")

        #Cardinal Directions
        self.front = msg.ranges[0]
        self.left = msg.ranges[90]
        self.back = msg.ranges[180]
        self.right = msg.ranges[270]

        #Ordinal Directions
        self.frontleft = msg.ranges[45]
        self.frontright = msg.ranges[315]
        self.backleft = msg.ranges[135]
        self.backright = msg.ranges[225]

        #2nd InterCardinal Directions
        self.frontfrontleft = msg.ranges[22]
        self.leftfrontleft = msg.ranges[67]
        self.frontfrontright = msg.ranges[337]
        self.rightfrontright = msg.ranges[292]

        self.backbackleft = msg.ranges[157]
        self.leftbackleft = msg.ranges[112]
        self.backbackright = msg.ranges[202]
        self.rightbackright = msg.ranges[247]

        #Obstacle Avoidance
        self.laserfrleft = msg.ranges[1:45]
        self.laserfrright = msg.ranges[315:359]
        self.laserbkleft = msg.ranges[135:179]
        self.laserbkright = msg.ranges[181:225]
        self.full = msg.ranges[1:359]
    
    #Checks if Lasers have Initialised (When starting, lasers will all return 0 when starting - this can cause errors)
    def checklaserinit(self):
        initialised = True
        if self.front == 0 and self.left == 0 and self.right == 0 and self.back == 0:
            if self.frontleft == 0 and self.frontright == 0 and self.backleft == 0 and self.backright == 0:
                if self.frontfrontleft == 0 and self.frontfrontright == 0 and self.leftfrontleft == 0 and self.rightfrontright == 0:
                    if self.backbackleft == 0 and self.backbackright ==0 and self.leftbackleft == 0 and self.rightbackright == 0:
                        initialised = False
        return initialised
    
    #Check if TOO Close to an Obstacle in Front (& STOP MOVING)
    def obstacle(self, mover):
        if self.front < 0.3 or min(self.laserfrright) < 0.3 or min(self.laserfrleft) < 0.3:
            mover.linear.x = 0
            mover.angular.z = 0
            self.moverPub.publish(mover)
            rospy.loginfo('Obstacle')
            return True
        return False
    
    #Check if TOO Close to an Obstacle Behind
    def backobstacle(self):
        if self.back < 0.2 or min(self.laserbkright) < 0.2 or min(self.laserbkleft) < 0.2:
            rospy.loginfo('Back Obstacle')
            return True
        return False
    
    #IF Too Close to an Obstacle, Turn Around Until in an Empty Space
    def avoidobstacle(self, mover): 
        while self.front < 0.5 or min(self.laserfrright) < 0.5 or min(self.laserfrleft) < 0.5: #Needs to remain WHILE - IF Breaks obstacle avoidance
            mover.linear.x = 0
            mover.angular.z = 0.4
            self.moverPub.publish(mover)
            rospy.loginfo('Avoiding Obstacle')
        mover.angular.z = 0
        self.moverPub.publish(mover)
        rospy.loginfo('Obstacle Avoided')
