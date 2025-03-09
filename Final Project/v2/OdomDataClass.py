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

#Set up Odometry Related Data
class OdomData:
    def __init__(self, occ):
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.pub = rospy.Publisher('turtlebotdriving', Odometry, queue_size=10)
        self.prevtheta = 0
        self.prevx = 0
        self.prevy = 0
        self.prevw = 0
        self.quaternion = 0
        self.firstmove = True
        self.startingx = 0
        self.startingy = 0
        self.distance = 0
        self.totaldistance = 0
        self.increment = 0
        self.occObj = occ
        self.distanceincreased = False

    def odom_callback(self, msg): 
        self.quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(self.quaternion)
        if self.firstmove == True:
            self.prevx = msg.pose.pose.position.x
            self.prevy = msg.pose.pose.position.y
            self.prevtheta = yaw
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.increment = abs(x - self.prevx) + abs(y - self.prevy)
        if self.increment > 0.1:
            self.distanceincreased = True
        self.distance += self.increment 
        self.totaldistance += self.increment 
        self.prevx = msg.pose.pose.position.x
        self.prevy = msg.pose.pose.position.y
        self.prevw = msg.pose.pose.orientation.w
        self.prevtheta = yaw
        self.firstmove = False

    def cardinaldirection(self): #1 = N, 2 = E, 3 = S, 4 = W
        robotdir = self.occObj.angle(0,self.prevtheta)
        if robotdir <= 45 and robotdir > 315:
            return 1
        if robotdir > 45 and robotdir <= 135:
            return 4
        if robotdir > 135 and robotdir <= 225:
            return 3
        if robotdir > 225 and robotdir <= 315:
            return 2
