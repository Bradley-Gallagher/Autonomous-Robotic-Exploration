#!/usr/bin/python3
import rospy
import tf.transformations
import cv2, cv_bridge
import argparse
import actionlib
import actionlib_tutorials.msg
import actionlib_msgs
import numpy
import math
import os
import numpy as np

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, LaserScan
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist, Pose2D, Pose, Point, Quaternion
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from math import sqrt, radians, pi, sin, cos

from OccDataClass import OccData

from OdomDataClass import OdomData

class GlobalMap(OccupancyGrid):
    def __init__(self):
        super().__init__()
        self.hasInit = False

        self.sub = rospy.Subscriber('/map', OccupancyGrid, self.callback)
        self.pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)

        self.occ_callback = OccupancyGrid()
        self.occ_publisher = OccupancyGrid()

        self.occObj = OccData()
        self.odom = OdomData(self.occObj)

        self.resolution = -1
        self.width = -1
        self.height = -1
        self.xpose = 0
        self.ypose = 0
        self.lethal = int(100)
        self.grid = []
        self.grid2D = []

        self.logSpace = 3
        self.logDistance = 15

    def initSelf(self):
        self.hasInit = True
        self.resolution = self.occ_callback.info.resolution
        self.width = self.occ_callback.info.width
        self.height = self.occ_callback.info.height
        self.grid = list(self.occ_callback.data)
        self.reshapeGrid()
        self.publish()
        self.reshapeGrid()

    def callback(self, msg):
        self.occ_callback = msg
        if(not self.hasInit):
           self.initSelf()

    def returnCentre(self):
        return self.indexToOcc(self.width // 2, self.height // 2)

    def indexToOcc(self, x, y):
        return self.height * y + x

    def publish(self):
        self.occ_publisher.data = list(np.reshape(self.grid2D, len(self.grid)))
        self.occ_publisher.info.resolution = self.occ_callback.info.resolution
        self.occ_publisher.info.width = self.occ_callback.info.width
        self.occ_publisher.info.height = self.occ_callback.info.width
        self.occ_publisher.info.origin = self.occ_callback.info.origin
        self.pub.publish(self.occ_publisher)

    def fill(self):
        self.grid = np.full(len(self.grid), 100, dtype=int).tolist()

    def clearGrid(self):
        self.grid = np.zeros(len(self.grid), dtype=int).tolist()

    def worldToGrid(self, x, y): 
        return int(x * self.height +  y)

    def reshapeGrid(self):
        self.grid2D = np.reshape(self.grid, (self.width, self.height))

    def worldToGrid2D(self, realX, realY):
        xgrid = int((self.width / 2) + (realX / self.resolution)) # * or /
        ygrid = int((self.height / 2) + (realY / self.resolution))

        xOffset = 12
        yOffset = 2

        xOut = int(xgrid + xOffset)
        yOut = int(ygrid + yOffset)

        return yOut, xOut

    def logObject(self, botX, botY, rotr, parsed = False):

        #error catch
        while(not self.hasInit):
            rospy.sleep(1)

        if(not parsed):
            roll, pitch, yaw = tf.transformations.euler_from_quaternion([rotr.x, rotr.y, rotr.z, rotr.w])
        else:
            yaw = rotr
        
        self.odom.prevx = botX
        self.odom.prevY = botY

        temp = self.occObj.endpos(self.occObj.angle(0, -yaw), self.logDistance * self.resolution, self.odom)
        newX = temp[0]
        newY = temp[1]

        try:        
            #already logged 
            if(self.grid2D[int(newX)][int(newY)] == self.lethal):
                return
            self.createSquare(newX , newY)
            self.publish()
        except Exception as e:
            print(f"e")
            print("SKIPPING..")    
    #end
    def createSquare(self,x,y):
        x,y = self.worldToGrid2D(x,y)
        for i in range(-self.logSpace,self.logSpace):
            for j in range(-self.logSpace,self.logSpace):
                self.grid2D[x + i][y + j] = self.lethal
