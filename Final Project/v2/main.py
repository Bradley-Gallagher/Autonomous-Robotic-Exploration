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
from CamDataClass import CamData
from GlobalMapClass import GlobalMap

import tf.transformations
import cv2, cv_bridge
import argparse
import actionlib
import actionlib_tutorials.msg
import actionlib_msgs
import numpy
import math
import time

#Clear Imports if not needed

#Set up Actions Required for MoveBase OR the Search Algorithm
class MoveBaseActionList:
    def __init__(self):
        self.i = 0
        self.iteration = 0
        self.freelist = []
        self.obslist = []
        self.frontiertime = 0
        self.movexstart = -1
        self.moveystart = -1
        self.x = 1000
        self.y = 1000
        self.robotangle = 1
        self.dircounter = 0
        self.greenobjects = []
        self.redobjects = []
        self.blueencountered = 0
        self.breaker = False 
        self.goalpos = []

        self.map = GlobalMap()
        self.occ = OccData()
        self.odom = OdomData(self.occ)
        self.laser = Lasers()
        self.cam = CamData()
        self.mover = Twist()
        self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.client.__init__

    #Uses MoveBase to Move to a Specified Location
    def movebase_client(self, x, y):
        quaternions = list()
            
        # First define the corner orientations as Euler angles
        euler_angles = (pi/2, pi, 3*pi/2, 0)
            
        # Then convert the angles to quaternions
        for angle in euler_angles:
            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
            q = Quaternion(*q_angle)
            quaternions.append(q)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1
        self.client.send_goal(goal)


    #Check IF in Range of the MoveBase Goal
    def tolerance(self, goalx, goaly):
        return (self.odom.prevx < goalx+0.3 and self.odom.prevx > goalx-0.3 and self.odom.prevy < goaly+0.3 and self.odom.prevy > goaly-0.3)


    #Check Distance From Robot to MoveBase Goal
    def distancetogoal(self, x, y):
        dist = abs(self.odom.prevx - x) + abs(self.odom.prevy - y)
        return dist
    
    #Complete a FULL-TURN to Check for Surrounding Objects of Interest
    def spinaround(self, currentangle): 
        currentangle -= 0.16
        if self.laser.front > 0.2 and min(self.laser.full) > 0.2: #If NOT Too Close to wall to safely spin
            if currentangle < 0:
                if self.odom.prevtheta < currentangle - 0.15 or self.odom.prevtheta > currentangle + 0.15:
                    self.mover.linear.x = 0
                    self.mover.angular.z = 0.4
                    pub.publish(self.mover)
                    return False
                else:
                    self.mover.linear.x = 0
                    self.mover.angular.z = 0
                    pub.publish(self.mover)
                    return True
            else:
                if self.odom.prevtheta < currentangle - 0.15 and self.odom.prevtheta > currentangle + 0.15:
                    self.mover.linear.x = 0
                    self.mover.angular.z = 0.4
                    pub.publish(self.mover)
                    return False
                else:
                    self.mover.linear.x = 0
                    self.mover.angular.z = 0
                    pub.publish(self.mover)
                    return True
        else:
            return True

    #Complete a HALF-TURN
    def halfturn(self, currentangle):
        desiredangle = 0
        if currentangle >= 0:
            desiredangle = currentangle - 3.1
        else:
            desiredangle = currentangle + 3.1

        if self.odom.prevtheta < desiredangle - 0.15 or self.odom.prevtheta > desiredangle + 0.15:
            self.mover.linear.x = 0
            self.mover.angular.z = 0.4
            pub.publish(self.mover)
            return False
        else:
            self.mover.linear.x = 0
            self.mover.angular.z = 0
            pub.publish(self.mover)
            return True

    #Determine the Robots Current Direction to Check if it is Moving in Reverse
    def currentdirection(self, previousx, previousy):
        thetaangle = self.occ.angle(self.odom.prevtheta, self.odom.prevtheta)
        north = False
        if thetaangle < 90 or thetaangle > 270: #was 45, 315
            north = True
        east = False
        if thetaangle < 360 and thetaangle > 180:
            east = True
        south = False
        if thetaangle > 90 and thetaangle < 270: #was 135, 225
            south = True
        west = False
        if thetaangle > 0 and thetaangle < 180:
            west = True
        if north == True:
            if self.odom.prevx+0.1 < previousx:
                return True
        if east == True:
            if self.odom.prevy-0.1 > previousy:
                return True
        if south == True:
            if self.odom.prevx-0.1 > previousx:
                return True
        if west == True:
            if self.odom.prevy+0.1 < previousy:
                return True
        return False
    
    def movebasePublishing(self):
        #BLOCK 1 - MOVEBASE PUBLISHING
        self.odom.distance = 0
        self.movebase_client(self.x, self.y)
        self.client.wait_for_result(timeout = rospy.Duration(1))
        if self.odom.distance < 0.01:
            return False #Discontinue Current Frontier
        else:
            self.occ.mapcleanse() #If Frontier Found - Remove Occupancy Grid Placeholders
        self.occ.occupy(self.laser, self.odom)

    def obstacleDetection(self):
        obst = self.laser.obstacle(self.mover)
        if obst == True:
            self.client.cancel_all_goals()
            rospy.sleep(1)
            self.laser.avoidobstacle(self.mover)
            return False 
        
        obstbk = self.laser.backobstacle()
        if obstbk == True:
            self.client.cancel_all_goals()
            rospy.sleep(1)

    def block3(self):
        if self.objectcheck():
            self.client.cancel_all_goals()
            rospy.sleep(1)
        while self.objectcheck():
            self.objectactions()

    def isReversing(self):
        direction = self.currentdirection(self.odom.prevx, self.odom.prevy)
        if direction:
            self.dircounter += 1
            self.client.cancel_all_goals()
            rospy.sleep(1) #Pause to try and fix Reverse
        else:
            self.dircounter = 0

    def exploration(self):
        while self.tolerance(self.x, self.y) == False:
            self.movebasePublishing()
            self.obstacleDetection()
            self.block3()

            #If Stuck in Blue - Try moving elsewhere
            if self.breaker == True and self.blueencountered >= 10:
                return False
            
            self.isReversing()
            
            if self.dircounter >= 3:
                rospy.loginfo('Correcting Reverse')
                self.client.cancel_all_goals()
                rospy.sleep(1)
                self.robotangle = self.odom.prevtheta
                keepturning = True
                while keepturning:
                    turn = self.halfturn(self.robotangle)
                    if turn:
                        rospy.loginfo('Finished Correcting Reverse')
                        keepturning = False
                        self.dircounter = 0

        return True #Completed Movebase Exploration Goal

    def objectactions(self):
        redobjno = len(self.redobjects)+1 
        greenobjno = len(self.greenobjects)+1 
        objectsearch = True
        self.client.cancel_all_goals()
        rospy.sleep(1)
        while objectsearch == True:
            #Blue Check 1
            while self.cam.IsBlueDetected(): #and (laser.front > 0.2 or min(laser.full) > 0.2):
                self.mover.linear.x = 0
                self.map.logObject(self.odom.prevx, self.odom.prevy, self.odom.prevtheta, True)
                #wait for stop
                while(self.mover.linear.x == 0):
                    self.blueencountered += 1
                self.breaker = True
                    
            #Green Check 1
            if self.cam.IsGreenDetected(self.laser, self.occ, self.odom):
                self.cam.gotogreen(greenobjno, self.laser, self.mover, self.occ)
                self.blueencountered = 0
                self.breaker = False
                    
            #Red Check 1
            elif self.cam.IsRedDetected(self.laser, self.occ, self.odom):
                self.cam.gotored(redobjno, self.laser, self, self.mover, self.occ, self.odom)
                self.blueencountered = 0
                self.breaker = False
            else:
                self.mover.linear.x = 0
                self.mover.angular.z = 0
                pub.publish(self.mover)
                #rospy.loginfo('Objects of Interest Not Found')
                break #Break out of object search
    
    def objectcheck(self):
        if self.cam.IsBlueDetected() or self.cam.IsRedDetected(self.laser, self.occ, self.odom) or self.cam.IsGreenDetected(self.laser, self.occ, self.odom):
            return True
        else:
            return False
        
    def initialiseparameters(self):
        if self.iteration == 0:
            self.odom.startingx = self.odom.prevx
            self.odom.startingy = self.odom.prevy
            self.robotangle = self.odom.prevtheta
            self.odom.totaldistance = 5
            self.occ.occupy(self.laser, self.odom)
            self.iteration+=1
    
    def visualexploration(self):
        while self.odom.totaldistance >= 1:
            rospy.loginfo('TURN AROUND')
            self.robotangle = self.odom.prevtheta
            keepturning = True
            while keepturning == True:
                rospy.loginfo('KEEP TURNING')
                turn = self.spinaround(self.robotangle)
                if turn == True:
                    self.odom.totaldistance = 0
                    keepturning = False
                if self.objectcheck(): #Break when Object seen to Investigate Object
                    rospy.loginfo('Break Turn - Object Spotted')
                    keepturning = False

    def iterationcleanup(self):
        self.client.cancel_all_goals()
        rospy.sleep(1)
        rospy.loginfo('GOAL REACHED')
        updategridpos = self.occ.to_grid(self.x, self.y)
        self.occ.grid[updategridpos[0]][updategridpos[1]] = 2
        if self.blueencountered > 4:
            self.occ.mapcleanse()
        self.blueencountered = 0
        self.breaker = False

    def finaloutput(self):
        rospy.loginfo('No Frontier Exists')
        if len(self.redobjects) == 0 and len(self.greenobjects) == 0:
            rospy.loginfo('TASK FAILED - NO OBJECTS OF INTEREST FOUND IN SEARCH')
        else:
            rospy.loginfo('')
            number = 1
            rospy.loginfo('Number of Green Objects Found = '+str(len(self.greenobjects)))
            for i in self.greenobjects:
                rospy.loginfo('Green object no. '+str(number)+' found at'+str(i))
                number += 1
            number = 1
            rospy.loginfo('')
            rospy.loginfo('Number of Red Objects Found = '+str(len(self.redobjects)))
            for i in self.redobjects:
                rospy.loginfo('Red object no. '+str(number)+' found at'+str(i))
                number += 1
        rospy.signal_shutdown('No Available Frontiers Left to Explore')

#Needs to Be Converted Into New Functions & Sufficiently Commented & Reviewed
if __name__ == '__main__':
    try:
        rospy.init_node('waypointnav', anonymous=False)

        scan = LaserScan()
        container = MoveBaseActionList()

        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        iteration = 0

        while not rospy.is_shutdown():

            #Check that Systems are Functional Before Starting Operations
            while container.laser.checklaserinit() == False:
                rospy.loginfo('Not Initialised Yet')
                rospy.sleep(1)
            
            #Log Starting Position - Prepare for starting search
            container.initialiseparameters()
            
            if container.laser.obstacle(container.mover):
                container.laser.avoidobstacle(container.mover)

            #If Moved a far distance, spin to check for desired objects
            container.visualexploration()
            
            #If Object Spotted - Investigate
            while container.objectcheck():
                container.objectactions()

            #Check for new frontier
            goalpos = container.occ.closestfrontier(container.odom)

            #If No Frontier Exists, End Search and Print Object Data - TURN INTO FUNCTION
            if len(goalpos) == 0: 
                container.finaloutput()
            else:
                container.x = goalpos[0]
                container.y = goalpos[1] 
            rospy.loginfo('NEW FRONTIER = '+str(container.x)+', '+str(container.y))

            container.exploration()
            
            container.iterationcleanup()
            #rospy.sleep(1) # Try Sleep as problems when running for a long time
            
    except rospy.ROSInterruptException:
        rospy.loginfo('Navigation Test Finished')