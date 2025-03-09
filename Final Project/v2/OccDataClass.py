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

import tf.transformations
import cv2, cv_bridge
import argparse
import actionlib
import actionlib_tutorials.msg
import actionlib_msgs
import numpy
import math
import time

#Set Up Occupancy Grid Related Data
class OccData:
    def __init__(self):
        #self.sub = rospy.Subscriber('/map', OccupancyGrid, self.occ_callback)
        self.pub = rospy.Publisher('occupancy', Odometry, queue_size=10)
        self.resolution = 0.5
        self.width = 41
        self.height = 41
        self.mymap = OccupancyGrid(
            header = Header(seq = 0, stamp = rospy.Time.now(), frame_id='map'),
            info = (MapMetaData(width = self.width, height = self.height, resolution = self.resolution, map_load_time = rospy.Time.now()))
        )
        self.rate = 10
        self.xpose = 0
        self.ypose = 0
        self.botrange = 0
        self.grid = numpy.ndarray((self.width, self.height), buffer=numpy.zeros((self.width, self.height), dtype=numpy.int), dtype= numpy.int)
        self.initcount = 0
        self.mymap.header.frame_id = 'map'
        self.mymap.info.resolution = self.resolution
        self.mymap.info.width = self.width
        self.mymap.info.height = self.height
        if self.initcount == 0:
            self.grid.fill(int(-1))
            self.initcount += 1
    
    def occ_callback(self, msg): 
        self.mymap.data = msg.data
    
    #Converts Odometry Position into Occupancy Grid
    def to_grid(self, px, py): 
        middlex = 30
        middley = 30
        posx = px 
        posy = py 
        xgrid = int(middlex + (posx / self.resolution)) 
        ygrid = int(middley + (posy / self.resolution))
        return [xgrid, ygrid]
    
    #Converts Occupancy Grid Position into a Real World Position
    def to_world(self, px, py): 
        middlex = 30
        middley = 30
        posx = px 
        posy = py 
        xgrid = (posx - middlex) * self.resolution 
        ygrid = (posy - middley) * self.resolution
        return [xgrid, ygrid]

    #Find ALL Occupancy Grid Positions Between Robot Position and Laser End Position
    def get_line(self, start, end, scandata):
        x1, y1 = start
        x2, y2 = end
        dx = x2 - x1
        dy = y2 - y1
        is_steep = abs(dy) > abs(dx)
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2
        swapped = False
        if x1>x2:
            x1,x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True
        dx = x2 - x1
        dy = y2 - y1
        error = int(dx/2.0)
        ystep = 1 if y1 < y2 else -1
        y = y1
        points = []
        for x in range(x1, x2+1):
            coord = (y,x) if is_steep else (x,y)
            if x in range(x1, x2):
                if self.grid[coord[0]][coord[1]] == -1 or self.grid[end[0]][end[1]] == 2:
                    self.grid[coord[0]][coord[1]] = 0
            if scandata != 3 and self.grid[end[0]][end[1]] == -1 or self.grid[end[0]][end[1]] == 0 or self.grid[end[0]][end[1]] == 2:
                self.grid[end[0]][end[1]] = 1
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx
        if swapped:
            points.reverse()
        return points

    #Converts the Robot Theta (front direction) from -Pi to Pi into an angle (0 to 360)
    def angle(self, theta, angle):
    #angle = odom.prevtheta
        negative = False
        if angle < 0:
            negative = True
        convert = math.degrees(angle)
        if negative == True:
            angle = 360+convert
        else:
            angle = convert
        angle+=theta
        if angle >= 360:
            angle-=360
        return angle
    
    #Uses the Angle of the Robot with Trigenometry to Determine the Laser End Position
    def endpos(self, angle, distance, odom):
        north = angle
        if abs(angle-360) < north:
            north = abs(angle-360)
        east = abs(270-angle)
        south = abs(180-angle)
        west = abs(90-angle)
        
        #north
        if north < east and north < south and north < west:
            y_dif = abs(distance * math.cos(north))
            x_dif = abs(distance * math.sin(north))
            y_dif = odom.prevy + y_dif
            if angle < 90:
                x_dif = odom.prevx - x_dif
            else:
                x_dif = odom.prevx + x_dif
            #rospy.loginfo('North Triggered '+str(odom.prevtheta))
        #east
        elif east < north and east < south and east < west:
            y_dif = abs(distance * math.sin(east))
            x_dif = abs(distance * math.cos(east))
            x_dif = odom.prevx + x_dif
            if angle >= 270:
                y_dif = odom.prevy + y_dif
            else:
                y_dif = odom.prevy - y_dif
            #rospy.loginfo('East Triggered')
        #south
        elif south < north and south < east and south < west:
            y_dif = abs(distance * math.cos(south))
            x_dif = abs(distance * math.sin(south))
            y_dif = odom.prevy - y_dif
            #rospy.loginfo(str(odom.prevy)+' - '+str(distance * math.cos(south))+' = '+str(y_dif))
            if angle >=180:
                x_dif = odom.prevx + x_dif
            else:
                x_dif = odom.prevx - x_dif
            #rospy.loginfo('South Triggered')
        #west
        else:
            y_dif = abs(distance * math.sin(west))
            x_dif = abs(distance * math.cos(west))
            x_dif = odom.prevx - x_dif
            if angle >= 90:
                y_dif = odom.prevy - y_dif
            else:
                y_dif = odom.prevy + y_dif
            #rospy.loginfo('West Triggered')
        
        return [x_dif, y_dif]

    #Checks if Laser Data is Valid & Uses get_line to update the occupancy grid
    def iterateoccupy(self, las, lasangle, odom):
        start = self.to_grid(odom.prevx, odom.prevy)
        if isinstance(las, float):
            laserangle = self.angle(lasangle, odom.prevtheta)
            #rospy.loginfo('Angle = '+str(laserangle))
            #rospy.loginfo('Distance = '+str(las))
            lasdist = las
            if str(las) == 'inf' or las == 0.0:
                lasdist = 1
            lasendpos = self.endpos(laserangle, lasdist, odom)

            #rospy.loginfo('END POSITION = '+str(lasendpos))
            lasgridend = self.to_grid(lasendpos[0], lasendpos[1])
            #rospy.loginfo(lasgridend)
            self.get_line(start, lasgridend, lasdist)

    #Iterates Through ALL Lasers to Update the Occupancy Grid
    def occupy(self, laser, odom):
        try:
            if laser.checklaserinit() == True:
                #Cardinal
                self.iterateoccupy(laser.front, 0, odom)
                self.iterateoccupy(laser.right, 270, odom)
                self.iterateoccupy(laser.back, 180, odom)
                self.iterateoccupy(laser.left, 90, odom)

                #Ordinal
                self.iterateoccupy(laser.frontleft, 45, odom)
                self.iterateoccupy(laser.frontright, 315, odom)
                self.iterateoccupy(laser.backleft, 135, odom)
                self.iterateoccupy(laser.backright, 225, odom)

                #2nd InterCardinal
                self.iterateoccupy(laser.frontfrontleft, 22, odom)
                self.iterateoccupy(laser.frontfrontright, 337, odom)
                self.iterateoccupy(laser.leftfrontleft, 67, odom)
                self.iterateoccupy(laser.rightfrontright, 292, odom)

                self.iterateoccupy(laser.backbackleft, 157, odom)
                self.iterateoccupy(laser.backbackright, 202, odom)
                self.iterateoccupy(laser.leftbackleft, 112, odom)
                self.iterateoccupy(laser.rightbackright, 247, odom)
            #else:
                #rospy.loginfo('Lasers Initialising - Not Ready Yet')
        except Exception as e:
            rospy.loginfo(f"WARNING: {e} skipping...")
    
    #Checks the Surrounding Positions of a Specified Occupancy Grid Position for UNKNOWN Positions
    def checksurrounding(self, x, y):
        counter = 0
        if self.grid[x+1][y-1] == -1:
            counter +=1
        if self.grid[x+1][y] == -1:
            counter +=1
        if self.grid[x+1][y+1] == -1:
            counter +=1
        if self.grid[x][y-1] == -1:
            counter +=1
        if self.grid[x][y+1] == -1:
            counter +=1
        if self.grid[x-1][y-1] == -1:
            counter +=1
        if self.grid[x-1][y] == -1:
            counter +=1
        if self.grid[x-1][y+1] == -1:
            counter +=1   
        return counter 
    
    #Checks the Surrounding Positions of a Specified Occupancy Grid Position for OCCUPIED Positions
    def checkwalls(self, x, y):
        counter = 0
        if x == 0 or x == 40 or y == 0 or y ==40:
            return counter
        if self.grid[x+1][y-1] == 1:
            counter +=1
        if self.grid[x+1][y] == 1:
            counter +=1
        if self.grid[x+1][y+1] == 1:
            counter +=1
        if self.grid[x][y-1] == 1:
            counter +=1
        if self.grid[x][y+1] == 1:
            counter +=1
        if self.grid[x-1][y-1] == 1:
            counter +=1
        if self.grid[x-1][y] == 1:
            counter +=1
        if self.grid[x-1][y+1] == 1:
            counter +=1   
        return counter 
    
    #Looks for FREE Positions Neighouring UNKNOWN Positions (Frontier Positions)
    def frontierlist(self):
        frontierlist = []
        for i in range(len(self.grid)):
            for j in range(len(self.grid[i])):
                if self.grid[i][j] == 0:
                    if self.checksurrounding(i, j) >= 1:
                        frontierlist.append([i, j])
                        #rospy.loginfo('Frontier Grid Added: '+str(i)+', '+str(j))
        return frontierlist

    #Looks for the Closest Frontier Position to the Robot
    def closestfrontier(self, odom):
        closest = []
        shortestdist = 1000
        currentpos = self.to_grid(odom.prevx, odom.prevy)
        #rospy.loginfo('Frontier List = '+str(frontierlist()))
        for i in self.frontierlist():
            xdif = abs(currentpos[0]) - abs(i[0])
            ydif = abs(currentpos[1]) - abs(i[1])
            dist = abs(xdif) + abs(ydif) #Added ABS to prevent negative distances
            if abs(dist) < abs(shortestdist):
                closest = i
                shortestdist = dist
        if len(closest) > 0:
            closest = self.to_world(closest[0], closest[1])
        #rospy.loginfo('Closest Frontier World = '+str(closest))
        return closest
    
    #Remove ANY Placeholders from the Occupancy Grid
    def mapcleanse(self):
        for i in range(len(self.grid)):
            for j in range(len(self.grid[i])):
                if self.grid[i][j] == 2: 
                    self.grid[i][j] = 0

    #Check IF a Logged/Found Object of Interest is Nearby
    def checkobjectsurrounding(self, x, y, itemno):
        counter = 0
        if self.grid[x][y] == itemno:
            counter += 1
        if self.grid[x+1][y-1] == itemno:
            counter +=1
        if self.grid[x+1][y] == itemno:
            counter +=1
        if self.grid[x+1][y+1] == itemno:
            counter +=1
        if self.grid[x][y-1] == itemno:
            counter +=1
        if self.grid[x][y+1] == itemno:
            counter +=1
        if self.grid[x-1][y-1] == itemno:
            counter +=1
        if self.grid[x-1][y] == itemno:
            counter +=1
        if self.grid[x-1][y+1] == itemno:
            counter +=1   
        return counter 
    
    #Check if the Object in Front has Already Been Logged at THIS Position or a NEARBY Position
    def checkobjectlogged(self, itemno, odom, laser):
        ang = self.angle(0, odom.prevtheta)
        endodom = self.endpos(ang, laser.front, odom)
        objgrid = self.to_grid(endodom[0], endodom[1])
        if self.checkobjectsurrounding(objgrid[0], objgrid[1], itemno) == 0:
            return False
        
        return True
    
    #Log a Green Object into the Occupancy Grid
    def loggreenobject(self, objno, itemno, laser, action, odom):
        frontangle = self.angle(0, odom.prevtheta)
        objdist = laser.front
        obj = self.endpos(frontangle, objdist, odom)
        loc = self.to_grid(obj[0], obj[1])
        if self.checkobjectlogged(itemno, odom, laser) == False:
            self.grid[loc[0]][loc[1]] = itemno
            worldloc = self.to_world(loc[0], loc[1])
            action.greenobjects.append(worldloc)
            rospy.loginfo("Green Obj No. "+str(objno)+" found at pos. "+str(obj))
            rospy.loginfo("Green Obj No. "+str(objno)+" stored in grid at pos. "+str(loc))
        else:
            rospy.loginfo('ERROR: This Object Has Already Been Logged')
    
    def logredobject(self, objno, itemno, laser, action, odom): #Odometry Position and number of object
        frontangle = self.angle(0, odom.prevtheta)
        objdist = laser.front
        obj = self.endpos(frontangle, objdist, odom)
        loc = self.to_grid(obj[0], obj[1])
        if self.checkobjectlogged(itemno, odom, laser) == False:
            self.grid[loc[0]][loc[1]] = itemno
            worldloc = self.to_world(loc[0], loc[1])
            action.redobjects.append(worldloc)
            rospy.loginfo("Red Obj No. "+str(objno)+" found at pos. "+str(obj))
            rospy.loginfo("Red Obj No. "+str(objno)+" stored in grid at pos. "+str(loc))
        else:
            rospy.loginfo('ERROR: This Object Has Already Been Logged')
