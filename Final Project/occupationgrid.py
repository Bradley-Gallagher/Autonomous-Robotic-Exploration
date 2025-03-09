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
import sys
from actionlib_msgs.msg import GoalID
numpy.set_printoptions(threshold = sys.maxsize)


from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist, Pose2D, Pose, Point, Quaternion
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from math import sqrt, radians, pi

class odomdata:
    def __init__(self):
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.pub = rospy.Publisher('turtlebotdriving', Odometry, queue_size=10)
        self.prevtheta = 0
        self.prevx = 0
        self.prevy = 0
        self.prevw = 0
        self.quaternion = 0
        self.firstmove = True

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
        self.pub.publish(msg)
        self.prevx = msg.pose.pose.position.x
        self.prevy = msg.pose.pose.position.y
        self.prevw = msg.pose.pose.orientation.w
        self.prevtheta = yaw
        self.firstmove = False

class lasers:
    def __init__(self):
        self.sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.pub = rospy.Publisher('scan2', LaserScan, queue_size=10)
        
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


    def laser_callback(self, msg): 
        
        self.pub.publish(scan)
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

class occdata:
    def __init__(self):
        self.sub = rospy.Subscriber('/map', OccupancyGrid, self.occ_callback)
        self.pub = rospy.Publisher('occupancy', Odometry, queue_size=10)
        self.mappub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
        self.resolution = 0.05
        self.width = 401
        self.height = 401
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
        self.mymap.data = self.grid
        self.mappub.publish(self.mymap)
    
    def occ_callback(self, msg): 
        self.mymap.data = msg.data
        self.mappub.publish(self.mymap)
        

def to_grid(px, py): #Check Changes Work
    middlex = 200
    middley = 200
    posx = px #- odom.startingx #Off set by starting Odom X
    posy = py #- odom.startingy #Off set by starting Odom Y
    xgrid = int(middlex + (posx / occ.resolution)) # * or /
    ygrid = int(middley + (posy / occ.resolution)) #use minus to correct grid - DELETE IF BAD
    return [xgrid, ygrid]

def to_world(px, py): #Check Changes Work
    middlex = 200
    middley = 200
    posx = px #+ odom.startingx #Off set by starting Odom X
    posy = py #+ odom.startingy #Off set by starting Odom Y
    xgrid = (posx - middlex) * occ.resolution # * or /
    ygrid = (posy - middley) * occ.resolution
    return [xgrid, ygrid]

def get_line(start, end, scandata):
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
    for x in range(x1, x2+1): #x2+1 = Obstacle
        coord = (y,x) if is_steep else (x,y) # MAY BE CAUSING ISSUES - CHANGE
        #rospy.loginfo('grid = '+str(coord) + ' ' + str(grid[coord[0]][coord[1]]))
        #if x not in range(x1, x2):
        #    if occ.grid[coord[0]][coord[1]] != 1 and scandata != 3.0:
        #        occ.grid[coord[0]][coord[1]] = 1
                #rospy.loginfo('Updated Free Space')
                #rospy.loginfo('Free = '+str(coord))
        if x in range(x1, x2):
            if occ.grid[coord[0]][coord[1]] == -1: #and scandata != 3.0:
                occ.grid[coord[0]][coord[1]] = 0
            #rospy.loginfo('Updated Obstacle')
            #rospy.loginfo('Obstacle = '+str(coord))
        if scandata != 3 and occ.grid[end[0]][end[1]] == -1 or occ.grid[end[0]][end[1]] == 0:
            occ.grid[end[0]][end[1]] = 1

        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx
    if swapped:
        points.reverse()

    return points

def startpos():
    xpos = odom.prevx
    ypos = odom.prevy
    position = to_grid(xpos, ypos)
    return position

def angle(theta):
    #Turn Theta into Degrees for sum
    angle = odom.prevtheta
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

def endpos(angle, distance):
    north = angle
    if abs(angle-360) < north:
        north = abs(angle-360)
    east = abs(270-angle)
    south = abs(180-angle)
    west = abs(90-angle)
    mini = 0
    
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
    
    return [x_dif, y_dif] #Was INT

def iterateoccupy(las, lasangle):
    start = to_grid(odom.prevx, odom.prevy)
    if isinstance(las, float):
        laserangle = angle(lasangle)
        #rospy.loginfo('Angle = '+str(laserangle))
        #rospy.loginfo('Distance = '+str(las))
        lasdist = las
        if str(las) == 'inf' or las == 0.0:
            lasdist = 3
        lasendpos = endpos(laserangle, lasdist)

        #rospy.loginfo('END POSITION = '+str(lasendpos))
        lasgridend = to_grid(lasendpos[0], lasendpos[1])
        #rospy.loginfo(lasgridend)
        get_line(start, lasgridend, lasdist)

def checklaserinit():
    initialised = True
    if laser.front == 0 and laser.left == 0 and laser.right == 0 and laser.back == 0:
        if laser.frontleft == 0 and laser.frontright == 0 and laser.backleft == 0 and laser.backright == 0:
            if laser.frontfrontleft == 0 and laser.frontfrontright == 0 and laser.leftfrontleft == 0 and laser.rightfrontright == 0:
                if laser.backbackleft == 0 and laser.backbackright ==0 and laser.leftbackleft == 0 and laser.rightbackright == 0:
                    initialised = False
    return initialised

def occupy():
    try:
        if checklaserinit() == True:
            #Cardinal
            iterateoccupy(laser.front, 0)
            iterateoccupy(laser.right, 270)
            iterateoccupy(laser.back, 180)
            iterateoccupy(laser.left, 90)

            #Ordinal
            iterateoccupy(laser.frontleft, 45)
            iterateoccupy(laser.frontright, 315)
            iterateoccupy(laser.backleft, 135)
            iterateoccupy(laser.backright, 225)

            #2nd InterCardinal
            iterateoccupy(laser.frontfrontleft, 22)
            iterateoccupy(laser.frontfrontright, 337)
            iterateoccupy(laser.leftfrontleft, 67)
            iterateoccupy(laser.rightfrontright, 292)

            iterateoccupy(laser.backbackleft, 157)
            iterateoccupy(laser.backbackright, 202)
            iterateoccupy(laser.leftbackleft, 112)
            iterateoccupy(laser.rightbackright, 247)
        #else:
            #rospy.loginfo('Lasers Initialising - Not Ready Yet')

    except Exception as e:
        rospy.loginfo(f"WARNING: {e} skipping...")

def greengrid(posx, posy, i): #Odometry Position and number of object
    #Green Obj Position
    frontangle = angle(0)
    greenobjdist = laser.front
    greenobj = endpos(frontangle, greenobjdist)
    greenloc = to_grid(greenobj[0], greenobj[1])
    occ.grid[greenloc[0]][greenloc[1]] = 2
    rospy.loginfo("Green Obj No. "+str(i)+" found at pos. "+str(greenobj))
    rospy.loginfo("Green Obj No. "+str(i)+" stored in grid at pos. "+str(greenloc))

def checksurrounding(x, y):
    counter = 0
    if occ.grid[x+1][y-1] == -1:
        counter +=1
    if occ.grid[x+1][y] == -1:
        counter +=1
    if occ.grid[x+1][y+1] == -1:
        counter +=1
    if occ.grid[x][y-1] == -1:
        counter +=1
    if occ.grid[x][y+1] == -1:
        counter +=1
    if occ.grid[x-1][y-1] == -1:
        counter +=1
    if occ.grid[x-1][y] == -1:
        counter +=1
    if occ.grid[x-1][y+1] == -1:
        counter +=1   
    return counter 

def checkwalls(x, y):
    counter = 0
    if x == 0 or x == 40 or y == 0 or y ==40:
        return counter
    if occ.grid[x+1][y-1] == 1:
        counter +=1
    if occ.grid[x+1][y] == 1:
        counter +=1
    if occ.grid[x+1][y+1] == 1:
        counter +=1
    if occ.grid[x][y-1] == 1:
        counter +=1
    if occ.grid[x][y+1] == 1:
        counter +=1
    if occ.grid[x-1][y-1] == 1:
        counter +=1
    if occ.grid[x-1][y] == 1:
        counter +=1
    if occ.grid[x-1][y+1] == 1:
        counter +=1   
    return counter 

def frontierlist():
    frontierlist = []
    for i in range(len(occ.grid)):
        for j in range(len(occ.grid[i])):
            if occ.grid[i][j] == 0: #swap UNSURE IF CHANGE IS WORKING OR NOT
                if checksurrounding(i, j) >= 1:
                    frontierlist.append([i, j])
                    #rospy.loginfo('Frontier Grid Added: '+str(i)+', '+str(j))
    return frontierlist

def closestfrontier():
    closest = []
    shortestdist = 100
    currentpos = to_grid(odom.prevx, odom.prevy)
    #rospy.loginfo('Frontier List = '+str(frontierlist()))
    for i in frontierlist():
        xdif = abs(currentpos[0]) - abs(i[0])
        ydif = abs(currentpos[1]) - abs(i[1])
        dist = xdif + ydif
        if abs(dist) < abs(shortestdist):
            closest = i

            shortestdist = dist
            #rospy.loginfo('New Closest = '+str(closest))
    #rospy.loginfo('Closest Frontier Grid = '+str(closest))
    #rospy.loginfo('CLOSEST X = '+str(closest[0])+', Closest Y = '+str(closest[1]))
    if len(closest) > 0:
        closest = to_world(closest[0], closest[1])
    #rospy.loginfo('Closest Frontier World = '+str(closest))
    return closest


if __name__ == '__main__':
    try:
        # init
        
        rospy.init_node('occupation', anonymous=False)

        #Occupancy Grid
        map = OccupancyGrid()
        occ = occdata()
        map.header.frame_id = 'map'

        odom = odomdata()
        scan = LaserScan()
        laser = lasers()
        r = rospy.Rate(10)
        cancelpub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        cancelmsg = GoalID()

        while not rospy.is_shutdown():
            print(occ.mymap.data)
    except rospy.ROSInterruptException:
        pass