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
        #front
        self.front = 0
        self.frontright = 0
        self.frontrightrange = 0
        self.frontleft = 0
        self.frontleftrange = 0
        #left
        self.left = 0
        self.leftfront = 0
        self.leftfrontrange = 0
        self.leftback = 0
        self.leftbackrange = 0
        #back
        self.back = 0
        self.backright = 0
        self.backrightrange = 0
        self.backleft = 0
        self.backleftrange = 0
        #right
        self.right = 0
        self.rightfront = 0
        self.rightfrontrange = 0
        self.rightback = 0
        self.rightbackrange = 0

        self.full = 0

    def laser_callback(self, msg): 
        
        self.pub.publish(scan)
        #change to broader ranges - e.g., 0-15
        self.front = msg.ranges[0]
        #left
        self.left = msg.ranges[90]

        #back
        self.back = msg.ranges[180]
        #right
        self.right = msg.ranges[270]

#Occupancy Grid
mymap = OccupancyGrid()
mymap.header.frame_id = 'map'
resolution = 1
width = 13
height = 16
rate = 5
xpose = 0
ypose = 0
botrange = 0
grid = numpy.ndarray((width, height), buffer=numpy.zeros((width, height), dtype=numpy.int), dtype= numpy.int)
grid.fill(int(-1))


def to_grid(px, py):
    middlex = 5
    middley = 11
    posx = px - -3 #Off set by starting Odom X
    posy = py - 1 #Off set by starting Odom Y
    xgrid = int(middlex + posx / resolution) #Convert to int for grid position
    ygrid = int(middley + posy / resolution)
    return numpy.array([xgrid, ygrid])

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
    for x in range(x1, x2): #x2+1 = Obstacle
        coord = (y,x) if is_steep else (x,y)
        #rospy.loginfo('grid = '+str(coord) + ' ' + str(grid[coord[0]][coord[1]]))
        if x not in range(x1, x2-1) and scandata != 3.0:
            if grid[coord[0]][coord[1]] != 1:
                grid[coord[0]][coord[1]] = 1
                #rospy.loginfo('Updated Obstacle')
                rospy.loginfo('Obstacle = '+str(coord))
        elif grid[coord[0]][coord[1]] != 0:
            grid[coord[0]][coord[1]] = 0
            #rospy.loginfo('Updated Free Space')
            rospy.loginfo('free = '+str(coord))

        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx
    if swapped:
        points.reverse()

    return points

def startpos():
    xpos = int(math.floor(odom.prevx))
    ypos = int(math.floor(odom.prevy))
    position = to_grid(xpos, ypos)
    return position

def angle(theta):
    #Turn Theta into Degrees for sum
    angle = -1.4
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
        y_dif = distance * math.cos(north)
        x_dif = distance * math.sin(north)
        y_dif = odom.prevy + y_dif
        if angle > 90:
            x_dif = odom.prevx + x_dif
        else:
            x_dif = odom.prevx - x_dif
    #east
    elif east < north and east < south and east < west:
        y_dif = distance * math.sin(south)
        x_dif = distance * math.cos(south)
        x_dif = odom.prevx + x_dif
        if angle >= 270:
            y_dif = odom.prevy + y_dif
        else:
            y_dif = odom.prevy - y_dif
    #south
    elif south < north and south < east and south < west:
        y_dif = distance * math.cos(south)
        x_dif = distance * math.sin(south)
        y_dif = odom.prevy - y_dif
        if angle >=180:
            x_dif = odom.prevx + x_dif
        else:
            x_dif = odom.prevx - x_dif
    #west
    else:
        y_dif = distance * math.sin(south)
        x_dif = distance * math.cos(south)
        x_dif = odom.prevy - x_dif
        if angle >= 90:
            y_dif = odom.prevy - y_dif
        else:
            y_dif = odom.prevy + y_dif
    
    return [int(x_dif), int(y_dif)]

def occupy():
    try:
        #rospy.loginfo('Start X: '+str(odom.prevx)+', Start Y: '+str(odom.prevy))
        start = to_grid(odom.prevx, odom.prevy)
        #rospy.loginfo('Start Pos = '+str(start))
        
        #Front Laser
        if isinstance(laser.front, float) and laser.front != 0:
            frontangle = angle(0)
            frontdistance = laser.front
            if str(laser.front) == 'inf':
                frontdistance = 3.0
            frontend = endpos(frontangle, frontdistance)
            fend = to_grid(frontend[0], frontend[1])
            get_line(start, fend, frontdistance)
            rospy.loginfo('Front laser updates: '+str(get_line(start, fend, frontdistance)))
        
        #Right Laser
        if isinstance(laser.right, float) and laser.right != 0:
            rightangle = angle(270)
            rightdistance = laser.right
            if str(laser.right) == 'inf':
                rightdistance = 3.0
            rightend = endpos(rightangle, rightdistance)
            rend = to_grid(rightend[0], rightend[1])
            get_line(start, rend, rightdistance)
            rospy.loginfo('Right laser updates: '+str(get_line(start, rend, rightdistance)))
        
        #Back Laser
        if isinstance(laser.back, float) and laser.back != 0:
            backangle = angle(180)
            backdistance = laser.back
            if str(laser.back) == 'inf':
                backdistance = 3.0
            backend = endpos(backangle, backdistance)
            bend = to_grid(backend[0], backend[1])
            get_line(start, bend, backdistance)
            rospy.loginfo('Back laser updates: '+str(get_line(start, bend, backdistance)))
        
        #Left Laser
        if isinstance(laser.left, float) and laser.left != 0:
            leftangle = angle(90)
            leftdistance = laser.left
            if str(laser.left) == 'inf':
                leftdistance = 3.0
            leftend = endpos(leftangle, leftdistance)
            lend = to_grid(leftend[0], leftend[1])
            get_line(start, lend, leftdistance)
            rospy.loginfo('Left laser updates: '+str(get_line(start, lend, leftdistance)))
    except Exception as e:
        rospy.loginfo(f"WARNING: {e} skipping...")


if __name__ == '__main__':
    try:
        # init

        rospy.init_node('occupation', anonymous=False)
        
        #gridpub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
        odom = odomdata()
        scan = LaserScan()
        laser = lasers()
        r = rospy.Rate(10)
        
        mymap.info.resolution = resolution
        mymap.info.width = width
        mymap.info.height = height
        mymap.data = range(width*height)
        mymap.info.origin.position.x = -1
        mymap.info.origin.position.y = -3

        while not rospy.is_shutdown():
            occupy()
            rospy.loginfo(grid)
            rospy.sleep(2)

    except rospy.ROSInterruptException:
        pass