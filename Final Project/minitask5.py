#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist, Pose2D, Pose, Point, Quaternion
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from math import sqrt, radians, pi
import tf.transformations
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
import cv2, cv_bridge
import argparse
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import actionlib_tutorials.msg
import actionlib_msgs
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler
import numpy
import math
from actionlib_msgs.msg import GoalID
import time

#Establish Odometry Related Data & Functions
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
        self.startingx = 0
        self.startingy = 0
        self.distance = 0
        self.totaldistance = 0
        self.increment = 0

    #Callback odom data from /odom and calculate new datapoints
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

    #Determine what the closest cardinal direction is to where the robot is facing
    def cardinaldirection(self): #1 = N, 2 = E, 3 = S, 4 = W
        robotdir = occ.angle(odom.prevtheta)
        if robotdir <= 45 and robotdir > 315:
            return 1
        if robotdir > 45 and robotdir <= 135:
            return 4
        if robotdir > 135 and robotdir <= 225:
            return 3
        if robotdir > 225 and robotdir <= 315:
            return 2

#Establish Laser Scan Related Data & Functions
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

        #Obstacle Avoidance Lasers
        self.laserfrleft = 0
        self.laserfrright = 0
        self.laserbkleft = 0
        self.laserbkright = 0
        self.full = 0

    #Callback laser data from /scan and select desired laser ranges
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

        #Obstacle Avoidance
        self.laserfrleft = msg.ranges[1:45]
        self.laserfrright = msg.ranges[315:359]
        self.laserbkleft = msg.ranges[135:179]
        self.laserbkright = msg.ranges[181:225]
        self.full = msg.ranges[1:359]
    
    #Checks if Lasers have Initialised (When starting, lasers will all return 0 - systems are not ready to begin yet)
    def checklaserinit(self):
        initialised = True
        if self.front == 0 and self.left == 0 and self.right == 0 and self.back == 0:
            if self.frontleft == 0 and self.frontright == 0 and self.backleft == 0 and self.backright == 0:
                if self.frontfrontleft == 0 and self.frontfrontright == 0 and self.leftfrontleft == 0 and self.rightfrontright == 0:
                    if self.backbackleft == 0 and self.backbackright ==0 and self.leftbackleft == 0 and self.rightbackright == 0:
                        initialised = False
        return initialised
    
    #Check if TOO Close to an Obstacle in Front (& STOP MOVING)
    def obstacle(self):
        if (self.front < 0.3 and self.front != 0) or (min(self.laserfrright) < 0.3 and min(self.laserfrright) != 0) or (min(self.laserfrleft) < 0.3 and min(self.laserfrleft) != 0 ):
            mover.linear.x = 0
            mover.angular.z = 0
            pub.publish(mover)
            rospy.loginfo('Obstacle Found')
            return True
        return False
    
    #Check if TOO Close to an Obstacle Behind
    def backobstacle(self):
        if (self.back < 0.2 and self.back != 0) or (min(self.laserbkright) < 0.2 and min(self.laserbkright) != 0) or (min(self.laserbkleft) < 0.2 and min(self.laserbkleft) != 0):
            rospy.loginfo('Obstacle Found Behind')
            return True
        return False
    
    #IF Too Close to an Obstacle in front, Turn Around Until in an Empty Space
    def avoidobstacle(self): 
        while ((self.front < 0.3 and self.front != 0) or (min(self.laserfrright) < 0.3 and min(self.laserfrright) != 0) or (min(self.laserfrleft) < 0.3 and min(self.laserfrleft) != 0)):
            mover.linear.x = 0
            mover.angular.z = 0.4
            pub.publish(mover)
            rospy.loginfo('Avoiding Obstacle')
        mover.angular.z = 0
        pub.publish(mover)
        rospy.loginfo('Obstacle Avoided')

#Establish Occupancy Grid Related Data & Functions
class occdata:
    def __init__(self):
        #self.sub = rospy.Subscriber('/map', OccupancyGrid, self.occ_callback)
        self.pub = rospy.Publisher('occupancy', Odometry, queue_size=10)
        self.mappub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
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
    
    #Converts Odometry Position into Occupancy Grid Position
    def to_grid(self, px, py): 
        middlex = 20
        middley = 20
        posx = px 
        posy = py 
        xgrid = int(middlex + (posx / self.resolution)) 
        ygrid = int(middley + (posy / self.resolution))
        return [xgrid, ygrid]
    
    #Converts Occupancy Grid Position into a Real World Odometry Position
    def to_world(self, px, py): 
        middlex = 20
        middley = 20
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
            #rospy.loginfo('grid = '+str(coord) + ' ' + str(grid[coord[0]][coord[1]]))
            if x in range(x1, x2):
                if self.grid[coord[0]][coord[1]] == -1 or self.grid[end[0]][end[1]] == 2:
                    self.grid[coord[0]][coord[1]] = 0
                #rospy.loginfo('Updated Obstacle')
                #rospy.loginfo('Obstacle = '+str(coord))
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
    
    #Converts the Position of the Robot into the Occupancy Grid
    def startpos(self):
        xpos = odom.prevx
        ypos = odom.prevy
        position = self.to_grid(xpos, ypos)
        return position
    
    #Converts the Robot Theta (front direction) from -Pi to Pi into an angle (0 to 360)
    def angle(self, theta):
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
    
    #Uses the Theta Angle of the Robot (with Trigenometry) to Determine the Laser End Position (X & Y Offset from current position)
    def endpos(self, angle, distance):
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
        #east
        elif east < north and east < south and east < west:
            y_dif = abs(distance * math.sin(east))
            x_dif = abs(distance * math.cos(east))
            x_dif = odom.prevx + x_dif
            if angle >= 270:
                y_dif = odom.prevy + y_dif
            else:
                y_dif = odom.prevy - y_dif
        #south
        elif south < north and south < east and south < west:
            y_dif = abs(distance * math.cos(south))
            x_dif = abs(distance * math.sin(south))
            y_dif = odom.prevy - y_dif
            if angle >=180:
                x_dif = odom.prevx + x_dif
            else:
                x_dif = odom.prevx - x_dif
        #west
        else:
            y_dif = abs(distance * math.sin(west))
            x_dif = abs(distance * math.cos(west))
            x_dif = odom.prevx - x_dif
            if angle >= 90:
                y_dif = odom.prevy - y_dif
            else:
                y_dif = odom.prevy + y_dif
        return [x_dif, y_dif]

    #Checks if Laser Data is Valid & Uses get_line to update the occupancy grid
    def iterateoccupy(self, las, lasangle):
        start = self.to_grid(odom.prevx, odom.prevy)
        if isinstance(las, float):
            laserangle = self.angle(lasangle)
            lasdist = las
            if str(las) == 'inf' or las == 0.0:
                lasdist = 3
            lasendpos = self.endpos(laserangle, lasdist)
            #rospy.loginfo('END POSITION = '+str(lasendpos))
            lasgridend = self.to_grid(lasendpos[0], lasendpos[1])
            #rospy.loginfo(lasgridend)
            self.get_line(start, lasgridend, lasdist)

    #Iterates Through ALL Selected Lasers to Update the Occupancy Grid
    def occupy(self):
        try:
            if laser.checklaserinit() == True:
                #Cardinal
                self.iterateoccupy(laser.front, 0)
                self.iterateoccupy(laser.right, 270)
                self.iterateoccupy(laser.back, 180)
                self.iterateoccupy(laser.left, 90)

                #Ordinal
                self.iterateoccupy(laser.frontleft, 45)
                self.iterateoccupy(laser.frontright, 315)
                self.iterateoccupy(laser.backleft, 135)
                self.iterateoccupy(laser.backright, 225)

                #2nd InterCardinal
                self.iterateoccupy(laser.frontfrontleft, 22)
                self.iterateoccupy(laser.frontfrontright, 337)
                self.iterateoccupy(laser.leftfrontleft, 67)
                self.iterateoccupy(laser.rightfrontright, 292)

                self.iterateoccupy(laser.backbackleft, 157)
                self.iterateoccupy(laser.backbackright, 202)
                self.iterateoccupy(laser.leftbackleft, 112)
                self.iterateoccupy(laser.rightbackright, 247)
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
                    if self.goaltooclose(i , j) == 0: 
                        frontierlist.append([i, j])
                        #rospy.loginfo('Frontier Grid Added: '+str(i)+', '+str(j))
        return frontierlist
    
    #Check if Position is TOO Close to nearby Object of Interest
    def goaltooclose(self, x, y):
        counter = 0
        for ix in range(0, 2):
            for iy in range(0, 2):
                if self.grid[x+ix][y+iy] == 3 or self.grid[x+ix][y+iy] == 4:
                    counter += 1
                if self.grid[x+ix][y-iy] == 3 or self.grid[x+ix][y-iy] == 4:
                    counter += 1
                if self.grid[x-ix][y+iy] == 3 or self.grid[x-ix][y+iy] == 4:
                    counter += 1
                if self.grid[x-ix][y-iy] == 3 or self.grid[x-ix][y-iy] == 4:
                    counter += 1
        return counter

    #Looks for the Closest Frontier Position to the Robot
    def closestfrontier(self):
        closest = []
        shortestdist = 1000
        currentpos = self.to_grid(odom.prevx, odom.prevy)
        #rospy.loginfo('Frontier List = '+str(frontierlist()))
        for i in self.frontierlist():
            xdif = abs(currentpos[0]) - abs(i[0])
            ydif = abs(currentpos[1]) - abs(i[1])
            dist = abs(xdif) + abs(ydif) 
            if abs(dist) < abs(shortestdist):
                closest = i
                shortestdist = dist
        #rospy.loginfo('Closest Frontier Grid = '+str(closest))
        if len(closest) > 0:
            closest = self.to_world(closest[0], closest[1])
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
        for ix in range(0, 3):
            for iy in range(0, 3):
                if self.grid[x+ix][y+iy] == itemno:
                    counter += 1
                if self.grid[x+ix][y-iy] == itemno:
                    counter += 1
                if self.grid[x-ix][y+iy] == itemno:
                    counter += 1
                if self.grid[x-ix][y-iy] == itemno:
                    counter += 1
        return counter 
    
    #Check if the Object in Front has Already Been Logged at THIS Position or a NEARBY Position
    def checkobjectlogged(self, itemno):
        ang = self.angle(0)
        endodom = self.endpos(ang, laser.front)
        objgrid = self.to_grid(endodom[0], endodom[1])
        if self.checkobjectsurrounding(objgrid[0], objgrid[1], itemno) == 0:
            return False
        else:
            return True
    
    #Log a Green Object into the Occupancy Grid
    def loggreenobject(self, objno, itemno):
        frontangle = self.angle(0)
        objdist = laser.front
        obj = self.endpos(frontangle, objdist)
        loc = self.to_grid(obj[0], obj[1])
        if self.checkobjectlogged(itemno) == False:
            self.grid[loc[0]][loc[1]] = itemno
            worldloc = self.to_world(loc[0], loc[1])
            action.greenobjects.append(worldloc)
            rospy.loginfo("Green Obj No. "+str(objno)+" found at pos. "+str(obj))
            rospy.loginfo("Green Obj No. "+str(objno)+" stored in grid at pos. "+str(loc))
        else:
            rospy.loginfo('ERROR: This Object Has Already Been Logged')
    
    #Log a Red Object into the Occupancy Grid
    def logredobject(self, objno, itemno):
        frontangle = self.angle(0)
        objdist = laser.front
        obj = self.endpos(frontangle, objdist)
        loc = self.to_grid(obj[0], obj[1])
        if self.checkobjectlogged(itemno) == False:
            self.grid[loc[0]][loc[1]] = itemno
            worldloc = self.to_world(loc[0], loc[1])
            action.redobjects.append(worldloc)
            rospy.loginfo("Red Obj No. "+str(objno)+" found at pos. "+str(obj))
            rospy.loginfo("Red Obj No. "+str(objno)+" stored in grid at pos. "+str(loc))
        else:
            rospy.loginfo('ERROR: This Object Has Already Been Logged')

#Establish Camera & Mask Related Data
class camdata:
    def __init__(self):
        self.imagesub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.camera_callback)
        self.pub = rospy.Publisher('camera2', Image, queue_size=10)
        self.bridge = cv_bridge.CvBridge()
        self.greenerrorx = 0
        self.greenerrory = 0
        self.rederrorx = 0
        self.rederrory = 0
        self.blueerrorx = 0
        self.blueerrory = 0
        #cv2.namedWindow("original",1)     

    #Callback camera data from /Camera and calculate the colour mask for the raw image
    def camera_callback(self, msg): 
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        (h, w) = img.shape[:2]
        img_resized = cv2.resize(img, (int(w/4), int(h/4)))
        img = img_resized
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lowergreen = numpy.array([60,250, 30], dtype = "uint8")
        uppergreen = numpy.array([62, 255, 100], dtype = "uint8") 
        lowerred = numpy.array([0, 247, 36], dtype = "uint8") 
        upperred = numpy.array([1, 255, 40], dtype = "uint8")
        lowerblue = numpy.array([120, 250, 250], dtype = "uint8") 
        upperblue = numpy.array([122, 255, 255], dtype = "uint8")
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
        cv2.imshow("blue", blueres) 
        cv2.imshow("green", greenres)
        cv2.imshow("red", redres)
        cv2.waitKey(3)    

    #Move Towards the RED Object of Interest
    def gotored(self, redobjno):
        checkobst = laser.obstacle()
        if checkobst == True:
            laser.avoidobstacle()
            return False
        if laser.front > 0.7 or laser.front == 0:
            mover.linear.x = 0.2
            mover.angular.z = -cam.rederrorx / 100
            pub.publish(mover)
        elif laser.front < 0.7 and laser.front > 0.5:
            mover.linear.x = 0.1
            mover.angular.z = -cam.rederrorx / 100
            pub.publish(mover)
        elif laser.front < 0.5 and laser.front != 0:
            mover.linear.x = 0
            mover.angular.z = 0
            pub.publish(mover)
            occ.logredobject(redobjno, 4)
            return True #True = Complete
        return False # False = Ongoing
    
    #Move Towards the Green Object of Interest
    def gotogreen(self, greenobjno):
        checkobst = laser.obstacle()
        if checkobst == True:
            laser.avoidobstacle()
            return False
        if laser.front > 0.7 or laser.front == 0:
            mover.linear.x = 0.2
            mover.angular.z = -cam.greenerrorx / 100
            pub.publish(mover)
        elif laser.front < 0.7 and laser.front > 0.5:
            mover.linear.x = 0.1
            mover.angular.z = -cam.greenerrorx / 100
            pub.publish(mover)
        elif laser.front < 0.5 and laser.front != 0:
            mover.linear.x = 0
            mover.angular.z = 0
            pub.publish(mover)
            occ.loggreenobject(greenobjno, 3) # 3 = Green
            return True #True = Complete
        return False # False = Ongoing

    #Turn Away From BLUE Objects IF Too Close to the Robot
    def avoidblue(self):
        if self.blueerrory > 40 and self.blueerrorx > -180 and self.blueerrorx < 180: 
            if self.blueerrory < 75: 
                mover.linear.x = 0.1 
            else:
                mover.linear.x = 0
            mover.angular.z = self.blueerrorx / 100
            #rospy.loginfo('TURN VALUE = '+str(self.blueerrorx / 100))
            if self.blueerrorx / 100 < 0.05 and self.blueerrorx / 100 >-0.05: 
                while self.blueerrorx != -105 and self.blueerrory != 105: 
                    mover.linear.x = 0 
                    mover.angular.z = 0.2 #If Centroid is central, give manual direction for tiebreaker
                    rospy.loginfo('Override Blue Avoidance')
                    pub.publish(mover)
            pub.publish(mover)
            return False # False = Ongoing
        else:
            mover.linear.x = 0
            mover.angular.z = 0
            pub.publish(mover)
            return True # True = Complete

    #Check IF Action is Required for BLUE Objects
    def bluecheck(self):
        if self.blueerrorx != -105 and self.blueerrory != 105 and self.blueerrory > 50 and self.blueerrorx > -180 and self.blueerrorx < 180:
            return True
        else:
            return False
    
    #Check IF Action is Required for GREEN Objects
    def greencheck(self):
        if self.greenerrorx != -105 and self.greenerrory != 105:
            ignoredirection = self.ignore(3)
            if ignoredirection == True:
                rospy.loginfo('GREEN IN LINE - IGNORE')
                return False
            if str(laser.front) == 'inf' or laser.front == 0:
                return True
            elif occ.checkobjectlogged(3) == False:
                return True
            elif occ.checkobjectlogged(3) == True:
                return False
        else:
            return False
        
    #Check IF Action is Required for RED Objects    
    def redcheck(self):
        if self.rederrorx != -105 and self.rederrory != 105:
            ignoredirection = self.ignore(4)
            if ignoredirection == True:
                rospy.loginfo('RED IN LINE - IGNORE')
                return False
            if str(laser.front) == 'inf' or laser.front == 0:
                return True
            elif occ.checkobjectlogged(4) == False:
                return True
            elif occ.checkobjectlogged(4) == True:
                return False
        else:
            return False
    
    #Check if an Object of Interest has Been Found in That Direction & Can be Ignored
    def ignore(self, itemno):
        botpos = occ.to_grid(odom.prevx, odom.prevy)
        objectcounter = 0
        if occ.checkobjectsurrounding(botpos[0], botpos[1], itemno) > 0:
            return True
        if odom.cardinaldirection() == 1:
            for x in range(botpos[0], 40):
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
            for y in range(botpos[1], 40):
                for x in range(botpos[0]-4, botpos[0]+4):
                    if occ.grid[x][y] == itemno:
                        objectcounter += 1
        #if in line has object ignore
        if objectcounter == 0:
            return False
        else:
            return True

#Set up Actions Required for MoveBase OR the Search Algorithm
class MoveBaseActionList:
    def __init__(self):
        self.i = 0
        self.iteration = 0
        self.freelist = []
        self.obslist = []
        self.frontiertime = 0
        self.movexstart = odom.prevx
        self.moveystart = odom.prevy
        self.x = 1000
        self.y = 1000
        self.robotangle = 1
        self.dircounter = 0
        self.greenobjects = []
        self.redobjects = []
        self.blueencountered = 0
        self.objectfound = False
        self.breaker = False 
        self.goalpos = []

    #Uses MoveBase to Move to a Specified Location
    def movebase_client(self, x, y):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1
        client.send_goal(goal)

    #Check IF in Range of the MoveBase Goal
    def tolerance(self, goalx, goaly):
        if odom.prevx < goalx+0.5 and odom.prevx > goalx-0.5 and odom.prevy < goaly+0.5 and odom.prevy > goaly-0.5:
            return True
        else:
            return False

    #Check Distance From Robot to MoveBase Goal
    def distancetogoal(x, y):
        dist = abs(odom.prevx - x) + abs(odom.prevy - y)
        return dist
    
    #Complete a FULL-TURN to Check for Surrounding Objects of Interest
    def spinaround(self, currentangle): 
        currentangle -= 0.16
        if laser.front > 0.2 and min(laser.full) > 0.2:
            if currentangle < 0:
                if odom.prevtheta < currentangle - 0.15 or odom.prevtheta > currentangle + 0.15:
                    mover.linear.x = 0
                    mover.angular.z = 0.4
                    pub.publish(mover)
                    return False
                else:
                    mover.linear.x = 0
                    mover.angular.z = 0
                    pub.publish(mover)
                    return True
            else:
                if odom.prevtheta < currentangle - 0.15 and odom.prevtheta > currentangle + 0.15:
                    mover.linear.x = 0
                    mover.angular.z = 0.4
                    pub.publish(mover)
                    return False
                else:
                    mover.linear.x = 0
                    mover.angular.z = 0
                    pub.publish(mover)
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

        if odom.prevtheta < desiredangle - 0.15 or odom.prevtheta > desiredangle + 0.15:
            mover.linear.x = 0
            mover.angular.z = 0.4
            pub.publish(mover)
            return False
        else:
            mover.linear.x = 0
            mover.angular.z = 0
            pub.publish(mover)
            return True

    #Determine the Robots Current Direction to Check if it is Moving in Reverse
    def currentdirection(self, previousx, previousy):
        thetaangle = occ.angle(odom.prevtheta)
        north = False
        if thetaangle < 90 or thetaangle > 270: 
            north = True
        east = False
        if thetaangle < 360 and thetaangle > 180:
            east = True
        south = False
        if thetaangle > 90 and thetaangle < 270: 
            south = True
        west = False
        if thetaangle > 0 and thetaangle < 180:
            west = True
        if north == True:
            if odom.prevx+0.1 < previousx:
                rospy.loginfo('MOVING IN REVERSE - LOOKING NORTH')
                return True
        if east == True:
            if odom.prevy-0.1 > previousy:
                rospy.loginfo('MOVING IN REVERSE - LOOKING EAST')
                return True
        if south == True:
            if odom.prevx-0.1 > previousx:
                rospy.loginfo('MOVING IN REVERSE - LOOKING SOUTH')
                return True
        if west == True:
            if odom.prevy+0.1 < previousy:
                rospy.loginfo('MOVING IN REVERSE - LOOKING WEST')
                return True
        return False
    
    #Use MoveBase to Move to a Frontier to explore the map (to look for Objects of Interest)
    def exploration(self):
        while action.tolerance(self.x, self.y) == False:
            #Establish Variables
            self.prevx = odom.prevx
            self.prevy = odom.prevy
            odom.distance = 0

            #Check if Frontier is too close to an Object of Interest - More Likely to get stuck around these
            goalgrid = occ.to_grid(self.x, self.y)
            if occ.goaltooclose(goalgrid[0], goalgrid[1]) != 0:
                return False

            #Check for Objects of Interest
            if self.objectcheck():
                client.cancel_all_goals()
                rospy.sleep(1)
            while self.objectcheck():
                self.objectactions()

            #Use MoveBase to Move to Frontier
            action.movebase_client(self.x, self.y)
            client.wait_for_result(timeout = rospy.Duration(1))
            if odom.distance < 0.01:
                rospy.loginfo('Break Exploration - PATH NOT FOUND')
                return False #Discontinue Current Frontier
            else:
                occ.mapcleanse() #If Frontier Found - Remove Occupancy Grid Placeholders
            occ.occupy()

            #OBSTACLE DETECTION
            obst = laser.obstacle()
            if obst == True:
                rospy.loginfo('OBSTACLE DETECTED')
                client.cancel_all_goals()
                rospy.sleep(1)
                laser.avoidobstacle()
                return False #Discontinue Current Frontier
            
            obstbk = laser.backobstacle()
            if obstbk == True:
                client.cancel_all_goals()
                rospy.sleep(1)
            
            #Re-Check for Objects of Interest After Movement
            if self.objectcheck():
                client.cancel_all_goals()
                rospy.sleep(1)
            while self.objectcheck():
                self.objectactions()
            
            #Set variables for Visual Exploration if Object Recently Found
            self.robotangle = odom.prevtheta
            if self.objectfound == True:
                odom.totaldistance = 6
                self.objectfound = False

            #If Stuck in Blue - Reset Try moving elsewhere
            if self.breaker == True and self.blueencountered >= 10:
                return False
            
            #Check if Robot is Moving in Reverse
            direction = self.currentdirection(self.prevx, self.prevy)
            if direction:
                self.dircounter += 1
                client.cancel_all_goals()
                rospy.sleep(1) #Pause to try and fix Reverse
            else:
                self.dircounter = 0
            
            #If Robot Continues to Move in Reverse - Manually Correct
            if self.dircounter >= 3:
                rospy.loginfo('Correcting Reverse')
                client.cancel_all_goals()
                rospy.sleep(1)
                self.robotangle = odom.prevtheta
                keepturning = True
                while keepturning:
                    turn = self.halfturn(self.robotangle)
                    if turn:
                        rospy.loginfo('Finished Correcting Reverse')
                        keepturning = False
                        self.dircounter = 0
        return True #Completed Movebase Exploration Goal

    #Act on the Object that has Been Spotted (Move to Red/Green OR Avoid Blue)
    def objectactions(self):
        redobjno = len(self.redobjects)+1 
        greenobjno = len(self.greenobjects)+1 
        objectsearch = True
        client.cancel_all_goals()
        rospy.sleep(1)
        while objectsearch == True:            
            #Blue Check
            while cam.bluecheck():
                rospy.loginfo('Blue Object Avoidance')
                cam.avoidblue()
                if mover.linear.x == 0:
                    self.blueencountered += 1
                self.breaker = True
                #self.blueencountered += 1
                    
            #Green Check
            if cam.greencheck():
                rospy.loginfo('Green Object Investigation')
                action.objectfound = cam.gotogreen(greenobjno)
                self.blueencountered = 0
                self.breaker = False
                    
            #Red Check
            elif cam.redcheck():
                rospy.loginfo('Red Object Investigation')
                action.objectfound = cam.gotored(redobjno)
                self.blueencountered = 0
                self.breaker = False
            else:
                mover.linear.x = 0
                mover.angular.z = 0
                pub.publish(mover)
                break #Break out of object search
    
    #Check if an Object of Interest has been Spotted (and is actionable)
    def objectcheck(self):
        if cam.bluecheck() or cam.redcheck() or cam.greencheck():
            return True
        else:
            return False
        
    #Initialise parameters to prepare for exploration
    def initialiseparameters(self):
        if self.iteration == 0:
            odom.startingx = odom.prevx
            odom.startingy = odom.prevy
            action.robotangle = odom.prevtheta
            odom.totaldistance = 5
            occ.occupy()
            self.iteration+=1
    
    #If Travelled a far distance, Turn Around to Look for Objects of Interest
    def visualexploration(self):
        while odom.totaldistance >= 5:
            rospy.loginfo('TURN AROUND')
            action.robotangle = odom.prevtheta
            keepturning = True
            while keepturning == True:
                #rospy.loginfo('KEEP TURNING')
                turn = action.spinaround(action.robotangle)
                if turn == True:
                    odom.totaldistance = 0
                    keepturning = False
                if action.objectcheck(): #Break when Object spotted to Investigate Object
                    rospy.loginfo('Break Turn - Object Spotted')
                    odom.totaldistance = 0
                    keepturning = False
        odom.totaldistance = 0

    #Prepare to search for the next frontier available
    def iterationcleanup(self):
        client.cancel_all_goals()
        rospy.sleep(1)
        rospy.loginfo('GOAL REACHED')
        updategridpos = occ.to_grid(action.x, action.y)
        occ.grid[updategridpos[0]][updategridpos[1]] = 2
        if action.blueencountered > 4:
            occ.mapcleanse()
        action.blueencountered = 0
        action.breaker = False

    #Print the Locations of Objects of Interest IF NO Frontiers exist
    def finaloutput(self):
        rospy.loginfo('No Frontier Exists')
        if len(action.redobjects) == 0 and len(action.greenobjects) == 0:
            rospy.loginfo('TASK FAILED - NO OBJECTS OF INTEREST FOUND IN SEARCH')
        else:
            rospy.loginfo('')
            number = 1
            rospy.loginfo('Number of Green Objects Found = '+str(len(action.greenobjects)))
            for i in action.greenobjects:
                rospy.loginfo('Green object no. '+str(number)+' found at'+str(i))
                number += 1
            number = 1
            rospy.loginfo('')
            rospy.loginfo('Number of Red Objects Found = '+str(len(action.redobjects)))
            for i in action.redobjects:
                rospy.loginfo('Red object no. '+str(number)+' found at'+str(i))
                number += 1
        rospy.signal_shutdown('No Available Frontiers Left to Explore')

# Main Executable
if __name__ == '__main__':
    try:
        rospy.init_node('explorer', anonymous=False)

        #Establish an Action Client for Movebase
        client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        client.__init__
        rospy.loginfo('Connecting to Server')
        client.wait_for_server()
        rospy.loginfo('Connect to Server')

        #Establish Parameters and Classes for robot interactions
        scan = LaserScan()
        mover = Twist()
        odom = odomdata()
        laser = lasers()
        occ = occdata()
        cam = camdata()
        action = MoveBaseActionList()
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        while not rospy.is_shutdown():

            #Check that Systems are Functional Before Starting Operations
            while laser.checklaserinit() == False:
                rospy.loginfo('Not Initialised Yet')
                rospy.sleep(1)
            
            #Log Starting Position - Prepare for starting search (FIRST LOOP ONLY)
            action.initialiseparameters()
            
            #IF Obstacle in Range, Avoid Obstacle
            if laser.obstacle():
                laser.avoidobstacle()

            #If Moved a far distance, spin to check for desired objects
            action.visualexploration()
            
            #If Object Spotted - Investigate
            while action.objectcheck():
                action.objectactions()

            #Repeat after Action Object to check new area for objects IF Applicable
            action.visualexploration()

            #Check for Next/New Frontier
            goalpos = occ.closestfrontier()

            #If No Frontier Exists, End Search and Print Object Data - TURN INTO FUNCTION
            if len(goalpos) == 0: 
                action.finaloutput()
            else:
                #Establish New Frontier
                action.x = goalpos[0]
                action.y = goalpos[1] 
                rospy.loginfo('NEW FRONTIER = '+str(action.x)+', '+str(action.y))

                #Attempt to Move to New Frontier
                action.exploration()

                #Prepare to Move the Next Frontier
                action.iterationcleanup()
                #rospy.sleep(1) # Creates better Object Interaction/Investigation - but slower exploration/frontier generation
            
    except rospy.ROSInterruptException:
        rospy.loginfo('Navigation Test Finished')
