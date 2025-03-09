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

        #Obstacle Avoidance Lasers
        self.laserfrleft = 0
        self.laserfrright = 0
        self.laserbkleft = 0
        self.laserbkright = 0
        self.full = 0

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




class speed:
    def __init__(self):
        self.sub = rospy.Subscriber('/cmd_vel', Twist, self.speed_callback)
        self.pub = rospy.Publisher('cmd_vel2', Twist, queue_size=10)
        self.speed = 0
        self.turn = 0

    def speed_callback(self, msg):
        self.speed = msg.linear.x
        self.turn = msg.angular.z

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

    def camera_callback(self, msg): 
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        (h, w) = img.shape[:2]
        img_resized = cv2.resize(img, (int(w/4), int(h/4)))
        img = img_resized
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lowergreen = numpy.array([60,250, 30], dtype = "uint8") #Tested - No Problems
        uppergreen = numpy.array([62, 255, 100], dtype = "uint8") 
        lowerred = numpy.array([0, 247, 36], dtype = "uint8") # Very Feint - Due to red brick wall creating pixels which can throw off the bot
        upperred = numpy.array([1, 255, 40], dtype = "uint8")
        lowerblue = numpy.array([120, 250, 250], dtype = "uint8") #needs Testing and tuning
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
        #cv2.imshow("original", greenres)
        cv2.imshow("blue", blueres) #try to comment out when done
        cv2.imshow("green", greenres)
        cv2.imshow("red", redres)
        cv2.waitKey(3)

def to_grid(px, py): #Check Changes Work
    middlex = 20
    middley = 20
    posx = px #- odom.startingx #Off set by starting Odom X
    posy = py #- odom.startingy #Off set by starting Odom Y
    xgrid = int(middlex + (posx / occ.resolution)) # * or /
    ygrid = int(middley + (posy / occ.resolution)) #use minus to correct grid - DELETE IF BAD
    return [xgrid, ygrid]

def to_world(px, py): #Check Changes Work
    middlex = 20
    middley = 20
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
            if occ.grid[coord[0]][coord[1]] == -1 or occ.grid[end[0]][end[1]] == 2:
                occ.grid[coord[0]][coord[1]] = 0
            #rospy.loginfo('Updated Obstacle')
            #rospy.loginfo('Obstacle = '+str(coord))
        if scandata != 3 and occ.grid[end[0]][end[1]] == -1 or occ.grid[end[0]][end[1]] == 0 or occ.grid[end[0]][end[1]] == 2:
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
    shortestdist = 1000
    currentpos = to_grid(odom.prevx, odom.prevy)
    #rospy.loginfo('Frontier List = '+str(frontierlist()))
    for i in frontierlist():
        xdif = abs(currentpos[0]) - abs(i[0])
        ydif = abs(currentpos[1]) - abs(i[1])
        dist = abs(xdif) + abs(ydif) #Added ABS to prevent negative distances
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

def movebase_client(x, y):
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
    #goal.target_pose.pose.orientation.z = -0.5
    client.send_goal(goal)
    #rospy.loginfo('Goal Sent')

    #wait = client.wait_for_result()
    #if not wait:
    #    rospy.loginfo('Server Not Available')
    #    rospy.signal_shutdown('Server not Available')
    #else:
        #return client.get_result()       

def tolerance(goalx, goaly):
    if odom.prevx < goalx+0.5 and odom.prevx > goalx-0.5 and odom.prevy < goaly+0.5 and odom.prevy > goaly-0.5:
        return True
    else:
        return False

def distancetogoal(x, y):
    dist = abs(odom.prevx - x) + abs(odom.prevy - y)
    return dist

def spinaround(currentangle): # REQUIRES TESTING
    currentangle -= 0.16
    if laser.front > 0.2 and min(laser.full) > 0.2: #Too Close to wall to safely spin
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

def halfturn(currentangle):
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

def currentdirection(previousx, previousy):
    thetaangle = angle(odom.prevtheta)
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
        if odom.prevx+0.1 < previousx:
            rospy.loginfo('MOVING IN REVERSE - LOOKING NORTH')
            rospy.loginfo('Prev X = '+str(previousx)+', Now  = '+str(odom.prevx))
            return True
    if east == True:
        if odom.prevy-0.1 > previousy:
            rospy.loginfo('MOVING IN REVERSE - LOOKING EAST')
            rospy.loginfo('Prev Y = '+str(previousy)+', Now  = '+str(odom.prevx))
            return True
    if south == True:
        if odom.prevx-0.1 > previousx: #TEST INVERTING NORTH AND SOUTH
            rospy.loginfo('MOVING IN REVERSE - LOOKING SOUTH')
            rospy.loginfo('Prev X = '+str(previousx)+', Now  = '+str(odom.prevx))
            return True
    if west == True:
        if odom.prevy+0.1 < previousy:
            rospy.loginfo('MOVING IN REVERSE - LOOKING WEST')
            rospy.loginfo('Prev Y = '+str(previousy)+', Now  = '+str(odom.prevx))
            return True
    return False

def obstacle():
    #In range of obstacles turn left (for right hand following)
    if laser.front < 0.3 or min(laser.laserfrright) < 0.3 or min(laser.laserfrleft) < 0.3:
        mover.linear.x = 0
        mover.angular.z = 0
        pub.publish(mover)
        rospy.loginfo('Obstacle')
        return True
    return False

def backobstacle():
    if laser.back < 0.2 or min(laser.laserbkright) < 0.2 or min(laser.laserbkleft) < 0.2:
        rospy.loginfo('Back Obstacle')
        return True
    return False

def avoidobstacle(): 
        while laser.front < 0.5 or min(laser.laserfrright) < 0.5 or min(laser.laserfrleft) < 0.5: #Needs to remain WHILE - IF Breaks obstacle avoidance
            mover.linear.x = 0
            mover.angular.z = 0.4
            pub.publish(mover)
            rospy.loginfo('Avoiding Obstacle')
        mover.angular.z = 0
        pub.publish(mover)
        rospy.loginfo('Obstacle Avoided')

def mapcleanse():
    for i in range(len(occ.grid)):
        for j in range(len(occ.grid[i])):
            if occ.grid[i][j] == 2: 
                occ.grid[i][j] = 0

def checkobjectsurrounding(x, y, itemno):
    counter = 0
    if occ.grid[x][y] == itemno:
        counter += 1
    if occ.grid[x+1][y-1] == itemno:
        counter +=1
    if occ.grid[x+1][y] == itemno:
        counter +=1
    if occ.grid[x+1][y+1] == itemno:
        counter +=1
    if occ.grid[x][y-1] == itemno:
        counter +=1
    if occ.grid[x][y+1] == itemno:
        counter +=1
    if occ.grid[x-1][y-1] == itemno:
        counter +=1
    if occ.grid[x-1][y] == itemno:
        counter +=1
    if occ.grid[x-1][y+1] == itemno:
        counter +=1   
    return counter 

def checkobjectlogged(itemno):
    ang = angle(0)
    endodom = endpos(ang, laser.front)
    objgrid = to_grid(endodom[0], endodom[1])
    if checkobjectsurrounding(objgrid[0], objgrid[1], itemno) == 0:
        return False
    else:
        return True

def loggreenobject(objno, itemno): #Odometry Position and number of object
    frontangle = angle(0)
    objdist = laser.front
    obj = endpos(frontangle, objdist)
    loc = to_grid(obj[0], obj[1])
    occ.grid[loc[0]][loc[1]] = itemno
    worldloc = to_world(loc[0], loc[1])
    greenobjects.append(worldloc)
    rospy.loginfo("Green Obj No. "+str(objno)+" found at pos. "+str(obj))
    rospy.loginfo("Green Obj No. "+str(objno)+" stored in grid at pos. "+str(obj))

def logredobject(objno, itemno): #Odometry Position and number of object
    frontangle = angle(0)
    objdist = laser.front
    obj = endpos(frontangle, objdist)
    loc = to_grid(obj[0], obj[1])
    occ.grid[loc[0]][loc[1]] = itemno
    worldloc = to_world(loc[0], loc[1])
    redobjects.append(worldloc)
    rospy.loginfo("Red Obj No. "+str(objno)+" found at pos. "+str(obj))
    rospy.loginfo("Red Obj No. "+str(objno)+" stored in grid at pos. "+str(obj))

#Go To Red
def gotored(redobjno):
    checkobst = obstacle()
    if checkobst == True:
        avoidobstacle()
        return False
    #If False - Cancel Movebase
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
        logredobject(redobjno, 4) # 4 = red
        return True #True = Complete
    return False # False = Ongoing

#Go To Green
def gotogreen(greenobjno):
    checkobst = obstacle()
    if checkobst == True:
        avoidobstacle()
        return False
    #If False - Cancel Movebase
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
        loggreenobject(greenobjno, 3) # 3 = Green
        return True #True = Complete
    return False # False = Ongoing

#Avoid Blue
def avoidblue():
    if cam.blueerrory > 50 and cam.blueerrorx > -180 and cam.blueerrorx < 180: #was 35 but reacted too soon
        rospy.loginfo('Try and turn away')
        mover.linear.x = 0
        mover.angular.z = cam.blueerrorx / 100
        rospy.loginfo('TURN VALUE = '+str(cam.blueerrorx / 100))
        if cam.blueerrorx / 100 < 0.01 and cam.blueerrorx / 100 >-0.01:
            while cam.blueerrorx != -105 and cam.blueerrory != 105: #Testing While Loops
                mover.angular.z = 0.2 #If Centroid is central, give manual direction for tiebreaker
                rospy.loginfo('Override Blue Avoidance')
                pub.publish(mover)
        pub.publish(mover)
        return False # False = Ongoing
    else:
        mover.angular.z = 0
        pub.publish(mover)
        return True # True = Complete

def bluecheck():
    if cam.blueerrorx != -105 and cam.blueerrory != 105 and cam.blueerrory > 50 and cam.blueerrorx > -180 and cam.blueerrorx < 180: # errory was 35 but reacted too soon
        return True
    else:
        return False
    
def greencheck():
    if cam.greenerrorx != -105 and cam.greenerrory != 105:
        ignoredirection = ignore(3)
        if ignoredirection == True:
            rospy.loginfo('GREEN IN LINE - IGNORE')
            return False
        if str(laser.front) == 'inf' or laser.front == 0:
            return True
        elif checkobjectlogged(3) == False:
            return True
        elif checkobjectlogged(3) == True:
            return False
    else:
        return False
    
def redcheck():
    if cam.rederrorx != -105 and cam.rederrory != 105:
        ignoredirection = ignore(4)
        if ignoredirection == True:
            rospy.loginfo('RED IN LINE - IGNORE')
            return False
        if str(laser.front) == 'inf' or laser.front == 0:
            return True
        elif checkobjectlogged(4) == False:
            return True
        elif checkobjectlogged(4) == True:
            return False
    else:
        return False

def ignore(itemno):
    botpos = to_grid(odom.prevx, odom.prevy)
    objectcounter = 0
    if checkobjectsurrounding(botpos[0], botpos[1], itemno) > 0:
        return True
    if cardinaldirection() == 1:
        for x in range(0, botpos[0]):
            for y in range(botpos[1]-3, botpos[1]+3):
                if occ.grid[x][y] == itemno:
                    objectcounter += 1
    if cardinaldirection() == 2:
        for y in range(0, botpos[1]):
            for x in range(botpos[0]-3, botpos[0]+3):
                if occ.grid[x][y] == itemno:
                    objectcounter += 1
    if cardinaldirection() == 3:
        for x in range(botpos[0], 40):
            for y in range(botpos[1]-3, botpos[1]+3):
                if occ.grid[x][y] == itemno:
                    objectcounter += 1
    if cardinaldirection() == 4:
        for y in range(botpos[1], 40):
            for x in range(botpos[0]-3, botpos[0]+3):
                if occ.grid[x][y] == itemno:
                    objectcounter += 1
    #if in line has object ignore
    if objectcounter == 0:
        return False
    else:
        return True

#def ignorealttest():
    #Current Pos
    # Get angle & Distance to end of occ grid
    # run through alternative to_line

def cardinaldirection(): #1 = N, 2 = E, 3 = S, 4 = W
    robotdir = angle(odom.prevtheta)
    if robotdir <= 45 and robotdir > 315:
        return 1
    if robotdir > 45 and robotdir <= 135:
        return 4
    if robotdir > 135 and robotdir <= 225:
        return 3
    if robotdir > 225 and robotdir <= 315:
        return 2

if __name__ == '__main__':
    try:
        rospy.init_node('waypointnav', anonymous=False)
        client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        client.__init__
        rospy.loginfo('Connecting to Server')
        client.wait_for_server()
        rospy.loginfo('Connect to Server')
        odom = odomdata()
        scan = LaserScan()
        laser = lasers()
        occ = occdata()
        cam = camdata()
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        mover = Twist()


        mappub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
        i = 0
        iteration = 0
        freelist = []
        obslist = []
        frontiertime = 0
        movexstart = odom.prevx
        moveystart = odom.prevy
        x = 1000
        y = 1000
        robotangle = 1
        dircounter = 0

        greenobjects = []
        redobjects = []
        blueencountered = 0
        while not rospy.is_shutdown():

            #Check that Systems are Functional
            while checklaserinit() == False:
                rospy.loginfo('Not Initialised Yet')
                rospy.sleep(1)
            
            #Log Starting Position
            if iteration == 0:
                odom.startingx = odom.prevx
                odom.startingy = odom.prevy
                robotangle = odom.prevtheta
                odom.totaldistance = 5
                occupy()
                iteration+=1
            
            #costgrid = numpy.reshape(numpy.array(occ.grid), (-1, 41))
            #costgrid = costgrid.flatten().tolist()
            #occ.mymap.data = costgrid
            #mappub.publish(occ.mymap)
            #rospy.loginfo('ATTEMPTINT TO PUBLISH COSTGRID')

            #If Moved a far distance, spin to check for desired objects - NEEDS CHECK FOR DESIRED OBJECT AND CANCEL IF FINDS ONE
            #SPIN AGAIN AFTER FINDING ONE
            while odom.totaldistance >= 5:
                rospy.loginfo('TURN AROUND')
                robotangle = odom.prevtheta
                keepturning = True
                while keepturning == True:
                    rospy.loginfo('KEEP TURNING')
                    turn = spinaround(robotangle)
                    #occupy()
                    if turn == True:
                        odom.totaldistance = 0
                        keepturning = False
                    
            #Check for new frontier
            goalpos = closestfrontier()
            #If Nowhere to Go, Search has Ended
            if len(goalpos) == 0: 
                rospy.loginfo('No Frontier Exists')
                if len(redobjects) == 0 and len(greenobjects) == 0:
                    rospy.loginfo('TASK FAILED - NO OBJECTS OF INTEREST FOUND IN SEARCH')
                else:
                    rospy.loginfo('')
                    number = 1
                    rospy.loginfo('Number of Green Objects Found = '+str(len(greenobjects)))
                    for i in greenobjects:
                        rospy.loginfo('Green object no. '+str(number)+' found at'+str(i))
                        number += 1
                    number = 1
                    rospy.loginfo('')
                    rospy.loginfo('Number of Red Objects Found = '+str(len(redobjects)))
                    for i in redobjects:
                        rospy.loginfo('Red object no. '+str(number)+' found at'+str(i))
                        number += 1
                rospy.signal_shutdown('No Available Frontiers Left to Explore')
                # PRINT ALL FOUND OBJECT LOCATIONS
                # CLOSE NODE
                continue

            x = goalpos[0]
            y = goalpos[1] 
            dircounter = 0
            rospy.loginfo('NEW FRONTIER = '+str(x)+', '+str(y))
            while tolerance(x,y) == False:
                #rospy.loginfo('LOOPING')
                #rospy.loginfo('Robot angle = '+str(odom.prevtheta)+', Angle = '+str(angle(odom.prevtheta)))
                prevx = odom.prevx
                prevy = odom.prevy
                odom.distance = 0
                movebase_client(x, y)
                client.wait_for_result(timeout= rospy.Duration(1)) #Try Different Wait Values
                if odom.distance < 0.01: #Previously tested with 0.01
                    rospy.loginfo('Break Order')
                    break
                else:
                    mapcleanse() # Not yet Tested
                occupy()

                #Check for Obstacles in front
                obst = obstacle()
                if obst == True:
                    rospy.loginfo('OBSTACLE DETECTED')
                    client.cancel_all_goals()
                    rospy.sleep(1)
                    avoidobstacle()
                    break
                
                #Check for Obstacles behind
                obstbk = backobstacle()
                if obstbk == True:
                    client.cancel_all_goals()
                    rospy.sleep(1)
                
                #Add Objects into centralised list for output
                #Add Occ Grid updates when wayfinding to object
                objectsearch = True
                redobjno = len(redobjects)+1
                greenobjno = len(greenobjects)+1
                breaker = False
                if bluecheck() == True or redcheck() == True or greencheck() == True:
                    objectsearch == True
                    client.cancel_all_goals()
                    #rospy.loginfo('Unknown Object of Interest Found')
                    rospy.sleep(1)
                    while objectsearch == True:
                        #rospy.loginfo('OBJECTSEARCH = TRUE')
                        if bluecheck() == True and (laser.front < 0.2 or min(laser.full) < 0.2):
                            #client.cancel_all_goals()
                            rospy.loginfo('Blue Object Avoidance With Wall')
                            robotangle = odom.prevtheta
                            keepturning = True
                            while keepturning == True:
                                turn = halfturn(robotangle)
                            #occupy()
                                if turn == True:
                                    keepturning = False
                            breaker = True
                            blueencountered += 1
                        while bluecheck() == True and (laser.front > 0.2 or min(laser.full) > 0.2):
                            #client.cancel_all_goals()
                            rospy.loginfo('Blue Object Avoidance')
                            avoidblue()
                            if mover.angular.z != 0:
                                blueencountered = 5
                            breaker = True
                            blueencountered += 1
                        if greencheck() == True:
                            #client.cancel_all_goals()
                            rospy.loginfo('Green Object Investigation')
                            gotogreen(greenobjno)
                            breaker = True
                            blueencountered = 0
                        elif redcheck() == True:
                            #client.cancel_all_goals()
                            rospy.loginfo('Red Object Investigation')
                            gotored(redobjno)
                            breaker = True
                            blueencountered = 0
                        else:
                            mover.linear.x = 0
                            mover.angular.z = 0
                            pub.publish(mover)
                            rospy.loginfo('Objects of Interest Not Found')
                            #objectsearch = False 
                            break
                if breaker == True and blueencountered >= 2: #TESTING
                    break
                #Check if moving in reverse
                direction = currentdirection(prevx, prevy)
                if direction == True:
                    dircounter += 1
                    client.cancel_all_goals()
                    rospy.sleep(1)
                    #above creates too much stuttering but does fix problem
                    #Removing stuttering creates turnaround loop
                else: 
                    dircounter = 0
                if dircounter >= 3: # 3 or 5 - 5 is inconsistent at catching but rarely wrong
                    rospy.loginfo('Correcting Reverse')
                    client.cancel_all_goals()
                    rospy.sleep(1)
                    robotangle = odom.prevtheta
                    keepturning = True
                    while keepturning == True:
                        turn = halfturn(robotangle)
                        #occupy()
                        if turn == True:
                            rospy.loginfo('Finished Correcting Reverse')
                            keepturning = False
                            dircounter = 0
                    #Spin 180
            client.cancel_all_goals()
            rospy.loginfo('GOAL REACHED')
            updategridpos = to_grid(x, y)
            occ.grid[updategridpos[0]][updategridpos[1]] = 2

            
            rospy.sleep(1) # Try Sleep as problems when running for a long time
            
    except rospy.ROSInterruptException:
        rospy.loginfo('Navigation Test Finished')