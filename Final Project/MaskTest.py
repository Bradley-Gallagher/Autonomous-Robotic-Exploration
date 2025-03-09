#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist, Pose2D, Quaternion
from sensor_msgs.msg import LaserScan
import math
from math import sqrt, radians, pi
import tf.transformations
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
import cv2, cv_bridge
import numpy
import argparse
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
import sys
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
numpy.set_printoptions(threshold = sys.maxsize)

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

class camdata:
    def __init__(self):
        self.imagesub = rospy.Subscriber('/camera/color/image_raw', Image, self.camera_callback)
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
        #lowergreen = numpy.array([60,250, 30], dtype = "uint8") 
        #uppergreen = numpy.array([62, 255, 100], dtype = "uint8") 
        lowergreen = numpy.array([42, 100, 130], dtype = "uint8") # - NEEDS FINE TUNING - NOT ALWAYS DETECING - Continue Testing
        uppergreen = numpy.array([48, 255, 255], dtype = "uint8") 
        #lowerred = numpy.array([0, 247, 36], dtype = "uint8") 
        #upperred = numpy.array([1, 255, 40], dtype = "uint8")
        lowerred = numpy.array([0, 150, 150], dtype = "uint8") 
        upperred = numpy.array([5, 255, 200], dtype = "uint8")
        #lowerblue = numpy.array([120, 250, 250], dtype = "uint8") 
        #upperblue = numpy.array([122, 255, 255], dtype = "uint8")
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
        cv2.imshow("blue", blueres) 
        cv2.imshow("green", greenres)
        cv2.imshow("red", redres)
        cv2.waitKey(3)

class avoidobstacle:
    def __init__(self):
        self.sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.pub = rospy.Publisher('scan2', LaserScan, queue_size=10)
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
        #change to broader ranges - e.g., 0-15
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
    
    return [x_dif, y_dif]

class occdata:
    def __init__(self):
        self.sub = rospy.Subscriber('/map', OccupancyGrid, self.occ_callback)
        self.pub = rospy.Publisher('occupancy', Odometry, queue_size=10)
        self.mappub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
        self.resolution = 0.5
        self.width = 41
        self.height = 41
        self.mymap = OccupancyGrid(
            header = Header(seq = 0, stamp = rospy.Time.now(), frame_id='map'),
            info = (MapMetaData(width = self.width, height = self.height, resolution = self.resolution, map_load_time = rospy.Time.now()))
        )
        self.rate = 5
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

def to_grid(px, py):
    middlex = 20
    middley = 20
    posx = px #Off set by starting Odom X
    posy = py #Off set by starting Odom Y
    xgrid = int(middlex + (posx / 0.5)) # * or /
    ygrid = int(middley + (posy / 0.5))
    return numpy.array([xgrid, ygrid])

def checksurrounding(x, y, itemno):
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

def logobject(itemno):
    ang = angle(0)
    endodom = endpos(ang, laser.front)
    objgrid = to_grid(endodom[0], endodom[1])
    if occ.grid[objgrid[0]][objgrid[1]] != itemno:
        occ.grid[objgrid[0]][objgrid[1]] = itemno
        
def checkobjectlogged(itemno):
    ang = angle(0)
    endodom = endpos(ang, laser.front)
    objgrid = to_grid(endodom[0], endodom[1])
    if checksurrounding(objgrid[0], objgrid[1], itemno) == 0:
        return False
    else:
        return True
    
def checklaserinit():
    initialised = True
    if laser.front == 0 and laser.left == 0 and laser.right == 0 and laser.back == 0:
        if laser.frontleft == 0 and laser.frontright == 0 and laser.backleft == 0 and laser.backright == 0:
            if laser.frontfrontleft == 0 and laser.frontfrontright == 0 and laser.leftfrontleft == 0 and laser.rightfrontright == 0:
                if laser.backbackleft == 0 and laser.backbackright ==0 and laser.leftbackleft == 0 and laser.rightbackright == 0:
                    initialised = False
    return initialised

def gotogreen():
    #If False - Cancel Movebase
    if laser.front > 0.7:
        mover.linear.x = 0.25
        mover.angular.z = -cam.greenerrorx / 100
        pub.publish(mover)
    elif laser.front < 0.7 and laser.front > 0.5:
        mover.linear.x = 0.1
        mover.angular.z = -cam.greenerrorx / 100
        pub.publish(mover)
    elif laser.front < 0.5:
        mover.linear.x = 0
        mover.angular.z = 0
        pub.publish(mover)
        logobject(2)

if __name__ == "__main__":
    try:
        #Initialise
        rospy.init_node('follower', anonymous=True)

        sub = rospy.Subscriber("/cmd_vel", Twist)
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        odom = odomdata()
        scan = LaserScan()
        r =rospy.Rate(10)
        mover = Twist()
        cam = camdata()
        laser = avoidobstacle()
        occ = occdata()

        while not rospy.is_shutdown():
            # ADD BEHAVIOUR - IF DOT IS CENTRAL: DO NOTHING
            rospy.loginfo('Red X, Y = '+str(cam.rederrorx)+', '+str(cam.rederrory))
            rospy.loginfo('Green X, Y = '+str(cam.greenerrorx)+', '+str(cam.greenerrory))
            rospy.loginfo('Blue X, Y = '+str(cam.blueerrorx)+', '+str(cam.blueerrory))
            #if cam.greenerrorx != -105 and cam.greenerrory != 105: #and laser.front!=0:
                #if str(laser.front) != 'inf': or != 0
                    #if checkobjectlogged(2) == False:
                        #Cancel Movebase
                        #gotogreen()
                        #rospy.loginfo('Green X, Y = '+str(cam.greenerrorx)+', '+str(cam.greenerrory))
                #else:
                    #Cancel Movebase
                    #gotogreen()
                    #rospy.loginfo('Green X, Y = '+str(cam.greenerrorx)+', '+str(cam.greenerrory))
            #if cam.blueerrorx != -105 and cam.blueerrory != 105:
            #    if cam.blueerrory > 35:
            #        rospy.loginfo('Try and turn away')
            #        mover.angular.z = cam.blueerrorx / 100
            #        rospy.loginfo('TURN VALUE = '+str(cam.blueerrorx / 100))
            #        pub.publish(mover)
            #    else:
            #        mover.angular.z = 0
            #        pub.publish(mover)
            #else:
            #    mover.angular.z = 0
            #    pub.publish(mover)
                
            #else:
            #    mover.linear.x = 0
            #    mover.angular.z = 0
            #    #pub.publish(mover)
            #    ang = angle(0)
            #    endodom = endpos(ang, laser.laserfront)
            #    rospy.loginfo('Green Object Position = '+str(endodom))
            #    rospy.loginfo('Green Object Grid Pos = '+str(to_grid(endodom[0], endodom[1])))
            #    rospy.loginfo('Bot Position = '+str(odom.prevx)+", "+str(odom.prevy))
            #    rospy.loginfo('Bot Grid Position = '+str(to_grid(odom.prevx, odom.prevy)))
                #Update Grid based on colour detected - If not blue - check if exists from a distance
                #If blue do not go to - only stop and turn if the centroid is in your path - but still log it 
                

    except rospy.ROSInterruptException:
        pass