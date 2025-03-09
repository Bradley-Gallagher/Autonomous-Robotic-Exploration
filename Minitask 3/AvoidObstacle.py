#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
from math import sqrt
import tf.transformations
import cv2, cv_bridge
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
import numpy as np

class avoidobstacle:
    def __init__(self):
        self.sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.pub = rospy.Publisher('scan2', LaserScan, queue_size=10)
        self.laserfrleft = 0 #front left
        self.laserfrright = 0 #front right
        self.laserfront = 0 # front
        self.laserleft = 0 # left hand side
        self.laserright = 0 # right hand side
        self.laserbkleft = 0 #back left
        self.laserbkright = 0 #back right
        self.laserback = 0 # back
        

    def laser_callback(self, msg): 
        self.pub.publish(scan)
        #change to broader ranges - e.g., 0-15
        self.laserfrleft = msg.ranges[1:45]
        self.laserfrright = msg.ranges[315:359]
        self.laserfront = msg.ranges[0]
        self.laserleft = msg.ranges[90]
        self.laserright = msg.ranges[270]
        self.laserbkleft = msg.ranges[150]
        self.laserbkright = msg.ranges[210]
        self.laserback = msg.ranges[180]

class speed:
    def __init__(self):
        self.sub = rospy.Subscriber('/cmd_vel', Twist, self.speed_callback)
        self.pub = rospy.Publisher('cmd_vel2', Twist, queue_size=10)
        self.speed = 0
        self.turn = 0

    def speed_callback(self, msg):
        self.speed = msg.linear.x
        self.turn = msg.angular.z

def obstacle():
    #Slow Down near Obstacles
    if avoid.laserfront < 0.5 or min(avoid.laserfrright) < 0.5 or min(avoid.laserfrleft) < 0.5:
        mover.linear.x = 0.1
        pub.publish(mover)
    #In range of obstacles turn left (for right hand following)
    if avoid.laserfront < 0.3 or min(avoid.laserfrright) < 0.3 or min(avoid.laserfrleft) < 0.3:
        mover.linear.x = 0
        mover.angular.z = 0
        pub.publish(mover)
        while avoid.laserfront < 0.5 or min(avoid.laserfrright) < 0.5 or min(avoid.laserfrleft) < 0.5: #Needs to remain WHILE - IF Breaks obstacle avoidance
            mover.linear.x = 0
            mover.angular.z = 0.2
            pub.publish(mover)
        mover.angular.z = 0
        pub.publish(mover)

def stop():

    #In range of obstacles turn left (for right hand following)
    if avoid.laserfront < 0.3 or min(avoid.laserfrright) < 0.3 or min(avoid.laserfrleft) < 0.3:
        mover.linear.x = 0
        mover.angular.z = 0
        pub.publish(mover)
    elif avoid.laserfront < 0.6 or min(avoid.laserfrright) < 0.6 or min(avoid.laserfrleft) < 0.6:
        mover.linear.x = 0.1
        pub.publish(mover)


class camdata:
    def __init__(self):
        self.imagesub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.camera_callback)
        self.pub = rospy.Publisher('camera2', Image, queue_size=10)
        self.bridge = cv_bridge.CvBridge()
        self.errorx = 0
        self.errory = 0
        #cv2.namedWindow("original",1)    
        

    def camera_callback(self, msg): 
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        (h, w) = img.shape[:2]
        img_resized = cv2.resize(img, (int(w/4), int(h/4)))
        img = img_resized
        height, width, channels = img.shape
        #Convert to HSV and Get Green Only
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lowergreen = np.array([50, 20, 20], dtype = "uint8")
        uppergreen = np.array([80, 255, 255], dtype = "uint8")
        mask = cv2.inRange(hsv, lowergreen, uppergreen)
        res = cv2.bitwise_and(img, img, None, mask=mask)

        #Calculate Centroid of colour - if centroid is not centre, turn and move forward - if centroid at bottom - wander
        m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cx, cy = height/2, width/2
        cv2.circle(res, (int(cx), int(cy)), 10,(0,0,255),-1)

        #Move based on Circle
        self.errorx = cx - width/2
        self.errory = cy - height/2

if __name__ == "__main__":
    try:
        #Initialise
        rospy.init_node('avoidobstacle', anonymous=True)
        avoid = avoidobstacle()
        scan = LaserScan()
        r =rospy.Rate(10)
        spd = speed()
        mover = Twist()
        cam = camdata()
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        while not rospy.is_shutdown():
            if spd.speed > 0 and cam.errory > 100:
                obstacle()
            if spd.speed > 0 and cam.errory < 100:
                stop()
    except rospy.ROSInterruptException:
        pass