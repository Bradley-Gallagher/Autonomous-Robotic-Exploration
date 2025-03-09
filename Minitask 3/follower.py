#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
from math import sqrt
import tf.transformations
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
import cv2, cv_bridge
import numpy as np
import argparse


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
        #print("Calculated Error = "+str(cx - width/2))
        #mover.angular.x = 0.2
        #mover.angular.z = -errorx / 100

        #Output Camera after splitting colours
        cv2.imshow("original", res)
        cv2.waitKey(3)
    
class speed:
    def __init__(self):
        self.sub = rospy.Subscriber('/cmd_vel', Twist, self.speed_callback)
        self.pub = rospy.Publisher('cmd_vel2', Twist, queue_size=10)
        self.speed = 0
        self.turn = 0

    def speed_callback(self, msg):
        self.speed = msg.linear.x
        self.turn = msg.angular.z

class avoidobstacle:
    def __init__(self):
        self.sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.pub = rospy.Publisher('scan2', LaserScan, queue_size=10)
        self.laserfrleft = 0
        self.laserfrright = 0
        self.laserfront = 0
        

    def laser_callback(self, msg): 
        self.pub.publish(scan)
        #change to broader ranges - e.g., 0-15
        self.laserfrleft = msg.ranges[1:45]
        self.laserfrright = msg.ranges[315:359]
        self.laserfront = msg.ranges[0]

if __name__ == "__main__":
    try:
        #Initialise
        rospy.init_node('follower', anonymous=True)
        sub = rospy.Subscriber("/cmd_vel", Twist)
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        scan = LaserScan()
        r =rospy.Rate(10)
        mover = Twist()
        cam = camdata()
        laser = avoidobstacle()
        while not rospy.is_shutdown():
            # ADD BEHAVIOUR - IF DOT IS CENTRAL: DO NOTHING
            if cam.errory < 100 and laser.laserfront > 0.4:
                mover.linear.x = 0.2
                mover.angular.z = -cam.errorx / 100
                pub.publish(mover)
    except rospy.ROSInterruptException:
        pass