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

class wallfollow:
    def __init__(self):
        self.sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.pub = rospy.Publisher('scan2', LaserScan, queue_size=10)
        self.laserfront = 0
        self.laserright = 0 # right hand side
        self.laserrightrange = 0
        self.laserfrright = 0
        self.laserfwdright = 0
        self.laserbkright = 0
        self.lasersdright = 0

    def laser_callback(self, msg): 
        self.pub.publish(scan)
        self.laserfront = msg.ranges[0]
        self.laserright = msg.ranges[270]
        self.laserfrright = msg.ranges[315]
        self.laserfwdright = msg.ranges[330]
        self.lasersdright = msg.ranges[300]
        self.laserbkright = msg.ranges[190:240]
        self.laserrightrange = msg.ranges[215:250] 

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

class odomcalc:
    def __init__(self):
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.pub = rospy.Publisher('odom2', Odometry, queue_size=10)
        self.prevtheta = 0
        self.prevx = 0
        self.prevy = 0
        self.distance = 0
        self.firstmove = True

    def odom_callback(self, msg): 
        quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
        if self.firstmove == True:
            self.prevx = msg.pose.pose.position.x
            self.prevy = msg.pose.pose.position.y
            self.prevtheta = yaw
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.pub.publish(msg)
        self.distance += math.sqrt(((self.prevx - msg.pose.pose.position.x)**2)+((self.prevy - msg.pose.pose.position.y)**2))
        self.prevx = msg.pose.pose.position.x
        self.prevy = msg.pose.pose.position.y
        self.prevtheta = yaw
        self.firstmove = False

def walltrack():
    #detect wall in range

    #Tested - Working (right range < can be adjusted)
    if not wall.laserright < 0.5 and min(wall.laserrightrange) < 0.4:
        mover.angular.z = -0.2
        mover.linear.x = 0 # Not Tested with this value
        pub.publish(mover)
        rospy.loginfo('TURN BACK')
    
    #avoid turning into obstacle
    #Tested - WORKING
    if wall.laserfrright < 0.2 or wall.laserfwdright < 0.2 or wall.lasersdright < 0.2: #Tested with 0.3
        mover.linear.x = 0.1
        mover.angular.z = 0.1
        pub.publish(mover)
        rospy.loginfo('TURNING AWAY FORWARD')

    #Tested - WORKING
    if wall.laserright < 0.5 and wall.laserfrright > 0.5 and wall.laserfwdright > 0.5 and wall.lasersdright > 0.5:
        mover.angular.z = -0.3
        mover.linear.x = 0.1
        pub.publish(mover)
        #rospy.loginfo('MAX TURN')
        if wall.laserright < 0.4:
            mover.angular.z = -0.2
            mover.linear.x = 0.1
            pub.publish(mover)
            #rospy.loginfo('NORMAL TURN')
            if wall.laserright < 0.3:
                mover.angular.z = 0
                mover.linear.x = 0.1
                pub.publish(mover)
                #rospy.loginfo('STOP TURN')
                if wall.laserright < 0.2:
                    mover.angular.z = 0.1
                    #mover.linear.x = 0
                    pub.publish(mover)
                    #rospy.loginfo('TURN AWAY')
    #else: #Remove?
     #   mover.linear.x = 0.1
     #   pub.publish(mover)
    if wall.laserright > 0.5 and mover.linear.x > 0:
        mover.angular.z = 0
        mover.linear.x = 0.2
        pub.publish(mover)


if __name__ == "__main__":
    try:
        #Initialise
        rospy.init_node('wallfollow', anonymous=True)
        sub = rospy.Subscriber("/cmd_vel", Twist, queue_size=10)
        wall = wallfollow()
        odom = odomcalc()
        scan = LaserScan()
        r =rospy.Rate(10)
        spd = speed()
        mover = Twist()
        cam = camdata()
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        while not rospy.is_shutdown():
            if spd.speed == 0 and spd.turn == 0 and cam.errory > 100:
                mover.linear.x = 0.2
                pub.publish(mover)
            if (spd.speed > 0) and cam.errory > 100:
                walltrack()

    except rospy.ROSInterruptException:
        pass