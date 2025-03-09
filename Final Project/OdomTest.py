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

if __name__ == '__main__':
    try:
        rospy.init_node('odomtest', anonymous=False)
        
        #gridpub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
        odom = odomdata()
        scan = LaserScan()

        i = 0
        iteration = 0
        while not rospy.is_shutdown():
            rospy.loginfo(str(odom.prevx)+','+str(odom.prevy))
        

    except rospy.ROSInterruptException:
        rospy.loginfo('Navigation Test Finished')