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

rospy.init_node('waypointnav', anonymous=False)

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
    
def movebase_client(x, y):
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.__init__
    rospy.loginfo('Connecting to Server')
    client.wait_for_server()
    rospy.loginfo('Connect to Server')

    quaternions = list()
        
    # First define the corner orientations as Euler angles
    euler_angles = (pi/2, pi, 3*pi/2, 0)
        
    # Then convert the angles to quaternions
    for angle in euler_angles:
        q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
        q = Quaternion(*q_angle)
        quaternions.append(q)
        
    # Create a list to hold the waypoint poses
    waypoints = list()
    waypoints.append(Pose(Point(-6.5, -2, 0.0), quaternions[3]))
    waypoints.append(Pose(Point(1, 3, 0.0), quaternions[3]))
    waypoints.append(Pose(Point(6, -4, 0.0), quaternions[0]))
        
    markers = Marker()
    markers.points = list()

    for waypoint in waypoints:           
        p = Point()
        p = waypoint.position
        markers.points.append(p)


    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 0.5
    goal.target_pose.pose.orientation.z = -0.5
 
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.loginfo('Server Not Available')
        rospy.signal_shutdown('Server not Available')
    else:
        return client.get_result()       

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
        #change to broader ranges - e.g., 0-15
        self.front = msg.ranges[0]
        #left
        self.left = msg.ranges[90]
        #back
        self.back = msg.ranges[180]
        #right
        self.right = msg.ranges[270]

        self.full = msg.ranges[0:359] 


if __name__ == '__main__':
    try:
        
        #gridpub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
        odom = odomdata()
        scan = LaserScan()
        
        i = 0
        x = 0
        y = 0

        while not rospy.is_shutdown():
            if i == 0 :
                x = 2
                y = 2
                rospy.loginfo('Moving to Back Office')
            result = movebase_client(x, y)
            if result:
                rospy.loginfo('Goal Complete')
                i+=1
    except rospy.ROSInterruptException:
        rospy.loginfo('Navigation Test Finished')