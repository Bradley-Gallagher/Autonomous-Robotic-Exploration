#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
from math import sqrt
import tf.transformations

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
        rospy.loginfo('Obstacle')
        while avoid.laserfront < 0.5 or min(avoid.laserfrright) < 0.5 or min(avoid.laserfrleft) < 0.5: #Needs to remain WHILE - IF Breaks obstacle avoidance
            mover.linear.x = 0
            mover.angular.z = 0.2
            pub.publish(mover)
        mover.angular.z = 0
        pub.publish(mover)

if __name__ == "__main__":
    try:
        #Initialise
        rospy.init_node('avoidobstacle', anonymous=True)
        avoid = avoidobstacle()
        scan = LaserScan()
        r =rospy.Rate(10)
        spd = speed()
        mover = Twist()
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        while not rospy.is_shutdown():
            if spd.speed == 0 and spd.turn == 0:
                mover.linear.x = 0.2
                pub.publish(mover)
            if spd.speed > 0:
                obstacle()
    except rospy.ROSInterruptException:
        pass