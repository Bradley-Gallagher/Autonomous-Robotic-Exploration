#!/usr/bin/python3
from sensor_msgs.msg import LaserScan
import rospy
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
import math
from math import sqrt
import tf.transformations
import random


class randomwalk:
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

class speed:
    def __init__(self):
        self.sub = rospy.Subscriber('/cmd_vel', Twist, self.speed_callback)
        self.pub = rospy.Publisher('cmd_vel2', Twist, queue_size=10)
        self.speed = 0
        self.turn = 0

    def speed_callback(self, msg):
        self.speed = msg.linear.x
        self.turn = msg.angular.z

# Drive straight along the X axis
def randommove():
    #rospy.loginfo('Move Start Position')
    #rospy.loginfo(odom.prevx)
    #rospy.loginfo(odom.prevy)
    #rospy.loginfo(odom.prevtheta)
    #rospy.loginfo('Moving')
    speed = 0.2
    if odom.distance >= 2: #Change to 1M for lab purposes
        rospy.loginfo('STOP & TURN')
        mover.linear.x = 0
        pub.publish(mover)
        randomturn()
        odom.distance = 0

    #rospy.loginfo('Move End Position')
    #rospy.loginfo(odom.prevx)
    #rospy.loginfo(odom.prevy)
    #rospy.loginfo(odom.prevtheta)
    

def randomturn():
    turnval = random.randint(-3145, 3145)/1000
    # CODE FOR DETERMINING LEFT OR RIGHT TURN
    # Left
    rospy.loginfo('Turning: Theta, Random Turn Data')
    rospy.loginfo(odom.prevtheta)
    rospy.loginfo(turnval)
    # Left
    while odom.prevtheta < turnval:
        if odom.prevtheta + 0.2 > turnval: #slow turn speed at the end of the turn to allow greater accuracy
            mover.angular.z = 0.2
        else:
            mover.angular.z = 0.4
        pub.publish(mover)
        r.sleep()
    # Right
    while odom.prevtheta > turnval:
        if odom.prevtheta - 0.2 > turnval: #slow turn speed at the end of the turn to allow greater accuracy
            mover.angular.z = -0.2
        else:
            mover.angular.z = -0.4
        pub.publish(mover)
        r.sleep()
    mover.angular.z = 0
    pub.publish(mover)
    # Print current position and direction
    #rospy.loginfo('Turn Position')
    #rospy.loginfo(odom.prevx)
    #rospy.loginfo(odom.prevy)
    #rospy.loginfo(odom.prevtheta)

if __name__ == "__main__":
    try:
        #Initialise
        rospy.init_node('randommove', anonymous=True)
        odom = randomwalk()
        r =rospy.Rate(10)
        mover = Twist()
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        spd = speed()
        while not rospy.is_shutdown():
            if spd.speed == 0 and spd.turn == 0:
                mover.linear.x = 0.2
                pub.publish(mover)
            randommove()

    except rospy.ROSInterruptException:
        pass