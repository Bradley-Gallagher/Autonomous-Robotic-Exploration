#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
import math
from math import sqrt
import tf.transformations

#Setup Odometry to call current position and direction during movements
class TurtlebotDriving:
    def __init__(self):
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.pub = rospy.Publisher('turtlebotdriving', Odometry, queue_size=10)
        self.prevtheta = 0
        self.prevx = 0
        self.prevy = 0
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
        self.prevx = msg.pose.pose.position.x
        self.prevy = msg.pose.pose.position.y
        self.prevtheta = yaw
        self.firstmove = False

    # Drive straight along the X axis
    def straightX(self, X):
        speed = 0.01
        if X > 0: 
            while odom.prevx < X:
                mover.linear.x = speed
                pub.publish(mover)
                #Slow down on approach
                if odom.prevx >= 0.8: #deceleration to minimise wheel drift under braking
                    if speed > 0.1:
                        speed -= 0.01
                else: 
                    if speed < 0.2: #controlled acceleration to minimise wheel drift under acceleration
                        speed += 0.01 
                r.sleep()
        if X <= 0:
            while odom.prevx > X:
                mover.linear.x = speed
                pub.publish(mover)
                if odom.prevx < 0.2:
                    if speed > 0.1:
                        speed -= 0.01
                else:
                    if speed < 0.2:
                        speed += 0.01
                r.sleep()
        mover.linear.x = 0
        pub.publish(mover)
        rospy.loginfo('Straight Position')
        rospy.loginfo(odom.prevx)
        rospy.loginfo(odom.prevy)
        rospy.loginfo(odom.prevtheta)

    # Drive straight along the Y axis
    def straightY(self, Y):
        speed = 0.01
        if Y > 0:
            while odom.prevy < Y:
                #start moving
                mover.linear.x = speed
                pub.publish(mover)
                #slow down on approach
                if odom.prevy >= 0.8:
                    if speed > 0.1:
                        speed -= 0.01
                else:
                    if speed < 0.2:
                        speed += 0.01
                r.sleep()
        if Y <= 0:
            while odom.prevy > Y:
                mover.linear.x = speed
                pub.publish(mover)
                if odom.prevy < 0.2:
                    if speed > 0.01:
                        speed -= 0.01
                else:
                    if speed < 0.2:
                        speed += 0.01
                r.sleep()
        mover.linear.x = 0
        pub.publish(mover)
        rospy.loginfo('Straight Position')
        rospy.loginfo(odom.prevx)
        rospy.loginfo(odom.prevy)
        rospy.loginfo(odom.prevtheta)

    # Turn the robot
    def turn(self, theta):
        #edge case 
        if theta != 3.14:
            while odom.prevtheta < theta:
                if odom.prevtheta + 0.2 > theta: #slow turn speed at the end of the turn to allow greater accuracy
                    mover.angular.z = 0.1
                    if odom.prevtheta +0.05 > theta: #continued deceleration to the turn speed
                        mover.angular.z = 0.02
                else:
                    mover.angular.z = 0.2
                pub.publish(mover)
                r.sleep()

        if theta == 3.14:
            while odom.prevtheta > 0:
                if odom.prevtheta + 0.2 > theta: #slow turn speed at the end of the turn to allow greater accuracy
                    mover.angular.z = 0.1
                    if odom.prevtheta +0.05 > theta: #continued deceleration to the turn speed
                        mover.angular.z = 0.02
                else:
                    mover.angular.z = 0.2
                pub.publish(mover)
                r.sleep()
        mover.angular.z = 0
        pub.publish(mover) 
        rospy.loginfo('Turn Position')
        rospy.loginfo(odom.prevx)
        rospy.loginfo(odom.prevy)
        rospy.loginfo(odom.prevtheta)
        r.sleep()

    def square(self):
        #1st Straight
        self.straightX(1)
        rospy.sleep(1)
        #1st Turn
        self.turn(1.57)
        rospy.sleep(1)
        #2nd Straight
        self.straightY(1)
        rospy.sleep(1)
        #2nd Turn
        self.turn(3.14)
        rospy.sleep(1)
        #3rd Straight
        self.straightX(0)
        rospy.sleep(1)
        #3rd Turn - set angle to be negative to allow Turn() to work
        while odom.prevtheta > 0 :
            mover.angular.z = 0.2
            pub.publish(mover)
            r.sleep()
        mover.angular.z = 0
        pub.publish(mover)
        #3rd Turn Cont.
        self.turn(-1.57)
        rospy.sleep(1)
        #4th Straight
        self.straightY(0)
        rospy.sleep(1)
        #4th Turn
        self.turn(0)
        r.sleep()

if __name__ == "__main__":
    try:
        #Initialise
        rospy.init_node('turtlebotdriving', anonymous=True)
        odom = TurtlebotDriving()
        r =rospy.Rate(10)
        mover = Twist()
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        while not rospy.is_shutdown():
        # Starting Position Data
            rospy.loginfo('Starting position: X, Y, Theta')
            rospy.loginfo(odom.prevx)
            rospy.loginfo(odom.prevy)
            rospy.loginfo(odom.prevtheta)

            odom.square()

            #Ending Position Data
            rospy.loginfo('Final position: X, Y, Theta')
            rospy.loginfo(odom.prevx)
            rospy.loginfo(odom.prevy)
            rospy.loginfo(odom.prevtheta)
    except rospy.ROSInterruptException:
        pass