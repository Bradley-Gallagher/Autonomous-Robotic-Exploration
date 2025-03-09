#!/usr/bin/python3
import rospy
import math
import tf.transformations

from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from math import sqrt

MAXSPEED = 0.4
ACCRATE = 0.01

class BotExplore():

	def __init__(self):
		#CORE VARIABLES
		self.speed = 0
		self.turnSpeed = 0
		self.twist = Twist()
		self.throttle = 1;

		#NODES
		self.laserSub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
		self.velSub = rospy.Subscriber('/cmd_vel', Twist, self.speed_callback)
        self.velPub = rospy.Publisher('cmd_vel2', Twist, queue_size=10)

    # get/set
    def speed_callback(self, msg):
    	self.speed = msg.linear.x
    	self.turnSpeed = msg.linear.z

   	def laser_callback(self, msg):
   		#TODO insert CODE

   	def max_throttle(self):
   		self.throttle = 1

   	def set_throttle(self, v):
   		self.throttle = v

   	def accelerate(self):
   		self.throttle = self.throttle + ACCRATE

   	def deaccelerate(self):
   		self.throttle = self.throttle - ACCRATE

   	def get_speed(self)
   		return MAXSPEED * self.throttle


   	# main control code
    def main(self):
    	while(not rospy.is_shutdown()):
    		if(self.speed == 0 and self.turnSpeed == 0):
    			self.twist.linear.x = get_speed()
    		else:
	    		wander()

	def wander():
		#minitask 3 code goes in here 

	def randomturn(self):

	    # CODE FOR DETERMINING LEFT OR RIGHT TURN
	    turnval = random.randint(0, math.floor(math.pi * PRECISION))/PRECISION
	    leftOrRight = random.choice([-1, 1])
	    turnval = turnval * leftOrRight

	    #turn lmao
	    while leftOrRight * (odom.prevtheta - turnval) < 0:
	        if odom.prevtheta + (leftOrRight * MAXSPEED * SLOWDOWN) > turnval: #slow turn speed at the end of the turn to allow greater accuracy
	            mover.angular.z = MAXSPEED * SLOWDOWN
	        else:
	            mover.angular.z = MAXSPEED

	        pub.publish(mover)
	        r.sleep()

	    mover.angular.z = 0
	    pub.publish(mover)
	    r.sleep()

	    rospy.loginfo('Finished Turn')
