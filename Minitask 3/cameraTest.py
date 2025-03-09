#!/usr/bin/python3
import rospy
import math
import tf.transformations
import cv2, cv_bridge
import numpy as np
import argparse

from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from math import sqrt




if __name__ == "__main__":

    img = cv2.imread("/home/team24/Pictures/test1.png")

    cv2.namedWindow("original",1)    

    if(img is None):
        print("broken")
    else:
        print(img) 

    # define the list of boundaries
    boundaries = [
        ([25, 25, 25], [40, 175, 40])
    ]

    # loop over the boundaries
    for (lower, upper) in boundaries:
        # create NumPy arrays from the boundaries
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")
        # find the colors within the specified boundaries and apply
        # the mask
        mask = cv2.inRange(img, lower, upper)
        output = cv2.bitwise_and(img, img, None, mask = mask)
        # show the images
        cv2.imshow("original", np.hstack([img, output]))
        cv2.waitKey(0)

    #cv2.imshow("original", img)
    #cv2.waitKey(0)

    print("finished")