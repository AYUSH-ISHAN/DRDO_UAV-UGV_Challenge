#!/usr/bin/env python3
from __future__ import print_function
from itertools import count

from matplotlib.pyplot import contour

import roslib
roslib.load_manifest('interiit22')
import sys
import rospy
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from math import *
from mavros import setpoint as SP



class Controller:
    
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/depth_camera/depth/image_raw",Image,self.depth_img)
        self.motion_sub = rospy.Subscriber("/depth_camera/rgb/image_raw", Image, self.rgb_img)


    def depth_img(self, data):
        self.imgd = self.bridge.imgmsg_to_cv2(data, "32FC1")
        self.imgd = np.nan_to_num(self.imgd)
        self.imgd *= 25.5
        self.imgd = self.imgd.astype(int)

    def rgb_img(self, data):
        self.imgrgb = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def controlling(self):#, data1):
        lower_d = np.array([0,0])
        higher_d = np.array([0,0])
        lower_rgb = np.array([35, 35, 35])
        upper_rgb = np.array([50, 50, 50])

        try:
            # mask = cv2.inRange(self.imgrgb, lower_rgb, upper_rgb)
            # contours, _= cv2.findContours(mask, cv2.RETR_TREE,
            #                    cv2.CHAIN_APPROX_SIMPLE)
            # for i, c in enumerate(contours):
 
            #     area = cv2.contourArea(c)
                
            #     # Ignore contours that are too small or too large
            #     if area < 7000:
            #         continue
                
            #     # Draw each contour only for visualisation purposes

                
            #     cv2.drawContours(self.imgrgb, contours, i, (0, 0, 255), 2)
                
            print(np.amax(self.imgd), self.imgd.shape, np.amin(self.imgd))
            ret, thresh1 = cv2.threshold(self.imgd, 0, 10, cv2.THRESH_BINARY)
            cv2.imshow("rgb image", self.imgrgb)
            cv2.imshow("depth_img", self.imgd)
            cv2.imshow("rgb maks", thresh1)
            cv2.waitKey(10)
        except Exception as e:
            print(e)
            

def main(args):
    rospy.init_node('controller', anonymous=True)
    ic = Controller()
    while not rospy.is_shutdown():
        ic.controlling()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        print("Shutting down")
    #   cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

