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


def drawAxis(img, p_, q_, color, scale):
  p = list(p_)
  q = list(q_)
 
  ## [visualization1]
  angle = atan2(p[1] - q[1], p[0] - q[0]) # angle in radians
  hypotenuse = sqrt((p[1] - q[1]) * (p[1] - q[1]) + (p[0] - q[0]) * (p[0] - q[0]))
 
  # Here we lengthen the arrow by a factor of scale
  q[0] = p[0] - scale * hypotenuse * cos(angle)
  q[1] = p[1] - scale * hypotenuse * sin(angle)
  cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv2.LINE_AA)
 
  # create the arrow hooks
  p[0] = q[0] + 9 * cos(angle + pi / 4)
  p[1] = q[1] + 9 * sin(angle + pi / 4)
  cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv2.LINE_AA)
 
  p[0] = q[0] + 9 * cos(angle - pi / 4)
  p[1] = q[1] + 9 * sin(angle - pi / 4)
  cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv2.LINE_AA)
  ## [visualization1]
 
def getOrientation(pts, img):
  ## [pca]
  # Construct a buffer used by the pca analysis
  sz = len(pts)
  data_pts = np.empty((sz, 2), dtype=np.float64)
  for i in range(data_pts.shape[0]):
    data_pts[i,0] = pts[i,0,0]
    data_pts[i,1] = pts[i,0,1]
 
  # Perform PCA analysis
  mean = np.empty((0))
  mean, eigenvectors, eigenvalues = cv2.PCACompute2(data_pts, mean)
 
  # Store the center of the object
  cntr = (int(mean[0,0]), int(mean[0,1]))
  ## [pca]
 
  ## [visualization]
  # Draw the principal components
  cv2.circle(img, cntr, 3, (255, 0, 255), 2)
  p1 = (cntr[0] + 0.02 * eigenvectors[0,0] * eigenvalues[0,0], cntr[1] + 0.02 * eigenvectors[0,1] * eigenvalues[0,0])
  p2 = (cntr[0] - 0.02 * eigenvectors[1,0] * eigenvalues[1,0], cntr[1] - 0.02 * eigenvectors[1,1] * eigenvalues[1,0])
  drawAxis(img, cntr, p1, (255, 255, 0), 1)
  drawAxis(img, cntr, p2, (0, 0, 255), 5)
 
  angle = atan2(eigenvectors[0,1], eigenvectors[0,0]) # orientation in radians
  ## [visualization]
 
  # Label with the rotation angle
  label = "  Rotation Angle: " + str(-int(np.rad2deg(angle)) - 90) + " degrees"
  textbox = cv2.rectangle(img, (cntr[0], cntr[1]-25), (cntr[0] + 250, cntr[1] + 10), (255,255,255), -1)
  cv2.putText(img, label, (cntr[0], cntr[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)
 
  return angle

class Controller:
    
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/depth_camera/depth/image_raw",Image,self.depth_img)
        self.motion_sub = rospy.Subscriber("/depth_camera/rgb/image_raw", Image, self.rgb_img)


    def depth_img(self, data):
        self.imgd = self.bridge.imgmsg_to_cv2(data, "32FC1")

    def rgb_img(self, data):
        self.imgrgb = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def controlling(self):#, data1):
        lower_d = np.array([0,0])
        higher_d = np.array([0,0])
        lower_rgb = np.array([60, 60, 60])
        upper_rgb = np.array([200, 200, 200])

        try:
            mask = cv2.inRange(self.imgrgb, lower_rgb, upper_rgb)
            contours, _= cv2.findContours(mask, cv2.RETR_TREE,
                               cv2.CHAIN_APPROX_SIMPLE)
            for i, c in enumerate(contours):
 
                area = cv2.contourArea(c)
                
                # Ignore contours that are too small or too large
                if area < 7000:
                    continue
                
                # Draw each contour only for visualisation purposes

                M = cv2.moments(c)
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                cv2.drawContours(self.imgrgb, contours, i, (0, 0, 255), 2)
                
                # Find the orientation of each shape
                angle = getOrientation(c, self.imgrgb)
                print(self.imgd)
            cv2.imshow("rgb image", self.imgrgb)
            cv2.imshow("depth_img", self.imgd)
            cv2.imshow("rgb maks", mask)
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


