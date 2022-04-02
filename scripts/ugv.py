#!/usr/bin/env python3
import rospy
import math
import tf
import numpy
import roslib
import random
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

img=Image()
bridge=CvBridge()
rgbimg=Image()
depthimg=Image()
def depthcall(data):
   global img,f 
   try:
     img = bridge.imgmsg_to_cv2(data, "32FC1")
     depthimg=img
     cv_image_array = np.array(img, dtype = np.dtype('f8'))
     cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
     #img has the depth values and cv_image_norm is img normalised from 0 to 1
     
   except CvBridgeError as e:
      print(e)

def rgbcall(data):
       #this is the rgb image u can use opencv functions on it
      img = bridge.imgmsg_to_cv2(data)
      rgbimg=img
       

rospy.init_node('depthdata', anonymous=True)
m1=rospy.Subscriber('/depth_camera/depth/image_raw',Image,depthcall,queue_size=1)
m2=rospy.Subscriber('/depth_camera/rgb/image_raw',Image,rgbcall,queue_size=1)
mask=rospy.Publisher('/inter_img',Image,queue_size=10)
rate=rospy.Rate(100)
while not rospy.is_shutdown():
       rate.sleep()