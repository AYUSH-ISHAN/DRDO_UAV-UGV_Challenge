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
import csv

img=Image()
bridge=CvBridge()
f=1
def pcall(data):
   global img,f 
   try:
     img = bridge.imgmsg_to_cv2(data, "32FC1")
     cv_image_array = np.array(img, dtype = np.dtype('f8'))
     print(cv_image_array)
     print(img.shape)
     cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
     for i in range(img.shape[0]):
          for j in range(img.shape[1]):
             if np.isnan(img[i][j]):
               img[i][j]=40
     cv2.imshow('mg',cv_image_norm)
     cv2.waitKey(30)
     if f==1:
        with open('/home/sandeepan/data.csv','a') as myfile:
          wr = csv.writer(myfile)
          wr.writerows(img)
     f+=1
   except CvBridgeError as e:
      print(e)

rospy.init_node('depthdata', anonymous=True)
m1=rospy.Subscriber('/depth_camera/depth/image_raw',Image,pcall,queue_size=1)
rate=rospy.Rate(100)
while not rospy.is_shutdown():
       rate.sleep()