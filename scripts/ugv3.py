#!/usr/bin/env python3
from importlib import import_module
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
from skimage.transform import resize
# IMPORTS
import pandas as pd
import torch
from torch.utils.data import DataLoader
import torchvision.transforms.functional as TF
import torch.nn as nn
import numpy as np
import albumentations as A
import torch.optim as optim
from albumentations.pytorch.transforms import ToTensorV2
import torchvision.transforms as transforms
from PIL import Image as im
import torch.nn.functional as F
import os
import torch
import torch.nn as nn
import torchvision.transforms.functional as TF

# device = "cuda" if torch.cuda.is_available() else "cpu"
device="cpu"
img=Image()
bridge=CvBridge()
rgbimg=Image()
depthimg=Image()
rospy.init_node('depthdata', anonymous=True)
mask=rospy.Publisher('/inter_img',Image,queue_size=10)
rate=rospy.Rate(100)
# UNET





class DoubleConv(nn.Module):
    def __init__(self, in_channels, out_channels):
        super(DoubleConv, self).__init__()
        self.conv = nn.Sequential(
            nn.Conv2d(in_channels, out_channels, 3, 1, 1, bias=False),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True),
            nn.Conv2d(out_channels, out_channels, 3, 1, 1, bias=False),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True),
        )

    def forward(self, x):
        return self.conv(x)

class UNET(nn.Module):
    def __init__(
            self, in_channels=3, out_channels=1, features=[64, 128, 256, 512],
    ):
        super(UNET, self).__init__()
        self.downs = nn.ModuleList()
        self.ups = nn.ModuleList()
        self.pool = nn.MaxPool2d(kernel_size=2, stride=2)

        # Down part of UNET
        for feature in features:
            self.downs.append(DoubleConv(in_channels, feature))
            in_channels = feature

        # Up part of UNET
        for feature in reversed(features):
            self.ups.append(
                nn.ConvTranspose2d(
                    feature*2, feature, kernel_size=2, stride=2,
                )
            )
            self.ups.append(DoubleConv(feature*2, feature))

        self.bottleneck = DoubleConv(features[-1], features[-1]*2)
        self.final_conv = nn.Conv2d(features[0], out_channels, kernel_size=1)

    def forward(self, x):
        skip_connections = []

        for down in self.downs:
            x = down(x)
            skip_connections.append(x)
            x = self.pool(x)

        x = self.bottleneck(x)
        skip_connections = skip_connections[::-1]

        for idx in range(0, len(self.ups), 2):
            x = self.ups[idx](x)
            skip_connection = skip_connections[idx//2]

            if x.shape != skip_connection.shape:
                x = TF.resize(x, size=skip_connection.shape[2:])

            concat_skip = torch.cat((skip_connection, x), dim=1)
            x = self.ups[idx+1](concat_skip)

        return self.final_conv(x)

val_transforms = A.Compose(
    [
        A.Resize(height=128, width= 128),
        A.Normalize(
            mean = 0.0,
            std = 1.0, 
            max_pixel_value = 255.0,
        ),
        ToTensorV2(),
    ]
)
# print(rgbimg.shape)
model = UNET(in_channels = 4, out_channels = 1).to(device)
# print("Load")
model.load_state_dict(torch.load("/home/sandeepan/iit_ws/src/interiit22/scripts/10Kepochsunet114imagestrainedweights.pt",map_location=torch.device(device)))
# rgbim=np.asarray(rgbimg)
def depthcall(data):
   global img, depthimg
   try:
     img = bridge.imgmsg_to_cv2(data, "32FC1")
     depthimg=img
     depthimg = cv2.cvtColor(depthimg, cv2.COLOR_GRAY2BGR)
     cv_image_array = np.array(img, dtype = np.dtype('f8'))
     cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
     #img has the depth values and cv_image_norm is img normalised from 0 to 1
     
   except CvBridgeError as e:
      print(e)

def rgbcall(data):
      global rgbimg, img
      img = bridge.imgmsg_to_cv2(data)
      rgbimg=img
      i = rgbimg
  
      i = np.asarray(i)
      # i = i.resize((128, 128))
      print(i.shape)
      # print(np.size(i))
      # np.resize(i, (128, 128, 3))
      # print(i.shape)
      if(i.shape[0]==3 or i.shape[1]==3 or i.shape[2]==3):
          li = [eh for eh in i]
          # li.append(np.ones((128, 128)))
          li = np.array(li)
          array = np.ones((480, 640))
          i = np.dstack((li, array))
          print(i.shape)
      
      augmentations = val_transforms(image=i)
      i = augmentations["image"]
      
      
      i = i.unsqueeze(0).to(device)
      ipred = model(i)
      # d = im.fromarray(depthimg)
      d=depthimg
      
      d = np.asarray(d)
    #   d = resize(d, (128, 128))
      # d = d.resize((128, 128))
      # np.resize(d, (128, 128, 3))
      print(d.shape)
      if(d.shape[0]==3 or d.shape[1]==3 or d.shape[2]==3):
          li = [eh for eh in d]
          li = np.array(li)
          array = np.ones((480, 640))
          d = np.dstack((li, array))
          print(d.shape)
      
      daugmentations = val_transforms(image=d)
      d = daugmentations["image"]
      
      
      d = d.unsqueeze(0).to(device)
      dpred = model(d)
      ipred = ipred.detach().numpy()
      ipred = ipred.astype(np.uint8)
      dpred = dpred.detach().numpy()
      d = d.detach().numpy()
      dpred = dpred.astype(np.uint8)
      d = d.astype(np.uint8)
    #   print(ipred[0])
      bitand = np.bitwise_and(ipred, d)
      image = bridge.cv2_to_imgmsg(ipred[0][0])
      mask.publish(image)
      
      cv2.imshow('img', ipred[0][0])
      cv2.waitKey(10)
            
m1=rospy.Subscriber('/depth_camera/depth/image_raw',Image,depthcall,queue_size=1)
m2=rospy.Subscriber('/depth_camera/rgb/image_raw',Image,rgbcall,queue_size=1)
      
       
while not rospy.is_shutdown():
       rate.sleep()