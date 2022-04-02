#!/usr/bin/env python3
from __future__ import print_function
from itertools import count
from unicodedata import name

from matplotlib.pyplot import contour

import roslib
roslib.load_manifest('interiit22')
from skimage.transform import resize
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
import argparse
import os
import sys
from pathlib import Path

import cv2
import torch
import torch.backends.cudnn as cudnn
import numpy as np

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from models.common import DetectMultiBackend
from utils.datasets import IMG_FORMATS, VID_FORMATS, LoadImages, LoadStreams
from utils.general import (LOGGER, check_file, check_img_size, check_imshow, check_requirements, colorstr,
                           increment_path, non_max_suppression, print_args, scale_coords, strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import select_device, time_sync

def letterbox(im, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
    # Resize and pad image while meeting stride-multiple constraints
    shape = im.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:  # only scale down, do not scale up (for better val mAP)
        r = min(r, 1.0)

    # Compute padding
    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
    if auto:  # minimum rectangle
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding
    elif scaleFill:  # stretch
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])
        ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
    return im, ratio, (dw, dh)

class Controller:
    
    def __init__(self):
        weights = '/home/sandeepan/iit_ws/src/interiit22/scripts/best_old.pt'
        data = '/home/sandeepan/iit_ws/src/interiit22/scripts/data.yaml'
        dnn = False
        half = True
        device = 'cpu'
        imgsz = (640, 640)
        self.device = select_device(device)
        self.model = DetectMultiBackend(weights, data=data, fp16=False)
        stride, names, pt = self.model.stride, self.model.names, self.model.pt
        self.imgsz = check_img_size(imgsz, s=stride)  # check image size
        bs = 1 #batch size
        self.model.warmup(imgsz=(1 if pt else bs, 3, *self.imgsz))  # warmup
        self.bridge = CvBridge()
        self.cx=-1
        self.cy=320
        self.lx=50
        self.ly=50
        self.carz=10
        self.image_sub = rospy.Subscriber("/depth_camera/depth/image_raw",Image,self.depth_img,queue_size=1)
        self.motion_sub = rospy.Subscriber("/depth_camera/rgb/image_raw", Image, self.rgb_img,queue_size=1)
        
    def calc_xyz(self,k, u, v, z):
       f = k[0]
       cx = k[2]
       cy = k[5]
       if np.isnan(z):
            z=10
       x = -(u - cx)*z/f
       y = (cy - v)*z/f
       print(y,x,z)
    #    return [x, y, z]

    def depth_img(self, data):
        self.imgd1 = self.bridge.imgmsg_to_cv2(data, "32FC1")
        self.imgd=cv2.resize(self.imgd1, (640, 640))
        if self.cx!=-1:
            # print(self.imgd.shape)
            img = np.array(self.imgd, dtype = np.dtype('f8'))
            # cv2.imshow('depth',img)
            img2=img[round(self.cy-self.ly/2):round(self.cy+self.ly/2),round(self.cx-self.lx/2):round(self.cx+self.lx/2)]
            imgnorm=cv2.normalize(img2, img2, 0, 1, cv2.NORM_MINMAX)
            newimg=np.zeros([img2.shape[0],img2.shape[1],3])
            for i in range(imgnorm.shape[0]):
             for j in range(imgnorm.shape[1]):
                 if not (imgnorm[i][j]>0 and imgnorm[i][j]<=0.5):
                   newimg[i][j][0]=0
                   newimg[i][j][1]=0
                   newimg[i][j][2]=0
                 else:
                   newimg[i][j][0]=255
                   newimg[i][j][1]=255
                   newimg[i][j][2]=255
            newimg=newimg.astype(np.uint8)
            img = cv2.cvtColor(newimg, cv2.COLOR_BGR2GRAY)
            img2=img.copy()
            kern1 = np.ones((5, 5), np.uint8)
            kern2 = np.ones((3, 3), np.uint8)
            img = cv2.erode(img, kern2)
            img = cv2.dilate(img, kern1)
            contours, hierarchy = cv2.findContours(
                   img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            cnts = sorted(contours, key=lambda x: cv2.contourArea(x))
            print(len(cnts))
            if len(cnts)>0:
               car=cnts[len(cnts)-1]
               ellipse = cv2.fitEllipse(car)
               print(ellipse[2])
               self.carz=self.imgd[round(self.cy)][round(self.cx)]
               cv2.ellipse(img2,ellipse,(0,255,0),2)
            #    cv2.imshow('depth',img)
            cv2.imshow('og',imgnorm)
            cv2.waitKey(1)
    
    def rgb_img(self, data):
        self.imgrgb = self.bridge.imgmsg_to_cv2(data, "bgr8")
        im = self.imgrgb.copy()
        im, _, _ = letterbox(im)
        im = im.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        im = np.ascontiguousarray(im)
        im = torch.from_numpy(im).to(self.device)
        im = im.half() if self.model.fp16 else im.float()  # uint8 to fp16/32
        im /= 255  # 0 - 255 to 0.0 - 1.0
        if len(im.shape) == 3:
            im = im[None]  # expand for batch dim
        pred = self.model(im, augment=False, visualize=False)
        pred = non_max_suppression(pred, 0.2, 0.45, None, False, max_det=1000)
        print(pred)
        for i, det in enumerate(pred):
            im1 = self.imgrgb.copy()
            im0=cv2.resize(im1, (640, 640))
            # print(im1.shape)
            names = ['car']
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]
            annotator = Annotator(im0, line_width=3, example=str(names))
            if len(det):
                det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()
                for *xyxy, conf, cls in reversed(det):
                    xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()
                    c = int(cls)  # integer class
                    label =  f'{names[c]} {conf:.2f}'
                    annotator.box_label(xyxy, label, color=colors(c, True))
                im0 = annotator.result()
                # print("xyxy",xyxy)
                print(xywh)
                print(xywh[0]*640,xywh[1]*640,xywh[2]*640,xywh[3]*640)
                self.cx=xywh[0]*640
                self.cy=xywh[1]*640
                self.lx=xywh[2]*640
                self.ly=xywh[3]*640
                k=[205.46963709898583, 0.0, 320.5, 0.0, 205.46963709898583, 240.5, 0.0, 0.0, 1.0]
                self.calc_xyz(k,self.cx,self.cy,self.carz)
                # print(im0.shape)
                # img2=im0[round(cy-ly2):round(cy+ly/2),round(cx-lx/2):round(cx+lx/2)]
                # img2=im0[0:100,0:200]
                # cv2.imshow("img", im0)
            # cv2.waitKey(1)  

             


def main(args):
    rospy.init_node('controller', anonymous=True)
    ic = Controller()
    # while not rospy.is_shutdown():
    #     ic.controlling()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        print("Shutting down")
    #   cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)



