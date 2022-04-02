#!/usr/bin/env python3
import rospy
import math
import tf
import numpy
import roslib
import random
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

rospy.init_node('plane', anonymous=True)
x,y,z=0.25,1,0
rx,ry,rz,rw=0,0,0,0
ps=PoseStamped()
ps.header.stamp=rospy.Time.now()
pose_pub=rospy.Publisher('/planeodom',PoseStamped,queue_size=10)
f=1
ix,iy,iz=0.25,1,0
irx,iry,irz,irw=0,0,0,0

def p1_callback(data):
    global x,y,z,rx,ry,rz,rw,f,ix,iy,iz,irx,iry,irz,irw
    # print(data.name)
    x=data.pose.position.x
    y=data.pose.position.y
    z=data.pose.position.z
    rx=data.pose.orientation.x
    ry=data.pose.orientation.y
    rz=data.pose.orientation.z
    rw=data.pose.orientation.w
#    print(x,y)

def main():
   m1=rospy.Subscriber('/odometry/filtered',Odometry,p1_callback,queue_size=1)
   rate=rospy.Rate(100)
   while not rospy.is_shutdown():
       ps.pose.position.x=x
       ps.pose.position.y=y
       ps.pose.position.z=0
       ps.pose.orientation.x=rx
       ps.pose.orientation.y=ry
       ps.pose.orientation.z=rz
       ps.pose.orientation.w=rw
       pose_pub.publish(ps)
       print("Sent")
    #    tfl.waitForTransform("odom","iris_actual",rospy.Time(0),rospy.Duration(4.0))
    #    (trans,rot)=tfl.lookupTransform('odom','iris_actual',rospy.Time(0))
    # #    print(trans,rot)
    #    (transx,rotx)=tfl.lookupTransform('iris_actual','ugv',rospy.Time(0))
    #    print(transx,rotx)
       rate.sleep()

br = tf.TransformBroadcaster()
tfl=tf.TransformListener()
main()
rospy.spin()