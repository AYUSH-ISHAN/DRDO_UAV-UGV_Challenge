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

rospy.init_node('marksub', anonymous=True)
x,y,z=0.25,1,0
rx,ry,rz,rw=0,0,0,0
ps=PoseStamped()
ps.header.stamp=rospy.Time.now()
pose_pub=rospy.Publisher('/cheat_pos',PoseStamped,queue_size=10)
f=1
ix,iy,iz=0.25,1,0
irx,iry,irz,irw=0,0,0,0

def p1_callback(data):
    global x,y,z,rx,ry,rz,rw,f,ix,iy,iz,irx,iry,irz,irw
    # print(data.name)
    i=data.name.index("iris::drone_with_depth_camera::iris::base_link")
    x=data.pose[i].position.x
    y=data.pose[i].position.y
    z=data.pose[i].position.z
    rx=data.pose[i].orientation.x
    ry=data.pose[i].orientation.y
    rz=data.pose[i].orientation.z
    rw=data.pose[i].orientation.w
    j=data.name.index("prius::base_link_ugv")
    cx=data.pose[j].position.x
    cy=data.pose[j].position.y
    cz=data.pose[j].position.z
    crx=data.pose[j].orientation.x
    cry=data.pose[j].orientation.y
    crz=data.pose[j].orientation.z
    crw=data.pose[j].orientation.w
    if f==1:
         ix=x
         iy=y
         iz=z
         irx=rx
         iry=ry
         irz=rz
         irw=rw
         f=2
    br.sendTransform([ix,iy,iz],[0,0,-0.7068252,0.7073883 ],rospy.Time.now(),"odom","world")
    br.sendTransform([x,y,z],[rx,ry,rz,rw],rospy.Time.now(),"iris_actual","world")
    br.sendTransform([cx,cy,cz],[crx,cry,crz,crw],rospy.Time.now(),"ugv","world")
#    print(x,y)

def main():
   m1=rospy.Subscriber('/gazebo/link_states',LinkStates,p1_callback,queue_size=1)
   rate=rospy.Rate(100)
   while not rospy.is_shutdown():
       ps.pose.position.x=x
       ps.pose.position.y=y
       ps.pose.position.z=z
       ps.pose.orientation.x=rx
       ps.pose.orientation.y=ry
       ps.pose.orientation.z=rz
       ps.pose.orientation.w=rw
       pose_pub.publish(ps)
       print("Sent")
       tfl.waitForTransform("odom","iris_actual",rospy.Time(0),rospy.Duration(4.0))
       (trans,rot)=tfl.lookupTransform('odom','iris_actual',rospy.Time(0))
    #    print(trans,rot)
       (transx,rotx)=tfl.lookupTransform('iris_actual','ugv',rospy.Time(0))
       print(transx,rotx)
       rate.sleep()

br = tf.TransformBroadcaster()
tfl=tf.TransformListener()
main()
rospy.spin()