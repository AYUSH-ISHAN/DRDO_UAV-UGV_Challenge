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
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

rospy.init_node('marksub', anonymous=True)
x,y,z=0.25,1,0
rx,ry,rz,rw=0,0,0,0
odom=Odometry()
pose_pub=rospy.Publisher('/prius/base_pose_ground_truth',Odometry,queue_size=10)
f=1
ix,iy,iz=0.25,1,0
irx,iry,irz,irw=0,0,0,0
vx,vy,vz,ax,ay,az=0,0,0,0,0,0
def p1_callback(data):
    global x,y,z,rx,ry,rz,rw,f,ix,iy,iz,irx,iry,irz,irw,vx,vy,vz,ax,ay,az
    # print(data.name)
    i=data.name.index("prius::base_link_ugv")
    x=data.pose[i].position.x
    y=data.pose[i].position.y
    z=data.pose[i].position.z
    rx=data.pose[i].orientation.x
    ry=data.pose[i].orientation.y
    rz=data.pose[i].orientation.z
    rw=data.pose[i].orientation.w
    vx=data.twist[i].linear.x
    vy=data.twist[i].linear.y
    vz=data.twist[i].linear.z
    ax=data.twist[i].angular.x
    ay=data.twist[i].angular.y
    az=data.twist[i].angular.z
    
    if f==1:
         ix=x
         iy=y
         iz=z
         irx=rx
         iry=ry
         irz=rz
         irw=rw
         f=2
 
    br.sendTransform([x,y,z],[rx,ry,rz,rw],rospy.Time.now(),"base_link_ugv","odom")
  

def main():
   m1=rospy.Subscriber('/gazebo/link_states',LinkStates,p1_callback,queue_size=1)
   rate=rospy.Rate(100)
   while not rospy.is_shutdown():
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "map"
    
    # set the position
    odom.pose.pose = Pose(Point(x, y, z), Quaternion(rx,ry,rz,rw))

    # set the velocity
    odom.child_frame_id = "base_link_ugv"
    odom.twist.twist = Twist(Vector3(vx, vy, vz), Vector3(ax, ay, az))

    pose_pub.publish(odom)

    rate.sleep()

br = tf.TransformBroadcaster()
tfl=tf.TransformListener()
main()
rospy.spin()
