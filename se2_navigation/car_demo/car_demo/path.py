#!/usr/bin/env python3

import sys
from xml.etree.ElementTree import TreeBuilder
import rospy
import actionlib
from se2_navigation_msgs.msg import PathRequestMsg
from se2_navigation_msgs.msg import ControllerCommandMsg
from se2_navigation_msgs.srv import *
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import os
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal

def active_cb():
  rospy.loginfo("Goal being processed")
def feedback_cb(feedback):
#  rospy.loginfo("Current="+str(feedback))
  pass
def done_cb(status,result):
  if status==3:
     rospy.loginfo("Goal reached")
  elif status==2 or status==8:
     rospy.loginfo("Goal cancelled")
  elif status==4:
     rospy.loginfo("Goal aborted")

def extract_data(data):
    array=[]

    # os.system("rosnode kill /move_base")
    print("Poses",len(data.poses))
    for i in range(len(data.poses)-1):
        xc=data.poses[i].pose.position.x
        yc=data.poses[i].pose.position.y
        oxc=data.poses[i].pose.orientation.x
        oyc=data.poses[i].pose.orientation.y
        ozc=data.poses[i].pose.orientation.z
        owc=data.poses[i].pose.orientation.w
        if i==0:
           xo=xc
           yo=yc
           oxo=oxc
           oyo=oyc
           ozo=ozc
           owo=owc
           continue
        if ((xo-xc)*2 + (yo-yc)*2)>25:
            move_path(xo,yo,0,oxo,oyo,ozo,owo,xc,yc,0,oxc,oyc,ozc,owc)
            rospy.sleep(3)
            array.append([xo,yo,0,oxo,oyo,ozo,owo,xc,yc,0,oxc,oyc,ozc,owc])
            xo=xc
            yo=yc
            oxo=oxc
            oyo=oyc
            ozo=ozc
            owo=owc
            print(i)

def move_path(sx,sy,sz,sox,soy,soz,sow,ex,ey,ez,eox,eoy,eoz,eow):
    rospy.wait_for_service('/se2_planner_node/ompl_rs_planner_ros/planning_service')
    try:
        move_car = rospy.ServiceProxy('/se2_planner_node/ompl_rs_planner_ros/planning_service',RequestPathSrv)
        con=PathRequestMsg()
        con.startingPose.position.x=sx
        con.startingPose.position.y=sy
        con.startingPose.position.z=sz
        con.startingPose.orientation.x=sox
        con.startingPose.orientation.y=soy
        con.startingPose.orientation.z=soz
        con.startingPose.orientation.w=sow
        con.goalPose.position.x=ex
        con.goalPose.position.y=ey
        con.goalPose.position.z=ez
        con.goalPose.orientation.x=eox
        con.goalPose.orientation.y=eoy
        con.goalPose.orientation.z=eoz
        con.goalPose.orientation.w=eow
        status=move_car(con)
        print(status)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    rospy.wait_for_service('/prius/controller_command_service')
    try:
        move_on = rospy.ServiceProxy('/prius/controller_command_service',SendControllerCommandSrv)
        con=ControllerCommandMsg()
        con.command=0
        status=move_on(con)
        print(status)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def move(gx,gy,gz,ox,oy,oz,ow):
 goal.target_pose.pose.position.x = gx
 goal.target_pose.pose.position.y = gy
 goal.target_pose.pose.orientation.z = gz
 goal.target_pose.pose.orientation.z = oz
 goal.target_pose.pose.orientation.x = ox
 goal.target_pose.pose.orientation.y = oy
 goal.target_pose.pose.orientation.w = ow
 navclient.send_goal(goal,done_cb,active_cb,feedback_cb) 
#  finished=navclient.wait_for_result()

if __name__ == "__main__":

    rospy.init_node('plan',anonymous=True)
    rospy.Subscriber('/move_base/TebLocalPlannerROS/global_plan',Path, extract_data)
    navclient=actionlib.SimpleActionClient('move_base',MoveBaseAction)
    navclient.wait_for_server()
    goal=MoveBaseGoal()
    rospy.spin()
    # move_path(0, 0 ,0, 0, 0, -0.920026, 0.3918572, -5.953334808349609, -21.258712768554688, 0, 0, 0, -0.920026, 0.3918572)