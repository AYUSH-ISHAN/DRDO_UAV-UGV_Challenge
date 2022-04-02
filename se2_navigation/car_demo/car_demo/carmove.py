#!/usr/bin/env python3

from __future__ import print_function

import sys
import rospy
from se2_navigation_msgs.msg import ControllerCommandMsg
from se2_navigation_msgs.srv import *

def add_two_ints_client():
    rospy.wait_for_service('/prius/controller_command_service')
    try:
        move = rospy.ServiceProxy('/prius/controller_command_service',SendControllerCommandSrv)
        con=ControllerCommandMsg()
        con.command=0
        status=move(con)
        print(status)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    add_two_ints_client()