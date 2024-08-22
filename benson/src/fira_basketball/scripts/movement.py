#! /usr/bin/python
import rospy
from op3_ros_utils import Robot
import sys

import cv2
import numpy as np
import math
import time   
76 
# init rosnode
rospy.init_node("fira_Archery_PF")
# rospy.init_node("fira_Archery_PF", anonymous=True)

# Create robot
robot = Robot()
#rospy.init_node("fira_Archery_PF")     #change to line 13
rospy.sleep(3) # Make sure every publisher has registered to their topic,
               # avoiding lost messages
               
if __name__ == '__main__':
    # while not rospy.is_shutdown():


    robot.setGeneralControlModule("action_module")
    robot.playMotion(2, wait_for_end=True)

    robot.setGeneralControlModule("action_module")
    robot.playMotion(96, wait_for_end=True)
    time.sleep(1)
