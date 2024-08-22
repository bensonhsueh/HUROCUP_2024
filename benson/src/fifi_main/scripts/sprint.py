#!/usr/bin/env python

import yaml
import rospy
import rospkg
import numpy as np
import time
from op3_utils import Robot

import cv2
import rosnode

class States:
    INIT = -1
    READY = 0 # Waits for start button

#Initialize Node
rospy.init_node("fira_sprint")

#Create robot
robot = Robot("op3_walking_module")

rospy.sleep(2)

def init():

    robot.setGeneralControlModule("action_module")
    robot.playMotion(1,wait_for_end=True)
    rospy.sleep(1)

currState = States.INIT
while not rospy.is_shutdown():
    # print(robot.pressed_button)
    
    if robot.pressed_button == "mode":
       rospy.loginfo("now is the mode")
       init()
       robot.pressed_button = None
    '''
    print(img.shape[0])
    print(img.shape[1])
    print(img.shape[2])
    '''
    # if robot.buttonCallBack("mode"):
    #    rospy.loginfo("now is the mode")
    # if currState == States.INIT:
    time.sleep(0.2)

     