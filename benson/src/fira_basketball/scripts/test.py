#!/usr/bin/env python

import rospy
from op3_utils import *
#from vision_weightlifting import *
import cv2
import sys
import rosnode




# Iinitialize Node
rospy.init_node('fira_weightlifting')


# Create robot ('package_name')
robot = Robot('fira_basketball')


while not rospy.is_shutdown():
    if '/op3_manager' in rosnode.get_node_names():
        rospy.loginfo('Found op3 manager')
        break
    else:
        rospy.loginfo('Waiting for op3 manager')
    rospy.Rate(20).sleep()



# Make sure every publisher has registered to their topic,
# avoiding lost messages
rospy.sleep(4) 

DEGREE2RADIAN = np.pi / 180

def init():
    # Set ctrl modules of all actions to joint, so we can reset robot position
    robot.setGeneralControlModule("action_module")
    
    #robot.moveGripper(left=100.0,right=100.0)
    #robot.setGrippersPos(left=0.0, right=0.0)
    # >0 is opened
    
    # Call initial robot position
    robot.playMotion(1, wait_for_end=True)

    # Set ctrl module to walking, this actually only sets the legs
    robot.walk_set_param_pub.publish(robot.walking_params[0])
    robot.setGeneralControlModule("walking_module")
    
    # Set joint modules of head joints to none so we can control them directly
    robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])
    
    
    robot.setJointPos(["head_tilt"], [-0.7])
    #0 is looking straight forward, <0 is looking down

    rospy.sleep(1.0)

tickrate = 30
rate = rospy.Rate(tickrate)

# currState = States.INIT

cap = cv2.VideoCapture(0)

while not rospy.is_shutdown():
    
    
    
    ret, frame = cap.read()

    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    frame = cv2.resize(frame, (0,0),fx=0.5,fy=0.5, interpolation=cv2.INTER_CUBIC)
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    cv2.imshow('Current view',hsv_frame)
    cv2.waitKey(33)

