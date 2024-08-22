#! /usr/bin/python

import rospy
import numpy as np
import cv2

from op3_ros_utils import getWalkingParams, Robot
from vision_old import *
from detect_marker import *
from copy import copy
import sys
import time

DEBUG_MODE = False # Show the detected image
DEBUG_MARKER = False
DL_MODE = False

MIN_AREA = 5000
SLOWDOWN_AREA = 40000
CROSS_AREA = 45000

class States:
    INIT = -1
    READY = 0 # Waits for start button
    WALK_FORWARD = 1 # Moves the head, looking for the ball
    WALK_SLOWLY_FORWARD = 2
    WALK_PREBACKWARDS = 3
    WALK_BACKWARDS = 4
    MARKER_TURN_LEFT = 5
    MARKER_TURN_RIGHT = 6
    MARKER_FORWARD = 7
    END = 99

class Label:
    FORWARD = 1
    LEFT = 2
    RIGHT = 3
    
marker = [999, States.MARKER_FORWARD, States.MARKER_TURN_LEFT, States.MARKER_TURN_RIGHT]

# Functions to be passed to vision system
# func1 = detectSingleColor
func1 = detect3Color

args1 = np.array(( (np.array([85, 68, 0]), np.array([179, 255, 255])),  #blue hsv
                    (np.array([0, 0, 14]), np.array([179, 146, 255])), #green
                    (np.array([0, 148, 68]), np.array([165, 255, 255])),)) #red lab
# Create vision system
vision = VisionSystem(pipeline_funcs=[func1],
                      pipeline_args=[args1], debug=DEBUG_MODE, verbose=0)


if DL_MODE:
    ssd = Detectnet(debug=DEBUG_MARKER, verbose=0)
    rospy.Subscriber("/detectnet/detections", Detection2DArray, ssd.read)
else:
    rospy.Subscriber("/cv_camera/image_raw", Image, vision.read, queue_size=1)
    rospy.Subscriber("/video_source/raw", Image, vision.read)

# Subscribe to cv_camera topic with vision system


# Iinitialize Node
rospy.init_node("fira_sprint")

# Create robot
robot = Robot()

rospy.sleep(3) # Make sure every publisher has registered to their topic,
               # avoiding lost messages
def init():
    # Set ctrl modules of all actions to joint, so we can reset robot position 
    # robot.setGeneralControlModule("action_module")

    # robot.setGrippersPos(left=0.0, right=0.0)

    # Call initial robot position
    # robot.playMotion(1, wait_for_end=True)

    # Set ctrl module to walking, this actually only sets the legs
    robot.setGeneralControlModule("walking_module")
    
    # Set joint modules of head joints to none so we can control them directly
    robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])

    robot.setJointPos(["head_tilt"], [-1])
    robot.setJointPos(["head_pan"], [0])

    rospy.sleep(1.0)