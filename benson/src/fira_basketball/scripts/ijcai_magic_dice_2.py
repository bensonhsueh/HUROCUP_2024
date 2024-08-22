#! /usr/bin/python

import rospy
import numpy as np
import cv2

from op3_utils import Robot_weightlift
#from vision_weightlifting import *
from std_msgs.msg import Int32, String, Float32, Float64, Bool, Float32MultiArray
from op3_ros_utils import getWalkingParams, Robot
from vision_old import *
from copy import copy
import sys

DEBUG_MODE = False # Show the detected image
MIN_AREA = 300
CROSS_AREA = 1165
DEGREE2RADIAN = np.pi / 180

class States:
    INIT = -1
    READY = 0 # Waits for start button
    SHAKE_DICES = 1
    CHANGE_CARDS = 2
    PUSH_DICE = 3
    END = 99

# Functions to be passed to vision system
func1 = detectSingleColor
args1 = ((np.array([98, 132, 123]), np.array([104, 255, 255])),)

# Create vision system
vision = VisionSystem(pipeline_funcs=[func1],
                      pipeline_args=[args1], debug=DEBUG_MODE, verbose=1)

# Subscribe to cv_camera topic with vision system
rospy.Subscriber("/cv_camera/image_raw", Image, vision.read, queue_size=1)
# Iinitialize Node
rospy.init_node("fira_sprint")

# Create robot
robot = Robot()
robot_weightlift = Robot_weightlift('fira_basketball')

rospy.sleep(3) # Make sure every publisher has registered to their topic,
               # avoiding lost messages

def init():
    # Set ctrl modules of all actions to joint, so we can reset robot position
    robot.setGeneralControlModule("action_module")
    online_walk_balance_pub = rospy.Publisher('/robotis/online_walking/wholebody_balance_msg', String, queue_size=1)
    # Set ctrl module to walking, this actually only sets the legs
    #robot.setGeneralControlModule("walking_module")

    # Call initial robot position
    robot.playMotion(15, wait_for_end=True)
    # robot_weightlift.walk_set_param_pub.publish(robot_weightlift.walking_params[0])
    
    # Set joint modules of head joints to none so we can control them directly
    robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])

    robot.setJointPos(["head_tilt"], [-0.3])

    rospy.sleep(1.0)

def center_head_on_object(obj_pos):
    cx, cy = 0.5, 0.5 # Center of image
    obj_x, obj_y = obj_pos

    dist_x = obj_x - cx
    dist_y = obj_y - cy

    head_curr_x = robot.joint_pos["head_pan"]
    head_curr_y = robot.joint_pos["head_tilt"]

    kp = 0.5
    new_head_x = head_curr_x + kp * -dist_x
    new_head_y = head_curr_y + kp * -dist_y

    robot.setJointPos(["head_tilt", "head_pan"], [new_head_y, new_head_x])

tickrate = 60
rate = rospy.Rate(tickrate)

# TODO remember to put to 0
STEP_LEVEL = 0

currState = States.INIT
while not rospy.is_shutdown():
    print(vision.status[0])
    
    if robot.buttonCheck("mode"):
        STEP_LEVEL = 0
        currState = States.INIT
    
    if DEBUG_MODE:
        if vision.status[0]:
            cv2.imshow("Frame", vision.debug_img[0])
            cv2.waitKey(1)

    if vision.status[0]:
        pos, obj_area = vision.results[0]
        print("Area: {}".format(obj_area))
        if obj_area > MIN_AREA:
            center_head_on_object(pos)

    if currState == States.INIT:
        print("[INIT]")
        init()
        tick_count = 0
        direction = False
        currState = States.READY

    elif currState == States.READY:
        print("[READY]")
        if robot.buttonCheck("start"):
            robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])
            robot.setJointPos(["head_pan", "head_tilt"], [0, -0.5])
            tick_count = 0

            currState = States.SHAKE_DICES    
 
    elif currState == States.SHAKE_DICES:
        rospy.loginfo("[SHAKE_DICES]")
        robot_weightlift.setGeneralControlModule("none")
        rospy.sleep(2)
        robot_weightlift.setGeneralControlModule("action_module")
        robot_weightlift.playMotion(236, wait_for_end=True)
        rospy.sleep(3.0)
        robot_weightlift.playMotion(240, wait_for_end=True)
        rospy.sleep(1.0)
        robot_weightlift.playMotion(241, wait_for_end=True)
        rospy.sleep(3.0)
        
        robot_weightlift.setGeneralControlModule("none")
        rospy.sleep(2)
        robot.setGeneralControlModule("online_walking_module")
        robot.onlineWalkSetup(x=-0.003, y=0.0, foot_dist=0.09, foot_height=0.05)
        #online_walk_balance_pub = rospy.Publisher('/robotis/online_walking/wholebody_balance_msg', String, queue_size=1)
        #online_walk_balance_pub.publish("balance_on")
        #rospy.sleep(2.0)
        robot.onlineWalkCommand(direction="turn_right", start_leg="right", step_num=10,
                 step_angle=0.1, step_time=0.4)
        
        currState = States.CHANGE_CARDS
        rospy.sleep(10.0)
    
    elif currState == States.CHANGE_CARDS:
        rospy.loginfo("[CHANGE_CARDS]")
        robot_weightlift.setGeneralControlModule("none")
        rospy.sleep(2)
        robot_weightlift.setGeneralControlModule("action_module")
        robot_weightlift.playMotion(237, wait_for_end=True)
        rospy.sleep(3.0)
        
        robot_weightlift.setGeneralControlModule("none")
        rospy.sleep(2)
        robot.setGeneralControlModule("online_walking_module")
        robot.onlineWalkSetup(x=-0.003, y=0.0, foot_dist=0.09, foot_height=0.05)
        robot.onlineWalkCommand(direction="turn_right", start_leg="right", step_num=10,
                 step_angle=0.1, step_time=0.4)
        
        currState = States.PUSH_DICE
    
    elif currState == States.PUSH_DICE:
        rospy.loginfo("[PUSH_DICE]")
        robot_weightlift.setGeneralControlModule("none")
        rospy.sleep(2)
        robot_weightlift.setGeneralControlModule("action_module")
        robot_weightlift.playMotion(238, wait_for_end=True)
        rospy.sleep(3.0)
        
        robot_weightlift.setGeneralControlModule("none")
        rospy.sleep(2)
        robot.setGeneralControlModule("online_walking_module")
        robot.onlineWalkSetup(x=-0.003, y=0.0, foot_dist=0.09, foot_height=0.05)
        robot.onlineWalkCommand(direction="turn_right", start_leg="right", step_num=10,
                 step_angle=0.1, step_time=0.4)
        
        currState = States.END
    
    elif currState == States.END:
        print("[END]")
    rate.sleep()