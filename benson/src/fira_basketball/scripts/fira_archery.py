#! /usr/bin/python

import rospy
import numpy as np
import cv2

from op3_ros_utils import getWalkingParams, Robot
from vision_old import *
from copy import copy
import sys

DEBUG_MODE = True # Show the detected image
MIN_AREA = 1000
SLOWDOWN_AREA = 60000
CROSS_AREA = 65000

########
#INIT
#state 1: find target, then define the height of it, 0 represents low target, 1 represents middle target, 2 represent high target
#state 2: change to walking module, then face to target 
#state 3: play action according to the height we get from state 1

class States:
    INIT = -1
    READY = 0 # Waits for start button
    FIND_TARGET = 1
    FACE_TO_TARGET = 2
    SHOOT = 3
    #WALK_FORWARD = 1 # Moves the head, looking for the ball
    #WALK_SLOWLY_FORWARD = 2
    #WALK_PREBACKWARDS = 3
    #WALK_BACKWARDS = 4
    #WALK_BACKWARDS_2 = 5
    END = 99

# Functions to be passed to vision system
func1 = detectSingleColor
func2 = detect2Color
#args1 = ((np.array([25, 120, 80]), np.array([255, 0, 0])),)
#args1 = ((np.array([20, 80, 150]), np.array([50, 130, 230])),)#green, yellow
args1 = ((np.array([0, 120, 45]), np.array([10, 255, 255])),
         (np.array([170, 120, 45]), np.array([180, 255, 255])))

# Create vision system
vision = VisionSystem(pipeline_funcs=[func2],
                      pipeline_args=[args1], debug=DEBUG_MODE, verbose=1)

# Subscribe to cv_camera topic with vision system
rospy.Subscriber("/cv_camera/image_raw", Image, vision.read, queue_size=1)
# rospy.Subscriber("/cv_camera/image_raw", Image, iimg, queue_size=1)
# Iinitialize Node
rospy.init_node("fira_sprint")

# Create robot
robot = Robot()

rospy.sleep(3) # Make sure every publisher has registered to their topic,
               # avoiding lost messages

def init():
    # Set ctrl modules of all actions to joint, so we can reset robot position 
    robot.setGeneralControlModule("action_module")

    # robot.setGrippersPos(left=0.0, right=0.0)
    #robot.playMotion(9, wait_for_end=True)#stand up

    #rospy.sleep(1)
    robot.playMotion(229, wait_for_end=True)#archery_set, put bow and arrow
    # Call initial robot position
    #robot.playMotion(230, wait_for_end=True)#play set-up motion of archery 

    # Set ctrl module to walking, this actually only sets the legs
    robot.setGeneralControlModule("online_walking_module")
    
    
    # Set joint modules of head joints to none so we can control them directly
    robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])

    robot.setJointPos(["head_tilt"], [-0.2])

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
    
    kd = 0.001
    dt = 1.0/float(50)
    deriv_error_x = dist_x/dt
    deriv_error_y = dist_y/dt

    pand = kd * deriv_error_x
    tiltd = kd * deriv_error_y
    new_head_x = new_head_x + pand
    new_head_y = new_head_y + tiltd
    
    print(new_head_y, new_head_x)
    robot.setJointPos(["head_tilt", "head_pan"], [new_head_y, new_head_x])

tickrate = 60
rate = rospy.Rate(tickrate)

# TODO remember to put to 0
STEP_LEVEL = 0

currState = States.INIT

count = 0
while not rospy.is_shutdown():
    print(vision.status[0])
    if robot.buttonCheck("mode"):
        STEP_LEVEL = 0
        currState = States.INIT
    
    if DEBUG_MODE:
        if vision.status[0]:
            
            cv2.imshow("Frame", vision.debug_img[0])
            cv2.waitKey(1)
            cv2.imwrite('input_imaghe.jpg', vision.img_buffer[-1])

    if vision.status[0]:
      
        pos, obj_area = vision.results[0]
        # print(pos)
        print("Area: {}".format(obj_area))
        if obj_area > MIN_AREA:
            center_head_on_object(pos)

    if currState == States.INIT:
        print("[INIT]")
        init()

        # Transition
        tick_count = 0

        direction = False
        currState = States.READY
    
    elif currState == States.READY:
        print("[READY]")
        if robot.buttonCheck("start"):
            robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])
            robot.setJointPos(["head_pan", "head_tilt"], [0, -0.2])
            tick_count = 0
            
            currState = FIND_TARGET
     
    elif currState == FIND_TARGET:

    elif currState == States.WALK_FORWARD:
        print("[WALK_FORWARD]")
        pan_angle = robot.joint_pos["head_pan"]
        print(pan_angle)
        if pan_angle > 0.1:
            theta = 5
        elif pan_angle < -0.1:
            theta = -5
        else:
            theta = 0

        robot.walkVelocities(x=32, th=theta, balance=True, hip_pitch=6)

        tick_count += 1

        if obj_area > SLOWDOWN_AREA:
            currState = States.WALK_SLOWLY_FORWARD

    elif currState == States.WALK_SLOWLY_FORWARD:
        print("[WALK_SLOWLY_FORWARD]")
        pan_angle = robot.joint_pos["head_pan"]
        if pan_angle > 0.025:
            theta = -0.7
            yamp = 0.3
        elif pan_angle < -0.03:
            theta = 0.5
            yamp = -0.3
        else:
            theta = 0
            yamp = 0
        robot.walkVelocities(x=21, y=yamp, balance=True, hip_pitch=5)
        
        if obj_area > CROSS_AREA:
            robot.walkStop()
            rospy.sleep(0.5)
            robot.walkStart()
            currState = States.WALK_PREBACKWARDS

    elif currState == States.WALK_PREBACKWARDS:
        print("[WALK_PREBACKWARDS]")
        pan_angle = robot.joint_pos["head_pan"]
        print('pangle', pan_angle)
        theta
        if pan_angle > 0.025:
            #theta = -0.7
            yamp = pan_angle * 20
        elif pan_angle < -0.025:
            #theta = 0.5
            yamp = pan_angle * 20
        else:
            theta = 0
            yamp = 0
        
        #yamp = pan_angle * 20
        robot.walkVelocities(x=-13, y=yamp, balance=True, hip_pitch=4)
        rospy.sleep(3)

        currState = States.WALK_BACKWARDS
    
    elif currState == States.WALK_BACKWARDS:
        print("[WALK_BACKWARDS]")
        pan_angle = robot.joint_pos["head_pan"]
        #print(pan_angle)
        if pan_angle > 0.025:
            #theta = -0.7
            yamp = pan_angle * 10
        elif pan_angle < -0.025:
            #theta = 0.5
            yamp = pan_angle * 10
            if count % 20 == 0:
                print('right')
        else:
            #theta = 0
            yamp = 0

        if count % 20 == 0:
            print('pan_angle', pan_angle)
            print('yamp', yamp)

        robot.walkVelocities(x=-26, y=yamp, balance=True, hip_pitch=4)
        rospy.sleep(7)
        currState = States.WALK_BACKWARDS_2
    
    elif currState == States.WALK_BACKWARDS_2:
        print("[WALK_BACKWARDS_2]")
        pan_angle = robot.joint_pos["head_pan"]
        #print(pan_angle)
        if pan_angle > 0.02:
            #theta = -0.7
            yamp = pan_angle * 15
        elif pan_angle < -0.02:
            #theta = 0.5
            yamp = pan_angle * 15
            if count % 20 == 0:
                print('right')
        else:
            #theta = 0
            yamp = 0

        if count % 20 == 0:
            print('pan_angle', pan_angle)
            print('yamp', yamp)

        robot.walkVelocities(x=-26, y=yamp, balance=True, hip_pitch=4)

    elif currState == States.END:
        print("[END]")
        robot.walkStop()


    count = count + 1
    rate.sleep()
