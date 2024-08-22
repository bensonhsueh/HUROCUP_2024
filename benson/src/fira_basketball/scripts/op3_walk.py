#! /usr/bin/python

import rospy
import numpy as np
import cv2

from op3_utils import Robot_weightlift
#from vision_weightlifting import *

from op3_ros_utils import getWalkingParams, Robot
from vision import *
from copy import copy
import sys

DEBUG_MODE = True # Show the detected image
MIN_AREA = 300
CROSS_AREA = 1165
DEGREE2RADIAN = np.pi / 180

class States:
    INIT = -1
    READY = 0 # Waits for start button
    WALK_FORWARD = 1 # Moves the head, looking for the ball
    
    END = 99

# Functions to be passed to vision system
func1 = detectSingleColor
##func1 = detect2Color
#args1 = ((np.array([25, 120, 80]), np.array([255, 0, 0])),)
args1 = ((np.array([98, 132, 123]), np.array([104, 255, 255])),)#green, yellow
#args1 = ((np.array([0, 120, 45]), np.array([10, 255, 255])),
#         (np.array([170, 120, 45]), np.array([180, 255, 255])))

# Create vision system
vision = VisionSystem(pipeline_funcs=[func1],
                      pipeline_args=[args1], debug=DEBUG_MODE, verbose=1)

# Subscribe to cv_camera topic with vision system
rospy.Subscriber("/cv_camera/image_raw", Image, vision.read, queue_size=1)
# rospy.Subscriber("/cv_camera/image_raw", Image, queue_size=1)
# Iinitialize Node
rospy.init_node("walk_op3")

# Create robot
robot = Robot()
robot_weightlift = Robot_weightlift('fira_basketball')

rospy.sleep(3) # Make sure every publisher has registered to their topic,
               # avoiding lost messages

def init():
    # Set ctrl modules of all actions to joint, so we can reset robot position
    robot.setGeneralControlModule("action_module")

    # robot.setGrippersPos(left=0.0, right=0.0)

    # Call initial robot position
    robot.playMotion(1, wait_for_end=True)
    robot_weightlift.walk_set_param_pub.publish(robot_weightlift.walking_params[0])

    # Set ctrl module to walking, this actually only sets the legs
    robot.setGeneralControlModule("walking_module")
    
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

        # Transition
        tick_count = 0
        direction = False
        currState = States.READY

    elif currState == States.READY:
        print("[READY]")
        if robot.buttonCheck("start"):
            robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])
            robot.setJointPos(["head_pan", "head_tilt"], [0, -0.5])
            tick_count = 0
            robot.walkStart()
            currState = States.WALK_FORWARD
        
    elif currState == States.WALK_FORWARD:
        print("[WALK_FORWARD]")
        tilt_angle = robot.joint_pos["head_tilt"]
        pan_angle = robot.joint_pos["head_pan"]
        print(pan_angle)
        print(tilt_angle)
        if pan_angle > 0.1:
            theta = pan_angle*50
        elif pan_angle < -0.1:
            theta = pan_angle*50
        else:
            theta = 0

        # robot_weightlift.walking_params.append(robot_weightlift.loadWalkingParams('pickup_param_0.yaml'))
        
        # robot_weightlift.walk_set_param_pub.publish(robot_weightlift.walking_params[1])
        # # Set ctrl module to walking, this actually only sets the legs
        # #print(robot_weightlift.walking_params[2])
        # rospy.sleep(1)
        # robot_weightlift.walkStart()
        # #rospy.sleep(14)
        # #robot_weightlift.moveGripper(left=15.0,right=15.0) 
        

        robot.walkVelocities(x=1, th=theta, balance=True, hip_pitch=7)

        tick_count += 1
        #rospy.sleep(8)
        #currState = States.PICK_BAR
        # TODO change this
        # if obj_area > CROSS_AREA:
        #      currState = States.PICK_BAR
        print(tilt_angle)
        if tick_count == 100:
            robot.walkStop()
            # robot_weightlift.onlineWalkCommand(direction="turn_left", start_leg="right", step_num=1,
            #         front_length=0.4, step_angle=15,step_time=0.5)
        
            currState = States.END
 

    elif currState == States.END:
        print("[END]")
        #robot_weightlift.walkStop()

        
    rate.sleep()

