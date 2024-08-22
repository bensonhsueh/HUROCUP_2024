# ! /usr/bin/python

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
    robot.setGeneralControlModule("walking_module") #設成走路模式
    
    # Set joint modules of head joints to none so we can control them directly
    robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])

    robot.setJointPos(["head_tilt"], [-1]) #低頭
    robot.setJointPos(["head_pan"], [0]) #頭轉正


    rospy.sleep(1.0)
    print("init OK")

def center_head_on_object(obj_pos):
    cx, cy = 0.5, 0.5 # Center of image
    obj_x, obj_y = obj_pos

    dist_x = obj_x - cx
    dist_y = obj_y - cy

    head_curr_x = robot.joint_pos["head_pan"]
    #head_curr_y = robot.joint_pos["head_tilt"]

    kp = 0.5
    new_head_x = head_curr_x + kp * -dist_x
    #new_head_y = head_curr_y + kp * -dist_y
    
    new_head_x = np.clip(new_head_x, np.radians(-45), np.radians(45))
    print("head", new_head_x)
    robot.setJointPos(["head_pan"], [new_head_x])
    
# def scan_next_marker():
    

tickrate = 60
rate = rospy.Rate(tickrate)#設定間隔的頻率，也就是每秒需要發送幾次訊息(60HZ)，->rate.sleep()

# TODO remember to put to 0
STEP_LEVEL = 0
pos = (0.5, 0.5)
currState = States.INIT
robot.joint_pos["head_pan"] = 0

while not rospy.is_shutdown():#-----------------------------------------------------------------------------------------------------
    # print(vision.status[0])
    if robot.buttonCheck("mode"):
        print("ydtfghjiodtcbhmkchgbj")
        # Trueug_img[0])
        if vision.status[0]:
           cv2.imshow("result", vision.results[0])
        cv2.waitKey(1)
        
    #get max_score
    # max_obj = max(ssd.detect, key=lambda obj: obj.score)
    # max_id = max_obj.id
    # print(max_id, max_obj.center, max_obj.score, max_obj.size)
    # if max_obj.score > 0.99 and max_obj.center[1] > 150:
    #     currState = marker[max_id]
    
    # for obj in ssd.detect:
    #     obj.score = 0
    
    if vision.status[0]:
        pos, obj_area = vision.results[0]
        print("Area: {}".format(obj_area))
        if obj_area > MIN_AREA and currState not in marker :
            center_head_on_object(pos)

    if currState == States.INIT:
        print("[INIT]")
        init()
        
        # Transition
        tick_count = 0
        direction = False
        currState = States.READY


    elif currState == States.READY:
        # print("[READY]")
        
        if robot.buttonCheck("start"):
            # robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])
            # robot.setJointPos(["head_pan", "head_tilt"], [0, -1])
            tick_count = 0
            robot.walkStart()
            currState = States.WALK_FORWARD
            while 1:
                # robot.walkVelocities(self, x=0.0, y=0.0, th=0, z_move_amplitude=None, balance=None,feet_pitch=None, z_offset=None, hip_pitch=None) #36
                robot.walkVelocities( x=20.0, y=0.0, th=0, z_move_amplitude=None, balance=True,feet_pitch=None, z_offset=None, hip_pitch=None) #36
                # robot.walkVelocities(x=s,th=theta, balance=True, hip_pitch=27)
        
    elif currState == States.WALK_FORWARD:
        print("[WALK_FORWARD]")
        pan_angle = robot.joint_pos["head_pan"]
        print(pan_angle)
        # center_head_on_object(pos)
        if pan_angle > 0.1:
        #robot.setJointPos(["head_tilt"], [-0.8])
            theta = 5
            s=0
        elif pan_angle < -0.1:
        #robot.setJointPos(["head_tilt"], [-0.8])
            theta = -5
            s=0
        else:
        #robot.setJointPos(["head_tilt"], [-0.8])
            theta = 0
            s=12
        
        robot.walkVelocities(x=s,y=0,th=theta, balance=True, hip_pitch=27) #36
        

        tick_count += 1

    # elif currState == States.WALK_SLOWLY_FORWARD:
    #     print("[WALK_SLOWLY_FORWARD]")
    #     pan_angle = robot.joint_pos["head_pan"]
    #     if pan_angle > 0.025:
    #         theta = -0.7
    #         yamp = 0.3
    #     elif pan_angle < -0.03:
    #         theta = 0.5
    #         yamp = -0.3
    #     else:
    #         theta = 0
    #         yamp = 0
    #     robot.walkVelocities(x=20, y=yamp, balance=True) #23
        
    elif currState == States.MARKER_TURN_RIGHT:
        print("[MARKER_RIGHT_FORWARD]")
        robot.walkVelocities(x=0, y=0, th=-10, balance=True)
        time.sleep(5)
        currState = States.MARKER_FORWARD
        
    elif currState == States.MARKER_TURN_LEFT:
        print("[MARKER_LEFT_FORWARD]")
        robot.walkVelocities(x=0, y=0, th=10, balance=True)
        
        
    elif currState == States.MARKER_FORWARD:
        print("[MARKER_FORWARD]")
        if robot.joint_pos["head_pan"] != 0:
            robot.setJointPos(["head_pan"], [0])
        if robot.joint_pos["head_tilt"] != -1:
            robot.setJointPos(["head_tilt"], [-1])
        robot.walkVelocities(x=10, y=0, th=0, balance=True)
    
    rate.sleep()
