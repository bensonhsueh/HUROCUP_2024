#! /usr/bin/python

import rospy
import numpy as np
import cv2
from op3_ros_utils import getWalkingParams, Robot
from vision_old import *
from copy import copy
import sys
from std_msgs.msg import Int32, String, Float32, Float64, Bool, Float32MultiArray
from sensor_msgs.msg import JointState
import time
from pygame import mixer
from itertools import count
import os


DEBUG_MODE = False # Show the detected image

MIN_AREA = 100 # Minimum area of objects to consider for vision

class States:

    INIT = -1
    READY = 0 # Waits for start button
    RAISE_RIGHT_HAND = 50
    READ = 100
    TEST = 110
    TALK_USR = 5
    INPUT_USR =6
    WAIT_BAG = 7 #Waiting for bag delivery
    REMOVE_BAG = 8 #Waiting for bag takeoff


# Functions to be passed to vision system
func1 = detectSingleColor
func2 = detect2Color
args1 = ((np.array([168, 60, 0]), np.array([180, 255, 255])),)# hsv value for ping pong
#0,130,0
args2 = ((np.array([0, 120, 0]), np.array([12, 255, 255])),
         (np.array([168, 60, 0]), np.array([180, 255, 255])))# hsv value for basket

# # Create vision system
vision = VisionSystem(pipeline_funcs=[func1, func2],
                      pipeline_args=[args1, args2], debug=DEBUG_MODE, verbose=0)

# # Subscribe to cv_camera topic with vision system
rospy.Subscriber("/cv_camera/image_raw", Image, vision.read, queue_size=1)
l_grip_pub = rospy.Publisher('/grippers/left_pos', Float32, queue_size = 1)
r_grip_pub = rospy.Publisher('/grippers/right_pos', Float32, queue_size = 1)

# Create robot
robot = Robot()


# Iinitialize Node
rospy.init_node("weight_magic_trick")

rospy.sleep(3) # Make sure every publisher has registered to their topic,
               # avoiding lost messages

def set_gripper(gripper = 'left', value = 0):
    if gripper == 'left':
        for i in range(4):
            l_grip_pub.publish(value)
            rospy.sleep(0.1)
    elif gripper == 'right':
        for i in range(4):
            r_grip_pub.publish(value)
            rospy.sleep(0.1)
    else:
        rospy.info('wrong name assigned to grippers')
    rospy.sleep(0.5)


def init():
    # Set ctrl modules of all actions to joint, so we can reset robot position
    robot.setGeneralControlModule("action_module")

    '''robot.setGrippersPos(left=100.0, right=0.0)'''
    set_gripper('left', 5)
    set_gripper('right', 5)
    # Call initial robot position
    robot.playMotion(1, wait_for_end=True)

    # Set ctrl module to walking, this actually only sets the legs
    robot.setGeneralControlModule("walking_module")
    
    # Set joint modules of head joints to none so we can control them directly
    robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])

    robot.setJointPos(["head_tilt"], [-0.7])

    time.sleep(2)
    

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

effort_values = []
test = []

if __name__ == "__main__":
    
    tickrate = 60
    rate = rospy.Rate(tickrate)
    x_values = []
    y_values_effort = []
    index = count()
    pre_effort = 0 
    global_weight = 0
    first_look_direction = ''
    
    currState = States.INIT
    while not rospy.is_shutdown():

        rate.sleep()

        if robot.buttonCheck("mode"):
            currState = States.INIT

        # if robot.buttonCheck("user"):
        #     currState = States.PICK_BAG

        if DEBUG_MODE:
            cv2.imshow("Image", vision.img_buffer[-1])
            cv2.imshow("Func1", vision.debug_img[0])
            cv2.imshow("Func2", vision.debug_img[1])
            if vision.status[0]:#the first function - detect single color
                print("Area 0: {}".format(vision.results[0][1]))#vision.results[0][0] is the center, vision.result[0][1] is the area.
            if vision.status[1]:#the second function - detect 2 color
                print("Area 1: {}".format(vision.results[1][1]))
            cv2.waitKey(1)

        if currState == States.INIT:
            rospy.loginfo("[INIT]")
            init()
            print("Current Version of Python is: ", sys.version)

            # Transition
            tick_count = 0
            direction = False
            
            finished_cycle = False
            time.sleep(2)
            robot.setJointPos(["head_pan", "head_tilt"], [0.0, 0.75])
            key = raw_input("Press Enter to continue")
            currState = States.RAISE_RIGHT_HAND

        if currState == States.RAISE_RIGHT_HAND:
            print("[RAISE RIGHT HAND]")

            robot.setGeneralControlModule("action_module")
            # robot.playMotion(74, wait_for_end=True)
            # set_gripper('right', 10 )

            r_sho_pitch = robot.joint_effort['r_sho_pitch']
            if(r_sho_pitch<=1):
                currState==States.WAIT_BAG
            else:
                effort_values.append(r_sho_pitch)

            #TODO: Failsafe for too heavy objects

            if len(effort_values) <= 10:
                avg_r_sho_pitch = np.abs(sum(effort_values) / len(effort_values))
                # print("HIHI")
                print("READY")


            elif len(effort_values) > 10:
                sum_effort = 0

                for i in range(3):
                    current_effort = robot.joint_effort['r_sho_pitch'] - avg_r_sho_pitch
                    # set_gripper('right', 58)

                    # print("HEOIHSE")

                    time.sleep(0.1)

                    robot.playMotion(16, wait_for_end=True)
                    r_sho_pitch = robot.joint_effort['r_sho_pitch']
                    x_values.append(0)
                    y_values_effort.append(np.abs(r_sho_pitch - avg_r_sho_pitch))
                    # print(effort_values)
                    time.sleep(0.1)

                    robot.playMotion(17, wait_for_end=True)
                    r_sho_pitch = robot.joint_effort['r_sho_pitch']
                    x_values.append(1)
                    y_values_effort.append(np.abs(r_sho_pitch - avg_r_sho_pitch))
                    # print(effort_values)
                    time.sleep(0.1)

                    robot.playMotion(18, wait_for_end=True)
                    r_sho_pitch = robot.joint_effort['r_sho_pitch']
                    x_values.append(2)
                    y_values_effort.append(np.abs(r_sho_pitch - avg_r_sho_pitch))
                    # print(effort_values)
                    time.sleep(0.1)

                    robot.playMotion(19, wait_for_end=True)
                    r_sho_pitch = robot.joint_effort['r_sho_pitch']
                    x_values.append(3)
                    y_values_effort.append(np.abs(r_sho_pitch - avg_r_sho_pitch))
                    # print(effort_values)
                    time.sleep(0.1)

                    robot.playMotion(20, wait_for_end=True)
                    r_sho_pitch = robot.joint_effort['r_sho_pitch']
                    x_values.append(4)
                    y_values_effort.append(np.abs(r_sho_pitch - avg_r_sho_pitch))
                    # print(effort_values)
                    time.sleep(3)

                    # set_gripper('right', 10)

                    print(sum(y_values_effort))

                    sum_effort = sum_effort + sum(y_values_effort)
                    
                    y_values_effort[:] = []

                sum_effort = sum_effort / 3
                print(sum_effort)

                if 1.2 <= sum_effort <= 1.7:
                    print("The weight is 50 grams")

                elif sum_effort <= 1.42:
                    global_weight = 100
                    print("The weight is 100 grams")

                elif sum_effort <= 2.2:
                    global_weight = 200
                    print("The weight is 200 grams")

                elif sum_effort <= 3.2:
                    global_weight = 300
                    print("The weight is 300 grams")

                elif sum_effort <= 4.2:
                    global_weight = 400
                    print("The weight is 400 grams")

                elif sum_effort <= 5.2:
                    global_weight = 500
                    print("The weight is 500 grams")

                elif sum_effort <= 6.2 :
                    global_weight = 600
                    print("The weigth is 600 grams")

                else:
                    global_weight = 700
                    print("The weight is 700 grams or more")

                robot.playMotion(16, wait_for_end=True)
            currState = States.REMOVE_BAG


        if currState == States.WAIT_BAG:
            print("Waiting for Bag")
            r_sho_pitch = robot.joint_effort['r_sho_pitch']
            
            robot.setJointPos(["head_pan", "head_tilt"], [0.0, 0.75])
            rospy.sleep(1)

            s = 0
            # arithmetic average 
            for i in range(30):
                s = s + robot.joint_effort['r_sho_pitch']
                rospy.sleep(0.02)
            print (s/30)
            if s/30 > 0.49:
                currState = States.RAISE_RIGHT_HAND
            rospy.sleep(1)    #do not start immediately!
            #TODO make robot look at the bag briefly
            # robot.setJointPos(["head_pan", "head_tilt"], [0.5, -0.5])

            

        if currState == States.REMOVE_BAG:
            robot.setJointPos(["head_pan", "head_tilt"], [0.0, 0.75])
            print("Please remove Bag")
            rospy.sleep(3)
            
            s = 0
            # arithmetic average 
            for i in range(30):
                s = s + robot.joint_effort['r_sho_pitch']
                rospy.sleep(0.02)
            print (s/30)
            print("Please refill the bag and show it to me")
            
