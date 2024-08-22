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

DEBUG_MODE = False  # Show the detected image

MIN_AREA = 100 # Minimum area of objects to consider for vision
BALL_PICKUP_SIZE = 9000
BASKET_DUNK_SIZE = 58000
SIDE_STEP_TIME = 3.5
HEAD_SEARCH_SPEED = 0.065

class States:

    INIT = -1
    READY = 0 # Waits for start button
    RAISE_RIGHT_HAND = 50
    READ = 100
    TEST = 110
    FIND_BAG = 1
    FOCUS_BAG = 2
    PICK_BAG = 3
    FACE_BAG = 4
    TALK_USR = 5
    INPUT_USR =6
    WAIT_BAG = 7 #Waiting for bag delivery
    REMOVE_BAG = 8 #Waiting for bag takeoff


# Functions to be passed to vision system
'''func1 = detectSingleColor
func2 = detect2Color
args1 = ((np.array([168, 60, 0]), np.array([180, 255, 255])),)# hsv value for ping pong
#0,130,0
args2 = ((np.array([0, 120, 0]), np.array([12, 255, 255])),
         (np.array([168, 60, 0]), np.array([180, 255, 255])))# hsv value for basket

# # Create vision system
vision = VisionSystem(pipeline_funcs=[func1, func2],
                      pipeline_args=[args1, args2], debug=DEBUG_MODE, verbose=0)'''

# # Subscribe to cv_camera topic with vision system
#rospy.Subscriber("/cv_camera/image_raw", Image, vision.read, queue_size=1)
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
    robot.playMotion(5, wait_for_end=True)
    #robot.playMotion(8, wait_for_end=True)
    # Set ctrl module to walking, this actually only sets the legs
    robot.setGeneralControlModule("walking_module")
    
    # Set joint modules of head joints to none so we can control them directly
    #robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])

    #robot.setJointPos(["head_tilt"], [-0.7])

    time.sleep(2)
    

'''def center_head_on_object(obj_pos):
    cx, cy = 0.5, 0.5 # Center of image
    obj_x, obj_y = obj_pos

    dist_x = obj_x - cx
    dist_y = obj_y - cy

    head_curr_x = robot.joint_pos["head_pan"]
    head_curr_y = robot.joint_pos["head_tilt"]

    kp = 0.5
    new_head_x = head_curr_x + kp * -dist_x
    new_head_y = head_curr_y + kp * -dist_y

    robot.setJointPos(["head_tilt", "head_pan"], [new_head_y, new_head_x])'''

effort_values = []
test = []

if __name__ == "__main__":
    
    tickrate = 60
    rate = rospy.Rate(tickrate)
    x_values = []
    y_values_effort = []
    index = count()
    pre_joint_effort = 0 
    global_weight = 0
    first_look_direction = ''
    
    currState = States.INIT
    while not rospy.is_shutdown():
      #for i in range(2):

        robot.setJointPos(["r_sho_pitch"], [1.57])
        rate.sleep()

        if robot.buttonCheck("mode"):
            currState = States.INIT

        '''if DEBUG_MODE:
            cv2.imshow("Image", vision.img_buffer[-1])
            cv2.imshow("Func1", vision.debug_img[0])
            #if vision.status[0]:
                #print("Area 0: {}".format(vision.results[0][1]))
            cv2.waitKey(1)'''

        if currState == States.INIT:
            rospy.loginfo("[INIT]")
            init()
            ##print("Current Version of Python is: ", sys.version)

            # Transition
            tick_count = 0
            direction = False
            
            finished_cycle = False
            time.sleep(2)
            #robot.setJointPos(["head_pan", "head_tilt"], [0.0, 0.75])
            #key = raw_input("Press Enter to continue")
            currState = States.RAISE_RIGHT_HAND

 
        if currState == States.RAISE_RIGHT_HAND:
            #robot.setGeneralControlModule("action_module")
            #TODO: Failsafe for too heavy objects

            if len(effort_values)==0:
                print("attach the bag")
                robot.playMotion(6, wait_for_end=True)
                time.sleep(5)



            elif len(effort_values) < 2:
                print("bag is attached")
                time.sleep(2)
                print("Ready to weigh")
                time.sleep(3)

                for i in range(10):
                    r_sho_pitch = robot.joint_effort['r_sho_pitch']
                    y_values_effort.append(r_sho_pitch)
 

                sum_effort = sum(y_values_effort)/len(y_values_effort)
                sum_effort=round(sum_effort,2)
                print(sum_effort)

                if (sum_effort == 2.81):
                    print("the weight is 50 grams")


                elif sum_effort >4 :
                    print("The weight is 150 grams")


                elif sum_effort >5:
                    print("The weight is 300 grams")


                elif sum_effort >6:
                    print("The weight is 400 grams")

                else:
                    print("I am confused torque measured is ",sum_effort)
      
                
                print("Remove the bag")
                print("Thankyou for weighing")
       


            





  
