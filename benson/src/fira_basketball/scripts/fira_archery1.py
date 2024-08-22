#! /usr/bin/python

import rospy
import numpy as np
import cv2

from op3_ros_utils import getWalkingParams, Robot
from vision_old import *
from copy import copy
import sys
from std_msgs.msg import Int32, String, Float32, Float64, Bool, Float32MultiArray
# Forgive me Sasuke
from pygame import mixer

DEBUG_MODE = True # Show the detected image

MIN_AREA = 100 # Minimum area of objects to consider for vision
BALL_PICKUP_SIZE = 9000
BASKET_DUNK_SIZE = 52000
SIDE_STEP_TIME = 3.5
HEAD_SEARCH_SPEED = 0.065

angle_threshold = 0.02

class States:
    INIT = -1
    READY = 0 # Waits for start button
    FIND_TARGET = 1
    FACE_TARGET = 2
    SHOOT = 3
    END = 99

# Functions to be passed to vision system
#func1 = detectSingleColor
func2 = detect2Color
#args1 = ((np.array([13, 90, 150]), np.array([25, 255, 255])),)# hsv value for ping pong
args2 = ((np.array([0, 130, 70]), np.array([10, 255, 255])),
         (np.array([170, 130, 70]), np.array([180, 255, 255])))# hsv value for basket

# Create vision system
vision = VisionSystem(pipeline_funcs=[func2],
                      pipeline_args=[args2], debug=DEBUG_MODE, verbose=0)

# Subscribe to cv_camera topic with vision system
rospy.Subscriber("/cv_camera/image_raw", Image, vision.read, queue_size=1)
l_grip_pub = rospy.Publisher('/grippers/left_pos', Float32, queue_size = 1)
r_grip_pub = rospy.Publisher('/grippers/right_pos', Float32, queue_size = 1)

#####grippers topic######
#/grippers/both_pos
#/grippers/left_pos
#/grippers/right_pos
#/grippers/torque
########################
#Float32MultiArray
#rospy.Subscriber("/grippers/left_pos", Float32, _gripper_left, queue_size = 1)
#rospy.Subscriber("/grippers/right_pos", Float32, _gripper_right, queue_size = 1)
#self.online_walk_command_pub = rospy.Publisher(
#            '/robotis/online_walking/foot_step_command', FootStepCommand, queue_size=1)

# Create robot
robot = Robot()


# Iinitialize Node
rospy.init_node("fira_archery_2021")

online_walk_balance_pub = rospy.Publisher('/robotis/online_walking/wholebody_balance_msg', String, queue_size=1)
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
    
def init():
    # Set ctrl modules of all actions to joint, so we can reset robot position
    
    robot.setGeneralControlModule("none")
    rospy.sleep(0.5)
    robot.setGeneralControlModule("action_module")

    '''robot.setGrippersPos(left=100.0, right=0.0)'''
    # Call initial robot position
    
    robot.playMotion(229, wait_for_end=True)#archery_set, put bow and arrow
    # Call initial robot position
    #robot.playMotion(230, wait_for_end=True)#play set-up motion of archery 

    # Set ctrl module to walking, this actually only sets the legs
    #robot.setGeneralControlModule("online_walking_module")
    
    robot.setGeneralControlModule('online_walking_module')
    rospy.sleep(0.5)
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

    robot.setJointPos(["head_tilt", "head_pan"], [new_head_y, new_head_x])

tickrate = 60
rate = rospy.Rate(tickrate)

first_look_direction = ''

currState = States.INIT
while not rospy.is_shutdown():
    if robot.buttonCheck("mode"):
        currState = States.INIT

    if DEBUG_MODE:
        cv2.imshow("Image", vision.img_buffer[-1])
        #cv2.imshow("Func1", vision.debug_img[0])
        cv2.imshow("Func2", vision.debug_img[0])
        if vision.status[0]:#the first function - detect single color
            print("Area 0: {}".format(vision.results[0][1]))#vision.results[0][0] is the center, vision.result[0][1] is the area.
        #if vision.status[1]:#the second function - detect 2 color
        #    print("Area 1: {}".format(vision.results[1][1]))
        cv2.waitKey(1)

    if currState == States.INIT:
        print("[INIT]")
        init()

        # Transition
        tick_count = 0
        direction = False
        #currState = States.FOCUS_BASKET
        currState = States.READY

    elif currState == States.READY:
        print("[READY]")
        if robot.buttonCheck("start"):

            finished_cycle = False
            currState = States.FIND_TARGET

    elif currState == States.FIND_TARGET:
        status = vision.status[0]
        height = 'low'
        if status == True and vision.results[0][1] > MIN_AREA:
            center, area = vision.results[0]
            print('center coordinate: {}, target_area:{}'.format(center, area))
            if center[1] > 0.5:
                height = 'low'
            elif center[1] < 0.3:
                height = 'high'
            else:
                center = 'middle'
    
            print('target is {}'.format(height))   
            currState = States.FACE_TARGET
            #robot.setGeneralControlModule('none')
            #rospy.sleep(0.5)
            #robot.onlineWalkSetup(x=0.0, y=0.0, z=0.0, foot_dist=0.070, foot_height=0.05, dsp_ratio=0.20, zmp_offset_x=0.0, zmp_offset_y=0.0)
            robot.onlineWalkSetup(x=0.015, z=-0.03, foot_dist=0.09, foot_height=0.04)
            rospy.sleep(1)
            
    
    elif currState == States.FACE_TARGET:
        print('FACE_TARGET')
        center, area = vision.results[0]
        center_head_on_object(center)
        pan = robot.joint_pos["head_pan"]#initial pan == 1.385
        print('pan differnece:', pan - 1.385)
        
        if pan - 1.385 > angle_threshold:#target is at left side
            robot.onlineWalkCommand(direction="turn_left", start_leg="right", step_num=2, front_length=-0.1, step_angle= 0.1, step_time=0.5)

        elif pan - 1.385 < -angle_threshold:#target is at right side
            robot.onlineWalkCommand(direction="turn_right", start_leg="right", step_num=2, front_length=-0.1, step_angle= 0.1, step_time=0.5)

        else:#target is in front of the robot
            currState = States.SHOOT
        
        #robot.onlineWalkSetup(x=0.0, y=0.0, z=0.0, foot_dist=0.070, foot_height=0.05, dsp_ratio=0.20, zmp_offset_x=0.0, zmp_offset_y=0.0)
        ##rospy.sleep(0.5)
        #robot.onlineWalkCommand(direction="turn_left", start_leg="right", step_num=2,
        #                front_length=-0.1, step_angle= 0.2, step_time=0.5)
        #robot.onlineWalkCommand(direction = 'Forward', start_leg = 'left', step_num = 2, front_length=0.0, side_length=0.0, step_angle=0.0, step_time=0.5)
        #robot.onlineWalkCommand(direction="left", start_leg="left", step_num=2,
        #             side_length=0.02, step_time=0.4)
        rospy.sleep(5)
        #if pan > 0.05:
        #    th = 4.0
        #elif pan < -0.05:
        #    th = -4.0
        #else:
        #    th = 0.0
        #robot.walkVelocities(x=-5.0, th=th)

        #if pan > -0.2 and pan < 0.2:
        #    tick_count += 1

        #    if tick_count > tickrate*1.5:
                # Transition
        #        tick_count = 0

        #        currState = States.WALK_FORWARD
        #
    
    elif currState == States.SHOOT:
        center, area = vision.results[0]
        pan = robot.joint_pos["head_pan"]
        print('center of target is {}, it is {}, and difference of the pan angle is {}'.format(center, height, pan - 1.385)) 
        robot.setGeneralControlModule("action_module")

        #robot.setGrippersPos(left=100.0, right=0.0)
        # Call initial robot position
        if height == 'low':
            robot.playMotion(233, wait_for_end=True)
        if height == 'middle':
            robot.playMotion(233, wait_for_end=True)
        if height == 'high':
            robot.playMotion(233, wait_for_end=True)
        currState = States.END

    elif currState == States.FIND_BALL:
        print("[FIND_BALL]")
        # Move head to find the ball
        head_curr_x = robot.joint_pos["head_pan"]
        if direction == False:
            new_x = head_curr_x + HEAD_SEARCH_SPEED
        else:
            new_x = head_curr_x - HEAD_SEARCH_SPEED

        if new_x > 1.0:
            direction = True
        elif new_x < -1.0:
            direction = False
            finished_cycle = True

        robot.setJointPos(["head_pan", "head_tilt"], [new_x, -0.6])


        # Retrieve results of first function
        status = vision.status[0]
        if finished_cycle:
            tick_count = 0
            currState = States.WALK_BACKWARDS
        elif status == True and vision.results[0][1] > MIN_AREA:
            tick_count += 1
        else:
            tick_count = 0
            currState = States.FIND_BALL

        if tick_count > tickrate // 3:
            tick_count = 0

            currState = States.FOCUS_BALL

    elif currState == States.WALK_BACKWARDS:
            print("[WALK_BACKWARDS]")
            robot.walkStart()
            robot.walkVelocities(x=-6.5, th=0)
            rospy.sleep(3)
            robot.walkStop()
            direction = False
            finished_cycle = False
            # Go back to looking for the ball
            currState = States.FIND_BALL

    elif currState == States.FOCUS_BALL:
        print("[FOCUS_BALL]")
        # Retrieve results of first function
        status = vision.status[0]
        if status == True and vision.results[0][1] > MIN_AREA:
            tick_count += 1
            center, area = vision.results[0]
            center_head_on_object(center)
            print("[A]")
        else:
            tick_count = 0
            currState = States.FIND_BALL
            print("[B]")

        if tick_count > tickrate:
            # Transition
            tick_count = 0
            if robot.joint_pos["head_pan"] > -0.2 and robot.joint_pos["head_pan"] < 0.2:
                if area > 4400:
                    print("[C]")
                    robot.walkStart()
                    currState = States.WALK_FORWARD
                    if robot.joint_pos["head_pan"] > 0.0:
                        first_look_direction = 'left'
                    elif robot.joint_pos["head_pan"] <= 0.0:
                        first_look_direction = 'right'
                else:
                    if robot.joint_pos["head_pan"] > 0.0:
                        first_look_direction = 'left'
                    elif robot.joint_pos["head_pan"] <= 0.0:
                        first_look_direction = 'right'

                    robot.walkStart()
                    print("[D]")
                    currState = States.WALK_FORWARD
            else:
                robot.walkStart()
                print("[E]")
                currState = States.FACE_BALL

    elif currState == States.FACE_BALL:
        print("[FACE_BALL]")
        center, area = vision.results[0]
        center_head_on_object(center)
        pan = robot.joint_pos["head_pan"]

        if pan > 0.05:
            th = 4.0
        elif pan < -0.05:
            th = -4.0
        else:
            th = 0.0
        robot.walkVelocities(x=-5.0, th=th)

        if pan > -0.2 and pan < 0.2:
            tick_count += 1

            if tick_count > tickrate*1.5:
                # Transition
                tick_count = 0

                currState = States.WALK_FORWARD

    elif currState == States.WALK_FORWARD:
        print("[WALK_FORWARD]")

        center, area = vision.results[0]
        center_head_on_object(center)
        pan = robot.joint_pos["head_pan"]
        tilt_angle = robot.joint_pos["head_tilt"]
        #print("Obj X: {}\t Area: {}\t Pan: {}".format(center[0], area, pan))

        if pan > 0.05:
            th = 4.0
            yamp = -1.0
        elif pan < -0.05:
            th = -3.0
            yamp = 0
        else:
            th = 0.0
            yamp = 1.0
        robot.walkVelocities(x=3.0, y=yamp, th=th, hip_pitch=8.0)

        if tilt_angle <= -0.89:
            tick_count += 1

            if tick_count > tickrate//2:
                # Transition
                tick_count = 0

                robot.walkStop()

                #robot.onlineWalkSetup(x=0.02, z=-0.045, foot_dist=0.08, foot_height=0.03)

                # TODO: testing making head centered
                robot.setJointPos(["head_pan"], [0.0])
                currState = States.PICK_BALL


    # elif currState == States.WALK_SIDEWAYS:
    #     print("[WALK_SIDEWAYS]")

    #     center, area = vision.results[0]
    #     detected = vision.status[0]

    #     print("X position of object in image: {} -- Y: {} -- Status: {}".format(center[0], center[1], detected))

    #     if center[0] > 0.29:
    #         tick_count = 0
    #         robot.onlineWalkCommand(direction="right", start_leg="right", step_num=2,
    #             side_length=0.04, step_time=0.4)
    #         #rospy.sleep(SIDE_STEP_TIME)
    #     elif center[0] < 0.205:
    #         tick_count = 0
    #         robot.onlineWalkCommand(direction="left", start_leg="left", step_num=2,
    #             side_length=0.02, step_time=0.4)
    #         #rospy.sleep(SIDE_STEP_TIME)
    #     else:
    #         tick_count += 1

    #         if tick_count > tickrate:
    #             # Transition
    #             currState = States.PICK_BALL

    elif currState == States.PICK_BALL:
        print("[PICK_BALL]")
        #set_gripper('left', 5)
        #set_gripper('right', 5)
        center, area = vision.results[0]

        # I think this will stop the robot from "jerking" when changing the module
        #robot.online_walk_balance_pub.publish("balance_off")
        # TODO testing jump thingie
        #rospy.sleep(3)
        robot.setGeneralControlModule("none")
        rospy.sleep(1)
        robot.setGeneralControlModule("action_module")
        
        robot.playMotion(255, wait_for_end=True)
        set_gripper('left', 85.0)
        #l_grip_pub.publish(80.0)
        rospy.sleep(1)
        robot.playMotion(254, wait_for_end=True)
        # robot.playMotion(72)
        robot.setGrippersPos(left=15.0)
        rospy.sleep(2)
        #robot.playMotion(71, wait_for_end=True)

        # Transition
        # Set ctrl module to walking, this actually only sets the legs
        robot.setJointsControlModule(["r_hip_yaw","l_hip_yaw","r_hip_roll","l_hip_roll","r_hip_pitch",
        "l_hip_pitch","r_knee","l_knee","r_ank_pitch","l_ank_pitch","r_ank_roll","l_ank_roll"],
        ["walking_module"])
        # Set joint modules of head joints to none so we can control them directly
        robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])
        
        robot.walkStart()
        #if first_look_direction == 'right':
        #    robot.walkVelocities(x=-10, y=5, th=0, hip_pitch=8)
        #elif first_look_direction == 'left':
        #    robot.walkVelocities(x=-10, y=-5, th=0, hip_pitch=8)
        #else:
        robot.walkVelocities(x=-10, y=-5, th=0, hip_pitch=8)

        rospy.sleep(10.0)# walk backward
        robot.walkStop()
        if first_look_direction == 'right':
            direction = True 
        elif first_look_direction == 'left':
            direction = False 
        else:
            print("Should never get here...")
        currState = States.FIND_BASKET

    elif currState == States.FIND_BASKET:
        print("[FIND_BASKET]")
        # Move head to find the ball
        head_curr_x = robot.joint_pos["head_pan"]
        if direction == False:
            new_x = head_curr_x + HEAD_SEARCH_SPEED
        else:
            new_x = head_curr_x - HEAD_SEARCH_SPEED

        if new_x > 1.0:
            direction = True
        elif new_x < -1.0:
            direction = False

        robot.setJointPos(["head_pan", "head_tilt"], [new_x, 0.0])

        # Retrieve results of first function
        status = vision.status[1]
        if status == True and vision.results[1][1] > MIN_AREA:
            tick_count += 1
        else:
            tick_count = 0

        if tick_count > tickrate // 3:
            tick_count = 0

            currState = States.FOCUS_BASKET

    elif currState == States.FOCUS_BASKET:
        print("[FOCUS_BASKET]")
        # Retrieve results of first function
        status = vision.status[1]
        if status == True and vision.results[1][1] > MIN_AREA:
            tick_count += 1
            center, area = vision.results[1]
            center_head_on_object(center)
        else:
            tick_count = 0
            currState = States.FIND_BASKET
        print(area)

        print(tick_count)

        if tick_count > tickrate:
            # Transition
            tick_count = 0
            if robot.joint_pos["head_pan"] > -0.2 and robot.joint_pos["head_pan"] < 0.2:
                robot.walkStart()
                currState = States.WALK_TO_BASKET
            else:
                robot.walkStart()
                currState = States.FACE_BASKET

    elif currState == States.FACE_BASKET:
        print("[FACE_BASKET]")
        center, area = vision.results[1]
        center_head_on_object(center)
        pan = robot.joint_pos["head_pan"]
        if pan > 0.05:
            th = 5.0
        elif pan < -0.05:
            th = -4.0
        else:
            th = 0.0
        robot.walkVelocities(x=-2.0, th=th, hip_pitch=8)
        robot.walkStop()
        rospy.sleep(2)
        robot.walkStart()
        if pan > -0.2 and pan < 0.2:
            tick_count += 1
            if tick_count > tickrate:
                # Transition
                tick_count = 0
        currState = States.WALK_TO_BASKET
        
    elif currState == States.WALK_TO_BASKET:
        print("[WALK_TO_BASKET]")

        center, area = vision.results[1]
        center_head_on_object(center)
        pan = robot.joint_pos["head_pan"]

        print("Obj X: {}\t Area: {}\t Pan: {}".format(center[0], area, pan))

        if pan > 0.05:
            th = 4.0
        elif pan < -0.05:
            th = -4.0
        else:
            th = 0.0
        robot.walkVelocities(x=5.5, th=th, hip_pitch=8)

        if area > BASKET_DUNK_SIZE:
            tick_count += 1

            if tick_count > tickrate//2:
                # Transition
                tick_count = 0

                robot.walkStop()

                # TODO: testing making head centered
                robot.setJointPos(["head_pan"], [0.0])
                robot.setGeneralControlModule("action_module") 
                
                set_gripper('right', 5.0)
                robot.playMotion(240, wait_for_end = True)
                set_gripper('left', 65)
                
                robot.playMotion(241, wait_for_end = True)
                set_gripper('left', 5.0)
                robot.playMotion(242, wait_for_end = True)
                
                #robot.onlineWalkSetup(x=0.02, z=-0.045, foot_dist=0.08, foot_height=0.03)

                currState = States.WALK_SIDEWAYS_BASKET

    elif currState == States.WALK_SIDEWAYS_BASKET:
        print("[WALK_SIDEWAYS]")
        #mixer.music.stop()

        center, area = vision.results[1]
        detected = vision.status[1]
        rospy.sleep(0.5)
        for i in range (0, 12):
            robot.onlineWalkCommand(direction="left", start_leg="left", step_num=2,
                    side_length=0.03, step_time=0.4)
            rospy.sleep(2.5)


        print("X position of object in image: {} -- Y: {} -- Status: {}".format(center[0], center[1], detected))

        # if center[0] > 0.14:
        #     robot.onlineWalkCommand(direction="right", start_leg="right", step_num=2,
        #         side_length=0.04, step_time=0.4)
        #     rospy.sleep(2.5)
        # elif center[0] < 0.08:
        #     robot.onlineWalkCommand(direction="left", start_leg="left", step_num=2,
        #         side_length=0.02, step_time=0.4)
        #     rospy.sleep(2.5)
        # else:
        #     tick_count += 1

        #     if tick_count > tickrate:
        #         # Transition
        #         currState = States.DUNK
        '''
         robot.onlineWalkCommand(direction="right", start_leg="right", step_num=2,
                  side_length=0.04, step_time=0.4)
         rospy.sleep(2.5)
         robot.onlineWalkCommand(direction="right", start_leg="right", step_num=2,
                  side_length=0.04, step_time=0.4)
         rospy.sleep(2.5)
         robot.onlineWalkCommand(direction="right", start_leg="right", step_num=2,
                  side_length=0.04, step_time=0.4)
         rospy.sleep(2.5)
         robot.onlineWalkCommand(direction="right", start_leg="right", step_num=2,
                  side_length=0.04, step_time=0.4)
         rospy.sleep(2.5)
        '''
        currState = States.DUNK
    
    elif currState == States.DUNK:
        # I think this will stop the robot from "jerking" when changing the module
        #robot.online_walk_balance_pub.publish("balance_off")
        # TODO testing if robot doesnt jump anymore
        rospy.sleep(1)
        robot.setGeneralControlModule("none")
        rospy.sleep(1)
        robot.setGeneralControlModule("action_module")
        robot.playMotion(251)
        rospy.sleep(5.25)
        # mixer.music.load('/home/robotis/sound/slam_verse.mp3')
        # mixer.music.play()
        robot.setGrippersPos(left=50.0)
        # rospy.sleep(1)
        # robot.setJointsControlModule(["r_hip_yaw","l_hip_yaw","r_hip_roll","l_hip_roll","r_hip_pitch",
        # "l_hip_pitch","r_knee","l_knee","r_ank_pitch","l_ank_pitch","r_ank_roll","l_ank_roll"],
        # ["walking_module"])
        # robot.walkVelocities(x=-5, hip_pitch=3)
        # robot.walkStart()
        # rospy.sleep(2)
        # robot.walkStop()
        # rospy.sleep(3)
        currState = States.END
    
    elif currState == States.END:
        #print("[END]")
        pass
        # robot.walkStop()

    rate.sleep()
