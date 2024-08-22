#! /usr/bin/python

import rospy
import numpy as np
import cv2

from op3_utils import Robot_weightlift
#from vision_weightlifting import *

from op3_ros_utils import getWalkingParams, Robot
from vision_old import *
from copy import copy
import sys
import sound


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
    FLOWER_APPEAR = 4
    POKE_DICE = 5
    END = 99

# Functions to be passed to vision system
func1 = detectSingleColor
func2 = detect2Color
func3 = detectSingleColor
func4 = detectSingleColor
args1 = ((np.array([95, 125, 120]), np.array([104, 255, 255])),)#blue
args2 = ((np.array([0, 130, 150]), np.array([4, 255, 255])),
         (np.array([160, 130, 70]), np.array([180, 200, 255])))#red
args3 = ((np.array([20, 55, 123]), np.array([28, 200, 255])),)#yellow
args4 = ((np.array([120, 25, 123]), np.array([127, 100, 255])),)#purple

# Create vision system
vision = VisionSystem(pipeline_funcs=[func1, func2, func3, func4],
                      pipeline_args=[args1, args2, args3, args4], debug=DEBUG_MODE, verbose=1)

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
        sound.speak('Hi everyone. My name is honey lemonade.')
        rospy.sleep(1.0)
        robot_weightlift.playMotion(229, wait_for_end=True)
        rospy.sleep(2.0)
        sound.speak(' And this is my assistant.')
        rospy.sleep(1.0)
        robot_weightlift.playMotion(228, wait_for_end=True)
        rospy.sleep(1.0)
        sound.speak('Today I will be introducing you the magical robot world which is similar to yours, umm .. I mean the humans world')
        rospy.sleep(1.0)
        sound.speak('As you might know already, things are consisted of small molecules.')
        rospy.sleep(1.0)
        sound.speak('Give me your attention.')
        robot_weightlift.playMotion(230, wait_for_end=True)
        rospy.sleep(5.0)
        robot_weightlift.playMotion(240, wait_for_end=True)
        rospy.sleep(1.0)
        robot_weightlift.playMotion(241, wait_for_end=True)
        rospy.sleep(3.0)
        sound.speak('See, heres how it goes.')
        
        currState = States.CHANGE_CARDS
    
    elif currState == States.CHANGE_CARDS:
        rospy.loginfo("[CHANGE_CARDS]")
        robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])
        robot.setJointPos(["head_pan", "head_tilt"], [0, -0.7])
        robot_weightlift.setGeneralControlModule("none")
        rospy.sleep(2)
        robot_weightlift.setGeneralControlModule("action_module")
        sound.speak('However, the world is not as simple as you can see. You cant judge a book by its cover, things might be out of your expectation.')
        robot_weightlift.playMotion(237, wait_for_end=True)
        rospy.sleep(0.5)
        sound.speak('Its red')
        robot_weightlift.playMotion(231, wait_for_end=True)
        rospy.sleep(0.5)
        sound.speak('Its black')
        robot_weightlift.playMotion(234, wait_for_end=True)
        rospy.sleep(0.5)
        sound.speak('Its red')
        robot_weightlift.playMotion(231, wait_for_end=True)
        rospy.sleep(0.5)
        sound.speak('Its magic!')
        robot_weightlift.playMotion(235, wait_for_end=True)
        rospy.sleep(3.0)
        sound.speak('WOw')
        rospy.sleep(0.5)
        sound.speak('I am good at this.')
        rospy.sleep(1.0)
        
        currState = States.PUSH_DICE
    
    elif currState == States.PUSH_DICE:
        rospy.loginfo("[PUSH_DICE]")
        robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])
        robot.setJointPos(["head_pan", "head_tilt"], [0, -0.5])
        robot_weightlift.setGeneralControlModule("none")
        rospy.sleep(2)
        robot_weightlift.setGeneralControlModule("action_module")
        sound.speak('As we have said, things are not that easy. There is also time that we run into problems.')
        rospy.sleep(0.1)
        sound.speak('Luckily, we can overcome the problem in multiple ways. Heres an example. Hey assistant, show them my magic card.')
        rospy.sleep(10.0)
        sound.speak('As you can see theres no way I can get the cube go through the card. But wait a second?')
        rospy.sleep(1.0)
        robot_weightlift.playMotion(238, wait_for_end=True)
        rospy.sleep(3.0)
        sound.speak('Dala, I did it. ')
        robot_weightlift.playMotion(227, wait_for_end=True)
        robot_weightlift.playMotion(226, wait_for_end=True)
        rospy.sleep(3.0)
        sound.speak('Everything is gonna work out.')
        
        currState = States.FLOWER_APPEAR
    
    elif currState == States.FLOWER_APPEAR:
        rospy.loginfo("[FLOWER_APPEAR]")
        robot_weightlift.setGeneralControlModule("none")
        rospy.sleep(2)
        robot_weightlift.setGeneralControlModule("action_module")
        sound.speak('Also, the world is constantly giving us a lot of surprises.')
        robot_weightlift.playMotion(239, wait_for_end=True)
        rospy.sleep(1.0)
        robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])
        robot.setJointPos(["head_pan", "head_tilt"], [-0.6, -0.7])
        sound.speak('See, heres a normal stick and a plan plastic container.')
        robot_weightlift.setGeneralControlModule("none")
        rospy.sleep(2.0)
        robot_weightlift.setGeneralControlModule("action_module")
        rospy.sleep(1.0)
        robot_weightlift.playMotion(233, wait_for_end=True)
        rospy.sleep(0.5)
        sound.speak('But, wowwwwww. A flower came out from there.')
        rospy.sleep(3.0)
        
        currState = States.POKE_DICE
    
    elif currState == States.POKE_DICE:
        rospy.loginfo("[POKE_DICE]")
        sound.speak('Okay, I think thats the end of todays story.')
        rospy.sleep(0.5)
        robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])
        robot.setJointPos(["head_pan", "head_tilt"], [-0.55, -0.6])
        rospy.sleep(0.5)
        robot_weightlift.setGeneralControlModule("none")
        rospy.sleep(2)
        robot_weightlift.setGeneralControlModule("action_module")
        rospy.sleep(0.5)
        sound.speak('Oh wait, theres another surprise from my assistant. Can I have the coin?')
        rospy.sleep(1.0)
        robot_weightlift.playMotion(232, wait_for_end=True)
        rospy.sleep(0.5)
        sound.speak('Ooooooops. I think I am in trouble. Bye guys. I got to go.')
        robot_weightlift.playMotion(236, wait_for_end=True)
        rospy.sleep(3.0)
        
        currState = States.END

    elif currState == States.END:
        print("[END]")
    rate.sleep()