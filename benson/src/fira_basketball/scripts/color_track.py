#! /usr/bin/python

import rospy
import numpy as np
import cv2

from op3_utils import Robot_weightlift

from op3_ros_utils import getWalkingParams, Robot
from vision import *
from copy import copy
import sys

DEBUG_MODE = True # Show the detected image
MIN_AREA = 300
CROSS_AREA = 1165
DEGREE2RADIAN = np.pi / 180
bgr_lower = np.array([15, 140, 140])   # 轉換成 NumPy 陣列，範圍稍微變小 ( 55->30, 70->40, 252->200 )
bgr_upper = np.array([80, 200, 200])

class States:
    INIT = -1
    READY = 0 # Waits for start button
    WALK_FORWARD = 1 # Moves the head, looking for the ball
    
    END = 99

func1 = detectSingleColor

args1 = ((np.array([98, 132, 123]), np.array([104, 255, 255])),)
lower = np.array([30,40,200])
upper = np.array([90,100,255])

vision = VisionSystem(pipeline_funcs=[func1],
                      pipeline_args=[args1], debug=DEBUG_MODE, verbose=1)

rospy.Subscriber("/cv_camera/image_raw", Image, vision.read, queue_size=1)

rospy.init_node("walk_op3")


robot = Robot()
robot_weightlift = Robot_weightlift('fira_basketball')

rospy.sleep(3)

def init():
    robot.setGeneralControlModule("action_module")

    robot.playMotion(1, wait_for_end=True)
    robot_weightlift.walk_set_param_pub.publish(robot_weightlift.walking_params[0])

    robot.setGeneralControlModule("walking_module")
    
    robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])

    robot.setJointPos(["head_tilt"], [-0.3])

    rospy.sleep(1.0)


    
    

def center_head_on_object(obj_pos):
    cx, cy = 0.5, 0.5 
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
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("Cannot open camera")
            exit()
       
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
            robot.walkStart()
            currState = States.WALK_FORWARD
        
    elif currState == States.WALK_FORWARD:
        print("[WALK_FORWARD]")

        ret, frame = cap.read()
        image = cv2.resize(frame,(640,360))
        output = cv2.inRange(image, bgr_lower, bgr_upper)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11))
        output = cv2.dilate(output, kernel)
        output = cv2.erode(output, kernel)
        contours, hierarchy = cv2.findContours(output, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            color = (0, 0, 255)  # 红色
            if(area > 300):
                x, y, w, h = cv2.boundingRect(contour)  
                center_x = int(x + w / 2) 
                center_y = int(y + h / 2) 
                image = cv2.rectangle(image, (x, y), (x + w, y + h), color, 3) 
                cv2.circle(image, (center_x, center_x), 5, color, -1)
        image_center_x, image_center_y = 320, 180  # 图像尺寸为640x360的中心坐标
        cv2.circle(image, (image_center_x, image_center_y), 10, (0, 255, 0), -1)
        cv2.imshow('Current view', image)
        if image_center_x != center_x or image_center_y != center_y :
            dist_x = (center_x - image_center_x)/640
            dist_y = (center_y - image_center_y)/360
            head_curr_x = robot.joint_pos["head_pan"]
            head_curr_y = robot.joint_pos["head_tilt"]
            kp = 0.5
            new_head_x = head_curr_x + kp * -dist_x
            new_head_y = head_curr_y + kp * -dist_y

            robot.setJointPos(["head_tilt", "head_pan"], [new_head_y, new_head_x])
    #     currState = States.END
 

    # elif currState == States.END:
    #     print("[END]")

        
    # rate.sleep()

