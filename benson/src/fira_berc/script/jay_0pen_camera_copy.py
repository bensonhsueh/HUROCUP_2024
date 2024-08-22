# import rospy
# import cv2
# from vision import *


# # Forgive me Sasuke
# #from pygame import mixer

# DEBUG_MODE = True # Show the detected image
# MIN_AREA = 50 # Minimum area of objects to consider for vision
# BALL_PICKUP_SIZE = 9000
# BASKET_DUNK_SIZE = 62050
# SIDE_STEP_TIME = 3.5
# HEAD_SEARCH_SPEED = 0.065

# # Functions to be passed to vision system
# func1 = detectSingleColor
# func3 = detect2Color
# func2 = detectSingleColor

# args1 = ((np.array([13, 145, 150]), np.array([25, 255, 255])),) #ball (orange)
# args3 = ((np.array([0, 130, 0]), np.array([12, 255, 255])),
#          (np.array([168, 60, 0]), np.array([180, 255, 255]))) #basket(red)
# args2 = ((np.array([98, 100, 100]), np.array([104, 255, 255])),) #hand(blue)
# #args3 = ((np.array([111, 100, 100]), np.array([153, 255, 255])),) #hand(purple)


# # Create vision system
# #vision = VisionSystem(pipeline_funcs=[func1, func2, func3],
# #                      pipeline_args=[args1, args2, args3], debug=DEBUG_MODE, verbose=0)
# vision = VisionSystem(pipeline_funcs=[func1, func2, func3],
#                      pipeline_args=[args1, args2, args3], debug=DEBUG_MODE, verbose=0)

# # Subscribe to cv_camera topic with vision system
# rospy.Subscriber("/cv_camera/image_raw", Image, vision.read, queue_size=1)



# # Iinitialize Node
# rospy.init_node("fira_basketball")

# rospy.sleep(3) # Make sure every publisher has registered to their topic,
#                # avoiding lost messages

# tickrate = 60
# rate = rospy.Rate(tickrate)

# first_look_direction = ''

# while not rospy.is_shutdown():
#     cv2.imshow("Image", vision.img_buffer[-1])
#     cv2.imshow("Func1", vision.debug_img[0])
#     cv2.imshow("Func2", vision.debug_img[1])
#     cv2.imshow("Func3", vision.debug_img[2])
#     # cv2.imshow("Func4", vision.debug_img[3])
#     if vision.status[0]:
#         print("Area 0: {}".format(vision.results[0][1]))
#     if vision.status[1]:
#         print("Area 1: {}".format(vision.results[1][1]))
#     if vision.status[2]:
#         print("Area 2: {}".format(vision.results[2][1]))
#     #if vision.status[3]:
#     #    print("Area 3: {}".format(vision.results[3][1]))
#     cv2.waitKey(1)


#     rate.sleep()

#! /usr/bin/env python

import rospy
from op3_ros_utils import *
import cv2
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision import *
import time
import numpy
# from PIL import  Image 


DEBUG_MODE = True # Show the detected image
DEBUG_MARKER = False
DL_MODE = False 


# Functions to be passed to vision system
# func1 = detectSingleColor
func1 = follow_line_benson

args1 = ((np.array([0, 50, 60]), np.array([40, 255, 255])),)#ball (orange)




vision = VisionSystem(pipeline_funcs=[func1],pipeline_args=[args1], debug=DEBUG_MODE, verbose=0)
rospy.Subscriber("/cv_camera/image_raw", Image, vision.read, queue_size=1)
# rospy.Subscriber("/video_source/raw", Image, vision.read)
rospy.init_node("benson_marathon")

robot = Robot()




rospy.sleep(4) # Make sure every publisher has registered to their topic,               # avoiding lost messages

# robot.setGeneralControlModule("action_module")
# robot.playMotion(1, wait_for_end=True)
def init():

    # Call initial robot position
    # robot.playMotion(9, wait_for_end=True)
    robot.setGeneralControlModule("walking_module")
    robot.setGeneralControlModule("head_control_module")
    robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])
    robot.setJointPos(["head_tilt","head_pan"], [np.radians(-50),np.radians(45)])


def caculate_theta_and_moving():
    global last_P
    global time_for_Pd
    aim_angle = 45
    if vision.status[0]:
        Pa = vision.results[0][0] - aim_angle # -0.5~0.5
        Dy = (Pa - last_P)/(time.time()-time_for_Pd)
        # print(time.time()-time_for_Pd)
        time_for_Pd = time.time()
        # Px = (vision.results[0][0][1])
        kp = 40
        kd = 1
        # print(Py*150,Dy*kd,Py*150 +  Dy*kd)
        theta = kp*Pa + kd*Dy
        last_P = Pa

        print(vision.results[0][1])
    else :
        theta = last_P

    return theta

#  global last_P
#     global time_for_Pd
#     pan_angle_old = np.degrees(robot.joint_pos["head_pan"])
#     pan_angle = 0
#     if vision.status[0]:
#         Py = (-1)*(vision.results[0][0][0]-0.5) # -0.5~0.5
#         Dy = (Py - last_P)/(time.time()-time_for_Pd)
#         time_for_Pd = time.time()
#         # Px = (vision.results[0][0][1])
#         kp = 80
#         kd = 0
#         pan_angle = pan_angle_old + Py*kp +  Dy*kd
#         # pan_angle = np.clip(pan_angle, -95, 95)
#         # print(Py*150,Dy*kd,Py*150 +  Dy*kd)
#         last_P = Py
#         robot.setJointPos(["head_pan"], [np.clip(np.radians(pan_angle), -95, 95)])

class States:
    INIT = -1
    READY = 0 # Waits for start button
    MOVING = 1 
    END = 99


tickrate = 60
rate = rospy.Rate(tickrate)

first_look_direction = ''



STEP_LEVEL = 0
time_for_Pd = time.time()
last_P = 0
Pthe = 0
Pa= 0
_the = 0
_y = 0
_head_pan = 0

currState = States.INIT
while not rospy.is_shutdown():
    
    if robot.buttonCheck("mode"):
    
        currState = States.INIT
        STEP_LEVEL = 0
        
    if DEBUG_MODE:
        # print("debug")
        # h ,w , x = vision.img_buffer[-1].shape
        # _theta = 15*(vision.results[0][0][0]-0.5)
        # print(h,w,x)
        # cv2.circle(vision.img_buffer[-1], (int(vision.results[0][0][0]*w),int(vision.results[0][0][1]*h)) , 3, (0,0,255), cv2.FILLED)
        # print(vision.img_buffer[-1].shape)
        # cv2.line(vision.img_buffer[-1], (vision.img_buffer[-1].shape[1]/2,0), (vision.img_buffer[-1].shape[1]/2,vision.img_buffer[-1].shape[0]), (0,0,255), 3)
        cv2.line(vision.img_buffer[-1], (vision.img_buffer[-1].shape[1]*27/40,0), (vision.img_buffer[-1].shape[1]*0,vision.img_buffer[-1].shape[0]), (0,0,255), 3)
        # (h, w) = vision.shape[:2]
        # center = (h//2,w//2)
        # rotated_image = vision.rotate(90)
        # output = cv2.warpAffine(vision, rotate, (480, 360))
        cv2.imshow("Image", vision.img_buffer[-1])
        # cv2.imshow("rotation_90",rotated_image)
        cv2.imshow("func1", vision.debug_img[0])
        # if vision.status[0]:
        #     print((vision.results))
        #     print((vision.results[0][0]))
        #     print((vision.results[0][1]))
            # print(caculate_theta_and_moving_head())
        caculate_theta_and_moving()
        print(vision.results[0][0])
        print(type(vision.results[0][0]))
            
            # print("--------------")
            
            # for i in range(w):
            #     print(vision.debug_img[0][10][i])
            # print("--------------")
            # print("Area 0: {}".format(vision.results[0][1]))
        cv2.waitKey(1)
    # if vision.status[0]:
    #     angle, distance = vision.results[0]
    #     print(round(angle,2), round(distance,2))
    
    if currState == States.INIT:
        print("[INIT]")
        robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])
        init()
        rospy.sleep(2)
        # Transition
        tick_count = 0
        direction = False
        currState = States.READY
        print("[INIT_FINISH]")

    elif currState == States.READY:
        # print("[READY]")
        
        if robot.buttonCheck("start"):
            DEBUG_MODE = False
        
            tick_count = 0
            currState = States.MOVING


    # elif currState == States.MOVING:
    #     # _the = caculate_theta_and_moving_head()
    #     # Py,Px,_the = caculate_theta_for_motor()
    #     # print(_the)
    #     caculate_theta_and_moving_head()


    rate.sleep()
        