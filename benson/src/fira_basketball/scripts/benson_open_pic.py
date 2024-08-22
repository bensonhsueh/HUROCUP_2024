#! /usr/bin/python3

import rospy
import cv2
from vision_old import *


# Forgive me Sasuke
#from pygame import mixer

DEBUG_MODE = True # Show the detected image
MIN_AREA = 50 # Minimum area of objects to consider for vision
BALL_PICKUP_SIZE = 9000
BASKET_DUNK_SIZE = 62050
SIDE_STEP_TIME = 3.5
HEAD_SEARCH_SPEED = 0.065

# Functions to be passed to vision system
func1 = detectSingleColor
func3 = detect2Color
func2 = detectSingleColor

args1 = ((np.array([13, 145, 150]), np.array([25, 255, 255])),) #ball (orange)
args3 = ((np.array([0, 130, 0]), np.array([12, 255, 255])),
         (np.array([168, 60, 0]), np.array([180, 255, 255]))) #basket(red)
args2 = ((np.array([98, 100, 100]), np.array([104, 255, 255])),) #hand(blue)
#args3 = ((np.array([111, 100, 100]), np.array([153, 255, 255])),) #hand(purple)


# Create vision system
#vision = VisionSystem(pipeline_funcs=[func1, func2, func3],
#                      pipeline_args=[args1, args2, args3], debug=DEBUG_MODE, verbose=0)
vision = VisionSystem(pipeline_funcs=[func1, func2, func3],
                     pipeline_args=[args1, args2, args3], debug=DEBUG_MODE, verbose=0)

# Subscribe to cv_camera topic with vision system
rospy.Subscriber("/cv_camera/image_raw", Image, vision.read, queue_size=1)



# Iinitialize Node
rospy.init_node("fira_basketball")

rospy.sleep(3) # Make sure every publisher has registered to their topic,
               # avoiding lost messages

tickrate = 60
rate = rospy.Rate(tickrate)

first_look_direction = ''

while not rospy.is_shutdown():
    cv2.imshow("Image", vision.img_buffer[-1])
    cv2.imshow("Func1", vision.debug_img[0])
    cv2.imshow("Func2", vision.debug_img[1])
    cv2.imshow("Func3", vision.debug_img[2])
    # cv2.imshow("Func4", vision.debug_img[3])
    if vision.status[0]:
        print("Area 0: {}".format(vision.results[0][1]))
    if vision.status[1]:
        print("Area 1: {}".format(vision.results[1][1]))
    if vision.status[2]:
        print("Area 2: {}".format(vision.results[2][1]))
    #if vision.status[3]:
    #    print("Area 3: {}".format(vision.results[3][1]))
    cv2.waitKey(1)


    rate.sleep()
