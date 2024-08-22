#! /usr/bin/python3

import rospy
import cv2
from vision_old import *


def empty(v):
    pass



DEBUG_MODE = True # Show the detected image

# Functions to be passed to vision system
func1 = detectSingleColor

args1 = ((np.array([60, 77, 50]), np.array([88, 202, 255])),) #ball (orange)


vision = VisionSystem(pipeline_funcs=[func1],pipeline_args=[args1], debug=DEBUG_MODE, verbose=0)
rospy.Subscriber("/cv_camera/image_raw", Image, vision.read, queue_size=1)

rospy.init_node("benson_marathon")


# cv2.namedWindow("track_bar")
# cv2.resizeWindow("track_bar", 640,320)
# cv2.createTrackbar("hue min", "track_bar",   0 , 179, empty)
# cv2.createTrackbar("hue max", "track_bar", 179 , 179, empty)
# cv2.createTrackbar("sat min", "track_bar",   0 , 255, empty)
# cv2.createTrackbar("sat max", "track_bar", 255 , 255, empty)
# cv2.createTrackbar("val min", "track_bar",   0 , 255, empty)
# cv2.createTrackbar("val max", "track_bar", 255 , 255, empty)

rospy.sleep(3) # Make sure every publisher has registered to their topic,
               # avoiding lost messages

tickrate = 60
rate = rospy.Rate(tickrate)

first_look_direction = ''

while not rospy.is_shutdown():
    # h_min = cv2.getTrackbarPos("hue min", "track_bar")
    # h_max = cv2.getTrackbarPos("hue max", "track_bar")
    # s_min = cv2.getTrackbarPos("sat min", "track_bar")
    # s_max = cv2.getTrackbarPos("sat max", "track_bar")
    # v_min = cv2.getTrackbarPos("val min", "track_bar")
    # v_max = cv2.getTrackbarPos("val max", "track_bar")
    # print(h_min, h_max, s_min, s_max, v_min, v_max )

    # lower = np.array([h_min,s_min,v_min])
    # upper = np.array([h_max,s_max,v_max])
    # args1 = ((lower, upper),)
    # vision = VisionSystem(pipeline_funcs=[func1],pipeline_args=[args1], debug=DEBUG_MODE, verbose=0)
    h ,w , x = vision.debug_img[0].shape
    print(h,w,x)

    cv2.imshow("Image", vision.img_buffer[-1])
    cv2.imshow("Func1", vision.debug_img[0])
    if vision.status[0]:
        print((vision.results))
        print("--------------")
        # print("Area 0: {}".format(vision.results[0][1]))
    cv2.waitKey(1)


    rate.sleep()
