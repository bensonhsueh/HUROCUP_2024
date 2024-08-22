#!/usr/bin/env python

import rospy
from op3_utils import Robot_weightlift
from vision_weightlifting import *
import cv2
import sys
import rosnode
import numpy as np

# Iinitialize Node
rospy.init_node('fira_hsv')


cap = cv2.VideoCapture(0)

rospy.sleep(1)

def nothing(x):
    pass

cv2.namedWindow('other')
cv2.moveWindow('other', 150, 50)
cv2.createTrackbar('H_lower', 'other', 0, 180, nothing)
cv2.createTrackbar('H_higher', 'other', 0, 180, nothing)
cv2.createTrackbar('S_lower', 'other', 0, 255, nothing)
cv2.createTrackbar('S_higher', 'other', 0, 255, nothing)
cv2.createTrackbar('V_lower', 'other', 0, 255, nothing)
cv2.createTrackbar('V_higher', 'other', 0, 255, nothing)

cv2.namedWindow('red1')
cv2.moveWindow('red1', 500, 50)
cv2.createTrackbar('red1_H_lower', 'red1', 0, 180, nothing)
cv2.createTrackbar('red1_H_higher', 'red1', 0, 180, nothing)
cv2.createTrackbar('red1_S_lower', 'red1', 0, 255, nothing)
cv2.createTrackbar('red1_S_higher', 'red1', 0, 255, nothing)
cv2.createTrackbar('red1_V_lower', 'red1', 0, 255, nothing)
cv2.createTrackbar('red1_V_higher', 'red1', 0, 255, nothing)

cv2.namedWindow('red2')
cv2.moveWindow('red2', 850, 50)
cv2.createTrackbar('red2_H_lower', 'red2', 0, 180, nothing)
cv2.createTrackbar('red2_H_higher', 'red2', 0, 180, nothing)
cv2.createTrackbar('red2_S_lower', 'red2', 0, 255, nothing)
cv2.createTrackbar('red2_S_higher', 'red2', 0, 255, nothing)
cv2.createTrackbar('red2_V_lower', 'red2', 0, 255, nothing)
cv2.createTrackbar('red2_V_higher', 'red2', 0, 255, nothing)

rospy.sleep(1)

lower_hsv = np.array([20,71,133])
upper_hsv = np.array([31,202,255])

lower_red1 = np.array([0, 0, 0])
upper_red1 = np.array([10, 255, 255])

lower_red2 = np.array([170, 0, 0])
upper_red2 = np.array([180, 0, 0])

while not rospy.is_shutdown():
    
    ret, frame = cap.read()
    H_l = cv2.getTrackbarPos('H_lower', 'other')
    H_h = cv2.getTrackbarPos('H_higher', 'other')
    S_l = cv2.getTrackbarPos('S_lower', 'other')
    S_h = cv2.getTrackbarPos('S_higher', 'other')
    V_l = cv2.getTrackbarPos('V_lower', 'other')
    V_h = cv2.getTrackbarPos('V_higher', 'other')
    lower_hsv = np.array([H_l, S_l, V_l])
    upper_hsv = np.array([H_h, S_h, V_h])

    red1_H_l = cv2.getTrackbarPos('red1_H_lower', 'red1')
    red1_H_h = cv2.getTrackbarPos('red1_H_higher', 'red1')
    red1_S_l = cv2.getTrackbarPos('red1_S_lower', 'red1')
    red1_S_h = cv2.getTrackbarPos('red1_S_higher', 'red1')
    red1_V_l = cv2.getTrackbarPos('red1_V_lower', 'red1')
    red1_V_h = cv2.getTrackbarPos('red1_V_higher', 'red1')
    lower_red1 = np.array([red1_H_l, red1_S_l, red1_V_l])
    upper_red1 = np.array([red1_H_h, red1_S_h, red1_V_h])

    red2_H_l = cv2.getTrackbarPos('red2_H_lower', 'red2')
    red2_H_h = cv2.getTrackbarPos('red2_H_higher', 'red2')
    red2_S_l = cv2.getTrackbarPos('red2_S_lower', 'red2')
    red2_S_h = cv2.getTrackbarPos('red2_S_higher', 'red2')
    red2_V_l = cv2.getTrackbarPos('red2_V_lower', 'red2')
    red2_V_h = cv2.getTrackbarPos('red2_V_higher', 'red2')
    lower_red2 = np.array([red2_H_l, red2_S_l, red2_V_l])
    upper_red2 = np.array([red2_H_h, red2_S_h, red2_V_h])

    frame = cv2.resize(frame, (0,0),fx=0.5,fy=0.5, interpolation=cv2.INTER_CUBIC)
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    mask_other = cv2.inRange(hsv_frame, lower_hsv, upper_hsv)

    mask_red1 = cv2.inRange(hsv_frame, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv_frame, lower_red2, upper_red2)
    mask_red = (mask_red1 + mask_red2)
    
    cv2.moveWindow('mask_for_other', 150, 350)
    cv2.imshow('mask_for_other', mask_other)

    cv2.moveWindow('mask_for_red', 500, 350)
    cv2.imshow('mask_for_red', mask_red)
    
    cv2.moveWindow('original_frame', 850, 350)
    cv2.imshow('original_frame', frame)

    print('\n\n\n\n\n')
    print('***************my value*****************')
    print('lower value of other:{}'.format(lower_hsv))
    print('upper value of other:{}'.format(upper_hsv))
    print('\n')
    print('lower value of red1:{}'.format(lower_red1))
    print('upper value of red1:{}'.format(upper_red1))
    print('lower value of red2:{}'.format(lower_red2))
    print('upper value of red2:{}'.format(upper_red2))
    print('****************************************')


    if cv2.waitKey(1) & 0xFF == ord('q'):
        print('lower value of HSV', lower_hsv)
        print('upper value of HSV', upper_hsv)
        break
