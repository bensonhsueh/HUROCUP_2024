#!/usr/bin/env python

import rospy
import cv2
import sys
import numpy as np
'''
lower_yellow = np.array([20,0,0])
upper_yellow = np.array([45,255,255])
lower_red_1 = np.array([0,30,25])
upper_red_1 = np.array([20,255,255])
lower_red_2 = np.array([140,0,0])
upper_red_2 = np.array([179,255,255])
'''
###old one### 
# lower_yellow = np.array([20,71,133])
# upper_yellow = np.array([31,202,255])
# lower_red_1 = np.array([1,116,178])
# upper_red_1 = np.array([5,208,224])
# lower_red_2 = np.array([172,38,100])
# upper_red_2 = np.array([179,201,255])
# ###new one###
# lower_yellow = np.array([20, 130, 90] )
# upper_yellow = np.array([38, 200, 150])
# lower_red_1 = np.array([0,130,30])
# upper_red_1 = np.array([14,255,200])
# lower_red_2 = np.array([174,130,30])
# upper_red_2 = np.array([180,255,200])
# right
# lower_yellow = np.array([20,60 , 109] )
# upper_yellow = np.array([39, 164, 191])
# lower_red_1 = np.array([0,90,88])
# upper_red_1 = np.array([12,255,200])
# lower_red_2 = np.array([174,130,30])
# upper_red_2 = np.array([180,255,200])
#left
lower_yellow = np.array([20,100 , 120] )
upper_yellow = np.array([40, 170, 200])
lower_red_1 = np.array([0,90,100])
upper_red_1 = np.array([20,255,200])
lower_red_2 = np.array([171,130,30])
upper_red_2 = np.array([180,255,200])

kernal = np.ones((5, 5), np.uint8)

def findYellowCnts(img):
    mask = cv2.inRange(img, lower_yellow, upper_yellow)
    blur = cv2.GaussianBlur(mask, (11,11),0)
    canny = cv2.Canny(blur, 30, 150)
    closing = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, kernal)
    opening = cv2.morphologyEx(closing, cv2.MORPH_CLOSE, kernal)
    cnts = cv2.findContours(opening, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[0]
    #print("Count {} Yellow". format(len(cnts)))
    print(len(cnts) )
    if(len(cnts) > 0):
        cnts = sorted(cnts, key = cv2.contourArea, reverse = True)
        return cnts[0]
    else:
        return None

def findRedCnts(img):
    mask_1 = cv2.inRange(img, lower_red_1, upper_red_1)
    mask_2 = cv2.inRange(img, lower_red_2, upper_red_2)
    mask = mask_1 + mask_2
    blur = cv2.GaussianBlur(mask, (11,11),0)
    canny = cv2.Canny(blur, 30, 150)
    closing = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, kernal)
    opening = cv2.morphologyEx(closing, cv2.MORPH_CLOSE, kernal)
    cnts = cv2.findContours(opening, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[0]
    #print("Count {} Red". format(len(cnts)))
    if(len(cnts) > 0):
        cnts = sorted(cnts, key = cv2.contourArea, reverse = True)
        return cnts[0]
    else:
        return None
        
def findCentroid(cnts):
    M = cv2.moments(cnts)
    cx = int(M["m10"]/(M["m00"]+1e-6))
    cy = int(M["m01"]/(M["m00"]+1e-6))
    return cx, cy


def focusCenter(img, cx_y, cx_r):
    bar_cx = (float(cx_y) + float(cx_r)) / 2
    view_cx = img.shape[1]/2
    delta_lr = (view_cx - bar_cx) / img.shape[1]
    #print(cx_y,cx_r,bar_cx,view_cx,img.shape[1])
    
    '''
    print("delta_lr: {}".format(delta_lr))
    if(delta_lr > 20):
        print("GO LEFT")
    elif(delta_lr < -20):
        print("GO RIGHT")
    else:
        print("GO FORWARD")  
    '''
    return delta_lr

def headTilt(img, cy_y, cy_r):
    bar_cy = (cy_y + cy_r) / 2
    view_cy = img.shape[0]/2
    delta_head = (view_cy - bar_cy) / img.shape[0] 
    '''
    print("delta_head: {}".format(delta_head))

    if(delta_head > 10):
        print("HEAD UP")
    elif(delta_head < -10):
        print("HEAD DOWN")
    else:
        print("HEAD KEEP")  
    '''
    return delta_head

def slope(cx_y, cy_y, cx_r, cy_r):
    
    bar_slope = (cy_y - cy_r) / (cx_y - cx_r + 1e-6)
    '''
    print("bar_slope: {}".format(bar_slope))
    if(bar_slope > 0.1):
        print("Turn facing right")
    elif(bar_slope < -0.1):
        print("Turn facing left")
    else:
        print("Keep facing forward")
    '''
    return bar_slope

        
if __name__ == "__main__":
    
    cap = cv2.VideoCapture(0)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        frame = cv2.resize(frame, (0,0),fx=0.5,fy=0.5, interpolation=cv2.INTER_CUBIC)
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        cnts_yellow = findYellowCnts(hsv_frame)
        cnts_red = findRedCnts(hsv_frame)
        cx_y, cy_y = findCentroid(cnts_yellow)
        cx_r, cy_r = findCentroid(cnts_red)

        
        focusCenter(hsv_frame, cx_y, cx_r)
        headTilt(hsv_frame, cy_y, cy_r)
        
        slope(cx_y, cy_y, cx_r, cy_r)
        
        cv2.drawContours(hsv_frame, cnts_yellow, -1, (255,0,0), 2)
        cv2.drawContours(hsv_frame, cnts_red, -1, (10,235,290), 2)
        
        cv2.circle(hsv_frame, (int((cx_y + cx_r) / 2),  int((cy_y + cy_r) / 2)),5,(130, 40, 255), -1)
        cv2.circle(hsv_frame, (int(frame.shape[1]/2), int(frame.shape[0]/2)),5,(130, 40, 255), -1)
        
        cv2.circle(hsv_frame, (cx_y, cy_y),5,(130, 40, 255), -1)
        cv2.circle(hsv_frame, (cx_r, cy_r),5,(130, 40, 255), -1)

        cv2.imshow('Current view',hsv_frame)
        
        cv2.waitKey(33)

