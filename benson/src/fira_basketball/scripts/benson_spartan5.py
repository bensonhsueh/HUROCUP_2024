#! /usr/bin/python

import rospy
import numpy as np
import cv2
import time
import math


from op3_ros_utils import getWalkingParams, Robot
from vision import *
from copy import copy
from sensor_msgs.msg import Imu
import sys

DEBUG_MODE = False
# DEBUG_MODE = True 
MIN_AREA = 300
CROSS_AREA = 1165
DEGREE2RADIAN = np.pi / 180

class States:
    INIT = -1
    READY = 0 # Waits for start button
    WALK_FORWARD = 1 # Moves the head, looking for the ball
    START_CLIMB = 5
    Adjust_Mode_V2 = 13
    CHECK_SLOPE_V2 = 14
    END = 99

z_gyro = 0.0
z_gyro_offset = 0.0
z_gyro_offset_for_caculate = 0.0

_count = 0
def imu_get_yaw_by_integral(data):
    global z_gyro
    global z_gyro_offset
    angular_velocity = data.angular_velocity
    # print("aa")
    # z_gyro = z_gyro - math.floor(angular_velocity.z/2*100)/100 + z_gyro_offset_for_caculate
    z_gyro = z_gyro - (angular_velocity.z/2) + z_gyro_offset_for_caculate
    # z_gyro = angular_velocity.z/2
    # print("z_gyro_offset_for_caculate:",z_gyro_offset_for_caculate)
    # print(z_gyro)
    z_gyro_offset = angular_velocity.z/2

rospy.Subscriber("/robotis/open_cr/imu", Imu, imu_get_yaw_by_integral, queue_size=1)

global last_Px, last_Py, time_for_Pd
last_Px = 0.0
last_Py = 0.0
time_for_Pd = time.time()

cap = cv2.VideoCapture(0)
ret, frame = cap.read()
hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
# Define your color range for detection

# lower_hsv = np.array([16, 77, 0])
# upper_hsv = np.array([43, 255, 198])
# (np.array([104, 86, 93]), np.array([179, 255, 255]
lower_hsv = np.array([100,  30, 75])
upper_hsv = np.array([ 120 ,255 ,255])

lower_g=np.array([68, 57, 48])
upper_g=np.array([ 96, 255, 255])

# lower_r=np.array([0, 59, 137])
# upper_r=np.array([ 180, 255, 181])

lower_w = np.array([ 0, 0, 190])
upper_w = np.array([180, 255, 255])

# lower value of yel1:[ 22  69 128]
# upper value of yel1:[ 38 255 255]

mask = cv2.inRange(hsv_frame, lower_hsv, upper_hsv)

go_back = 0
position_correct = []

# Iinitialize Node
rospy.init_node("fira_sprint")

# Create robot
robot = Robot()



rospy.sleep(3) # Make sure every publisher has registered to their topic,
               # avoiding lost messages
# rospy.sleep(2) # Make sure every publisher has registered to their topic,               # avoiding lost messages
z_gyro_offset_for_caculate = z_gyro_offset
z_gyro = 0.0
#sure
def detect_single_color(frame, lower_hsv, upper_hsv):
    # Convert frame to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Create a mask with the given HSV range
    mask = cv2.inRange(hsv_frame, lower_hsv, upper_hsv)
    
    # Convert mask to a 3-channel image to put a colored dot
    colored_mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    
    # Find contours in the mask
    # Note: cv2.findContours returns 3 values in older OpenCV versions used with Python 2.7
    _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    
    # Ensure at least one contour was found
    if contours:
        # Find the largest contour based on area
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Calculate the center of the contour
        M = cv2.moments(largest_contour)

        selection = cv2.minAreaRect(largest_contour)
        (x, y), (width, height), slope = selection    
        angle = compute_angle(width, height, slope)

        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            # Draw a circle at the center on the original frame
            cv2.circle(frame, (cX, cY), 5, (255, 0, 0), -1)  # Blue dot
            # Draw the largest contour on the original frame
            cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 2)  # Green contour
            return cX, cY, angle
        
        else:
            # cX, cY = 0, 0  # Default to (0,0) if contour is a line
            return None, None,None
        
def detect_single_color_V2(frame, lower_hsv, upper_hsv):
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_frame, lower_hsv, upper_hsv)

    _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    ratio = 2.0
    hor_contour = []
    for contour in contours:
        rect = cv2.minAreaRect(contour)
        (x, y), (width, height), slope = rect

        if width > height * ratio:
            hor_contour.append(contour)

    if hor_contour:

        largest_contour = max(hor_contour, key=cv2.contourArea)
        
        M = cv2.moments(largest_contour)

        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            cv2.circle(frame, (cX, cY), 5, (255, 0, 0), -1)  # Blue dot
            cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 2)  # Green contour

            angle = rect[2]
            if width < height:
                angle += 90
            
            cv2.imshow("frame",frame)
                
            return cX, cY, angle
        else:
            return None, None,None



def detect_line(frame,hsv_l,hsv_h):
    height,width=frame.shape[:2]
    point_x=width//2
    point_y=height-1-320

    color = frame[point_y,point_x]
    hsv_color=cv2.cvtColor(np.uint8([[color]]),cv2.COLOR_BGR2HSV)

    detected = cv2.inRange(hsv_color,hsv_l,hsv_h).any()
    circle_color = (0,0,0)
    cv2.circle(frame,(point_x,point_y),5,circle_color,-1)
    # return frame,detected
    return frame,detected

def compute_angle(width, height, angle):
    if angle < -90 or (width > height and angle < 0):
        return 90 + angle
    if width < height and angle > 0:
        return (90 - angle) * -1 
    return angle
#sure
def camera_theta(cX, cY, frame_width, frame_height):
    global last_Px, last_Py, time_for_Pd
    
    # Convert cX and cY to normalized coordinates relative to the frame center
    Px = -1 * ((cX / float(frame_width)) - 0.5)
    Py = -1 * ((cY / float(frame_height)) - 0.5)
    
    # Assuming robot.joint_pos is accessible and contains the current angles in radians
    pan_angle_old = np.degrees(robot.joint_pos["head_pan"])
    tilt_angle_old = np.degrees(robot.joint_pos["head_tilt"])
    
    # Calculate derivative of position
    Dx = (Px - last_Px) / (time.time() - time_for_Pd)
    Dy = (Py - last_Py) / (time.time() - time_for_Pd)
    time_for_Pd = time.time()
    
    # Proportional and derivative gains (tune these according to your robot's characteristics)
    kp = 20
    kd = 0
    
    # Calculate new angles
    pan_angle = pan_angle_old + Px * kp + Dx * kd
    tilt_angle = tilt_angle_old + Py * kp + Dy * kd
    
    # Update last position for next iteration
    last_Px, last_Py = Px, Py
    
    # Apply the new angles, ensuring they're within the mechanical limits of the robot
    robot.setJointPos(["head_pan", "head_tilt"], [np.clip(np.radians(pan_angle), np.radians(-95), np.radians(95)), np.clip(np.radians(tilt_angle), np.radians(-95), np.radians(95))])
    
    print("Pan angle:", pan_angle)
    print("Tilt angle:", tilt_angle)
    return pan_angle, tilt_angle

def detect_middle(frame,Cx):
    height, width = frame.shape[:2]
    frame_middle_x = width // 2
    orientation = 2
    cv2.line(frame,(int(Cx),0),(Cx,height),(255,0,0),2)
    if (frame_middle_x+4) >= Cx >= (frame_middle_x-4):
        print("touching")
        detect_touch = True
    elif Cx > frame_middle_x+4:           #line on the right
        orientation = 0
        detect_touch = False
    elif Cx < frame_middle_x-4:
        orientation = 1
        detect_touch = False
    # cv2.imshow('Frame_touch',frame)
    return frame,detect_touch, orientation


def init():
    # Set ctrl modules of all actions to joint, so we can reset robot position 
    robot.setGeneralControlModule("action_module")

    # robot.setGrippersPos(left=0.0, right=0.0)

    # Call initial robot position
    robot.playMotion(1, wait_for_end=True)

    # Set ctrl module to walking, this actually only sets the legs
    robot.setGeneralControlModule("walking_module")
    
    # Set joint modules of head joints to none so we can control them directly
    robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])

    robot.setJointPos(["head_tilt" , "head_pan"] , [-0.4 , 0])

    rospy.sleep(1.0)

tickrate = 60
rate = rospy.Rate(tickrate)

# TODO remember to put to 0
STEP_LEVEL = 0

currState = States.INIT
while not rospy.is_shutdown():
    


    ret, frame = cap.read()
    if ret:
        frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        result = detect_single_color(frame, lower_hsv, upper_hsv)
        if result is not None:  # Ensure there's a result to unpack
            cX, cY ,ang= result
            print("computed:",ang)
            if ang is not None:
                if ang>0:
                    ang=90.0-ang
                elif ang<=0:
                    ang=(-90.0)-ang
                    print("slope:",ang)
            if cX is not None and cY is not None:
                # Now it's safe to use cX and cY
                pan_angle, tilt_angle = camera_theta(cX, cY, frame_width, frame_height)
        cv2.imshow('MASK',mask)
        cv2.imshow('frame',frame)
        cv2.waitKey(1)
    
    if robot.buttonCheck("mode"):
        STEP_LEVEL = 0
        currState = States.INIT
    
    if DEBUG_MODE:
        lower_hsv = np.array([22,  69, 128])
        upper_hsv = np.array([ 38 ,255 ,255])
        lower_g=np.array([68, 57, 48])
        upper_g=np.array([ 96, 255, 255])

        lower_r=np.array([0, 59, 137])
        upper_r=np.array([ 180, 255, 181])

        lower_w = np.array([ 0, 0, 231])
        upper_w = np.array([180, 255, 255])
        
        # lower value of yellow1:[ 11  11 164]
        # upper value of yellow1:[ 68 255 255]
        # lower value of red1:[95 81  0]
# upper value of red1:[124 255 255]
        # ret, frame = cap.read()
        # detect_single_color(frame,lower_hsv,upper_hsv)
        
        ret, frame = cap.read()
        print("ret,",ret)
        height, width = frame.shape[:2]
        frame_middle_x = width // 2
        print(frame_middle_x)
        print("width",width)
        # line_frame,detect = detect_line(frame,lower_w,upper_w)
        print("WHATATATAT")
        # touch_frame,_,_=detect_middle(frame,cX)        
        # cv2.imshow("god",touch_frame)

        if ret:
            frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            # result = detect_single_color(frame, lower_hsv, upper_hsv)
            # result = detect_single_color(frame, lower_g, upper_g)
            result = detect_single_color(frame, lower_hsv, upper_hsv)

            # cX, cY ,ang= detect_single_color(frame, lower_hsv, upper_hsv)  # This function should return the centroid coordinates\
            
            if result is not None:  # Ensure there's a result to unpack
                cX, cY , ang = result
                if ang>0:
                    ang=90-ang
                elif ang<=0:
                    ang=(-90)-ang
                if cX is not None and cY is not None:
                    # Now it's safe to use cX and cY
                    # pan_angle,_= camera_theta(cX, cY, frame_width, frame_height)
                    print("slope:",ang)
                    print("robot_pan_ang:",abs(np.degrees(robot.joint_pos["head_pan"])))
                    touch_frame,tf,what=detect_middle(frame,cX)        
                    print("tf:",tf)
                    print("what:",what)
                    cv2.imshow("god",touch_frame)
        # cv2.imshow('MASK',mask)
        # cv2.imshow('frame',frame)
        cv2.waitKey(1)


    if currState == States.INIT:
        print("[INIT]")
        init()

        # Transition
        tick_count = 0
        direction = False
        currState = States.READY

    elif currState == States.READY:
        print("[READY]")
        if robot.buttonCheck("start"):
            # rospy.sleep(1)
            rospy.sleep(5)                      #for video taping
            z_gyro_offset_for_caculate = z_gyro_offset
            z_gyro = 0.0
            robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])
            robot.setJointPos(["head_pan", "head_tilt"], [0, -0.4])
            tick_count = 0
            robot.walkStart()
            currState = States.WALK_FORWARD
            

        
    elif currState == States.WALK_FORWARD:
        print("[WALK_FORWARD]")
        #robot.walkStart()

        # robot.walkVelocities(x=-4, th=5,z_move_amplitude=0.045, balance=True)

        #robot.walkVelocities(x = 1, y = -2, th=np.clip(pan_angle, -10, 10),z_move_amplitude=0.03, balance=True, z_offset=0,hip_pitch=9)
        # robot.walkVelocities(x = -1, y = 0, th=0, z_move_amplitude=0.035, balance=True, z_offset=0)
        # robot.walkVelocities(x = -2.5, y = 8, th=0, z_move_amplitude=0.045,balance=True, z_offset=0)
        # robot.walkVelocities(x = -4, y = 8, th=0, z_move_amplitude=0.045, balance=True, z_offset=0)
        # robot.walkVelocities(x=-5, th=3, balance=True)  #spin
        # robot.walkVelocities(x = -6, y = 8, th=0, z_move_amplitude=0.035,balance=True, z_offset=0)
        # robot.walkVelocities(x=-4, th=4,z_move_amplitude=0.045, balance=True)  #spin

        if result is not None and tilt_angle <= -45 and go_back == 0:        #-73 -50
            #robot.walkStop()
            currState = States.Adjust_Mode_V2
            rospy.sleep(2)
        if result is not None and tilt_angle <= -62 and go_back == 1:        #-73 -68
            #robot.walkStop()
            position_correct = False
            currState = States.Adjust_Mode_V2
            rospy.sleep(2)

        if result is not None and tilt_angle <= -62 and go_back == 2:        #-73 -68
            #robot.walkStop()
            position_correct = False
            currState = States.Adjust_Mode_V2
            rospy.sleep(2)
    # elif currState == States.Buffer:
    #     print('buffer')
    #     robot.walkstop()
    #     if slope < 2  

    elif currState == States.Adjust_Mode_V2:

        _count = _count+1


        # robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])
        # robot.setJointPos(["head_tilt" , "head_pan"] , [h_tilt , 0])              #70-->-1.5      45-->-0.8
        ret, frame = cap.read()

        if ret:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        lower_yellow = np.array([22,  69, 128])
        upper_yellow = np.array([ 38 ,255 ,255])

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        blurred = cv2.GaussianBlur(mask, (5, 5), 0)
        blurred = cv2.medianBlur(mask, 5)
        edges = cv2.Canny(blurred, 50, 150)
        kernel = np.ones((5,5),np.uint8)
        dilated_edges = cv2.dilate(edges, kernel, iterations=1)
        
        
        robot.walkStart()  # y =  np.clip(3*np.degrees(robot.joint_pos["head_pan"] ), -18, 10)  th=np.clip(z_gyro*1,-3,3) x = (-65-np.degrees(robot.joint_pos["head_tilt"]))*0.005
        robot.walkVelocities(x = 0.1, y = np.clip(np.degrees(robot.joint_pos["head_pan"]),-3,3)*1.5, th=np.clip(z_gyro,-1.8,1.8),balance=True,hip_pitch=10)# ,balance=True
        cv2.imshow('Video', frame)
        cv2.waitKey(1)
        print("[ANGLE_CORRECTION]")
        print(abs(70+np.degrees(robot.joint_pos["head_tilt"])+8),  (np.degrees(robot.joint_pos["head_pan"])),   z_gyro)
        if(abs(70+np.degrees(robot.joint_pos["head_tilt"] )+8)  < 7.5 and abs(np.degrees(robot.joint_pos["head_pan"])) < 5 and abs(z_gyro) < 10) or _count > 300:
            robot.walkStop()
            position_correct = False
            _count = 0
            currState = States.CHECK_SLOPE_V2
            rospy.sleep(1)
        # print('position',position_correct)



    elif currState == States.CHECK_SLOPE_V2:
        robot.walkStop()
        if _count < 60:
            _count = _count+1
            continue


        if(abs(70+np.degrees(robot.joint_pos["head_tilt"])+8) < 7.5 and abs(np.degrees(robot.joint_pos["head_pan"])) < 5 and abs(z_gyro) < 10):
            currState = States.START_CLIMB
            _count = 0
        else: 
            currState = States.START_CLIMB #Adjust_Mode_V2
            _count = 0

    elif currState == States.START_CLIMB:
        print("[GO_UP_STAIRS]")
        robot.setGeneralControlModule("action_module")
        robot.playMotion(2, wait_for_end=True)
        robot.playMotion(100, wait_for_end=True)
        # robot.playMotion(101, wait_for_end=True)
        robot.playMotion(2, wait_for_end=True)
        rospy.sleep(1.0)
        init()
        currState = States.WALK_TO_THE_LINE

 
    rate.sleep()
()