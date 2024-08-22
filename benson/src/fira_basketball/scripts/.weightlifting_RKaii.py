import rospy
import numpy as np
import cv2
import time
from op3_utils import Robot_weightlift
#from vision_weightlifting import *

from op3_ros_utils import getWalkingParams, Robot
from copy import copy
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
    PICK_BAR = 5
    WALK_WITH_BAR = 6
    LIFT_BAR = 7
    WALK_2_FINISH = 8
    END = 99

global last_Px, last_Py, time_for_Pd
last_Px = 0.0
last_Py = 0.0
time_for_Pd = time.time()

cap = cv2.VideoCapture(0)
ret, frame = cap.read()
hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
# Define your color range for detection
# lower_hsv = np.array([103, 169, 116])
# upper_hsv = np.array([179, 255, 255])
lower_hsv = np.array([121, 60, 143])
upper_hsv = np.array([179, 255, 255])
mask = cv2.inRange(hsv_frame, lower_hsv, upper_hsv)

# Iinitialize Node
rospy.init_node("fira_sprint")

# Create robot
robot = Robot()
robot_weightlift = Robot_weightlift('fira_basketball')

rospy.sleep(3) # Make sure every publisher has registered to their topic,
               # avoiding lost messages
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
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            # Draw a circle at the center on the original frame
            cv2.circle(frame, (cX, cY), 5, (255, 0, 0), -1)  # Blue dot
            # Draw the largest contour on the original frame
            cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 2)  # Green contour

            return cX, cY
        else:
            # cX, cY = 0, 0  # Default to (0,0) if contour is a line
            return None, None

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
    kp = 30
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

def init():
    # Set ctrl modules of all actions to joint, so we can reset robot position
    robot.setGeneralControlModule("action_module")

    # robot.setGrippersPos(left=0.0, right=0.0)

    # Call initial robot position
    # robot.playMotion(88, wait_for_end=True)
    robot_weightlift.walk_set_param_pub.publish(robot_weightlift.walking_params[0])

    # Set ctrl module to walking, this actually only sets the legs
    robot.setGeneralControlModule("walking_module")
    
    # Set joint modules of head joints to none so we can control them directly
    robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])

    robot.setJointPos(["head_tilt"], [-0.5])
    robot.setJointPos(["head_pan"], [0])

    rospy.sleep(1.0)

tickrate = 60
rate = rospy.Rate(tickrate)

# TODO remember to put to 0
STEP_LEVEL = 0

currState = States.INIT
while not rospy.is_shutdown():
    # print(vision.status[0])
    
    if robot.buttonCheck("mode"):
        STEP_LEVEL = 0
        currState = States.INIT
    
    if DEBUG_MODE:
        # pass
        ret, frame = cap.read()
        lower_hsv = np.array([103, 169, 116])
        upper_hsv = np.array([179, 255, 255])
        if ret:
            frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            result = detect_single_color(frame, lower_hsv, upper_hsv)
    
            # cX, cY = detect_single_color(frame, lower_hsv, upper_hsv)  # This function should return the centroid coordinates\
            if result is not None:  # Ensure there's a result to unpack
                cX, cY = result
                if cX is not None and cY is not None:
                    # Now it's safe to use cX and cY
                    camera_theta(cX, cY, frame_width, frame_height)
                else:
                    # Handle the case when no color is detected
                    print("No color detected")

            else:
                print("Detection function returned None")


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
            robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])
            robot.setJointPos(["head_pan", "head_tilt"], [0, -0.4])
            tick_count = 0
            robot.walkStart()
            currState = States.WALK_FORWARD
        
    elif currState == States.WALK_FORWARD:
        print("[WALK_FORWARD]")
        ret, frame = cap.read()
        if ret:
            frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            result = detect_single_color(frame, lower_hsv, upper_hsv)
            # cX, cY = detect_single_color(frame, lower_hsv, upper_hsv)  # This function should return the centroid coordinates\
            if result is not None:  # Ensure there's a result to unpack
                cX, cY = result
                if cX is not None and cY is not None:
                    # Now it's safe to use cX and cY
                    pan_angle, tilt_angle = camera_theta(cX, cY, frame_width, frame_height)
                    robot.walkVelocities(x = 0.015, y = 0, th=-np.clip(pan_angle, 10, -10), z_move_amplitude=0.03, balance=True, z_offset=0 )
                else:
                    # Handle the case when no color is detected
                    print("No color detected")
            else:
                print("Detection function returned None")
        cv2.imshow('Mask', mask)
        cv2.imshow('Frame', frame)
        if tilt_angle <= -80:
            robot.walkStop()
            # robot_weightlift.onlwalkCommand(direction="turn_left", start_leg="right", step_num=1,
            #         front_length=0.4, step_angle=15,step_time=0.5)
                
            currState = States.PICK_BAR
 
    elif currState == States.PICK_BAR:
        rospy.loginfo("[PICK_BAR]")
        # TODO testing
        #rospy.sleep(2)
        robot_weightlift.setGeneralControlModule("none")
        rospy.sleep(2)
        # robot.onlineWalkCommand(direction="turn_left", start_leg="right", step_num=3,
        #         front_length=1.5, step_angle=20,step_time=0.5)
        robot.setGeneralControlModule("action_module")
        robot.playMotion(177, wait_for_end=True)     #wait pos
        time.sleep(2)
        currState = States.WALK_WITH_BAR

    elif currState == States.WALK_WITH_BAR:
        print("[WALK_WITH_BAR]")
        

        # robot_weightlift.walking_params.append(robot_weightlift.loadWalkingParams('pickup_param.yaml'))
        #robot_weightlift.walking_params[2].hip_pitch_offset = -5
        #print(robot_weightlift.walking_params[2])
        # robot_weightlift.walking_params[1].init_x_offset = 0.015
        # robot_weightlift.walking_params[1].x_move_amplitude = 0.01
        # #print(robot_weightlift.walking_params[2])
        # robot_weightlift.walking_params[1].y_move_amplitude = -0.005
        # #TODO change the a move amplitude to 1
        # robot_weightlift.walking_params[1].angle_move_amplitude = 0 * DEGREE2RADIAN
        # robot_weightlift.walk_set_param_pub.publish(robot_weightlift.walking_params[1])
        # # Set ctrl module to walking, this actually only sets the legs
        # robot_weightlift.setJointsControlModule(["r_hip_yaw","l_hip_yaw","r_hip_roll","l_hip_roll","r_hip_pitch",
        # "l_hip_pitch","r_knee","l_knee","r_ank_pitch","l_ank_pitch","r_ank_roll","l_ank_roll"],
        # ["walking_module"])
        # #print(robot_weightlift.walking_params[2])
        # rospy.sleep(3)
        # robot_weightlift.walkStart()
        # rospy.sleep(11)
        #robot_weightlift.moveGripper(left=15.0,right=15.0) 
        

    #     robot_weightlift.walkStop()
    #     rospy.sleep(2)
    #     currState = States.LIFT_BAR

    elif currState == States.LIFT_BAR:
        print("[LIFT_BAR]")
        robot.setGeneralControlModule("action_module")
        robot.playMotion(178, wait_for_end=True)     #wait pos
        time.sleep(2)
            
    #     robot.setGeneralControlModule("action_module")
    #     robot.playMotion(183, wait_for_end=True)     #wait pos
    #     time.sleep(2)
        
    #     robot_weightlift.setJointsControlModule(['head_pan', 'head_tilt'],['none','none'])
    #     #robot_weightlift.moveHead(0,1.5)


    #     currState = States.WALK_2_FINISH

    # elif currState == States.WALK_2_FINISH:
    #     print("WALK_2_FINISH")
    #     robot.setJointPos(["head_tilt"], [-0.3])
    #     robot_weightlift.walking_params.append(robot_weightlift.loadWalkingParams('pickup_param_2.yaml'))
    #     #robot_weightlift.walking_params[2].hip_pitch_offset = -5
    #     robot_weightlift.walking_params[2].init_x_offset = -0.028
    #     robot_weightlift.walking_params[2].x_move_amplitude = 0.001
    #     robot_weightlift.walking_params[2].y_move_amplitude = 0.0
    #     #TODO change the a move amplitude to 1
    #     robot_weightlift.walking_params[2].angle_move_amplitude = 0 * DEGREE2RADIAN
    #     robot_weightlift.walk_set_param_pub.publish(robot_weightlift.walking_params[2])
    #     # Set ctrl module to walking, this actually only sets the legs
    #     robot_weightlift.setJointsControlModule(["r_hip_yaw","l_hip_yaw","r_hip_roll","l_hip_roll","r_hip_pitch",
    #     "l_hip_pitch","r_knee","l_knee","r_ank_pitch","l_ank_pitch","r_ank_roll","l_ank_roll"],
    #     ["walking_module"])
    #     #print(robot_weightlift.walking_params[2])
    #     rospy.sleep(3)
    #     robot_weightlift.walkStart()
    #     rospy.sleep(2)
    #     robot_weightlift.walkStop()
    #     robot.onlineWalkCommand(direction="turn_right", start_leg="right", step_num=110,
    #                 front_length=0.2, step_angle=70,step_time=0.5)
    #     robot_weightlift.walkStart()
    #     rospy.sleep(100)

        #robot_weightlift.moveGripper(left=15.0,right=15.0) 

        # robot.setJointPos(["head_tilt"], [-0.3])
        # robot_weightlift.walking_params.append(robot_weightlift.loadWalkingParams('pickup_param_2.yaml'))
        # #robot_weightlift.walking_params[2].hip_pitch_offset = -5
        # robot_weightlift.walking_params[1].init_x_offset = -0.032
        # robot_weightlift.walking_params[1].x_move_amplitude = 0.001
        # robot_weightlift.walking_params[1].y_move_amplitude = 0.0
        # #TODO change the a move amplitude to 1
        # robot_weightlift.walking_params[1].angle_move_amplitude = 0 * DEGREE2RADIAN
        # robot_weightlift.walk_set_param_pub.publish(robot_weightlift.walking_params[1])
        # # Set ctrl module to walking, this actually only sets the legs
        # robot_weightlift.setJointsControlModule(["r_hip_yaw","l_hip_yaw","r_hip_roll","l_hip_roll","r_hip_pitch",
        # "l_hip_pitch","r_knee","l_knee","r_ank_pitch","l_ank_pitch","r_ank_roll","l_ank_roll"],
        # ["walking_module"])
        # #print(robot_weightlift.walking_params[2])
        # rospy.sleep(3)
        # robot_weightlift.walkStart()
        # rospy.sleep(100)
        # #robot_weightlift.moveGripper(left=15.0,right=15.0) 

        # robot_weightlift.walkStop()
        # rospy.sleep(2)
        # currState = States.END
        # robot_weightlift.walking_params.append(robot_weightlift.loadWalkingParams('pickup_param.yaml'))
        # robot_weightlift.walking_params[2].hip_pitch_offset = 1 * DEGREE2RADIAN #1.5
        # robot_weightlift.walking_params[2].x_move_amplitude = 0
        # robot_weightlift.walking_params[2].balance_enable = True
        # robot_weightlift.walk_set_param_pub.publish(robot_weightlift.walking_params[2])
        
        # # Set ctrl module to walking, this actually only sets the legs
        # robot_weightlift.setJointsControlModule(["r_hip_yaw","l_hip_yaw","r_hip_roll","l_hip_roll","r_hip_pitch",
        # "l_hip_pitch","r_knee","l_knee","r_ank_pitch","l_ank_pitch","r_ank_roll","l_ank_roll"],
        # ["walking_module"])
        # rospy.sleep(5)
        # robot_weightlift.walkStart()
        # rospy.sleep(3)
        # robot_weightlift.walking_params[2].x_move_amplitude = 0.005
        # robot_weightlift.walk_set_param_pub.publish(robot_weightlift.walking_params[2])
        # rospy.sleep(1117)
        # robot_weightlift.walkStop()
        # currState = States.END
    #     # rate.sleep()    
    # elif currState == States.END:
    #     print("[END]")
    #     #robot_weightlift.walkStop()

        
    # rate.sleep()
