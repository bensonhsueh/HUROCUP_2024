# ! /usr/bin/python

import rospy
from op3_ros_utils import getWalkingParams, Robot
import time
import rospy
from sensor_msgs.msg import Imu
import numpy as np
import cv2

cap = cv2.VideoCapture(0)

class States:
    INIT = -1
    READY = 0 # Waits for start button
    WALK_FORWARD = 1 # Moves the head, looking for the ball
    # DETECT_RED_LINE = 2
    WALK_BACKWARDS = 2
    
    END = 99


# Iinitialize Node
rospy.init_node("fira_archery")

# Create robot
robot = Robot()

rospy.sleep(3) # Make sure every publisher has registered to their topic,
               # avoiding lost messages

def init():
    robot.setGeneralControlModule("walking_module")
    
    # Set joint modules of head joints to none so we can control them directly
    robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])

    robot.setJointPos(["head_tilt"], [-1]) 
    robot.setJointPos(["head_pan"], [0]) 

    rospy.sleep(1.0)
    print("init OK")

z_gyro = 0.0
z_gyro_offset = 0.0
z_gyro_offset_for_caculate = 0.0

def imu_get_yaw_by_integral(data):
    global z_gyro
    global z_gyro_offset
    angular_velocity = data.angular_velocity
    # print("aa")
    z_gyro = z_gyro - angular_velocity.z/2 + z_gyro_offset_for_caculate
    # z_gyro = angular_velocity.z/2
    # print("z_gyro_offset_for_caculate:",z_gyro_offset_for_caculate)
    z_gyro_offset = angular_velocity.z/2

def process_frame(frame):
    # Convert frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define lower and upper bounds for white color
    lower_white = np.array([0, 0, 200], dtype=np.uint8)
    upper_white = np.array([180, 25, 255], dtype=np.uint8)
    
    # Threshold the HSV image to get only white colors
    mask = cv2.inRange(hsv, lower_white, upper_white)
    
    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(mask, (5, 5), 0)
    
    # Apply Canny edge detection
    edges = cv2.Canny(blurred, 50, 150)
    
    # Detect lines using Hough transform
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=100, minLineLength=100, maxLineGap=10)
    
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    
            return True

rospy.Subscriber("/robotis/open_cr/imu", Imu, imu_get_yaw_by_integral, queue_size=1)
    
tickrate = 60
rate = rospy.Rate(tickrate)

currState = States.INIT
z_gyro_offset_for_caculate = z_gyro_offset
z_gyro = 0.0

reset = []
count = 0

# process_frame is not True

while not rospy.is_shutdown():

    ret, img = cap.read()
    if not ret:
        break

    if currState == States.INIT:
        print("[INIT]")
        init()
        
        currState = States.WALK_FORWARD
    
    elif currState == States.WALK_FORWARD:

        # print(z_gyro)
        count += 1
        print(count)
        robot.walkStart()
        robot.walkVelocities(x=15, y=10, th=np.clip(z_gyro, -15, 15), z_move_amplitude=0.045, balance=True, z_offset=0)
    
        # processed_frame = process_frame(img)
        
        # # Display processed frame
        # cv2.imshow('Video', processed_frame)
        
        # Check for 'q' key press to exit
        # if process_frame is not True:
        #     break

        # elif process_frame is True:
        #     currState == States.WALK_BACKWARDS
        
        if count >= 500:
            currState = States.WALK_BACKWARDS

    elif currState == States.WALK_BACKWARDS:

        if reset is None:
            z_gyro = 0
            rospy.sleep(1)

        # print(z_gyro)
        robot.walkStart()
        robot.walkVelocities(x=-15, y=-12, th=np.clip(z_gyro, -15, 15), z_move_amplitude=0.045, balance=True, z_offset=0)

        count -= 1

        if count == 0: 
            currState = States.END

    elif currState == States.END:

        robot.walkStop()
        break
    
    rate.sleep()

# cap.release()
# cv2.destroyAllWindows()





