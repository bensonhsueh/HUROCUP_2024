import rospy
from op3_ros_utils import getWalkingParams, Robot
import cv2
import numpy as np
import time
import math
from scipy import optimize
from sensor_msgs.msg import Imu

z_gyro = 0.0
z_gyro_offset = 0.0
z_gyro_offset_for_caculate = 0.0

def imu_get_yaw_by_integral(data):
    global z_gyro
    global z_gyro_offset
    angular_velocity = data.angular_velocity
    z_gyro = z_gyro - angular_velocity.z / 2 + z_gyro_offset_for_caculate
    z_gyro_offset = angular_velocity.z / 2

rospy.Subscriber("/robotis/open_cr/imu", Imu, imu_get_yaw_by_integral, queue_size=1)

def empty(v):
    pass

def init():
    robot.setGeneralControlModule("action_module")
    robot.playMotion(1, wait_for_end=True)
    robot.setGeneralControlModule("walking_module")
    robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])
    robot.setJointPos(["head_pan", "head_tilt"], [0, -1.6])
    rospy.sleep(1.0)

rospy.init_node("fira_Archery")

robot = Robot()

class States:
    INIT = -1
    READY = 0
    WALK_TO_THE_LINE = 1
    CORRECT_ANGLE = 2
    GO_UP_STAIRS = 3
    END = 99

cap = cv2.VideoCapture(0)

start_time = time.time()
current_time = start_time

position_correct = False
detect_line = False

currState = States.INIT

rospy.sleep(1)

z_gyro_offset_for_caculate = z_gyro_offset
z_gyro = 0.0
count = 0

while not rospy.is_shutdown():
    ret, frame = cap.read()

    if not ret:
        break
    
    height = frame.shape[0]
    crop_height = int(height * 0.25)
    crop = 100
    frame = frame[:crop_height, crop:-crop]

    img_blur = cv2.GaussianBlur(frame, (3,3) , 5)
    hsv_img = cv2.cvtColor(img_blur, cv2.COLOR_BGR2HSV)
    # h_min = cv2.getTrackbarPos("hue min", "track_bar")
    # h_max = cv2.getTrackbarPos("hue max", "track_bar")
    # s_min = cv2.getTrackbarPos("sat min", "track_bar")
    # s_max = cv2.getTrackbarPos("sat max", "track_bar")
    # v_min = cv2.getTrackbarPos("val min", "track_bar")
    # v_max = cv2.getTrackbarPos("val max", "track_bar")
    h_min, h_max, s_min, s_max, v_min, v_max = 90, 120, 100, 255, 50, 255
    # print(h_min, h_max, s_min, s_max, v_min, v_max )
    lower = np.array([h_min,s_min,v_min])
    upper = np.array([h_max,s_max,v_max])

    mask_img = cv2.inRange(hsv_img, lower, upper)
    
    # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # img_blurred = cv2.GaussianBlur(gray, (7, 7), 0)
   
    img_blurred = cv2.GaussianBlur(mask_img, (7, 7), 0)
    img_blurred = cv2.medianBlur(img_blurred, 5)
    img_blurred = cv2.medianBlur(img_blurred, 5)
   
    edges = cv2.Canny(img_blurred, 50, 150)
   
    lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi/180, threshold=50, 
                            minLineLength=30, maxLineGap=20)
    
    smallest_y_line = None
    smallest_y_value = float('inf')
    
    detect_line = False

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = float(y2 - y1) / float(x2 - x1 + 1e-6)
            if abs(slope) < 0.3:
                detect_line = True
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)
        # print("Smallest y line parameters: ({}, {}, {}, {})".format(smallest_y_line[0], smallest_y_line[1], smallest_y_line[2], smallest_y_line[3]))

    cv2.imshow('Video', frame)
    cv2.imshow("mask",mask_img)
    cv2.imshow("mask",edges)
    cv2.waitKey(1)

    if currState == States.INIT:
        print("[INIT]")
        init()
        currState = States.READY

    elif currState == States.READY:
        print("[READY]")
        if robot.buttonCheck("start"):
            detect_line = []
            currState = States.WALK_TO_THE_LINE
            start_time = current_time

    elif currState == States.WALK_TO_THE_LINE:
        print("[WALK_TO_THE_LINE]")
        robot.walkStart()
        robot.walkVelocities(x=3, y=0, th=np.clip(z_gyro, -15, 15), z_move_amplitude=0.045, balance=True, z_offset=0)
        # robot.walkStop()

        if detect_line is True:
            print("STOP!!!")
            rospy.sleep(0.5)
            robot.walkStop()
            rospy.sleep(1)
            currState = States.CORRECT_ANGLE
            rospy.sleep(1)

            robot.setGeneralControlModule("walking_module")
            robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])
            robot.setJointPos(["head_pan", "head_tilt"], [0, -1.7])
            rospy.sleep(1.0)

    elif currState == States.CORRECT_ANGLE:
        print("[CORRECT_ANGLE]")
        if slope < -0.05:
            print("turn left")
            robot.walkStart()
            robot.walkVelocities(x=-5, th=5, balance=True, hip_pitch=7)
            robot.walkStop()
            rospy.sleep(0.1)
        elif slope > 0.05:
            print("turn right")
            robot.walkStart()
            robot.walkVelocities(x=-5, th=-5, balance=True, hip_pitch=7)
            robot.walkStop()
            rospy.sleep(0.1)
        else:
            print("position correct")
            position_correct = True
            rospy.sleep(1)

        if position_correct == True:
            robot.walkStop()
            currState = States.GO_UP_STAIRS
            rospy.sleep(2)

        z_gyro = 0

    elif currState == States.GO_UP_STAIRS:
        print("[GO_UP_STAIRS]")
        robot.setGeneralControlModule("action_module")
        robot.playMotion(2, wait_for_end=True)
        robot.playMotion(100, wait_for_end=True)
        robot.playMotion(101, wait_for_end=True)
        robot.playMotion(2, wait_for_end=True)
        rospy.sleep(1.0)
        break

    elif currState == States.END:
        print("[END]")
        break

    cv2.waitKey(1)

while True:
    ret, frame = cap.read()

    if not ret:
        break
    
    height = frame.shape[0]
    crop_height = int(height * 0.25)
    crop = 100
    frame = frame[:crop_height, crop:-crop]

    img_blur = cv2.GaussianBlur(frame, (3,3) , 5)
    hsv_img = cv2.cvtColor(img_blur, cv2.COLOR_BGR2HSV)
    h_min, h_max, s_min, s_max, v_min, v_max = 90, 120, 100, 255, 50, 255
    lower = np.array([h_min,s_min,v_min])
    upper = np.array([h_max,s_max,v_max])

    mask_img = cv2.inRange(hsv_img, lower, upper)
    
    img_blurred = cv2.GaussianBlur(mask_img, (7, 7), 0)
    img_blurred = cv2.medianBlur(img_blurred, 5)
    img_blurred = cv2.medianBlur(img_blurred, 5)
   
    edges = cv2.Canny(img_blurred, 50, 150)
   
    lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi/180, threshold=50, 
                            minLineLength=30, maxLineGap=20)
    
    smallest_y_line = None
    smallest_y_value = float('inf')
    
    detect_line = False

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = float(y2 - y1) / float(x2 - x1 + 1e-6)
            if abs(slope) < 0.3:
                detect_line = True
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)

    cv2.imshow('Video', frame)
    cv2.imshow("mask",mask_img)
    cv2.imshow("mask",edges)
    cv2.waitKey(1)

    if currState == States.INIT:
        print("[INIT]")
        init()
        currState = States.READY

    elif currState == States.READY:
        print("[READY]")
        detect_line = []
        currState = States.WALK_TO_THE_LINE
        start_time = current_time

    elif currState == States.WALK_TO_THE_LINE:
        print("[WALK_TO_THE_LINE]")
        robot.walkStart()
        robot.walkVelocities(x=3, y=0, th=np.clip(z_gyro, -15, 15), z_move_amplitude=0.045, balance=True, z_offset=0)
        # robot.walkStop()

        if detect_line is True:
            print("STOP!!!")
            rospy.sleep(0.5)
            robot.walkStop()
            rospy.sleep(1)
            currState = States.CORRECT_ANGLE
            rospy.sleep(1)

            robot.setGeneralControlModule("walking_module")
            robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])
            robot.setJointPos(["head_pan", "head_tilt"], [0, -1.7])
            rospy.sleep(1.0)

    elif currState == States.CORRECT_ANGLE:
        print("[CORRECT_ANGLE]")
        if slope < -0.05:
            print("turn left")
            robot.walkStart()
            robot.walkVelocities(x=-5, th=5, balance=True, hip_pitch=7)
            robot.walkStop()
            rospy.sleep(0.1)
        elif slope > 0.05:
            print("turn right")
            robot.walkStart()
            robot.walkVelocities(x=-5, th=-5, balance=True, hip_pitch=7)
            robot.walkStop()
            rospy.sleep(0.1)
        else:
            print("position correct")
            position_correct = True
            rospy.sleep(1)

        if position_correct == True:
            robot.walkStop()
            currState = States.GO_UP_STAIRS
            rospy.sleep(2)

        z_gyro = 0

    elif currState == States.GO_UP_STAIRS:
        print("[GO_UP_STAIRS]")
        robot.setGeneralControlModule("action_module")
        robot.playMotion(2, wait_for_end=True)
        robot.playMotion(100, wait_for_end=True)
        robot.playMotion(101, wait_for_end=True)
        robot.playMotion(2, wait_for_end=True)
        rospy.sleep(1.0)
        break

    elif currState == States.END:
        print("[END]")
        break

    cv2.waitKey(1)