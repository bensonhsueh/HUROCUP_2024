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
r_ang = 110

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
    # robot.setJointPos(["head_pan", "head_tilt"], [0, -1.6])
    robot.setJointPos(["head_pan", "head_tilt"], [0,np.radians(-85)])
    rospy.sleep(2.0)
    z_gyro = 0.0
    z_gyro_offset_for_caculate = z_gyro_offset

rospy.init_node("fira_Archery")

robot = Robot()

class States:
    INIT = -1
    READY = 0
    TURN = 1
    WALK_Y = 2
    STOP_ON_TOP = 7
    WALK_Y_DOWN = 8
    CORRECT_ANGLE = 3
    GO_UP_STAIRS = 4
    GO_DOWN_STAIRS = 5
    STOP = 6
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
pass_time = time.time()

tagret_degree = 0

hiest_stair = 1
current_stair = 0

while not rospy.is_shutdown():
    ret, frame = cap.read()

    if not ret:
        break
    
    height = frame.shape[0]
    crop_height = int(height * 0.6)
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
   
    


    cv2.imshow('Video', frame)
    cv2.imshow("mask_img",mask_img)
    cv2.imshow("edges",edges)
    cv2.waitKey(1)

    if currState == States.INIT:
        print("[INIT]")
        init()
        # rospy.sleep(1)

        z_gyro_offset_for_caculate = z_gyro_offset
        z_gyro = 0.0
        currState = States.READY

    elif currState == States.READY:
        print("[READY]")
        if robot.buttonCheck("start"):
            detect_line = []
            start_time = time.time()
            currState = States.TURN

# fdjoirjhoidthokdtfjmhgodfji
            
    elif currState == States.TURN:
        print("[WALK_Y]")
        robot.walkStart()
        robot.walkVelocities(x=-5, y=0, th=np.clip(z_gyro-r_ang, -10, 10), z_move_amplitude=0.045, balance=True, z_offset=0)
        if(time.time() - start_time)> 5:
            start_time = time.time()
            currState = States.WALK_Y


# kdthjfioesjftjkgdjnhm
            

    elif currState == States.WALK_Y:
        print("[WALK_Y]")
        robot.walkStart()
        robot.walkVelocities(x=-2, y=40, th=np.clip(z_gyro-r_ang, -10, 10), z_move_amplitude=0.045, balance=True, z_offset=0)
        if(time.time() - start_time)> 9:
            start_time = time.time()
            currState = States.STOP_ON_TOP
        # robot.walkStop()
            
    elif currState == States.STOP_ON_TOP:
        print("[WALK_Y]")
        robot.walkStop()
        if(time.time() - start_time)> 4:
            start_time = time.time()
            currState = States.WALK_Y_DOWN

    elif currState == States.WALK_Y_DOWN:
        print("[WALK_Y]")
        robot.walkStart()
        robot.walkVelocities(x=-2, y=-40, th=np.clip(z_gyro-r_ang, -10, 10), z_move_amplitude=0.045, balance=True, z_offset=0)
        if(time.time() - start_time)> 9:
            currState = States.STOP
        # robot.walkStop()
            
    elif currState == States.STOP:
        robot.walkStop()
        print("[STOP]")
        break

    elif currState == States.END:
        print("[END]")
        break

    cv2.waitKey(1)