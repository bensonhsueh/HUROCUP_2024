#! /usr/bin/env python

import rospy
from op3_ros_utils import *
import cv2
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision import *
import time


DEBUG_MODE =  False# Show the detected image
DEBUG_MARKER = False
DL_MODE = False 


# Functions to be passed to vision system
func1 = detectSingleColor

args1 = ((np.array([0, 50, 60]), np.array([40, 255, 255])),) #yellow matt

cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
cv2.namedWindow("Func1", cv2.WINDOW_NORMAL)


vision = VisionSystem(pipeline_funcs=[func1],pipeline_args=[args1], debug=DEBUG_MODE, verbose=0)
rospy.Subscriber("/cv_camera/image_raw", Image, vision.read, queue_size=1)
# rospy.Subscriber("/video_source/raw", Image, vision.read)
rospy.init_node("Ray_obstacle")

robot = Robot()




rospy.sleep(4) # Make sure every publisher has registered to their topic,               # avoiding lost messages

# robot.setGeneralControlModule("action_module")
# robot.playMotion(1, wait_for_end=True)
def init():

    # Call initial robot position
    # robot.playMotion(9, wait_for_end=True)
    # robot.setGeneralControlModule("walking_module")
    robot.setGeneralControlModule("head_control_module")
    robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])
    robot.setJointPos(["head_tilt","head_pan"], [np.radians(-30),np.radians(0)])
    robot.setGeneralControlModule("walking_module")
    robot.walkStop()

def camera_theta():
    global last_P
    global time_for_Pd
    pan_angle_old = np.degrees(robot.joint_pos["head_pan"])
    pan_angle = 0
    if vision.status[0]:
        Py = (-1)*(vision.results[0][0][0]-0.5) # -0.5~0.5
        Dy = (Py - last_P)/(time.time()-time_for_Pd)
        time_for_Pd = time.time()
        # Px = (vision.results[0][0][1])
        kp = 80
        kd = 0
        pan_angle = pan_angle_old + Py*kp +  Dy*kd
        # pan_angle = np.clip(pan_angle, -95, 95)
        # print(Py*150,Dy*kd,Py*150 +  Dy*kd)
        last_P = Py
        robot.setJointPos(["head_pan"], [np.clip(np.radians(pan_angle), -95, 95)])

    # robot.setHeadJointPos(["head_pan"], [45])
    # print("sdfghjk")
    # rospy.sleep(0.1)
    # return np.clip(pan_angle, -45, 45)
    print(pan_angle)
    return pan_angle


class States:
    INIT = -1
    READY = 0 # Waits for start button
    WALK_FORWARD = 1 # Moves the head, looking for the ball
    WALK_SLOWLY_FORWARD = 2
    WALK_PREBACKWARDS = 3
    WALK_BACKWARDS = 4
    TURN_RIGHT = 5
    TURN_LEFT = 6
    END = 99
    WALK_FORWARD2 = 7


tickrate = 60
rate = rospy.Rate(tickrate)

first_look_direction = ''



STEP_LEVEL = 0
time_for_Pd = time.time()
last_P = 0
Pthe = 0
Py = 0
_the = 0
_y = 0
_head_pan = 0

currState = States.INIT
while not rospy.is_shutdown():
    
    if robot.buttonCheck("mode"):
    
        currState = States.INIT
        STEP_LEVEL = 0
    
    if DEBUG_MODE:
        camera_theta()
        # h ,w , x = vision.img_buffer[-1].shape
        # _theta = 15*(vision.results[0][0][0]-0.5)
        # print(h,w,x)
        # cv2.circle(vision.img_buffer[-1], (int(vision.results[0][0][0]*w),int(vision.results[0][0][1]*h)) , 3, (0,0,255), cv2.FILLED)
        cv2.imshow("Image", vision.img_buffer[-1])
        cv2.imshow("Func1", vision.debug_img[0])
        if vision.status[0]:
            print((vision.results))
            # print(vision.results[0][1])
            # print((vision.results[0][0][0]))
            # print((vision.results[0][1]))
            # print(caculate_theta_for_motor())
            # print(caculate_theta_and_moving_head())
            
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
        # robot.walkStart()
        rospy.sleep(2)
        # Transition
        tick_count = 0
        direction = False
        currState = States.READY
        print("[INIT_FINISH]")

    elif currState == States.READY:
        # print("[READY]")
        
        if robot.buttonCheck("start"):
        
            tick_count = 0
            robot.walkStart()
            currState = States.WALK_FORWARD

        if robot.buttonCheck("user"):
        
            tick_count = 0
            robot.walkStart()
            currState = States.WALK_BACKWARDS 
    elif currState == States.WALK_FORWARD:
        print('into walking')
        _the = camera_theta()
        if vision.status[0]:
            print('enter image')
            robot.walkVelocities(x=10, y=0, th=np.clip(_the, -20, 20), z_move_amplitude=0.025, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right
            if vision.results[0][1] > 160000:
                currState = States.TURN_RIGHT
                # if camera_theta():
                #     robot.walkVelocities(x=10, y=0, th=np.clip(_the, -20, 20), z_move_amplitude=0.025, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right
    elif currState == States.TURN_RIGHT:
        camera_theta()
        print('turning')
        robot.walkVelocities(x=0, y=0, th=-15, z_move_amplitude=0.025, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right 
        if not (np.degrees(robot.joint_pos["head_pan"]) <= 80 and np.degrees(robot.joint_pos["head_pan"]) >= -80):
            currState = States.WALK_FORWARD2
    elif currState == States.WALK_FORWARD2:
        print('walk forward 2')
        if vision.status[0]:
            print("AHHHHH")
            print((vision.results))
            print("BAAAAAAA")
            Center = vision.results[0][0][0]
            print("Center:",Center)
        robot.walkVelocities(x=10, y=0, th=0, z_move_amplitude=0.025, balance=True, z_offset=0)
        print("CCCCCCCCC")
        # time.sleep(3)
        # robot.walkStop()
        if Center >= 0.8:
            currState =  States.END 
            print('walk forward')
    elif currState == States.END:
        print('stop')
        _the = camera_theta()
        print(_the)
        if vision.status[0]:
            robot.walkVelocities(x=10, y=0, th=np.clip(_the, -15, 15), z_move_amplitude=0.025, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right
            if vision.results[0][1] > 200000:
                robot.walkStop()
                time.sleep(300)

    
        




                    

    rate.sleep()
