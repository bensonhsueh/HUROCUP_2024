#! /usr/bin/env python

import rospy
from op3_ros_utils import *
import cv2
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision import *
import time


DEBUG_MODE = True # Show the detected image
DEBUG_MARKER = False
DL_MODE = False 


# Functions to be passed to vision system
func1 = detectSingleColor

args1 = ((np.array([0, 69, 203]), np.array([34, 255, 255])),) #ball (orange)




vision = VisionSystem(pipeline_funcs=[func1],pipeline_args=[args1], debug=DEBUG_MODE, verbose=0)
rospy.Subscriber("/cv_camera/image_raw", Image, vision.read, queue_size=1)
# rospy.Subscriber("/video_source/raw", Image, vision.read)
rospy.init_node("assinmant2")

robot = Robot()




rospy.sleep(4) # Make sure every publisher has registered to their topic,               # avoiding lost messages

# robot.setGeneralControlModule("action_module")
# robot.playMotion(1, wait_for_end=True)
def init():

    # Call initial robot position
    # robot.playMotion(9, wait_for_end=True)

    robot.setGeneralControlModule("head_control_module")
    robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])
    robot.setJointPos(["head_tilt","head_pan"], [np.radians(-12),np.radians(0)])



def caculate_theta_and_moving_head():
    global last_P
    global time_for_Pd
    tilt_angle_old = np.degrees(robot.joint_pos["head_tilt"])+12
    tilt_angle = 0
    if vision.status[0]:
        Py = (-1)*(vision.results[0][0][0]-0.5) # -0.5~0.5
        Dy = (Py - last_P)/(time.time()-time_for_Pd)
        # print(time.time()-time_for_Pd)
        time_for_Pd = time.time()
        # Px = (vision.results[0][0][1])
        kp = 60
        kd = 10
        tilt_angle =  Py*kp +  Dy*kd
        tilt_angle_out = np.clip(tilt_angle, -6, 6)-12
        # print(Py*150,Dy*kd,Py*150 +  Dy*kd)

        last_P = Py
        robot.setJointPos(["head_tilt"], [np.radians(tilt_angle_out)])

    # robot.setHeadJointPos(["head_pan"], [45])
        # print("Py:",Py,"tilt_angle:",tilt_angle,"tilt_angle_out",tilt_angle_out)
        # print("Py:",Py,"Dy",Dy,"tilt_angle:",tilt_angle,"tilt_angle_out",tilt_angle_out)
    # rospy.sleep(0.1)
    # return np.clip(pan_angle, -45, 45)


class States:
    INIT = -1
    READY = 0 # Waits for start button
    MOVING = 1 
    END = 99


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
        # print("debug")
        # h ,w , x = vision.img_buffer[-1].shape
        # _theta = 15*(vision.results[0][0][0]-0.5)
        # print(h,w,x)
        # cv2.circle(vision.img_buffer[-1], (int(vision.results[0][0][0]*w),int(vision.results[0][0][1]*h)) , 3, (0,0,255), cv2.FILLED)
        # print(vision.img_buffer[-1].shape)
        cv2.line(vision.img_buffer[-1], (vision.img_buffer[-1].shape[1]/2,0), (vision.img_buffer[-1].shape[1]/2,vision.img_buffer[-1].shape[0]), (0,0,255), 3)
        cv2.imshow("Image", vision.img_buffer[-1])
        cv2.imshow("Func1", vision.debug_img[0])
        # if vision.status[0]:
        #     print((vision.results))
        #     print((vision.results[0][0]))
        #     print((vision.results[0][1]))
            # print(caculate_theta_and_moving_head())
        caculate_theta_and_moving_head()
            
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
        rospy.sleep(2)
        # Transition
        tick_count = 0
        direction = False
        currState = States.READY
        print("[INIT_FINISH]")

    elif currState == States.READY:
        # print("[READY]")
        
        if robot.buttonCheck("start"):
            DEBUG_MODE = False
        
            tick_count = 0
            currState = States.MOVING


    elif currState == States.MOVING:
        caculate_theta_and_moving_head()

    rate.sleep()
        
