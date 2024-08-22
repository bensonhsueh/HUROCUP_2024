#! /usr/bin/env python

import rospy
from op3_ros_utils import *
import cv2
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_old import *
import time


DEBUG_MODE = True # Show the detected image
DEBUG_MARKER = False
DL_MODE = False 


# Functions to be passed to vision system
func1 = detectSingleColor
# func1 = follow_line_benson

args1 = ((np.array([60, 50, 50]), np.array([88, 202, 255])),) #ball (orange)




vision = VisionSystem(pipeline_funcs=[func1],pipeline_args=[args1], debug=DEBUG_MODE, verbose=0)
rospy.Subscriber("/cv_camera/image_raw", Image, vision.read, queue_size=1)
# rospy.Subscriber("/video_source/raw", Image, vision.read)
rospy.init_node("benson_marathon")

robot = Robot()




rospy.sleep(4) # Make sure every publisher has registered to their topic,               # avoiding lost messages

# robot.setGeneralControlModule("action_module")
# robot.playMotion(1, wait_for_end=True)
def init():

    # Call initial robot position
    # robot.playMotion(9, wait_for_end=True)
    # robot.setGeneralControlModule("walking_module")
    # robot.setGeneralControlModule("head_control_module")
    robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])
    
    robot.setJointPos(["head_tilt","head_pan"], [np.radians(-100),np.radians(0)])
    # robot.setGeneralControlModule("walking_module")
    # robot.walkStop()
    time.sleep(3)
    robot.setJointsControlModule(["r_knee", "l_knee","r_hip_pitch","l_hip_pitch","r_hip_roll","l_hip_roll"], ["none", "none", "none", "none", "none", "none"])






def B():
    robot.setJointPos_V(["l_hip_pitch","r_hip_pitch"], [np.radians(57),np.radians(-57)],[0,0])
    # time.sleep(sec)
    # robot.setJointPos_V(["l_hip_pitch","r_hip_pitch"], [np.radians(23),np.radians(-23)],[0,0])
    # time.sleep(0.1)
    # robot.setJointPos_V(["l_hip_pitch","r_hip_pitch"], [np.radians(29),np.radians(-29)],[-1,-1])
    # time.sleep(sec)
    # robot.setJointPos_V(["l_hip_pitch","r_hip_pitch"], [np.radians(22),np.radians(-22)],[0,0])
    # time.sleep(1)



def F():
    robot.setJointPos_V(["l_hip_pitch","r_hip_pitch"], [np.radians(75),np.radians(-75)],[0,0])
    # time.sleep(sec)
    # robot.setJointPos_V(["l_hip_pitch","r_hip_pitch"], [np.radians(23),np.radians(-23)],[0,0])
    # time.sleep(0.1)

def R():
    robot.setJointPos_V(["r_hip_roll","l_hip_roll"], [np.radians(12),np.radians(12)],[0,0])
    # time.sleep(sec)
    # robot.setJointPos_V(["r_hip_roll","l_hip_roll"], [np.radians(0),np.radians(0)],[0,0])
    # time.sleep(0.1)

def L():
    robot.setJointPos_V(["r_hip_roll","l_hip_roll"], [np.radians(-20),np.radians(-20)],[0,0])
    # time.sleep(sec)
    # robot.setJointPos_V(["r_hip_roll","l_hip_roll"], [np.radians(0),np.radians(0)],[0,0])
    # time.sleep(0.1

def M_FB():
    robot.setJointPos_V(["l_hip_pitch","r_hip_pitch"], [np.radians(64),np.radians(-64)],[0,0])
    time.sleep(0.1)
    
def M_LR():
    robot.setJointPos_V(["r_hip_roll","l_hip_roll"], [np.radians(0),np.radians(0)],[0,0])
    time.sleep(0.1)

# def B():
#     robot.setJointPos_V(["l_hip_pitch","r_hip_pitch"], [np.radians(18),np.radians(-18)],[0,0])
#     # time.sleep(sec)
#     # robot.setJointPos_V(["l_hip_pitch","r_hip_pitch"], [np.radians(23),np.radians(-23)],[0,0])
#     # time.sleep(0.1)

# def R():
#     robot.setJointPos_V(["r_hip_roll","l_hip_roll"], [np.radians(3),np.radians(3)],[0,0])
#     # time.sleep(sec)
#     # robot.setJointPos_V(["r_hip_roll","l_hip_roll"], [np.radians(0),np.radians(0)],[0,0])
#     # time.sleep(0.1)

# def L():
#     robot.setJointPos_V(["r_hip_roll","l_hip_roll"], [np.radians(-15),np.radians(-15)],[0,0])
#     # time.sleep(sec)
#     # robot.setJointPos_V(["r_hip_roll","l_hip_roll"], [np.radians(0),np.radians(0)],[0,0])
#     # time.sleep(0.1

# def M_FB():
#     robot.setJointPos_V(["l_hip_pitch","r_hip_pitch"], [np.radians(23),np.radians(-23)],[0,0])
#     time.sleep(0.1)
    
# def M_LR():
#     robot.setJointPos_V(["r_hip_roll","l_hip_roll"], [np.radians(0),np.radians(0)],[0,0])
#     time.sleep(0.1)
    


class States:
    INIT = -1
    READY = 0 # Waits for start button
    MOVING = 1 
    END = 99


tickrate = 60
rate = rospy.Rate(tickrate)

first_look_direction = ''



STEP_LEVEL = 0
Pthe = 0
Py = 0
_the = 0
_y = 0
_head_pan = 0

currState = States.INIT
while not rospy.is_shutdown():#--
    if currState == States.INIT:
        print("[INIT]")
        init()
        M_FB()
        M_LR()
        # robot.setJointPos_V(["r_knee"], [np.radians(65)])
        # robot.setJointPos_V(["n_knee"], [np.radians(65)])
        print("[INIT_FINISH]")
        # Transition
        currState = States.MOVING
        
    
    elif currState == States.MOVING:
        print(np.degrees(robot.joint_pos["r_hip_pitch"]))
        # robot.setJointPos_V(["r_hip_roll"], [np.radians(2)])
        # robot.setJointPos_V(["r_knee"], [np.radians(65)])
        # robot.setJointPos_V(["r_knee","l_knee"], [np.radians(65),np.radians(-65)])
        # robot.setJointPos_V(["l_hip_pitch","r_hip_pitch"], [np.radians(22),np.radians(-22)])
        while robot.buttonCheck("start"):
            # robot.setJointPos_V(["r_hip_roll","l_hip_roll"], [np.radians(10),np.radians(10)])

            # time.sleep(1)
            # robot.setJointPos_V(["r_hip_roll","l_hip_roll"], [np.radians(2),np.radians(2)])
            # F()
            # time.sleep(1)
            F()
            time.sleep(1)
            R()
            time.sleep(3)
            L()
            time.sleep(3)
            M_FB()
            time.sleep(3)
            B()
            time.sleep(3)
            L()
            time.sleep(2)
            F()
            time.sleep(2)
            R()
            time.sleep(1)

            # B(1)
            # L(1)
            # R(1)






    
    rate.sleep()
