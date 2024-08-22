import rospy
from op3_ros_utils import *
import cv2
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision import *
import time


DEBUG_MODE = False # Show the detected image
DEBUG_MARKER = False
DL_MODE = False 


func1 = detectSingleColor

args1 = ((np.array([0, 50, 60]), np.array([40, 255, 255])),)#carpit(Yellow)

vision = VisionSystem(pipeline_funcs=[func1],pipeline_args=[args1], debug=DEBUG_MODE, verbose=0)
rospy.Subscriber("/cv_camera/image_raw", Image, vision.read, queue_size=1)
# rospy.Subscriber("/video_source/raw", Image, vision.read)
rospy.init_node("jay_sprint_test")

robot = Robot()

rospy.sleep(4)


tickrate = 60
rate = rospy.Rate(tickrate)

class States:
    INIT = -1
    INITB = -2
    READY = 0 # Waits for start button
    WALK_FORWARD= 1
    WALK_BACKWARDS=2 
    END = 99


def init():
    
    # Call initial robot position
    # robot.playMotion(9, wait_for_end=True)
    # robot.setGeneralControlModule("walking_module")
    robot.setGeneralControlModule("walking_module")
    robot.walkStop()

def initb():
    
    # Call initial robot position
    # robot.playMotion(9, wait_for_end=True)
    # robot.setGeneralControlModule("walking_module")
    robot.setGeneralControlModule("walking_module")
    robot.walkStop()

def caculate_theta_and_moving_head():
    global last_P
    global time_for_Pd
    pan_angle_old = np.degrees(robot.joint_pos["head_pan"])
    pan_angle = 0
    if vision.status[0]:
        Py = (-1)*(vision.results[0][0][0]-0.5) # -0.5~0.5
        Dy = (Py - last_P)/(time.time()-time_for_Pd)
        # print(time.time()-time_for_Pd)
        time_for_Pd = time.time()
        # Px = (vision.results[0][0][1])
        kp = 40
        kd = 1
        pan_angle = pan_angle_old+ Py*kp +  Dy*kd
        pan_angle= np.clip(pan_angle, -45, 45)
        # print(Py*150,Dy*kd,Py*150 +  Dy*kd)

        last_P = Py

        robot.setJointPos(["head_pan"], [np.radians(pan_angle)])
        print(vision.results[0][1])
    else :
        pan_angle = last_P

    return pan_angle


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
        caculate_theta_and_moving_head()

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
        robot.setJointPos(["head_pan"], [np.radians(0)])
        rospy.sleep(1)
        robot.setJointPos(["head_tilt"], [np.radians(-10)])
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

    elif  currState == States.WALK_FORWARD :
        _the = caculate_theta_and_moving_head()
        robot.walkVelocities(x=20, y=0, th=np.clip(_the, -20, 20), z_move_amplitude=0.035, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right
        # if abs(_the) > 30:
        #     robot.walkVelocities(x=0, y=0, th=np.clip(_the, -20, 20), z_move_amplitude=0.035, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right
        if vision.results[0][1]>110000:
            robot.walkStop()
            currState = States.INITB 
        # else:
        #     robot.walkVelocities(x=20, y=0*Py, th=np.clip(_the, -20, 20), z_move_amplitude=0.025, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right
        # _the = caculate_theta_and_moving_head()
        # robot.walkVelocities(x=20, y=0*Py, th=np.clip(_the, -30, 30), z_move_amplitude=0.025, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right
        # robot.walkVelocities(x=0, y=0, th=20, z_move_amplitude=0.025, balance=True, z_offset=0)
        # robot.walkVelocities(x=0, y=0, th=10, z_move_amplitude=0.025, balance=True, z_offset=0)
    elif  currState == States.WALK_BACKWARDS  :
        rospy.sleep(1)
        _the = caculate_theta_and_moving_head()
        robot.walkVelocities(x=-20, y=0, th=np.clip(-_the, -20, 20),z_move_amplitude=0.035, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right
        # if abs(_the) > 30:
        #     robot.walkVelocities(x=0, y=0, th=np.clip(_the, -20, 20), z_move_amplitude=0.035, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right
        if vision.results[0][1]>110000:
            robot.walkStop()
            currState = States.INIT
        # else:
        #     robot.walkVelocities(x=20, y=0*Py, th=np.clip(_the, -20, 20), z_move_amplitude=0.025, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right
        # _the = caculate_theta_and_moving_head()
        # robot.walkVelocities(x=20, y=0*Py, th=np.clip(_the, -30, 30), z_move_amplitude=0.025, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right
        # robot.walkVelocities(x=0, y=0, th=20, z_move_amplitude=0.025, balance=True, z_offset=0)
        # robot.walkVelocities(x=0, y=0, th=10, z_move_amplitude=0.025, balance=True, z_offset=0)
    elif currState == States.INITB :
            rospy.sleep(1)




    rate.sleep()

