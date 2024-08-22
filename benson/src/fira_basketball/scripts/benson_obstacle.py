
#! /usr/bin/env python
print('a')
import rospy
print('b')
from op3_ros_utils import *
print('c')
import cv2
print('d')
from std_msgs.msg import Float32
print('e')
from sensor_msgs.msg import Image
print('f')
from cv_bridge import CvBridge, CvBridgeError
print('g')
from vision import *
print('h')
import time
print('i')
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
# from benson_imu import *
print('j')

DEBUG_MODE = True
# DEBUG_MODE = False # Show the detected image True
DEBUG_MARKER = False
DL_MODE = False 


# Functions to be passed to vision system
func1 = detectSingleColor
func2 = detectSingleColor
func3 = detect2Color
# func1 = detect2Color
# func1 = follow_line_benson_V5

# func2 = benson_arrow_detect_CV
# func1 = follow_line_benson

args1 = ((np.array([15, 50, 50]), np.array([30, 255, 255])),) #yellow
args2 = ((np.array([100, 35, 30]), np.array([113, 255, 255])),) #blue
args3 = ((np.array([0, 50, 0]), np.array([10, 255, 255])),
         (np.array([170, 50, 20]), np.array([179, 255, 255]))) #red

argsIMU = [68,True]
vision = VisionSystem(pipeline_funcs=[func1,func2,func3],pipeline_args=[args1,args2,args3], debug=DEBUG_MODE, verbose=0)
rospy.Subscriber("/cv_camera/image_raw", Image, vision.read, queue_size=1)
# rospy.Subscriber("/video_source/raw", Image, vision.read)

pass_time = time.time()

z_gyro = 0.0
pitch = 0.0
z_gyro_offset = 0.0
z_gyro_offset_for_caculate = 0.0
def read_IMU(data):#imu_get_yaw_by_integral 
    global z_gyro
    global pitch
    global z_gyro_offset
    global argsIMU
    angular_velocity = data.angular_velocity
    orientation = data.orientation
    orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
    roll, pitch, _ = euler_from_quaternion(orientation_list)
    pitch = np.degrees(pitch)
    # print("aa")
    z_gyro = z_gyro - angular_velocity.z/2 + z_gyro_offset_for_caculate
    # z_gyro = angular_velocity.z/2
    # print("z_gyro_offset_for_caculate:",z_gyro_offset_for_caculate)
    z_gyro_offset = angular_velocity.z/2
    try:
        argsIMU[0] = pitch - np.degrees(robot.joint_pos["head_tilt"])#for caculating angle about camera and floor
    except:
        pass
    

rospy.Subscriber("/robotis/open_cr/imu", Imu, read_IMU, queue_size=1)




rospy.init_node("benson_obstacle")
robot = Robot()

rospy.sleep(2) # Make sure every publisher has registered to their topic,               # avoiding lost messages
z_gyro_offset_for_caculate = z_gyro_offset
z_gyro = 0.0
# robot.setGeneralControlModule("action_module")
# robot.playMotion(1, wait_for_end=True)
def init():
    # Call initial robot position
    # robot.playMotion(9, wait_for_end=True)
    # robot.setGeneralControlModule("walking_module")
    robot.setGeneralControlModule("head_control_module")
    robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])
    robot.setJointPos(["head_tilt","head_pan"], [np.radians(-60),np.radians(0)])
    robot.setGeneralControlModule("walking_module")
    robot.walkStop()

def caculate_theta_and_moving_head():
    global last_P
    global time_for_Pd
    pan_angle_old = np.degrees(robot.joint_pos["head_pan"])
    pan_angle = 0
    if vision.status[0] and vision.results[0][1] > 1000:
        Py = (-1)*(vision.results[0][0][0]-0.5) # -0.5~0.5
        Dy = (Py - last_P)/(time.time()-time_for_Pd)
        time_for_Pd = time.time()
        # Px = (vision.results[0][0][1])
        kp = 100
        kd = 0
        pan_angle = pan_angle_old + Py*kp +  Dy*kd
        pan_angle = np.clip(pan_angle, -70, 70)
        # print(Py*150,Dy*kd,Py*150 +  Dy*kd)
        last_P = Py
        robot.setJointPos(["head_pan"], [np.radians(pan_angle)])
    else:
        pan_angle = last_P

    # robot.setHeadJointPos(["head_pan"], [45])
    # print("sdfghjk")
    # rospy.sleep(0.1)
    # return np.clip(pan_angle, -45, 45)
    adjust =1
    return pan_angle*adjust


class States:
    INIT = -1
    READY = 0 # Waits for start button
    FOLLOW_LINE = 1
    FOLLOW_ARROW = 2
    FIDIND_ARROW = 3
    FOLLOW_IMU = 4
    END = 99

tickrate = 60
rate = rospy.Rate(tickrate)


STEP_LEVEL = 0
time_for_Pd = time.time()
last_P = 0
last_Px = 0
Pthe = 0
Py = 0
_the = 0
_y = 0
_head_pan = 0
arrow_count = 0
line_count = 0

search_angle_range = 45
search_angle_speed = 120

currState = States.INIT
while not rospy.is_shutdown():
    # args2[0] = pitch - np.degrees(robot.joint_pos["head_tilt"])#for caculating angle about camera and floor
    # caculate_theta_and_moving_head()
    # print(currState)
    
    if robot.buttonCheck("mode"):
    
        currState = States.INIT
        STEP_LEVEL = 0
    
    if DEBUG_MODE:
        pass
        cv2.imshow("Image", vision.img_buffer[-1])
        cv2.imshow("Func1", vision.debug_img[0])
        cv2.imshow("Func2", vision.debug_img[1])
        cv2.imshow("Func3", vision.debug_img[2])
        cv2.waitKey(1)
        
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
            z_gyro = 0.0
            currState = States.FOLLOW_LINE # FOLLOW_LINE
  
    elif currState == States.FOLLOW_LINE:
        robot.walkStart()
        # _the = caculate_theta_and_moving_head()
        caculate_theta_and_moving_head()
        _the = np.degrees(robot.joint_pos["head_pan"])
        if abs(_the) > 40:
            robot.walkVelocities(x=0, y=0, th=np.clip(_the, -15, 15), z_move_amplitude=0.045, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right
        else:
            robot.walkVelocities(x=20, y=0, th=np.clip(_the, -10, 10), z_move_amplitude=0.045, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right
        

    rate.sleep()

   