
#! /usr/bin/env python
import rospy
from op3_ros_utils import *
import cv2
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import numpy as np
# from benson_imu import *

DEBUG_MODE = True
# DEBUG_MODE = False # Show the detected image True
DEBUG_MARKER = False
DL_MODE = False 


args1 = (np.array([15, 50, 50]), np.array([30, 255, 255])) #yellow
args2 = (np.array([100, 35, 30]), np.array([113, 255, 255])) #blue
args3 = ((np.array([0, 50, 0]), np.array([10, 255, 255])),
         (np.array([170, 50, 20]), np.array([179, 255, 255]))) #red

argsIMU = [68,True]

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


def detectSingleColor(img, hsv_params, color_name):
    img_copy = img.copy()
    lower = hsv_params[0]
    upper = hsv_params[1]

    hsv_img = cv2.cvtColor(img_copy, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_img, lower, upper)
    
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5)))

    # center_pos, area = findCenterOfLargestContour(mask)
    
    # img_copy[:, :, 2] = mask
    # img_copy[:, :, :2] = 0

    if DEBUG_MODE:
        result = cv2.bitwise_and(img_copy, img_copy, mask=mask)
        cv2.imshow(str(color_name), result)
        # cv2.imshow(str(color_name), img_copy)
        # cv2.waitKey(1)
    
    # if center_pos is not None:
    #     # Normalize by image dimension
    #     c_x = center_pos[0] / float(img.shape[1])
    #     c_y = center_pos[1] / float(img.shape[0])

        # center = (c_x, c_y)
        
    #     return (center, area), True
    # else:
    #     return None, False  

def detect2Color(img, hsv_params , color_name):
    img_copy = img.copy()
    lower1 = hsv_params[0][0]
    upper1 = hsv_params[0][1]
    lower2 = hsv_params[1][0]
    upper2 = hsv_params[1][1]

    hsv_img = cv2.cvtColor(img_copy, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv_img, lower1, upper1)
    mask2 = cv2.inRange(hsv_img, lower2, upper2)

    mask = cv2.bitwise_or(mask1, mask2)

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5)))

    # center_pos, area = findCenterOfLargestContour(mask)

    # img_copy[:, :, 2] = mask
    # img_copy[:, :, :2] = 0

    if DEBUG_MODE:
        result = cv2.bitwise_and(img_copy, img_copy, mask=mask)
        cv2.imshow(str(color_name), result)
        # cv2.waitKey(1)

    # if center_pos is not None:
    #     # Normalize by image dimension
    #     c_x = center_pos[0] / float(img.shape[1])
    #     c_y = center_pos[1] / float(img.shape[0])

    #     center = (c_x, c_y)
        
    #     return (center, area), True
    # else:
    #     return None, False

def findCenterOfLargestContour(binary_mask):
    """ Detects all contours in the image and returns the center position and 
        area of the largest contour.

        Parameters:
            binary_mask: A binary image, to detect the contours.

        Returns:
            (center_x, center_y), area: If no contours are detect it returns
                None, None.

    """
    _,contours,_ = cv2.findContours(binary_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) == 1:
        largest_cnt = 0
        largest_area = cv2.contourArea(contours[0])
    elif len(contours) > 1:
        largest_cnt = 0
        largest_area = cv2.contourArea(contours[0])
        for i, cnt in enumerate(contours[1:]):
            cnt_area = cv2.contourArea(cnt)
            if cnt_area > largest_area:
                largest_area = cnt_area
                largest_cnt = i+1 # Enumerate starts from 0, increment 1 here 
                                  # because we skip the first contour
    else: # No contours were found
        return None, None

    # Get moments of largest contour
    M = cv2.moments(contours[largest_cnt])
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    return (cx, cy), largest_area

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

time_for_Pd = time.time()


cap = cv2.VideoCapture(0)
currState = States.INIT
while not rospy.is_shutdown():
    if_get, img =  cap.read()
    if not if_get:
        print("no pic!")
        continue
    detectSingleColor(img,args1,"yellow")
    detectSingleColor(img,args2,"blue")
    detect2Color(img,args3,"red")

    if robot.buttonCheck("mode"):
        currState = States.INIT
    
    if DEBUG_MODE:
        cv2.imshow("img", img)
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
            currState = States.FOLLOW_LINE
  
   

    rate.sleep()

   