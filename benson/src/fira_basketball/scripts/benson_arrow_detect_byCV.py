#! /usr/bin/env python

import rospy
from op3_ros_utils import *
import cv2
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision import *
import time
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion


DEBUG_MODE = True # Show the detected image
DEBUG_MARKER = False
DL_MODE = False 


# Functions to be passed to vision system
func1 = benson_arrow_detect_CV
# func1 = detectSingleColor
# func1 = follow_line_benson

argsIMU = [68,True]
# argsIMU = [68,False]


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


vision = VisionSystem(pipeline_funcs=[func1],pipeline_args=[argsIMU], debug=DEBUG_MODE, verbose=0)
rospy.Subscriber("/cv_camera/image_raw", Image, vision.read, queue_size=1)
# rospy.Subscriber("/video_source/raw", Image, vision.read)
rospy.init_node("benson_arrow_detect_byCV")

robot = Robot()


# fourcc = cv2.VideoWriter_fourcc(*'X264')
# out = cv2.VideoWriter('output.mp4', fourcc, 20.0, (640, 480))

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))

rospy.sleep(1) # Make sure every publisher has registered to their topic,               # avoiding lost messages

# robot.setGeneralControlModule("action_module")
# robot.playMotion(1, wait_for_end=True)
def init():

    # Call initial robot position
    # robot.playMotion(9, wait_for_end=True)

    robot.setGeneralControlModule("head_control_module")
    robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])
    # robot.setJointPos(["head_tilt","head_pan"], [np.radians(-80),np.radians(0)])




class States:
    INIT = -1
    READY = 0 # Waits for start button
    START = 1
    MOVING = 2
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
        # cv2.line(vision.img_buffer[-1], (vision.img_buffer[-1].shape[1]/2,0), (vision.img_buffer[-1].shape[1]/2,vision.img_buffer[-1].shape[0]), (0,0,255), 3)
        cv2.imshow("Image", vision.img_buffer[-1])
        cv2.imshow("Func1", vision.debug_img[0])
        out.write(vision.debug_img[0])
        # print(vision.debug_img[0].shape)
        # print(vision.results[1])
        # cv2.imwrite("Image.png", vision.debug_img[0])
        # imggg = vision.debug_img[0]
        # blur_img = cv2.GaussianBlur(imggg, (5,5), 0)
        # gray_blur_img = cv2.cvtColor(blur_img, cv2.COLOR_BGR2GRAY)
        # ret, thresh_img = cv2.threshold(gray_blur_img, 127, 255, cv2.THRESH_BINARY)
        # contours,_ = cv2.findContours(thresh_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # if len(contours) > 0:
        #     for contour in contours:
        #         contour = cv2.convexHull(contour)
                # vertical = cv2.approxPolyDP(contours[0], 1,True)
                # for point in contour:
                #     x, y = point[0]
                #     cv2.circle(thresh_img, (x, y), radius=3, color=(200), thickness=-1)

        
        # vertical = cv2.apPROXPOLYDP(contours[0], 1)
        # 
        if vision.status[0]:
            print((vision.results))
            cv2.imwrite("Image.png", vision.debug_img[0])
        #     print((vision.results[0][0]))
        #     print((vision.results[0][1]))
            # print(caculate_theta_and_moving_head())
        # caculate_theta_and_moving_head()
            
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
            currState = States.START


    elif currState == States.START:
        pass
        # _the = caculate_theta_and_moving_head()
        # Py,Px,_the = caculate_theta_for_motor()
        # print(_the)
        # caculate_theta_and_moving_head()

    rate.sleep()
        
        
print("end")