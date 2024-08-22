#!/usr/bin/env python
import cv2
import rospy
from op3_ros_utils import *
from std_msgs.msg import Float32 , Int8
from sensor_msgs.msg import Image
import time
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import heapq
import math
from cv_bridge import CvBridge, CvBridgeError
from fira_basketball.msg import ThreeDouble


DEBUG_MODE = True
# DEBUG_MODE = False # Show the detected image True
DEBUG_MARKER = False
DL_MODE = False 


argsIMU = [68,True]
camera_ang = 60
camera_h = 44.0
hfov = 53.0
vfov = 38.0#41.2794
hfov_rad = np.radians(hfov)
vfov_rad = np.radians(vfov)

pass_time = time.time()

oimg = []

IMU_RECORD = True
z_gyro = 0.0
pitch = 0.0
roll = 0.0
z_gyro_offset = 0.0
z_gyro_offset_for_caculate = 0.0
flag = 0

#for door
slop, err, _len = 0.0 ,0.0 ,0.0
pan_target  =  35
tilt_target = -18

def read_IMU(data):#imu_get_yaw_by_integral 
    if IMU_RECORD:
        global z_gyro
        global pitch
        global roll
        global z_gyro_offset
        global argsIMU
        angular_velocity = data.angular_velocity
        orientation = data.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        roll, pitch, _ = euler_from_quaternion(orientation_list)
        pitch = np.degrees(pitch)
        roll  = np.degrees(roll)
        # print("aa")
        z_gyro = z_gyro - angular_velocity.z/2 + z_gyro_offset_for_caculate
        # z_gyro = angular_velocity.z/2
        # print("z_gyro:",z_gyro)
        # print("z_gyro_offset_for_caculate:",z_gyro_offset_for_caculate)
        z_gyro_offset = angular_velocity.z/2
        try:
            argsIMU[0] = pitch - np.degrees(robot.joint_pos["head_tilt"])#for caculating angle about camera and floor
            # print("pitch",pitch)
            # print("roll",roll)
        except:
            pass
        # print("???:",z_gyro)
        # try:
            # robot.setJointPos(["head_pan"], [np.radians(np.clip(z_gyro,-60,60))])
            # robot.setJointPos(["head_tilt"], [np.radians(pitch - camera_ang)])
            # print(-camera_ang + pitch)
            # prtin
        # except:
        #     pass
    

directionx = 0.0
directiony = 0.0
find_the_path = 0.0

args_red = ((np.array([0, 50, 0]), np.array([10, 255, 255])),
         (np.array([170, 50, 20]), np.array([179, 255, 255]))) #red

def rotate_point(point, center, angle):

    angle_rad = np.radians(angle)
    
    translated_point = (point[0] - center[0], point[1] - center[1])
    
    rotation_matrix = np.array([
        [np.cos(angle_rad), -np.sin(angle_rad)],
        [np.sin(angle_rad), np.cos(angle_rad)]
    ])
    
    rotated_point = np.dot(rotation_matrix, translated_point)
    
    final_point = (rotated_point[0] + center[0], rotated_point[1] + center[1])
    
    return final_point

def read_obstacle_vision_data(data):#imu_get_yaw_by_integral 
    global directionx
    global directiony
    global find_the_path
    directionx = data.first
    directiony = data.second
    find_the_path = data.third
    # print("directionx:",directionx,"directiony",directiony,"find_the_path",find_the_path)


def get_img(data):
    bridge = CvBridge()
    try:
        # Convert the ROS Image message to OpenCV format
        global oimg
        oimg = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

def rotate_image(image_path, angle):
    # image = cv2.imread(image_path)
    image = image_path.copy()

    (h, w) = image.shape[:2]
    center = (w // 2, h // 2)

    M = cv2.getRotationMatrix2D(center, -angle, 1.0)
    rotated = cv2.warpAffine(image, M, (w, h))
    return rotated

def see_the_door():
    img_copy = oimg.copy()
    lower1 = args_red[0][0]
    upper1 = args_red[0][1]
    lower2 = args_red[1][0]
    upper2 = args_red[1][1]
    hsv_img = cv2.cvtColor(img_copy, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv_img, lower1, upper1)
    mask2 = cv2.inRange(hsv_img, lower2, upper2)
    mask = cv2.bitwise_or(mask1, mask2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5)))

    result = cv2.bitwise_and(img_copy, img_copy, mask=mask)

    _,contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    largest_cnt = 0
    largest_area = 0
    for contour in contours:
        if (cv2.contourArea(contour) > largest_area):
            largest_area = cv2.contourArea(contour)
            largest_cnt = contour
    # area = largest_area
    if largest_area > 20000:
        return True

    return False

def get_door_information():
    img_w = oimg.shape[1]
    img_h = oimg.shape[0]
    img_copy = oimg.copy()
    lower1 = args_red[0][0]
    upper1 = args_red[0][1]
    lower2 = args_red[1][0]
    upper2 = args_red[1][1]
    hsv_img = cv2.cvtColor(img_copy, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv_img, lower1, upper1)
    mask2 = cv2.inRange(hsv_img, lower2, upper2)
    mask = cv2.bitwise_or(mask1, mask2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5)))

    result = cv2.bitwise_and(img_copy, img_copy, mask=mask)

    _,contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    largest_cnt = 0
    largest_area = 0
    for contour in contours:
        if (cv2.contourArea(contour) > largest_area):
            largest_area = cv2.contourArea(contour)
            largest_cnt = contour
    # area = largest_area
    if largest_area < 5000:
        return False,0,(0,0)
    
    cnt =  cv2.approxPolyDP(largest_cnt, 7,True)
    xl,yl,xr,yr = 0,0,0,0
    max_left_down_val  = -1000000
    max_right_down_val = -1000000
    camera_ang_door = pitch-np.degrees(robot.joint_pos["head_tilt"])
    for point in cnt:
        x,y = point[0][0],point[0][1]
        left_down_val =  y - (x-img_w/2)
        right_down_val = y + (x-img_w/2)
        if left_down_val > max_left_down_val:
            xl,yl = x,y
            max_left_down_val = left_down_val
        if right_down_val > max_right_down_val:
            xr,yr = x,y
            max_right_down_val = right_down_val
        # result = cv2.circle(result, (point[0][0],point[0][1]), 10, (0,255,0), 3)
    try:
        ztheta = np.degrees(np.arctan((yl-(img_h/2))*np.tan(np.radians(vfov/2))/(img_h/2))) + camera_ang_door
        z = ((camera_h-30) / np.sin((ztheta) / 180 * np.pi)) * np.sin((90 - ztheta) / 180 * np.pi)
        
        rotate_ang = np.radians(abs(camera_ang_door-ztheta))
        rotate_z = z/np.cos(np.radians(ztheta))
        hfov_rotate = 2*np.arcsin( np.tan(hfov_rad/2) /       np.sqrt( np.tan(rotate_ang)**2     +   (1/(np.cos(hfov_rad/2)))**2 ) )
        hfov_rotate = np.degrees(hfov_rotate)

        xtheta = np.degrees(np.arctan((xl-(img_w/2))*np.tan(np.radians(hfov_rotate/2))/(img_w/2)))
        x = rotate_z * np.tan(np.radians(xtheta))

        yl_t = int(z)
        xl_t = int(x)


        ztheta = np.degrees(np.arctan((yr-(img_h/2))*np.tan(np.radians(vfov/2))/(img_h/2))) + camera_ang_door
        z = ((camera_h-30) / np.sin((ztheta) / 180 * np.pi)) * np.sin((90 - ztheta) / 180 * np.pi)
        
        rotate_ang = np.radians(abs(camera_ang_door-ztheta))
        rotate_z = z/np.cos(np.radians(ztheta))
        hfov_rotate = 2*np.arcsin( np.tan(hfov_rad/2) /       np.sqrt( np.tan(rotate_ang)**2     +   (1/(np.cos(hfov_rad/2)))**2 ) )
        hfov_rotate = np.degrees(hfov_rotate)

        xtheta = np.degrees(np.arctan((xr-(img_w/2))*np.tan(np.radians(hfov_rotate/2))/(img_w/2)))
        x = rotate_z * np.tan(np.radians(xtheta))

        yr_t = int(z)
        xr_t = int(x)

        print(xl,yl,xr,yr)

        print(xl_t,yl_t,xr_t,yr_t)

        print("--------------------")

        slope_t = float(yr_t - yl_t) / float(xr_t - xl_t)
        slope = float(yr - yl) / float(xr - xl)
        # slope = 10 
        slope = -math.degrees(math.atan(slope))
        slope_t = math.degrees(math.atan(slope_t))
        print("slope:",slope,"slope_t",slope_t,"pan ang:",np.degrees(robot.joint_pos["head_pan"]))
        # slope -= roll
    except Exception as e:
        print("slope error:",e)
        slope_t = 0
        slope = 0
    xm = (xr + xl) / 2
    ym = (yr + yl) / 2
    result = cv2.circle(result, (xl,yl), 10, (0,255,0), 3)
    result = cv2.circle(result, (xr,yr), 10, (255,0,0), 3)
    result = cv2.circle(result, (xm,ym), 10, (0,0,255), 3)
    _len = np.sqrt((xr - xl) ** 2 + (yr - yl) ** 2)
    # print(xl,yl,mask.shape[1],mask.shape[0],float(xl)/mask.shape[1],float(yl)/mask.shape[0])
    
    Px = 0.5 - float(yl)/mask.shape[0]
    Py = 0.5 - float(xl)/mask.shape[1]
    # print(Px,Py)

    # pan_angle_old = np.degrees(robot.joint_pos["head_pan"])
    # tilt_angle_old = np.degrees(robot.joint_pos["head_tilt"])
    # print(pan_angle_old,tilt_angle_old)
    # kp = 10
    # pan_angle = pan_angle_old + Py*kp
    # pan_angle = np.clip(pan_angle, -70, 70)
    # tilt_angle = tilt_angle_old + Px*kp
    # tilt_angle = np.clip(tilt_angle, -70, 70)
    # # print(Py*150,Dy*kd,Py*150 +  Dy*kd)
    # # last_P = Py
    # robot.setJointPos(["head_pan","head_tilt"], [np.radians(pan_angle),np.radians(tilt_angle)])
    # robot.setJointPos(["head_pan"], [np.radians(pan_angle)])
    # print("slope: ",slope)
    # print("mask.shape: ",mask.shape)
    # print(tilt_angle,pan_angle)


    if DEBUG_MODE:
        pass
        # result = cv2.bitwise_and(img_copy, img_copy, mask=mask)
        # cv2.line(result, (mx1, my1), (mx2, my2), (0,255,0), 2)
        # if lines is not None:
        #     for line in lines:
        #         x1, y1, x2, y2 = line[0]
        #         color = (np.random.randint(0, 256), np.random.randint(0, 256), np.random.randint(0, 256))
        #         cv2.line(result, (x1, y1), (x2, y2), color, 2)
        # result = rotate_image(result, roll)
        # cv2.imshow("red", result)

        # cv2.imshow("lines", lines)
        cv2.waitKey(1)
    return True, slope_t, (Px, Py)

def init():
    robot.walkStop()
    robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])
    robot.setJointPos(["head_pan","head_tilt"], [0,np.radians(pitch - camera_ang)])
    robot.setJointsControlModule(["l_sho_pitch","l_sho_roll","l_el"],["none","none","none"])
    robot.setJointPos(["l_sho_pitch","l_sho_roll","l_el"],[0.2,1.5,0])
    robot.setJointsControlModule(["r_sho_pitch","r_sho_roll","r_el"],["none","none","none"])
    robot.setJointPos(["r_sho_pitch","r_sho_roll","r_el"],[-0.2,-1.5,0])
    # robot.setGeneralControlModule("walking_module")
    robot.setJointsControlModule(["r_hip_yaw","l_hip_yaw","r_hip_roll","l_hip_roll","r_hip_pitch","l_hip_pitch","r_knee","l_knee","r_ank_pitch","l_ank_pitch","r_ank_roll","l_ank_roll"],["walking_module"])
    
    robot.walkStop()
    

class States:
    INIT = -1
    READY = 0 # Waits for start button
    OBSTACLE = 1
    OBSTACLE_NO_PATH = 2
    DOOR_LOCATE = 3
    DOOR_CROSS = 4
    END = 99

#HSV 
lower = np.array([[10, 90, 180], [90, 75, 0]])
upper = np.array([[30, 255, 255], [115, 255, 255]])
cl = ["y", "b"]


mapx = 90
mapz = 100

rospy.Subscriber("/cv_camera/image_raw", Image, get_img, queue_size=1)
rospy.Subscriber("/robotis/open_cr/imu", Imu, read_IMU, queue_size=1)
rospy.Subscriber('obstacle_vision_data', ThreeDouble, read_obstacle_vision_data)
pub = rospy.Publisher('obstacle_head_angle', Int8, queue_size=1)


rospy.init_node("benson_obstacle")
robot = Robot()

rospy.sleep(2) # Make sure every publisher has registered to their topic,               # avoiding lost messages
z_gyro_offset_for_caculate = z_gyro_offset
z_gyro = 0.0


tickrate = 60
rate = rospy.Rate(tickrate)

time_for_Pd = time.time()


currState = States.INIT
while not rospy.is_shutdown():
    # get_door_information()


    
    if robot.buttonCheck("mode"):
        currState = States.INIT
    if DEBUG_MODE:
        # robot.setJointPos(["head_tilt"], [np.radians(pitch - camera_ang)])
        # print(np.degrees(robot.joint_pos["head_tilt"]),pitch,-np.degrees(robot.joint_pos["head_tilt"])+pitch)
        # print(directionx,find_the_path)
        pass
    if currState == States.INIT:
        print("[INIT]")
        pub.publish(camera_ang)
        
        
        init()
        # robot.walkStart()
        rospy.sleep(3)
        
        # Transition
        tick_count = 0
        direction = False
        currState = States.READY
        # currState = States.DOOR_LOCATE
        # currState = States.DOOR_CROSS

        z_gyro_offset_for_caculate = z_gyro_offset
        z_gyro = 0.0
        print("[INIT_FINISH]")
    elif currState == States.READY:
        # print("[READY]")
        
        if robot.buttonCheck("start"):
        
            tick_count = 0
            robot.walkStart()
            z_gyro = 0.0
            currState = States.OBSTACLE
            # currState = States.DOOR_CROSS
            # currState = States.DOOR_LOCATE
            # currState = States.DOOR_CROSS
            print("start")

    elif currState == States.OBSTACLE:
        robot.setJointPos(["head_pan"], [np.radians(np.clip(z_gyro,-60,60))])
        step_lenx = 10
        step_leny = 20

        if find_the_path < 0.2:#==0
            currState = States.OBSTACLE_NO_PATH
            continue

        if abs(z_gyro) > 20:
            robot.walkVelocities(x=-0.1, y=0, th=np.clip(z_gyro, -5, 5), z_move_amplitude=0.045, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right
        else:
            robot.walkVelocities(x=directionx * step_lenx, y= directiony * step_leny, th=np.clip(z_gyro, -5, 5), z_move_amplitude=0.045, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right
        # robot.walkVelocities(x=directionx * step_lenx, y=directiony * step_leny, th=np.clip(z_gyro, -15, 15), z_move_amplitude=0.045, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right
        if(see_the_door()):
            robot.walkStop()
            currState = States.DOOR_LOCATE
            get_door_information()

    elif currState == States.OBSTACLE_NO_PATH:
        _flag = True
        ang = 0.0
        robot.walkStop()
        order = [30,40,60,75]
        for i in order:
            robot.setJointPos(["head_pan"], [np.radians(i)])
            rospy.sleep(2)
            if find_the_path > 0.5:
                _flag = False
                ang = i
                break
            robot.setJointPos(["head_pan"], [np.radians(-i)])
            rospy.sleep(2)
            if find_the_path > 0.5:
                _flag = False
                ang = -i
                break
        # for i in order:
        #     if _flag:
        #         robot.setJointPos(["head_pan"], [np.radians(-i)])
        #         rospy.sleep(2)
        #         if find_the_path > 0.5:
        #             _flag = False
        #             ang = -i
        #             break

        # while not robot.buttonCheck("start"):
        #     robot.walkStop()

        if not _flag:
            robot.setJointPos(["head_pan"], [np.radians(np.clip(z_gyro,-60,60))])
            step_lenx = 0
            step_leny = 25
            rotate_XY = rotate_point((directionx,directiony), (0,0), ang)
            directionx_rotate = rotate_XY[0]
            directiony_rotate = rotate_XY[1]
            print("directionx_rotate:",directionx_rotate,"directiony_rotate:",directiony_rotate)
            # y= directiony_rotate * step_leny
            pass_time = time.time()
            while(time.time() - pass_time) < 15:
                robot.walkStart()
                robot.walkVelocities(x=-7, y= directiony_rotate/abs(directiony_rotate) * step_leny, th=np.clip(z_gyro, -5, 5), z_move_amplitude=0.045, balance=True, z_offset=0)
                if (time.time() - pass_time) > 2 and find_the_path > 0.2:
                    break
            # robot.walkStop()
        else:
            robot.setJointPos(["head_pan"], [np.radians(np.clip(z_gyro,-60,60))])
            print("still no path, go back")
            pass_time = time.time()
            while(time.time() - pass_time) < 15:
                robot.walkStart()
                robot.walkVelocities(x=-10, y=0, th=np.clip(z_gyro, -3, 3), z_move_amplitude=0.045, balance=True, z_offset=0)
                if (time.time() - pass_time) > 2 and find_the_path > 0.2:
                    break

        # while not robot.buttonCheck("start"):
        #     robot.walkStop()
        currState = States.OBSTACLE
        
    elif currState == States.DOOR_LOCATE:
        if flag < 180:
            if flag == 0:
                robot.setJointPos(["head_pan"], [np.radians(0)])
                rospy.sleep(1)
            # robot.setJointPos(["head_pan","head_tilt"], [np.radians(0), np.radians(-40)])
            # robot.setJointPos(["head_pan"], [np.radians(0)])
            robot.walkStop()
            ret, _slop, _err = get_door_information()
            flag = flag + 1
            if ret:
                robot.walkStop()
                slop, err = _slop, _err
                target_slop = slop + np.degrees(robot.joint_pos["head_pan"])
            print("target_slop:",target_slop)

            pass_time = time.time()
            Px , Py  = _err[0] , _err[1]
            pan_angle_old = np.degrees(robot.joint_pos["head_pan"])
            tilt_angle_old = np.degrees(robot.joint_pos["head_tilt"])
            print(tilt_angle_old,pan_angle_old,z_gyro + target_slop)
            kp = 20
            pan_angle = pan_angle_old + Py*kp
            pan_angle = np.clip(pan_angle, -70, 70)
            tilt_angle = tilt_angle_old + Px*kp
            tilt_angle = np.clip(tilt_angle, -70, 70)
            # print(Py*150,Dy*kd,Py*150 +  Dy*kd)
            # last_P = Py
            x_step = tilt_angle - tilt_target
            y_step = pan_angle - pan_target
            robot.setJointPos(["head_pan","head_tilt"], [np.radians(pan_angle),np.radians(tilt_angle)])
            # if flag == 300:
                # robot.setJointPos(["head_pan","head_tilt"], [np.radians(0), np.radians(pitch - 15)])
            # robot.setJointPos(["head_pan","head_tilt"], [np.radians(0), np.radians(pitch - 35)])
            # print("z to :",target_slop)

                # z_gyro -= slop
            continue



        # while not robot.buttonCheck("start"):
        #     robot.walkStop()
        ret, _slop, _err = get_door_information()
        if ret:
            slop, err = _slop, _err

        Px , Py  = _err[0] , _err[1]
        pan_angle_old = np.degrees(robot.joint_pos["head_pan"])
        tilt_angle_old = np.degrees(robot.joint_pos["head_tilt"])
        # print(tilt_angle_old,pan_angle_old,z_gyro + target_slop)
        kp = 20
        pan_angle = pan_angle_old + Py*kp
        pan_angle = np.clip(pan_angle, -70, 70)
        tilt_angle = tilt_angle_old + Px*kp
        tilt_angle = np.clip(tilt_angle, -70, 70)
        # print(Py*150,Dy*kd,Py*150 +  Dy*kd)
        # last_P = Py
        x_step = tilt_angle - tilt_target
        y_step = pan_angle - pan_target
        # print(x_step,y_step,z_gyro + target_slop)
        robot.setJointPos(["head_pan","head_tilt"], [np.radians(pan_angle),np.radians(tilt_angle)])

        if (time.time() - pass_time) > 2:
            robot.walkStart()
        robot.walkVelocities(x=np.clip(x_step, -15, 5), y=np.clip(y_step, -15, 15), th=np.clip(z_gyro + target_slop, -5, 5), z_move_amplitude=0.045, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right
        

        if(abs(x_step) < 5 and abs(y_step) < 3 and abs(z_gyro + target_slop) < 3) or (time.time() - pass_time) > 25:
            flag = 0
            robot.walkStop()
            currState = States.DOOR_CROSS
            robot.walkStop()
            # while not robot.buttonCheck("start"):
            #     robot.walkStop()
        # robot.walkVelocities(x=np.clip(-5, -15, 5), y=np.clip(y_step, -15, 15), th=np.clip(0, -5, 5), z_move_amplitude=0.045, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right

    elif currState == States.DOOR_CROSS:
        # while not robot.buttonCheck("start"):
        #     robot.walkStop()
        robot.setGeneralControlModule("action_module")
        IMU_RECORD = False
        robot.playMotion(30, wait_for_end=True)
        for i in range(2):
            robot.playMotion(31, wait_for_end=True)
        IMU_RECORD = True
        robot.playMotion(32, wait_for_end=True)
        rospy.sleep(1)
        IMU_RECORD = False
        currState = States.OBSTACLE
        init()
        robot.setJointPos(["head_tilt"], [np.radians(pitch - camera_ang)])
        rospy.sleep(5)
        z_gyro_offset_for_caculate = z_gyro_offset
        z_gyro = z_gyro + 30
        IMU_RECORD = True
        # while not robot.buttonCheck("start"):
        #     robot.setJointPos(["head_pan"], [np.radians(np.clip(z_gyro,-60,60))])
        #     robot.walkStop()
        rospy.sleep(0.2)
        robot.walkStart()




robot.walkStop()



    

