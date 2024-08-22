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
DEBUG_MODE = False # Show the de/tected image True
DEBUG_MARKER = False
DL_MODE = False 


# Functions to be passed to vision system
# func1 = detectSingleColor
# func1 = detect2Color
func1 = follow_line_benson_V5

func2 = benson_arrow_detect_CV
# func1 = follow_line_benson

# args1 = ((np.array([80, 80, 0]), np.array([100, 255, 255])),) #green
# args1 = ((np.array([104, 86, 0]), np.array([140, 255, 255])),) #blue
# args1 = ((np.array([80, 125, 0]), np.array([100, 255, 255])),
#         (np.array([100, 35, 30]), np.array([113, 255, 255])),
#         (np.array([0, 0, 0]), np.array([0, 0, 0]))) #green + blue

args1 = ((np.array([0, 60, 0]), np.array([30, 255, 255])),
        (np.array([150, 60, 0]), np.array([179, 255, 255])),
        (np.array([0, 0, 0]), np.array([0, 0, 0]))) #red
# args2 = ((np.array([65, 1, 0]), np.array([140, 255, 135])),) #I don't know but actually useless-=XZ
# args2 = (68,) #I don't know but actually useless-=XZ
argsIMU = [68,True]
vision = VisionSystem(pipeline_funcs=[func1,func2],pipeline_args=[args1,argsIMU], debug=DEBUG_MODE, verbose=0)
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
    # print("z_gyro:",z_gyro)
    # z_gyro = angular_velocity.z/2
    # print("z_gyro_offset_for_caculate:",z_gyro_offset_for_caculate)
    z_gyro_offset = angular_velocity.z/2
    try:
        argsIMU[0] = pitch - np.degrees(robot.joint_pos["head_tilt"])#for caculating angle about camera and floor
    except:
        pass
    

rospy.Subscriber("/robotis/open_cr/imu", Imu, read_IMU, queue_size=1)




rospy.init_node("benson_marathon")
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
    robot.setJointPos(["head_tilt","head_pan"], [np.radians(-40),np.radians(0)])
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

def caculate_theta_and_moving_head_arrow():
    global last_P
    global last_Px
    global time_for_Pd
    pan_angle_old = np.degrees(robot.joint_pos["head_pan"])
    tilt_angle_old = np.degrees(robot.joint_pos["head_tilt"])
    pan_angle = 0
    tilt_angle = 0
    Px = 0
    arrow_ang = 0
    if vision.status[1]:
        arrow_ang = vision.results[1][1]
        Py = (-1)*(vision.results[1][0][0]-0.5) # -0.5~0.5
        Px = (0.5-vision.results[1][0][1])
        # print(Px)
        Dy = (Py - last_P)/(time.time()-time_for_Pd)
        Dx = (Px - last_Px)/(time.time()-time_for_Pd)
        time_for_Pd = time.time()
        # Px = (vision.results[1][0][1])
        kp = 30
        kd = 0
        kpx = 30
        kdx = 0.1
        pan_angle = pan_angle_old + Py*kp +  Dy*kd
        pan_angle = np.clip(pan_angle, -45, 45)
        tilt_angle = tilt_angle_old + Px*kpx +  Dx*kdx
        tilt_angle = np.clip(tilt_angle, -80, -50)
        # pan_angle = Py*kp +  Dy*kd
        # pan_angle = np.clip(_theta, -70, 70)
        # print(Py*150,Dy*kd,Py*150 +  Dy*kd)
        last_P = Py
        last_Px = Px
        # robot.setJointPos(["head_pan","head_tilt"], [np.radians(pan_angle),np.radians(tilt_angle)])
        # robot.setJointPos(["head_pan","head_tilt"], [np.radians(pan_angle),np.radians(-70)])
        robot.setJointPos(["head_pan"], [np.radians(pan_angle)])
        # robot.setJointPos(["head_tilt"], [np.radians(tilt_angle)])
    else:
        pan_angle = last_P

    return pan_angle,arrow_ang,Px*30

def sea_the_line():
    return (vision.status[0] and vision.results[0][1] > 2000)

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
        # args2[0] = pitch - np.degrees(robot.joint_pos["head_tilt"])
        # print("z_gyro:",z_gyro,"pitch",pitch)
        # print("head_tilt",np.degrees(robot.joint_pos["head_tilt"]))
        # print(caculate_theta_and_moving_head_arrow())
        # robot.setJointPos(["head_tilt"], [np.radians(-90)])
        # caculate_theta_and_moving_head()
        # caculate_theta_and_moving_head_arrow()
        # robot.setJointPos(["head_pan"], [np.radians((time.time()-pass_time)%90 + 45)])
        # print()
        # search_angle = 0.0
        # search_angle_orig = (((time.time()-pass_time)*(search_angle_speed))%(search_angle_range*4))
        # if(search_angle_range*2 >= search_angle_orig  >=0):
        #     search_angle = search_angle_orig - search_angle_range
        # else:
        #     search_angle = ((search_angle_range*3) - (search_angle_orig ) )
        # print(search_angle)
        # robot.setJointPos(["head_pan"], [np.radians(search_angle)])
        # h ,w , x = vision.img_buffer[-1].shape
        # if vision.status[1]:
        #     print(abs(0.5-vision.results[1][0][1]), (abs( (0.5-vision.results[1][0][1])  )<0.1))
        #     dir = vision.results[1][1]
        #     # robot.setJointPos(["head_pan"], [np.radians(np.degrees(robot.joint_pos["head_pan"]) + 3*dir/abs(dir))])
        #     print(np.degrees(robot.joint_pos["head_pan"]) + 3*dir/abs(dir))
        #     print(vision.results[1])
            # print(vision.results[1][2])
            # _the = caculate_theta_and_moving_head_arrow()
            # print(_the)

        # caculate_theta_and_moving_head_arrow()
        # _theta = 15*(vision.results[0][0][0]-0.5)
        # print(h,w,x)
        # cv2.circle(vision.img_buffer[-1], (int(vision.results[0][0][0]*w),int(vision.results[0][0][1]*h)) , 3, (0,0,255), cv2.FILLED)
        cv2.imshow("Image", vision.img_buffer[-1])
        cv2.imshow("Func1", vision.debug_img[0])
        cv2.imshow("Func2", vision.debug_img[1])
        # print((2) < 3 and (not sea_the_line()))
        # print(sea_the_line())
        # if vision.status[0]:
        #     pass
            # print((vision.results[0]))
            # print((vision.results[0][0]))
            # print((vision.results[0][1]))
            # print(caculate_theta_for_motor())
            # print(caculate_theta_and_moving_head())
        # caculate_theta_and_moving_head()
            
            
            # print("--------------")
            
            # for i in range(w):
            #     print(vision.debug_img[0][10][i])
            # print("--------------")
            # print("Area 0: {}".format(vision.results[0][1]))
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
            robot.walkVelocities(x=25, y=0, th=np.clip(_the, -8, 8
                                                        ), z_move_amplitude=0.045, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right
        



        
        
    elif currState == States.FOLLOW_ARROW:
        # print("start FOLLOW_ARROW")
        # _the,arrow_angle,_x = caculate_theta_and_moving_head_arrow()
        arrow_angle = 0
        arrow_ans = "none"
        caculate_theta_and_moving_head_arrow()
        _the = np.degrees(robot.joint_pos["head_pan"])
        # print(_the)
        if abs(_the) > 40:
            robot.walkVelocities(x=-0.1, y=0, th=np.clip(_the, -15, 15), z_move_amplitude=0.045, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right
        else:
            robot.walkVelocities(x=np.clip(5, -10, 10), y=0, th=np.clip(_the, -15, 15), z_move_amplitude=0.045, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right
        robot.walkStart()
        if(vision.status[1]):
            print("maybe is arrow")
            robot.walkStop()
            pass_time = time.time()
            rospy.sleep(1)
            if(not vision.status[1]):
                print("vision.status GG")
                arrow_count = arrow_count + 1
                if(arrow_count > 1):
                    arrow_count = 0
                    print("give up arrow, back to line mode")
                    currState = States.FOLLOW_LINE
                # robot.walkStart()
                continue
            else:
                arrow_count = 0
                # cv2.imwrite('marker_status.png', vision.debug_img[1])
                arrow_angle = vision.results[1][1]
                arrow_ans = vision.results[1][2]
                


            rospy.sleep(1)
            angle_offset = 1
            # z_gyro_offset_for_caculate = z_gyro_offset
            z_gyro = (np.degrees(robot.joint_pos["head_pan"]))
            # z_gyro = (arrow_angle + np.degrees(robot.joint_pos["head_pan"])*angle_offset)
            print("ready to face to arrow, arrow_angle = " + str(arrow_angle) + " ,joint_pos" + str(np.degrees(robot.joint_pos["head_pan"])))
            print("z_gyro setting to " + str(z_gyro))
            robot.walkStart()
            pass_time = time.time()
            while((time.time() - pass_time) < 5 and abs(z_gyro)>3 ):
                caculate_theta_and_moving_head_arrow()
                if(vision.status[1]):
                    arrow_angle = vision.results[1][1]
                    arrow_ans = vision.results[1][2]
                z_gyro = (np.degrees(robot.joint_pos["head_pan"]))
                robot.walkVelocities(x = -0.1, y=0, th=np.clip(z_gyro, -10, 10), z_move_amplitude=0.045, balance=True, z_offset=0)
            z_gyro = 0
            robot.walkStop()
            robot.setJointPos(["head_pan"], [np.radians(0)])
            # rospy.sleep(1)


            # while not robot.buttonCheck("start"):
            #     robot.walkStop()
            # robot.walkStart()
            
            rospy.sleep(1)
            pass_time = time.time()
            while((time.time() - pass_time) < 1):
                if(vision.status[1]):
                    arrow_angle = vision.results[1][1]
                    arrow_ans = vision.results[1][2]
                    # cv2.imwrite('marker_status.png', vision.debug_img[1])
                    # cv2.waitKey(1)
            print("got angle after face to arrow = ", arrow_angle)

            # while not robot.buttonCheck("start"):
            #     robot.walkStop()

            
            pass_time = time.time()
            while((time.time() - pass_time) < 2):
                if(vision.status[1] and vision.results[1][0][1] > 0.5):
                    break
                robot.walkStart()
                robot.walkVelocities(x = 10, y=0, th=np.clip(z_gyro, -10, 10), z_move_amplitude=0.045, balance=True, z_offset=0)
                
            print("near to arrow")

            
            robot.walkStop()
            rospy.sleep(1)
            pass_time = time.time()
            while((time.time() - pass_time) < 1):
                if(vision.status[1]):
                    arrow_angle = vision.results[1][1]
                    arrow_ans = vision.results[1][2]
                    # cv2.imwrite('marker_status.png', vision.debug_img[1])
                    # cv2.waitKey(1)
            print("got angle after near to arrow = ", arrow_angle)



            # while not robot.buttonCheck("start"):
            #     robot.walkStop()

            robot.walkStart()
            pass_time = time.time()
            while((time.time() - pass_time) < 3.5):
                robot.walkVelocities(x = 10, y=0, th=np.clip(z_gyro, -10, 10), z_move_amplitude=0.045, balance=True, z_offset=0)
            
            z_gyro = ((arrow_angle + np.degrees(robot.joint_pos["head_pan"]))*angle_offset)
            print("ready to go to arrow, arrow_angle = " + str(arrow_angle) + " ,joint_pos" + str(np.degrees(robot.joint_pos["head_pan"])))
            print("z_gyro setting to " + str(z_gyro))

            # while not robot.buttonCheck("start"):
            #     robot.walkStop()
            
            pass_time = time.time()
            robot.walkStart()
            # if(arrow_ans == "straight"):
            #     while((time.time() - pass_time) < 5):
            #         robot.walkVelocities(x = 15, y=0, th=np.clip(z_gyro, -10, 10), z_move_amplitude=0.045, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right
            # elif(arrow_ans == "left"):
            #     while((time.time() - pass_time) < 4):
            #         robot.walkVelocities(x = 10, y=0, th=np.clip(z_gyro-90, -10, 10), z_move_amplitude=0.045, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right
            #     while((time.time() - pass_time) < 6):
            #         if abs(z_gyro) > 20:
            #             robot.walkVelocities(x=0, y=0, th=np.clip(z_gyro, -15, 15), z_move_amplitude=0.045, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right
            #         else:
            #             robot.walkVelocities(x=15, y=0, th=np.clip(z_gyro, -10, 10), z_move_amplitude=0.045, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right
            # elif(arrow_ans == "right"):
            #     while((time.time() - pass_time) < 4):
            #         robot.walkVelocities(x = 10, y=0, th=np.clip(z_gyro+90, -10, 10), z_move_amplitude=0.045, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right
            #     while((time.time() - pass_time) < 6):
            #         if abs(z_gyro) > 20:
            #             robot.walkVelocities(x=0, y=0, th=np.clip(z_gyro, -15, 15), z_move_amplitude=0.045, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right
            #         else:
            #             robot.walkVelocities(x=15, y=0, th=np.clip(z_gyro, -10, 10), z_move_amplitude=0.045, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right
            currState = States.FOLLOW_IMU
            print("ready to FOLLOW_IMU")
            robot.setJointPos(["head_pan"], [np.radians(0)])
        # else:
        #     robot.walkStop()
        #     t = time.time()
        #     while(time.time() - t < 0.5):
        #         caculate_theta_and_moving_head_arrow()
            # robot.walkStart()
        #     robot.walkVelocities(x=-5, y=0, th=0, z_move_amplitude=0.045, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right

    elif currState == States.FOLLOW_IMU: #IMU should be reset before
        search_angle = 0.0
        search_angle_orig = (((time.time()-pass_time)*(search_angle_speed))%(search_angle_range*4))
        if(search_angle_range*2 >= search_angle_orig  >=0):
            search_angle = search_angle_orig - search_angle_range
        else:
            search_angle = ((search_angle_range*3) - (search_angle_orig ) )
        # print(search_angle)
        robot.setJointPos(["head_pan"], [np.radians(search_angle)])
        robot.walkStart()
        # print(z_gyro)
        if abs(z_gyro) > 20:
            robot.walkVelocities(x=-0.1, y=0, th=np.clip(z_gyro, -15, 15), z_move_amplitude=0.045, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right
        else:
            robot.walkVelocities(x=10, y=0, th=np.clip(z_gyro, -10, 10), z_move_amplitude=0.045, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right
    # robot.walkVelocities(x=0, y=0, th=np.clip(z_gyro, -15, 15), z_move_amplitude=0.045, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right
        if(vision.status[1]):
            print("maybe is arrow")
            robot.walkStop()
            pass_time = time.time()
            # while((time.time() - pass_time) < 0.5):
            #     caculate_theta_and_moving_head_arrow()
            rospy.sleep(1)
            if(vision.status[1]):
                arrow_angle = 0.0
                print("back_to_arrow")
                # robot.walkStop()
                # while not robot.buttonCheck("start"):
                #     robot.walkStop()
                arrow_count = 0
                currState = States.FOLLOW_ARROW
                # rospy.sleep(1)
            else:
                pass_time = pass_time + 1
                # robot.walkStart()
        elif(sea_the_line()):
            print("maybe is line")
            robot.walkStop()
            rospy.sleep(0.5)
            if(sea_the_line()):
                pass_time2 = time.time()
                while((time.time() - pass_time2) < 1):
                    caculate_theta_and_moving_head()
                print("back_to_line")
                

                # while not robot.buttonCheck("start"):
                    # robot.walkStop()
                currState = States.FOLLOW_LINE
                # rospy.sleep(1)
            else:
                pass_time = pass_time + 1
                robot.walkStart()

    rate.sleep()

   