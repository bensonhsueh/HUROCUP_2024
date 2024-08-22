#! /usr/bin/env python
import cv2
import rospy
from op3_ros_utils import *
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
import time
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import heapq
import math

DEBUG_MODE = True
# DEBUG_MODE = False # Show the detected image True
DEBUG_MARKER = False
DL_MODE = False 

argsIMU = [68,True]
camera_ang = 45

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
    # print("z_gyro:",z_gyro)
    # print("z_gyro_offset_for_caculate:",z_gyro_offset_for_caculate)
    z_gyro_offset = angular_velocity.z/2
    try:
        argsIMU[0] = pitch - np.degrees(robot.joint_pos["head_tilt"])#for caculating angle about camera and floor
        # print("pitch",pitch)
    except:
        pass
    try:
        robot.setJointPos(["head_pan"], [np.radians(np.clip(z_gyro,-60,60))])
        # robot.setJointPos(["head_tilt"], [np.radians(pitch - camera_ang)])
        # print(-camera_ang + pitch)
        # prtin
    except:
        pass
    
    
def init():
    # Call initial robot position
    # robot.playMotion(9, wait_for_end=True)
    # robot.setGeneralControlModule("walking_module")
    robot.setGeneralControlModule("head_control_module")
    robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])
    robot.setJointPos(["head_tilt","head_pan"], [np.radians(-40),np.radians(0)])
    robot.setGeneralControlModule("walking_module")
    robot.walkStop()

def update_plot(px, py, color="red"):
    ax.clear()
    ax.set_xlim(-mapx, mapx)
    ax.set_ylim(0, mapz)
    ax.scatter(px, py, color=color, s=1)
    plt.draw()
    plt.pause(0.001)

def add_black_border(image, border_size):
    bordered_image = cv2.copyMakeBorder(
        image,
        top=border_size,
        bottom=border_size,
        left=border_size,
        right=border_size,
        borderType=cv2.BORDER_CONSTANT,
        value=[0, 0, 0]
    )
    return bordered_image

def a_star(map):
    start_point = 10
    start = (map.shape[0] - 1 - start_point, int((map.shape[1] - 1) / 2))
    # goal = (0, int((map.shape[1] - 1) / 2))
    goal = 0
    # goal = (0, np.argmin(map[0]))
    # print("goal:", goal)
    # print("start:", start)

    def heuristic(node, goal):
        return (node[0] - goal)
    

    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = np.full((map.shape[0], map.shape[1], 2), -1, dtype=int)
    g_score = np.full(map.shape, float('inf'), dtype=np.float64)
    g_score[start] = 0
    f_score = np.full(map.shape, float('inf'), dtype=np.float64)
    f_score[start] = heuristic(start, goal)

    while open_list:
        current = heapq.heappop(open_list)[1]

        if current[0] == goal:
            final_path = []
            while current != start:
                final_path.append(current)
                current = tuple(came_from[current[0], current[1]])
            final_path.append(start)
            final_path.reverse()
            # print("final_path:",final_path)
            return final_path

        for x in range(current[0] - 1, current[0] + 2):
            for y in range(current[1] - 1, current[1] + 2):
                if (x == current[0] and y == current[1]) or x < 0 or y < 0 or x >= map.shape[0] or y >= map.shape[1] or map[x, y] != 0:
                    continue
                
                tentative_g_score = g_score[current] + np.sqrt((x - current[0]) ** 2 + (y - current[1]) ** 2)

                if tentative_g_score < g_score[x, y]:
                    came_from[x, y] = current
                    g_score[x, y] = tentative_g_score
                    f_score[x, y] = tentative_g_score + heuristic((x, y), goal)
                    heapq.heappush(open_list, (f_score[x, y], (x, y)))

    print("No path found")
    return []

def create_map(oimg):
    oimg = add_black_border(oimg, 1)
    img_w = oimg.shape[1]
    img_h = oimg.shape[0]
    
    # print(img_w, img_h)

    hsv_img = cv2.cvtColor(oimg, cv2.COLOR_BGR2HSV)
    hsv_img = cv2.GaussianBlur(hsv_img, (5, 5), 0)
    px = []
    py = []
    list_x = []
    list_y = []
    stapx = []
    stapy = []
    for _ in range(0, len(lower)):

        output = cv2.inRange(hsv_img, lower[_], upper[_])
        output = cv2.Laplacian(output, -1, 1, 1)
        # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 1))
        # output = cv2.erode(output, kernel, iterations = 1)
        # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        ret, output = cv2.threshold(output, 30, 255, cv2.THRESH_BINARY)

        cv2.imshow(str(cl[_]), output)
        cv2.waitKey(1)

        __,contours, hierarchy = cv2.findContours(output, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        for contour in contours:
            if (cv2.contourArea(contour) < 1500):
                continue
            max_y = max(point[0][1] for point in contour)
            min_y = min(point[0][1] for point in contour)
            half_y = (max_y + min_y) // 2
            for i in range(len(contour)):
                if cl[_] == 'y':
                        # if slop > 0.1:# and contour[i][0][1] < img_h / 2:
                    stapx += [contour[i][0][0]]
                    stapy += [contour[i][0][1]]
                    # try:
                    #     map[contour[i][0][1]][contour[i][0][0]] = 255
                    # except:
                    #     pass
                    # map = cv2.circle(map, (contour[i][0][0], contour[i][0][1]), 2, 255, 3)
                if (contour[i - 1][0][0] - contour[i][0][0]) == 0:
                    continue
                slop = abs((contour[i - 1][0][1] - contour[i][0][1]) / (contour[i - 1][0][0] - contour[i][0][0]))
                if cl[_] == 'b' and contour[i][0][1] >= half_y:
                    if slop < 0.2:
                        stapx += [contour[i][0][0]]
                        stapy += [contour[i][0][1]]
                        # try:
                        #     map[contour[i][0][1]][contour[i][0][0]] = 255
                        # except:
                        #     pass
                        # map = cv2.circle(map, (contour[i][0][0], contour[i][0][1]), 2, 255, 3)
                

            list_x += [stapx]
            list_y += [stapy]

        """draw point"""
        color = (0, 0, 255)
        for x in range(0, len(list_x)):
            for seq in range(0, len(list_x[x])):
                oimg = cv2.circle(oimg, (list_x[x][seq], list_y[x][seq]), 2, color, 3)
                # map = cv2.circle(map, (list_x[x][seq], list_y[x][seq]), 2, 255, 3)

        camera_h = 46.0  # 43 cm
        # camera_ang = argsIMU[0]
        focal = 3.67
        h_sensor = 5.7
        v_sensor = img_w / img_h * h_sensor
        hfov = 78.0
        vfov = 42.0
        map = np.zeros((mapz,mapx*2,1),np.uint8)

        for i in range(0, len(list_x)):
            for seq in range(0, len(list_x[i])):
                ztheta = ((vfov / 2) / (img_h / 2)) * (list_y[i][seq] - (img_h / 2)) + camera_ang
                # print(ztheta)
                # print(math.sin((ztheta) / 180 * math.pi)) 
                # print(math.sin((ztheta)))
                # print("--------")
                z = (camera_h / np.sin((ztheta) / 180 * np.pi)) * np.sin((90 - ztheta) / 180 * np.pi)

                xtheta = ((hfov / 2) / (img_w / 2)) * (abs(list_x[i][seq] - (img_w / 2)))
                x = (z / np.sin((180 - 90 - xtheta) / 180 * np.pi)) * np.sin(xtheta / 180 * np.pi)
                if (list_x[i][seq] - (img_w / 2)) != 0:
                    x = x * ((list_x[i][seq] - (img_w / 2)) / abs((list_x[i][seq] - (img_w / 2))))
                else:
                    x = 0

                px += [x]
                py += [z]
                try:
                    if((mapz - int(z)) > 0):
                        map[mapz - int(z)][int(x) + mapx] = 255
                except:
                    pass
                # map = cv2.circle(map, (contour[i][0][0], contour[i][0][1]), 2, 255, 3)
                # print("x:", x, "z:", z)

    

    map = cv2.dilate(map, (np.ones((3, 3), np.uint8)), iterations=3)
    # map = cv2.resize(map, (0,0), fx=0.5, fy = 0.5)

    final_path = a_star(map)
    # directionx = 0
    # directionz = 0
    # direction  = 0

    dir_len = 10
    
    
    # if(final_path == [] or (len(final_path) < dir_len+1)):
    #     # directionx = 0
    #     # directionz = -1
    #     # direction = -100000
    #     # direction = -90
    #     directionx = 0
    #     directionz = -0.5
    # else:
    #     normalization = np.sqrt((final_path[dir_len][1] - final_path[0][1])**2 + (final_path[dir_len][0] - final_path[0][0])**2)
    #     directionx = -(final_path[dir_len][1] - final_path[0][1])/normalization
    #     directionz = -(final_path[dir_len][0] - final_path[0][0])/normalization
    #     # print(final_path[dir_len][1],final_path[0][1])
    #     # direction = directionx/directionz

    if(final_path == [] or (len(final_path) < dir_len+1)):
        # directionx = 0
        # directionz = -1
        # direction = -100000
        # direction = -90
        directionx = -1
        directionz = -0.5
        directionthe = 0
    else:
        normalization = np.sqrt((final_path[dir_len][1] - final_path[0][1])**2 + (final_path[dir_len][0] - final_path[0][0])**2)
        directionx = -(final_path[dir_len][0] - final_path[0][0])/normalization
        directionz = (final_path[dir_len][1] - final_path[0][1])/normalization
        directionthe = np.degrees(math.atan(directionx/directionz))
        if(directionthe >= 0):
            print("r")
            directionthe = 90 - directionthe
        else:
            print("l")
            directionthe = -90 - directionthe

    # print(directionx,directionz,directionthe)

    update_plot(px, py)
    # print("final_path :",final_path)
    map_with_line = map.copy()
    for i in range(0, len(final_path)):
        map_with_line[final_path[i][0]][final_path[i][1]] = 128
    map_with_line = cv2.resize(map_with_line, (0,0), fx=3, fy = 3)

    cv2.imshow("Video Feed", oimg)
    cv2.imshow("map_with_line", map_with_line)
    cv2.imshow("Mask", map)
    cv2.waitKey(1)

    return(directionx,directionthe)

class States:
    INIT = -1
    READY = 0 # Waits for start button
    OBSTACLE = 1
    FOLLOW_ARROW = 2
    FIDIND_ARROW = 3
    FOLLOW_IMU = 4
    END = 99

#HSV 
lower = np.array([[10, 90, 180], [90, 75, 0]])
upper = np.array([[30, 255, 255], [115, 255, 255]])
cl = ["y", "b"]


mapx = 90
mapz = 100

fig, ax = plt.subplots()
plt.axis('square')
plt.xlim(-mapx, mapx)
plt.ylim(0, mapz)

rospy.Subscriber("/robotis/open_cr/imu", Imu, read_IMU, queue_size=1)


rospy.init_node("benson_obstacle")
robot = Robot()

rospy.sleep(2) # Make sure every publisher has registered to their topic,               # avoiding lost messages
z_gyro_offset_for_caculate = z_gyro_offset
z_gyro = 0.0


tickrate = 60
rate = rospy.Rate(tickrate)

time_for_Pd = time.time()


cap = cv2.VideoCapture(0)
currState = States.INIT
while not rospy.is_shutdown():
    ret, oimg = cap.read()
    directionx,directionthe = create_map(oimg)
    # print("x :", directiony, "z :", directionx)
    # print("x :", directionx, "the :", directionthe)
    if not ret:
        break
    if robot.buttonCheck("mode"):
        currState = States.INIT
    
    if DEBUG_MODE:
        
        pass

        # cv2.imshow("img", img)
        # cv2.waitKey(1)
        
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
            currState = States.OBSTACLE

    elif currState == States.OBSTACLE:
        step_lenx = 0
        directionx = 0
        directionthe = 45
        # step_lenthe = 10
        if abs(z_gyro) > 20:
            robot.walkVelocities(x=-0.1, y=0, th=np.clip(z_gyro - directionthe, -15, 15), z_move_amplitude=0.045, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right
        else:
            robot.walkVelocities(x=directionx * step_lenx, y=0, th=np.clip(z_gyro - directionthe, -10, 10), z_move_amplitude=0.045, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right
        # robot.walkVelocities(x=directionx * step_lenx, y=directiony * step_leny, th=np.clip(z_gyro, -15, 15), z_move_amplitude=0.045, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right




    

