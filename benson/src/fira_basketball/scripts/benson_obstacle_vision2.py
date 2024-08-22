#! /usr/bin/env python
import cv2
import rospy
from std_msgs.msg import Float32 , Int8
from sensor_msgs.msg import Image
import time
from tf.transformations import euler_from_quaternion
import numpy as np
import matplotlib.pyplot as plt
import heapq
import math
from cv_bridge import CvBridge, CvBridgeError
from fira_basketball.msg import ThreeDouble

DEBUG_MODE = True
# DEBUG_MODE = False # Show the detected image True
DEBUG_MARKER = False
DL_MODE = False 

camera_ang = 45

mapx = 90
mapz = 120

directionx = 0.0
directiony = 0.0
directionthe = 0.0
# close_to_obsticle = False

fig, ax = plt.subplots()

pass_time = time.time()
    
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
    start_point = 0
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

    # print("No path found")
    return []

def create_map(data):
    bridge = CvBridge()
    try:
        # Convert the ROS Image message to OpenCV format
        oimg = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    # cv2.imshow("oimg", oimg)
    # cv2.waitKey(1)

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

        # cv2.imshow(str(cl[_]), output)
        # cv2.waitKey(1)

        __,contours, hierarchy = cv2.findContours(output, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        for contour in contours:
            if (cv2.contourArea(contour) < 1200):
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
                # if cl[_] == 'b' and contour[i][0][1] >= half_y:
                try:
                    # point = (contour[i][0][0], contour[i][0][1])
                    # point[0]-=1
                    incontour =  cv2.pointPolygonTest(contour, (contour[i][0][0], contour[i][0][1]-1), False)
                except Exception as e:
                    print(e)
                    incontour = -10
                # print(incontour)
                if cl[_] == 'b' and incontour > 0:
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
        focal = 3.67
        h_sensor = 5.7
        v_sensor = img_w / img_h * h_sensor
        hfov = 78.0
        vfov = 42.0
        map = np.zeros((mapz,mapx*2,1),np.uint8)
        minx = 1000000
        minz = 1000000
        ztheta = ((vfov / 2) / (img_h / 2)) * ((img_h-2) - (img_h / 2)) + camera_ang
        nearens_z = (camera_h / np.sin((ztheta) / 180 * np.pi)) * np.sin((90 - ztheta) / 180 * np.pi)
        # print("nearens_z",nearens_z)
        closest_obstacle = [minx,minz]

        for i in range(0, len(list_x)):
            for seq in range(0, len(list_x[i])):
                # print(list_y[i][seq],img_h)
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
                
                if (z < minz):
                    minz = z
                    minx = x
                    closest_obstacle = [x,z]
                # elif (z == minz):
                #     if (x < minx):
                #         minx = x
            
                # print("x:", x, "z:", z)
                # print("--------")
                try:
                    if((mapz - int(z)) > 0):
                        map[mapz - int(z)][int(x) + mapx] = 255
                except:
                    pass
                # map = cv2.circle(map, (contour[i][0][0], contour[i][0][1]), 2, 255, 3)
                # print("x:", x, "z:", z)

    
    _resize = 0.3
    # print("closest_obstacle:",closest_obstacle)
    
    map = cv2.dilate(map, (np.ones((3, 3), np.uint8)), iterations=20)
    map = cv2.resize(map, (0,0), fx=_resize, fy = _resize)

    final_path = a_star(map)

    dir_len = int(mapz * _resize /3)


    global directionx
    global directiony
    global directionthe

    if(final_path == [] or (len(final_path) < dir_len+1)):
        # directionx = 0
        # directionz = -1
        # direction = -100000
        # direction = -90
        directionx = -1
        directiony = 0
        directionthe = 0
    else:
        normalization = np.sqrt((final_path[dir_len][1] - final_path[0][1])**2 + (final_path[dir_len][0] - final_path[0][0])**2)
        directionx = -(final_path[dir_len][0] - final_path[0][0])/normalization
        directiony = (final_path[dir_len][1] - final_path[0][1])/normalization
        directionthe = np.degrees(math.atan(directionx/directiony))
        if(directionthe >= 0):
            # print("r")
            directionthe = 90 - directionthe
        else:
            # print("l")
            directionthe = -90 - directionthe
    if(closest_obstacle[1] <= nearens_z+10 ):
        # print("qqq")
        directionx = -0.1
        if directiony > 0:
            directiony = 1
        elif directiony < 0:
            directiony = -1
        

        directiony *= (-1)
        # print(directionx,directiony,directionthe)
    chech_obs_point = map.shape[0]/5
    # if(map[map.shape[0] - 1 - chech_obs_point][int((map.shape[1] - 1) / 2)] == 255):
    #     directionx = 0
    #     if directiony > 0:
    #         directiony = 1
    #     elif directiony < 0:
    #         directiony = -1
            
    if directiony > 0:
        directiony = 1
    elif directiony < 0:
        directiony = -1

        # (map.shape[0] - 1 - chech_obs_point, int((map.shape[1] - 1) / 2))

    # print(directionx,directionz,directionthe)

    # update_plot(px, py)
    # rospy.Timer(rospy.Duration(0.001), lambda event: update_plot(px, py), oneshot=True)
    # print("final_path :",final_path)
    map_with_line = map.copy()
    
    for i in range(0, len(final_path)):
        map_with_line[final_path[i][0]][final_path[i][1]] = 128
    # map_with_line[map_with_line.shape[0] - 1 - chech_obs_point][int((map_with_line.shape[1] - 1) / 2)] = 128
    cv2.circle(map_with_line, (int((map_with_line.shape[1] - 1) / 2) , map_with_line.shape[0] - 1 - chech_obs_point), 1, 128,1)
    map_with_line = cv2.resize(map_with_line, (0,0), fx=8, fy = 8)

    cv2.imshow("Video Feed", oimg)
    cv2.imshow("map_with_line", map_with_line)
    # cv2.imshow("Mask", map)
    cv2.waitKey(1)

    

    # return(directionx,directionthe)

def head_angle(msg):
    global camera_ang
    camera_ang = msg.data
    # print("head_angle :", msg.data)

lower = np.array([[10, 90, 180], [90, 75, 0]])
upper = np.array([[30, 255, 255], [115, 255, 255]])
cl = ["y", "b"]



def main():
    plt.axis('square')
    plt.xlim(-mapx, mapx)
    plt.ylim(0, mapz)

    rospy.Subscriber("/cv_camera/image_raw", Image, create_map, queue_size=1)
    rospy.Subscriber("obstacle_head_angle", Int8, head_angle, queue_size=1)
    pub = rospy.Publisher('obstacle_vision_data', ThreeDouble, queue_size=1)

    rospy.init_node("benson_obstacle_vision")

    tickrate = 5
    rate = rospy.Rate(tickrate)


    # cap = cv2.VideoCapture(0)
    while not rospy.is_shutdown():
        # print(camera_ang)
        msg = ThreeDouble()

        msg.first = directionx
        msg.second = directiony
        msg.third = directionthe

        # rospy.loginfo("Publishing: first = %f, second = %f", msg.first, msg.second)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    main()