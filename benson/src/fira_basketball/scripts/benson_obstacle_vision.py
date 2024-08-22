#! /usr/bin/env python
import cv2
import time
import numpy as np
import matplotlib.pyplot as plt
import heapq
import math

incomputer = False
if(not incomputer):
    import rospy
    from std_msgs.msg import Float32 , Int8
    from sensor_msgs.msg import Image
    from tf.transformations import euler_from_quaternion
    from cv_bridge import CvBridge, CvBridgeError
    from fira_basketball.msg import ThreeDouble



DEBUG_MODE = True
# DEBUG_MODE = False # Show the detected image True
DEBUG_MARKER = False
DL_MODE = False 

args_red = ((np.array([0, 50, 0]), np.array([10, 255, 255])),
         (np.array([170, 50, 20]), np.array([179, 255, 255]))) #red

class States:
    INIT = -1
    READY = 0 # Waits for start button
    OBSTACLE = 1
    CROSS_DOOR = 2
    FIDIND_ARROW = 3
    FOLLOW_IMU = 4
    END = 99

camera_ang = 60
x_angle = 0
camera_h = 44.0  # 43 cm
focal = 3.67
h_sensor = 5.7
# v_sensor = img_w / img_h * h_sensor
hfov = 53.0
vfov = 38.0#41.2794
# hfov = 68.0
# vfov = 38.0
hfov_rad = np.radians(hfov)
vfov_rad = np.radians(vfov)
obs_circle_default = 16
obs_circle = obs_circle_default
_resize = 0.4
closest_obs_z = 1000000

have_path = 1


mapx = 120
mapz = 250

directionx = 0.0
directiony = 0.0
find_the_path = 1
# close_to_obsticle = False

fig, ax = plt.subplots()

pass_time = time.time()
    
def update_plot(px, py, color="red"):
    ax.clear()
    ax.set_xlim(-mapx, mapx)
    ax.set_ylim(0, mapz)
    ax.set_aspect('equal')
    ax.scatter(px, py, color=color, s=1)
    plt.draw()
    plt.pause(0.001)


def a_star(map):
    # return []
    start_point = 0
    start = (map.shape[0] - 1 - start_point, int((map.shape[1] - 1) / 2))
    # start = map.shape[0] - 1 - start_point
    goal = 0

    # goal = (0, int((map.shape[1] - 1) / 2))

    # start = (0, int((map.shape[1] - 1) / 2))
    # goal = map.shape[0] - 1
    # goal = (0, np.argmin(map[0]))
    # print("goal:", goal)
    # print("start:", start)

    def heuristic(node, goal):
        return (node[0] - goal)
    

    open_list = []
    heapq.heappush(open_list, (0, start))

    came_from = np.full((map.shape[0], map.shape[1], 2), -1, dtype=int)
    # start = heapq.heappop(open_list)[1]
    # print("start:",start)
    # print("map.shape:",map.shape)
    g_score = np.full(map.shape, float('inf'), dtype=np.float64)
    g_score[start] = 0
    f_score = np.full(map.shape, float('inf'), dtype=np.float64)
    f_score[start] = heuristic(start, goal)

    while open_list:
        current = heapq.heappop(open_list)[1]

        if current[0] == goal:
            start_point = (map.shape[0]-1,int((map.shape[1] - 1)/2))
            end_point = (0, int((map.shape[1] - 1) / 2))
            # print("start_point",start_point[0],start_point[1])
            final_path = []
            # final_path.append(start_point)
            # final_path.append(end_point)
            while current != start:
                final_path.append(current)
                current = tuple(came_from[current[0], current[1]])
            final_path.append(start)
            final_path.reverse()
            # print("final_path:",final_path)
            return final_path

        orderx = [ -1 ,0 , 1]
        ordery = [ -2 ,0 , 2]
        # for x in range(current[0] - 1, current[0] + 2):
        #     for y in range(current[1] - 1, current[1] + 2):

        for i in orderx:
            for j in ordery:
                x = current[0] + i
                y = current[1] + j
                if (x == current[0] and y == current[1]) or x < 0 or y < 0 or x >= map.shape[0] or y >= map.shape[1] or map[x, y] != 0:
                    continue
                
                tentative_g_score = g_score[current] + np.sqrt(((x - current[0]) ** 2) + (y - current[1]) ** 2)

                if tentative_g_score < g_score[x, y]:
                    came_from[x, y] = current
                    g_score[x, y] = tentative_g_score
                    f_score[x, y] = tentative_g_score + heuristic((x, y), goal)
                    heapq.heappush(open_list, (f_score[x, y], (x, y)))

    # print("No path found")
    return []

if(incomputer):
    def empty(v):
        pass
    cv2.namedWindow("track_bar")
    cv2.resizeWindow("track_bar", 640,320)
    cv2.createTrackbar("x_angle", "track_bar",   0 , 360, empty)
    cv2.createTrackbar("z_angle", "track_bar",   0 , 360, empty)
    cv2.createTrackbar("hfov", "track_bar",   0 , 100, empty)
    cv2.createTrackbar("vfov", "track_bar",   0 , 100, empty)
    cv2.setTrackbarPos("x_angle", "track_bar", 180)
    cv2.setTrackbarPos("z_angle", "track_bar", 180+int(camera_ang))
    cv2.setTrackbarPos("hfov", "track_bar", int(hfov))
    cv2.setTrackbarPos("vfov", "track_bar", int(vfov))
    cap = cv2.VideoCapture(0)
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

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

def create_map(data):
    if(incomputer):
        try:
            ret, oimg = cap.read()
            # oimg = cv2.resize(oimg, (640, 480), interpolation=cv2.INTER_AREA)
        except CvBridgeError as e:
            print("no pic:", e)        
    else:
        bridge = CvBridge()
        try:
            # Convert the ROS Image message to OpenCV format
            oimg = bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))



    # cv2.imshow("oimg", oimg)
    # cv2.waitKey(1)
    # obs_circle = 2
    global obs_circle
    # global camera_ang
    # print(have_path)
    oimg = cv2.GaussianBlur(oimg, (5, 5), 0)
    # top, bottom, left, right = 1, 1, 1, 1
    # oimg = cv2.copyMakeBorder(oimg, top, bottom, left, right, cv2.BORDER_CONSTANT, value=[128, 250, 250])
    top, bottom, left, right = 1, 1, 1, 1
    oimg = cv2.copyMakeBorder(oimg, top, bottom, left, right, cv2.BORDER_CONSTANT, value=[0, 0, 0])
    img_w = oimg.shape[1]
    img_h = oimg.shape[0]
    
    # print(img_w, img_h)

    hsv_img = cv2.cvtColor(oimg, cv2.COLOR_BGR2HSV)
    
    px = []
    py = []
    list_x = []
    list_y = []
    list_x_edge = []
    list_y_edge = []
    px_edge = []
    py_edge = []
    stapx = []
    stapy = []
    
        
    
    list_x_edge += [[0]]
    list_y_edge += [[img_h]]
    
    list_x_edge += [[0]]
    list_y_edge += [[0]]

    list_x_edge += [[img_w]]
    list_y_edge += [[0]]

    list_x_edge += [[img_w]]
    list_y_edge += [[img_h]]

    
    # map = np.zeros((img_h,img_w,1),np.uint8)
    for _ in range(0, len(lower)):

        output = cv2.inRange(hsv_img, lower[_], upper[_])
        output = cv2.Laplacian(output, -1, 1, 1)
        # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 1))
        # output = cv2.erode(output, kernel, iterations = 1)
        # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        ret, output = cv2.threshold(output, 30, 255, cv2.THRESH_BINARY)

        # cv2.imshow(str(cl[_]), output)
        cv2.waitKey(1)

        if(not incomputer):
            __,contours, hierarchy = cv2.findContours(output, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        else:
            contours, hierarchy = cv2.findContours(output, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            
        

        for contour in contours:
            if (cv2.contourArea(contour) < 500):
                continue
            max_y = max(point[0][1] for point in contour)
            min_y = min(point[0][1] for point in contour)
            half_y = (max_y + min_y) // 2
            step = 1
            for i in range(0, len(contour), 10):
                # print(i)
                if cl[_] == 'y':
                        # if slop > 0.1:# and contour[i][0][1] < img_h / 2:
                    # print("y")
                    stapx += [contour[i][0][0]]
                    stapy += [contour[i][0][1]]
                    # try:
                    #     map[contour[i][0][1]][contour[i][0][0]] = 255
                    # except:
                    #     pass
                    # map = cv2.circle(map, (contour[i][0][0], contour[i][0][1]), 2, 255, 3)
                if (contour[i - 1][0][0] - contour[i][0][0]) == 0:
                    # print("div by zero")
                    continue
                slop = abs((contour[i - 1][0][1] - contour[i][0][1]) / (contour[i - 1][0][0] - contour[i][0][0]))
                # print(contour[i - 5][0][1] - contour[i][0][1])
                # print(contour[i - 5][0][0] - contour[i][0][0])
                # print("slop:",slop)
                # if cl[_] == 'b' and contour[i][0][1] >= half_y:
                try:
                    # point = (contour[i][0][0], contour[i][0][1])
                    # point[0]-=1
                    # pass
                    incontour =  cv2.pointPolygonTest(contour, (int(contour[i][0][0]), int(contour[i][0][1]-1)), False)
                    # contour = contour.reshape((-1, 1, 2))
                    # point = (int(contour[i][0][0]),int(contour[i][0][1] - 1))

                    # print("point: ",point)
                    # print("contour: ",contour)
                    # # print("point: ",point)
                    # incontour = cv2.pointPolygonTest(contour, point, False)
                except Exception as e:
                    print(e)
                    incontour = -10
                # print(incontour)
                if cl[_] == 'b' and incontour > 0:
                    # print("b")
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
            # print("stapx,stapy:",stapx,stapy)

        """draw point"""
        color = (255, 255, 0)
        for x in range(0, len(list_x)):
            for seq in range(0, len(list_x[x])):
                if list_x[x][seq] < 100000:
                    oimg = cv2.circle(oimg, (list_x[x][seq], list_y[x][seq]), 2, color, 3)
                # cv2.circle(map, ((list_x[x][seq], list_y[x][seq])), obs_circle, 255,-1)
                # map = cv2.circle(map, (list_x[x][seq], list_y[x][seq]), 2, 255, 3)

        # ztheta = camera_ang - vfov/2
        # z = (camera_h / np.sin((ztheta) / 180 * np.pi)) * np.sin((90 - ztheta) / 180 * np.pi)
        hfov_rad = np.radians(hfov)
        vfov_rad = np.radians(vfov)
        
        
        
        ztheta = np.degrees(np.arctan((0-(img_h/2))*np.tan(np.radians(vfov/2))/(img_h/2))) + camera_ang
        z = (camera_h / np.sin((ztheta) / 180 * np.pi)) * np.sin((90 - ztheta) / 180 * np.pi)
        
        
        rotate_ang = np.radians(abs(camera_ang-ztheta))
        rotate_z = z/np.cos(np.radians(ztheta))
        hfov_rotate = 2*np.arcsin( np.tan(hfov_rad/2) /       np.sqrt( np.tan(rotate_ang)**2     +   (1/(np.cos(hfov_rad/2)))**2 ) )
        hfov_rotate = np.degrees(hfov_rotate)

        xtheta = np.degrees(np.arctan((img_w-(img_w/2))*np.tan(np.radians(hfov_rotate/2))/(img_w/2)))
        x = rotate_z * np.tan(np.radians(xtheta))

        x_rotate = rotate_point((x, z), (0, 0), -abs(x_angle))
        x = max(x,x_rotate[0])
        z = max(z,x_rotate[1])




        global mapx
        global mapz
        try:
            mapx = min(abs(int(x-5)),300)
            mapz = min(abs(int(z-2)),450)
            # mapx = 300
            # mapz = 450
        except:
            mapx = 100
            mapz = 80
        map = np.zeros((mapz,mapx*2,1),np.uint8)
        closest_obs_z = 1000000
        hfov_rad = np.radians(hfov)
        vfov_rad = np.radians(vfov)

        for i in range(0, len(list_x)):
            for seq in range(0, len(list_x[i])):
                ztheta = np.degrees(np.arctan((list_y[i][seq]-(img_h/2))*np.tan(np.radians(vfov/2))/(img_h/2))) + camera_ang
                z = (camera_h / np.sin((ztheta) / 180 * np.pi)) * np.sin((90 - ztheta) / 180 * np.pi)
                
                
                rotate_ang = np.radians(abs(camera_ang-ztheta))
                rotate_z = z/np.cos(np.radians(ztheta))
                hfov_rotate = 2*np.arcsin( np.tan(hfov_rad/2) /       np.sqrt( np.tan(rotate_ang)**2     +   (1/(np.cos(hfov_rad/2)))**2 ) )
                hfov_rotate = np.degrees(hfov_rotate)

                xtheta = np.degrees(np.arctan((list_x[i][seq]-(img_w/2))*np.tan(np.radians(hfov_rotate/2))/(img_w/2)))
                x = rotate_z * np.tan(np.radians(xtheta))
                # x = (rotate_z / np.sin((180 - 90 - xtheta) / 180 * np.pi)) * np.sin(xtheta / 180 * np.pi)

                x_rotate = rotate_point((x, z), (0, 0), -x_angle)
                # cv2.circle(map, (mapx, mapz), obs_circle, 255,-1)
                x = x_rotate[0]
                z = x_rotate[1]

                px += [x]
                py += [z]

                if z<closest_obs_z:
                    closest_obs_z = z

                try:
                    if((mapz - int(z)) > 0):
                        # cv2.circle(map, ((int(x) + mapx) , (mapz - int(z))), obs_circle, 128,-1)
                        # if()
                        cv2.circle(map
, ((int(x) + mapx) , (mapz - int(z))), obs_circle, 255,-1)
                        # map[mapz - int(z)][int(x) + mapx] = 255
                except:
                    pass
                # map = cv2.circle(map, (contour[i][0][0], contour[i][0][1]), 2, 255, 3)
                # print("x:", x, "z:", z)

    for i in range(0, len(list_x_edge)):
        for seq in range(0, len(list_x_edge[i])):
            ztheta = np.degrees(np.arctan((list_y_edge[i][seq]-(img_h/2))*np.tan(np.radians(vfov/2))/(img_h/2))) + camera_ang
            z = (camera_h / np.sin((ztheta) / 180 * np.pi)) * np.sin((90 - ztheta) / 180 * np.pi)
            
            
            rotate_ang = np.radians(abs(camera_ang-ztheta))
            rotate_z = z/np.cos(np.radians(ztheta))
            hfov_rotate = 2*np.arcsin( np.tan(hfov_rad/2) /       np.sqrt( np.tan(rotate_ang)**2     +   (1/(np.cos(hfov_rad/2)))**2 ) )
            hfov_rotate = np.degrees(hfov_rotate)

            xtheta = np.degrees(np.arctan((list_x_edge[i][seq]-(img_w/2))*np.tan(np.radians(hfov_rotate/2))/(img_w/2)))
            x = rotate_z * np.tan(np.radians(xtheta))
            # x = (rotate_z / np.sin((180 - 90 - xtheta) / 180 * np.pi)) * np.sin(xtheta / 180 * np.pi)
            px += [x]
            py += [z]
            px_edge += [x]
            py_edge += [z]

            try:
                if((mapz - int(z)) > 0):
                    pass
                    # cv2.circle(map, ((int(x) + mapx) , (mapz - int(z))), obs_circle, 128,-1)
                    # if()
                    # cv2.circle(map, ((int(x) + mapx) , (mapz - int(z))), 5, 255,-1)
                    # map[mapz - int(z)][int(x) + mapx] = 255
            except:
                pass
    # print("px_edge:",px_edge)

    # x_rotate = rotate_point((px_edge[0], py_edge[0]), (0, 0), abs(x_angle))
    # x0 = x_rotate[0]
    # z0 = x_rotate[1]

    # x_rotate = rotate_point((px_edge[1], py_edge[1]), (0, 0), abs(x_angle))
    # x1 = x_rotate[0]
    # z1 = x_rotate[1]

    # x_rotate = rotate_point((px_edge[2], py_edge[2]), (0, 0), -abs(x_angle))
    # x2 = x_rotate[0]
    # z2 = x_rotate[1]

    # x_rotate = rotate_point((px_edge[3], py_edge[3]), (0, 0), -abs(x_angle))
    # x3 = x_rotate[0]
    # z3 = x_rotate[1]
    # cv2.line(map, ((int(x0) + mapx) , (mapz - int(z0))) ,((int(x1) + mapx) , (mapz - int(z1))), 255, 2)
    # cv2.line(map, ((int(x2) + mapx) , (mapz - int(z2))) ,((int(x3) + mapx) , (mapz - int(z3))), 255, 2)

    
    
    # print("closest_obstacle:",closest_obstacle)
    
    
    map = cv2.resize(map, (0,0), fx=_resize, fy = _resize)
    top, bottom, left, right = 1, 1, 0, 0
    map = cv2.copyMakeBorder(map, top, bottom, left, right, cv2.BORDER_CONSTANT, value=[0, 0, 0])

    
    final_path = a_star(map)
    

    # dir_len = int(mapz * _resize /5)
    dir_len = int(len(final_path) / 3)
    # print("dir_len",dir_len)
    # start_point = (map.shape[0]-1,int(map.shape[1]/2))
    # print("start_point",start_point[0],start_point[1])


    global directionx
    global directiony
    global find_the_path

    if(final_path == []):
        if(closest_obs_z < 25):
            print("no path")
            directionx = -1
            directiony = 0
            find_the_path = 0
        else:
            print("no path but try to go forward")
            directionx = 0.5

            directiony = 0
            find_the_path = 0.3
    else:
        # final_path.reverse()
        # endlen = len(final_path)
        normalization = np.sqrt((final_path[dir_len][1] - final_path[0][1])**2 + (final_path[dir_len][0] - final_path[0][0])**2)
        directionx = -(final_path[dir_len][0] - final_path[0][0])/normalization
        directiony = -(final_path[dir_len][1] - final_path[0][1])/normalization
        # find_the_path = np.degrees(math.atan(directionx/directiony))
        # print("have path")
        # normalization = np.sqrt((final_path[len(final_path)-1][1] - final_path[0][1])**2 + (final_path[len(final_path)-1][0] - final_path[0][0])**2)
        # directionx = -(final_path[len(final_path)-1][0] - final_path[0][0])/normalization
        # directiony = -(final_path[len(final_path)-1][1] - final_path[0][1])/normalization
        # print(final_path[len(final_path)-1][0],final_path[0][0],final_path[len(final_path)-1][1],final_path[0][1])
        find_the_path = 1.0
    # if(closest_obstacle[1] <= nearens_z+10 ):
    #     # print("qqq")
    #     directionx = -0.1
        if directiony > 0.2:
            directiony = 1
            directionx = -0.08
        elif directiony < -0.2:
            directiony = -1
            directionx = -0.08
        

        # directiony *= (-1)
        
    chech_obs_point = map.shape[0]/5

    print("dir: ",directionx,directiony,find_the_path)

    # print(directionx,directionz,find_the_path)

    # update_plot(px, py)
    # print("final_path:",final_path)
    # print("len(final_path):",len(final_path))
    map_with_line = map.copy()
    
    for i in range(0, len(final_path)):
        map_with_line[final_path[i][0]][final_path[i][1]] = 128
    # map_with_line[map_with_line.shape[0] - 1 - chech_obs_point][int((map_with_line.shape[1] - 1) / 2)] = 128
    # cv2.circle(map_with_line, (int((map_with_line.shape[1] - 1) / 2) , map_with_line.shape[0] - 1 - chech_obs_point), 1, 128,1)
    size_show = 5/_resize
    map_with_line = cv2.resize(map_with_line, (0,0), fx=size_show, fy = size_show)

    cv2.imshow("Video Feed", oimg)
    cv2.imshow("map_with_line", map_with_line)
    # cv2.imshow("Mask", map)
    cv2.waitKey(1)

        

        # return(directionx,find_the_path)

def head_angle(msg):
    global camera_ang
    camera_ang = msg.data
    # print("head_angle :", msg.data)

lower = np.array([[8, 0, 150], [100, 100, 0]])
upper = np.array([[51, 255, 255], [115, 255, 255]])
cl = ["y", "b"]



def main():
    plt.axis('square')
    plt.xlim(-mapx, mapx)
    plt.ylim(0, mapz)

    if(not incomputer):

        rospy.Subscriber("/cv_camera/image_raw", Image, create_map, queue_size=1)
        rospy.Subscriber("obstacle_head_angle", Int8, head_angle, queue_size=1)
        pub = rospy.Publisher('obstacle_vision_data', ThreeDouble, queue_size=1)

        rospy.init_node("benson_obstacle_vision")

        tickrate = 2
        rate = rospy.Rate(tickrate)


    # cap = cv2.VideoCapture(0)
    if(not incomputer):
        while not rospy.is_shutdown():
            # print(camera_ang)
            msg = ThreeDouble()

            msg.first = directionx
            msg.second = directiony
            msg.third = find_the_path

            # rospy.loginfo("Publishing: first = %f, second = %f", msg.first, msg.second)
            pub.publish(msg)
            rate.sleep()
    else:
        while True:
            create_map(0)
            global camera_ang
            global x_angle
            global hfov
            global vfov
            global hfov_rad
            global vfov_rad
            x_angle = cv2.getTrackbarPos("x_angle", "track_bar")-180
            camera_ang = cv2.getTrackbarPos("z_angle", "track_bar")-180
            hfov = cv2.getTrackbarPos("hfov", "track_bar")
            vfov = cv2.getTrackbarPos("vfov", "track_bar")
            hfov_rad = np.radians(hfov)
            vfov_rad = np.radians(vfov)
            # print("test")
            pass

if __name__ == '__main__':
    main()