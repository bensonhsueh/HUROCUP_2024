#!/usr/bin/env python
import numpy as np
import cv2 as cv
from cv2 import aruco
import rospy
import roslib
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
global rgb_img
# rgb_img = np.zeros((240, 320, 3), np.uint8)
#rgb_img = np.zeros((rospy.get_param("/usb_cam/image_height"), rospy.get_param("/usb_cam/image_width"), 3), np.uint8)
rgb_img = np.zeros((480, 640, 3), np.uint8)


# file_name = 'sprint.avi'

def img_sub_callback(img_msg):
    try:
        global rgb_img
        rgb_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        rospy.loginfo(e)

def kill_node():
    rospy.signal_shutdown("shutdown time.") 

def main():
    rospy.loginfo("Sprint ArUco Detector - Running")
    rospy.init_node("sprint_aruco_detector")
    aruco_img_sub = rospy.Subscriber("/cv_camera/image_raw", Image, img_sub_callback)
    # global aruco_img_pub, aruco_pos_pub
    aruco_img_pub = rospy.Publisher("/sprint/marker/image", Image, queue_size=1)
    aruco_pos_pub = rospy.Publisher("/sprint/marker/position", Int32MultiArray, queue_size=1)
    rate = rospy.Rate(50)
    # writer = cv.VideoWriter(file_name, cv.VideoWriter_fourcc('M','J','P','G'), 60, (640,480))
    while not rospy.is_shutdown():
        global rgb_img
        result_img = rgb_img.copy()
        cv.imshow('result', result_img)
        
        gray_img = cv.cvtColor(rgb_img, cv.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        parameters =  aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_img, aruco_dict, parameters=parameters)
        cx = -1
        cy = -1
        size = -1
        
        if len(corners) > 0 and ids[0,0] == 0:
            M = cv.moments(corners[0])
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            size = int(M["m00"])
            print("ID :", ids[0,0], "Pos :",cx,cy)
            cv.circle(result_img,(cx,cy), 45, (0,0,255), 3)
        
        marker_pos = Int32MultiArray()
        marker_pos.data = [cx, cy, size]
        
        # writer.write(rgb_img)
        result_img = aruco.drawDetectedMarkers(result_img, corners, ids)
        cv.imshow('result_img', result_img)
        print('number of corners:', len(corners))
        cv.waitKey(1)
        # rospy.loginfo("ArUco Marker : \n" + str(marker_pos))
        #global aruco_img_pub, aruco_pos_pub
        aruco_img_pub.publish(bridge.cv2_to_imgmsg(result_img, "bgr8"))
        aruco_pos_pub.publish(marker_pos)
        # rate.sleep()

    rospy.loginfo("Sprint ArUco Detector - Shut Down")
    rospy.on_shutdown(kill_node)

if __name__ == "__main__":
    main()
