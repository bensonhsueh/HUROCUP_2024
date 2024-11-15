#!/usr/bin/env python3

import cv2
import rospy
import yaml
import numpy as np

from cv2 import aruco
from sensor_msgs.msg import Image,CompressedImage
from robot_msgs.msg import ArucoData 
# from cv_bridge import CvBridge, CvBridgeError

class Vision:
    def __init__(self):
        rospy.init_node('sprint_aruco')
        rospy.loginfo("[Vision] Sprint Aruco - Running")

        self.python2 = False
        if self.python2:
            self.bridge = CvBridge()

        self.frame_w      = 640 #rospy.get_param("/usb_cam/image_width")
        self.frame_h      = 480 #rospy.get_param("/usb_cam/image_height")
        # self.frame_w      = rospy.get_param("/uvc_webcam/width")
        # self.frame_h      = rospy.get_param("/uvc_webcam/height")
        self.source_img   = np.zeros((self.frame_w, self.frame_h, 3), np.uint8)
        self.aruco_dict   = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = aruco.DetectorParameters_create()
        self.aruco_data   = ArucoData()
        self.debug        = False
        self.Info         = True
        self.camera       = cv2.VideoCapture('/dev/video0')
        # Load calibration.yaml
        # with open('/home/robotis/joe_ws/src/sprint_aruco_detector/calibration/calibration.yaml') as f:
        
        with open('/home/robotis/Kulin_ws/Other Codes/calibration.yaml') as f:
            loadeddict = yaml.load(f,Loader=yaml.FullLoader)
        self.mtx = np.array(loadeddict.get('camera_matrix'))
        self.dist = np.array(loadeddict.get('dist_coeff'))
        
        # Subscriber
        rospy.Subscriber("/cv_camera/image_raw", Image, self.img_callback)
        # rospy.Subscriber("/image_raw", Image, self.img_callback)

        # Publisher
        self.aruco_pos_pub = rospy.Publisher("/sprint/marker/position", ArucoData, queue_size=1)
        # self.aruco_img_pub = rospy.Publisher("/sprint/marker/image",    Image,     queue_size=1)

    def img_callback(self, msg):
        if self.python2:
            try:
                self.source_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError as e:
                rospy.loginfo(e)
        else:
            dt  = np.dtype(np.uint8)
            dt  = dt.newbyteorder('>')
            arr = np.frombuffer(msg.data, dtype=dt)

            arr = np.reshape(arr, (self.frame_h, self.frame_w ,3))
            self.source_img = arr#cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)

    def kill_node(self):
        cv2.destroyAllWindows()
        rospy.signal_shutdown("[Vision] Shutdown Time...") 

    def run(self):
        count = 0
        ret,_ = self.camera.read()
        # print(ret)
        #while not rospy.is_shutdown():
        print(ret)
        while ret:
            
            ret,rgb_img = self.camera.read()
            #rgb_img    = self.source_img.copy()
            
            # cv2.imshow('test',frame)
            # cv2.waitKey(1)
            blur_img   = cv2.GaussianBlur(rgb_img, (5,5), 0)

            gray_img   = cv2.cvtColor(blur_img, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_img, self.aruco_dict, parameters=self.aruco_params)
            cx, cy, size = -1, -1, -1
            # print(ids)

            # if len(corners) > 0 and ids[0,0] == 1:

            if np.all(ids != None):
                for i in range(0, ids.size):
    
                    if ids[i,0] == 0:
                        M = cv2.moments(corners[i])
                        cx   = int(M["m10"] / M["m00"])
                        cy   = int(M["m01"] / M["m00"])
                        _, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 20, self.mtx, self.dist)
                        size = int(M["m00"])
                        
                        # distance
                        distance = int(tvec[0][0][2])
                        
                        if self.debug:
                            rospy.loginfo("[Vision] ID : {}, Pos: {},{}".format(ids[0,0], cx, cy))

                        cv2.circle(rgb_img, (cx,cy), 5, (0, 0, 255), -1)

                        aruco.drawDetectedMarkers(rgb_img, corners, ids)
                        break
            else:
                distance = False
            
            count += 1
            
            # arcuo coordinates
            self.aruco_data.x = cx
            self.aruco_data.y = cy
            if distance:
                self.aruco_data.size = distance
                if self.Info and (count % 50) == 0:
                    rospy.loginfo("[Vision] Distance : {}".format(distance))
            else:
                self.aruco_data.size = -1
            # publishing aruco result
            self.aruco_pos_pub.publish(self.aruco_data)
            # self.aruco_img_pub.publish(self.bridge.cv2_to_imgmsg(rgb_img, "bgr8"))

            k = cv2.waitKey(1)
            
            # print(gray_img.shape)
            cv2.imshow('image', rgb_img)
            cv2.waitKey(1)  
            if k == 27 or k == ord('q'):
                break
        
        rospy.on_shutdown(self.kill_node)

if __name__ == '__main__':
    print('start')
    vision = Vision()
    vision.run()
