#! /usr/bin/env python
import numpy as np
import math
import cv2
from cv2 import aruco
import glob
from time import time

import rospy
import rospkg
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int16, Float32

if __name__ =='__main__':

    rospy.init_node('aruco_tracker')
    rospy.loginfo('aruco_tracker activated ! ')
    
    pub_position  = rospy.Publisher('/get_aruco_position' , Pose2D,  queue_size = 1)
    pub_size      = rospy.Publisher('/get_aruco_size'     , Int16 ,  queue_size = 1)
    cap = cv2.VideoCapture('/dev/video0')
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH , 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    
    video_fns = glob.glob("*.avi")
    video_id = len(video_fns)
    video_name = "sprint_extra_" + str(video_id) + ".avi"

    out_video = cv2.VideoWriter(video_name, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'),
                                30, (320, 240))

    start_time = time()

    position  = Pose2D()
    size      = -1
    
    lower_red = np.array([156,43,46]) 
    upper_red = np.array([180, 255, 255])
    
    rospy.loginfo("YES I AM RUNNINGINGINGINIG")

    ###------------------ ARUCO TRACKER ---------------------------
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        # operations on the frame
        if ret :
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower_red, upper_red)
            moments = cv2.moments(mask)
            m00 = moments['m00']
            if m00 != 0:
                position.x = int(moments['m10']/m00)#Take X coordinate
                position.y = int(moments['m01']/m00)#
            else:
                position.x = -1
                position.y = -1
                size       = -1
            pub_position.publish(position)
            pub_size.publish(size)
        else:
            position.x = -1
            position.y = -1
            size       = -1
        pub_position.publish(position)
        pub_size.publish(size)
            
        # display the resulting frame
        # cv2.circle(frame, (position.x, position.y), 5, (0, 255, 0), -1)
        # cv2.imshow('frame',frame)

        elapsed = time() - start_time
        if elapsed > 5:
            rospy.loginfo("RECORDING!!!! {0:.1f} seconds have passed!".format(elapsed))
            out_video.write(frame)
        if elapsed > 35:
            break
            
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
    rospy.loginfo('aruco tracker terminated ! ')
