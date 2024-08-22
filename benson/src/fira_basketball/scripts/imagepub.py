#! /usr/bin/python

import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import sys

def cvcamImagePub():

    #init ros_node
    rospy.init_node("fira_weightlifting_test")
    img_pub = rospy.Publisher("/cv_camera/image_raw", Image, queue_size=2)
    rate = rospy.Rate(5)#5hz

    cap = cv2.VideoCapture(0)

    scaling_factor = 0.5

    bridge = CvBridge()

    if not cap.isOpened():
        sys.stdout.write("Cam is not available!")
        return -1

    count = 0
    #loop until press 'esc' or 'q'
    while not rospy.is_shutdown():
        ret, frame = cap.read()

        if ret:
            count = count + 1
        else:
            rospy.loginfo("Capturing image failed.")
        if count == 2:
            count = 0
            frame = cv2.resize(frame,None, fx=scaling_factor,fy=scaling_factor,interpolation=cv2.INTER_AREA)
            msg = bridge.cv2_to_compressed_imgmsg(frame, encoding="bgr8")
            img_pub.publish(msg)
            print '**publishing cvcam_frame**'
        rate.sleep()
if __name__ == '__main__':
    try:
        cvcamImagePub()
    except rospy.ROSInternalException:
        pass


    

