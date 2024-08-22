#! /usr/bin/env python
import cv2
import numpy as np
import rospy
from std_msgs.msg import Float32 , Int8
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge, CvBridgeError
from fira_basketball.msg import ThreeDouble








def show_img(data):

    bridge = CvBridge()
    try:
        # Convert the ROS Image message to OpenCV format
        oimg = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    # print("ss")



    cv2.imshow("oimg", oimg)
    cv2.waitKey(1)






def main():

    rospy.Subscriber("/cv_camera/image_raw", Image, show_img, queue_size=1)

    rospy.init_node("benson_vision")

    tickrate = 60
    rate = rospy.Rate(tickrate)

    while not rospy.is_shutdown():
        # print(camera_ang)

        rate.sleep()


if __name__ == '__main__':
    main()