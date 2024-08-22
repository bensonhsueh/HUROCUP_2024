#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np

class Nodo(object):
    def __init__(self):
        # Params
        self.image = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)
                    
        self.cap = cv2.VideoCapture(0)

        # Publishers
        self.pub = rospy.Publisher('Robot_Vision', Image,queue_size=10)

        # Subscribers
        rospy.Subscriber("/camera/image_color",Image,self.callback)

    def callback(self, msg):
        rospy.loginfo('Image received...')
        self.image = self.br.imgmsg_to_cv2(msg)


    def start(self):
        rospy.loginfo("Timing images")
        #rospy.spin()
        while not rospy.is_shutdown():
            # rospy.loginfo('publishing image')
            #br = CvBridge()
            _,img = self.cap.read()
            img = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
            self.image = cv2.resize(img,(80,60))
            if self.image is not None:
                self.pub.publish(self.br.cv2_to_imgmsg(self.image,encoding='8UC1'))
            # self.loop_rate.sleep()



if __name__ == '__main__':
    rospy.init_node("Robot_Eye", anonymous=True)
    my_node = Nodo()
    my_node.start()