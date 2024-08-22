#!/usr/bin/env python
import rospy
import time
import cv2
import numpy as np

from op3_ros_utils import getWalkingParams, Robot
from copy import copy
from std_msgs.msg import Int32, String, Float32, Float64, Bool, Float32MultiArray




rospy.init_node("action_test")

cap=cv2.VideoCapture(0)
robot=Robot()

robot.setGeneralControlModule("walking_module")
rospy.sleep(1)

robot.setGeneralControlModule("action_module")
robot.setJointsControlModule(["head_pan","head_tilt"],["none","none"])

robot.setJointPos(["head_pan","head_tilt"],[-0.4,-0.5])
rospy.sleep(1)


upper1=np.array([145,200,255])
lower1=np.array([50,150,220])

upper2=np.array([130,160,255])
lower2=np.array([20,100,190])

upper3=np.array([65,130,185])
lower3=np.array([20,95,170])
        
while not rospy.is_shutdown():
	ret,frame = cap.read()
	
	mask1=cv2.inRange(frame,lower1,upper1)
	mask2=cv2.inRange(frame,lower2,upper2)
	mask3=cv2.inRange(frame,lower3,upper3)
	mask=cv2.bitwise_or(mask1,mask2)
	mask=cv2.bitwise_or(mask,mask3)

	kn = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
	mask = cv2.dilate(mask,kn,iterations=3)
	cv2.imshow("img",frame)
	cv2.imshow("img2",mask)
	cv2.waitKey(1)

	if robot.buttonCheck("user"):
		robot.walkStop()
		break

	

	