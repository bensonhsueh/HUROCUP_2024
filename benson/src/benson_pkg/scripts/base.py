#!/usr/bin/env python
import rospy
import time
import numpy as np
from op3_ros_utils import getWalkingParams, Robot
from copy import copy
from std_msgs.msg import Int32, String, Float32, Float64, Bool, Float32MultiArray

rospy.init_node("action_test")

robot=Robot()

"""
robot.setGeneralControlModule("action_module")
robot.setGeneralControlModule("walking_module")

robot.walkVelocities(x=10, th=0, balance=True)
robot.walkStart()
robot.walkStop()


#robot.playMotion(7, wait_for_end=True)
#rospy.spin()


robot.setGeneralControlModule("walking_module")
rospy.sleep(5)
robot.setGeneralControlModule("action_module")
robot.setJointsControlModule(["head_pan"],["none"])

robot.setJointPos(["head_pan"],[pos])
	
"""

robot.setGeneralControlModule("walking_module")

while not rospy.is_shutdown():

	robot.walkVelocities(x=-3, th=0, balance=True )
	robot.walkStart()

	if robot.buttonCheck("user"):
		robot.walkStop()
		break

	

	
	

