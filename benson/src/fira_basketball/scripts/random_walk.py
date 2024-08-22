#! /usr/bin/env python
import rospy
from op3_ros_utils import *
# import cv2
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError
# import time
import random
rospy.init_node("move")



robot = Robot()
# rospy.sleep(3)

def init():
    # Set ctrl modules of all actions to joint, so we can reset robot position
    # robot.setGeneralControlModule("action_module")
    # robot.setGeneralControlModule("action_module")

    # robot.setGrippersPos(left=0.0, right=0.0)

    # Call initial robot position
    # robot.playMotion(1, wait_for_end=True)
    robot.setJointsControlModule(["head_tilt"], ["none"])
    robot.setGeneralControlModule("walking_module")

init()


     

    
# rospy.sleep(3)    
        # theta = float(raw_input("theta"))


while not rospy.is_shutdown():

    # go = raw_input("Go")

    # rospy.sleep(3)   
    # if go == "y":


        speed = 15
        # theta = random.uniform(1.0,3.5)

        # speed = raw_input("Speed")
        theta = raw_input("Theta")
        # theta = 3.0
        # theta = random.uniform(1.0,3.0)
        print(theta)
        walk_time = 3
        robot.walkVelocities(x=int(-speed), th=float(theta), balance=True, hip_pitch=4.0) #36

        robot.walkStart()
        rospy.sleep(walk_time)
        robot.walkStop()
        # rospy.sleep(3)

    # else:

    #     break

print("STOP")