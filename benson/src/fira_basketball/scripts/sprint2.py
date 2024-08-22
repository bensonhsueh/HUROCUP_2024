#! /usr/bin/env python

import rospy
from op3_ros_utils import *

rospy.init_node("sprint")

robot = Robot()
rospy.sleep(3)

def init():
    # Set ctrl modules of all actions to joint, so we can reset robot position
    robot.setGeneralControlModule("action_module")

    #robot.setGrippersPos(left=60.0, right=60.0)

    # Call initial robot position
    robot.playMotion(1, wait_for_end=True)

    # Set ctrl module to walking, this actually only sets the legs
    rospy.sleep(1.0)


init()

robot.setGeneralControlModule("walking_module")
raw_input()
rospy.sleep(3)

robot.walkVelocities(x=2.0, y=0.0, th=-5, z_move_amplitude=0.045, balance=False,
    z_offset=0.035)
robot.walkStart()
rospy.sleep(4.2)
robot.walkStop()

rospy.sleep(3)

#robot.walkVelocities(x=3.0, y=0.0, th=0, z_move_amplitude=0.045, balance=False,
#    z_offset=0.035)
#robot.walkStart()
#rospy.sleep(4)
#robot.walkStop()

robot.setGeneralControlModule("action_module")
robot.playMotion(1, wait_for_end=True)
rospy.sleep(0.25)
robot.playMotion(88, wait_for_end=True)
#robot.setGrippersPos(left=0.0, right=0.0)
rospy.sleep(0.25)
robot.playMotion(89, wait_for_end=True)

rospy.spin()


