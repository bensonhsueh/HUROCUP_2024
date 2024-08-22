#! /usr/bin/env python
import rospy
from op3_ros_utils import *
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
from robot_msgs.msg import Button

rospy.init_node("move")
robot = Robot()
rospy.sleep(3)
# button
middle_button = { 'last': False, 'current': False }
right_button  = { 'last': False, 'current': False }

def init():
    # Set ctrl modules of all actions to joint, so we can reset robot position
    # robot.setGeneralControlModule("action_module")
    robot.setGeneralControlModule("action_module")

    # robot.setGrippersPos(left=0.0, right=0.0)

    # Call initial robot position
    robot.playMotion(1, wait_for_end=True)
    robot.setJointsControlModule(["head_tilt"], ["none"])
    robot.setGeneralControlModule("walking_module")
    rospy.Subscriber("/robotis/cm_740/button/", button_callback)


def button_callback(msg):
'''	
	middle_button['current'] = msg.middle
	right_button['current']  = msg.right

	# detect button transition
	# middle button
	if self.middle_button['last'] != self.middle_button['current']:
		print('middle button presssed')
'''
		if self.middle_button['current']:
			if self.rocknroll:
				rospy.loginfo("[Player] Sit down")
				self.motion_pub.publish("sit")
				self.state     = None
				self.rocknroll = False
			else:
				rospy.loginfo("[Player] Standup")
				self.motion_pub.publish("stand")
				self.state	   = 'initial'
				self.rocknroll = True
'''
	self.middle_button['last'] = self.middle_button['current']


	# right button
	if self.right_button['last'] != self.right_button['current']:
		if self.right_button['current']:
			print('right button presssed')
			pass
	self.right_button['last'] = self.right_button['current']
	
	
	print(button['middle'], button['right'])
	#if self.debug:
		#rospy.loginfo('[Player] Button: {}'.format(self.button))
'''	

class States:
    INIT = -1
    READY = 0 # Waits for start button
    WALK_FORWARD = 1 # Moves the head, looking for the ball
    WALK_SLOWLY_FORWARD = 2
    WALK_PREBACKWARDS = 3
    WALK_BACKWARDS = 4
    END = 99

STEP_LEVEL = 0

# rospy.sleep(3)
currState = States.INIT
while not rospy.is_shutdown():
   button_callback()
