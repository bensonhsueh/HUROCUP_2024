#! /usr/bin/python

from __future__ import division
import rospy
import numpy as np
import cv2
from robot_msgs.msg import HeadMove, Button, ArucoData 
from op3_ros_utils import getWalkingParams, Robot
from vision_old import *
from copy import copy
import sys

DEBUG_MODE = True # Show the detected image
MAX_ARUCO = 500

MIN_ARUCO = 1000
SHOWARUCO = 100
CROSS_ARUCO=70

sum_err_pan = 0
sum_err_tilt = 0
last_error_x = 0
last_error_y = 0
def head_track_marker():
	global pos_pan, pos_tilt, sum_err_pan, sum_err_tilt, last_error_x, last_error_y
	global freq
	dt = 1.0 / float(freq)
	KP_pan = rospy.get_param("/sprint_params/player/Pan_KP")
	KI_pan = rospy.get_param("/sprint_params/player/Pan_KI")
	KD_pan = rospy.get_param("/sprint_params/player/Pan_KD")
	KP_tilt = rospy.get_param("/sprint_params/player/Tilt_KP")
	KI_tilt = rospy.get_param("/sprint_params/player/Tilt_KI")
	KD_tilt = rospy.get_param("/sprint_params/player/Tilt_KD")
	if marker_x != -1 and marker_y != -1:	
		error_x = (frame_w/2) - marker_x
		error_x *= 77.32 / frame_w
		error_x = (error_x * math.pi)/ 180
		error_x_diff = error_x - last_error_x

		P_pan  = last_error_x * KP_pan
		sum_err_pan += error_x * dt
		I_pan = sum_err_pan * KI_pan
		deriv_err_pan = error_x_diff / dt
		D_pan = deriv_err_pan * KD_pan
		last_error_x = error_x
		pos_pan += (P_pan + I_pan + D_pan)*-1

		error_y = (frame_h/2) - marker_y
		error_y *= -1
		error_y *= 61.93 / frame_h
		error_y = (error_y * math.pi) /180
		error_y_diff = error_y - last_error_y

		P_tilt  = last_error_y * KP_tilt
		sum_err_tilt += error_y * dt
		I_tilt = sum_err_tilt * KI_tilt
		deriv_err_tilt = sum_err_tilt / dt
		D_tilt = deriv_err_tilt * KD_tilt
		last_error_y = error_y
		pos_tilt += (P_tilt + I_tilt + D_tilt)*-1

		head_pos = head_limit(pos_pan, round(pos_tilt, 2))
		#print("Marker Size: %d", marker_size)
		pos_pan, pos_tilt = head_pos.data		
		head_pub.publish(head_pos)


class States:
    INIT = -1
    READY = 0 # Waits for start button
    FUCK = 69
    WALK_FORWARD = 1 # Moves the head, looking for the ball
    WALK_SLOWLY_FORWARD = 2
    WALK_PREBACKWARDS = 3
    WALK_BACKWARDS = 4
    WALK_BACKWARDS_2 = 5
    END = 99


rospy.sleep(3) # Make sure every publisher has registered to their topic,
               # avoiding lost messages
class aruco_sprint:

    def __init__(self):
        rospy.init_node("fira_sprint")

        # Subscribe to cv_camera topic with vision system
        #rospy.Subscriber("/cv_camera/image_raw", Image, vision.read, queue_size=1)
        rospy.Subscriber("/sprint/marker/position", ArucoData,	self.marker_pos_callback)
        

        self.robot = Robot()
        self.aruco_x    = -1
        self.aruco_y    = -1
        self.distance   = -1
        self.debug      = True
        self.freq = 60

    def init(self):


        # rospy.Subscriber("/cv_camera/image_raw", Image, iimg, queue_size=1)
        # Iinitialize Node

        # Create robot

        rospy.sleep(1)
        # Set ctrl modules of all actions to joint, so we can reset robot position 
        self.robot.setGeneralControlModule("action_module")

        # robot.setGrippersPos(left=0.0, right=0.0)

        # Call initial robot position
        self.robot.playMotion(1, wait_for_end=True)

        # Set ctrl module to walking, this actually only sets the legs
        self.robot.setGeneralControlModule("walking_module")
        
        # Set joint modules of head joints to none so we can control them directly
        self.robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])

        self.robot.setJointPos(["head_tilt"], [-0.2])
        
        rospy.sleep(1.0)

    def center_head_on_object(self,msg):
        cx, cy = 0.5, 0.5 # Center of image
        obj_x, obj_y = float( self.aruco_x /640) , float(self.aruco_y/480)
        print(obj_x, obj_y)
        dt = 1.0 / 60.0

        dist_x = obj_x - cx
        dist_y = obj_y - cy

        head_curr_x = self.robot.joint_pos["head_pan"]
        head_curr_y = self.robot.joint_pos["head_tilt"]
        
        kp = 1.0
        new_head_x = head_curr_x + kp * -dist_x
        new_head_y = head_curr_y + kp * -dist_y
        
        kd = 0.003
        deriv_error_x = dist_x / dt
        deriv_error_y = dist_y / dt
        
        new_head_x = new_head_x + kd * deriv_error_x
        new_head_y = new_head_y + kd * deriv_error_y
        self.robot.setJointPos(["head_tilt", "head_pan"], [new_head_y, new_head_x])
    def marker_pos_callback(self, msg):
        #  Aruco center marker
        self.aruco_x     = msg.x
        self.aruco_y     = msg.y
        
        # Distance
        self.distance = msg.size
        
        if self.debug:
            rospy.loginfo('[Player] Distance: {}'.format(self.distance))
    def run(self):
        tickrate = 20
        rate = rospy.Rate(tickrate)

        # TODO remember to put to 0
        STEP_LEVEL = 0

        currState = States.INIT

        count = 0
        while not rospy.is_shutdown():
            
            if self.robot.buttonCheck("mode"):
                STEP_LEVEL = 0
                currState = States.INIT
            
            # if DEBUG_MODE:
                                    
            #         cv2.imshow("Frame", vision.debug_img[0])
            #         cv2.waitKey(1)

            if self.distance !=-1:
            
                if self.distance < MAX_ARUCO:
                    self.center_head_on_object(self.distance)

            if currState == States.INIT:
                print("[INIT]")
                self.init()

                # Transition
                tick_count = 0

                direction = False
                currState = States.READY
            
            elif currState == States.READY:
                print("[READY]")
                if self.robot.buttonCheck("start"):
                    self.robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])
                    self.robot.setJointPos(["head_pan", "head_tilt"], [0, -0.2])
                    tick_count = 0
                    self.robot.walkStart()
                    currState = States.FUCK
            elif currState == States.FUCK:
                self.robot.walkVelocities(x=20, th=0, balance=True, hip_pitch=6, z_move_amplitude = 0.055)
                rospy.sleep(6)
                currState = States.WALK_FORWARD
            
            elif currState == States.WALK_FORWARD:
                print("[WALK_FORWARD]")
                pan_angle = self.robot.joint_pos["head_pan"]
                print(pan_angle)
                if pan_angle > 0.1:
                    theta = 5
                elif pan_angle < -0.1:
                    theta = -5
                else:
                    theta = 0

                self.robot.walkVelocities(x=26, th=theta, balance=True, hip_pitch=6, z_move_amplitude = 0.055)

                tick_count += 1

                if self.distance < SHOWARUCO and self.distance != -1:
                    currState = States.WALK_SLOWLY_FORWARD

            elif currState == States.WALK_SLOWLY_FORWARD:
                print("[WALK_SLOWLY_FORWARD]")
                pan_angle = self.robot.joint_pos["head_pan"]
                if pan_angle > 0.025:
                    theta = -0.7
                    yamp = 0.3
                elif pan_angle < -0.03:
                    theta = 0.5
                    yamp = -0.3
                else:
                    theta = 0
                    yamp = 0
                self.robot.walkVelocities(x=18, y=yamp,th=theta, balance=True, hip_pitch=5)
                
                if self.distance < CROSS_ARUCO and self.distance != -1:
                    self.robot.walkStop()
                    rospy.sleep(1.5)
                    self.robot.walkStart()
                    currState = States.WALK_PREBACKWARDS

            elif currState == States.WALK_PREBACKWARDS:
                print("[WALK_PREBACKWARDS]")
                pan_angle = self.robot.joint_pos["head_pan"]
                print('pangle', pan_angle)
                
                if pan_angle > 0.025:
                    theta = -1#-0.7
                    yamp = pan_angle * 5
                elif pan_angle < -0.025:
                    theta = 1.0#0.5
                    yamp = pan_angle * 5
                else:
                    theta = 0
                    yamp = 0
                # yamp = 30
                #yamp = pan_angle * 20
                self.robot.walkVelocities(x=-15, y = yamp, th = theta - 1.0, balance=True, hip_pitch=4)
                #self.robot.setJointPos(["head_pan", "head_tilt"], [0, -0.2])
                #rospy.sleep(1)

                currState = States.WALK_BACKWARDS
            
            elif currState == States.WALK_BACKWARDS:
                print("[WALK_BACKWARDS]")
                #pan_angle = self.robot.joint_pos["head_pan"]
                #print(pan_angle)
                #if pan_angle > 0.025:
                #    theta = -1
                #    yamp = pan_angle * 10
                #elif pan_angle < -0.025:
                #    theta = 1.0
                #    yamp = pan_angle * 10
                #    if count % 20 == 0:
                #        print('right')
                #else:
                #    theta = -0
                #    yamp = -0

                #if count % 20 == 0:
                #    print('pan_angle', pan_angle)
                #    print('yamp', yamp)
                #yamp = 30######forced adjust
                
                self.robot.walkVelocities(x=-20, y = yamp, th = theta - 1.1 ,balance=True, hip_pitch=4)
                #rospy.sleep(7)
                currState = States.WALK_BACKWARDS_2
                #self.walk.Stop()
                #rospy.sleep(1.5)
                #self.walk.start()

            elif currState == States.WALK_BACKWARDS_2:
                print("[WALK_BACKWARDS_2]")
                pan_angle = self.robot.joint_pos["head_pan"]
                #print(pan_angle)
                #if pan_angle > 0.02:
                #    theta = -1
                #    yamp = pan_angle * 15
                #elif pan_angle < -0.02:
                #    theta = 1.0
                #    yamp = pan_angle * 15
                #    if count % 20 == 0:
                #        print('right')
                #else:
                #    theta = 0
                #    yamp = 0

                #if count % 20 == 0:
                #    print('pan_angle', pan_angle)
                #    print('yamp', yamp)
                theta = 0
                yamp = 0
                self.robot.walkVelocities(x=-20, y = yamp, th=theta - 1.1, balance=True, hip_pitch=4)

            elif currState == States.END:
                print("[END]")
                self.robot.walkStop()


            count = count + 1
            
            rate.sleep()


if __name__ == "__main__":
    sprint = aruco_sprint()
    sprint.run()
