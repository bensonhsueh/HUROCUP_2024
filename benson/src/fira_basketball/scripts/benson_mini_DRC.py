#!/usr/bin/env python
import rospy
from op3_ros_utils import getWalkingParams, Robot
import cv2
import numpy as np
import time
import math
from scipy import optimize
from sensor_msgs.msg import Imu
from pynput import keyboard


z_gyro = 0.0
z_gyro_offset = 0.0
z_gyro_offset_for_caculate = 0.0


def init():
    robot.walkStop()
    robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])
    robot.setJointPos(["head_pan","head_tilt"], [0,np.radians(-60)])
    robot.setJointsControlModule(["l_sho_pitch","l_sho_roll","l_el"],["none","none","none"])
    robot.setJointPos(["l_sho_pitch","l_sho_roll","l_el"],[0.2,1.5,0])
    robot.setJointsControlModule(["r_sho_pitch","r_sho_roll","r_el"],["none","none","none"])
    robot.setJointPos(["r_sho_pitch","r_sho_roll","r_el"],[-0.2,-1.5,0])
    robot.setJointsControlModule(["l_gripper", "r_gripper"], ["none", "none"])
    # robot.setGeneralControlModule("walking_module")
    robot.setJointsControlModule(["r_hip_yaw","l_hip_yaw","r_hip_roll","l_hip_roll","r_hip_pitch","l_hip_pitch","r_knee","l_knee","r_ank_pitch","l_ank_pitch","r_ank_roll","l_ank_roll"],["walking_module"])
    
    robot.walkStop()

rospy.init_node("fira_Archery")


key_state = {
    'w': False, 'a': False, 's': False, 'd': False, 'q': False, 'e': False, '3': False, '4': False, '5': False,
    'c': False, 'x': False, 'z': False,
    'f': False, 'b': False, 'r': False,
    'y': False, 'h': False, 't': False, 'u': False, 'g': False, 'j': False, '1': False, '2': False,
    'o': False, 'l': False, 'i': False, 'p': False, 'k': False, ';': False, '0': False, '9': False,
    'up': False, 'down': False, 'left': False, 'right': False
}

def on_press(key):
    try:
        if key.char in key_state:
            key_state[key.char] = True
            # print(f'Key {key.char} pressed')
    except AttributeError:
        if key == keyboard.Key.up:
            key_state['up'] = True
            # print('Key up pressed')
        elif key == keyboard.Key.down:
            key_state['down'] = True
            # print('Key down pressed')
        elif key == keyboard.Key.left:
            key_state['left'] = True
            # print('Key left pressed')
        elif key == keyboard.Key.right:
            key_state['right'] = True
            # print('Key right pressed')

def on_release(key):
    try:
        if key.char in key_state:
            key_state[key.char] = False
            # print(f'Key {key.char} released')
    except AttributeError:
        if key == keyboard.Key.up:
            key_state['up'] = False
            # print('Key up released')
        elif key == keyboard.Key.down:
            key_state['down'] = False
            # print('Key down released')
        elif key == keyboard.Key.left:
            key_state['left'] = False
            # print('Key left released')
        elif key == keyboard.Key.right:
            key_state['right'] = False
            # print('Key right released')

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

robot = Robot()

class States:
    INIT = -1
    READY = 0
    KEYBOARD_CTRL = 1
    CORRECT_ANGLE = 2
    GO_UP_STAIRS = 3
    END = 99

# cap = cv2.VideoCapture(0)


currState = States.INIT

DEBUG_MODE = True
# DEBUG_MODE = False # Show the detected image True

while not rospy.is_shutdown():
    

    if robot.buttonCheck("mode"):
        currState = States.INIT
    if DEBUG_MODE:
    #     if key_state['w']:
    #         print('Key w is being held down')
    #     if key_state['a']:
    #         print('Key a is being held down')
    #     if key_state['s']:
    #         print('Key s is being held down')
    #     if key_state['d']:
    #         print('Key d is being held down')
    #     if key_state['up']:
    #         print('Key up is being held down')
    #     if key_state['down']:
    #         print('Key down is being held down')
    #     if key_state['left']:
    #         print('Key left is being held down')
    #     if key_state['right']:
    #         print('Key right is being held down')
        pass
    if currState == States.INIT:
        print("[INIT]")
        
        
        init()
        # robot.walkStart()
        rospy.sleep(3)
        
        # Transition
        tick_count = 0
        direction = False
        # currState = States.READY
        currState = States.KEYBOARD_CTRL
        # currState = States.DOOR_CROSS

        z_gyro_offset_for_caculate = z_gyro_offset
        z_gyro = 0.0
        print("[INIT_FINISH]")
    elif currState == States.READY:
        # print("[READY]")
        
        if robot.buttonCheck("start"):
        
            tick_count = 0
            robot.walkStart()
            z_gyro = 0.0
            currState = States.KEYBOARD_CTRL
            # currState = States.DOOR_CROSS
            # currState = States.DOOR_LOCATE
            # currState = States.DOOR_CROSS
            print("start")

    elif currState == States.KEYBOARD_CTRL:
        _x,_y,_z = -5,0,0

        if key_state['w']:
            _x = 10
        if key_state['s']:
             _x = -20
        if key_state['a']:
            _y = 20
        if key_state['d']:
            _y = -20
        if key_state['q']:
            _z = 8
        if key_state['e']:
            _z = -8
        if key_state['3']:
            _y = 40
            _x = -2
        if key_state['4']:
            _y = -40
            _x = -2
        if key_state['5']:
            _x = 1
        if key_state['w'] or key_state['a'] or key_state['s'] or key_state['d'] or key_state['q'] or key_state['e'] or key_state['3'] or key_state['4'] or key_state['5']:
            robot.walkStart()
            robot.walkVelocities(x=_x, y=_y, th=_z, z_move_amplitude=0.045, balance=True, z_offset=0)# y+:left,-:right the+:left, -:right
        else:
            robot.walkStop()
        
        if key_state['left']:
            pan_angle_old = np.degrees(robot.joint_pos["head_pan"])
            pan_angle_new = pan_angle_old + 5
            robot.setJointPos(["head_pan"], [np.radians(pan_angle_new)])
            print("pan:",np.degrees(robot.joint_pos["head_pan"]),"tilt:",np.degrees(robot.joint_pos["head_tilt"]))
        if key_state['right']:
            pan_angle_old = np.degrees(robot.joint_pos["head_pan"])
            pan_angle_new = pan_angle_old - 5
            robot.setJointPos(["head_pan"], [np.radians(pan_angle_new)])
            print("pan:",np.degrees(robot.joint_pos["head_pan"]),"tilt:",np.degrees(robot.joint_pos["head_tilt"]))
        if key_state['up']:
            tilt_angle_old = np.degrees(robot.joint_pos["head_tilt"])
            tilt_angle_new = tilt_angle_old + 5
            robot.setJointPos(["head_tilt"], [np.radians(tilt_angle_new)])
            print("pan:",np.degrees(robot.joint_pos["head_pan"]),"tilt:",np.degrees(robot.joint_pos["head_tilt"]))
        if key_state['down']:
            tilt_angle_old = np.degrees(robot.joint_pos["head_tilt"])
            tilt_angle_new = tilt_angle_old - 5
            robot.setJointPos(["head_tilt"], [np.radians(tilt_angle_new)])
            print("pan:",np.degrees(robot.joint_pos["head_pan"]),"tilt:",np.degrees(robot.joint_pos["head_tilt"]))

        if key_state['c']:
            print("[GO_UP_STAIRS]")
            robot.setGeneralControlModule("action_module")
            robot.playMotion(2, wait_for_end=True)
            robot.playMotion(100, wait_for_end=True)
            # robot.playMotion(101, wait_for_end=True)
            robot.playMotion(2, wait_for_end=True)
            rospy.sleep(1.0)
            init()
            rospy.sleep(3.0)
        if key_state['x']:
            print("[GO_DOWN_STAIRS]")
            robot.setGeneralControlModule("action_module")
            robot.playMotion(2, wait_for_end=True)
            robot.playMotion(102, wait_for_end=True)
            # robot.playMotion(101, wait_for_end=True)
            robot.playMotion(2, wait_for_end=True)
            rospy.sleep(1.0)
            init()
            rospy.sleep(3.0)
        if key_state['z']:
            print("[CROSS_DOOR]")
            robot.setGeneralControlModule("action_module")
            robot.playMotion(30, wait_for_end=True)
            for i in range(2):
                robot.playMotion(31, wait_for_end=True)
            robot.playMotion(32, wait_for_end=True)
            rospy.sleep(1.0)
            init()
            rospy.sleep(3.0)

        if key_state['r']:
            print("[INIT]")
            init()
            rospy.sleep(3.0)
        if key_state['f']:
            print("[UP_FRONT]")
            robot.setGeneralControlModule("action_module")
            robot.playMotion(85, wait_for_end=True)
            rospy.sleep(1.0)
            init()
            rospy.sleep(3.0)
        if key_state['b']:
            print("[UP_BACK]")
            robot.setGeneralControlModule("action_module")
            robot.playMotion(86, wait_for_end=True)
            rospy.sleep(1.0)
            init()
            rospy.sleep(3.0)

        if key_state['y']:
            l_sho_pitch_angle_old = np.degrees(robot.joint_pos["l_sho_pitch"])
            l_sho_pitch_angle_new = l_sho_pitch_angle_old - 2
            robot.setJointPos(["l_sho_pitch"], [np.radians(l_sho_pitch_angle_new)])
        if key_state['h']:
            l_sho_pitch_angle_old = np.degrees(robot.joint_pos["l_sho_pitch"])
            l_sho_pitch_angle_new = l_sho_pitch_angle_old + 2
            robot.setJointPos(["l_sho_pitch"], [np.radians(l_sho_pitch_angle_new)])
        if key_state['t']:
            l_el_angle_old = np.degrees(robot.joint_pos["l_el"])
            l_el_angle_new = l_el_angle_old + 2
            robot.setJointPos(["l_el"], [np.radians(l_el_angle_new)])
        if key_state['u']:
            l_el_angle_old = np.degrees(robot.joint_pos["l_el"])
            l_el_angle_new = l_el_angle_old - 2
            robot.setJointPos(["l_el"], [np.radians(l_el_angle_new)])
        if key_state['g']:
            l_sho_roll_angle_old = np.degrees(robot.joint_pos["l_sho_roll"])
            l_sho_roll_angle_new = l_sho_roll_angle_old - 2
            robot.setJointPos(["l_sho_roll"], [np.radians(l_sho_roll_angle_new)])
        if key_state['j']:
            l_sho_roll_angle_old = np.degrees(robot.joint_pos["l_sho_roll"])
            l_sho_roll_angle_new = l_sho_roll_angle_old + 2
            robot.setJointPos(["l_sho_roll"], [np.radians(l_sho_roll_angle_new)])

        if key_state['o']:
            r_sho_pitch_angle_old = np.degrees(robot.joint_pos["r_sho_pitch"])
            r_sho_pitch_angle_new = r_sho_pitch_angle_old + 2
            robot.setJointPos(["r_sho_pitch"], [np.radians(r_sho_pitch_angle_new)])
        if key_state['l']:
            r_sho_pitch_angle_old = np.degrees(robot.joint_pos["r_sho_pitch"])
            r_sho_pitch_angle_new = r_sho_pitch_angle_old - 2
            robot.setJointPos(["r_sho_pitch"], [np.radians(r_sho_pitch_angle_new)])
        if key_state['i']:
            r_el_angle_old = np.degrees(robot.joint_pos["r_el"])
            r_el_angle_new = r_el_angle_old + 2
            robot.setJointPos(["r_el"], [np.radians(r_el_angle_new)])
        if key_state['p']:
            r_el_angle_old = np.degrees(robot.joint_pos["r_el"])
            r_el_angle_new = r_el_angle_old - 2
            robot.setJointPos(["r_el"], [np.radians(r_el_angle_new)])
        if key_state['k']:
            r_sho_roll_angle_old = np.degrees(robot.joint_pos["r_sho_roll"])
            r_sho_roll_angle_new = r_sho_roll_angle_old - 2
            robot.setJointPos(["r_sho_roll"], [np.radians(r_sho_roll_angle_new)])
        if key_state[';']:
            r_sho_roll_angle_old = np.degrees(robot.joint_pos["r_sho_roll"])
            r_sho_roll_angle_new = r_sho_roll_angle_old + 2
            robot.setJointPos(["r_sho_roll"], [np.radians(r_sho_roll_angle_new)])

        if key_state['1']:
            l_gripper_angle_old = np.degrees(robot.joint_pos["l_gripper"])
            l_gripper_angle_new = l_gripper_angle_old - 3
            robot.setJointPos(["l_gripper"], [np.radians(l_gripper_angle_new)])
        if key_state['2']:
            l_gripper_angle_old = np.degrees(robot.joint_pos["l_gripper"])
            l_gripper_angle_new = l_gripper_angle_old + 3
            robot.setJointPos(["l_gripper"], [np.radians(l_gripper_angle_new)])
        if key_state['0']:
            r_gripper_angle_old = np.degrees(robot.joint_pos["r_gripper"])
            r_gripper_angle_new = r_gripper_angle_old + 3
            robot.setJointPos(["r_gripper"], [np.radians(r_gripper_angle_new)])
        if key_state['9']:
            r_gripper_angle_old = np.degrees(robot.joint_pos["r_gripper"])
            r_gripper_angle_new = r_gripper_angle_old - 3
            robot.setJointPos(["r_gripper"], [np.radians(r_gripper_angle_new)])




    # ret, img = cap.read()

    # if not ret:
    #     break
    
    # cv2.imshow("img", img)
    # cv2.waitKey(1)