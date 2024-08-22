# ! /usr/bin/python

import rospy

from op3_ros_utils import getWalkingParams, Robot
import time


class States:
    INIT = -1
    READY = 0 # Waits for start button
    WALK_FORWARD = 1 # Moves the head, looking for the ball
    WALK_SLOWLY_FORWARD = 2
    WALK_PREBACKWARDS = 3
    WALK_BACKWARDS = 4
    MARKER_TURN_LEFT = 5
    MARKER_TURN_RIGHT = 6
    MARKER_FORWARD = 7
    END = 99







# Iinitialize Node
rospy.init_node("fira_sprint")

# Create robot
robot = Robot()

rospy.sleep(3) # Make sure every publisher has registered to their topic,
               # avoiding lost messages

def init():
    robot.setGeneralControlModule("walking_module") #設成走路模式
    
    # Set joint modules of head joints to none so we can control them directly
    robot.setJointsControlModule(["head_pan", "head_tilt"], ["none", "none"])

    robot.setJointPos(["head_tilt"], [-1]) #低頭
    robot.setJointPos(["head_pan"], [0]) #頭轉正


    rospy.sleep(1.0)
    print("init OK")

    
# def scan_next_marker():
    

tickrate = 60
rate = rospy.Rate(tickrate)#設定間隔的頻率，也就是每秒需要發送幾次訊息(60HZ)，->rate.sleep()

# TODO remember to put to 0
currState = States.INIT

while not rospy.is_shutdown():#-----------------------------------------------------------------------------------------------------
    if currState == States.INIT:
        print("[INIT]")
        init()
        # Transition
        currState = States.WALK_FORWARD
        
    
    elif currState == States.WALK_FORWARD:
        # robot.walkVelocities(self, x=0.0, y=0.0, th=0, z_move_amplitude=None, balance=None,feet_pitch=None, z_offset=None, hip_pitch=None) #36
        while robot.buttonCheck("start"):
            robot.walkStart()
            robot.walkVelocities( x=40.0, y=0.0, th=0, z_move_amplitude=None, balance=True,feet_pitch=None, z_offset=None, hip_pitch=None)
            time.sleep(5)
            robot.walkStop()
            time.sleep(1)

            # robot.walkStart()
            # robot.walkVelocities( x=-20.0, y=0.0, th=0, z_move_amplitude=None, balance=True,feet_pitch=None, z_offset=None, hip_pitch=None)
            # time.sleep(2)
            # robot.walkStop()
            # time.sleep(1)


            # robot.walkStart()
            # robot.walkVelocities( x=0.0, y=50.0, th=0, z_move_amplitude=None, balance=True,feet_pitch=None, z_offset=None, hip_pitch=None)
            # time.sleep(2)
            # robot.walkStop()
            # time.sleep(1)

            # robot.walkStart()
            # robot.walkVelocities( x=0.0, y=-50.0, th=0, z_move_amplitude=None, balance=True,feet_pitch=None, z_offset=None, hip_pitch=None)
            # time.sleep(2)
            # robot.walkStop()
            # time.sleep(1)


            # robot.walkStart()
            # robot.walkVelocities( x=0.0, y=0.0, th=30, z_move_amplitude=None, balance=True,feet_pitch=None, z_offset=None, hip_pitch=None)#turn left
            # time.sleep(2)
            # robot.walkStop()
            # time.sleep(1)

            # robot.walkStart()
            # robot.walkVelocities( x=0.0, y=0.0, th=-30, z_move_amplitude=None, balance=True,feet_pitch=None, z_offset=None, hip_pitch=None)
            # time.sleep(2)
            # robot.walkStop()
            # time.sleep(1)



            robot.walkStop()

    
    rate.sleep()
