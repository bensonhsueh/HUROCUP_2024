#! /usr/bin/env python
import rospy
from op3_ros_utils import *
import cv2
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision import *
import time


# DEBUG_MODE = True # Show the detected image
# MIN_AREA = 1000
# SLOWDOWN_AREA = 40000
# CROSS_AREA = 45000

# # Functions to be passed to vision system
# func1 = detect2Color
# # HSV threshold
# args1 = ((np.array([0, 120, 45]), np.array([10, 255, 255])),
#        (np.array([170, 120, 45]), np.array([180, 255, 255])))
# # Create vision system
# vision = VisionSystem(pipeline_funcs=[func1],
#                       pipeline_args=[args1], debug=DEBUG_MODE, verbose=0)



rospy.init_node("move")
class Vc_listener:
    def __init__(self):
        
        self.robot_yaw = 0
        rospy.Subscriber("chatter", Float32, self._vusual_compass_callback)
        rospy.spin()
        
        
    def _vusual_compass_callback(self, msg):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        self.robot_yaw = msg.data


    # spin() simply keeps python from exiting until this node is stopped



robot = Robot()
rospy.sleep(3)

def init():
    # Set ctrl modules of all actions to joint, so we can reset robot position
    # robot.setGeneralControlModule("action_module")
    robot.setGeneralControlModule("action_module")

    # robot.setGrippersPos(left=0.0, right=0.0)

    # Call initial robot position
    robot.playMotion(1, wait_for_end=True)
    robot.setJointsControlModule(["head_tilt"], ["none"])
    robot.setGeneralControlModule("walking_module")




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
     
    if robot.buttonCheck("mode"):

        currState = States.INIT
        STEP_LEVEL = 0
        
    if currState == States.INIT:
        print("[INIT]")
        init()
        
        # Transition
        tick_count = 0
        direction = False
        currState = States.READY
    elif currState == States.READY:
        print("[READY]")
        
        if robot.buttonCheck("start"):
        
            tick_count = 0
            robot.walkStart()
            currState = States.WALK_FORWARD

        if robot.buttonCheck("user"):
        
            tick_count = 0
            robot.walkStart()
            currState = States.WALK_BACKWARDS 


    elif currState == States.WALK_FORWARD:

        yaw_gradient = Vc_listener.robot_yaw
        if yaw_gradient > 10:
            theta = 5
        elif yaw_gradient < -10:
            theta = -5
        else:
            theta = 0        
        
        print(yaw_gradient, theta )
        robot.walkVelocities(x=15.0, y=0, th=theta, z_move_amplitude=0.045, balance=True, z_offset=0)
        
        
        # if obj_area > CROSS_AREA:
        #     robot.walkStop()
        #     rospy.sleep(0.5)
        #     robot.walkStart()
        #     currState = States.WALK_PREBACKWARDS

        
        # time.sleep(10)
        # robot.walkStop()
        currState = States.INIT
        
    elif currState == States.WALK_BACKWARDS:

        robot.walkVelocities(x=-10.0, y=0, th=0.0, z_move_amplitude=0.045, balance=True, z_offset=0)
        rospy.sleep(3)
        robot.walkStart()
        rospy.sleep(5)
        robot.walkStop()
        currState = States.INIT
        
 

